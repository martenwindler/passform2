import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware

# Importe deiner Module
from .config import config, SystemMode
from .managers.node_manager import router as node_manager_router
from .managers.path_manager import router as path_manager_router
from .system_api import router as system_router
from .socket.socket_io_manager import socket_manager
from .managers.agent_manager import agent_manager
from .ros.ros_client import get_ros_client
from .logic.planner import planner

# Logger Setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("main")

# --- LIFESPAN MANAGEMENT ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    # --- STARTUP ---
    logger.info(f"🚀 PassForm Backend startet im Modus: {config.get_current_mode().value}")
    
    # 1. ROS Client initialisieren
    get_ros_client()
    
    # 2. Synchronisations-Logik für neue Clients (z.B. nach Elm Hot-Reload)
    async def sync_state_for_client(sid):
        """Wird aufgerufen, sobald sich ein Client per Socket.IO verbindet."""
        try:
            # A. Willkommens-Nachricht (Fundament der Log-Liste)
            welcome_msg = {
                "message": f"Verbindung stabilisiert: {socket_manager.active_clients_count()} Client(s) online", 
                "level": "success"
            }
            await socket_manager.emit_event('system_log', welcome_msg, target_sid=sid)
            
            # B. Log-Historie aus dem Speicher senden
            history = agent_manager.get_logs_history()
            for log_entry in history:
                await socket_manager.emit_event('system_log', log_entry, target_sid=sid)
            
            # C. Aktuelle Agenten-Liste senden (Nur im Hardware-Modus relevant)
            agents_data = [a.to_dict() for a in agent_manager.agents.values()]
            await socket_manager.emit_event('active_agents', {'agents': agents_data}, target_sid=sid)
            
            logger.info(f"✅ Sync für Client {sid} abgeschlossen.")
        except Exception as e:
            logger.error(f"Fehler beim Client-Sync (sid: {sid}): {e}")

    # Registriere den Sync-Handler im SocketManager
    socket_manager.set_on_new_client_callback(sync_state_for_client)
    
    yield
    
    # --- SHUTDOWN ---
    logger.info("🛑 PassForm Backend wird beendet...")
    from .managers.node_manager import node_manager
    node_manager.kill_all_nodes()

# --- APP KONFIGURATION ---
fastapi_app = FastAPI(
    title="PassForm Agent Backend",
    description="REST API und WebSocket-Server für ROS-Kommunikation",
    version="1.0.0",
    lifespan=lifespan
)

# Middleware (Erlaubt Elm-Frontend den Zugriff)
fastapi_app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- SOCKET.IO EVENT HANDLER ---

@socket_manager.sio.on('plan_path')
async def handle_plan_path(sid, payload):
    """
    Berechnet Pfad. Nutzt im Simulationsmodus die Daten aus dem Elm-Payload,
    im Hardware-Modus die Daten aus dem ROS-Weltmodell.
    """
    try:
        start = payload.get("start")
        goal = payload.get("goal")
        elm_agents = payload.get("agents") # Liste von Agenten-Objekten aus Elm
        
        if not start or not goal:
            return

        # 1. Welches Weltmodell nutzen wir? (Teile & Herrsche)
        if config.get_current_mode() == SystemMode.SIMULATION and elm_agents:
            # Simulation: Wir nehmen das Gitter, das Elm uns schickt
            grid = {(a["x"], a["y"]): a for a in elm_agents}
            logger.info("Pfadplanung: Nutze Simulations-Layout von Elm.")
        else:
            # Hardware: Wir nehmen das Gitter aus dem AgentManager (ROS)
            grid = agent_manager.get_grid_model()
            logger.info("Pfadplanung: Nutze Hardware-Layout vom Backend.")

        start_tuple = (int(start["x"]), int(start["y"]))
        goal_tuple = (int(goal["x"]), int(goal["y"]))

        agent_manager.log_to_system(f"Starte A* Planung: {start_tuple} ➔ {goal_tuple}", "info")
        
        # 2. A* Berechnung
        path, cost, message = planner.a_star(start_tuple, goal_tuple, grid)
        
        if path:
            agent_manager.log_to_system(message, "success")
            await socket_manager.emit_event('path_complete', {
                "status": 1,
                "cost": float(cost),
                "path": path
            }, target_sid=sid)
        else:
            agent_manager.log_to_system(f"Planung abgebrochen: {message}", "warning")
            await socket_manager.emit_event('path_complete', {"status": 0, "cost": 0.0, "path": []}, target_sid=sid)

    except Exception as e:
        logger.error(f"Fehler in handle_plan_path: {e}", exc_info=True)
        agent_manager.log_to_system("Schwerer Fehler im Backend-Planer", "error")

@socket_manager.sio.on('set_mode')
async def handle_set_mode(sid, data):
    """Verarbeitet den Modus-Wechsel und stellt Typ-Sicherheit sicher."""
    try:
        mode_str = data if isinstance(data, str) else data.get("mode")
        
        # Umwandlung String -> Enum (Pydantic-kompatibel)
        new_mode = SystemMode(mode_str)
        
        # Modus in der globalen Config setzen
        changed = config.set_mode(new_mode)

        if changed:
            if new_mode == SystemMode.HARDWARE:
                # Hardware-Modus: Altes Simulations-Gitter löschen
                agent_manager.clear_all_agents()
                agent_manager.log_to_system("Hardware-Modus aktiv: Suche nach physischen Agenten...", "warning")
            else:
                agent_manager.log_to_system("Simulations-Modus aktiv: Manuelle Steuerung.", "info")

            # Bestätigung an alle Clients senden
            await socket_manager.emit_event('mode', mode_str)
            
    except Exception as e:
        logger.error(f"Fehler beim Modus-Wechsel: {e}")
        agent_manager.log_to_system(f"Modus-Wechsel fehlgeschlagen: {str(e)}", "error")

# --- ROUTER & ENDPOINTS ---
fastapi_app.include_router(node_manager_router, prefix="/api/nodes")
fastapi_app.include_router(path_manager_router, prefix="/api/path")
fastapi_app.include_router(system_router, prefix="/api/system")

@fastapi_app.get("/")
async def root():
    return {
        "service": "PassForm Agent Backend",
        "version": "1.0.0",
        "status": "online",
        "mode": config.get_current_mode().value
    }

# Zentrales Error-Handling für REST-Anfragen
@fastapi_app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled Exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"message": str(exc), "type": exc.__class__.__name__},
    )

# --- SOCKET.IO INTEGRATION ---
socket_manager.attach_fastapi(fastapi_app)
app = socket_manager.app_sio

if __name__ == "__main__":
    import uvicorn
    logger.info(f"🟢 Starte Uvicorn Server auf {config.host}:{config.port}")
    uvicorn.run("app.main:app", host=config.host, port=config.port, reload=False)