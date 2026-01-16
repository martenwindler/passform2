import logging
import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware

# System-Importe
from app.config import config, SystemMode
from .managers.node_manager import router as node_manager_router
from .managers.path_manager import router as path_manager_router
from .system_api import router as system_router
from .socket.socket_io_manager import socket_manager
from .managers.agent_manager import agent_manager
from .ros.ros_client import get_ros_client
from .logic.planner import planner

# Logger Konfiguration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("main")

# --- LIFESPAN MANAGEMENT ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Verwaltet Startup und Shutdown des Backends."""
    logger.info(f"🚀 PassForm Backend startet im Modus: {config.get_current_mode().value}")
    
    # ROS-Client initialisieren
    get_ros_client()
    
    # Callback für neue Socket-Verbindungen (Daten-Synchronisation)
    async def sync_state_for_client(sid):
        try:
            welcome_msg = {
                "message": f"Verbindung stabilisiert: {socket_manager.active_clients_count()} Client(s) online", 
                "level": "success"
            }
            await socket_manager.emit_event('system_log', welcome_msg, target_sid=sid)
            
            # Bestehende Logs und Agenten senden
            for log_entry in agent_manager.get_logs_history():
                await socket_manager.emit_event('system_log', log_entry, target_sid=sid)
            
            agents_data = [a.to_dict() for a in agent_manager.agents.values()]
            await socket_manager.emit_event('active_agents', {'agents': agents_data}, target_sid=sid)
            
            logger.info(f"✅ Sync für Client {sid} abgeschlossen.")
        except Exception as e:
            logger.error(f"Fehler beim Client-Sync: {e}")

    socket_manager.set_on_new_client_callback(sync_state_for_client)
    yield
    
    logger.info("🛑 PassForm Backend wird beendet...")
    from .managers.node_manager import node_manager
    node_manager.kill_all_nodes()

# --- APP SETUP ---
fastapi_app = FastAPI(
    title="PassForm Agent Backend",
    description="Zentrale Steuerung für FTF und statische Module",
    version="1.2.0",
    lifespan=lifespan
)

fastapi_app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- SOCKET.IO MISSIONS-STEUERUNG ---

@socket_manager.sio.on('plan_path')
async def handle_plan_path(sid, payload):
    """
    Empfängt Planungsanfragen. 
    Entscheidet zwischen FTF-Mission (Dynamisch) und Modul-Kette (Statisch).
    """
    try:
        start = payload.get("start")
        goal = payload.get("goal")
        elm_agents = payload.get("agents", {}) 
        
        # 1. Gitter normalisieren & FTF identifizieren
        grid = {}
        agents_list = elm_agents.values() if isinstance(elm_agents, dict) else elm_agents
        ftf_agent = None

        for agent in agents_list:
            pos = agent.get("position", agent) 
            x, y = int(pos.get("x")), int(pos.get("y"))
            a_data = agent.copy()
            a_data["x"], a_data["y"] = x, y
            grid[(x, y)] = a_data
            
            if agent.get("module_type") == "ftf":
                ftf_agent = a_data

        start_tuple = (int(start["x"]), int(start["y"]))
        goal_tuple = (int(goal["x"]), int(goal["y"]))

        # 2. STRATEGIE-WAHL
        if ftf_agent:
            # --- DYNAMISCHE FTF MISSION ---
            ftf_id = ftf_agent["agent_id"]
            ftf_pos = (int(ftf_agent["x"]), int(ftf_agent["y"]))
            agent_manager.log_to_system(f"FTF Mission: Starte Berechnung für {ftf_id}...", "info")

            # Phase A: Anfahrt zum Startpunkt (Leerfahrt)
            path_a, _, _ = planner.a_star(ftf_pos, start_tuple, grid, travel_mode="ftf")
            
            # Phase B: Transport zum Zielpunkt (Lastfahrt)
            path_b, cost_b, _ = planner.a_star(start_tuple, goal_tuple, grid, travel_mode="ftf")
            
            if path_a and path_b:
                full_path = path_a + path_b[1:] # Pfade verknüpfen
                
                # Pfad an Frontend zur Visualisierung senden
                await socket_manager.emit_event('path_complete', {
                    "status": 1, "cost": float(cost_b), "path": full_path
                }, target_sid=sid)

                # MISSION STARTEN (Hintergrund-Simulation)
                agent_manager.log_to_system(f"Mission bestätigt. FTF setzt sich in Bewegung.", "success")
                asyncio.create_task(
                    agent_manager.execute_mission(ftf_id, full_path, start_tuple, goal_tuple)
                )
            else:
                agent_manager.log_to_system("Abbruch: Kein Pfad für FTF möglich.", "warning")
                await socket_manager.emit_event('path_complete', {"status": 0, "path": []}, target_sid=sid)

        else:
            # --- STATISCHE MODUL-KETTE ---
            agent_manager.log_to_system("Kein FTF vorhanden. Suche statische Verbindung...", "info")
            path, cost, msg = planner.a_star(start_tuple, goal_tuple, grid, travel_mode="chain")
            
            if path:
                agent_manager.log_to_system(f"Kette gefunden: {msg}", "success")
                await socket_manager.emit_event('path_complete', {"status": 1, "cost": float(cost), "path": path}, target_sid=sid)
            else:
                agent_manager.log_to_system(f"Keine Kette möglich: {msg}", "warning")
                await socket_manager.emit_event('path_complete', {"status": 0, "path": []}, target_sid=sid)

    except Exception as e:
        logger.error(f"Kritischer Fehler in handle_plan_path: {e}", exc_info=True)
        agent_manager.log_to_system("Interner Fehler bei der Missionsplanung.", "error")

@socket_manager.sio.on('set_mode')
async def handle_set_mode(sid, data):
    """Wechselt zwischen Simulation und Hardware-Modus."""
    try:
        mode_str = data if isinstance(data, str) else data.get("mode")
        new_mode = SystemMode(mode_str)
        if config.set_mode(new_mode):
            if new_mode == SystemMode.HARDWARE:
                agent_manager.clear_all_agents()
                agent_manager.log_to_system("Hardware-Modus aktiv: Warte auf ROS-Agenten...", "warning")
            else:
                agent_manager.log_to_system("Simulations-Modus aktiv: Manuelle Steuerung.", "info")
            await socket_manager.emit_event('mode', mode_str)
    except Exception as e:
        logger.error(f"Modus-Fehler: {e}")

# --- API ENDPUNKTE ---
fastapi_app.include_router(node_manager_router, prefix="/api/nodes")
fastapi_app.include_router(path_manager_router, prefix="/api/path")
fastapi_app.include_router(system_router, prefix="/api/system")

@fastapi_app.get("/")
async def root():
    return {"service": "PassForm Backend", "mode": config.get_current_mode().value, "status": "online"}

@fastapi_app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled Exception: {exc}", exc_info=True)
    return JSONResponse(status_code=500, content={"message": str(exc)})

# Socket.IO an FastAPI binden
socket_manager.attach_fastapi(fastapi_app)
app = socket_manager.app_sio

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host=config.host, port=config.port, reload=False)