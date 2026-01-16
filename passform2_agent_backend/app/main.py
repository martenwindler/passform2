import logging
import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware

# System-Importe
from app.config import config, SystemMode
from .managers.node_manager import router as node_manager_router
# [Image of a typical FastAPI project structure with routers and managers]
from .managers.path_manager import router as path_manager_router
from .system_api import router as system_router
from .socket.socket_io_manager import socket_manager
from .managers.agent_manager import agent_manager
from .managers.nfc_manager import nfc_manager 
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
    
    # 1. ROS-Client initialisieren
    get_ros_client()

    # 2. NFC-Reader starten (Hintergrund-Thread für kontinuierliches Lesen)
    nfc_manager.start_reading()
    
    # 3. Callback für neue Socket-Verbindungen (Daten-Synchronisation)
    async def sync_state_for_client(sid):
        try:
            # Begrüßung
            welcome_msg = {
                "message": f"Verbindung stabil: {socket_manager.active_clients_count()} Client(s) online", 
                "level": "success"
            }
            await socket_manager.emit_event('system_log', welcome_msg, target_sid=sid)
            
            # NEU: Hardware-Status des NFC-Readers synchronisieren
            await socket_manager.emit_event('nfc_status', {
                "status": nfc_manager.get_status()  # "online" oder "missing"
            }, target_sid=sid)
            
            # Bestehende Logs und Agenten senden
            for log_entry in agent_manager.get_logs_history():
                await socket_manager.emit_event('system_log', log_entry, target_sid=sid)
            
            agents_data = [a.to_dict() for a in agent_manager.agents.values()]
            await socket_manager.emit_event('active_agents', {'agents': agents_data}, target_sid=sid)
            
            logger.info(f"✅ Sync für Client {sid} abgeschlossen (NFC-Status: {nfc_manager.get_status()}).")
        except Exception as e:
            logger.error(f"Fehler beim Client-Sync: {e}")

    socket_manager.set_on_new_client_callback(sync_state_for_client)
    
    yield
    
    # --- SHUTDOWN ---
    logger.info("🛑 PassForm Backend wird beendet...")
    nfc_manager.running = False  # Stoppt den NFC-Lese-Thread
    from .managers.node_manager import node_manager
    node_manager.kill_all_nodes()

# --- APP SETUP ---
fastapi_app = FastAPI(
    title="PassForm Agent Backend",
    description="Zentrale Steuerung für FTF, statische Module und RFID-Hardware",
    version="1.3.2",
    lifespan=lifespan
)

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
    """Missionsplanung: Entscheidet zwischen FTF-Transport und Modul-Kette."""
    try:
        start, goal = payload.get("start"), payload.get("goal")
        elm_agents = payload.get("agents", {}) 
        
        # Grid normalisieren
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

        if ftf_agent:
            ftf_id = ftf_agent["agent_id"]
            ftf_pos = (int(ftf_agent["x"]), int(ftf_agent["y"]))
            agent_manager.log_to_system(f"FTF Mission: Berechne Pfad für {ftf_id}...", "info")

            path_a, _, _ = planner.a_star(ftf_pos, start_tuple, grid, travel_mode="ftf")
            path_b, cost_b, _ = planner.a_star(start_tuple, goal_tuple, grid, travel_mode="ftf")
            
            if path_a and path_b:
                full_path = path_a + path_b[1:]
                await socket_manager.emit_event('path_complete', {
                    "status": 1, "cost": float(cost_b), "path": full_path
                }, target_sid=sid)
                agent_manager.log_to_system(f"Mission bestätigt. FTF startet.", "success")
                asyncio.create_task(agent_manager.execute_mission(ftf_id, full_path, start_tuple, goal_tuple))
            else:
                agent_manager.log_to_system("Kein Pfad für FTF möglich.", "warning")
                await socket_manager.emit_event('path_complete', {"status": 0, "path": []}, target_sid=sid)
        else:
            path, cost, msg = planner.a_star(start_tuple, goal_tuple, grid, travel_mode="chain")
            if path:
                agent_manager.log_to_system(f"Kette gefunden: {msg}", "success")
                await socket_manager.emit_event('path_complete', {"status": 1, "cost": float(cost), "path": path}, target_sid=sid)
            else:
                agent_manager.log_to_system("Keine Kette möglich.", "warning")
                await socket_manager.emit_event('path_complete', {"status": 0, "path": []}, target_sid=sid)

    except Exception as e:
        logger.error(f"Fehler in Pfadplanung: {e}", exc_info=True)

@socket_manager.sio.on('write_nfc')
async def handle_write_nfc(sid, data):
    """Hardware-Schreibvorgang mit Thread-Auslagerung."""
    tag_content = data if isinstance(data, str) else data.get("text", "")
    
    if nfc_manager.get_status() == "missing":
        await socket_manager.emit_event('system_log', {
            "message": "🚫 NFC Fehler: Hardware (RC522) nicht gefunden!",
            "level": "error"
        }, target_sid=sid)
        return

    result = await asyncio.to_thread(nfc_manager.write_tag, tag_content)
    
    if result == "success":
        msg, level = f"✅ NFC: '{tag_content}' erfolgreich geschrieben.", "success"
    elif result == "timeout":
        msg, level = "❌ NFC Abbruch: Kein Chip erkannt.", "warning"
    else:
        msg, level = "🚫 NFC: Hardware-Fehler.", "error"

    await socket_manager.emit_event('system_log', {"message": msg, "level": level}, target_sid=sid)

@socket_manager.sio.on('set_mode')
async def handle_set_mode(sid, data):
    """Wechselt zwischen Simulation und Hardware-Modus."""
    try:
        mode_str = data if isinstance(data, str) else data.get("mode")
        new_mode = SystemMode(mode_str)
        if config.set_mode(new_mode):
            if new_mode == SystemMode.HARDWARE:
                agent_manager.clear_all_agents()
                agent_manager.log_to_system("Hardware aktiv: Warte auf ROS...", "warning")
            else:
                agent_manager.log_to_system("Simulation aktiv.", "info")
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