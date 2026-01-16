import logging
import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware

# System-Importe
from app.config import config, SystemMode
from .managers.socket_io_manager import socket_manager
from .managers.agent_manager import agent_manager
from .managers.nfc_manager import nfc_manager 
from .managers.config_manager import config_manager
from .managers.node_manager import node_manager, router as node_manager_router
from .managers.path_manager import router as path_manager_router

from .system_api import router as system_router
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
    
    # 0. SSoT laden (config.json)
    agent_manager.load_from_ssot()

    # 1. ROS-Nodes Orchestrierung
    try:
        # A) Zentralen System-Monitor/Planner starten
        node_manager.start_system_nodes()
        
        # B) Agenten-Nodes aus SSoT spawnen
        agents_to_spawn = [a.to_dict() for a in agent_manager.agents.values()]
        node_manager.start_agents_from_config(agents_to_spawn)
        
        logger.info(f"✅ Orchestrator: System-Nodes und {len(agents_to_spawn)} Agenten gestartet.")
    except Exception as e:
        logger.error(f"❌ Kritischer Fehler beim Starten der ROS-Nodes: {e}")

    # 2. ROS-Client initialisieren
    try:
        get_ros_client()
    except Exception as e:
        logger.warning(f"⚠️ ROS-Client konnte nicht initialisiert werden: {e}")

    # 3. NFC-Reader starten
    if nfc_manager.get_status() == "online":
        nfc_manager.start_reading()
        logger.info("📡 NFC-Hintergrund-Thread gestartet.")

    # 4. NEU: Health-Broadcast-Loop für Signalstärken starten
    health_task = asyncio.create_task(agent_manager.start_health_broadcast_loop())
    logger.info("💓 Health-Monitoring-Loop für Agenten gestartet.")
    
    # 5. Callback für neue Socket-Verbindungen (Daten-Synchronisation)
    async def sync_state_for_client(sid):
        try:
            logger.info(f"🔗 Synchronisiere neuen Client: {sid}")
            
            # NFC & Modus Status
            await socket_manager.emit_event('nfc_status', {"status": nfc_manager.get_status()}, target_sid=sid)
            await socket_manager.emit_event('mode', config.get_current_mode().value, target_sid=sid)
            
            # Begrüßung & System-Log
            await socket_manager.emit_event('system_log', {
                "message": f"System bereit (Hz: {config.heartbeat_hz})", 
                "level": "success"
            }, target_sid=sid)
            
            # Bestehende Logs senden
            for log_entry in agent_manager.get_logs_history():
                await socket_manager.emit_event('system_log', log_entry, target_sid=sid)
            
            # Aktuelle Agenten-Topologie inkl. Signalstärke senden
            agent_manager.send_agent_list()
            
        except Exception as e:
            logger.error(f"❌ Fehler beim Client-Sync: {e}")

    socket_manager.set_on_new_client_callback(sync_state_for_client)
    
    yield # --- App läuft hier ---
    
    # --- SHUTDOWN ---
    logger.info("🛑 PassForm Backend wird beendet...")
    health_task.cancel() # Hintergrund-Task beenden
    nfc_manager.running = False 
    node_manager.kill_all_nodes()

# --- APP SETUP ---
fastapi_app = FastAPI(
    title="PassForm Agent Backend",
    version="1.7.0",
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
    try:
        start, goal = payload.get("start"), payload.get("goal")
        elm_agents = payload.get("agents", {}) 
        
        # Umwandlung für Planner
        grid = { (int(a.get("x")), int(a.get("y"))): a for a in (elm_agents.values() if isinstance(elm_agents, dict) else elm_agents) }
        
        ftf_agent = next((a for a in grid.values() if a.get("module_type") == "ftf"), None)
        start_tuple = (int(start["x"]), int(start["y"]))
        goal_tuple = (int(goal["x"]), int(goal["y"]))

        if ftf_agent:
            ftf_id = ftf_agent["agent_id"]
            ftf_pos = (int(ftf_agent["x"]), int(ftf_agent["y"]))
            
            # Kombinierter Pfad (FTF -> Pickup -> Dropoff)
            path_a, _, _ = planner.a_star(ftf_pos, start_tuple, grid, travel_mode="ftf")
            path_b, cost_b, _ = planner.a_star(start_tuple, goal_tuple, grid, travel_mode="ftf")
            
            if path_a and path_b:
                full_path = path_a + path_b[1:]
                await socket_manager.emit_event('path_complete', {"status": 0, "cost": float(cost_b), "path": full_path}, target_sid=sid)
                # Mission asynchron ausführen (Animation/Simulation)
                asyncio.create_task(agent_manager.execute_mission(ftf_id, full_path, start_tuple, goal_tuple))
            else:
                agent_manager.log_to_system("Kein Pfad für FTF möglich.", "warning")
        else:
            # Reine Ketten-Planung (ohne FTF Bewegung)
            path, cost, _ = planner.a_star(start_tuple, goal_tuple, grid, travel_mode="chain")
            if path:
                await socket_manager.emit_event('path_complete', {"status": 0, "cost": float(cost), "path": path}, target_sid=sid)

    except Exception as e:
        logger.error(f"Fehler in Pfadplanung: {e}")

@socket_manager.sio.on('update_planning_config')
async def handle_update_planning_config(sid, data):
    try:
        config_manager.current_data["config"]["planning_weights"] = data
        config_manager.save_to_ssot()
        agent_manager.log_to_system("⚙️ Planungs-Parameter in SSoT gesichert.", "success")
    except Exception as e:
        logger.error(f"Fehler beim Gewichts-Update: {e}")

@socket_manager.sio.on('write_nfc')
async def handle_write_nfc(sid, data):
    tag_content = data if isinstance(data, str) else data.get("text", "")
    if nfc_manager.get_status() == "missing":
        agent_manager.log_to_system("🚫 NFC Fehler: Hardware fehlt.", "error")
        return
    result = await asyncio.to_thread(nfc_manager.write_tag, tag_content)
    agent_manager.log_to_system(f"NFC: {result}", "success" if result == "success" else "warning")

@socket_manager.sio.on('set_mode')
async def handle_set_mode(sid, data):
    mode_str = data if isinstance(data, str) else data.get("mode")
    try:
        new_mode = SystemMode(mode_str)
        if config.set_mode(new_mode):
            agent_manager.clear_all_agents()
            await socket_manager.emit_event('mode', mode_str)
            agent_manager.log_to_system(f"🔄 System auf {mode_str} umgeschaltet.", "info")
    except Exception as e:
        logger.error(f"Fehler beim Modus-Wechsel: {e}")

@socket_manager.sio.on('set_heartbeat_rate')
async def handle_set_heartbeat_rate(sid, data):
    try:
        # Daten können ein Float oder ein Dict sein
        new_hz = float(data if isinstance(data, (int, float)) else data.get("hz", 1.0))
        
        if config.set_heartbeat_hz(new_hz):
            # Bestätigung an alle Clients
            await socket_manager.emit_event('system_log', {
                "message": f"⚡ System-Takt auf {new_hz}Hz geändert.",
                "level": "info"
            })
            
            # WICHTIG: Die Agent-Liste neu senden, damit das UI die 
            # Signalstärken-Berechnung sofort anpasst
            agent_manager.send_agent_list()
            
    except Exception as e:
        logger.error(f"Fehler beim Takt-Update: {e}")

# --- API ENDPUNKTE ---
fastapi_app.include_router(node_manager_router, prefix="/api/nodes")
fastapi_app.include_router(path_manager_router, prefix="/api/path")
fastapi_app.include_router(system_router, prefix="/api/system")

@fastapi_app.get("/")
async def root():
    return {
        "service": "PassForm Backend", 
        "mode": config.get_current_mode().value,
        "heartbeat_ssot": f"{config.heartbeat_hz}Hz",
        "nodes_active": len(node_manager.processes)
    }

# --- SOCKET.IO INTEGRATION ---
socket_manager.attach_fastapi(fastapi_app)
app = socket_manager.app_sio

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")