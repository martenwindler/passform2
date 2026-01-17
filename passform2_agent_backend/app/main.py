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

# --- HARDWARE REGISTRY ---
hardware_registry = {}

# --- LIFESPAN MANAGEMENT ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Verwaltet Startup und Shutdown des Backends."""
    logger.info(f"🚀 PassForm Backend startet im Modus: {config.get_current_mode().value}")
    
    # 0. SSoT laden
    agent_manager.load_from_ssot()

    # 1. ROS-Nodes Orchestrierung
    await asyncio.sleep(0.5)
    try:
        node_manager.start_system_nodes()
        agents_to_spawn = [a.to_dict() for a in agent_manager.agents.values()]
        node_manager.start_agents_from_config(agents_to_spawn)
        logger.info(f"✅ Orchestrator: System-Nodes und {len(agents_to_spawn)} Agenten gestartet.")
    except Exception as e:
        logger.error(f"❌ Kritischer Fehler beim Starten der ROS-Nodes: {e}")

    # 2. ROS-Client
    try:
        get_ros_client()
    except Exception as e:
        logger.warning(f"⚠️ ROS-Client konnte nicht sofort initialisiert werden: {e}")

    # 3. Lokaler NFC-Reader
    if nfc_manager.get_status() == "online":
        nfc_manager.start_reading()
        logger.info("📡 Lokaler NFC-Hintergrund-Thread gestartet.")

    # 4. Health-Monitoring Task
    health_task = asyncio.create_task(agent_manager.start_health_broadcast_loop())
    
    # --- SYNC LOGIK FÜR NEUE CLIENTS ---
    async def sync_state_for_client(sid):
        """Versorgt neue Browser-Verbindungen sofort mit allen SSoT-Daten."""
        try:
            current_mode = config.get_current_mode().value
            await socket_manager.emit_event('mode', current_mode, target_sid=sid)
            await socket_manager.emit_event('hardware_update', list(hardware_registry.values()), target_sid=sid)
            await socket_manager.emit_event('nfc_status', {"status": nfc_manager.get_status()}, target_sid=sid)
            
            agents = [a.to_dict() for a in agent_manager.agents.values()]
            await socket_manager.emit_event('active_agents', agents, target_sid=sid)

            await socket_manager.emit_event('system_log', {
                "message": f"Synchronisation abgeschlossen (Modus: {current_mode})", 
                "level": "success"
            }, target_sid=sid)
        except Exception as e:
            logger.error(f"❌ Fehler beim Client-Sync für SID {sid}: {e}")

    socket_manager.set_on_new_client_callback(sync_state_for_client)
    
    # --- APP LÄUFT ---
    yield 
    
    # --- SHUTDOWN (Wird bei STRG+C ausgelöst) ---
    logger.info("🛑 Shutdown eingeleitet: Räume Ressourcen auf...")
    
    # 1. Health-Task abbrechen
    health_task.cancel()
    try:
        await asyncio.wait_for(health_task, timeout=2.0)
    except (asyncio.CancelledError, asyncio.TimeoutError):
        logger.info("✅ Health-Monitoring Task beendet.")

    # 2. NFC-Manager stoppen
    nfc_manager.running = False
    
    # 3. ROS-Nodes terminieren
    logger.info("📡 Beende alle ROS-Nodes...")
    node_manager.kill_all_nodes()
    
    # 4. Clients informieren
    await socket_manager.emit_event('system_log', {
        "message": "Backend wurde manuell beendet (SIGINT).",
        "level": "warning"
    })
    
    # Kurze Pause für den finalen Netzwerk-Flush
    await asyncio.sleep(0.2)
    logger.info("✅ PassForm Backend sauber beendet.")

# --- APP SETUP ---
fastapi_app = FastAPI(title="PassForm Agent Backend", version="1.7.5", lifespan=lifespan)

fastapi_app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- API ROUTER ---
fastapi_app.include_router(node_manager_router, prefix="/api/nodes")
fastapi_app.include_router(path_manager_router, prefix="/api/path")
fastapi_app.include_router(system_router, prefix="/api/system")

@fastapi_app.get("/")
async def root():
    return {
        "service": "PassForm Backend", 
        "mode": config.get_current_mode().value,
        "nodes_active": len(node_manager.processes)
    }

# --- SOCKET.IO EVENT HANDLER ---

@socket_manager.sio.on('connect')
async def handle_connect(sid, environ):
    logger.info(f"🔗 Socket verbunden: {sid}")

@socket_manager.sio.on('disconnect')
async def handle_disconnect(sid):
    if sid in hardware_registry:
        lost_device = hardware_registry.pop(sid)
        logger.info(f"🖥️ Hardware-Pi verloren: {lost_device.get('pi_id')}")
        await socket_manager.emit_event('hardware_update', list(hardware_registry.values()))
    else:
        logger.info(f"❌ Browser-Client getrennt: {sid}")

@socket_manager.sio.on('pi_hardware_update')
async def handle_pi_hardware(sid, data):
    hardware_registry[sid] = data
    logger.info(f"🖥️ Pi Hardware Update: {data.get('pi_id')}")
    await socket_manager.emit_event('hardware_update', list(hardware_registry.values()))

@socket_manager.sio.on('rfid_scanned')
async def handle_rfid_scanned(sid, data):
    logger.info(f"🎴 RFID Scan: {data.get('id')}")
    await socket_manager.emit_event('rfid_scanned', data)

@socket_manager.sio.on('plan_path')
async def handle_plan_path(sid, payload):
    try:
        start, goal = payload.get("start"), payload.get("goal")
        elm_agents = payload.get("agents", {}) 
        raw_agents = elm_agents.values() if isinstance(elm_agents, dict) else elm_agents
        grid = { (int(a.get("x")), int(a.get("y"))): a for a in raw_agents if a }
        
        start_t = (int(start["x"]), int(start["y"]))
        goal_t = (int(goal["x"]), int(goal["y"]))
        
        ftf_agent = next((a for a in grid.values() if a.get("module_type") == "ftf"), None)

        if ftf_agent:
            ftf_id = ftf_agent["agent_id"]
            ftf_pos = (int(ftf_agent["x"]), int(ftf_agent["y"]))
            path_a, _, _ = planner.a_star(ftf_pos, start_t, grid, travel_mode="ftf")
            path_b, cost_b, _ = planner.a_star(start_t, goal_t, grid, travel_mode="ftf")
            
            if path_a and path_b:
                full_path = path_a + path_b[1:]
                await socket_manager.emit_event('path_complete', {"status": 0, "cost": float(cost_b), "path": full_path}, target_sid=sid)
                asyncio.create_task(agent_manager.execute_mission(ftf_id, full_path, start_t, goal_t))
        else:
            path, cost, _ = planner.a_star(start_t, goal_t, grid, travel_mode="chain")
            if path:
                await socket_manager.emit_event('path_complete', {"status": 0, "cost": float(cost), "path": path}, target_sid=sid)
    except Exception as e:
        logger.error(f"❌ Pfadplanung Fehler: {e}")

@socket_manager.sio.on('set_mode')
async def handle_set_mode(sid, data):
    mode_str = data if isinstance(data, str) else data.get("mode")
    try:
        new_mode = SystemMode(mode_str)
        if config.set_mode(new_mode):
            agent_manager.clear_all_agents()
            await socket_manager.emit_event('mode', mode_str)
            logger.info(f"🔄 Modus auf {mode_str} gewechselt.")
    except Exception as e:
        logger.error(f"❌ Modus-Wechsel Fehler: {e}")

# --- SOCKET.IO INTEGRATION ---
socket_manager.attach_fastapi(fastapi_app)
app = socket_manager.app_sio

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="127.0.0.1", port=8080, reload=True, ws="websockets")