import logging
import asyncio
import json
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

    # 2. ROS-Client & Health Task
    try:
        get_ros_client()
    except Exception:
        pass
    health_task = asyncio.create_task(agent_manager.start_health_broadcast_loop())

    # --- DEFINITION SYNC CALLBACK ---
    async def sync_state_for_client(sid):
        try:
            current_mode = config.get_current_mode().value
            
            # 1. Status senden
            await socket_manager.emit_event('mode', current_mode, target_sid=sid)
            await socket_manager.emit_event('hardware_update', list(hardware_registry.values()), target_sid=sid)
            
            # 2. Agenten-Sync
            config_manager.load_from_ssot() 
            agents_list = config_manager.current_data.get("agents", [])
            
            if isinstance(agents_list, dict):
                agents_list = list(agents_list.values())

            await socket_manager.emit_event('active_agents', {"agents": agents_list}, target_sid=sid)
            logger.info(f"✅ Sync an Client {sid}: {len(agents_list)} Agenten.")
        except Exception as e: # <--- DIESER BLOCK HAT GEFEHLT
            logger.error(f"❌ Fehler beim Sync für Client {sid}: {e}")
    
    # --- APP LÄUFT ---
    yield 
    
    # --- SHUTDOWN (STRG+C) ---
    logger.info("🛑 Shutdown eingeleitet: Räume Ressourcen auf...")
    health_task.cancel()
    try:
        await asyncio.wait_for(health_task, timeout=2.0)
    except:
        pass

    nfc_manager.running = False
    node_manager.kill_all_nodes()
    
    await socket_manager.emit_event('system_log', {
        "message": "Backend beendet.",
        "level": "warning"
    })
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

@socket_manager.sio.on('push_config')
async def handle_push_config(sid, data):
    print(f"\n🔥 BACKEND: Event 'push_config' von {sid} empfangen!")
    
    if "agents" in data:
        count = len(data["agents"])
        print(f"📦 BACKEND: Erhaltene Agenten-Anzahl: {count}")
        
        config_manager.current_data["agents"] = data["agents"]
        success = config_manager.save_to_ssot()
        
        if success:
            print(f"💾 BACKEND: SSoT (config.json) erfolgreich geschrieben.")
            # Broadcast zurück
            await socket_manager.emit_event('active_agents', {"agents": data["agents"]})
        else:
            print(f"❌ BACKEND: Fehler beim Schreiben der config.json!")

@socket_manager.sio.on('plan_path')
async def handle_plan_path_routing(sid, payload):
    await handle_plan_path_socket_io(sid, payload)

async def handle_plan_path_socket_io(sid, data):
    try:
        inner_payload = data.get("payload", {})
        start = inner_payload.get("start")
        goal = inner_payload.get("goal")
        if not start or not goal: return

        f_weights = inner_payload.get("weights", {})
        val_weights = {
            "execution_time_default": max(0.1, float(f_weights.get("execution_time_default", 1.0))),
            "complex_module_time": max(0.1, float(f_weights.get("complex_module_time", 3.5))),
            "human_extra_weight": max(0.0, float(f_weights.get("human_extra_weight", 1.0))),
            "proximity_penalty": max(0.0, float(f_weights.get("proximity_penalty", 0.5))),
            "hardware_safety_factor": max(1.0, float(f_weights.get("hardware_safety_factor", 1.2)))
        }

        elm_agents = inner_payload.get("agents", {})
        raw_agents = elm_agents.values() if isinstance(elm_agents, dict) else elm_agents
        grid = { (int(a["x"]), int(a["y"])): a for a in raw_agents if a }

        path, cost, msg = planner.a_star(
            (int(start["x"]), int(start["y"])), 
            (int(goal["x"]), int(goal["y"])), 
            grid, travel_mode="chain", weights=val_weights
        )

        if path:
            formatted = [{"agent_id": str(s.get("agent_id", "unknown")), "position": {"x": int(s.get("x", 0)), "y": int(s.get("y", 0))}} for s in path]
            await socket_manager.emit_event('path_complete', {"status": 200, "cost": float(cost), "path": formatted}, target_sid=sid)
        else:
            await socket_manager.emit_event('path_complete', {"status": 404, "path": []}, target_sid=sid)
    except Exception as e:
        logger.error(f"❌ Planungsfehler: {e}")

@socket_manager.sio.on('set_mode')
async def handle_set_mode(sid, data):
    mode_str = data if isinstance(data, str) else data.get("mode")
    try:
        if config.set_mode(SystemMode(mode_str)):
            agent_manager.clear_all_agents()
            await socket_manager.emit_event('mode', mode_str)
    except Exception as e:
        logger.error(f"❌ Modus-Wechsel Fehler: {e}")

socket_manager.attach_fastapi(fastapi_app)
app = socket_manager.app_sio

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="127.0.0.1", port=8080, reload=True, ws="websockets")