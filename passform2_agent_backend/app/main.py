import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware

# Importe deiner Module
from .managers.node_manager import router as node_manager_router
from .managers.path_manager import router as path_manager_router
from .system_api import router as system_router
from .socket.socket_io_manager import socket_manager
from .managers.agent_manager import agent_manager
from .ros.ros_client import get_ros_client, restart_ros_client

# Logger Setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("main")

# --- LIFESPAN MANAGEMENT ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    # STARTUP:
    logger.info("🚀 PassForm Backend wird gestartet...")
    
    # 1. ROS Client initialisieren
    get_ros_client()
    
    # 2. Socket.IO Callbacks registrieren (verhindert zirkuläre Importe)
    socket_manager.set_on_new_client_callback(agent_manager.send_agent_list)
    
    yield
    
    # SHUTDOWN:
    logger.info("🛑 PassForm Backend wird beendet...")
    # Hier alle Simulations-Nodes killen, damit keine ROS-Geisterprozesse bleiben
    from .managers.node_manager import node_manager
    node_manager.kill_all_nodes()

# --- APP KONFIGURATION ---
fastapi_app = FastAPI(
    title="PassForm Agent Backend",
    description="REST API und WebSocket-Server für ROS-Kommunikation",
    version="1.0.0",
    lifespan=lifespan
)

# Middleware
fastapi_app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Router einbinden
fastapi_app.include_router(node_manager_router, prefix="/api/nodes")
fastapi_app.include_router(path_manager_router, prefix="/api/path")
fastapi_app.include_router(system_router, prefix="/api/system")

@fastapi_app.get("/")
async def root():
    return {
        "service": "PassForm Agent Backend",
        "version": "1.0.0",
        "status": "online"
    }

# Zentrales Error-Handling
@fastapi_app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled Exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"message": str(exc), "type": exc.__class__.__name__},
    )

# 🔗 Socket.IO + FastAPI verbinden
socket_manager.attach_fastapi(fastapi_app)
# Dies ist das Objekt, das uvicorn starten muss
app = socket_manager.app_sio

if __name__ == "__main__":
    import uvicorn
    logger.info("🟢 Starte Uvicorn Server auf Port 8000")
    # Beachte: "app.main:app" Pfad muss zu deiner Ordnerstruktur passen
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=False)
