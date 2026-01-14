from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from .managers.node_manager import router as node_manager_router
from .managers.path_manager import router as path_manager_router
from .system_api import router as system_router
from .socket.socket_io_manager import socket_manager
from .managers.agent_manager import agent_manager
from .ros.ros_client import get_ros_client

# ROS Client beim Start initialisieren
get_ros_client()

# FastAPI-App konfigurieren
fastapi_app = FastAPI(
    title="PassForm Agent Backend",
    description="REST API und WebSocket-Server für ROS-Kommunikation",
    version="1.0.0"
)

# Middleware & Router
fastapi_app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
fastapi_app.include_router(node_manager_router, prefix="/api/nodes")
fastapi_app.include_router(path_manager_router, prefix="/api/path")
fastapi_app.include_router(system_router, prefix="/api/system")

@fastapi_app.get("/")
async def root():
    return {
        "service": "PassForm Agent Backend",
        "version": "1.0.0",
        "endpoints": {
            "rest_api": "/api",
            "websocket": "/socket.io"
        }
    }

@fastapi_app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    return JSONResponse(
        status_code=500,
        content={"message": str(exc), "type": exc.__class__.__name__},
    )

# 🔗 Socket.IO + FastAPI verbinden
socket_manager.attach_fastapi(fastapi_app)
app = socket_manager.app_sio

if __name__ == "__main__":
    import uvicorn
    print("🟢 Starte Uvicorn aus main.py")
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=False)
