# socket_io_manager.py
import socketio
import asyncio
from typing import Any, Optional
from app.config import config

class SocketIOManager:
    def __init__(self):
        self.sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")
        self.app_sio = None  # wird später gesetzt
        self.active_clients = set()
        self._register_events()

        self.loop = None  # Speichert später den Haupt-Eventloop

    def attach_fastapi(self, fastapi_app):
        self.app_sio = socketio.ASGIApp(self.sio, other_asgi_app=fastapi_app)
        self.loop = asyncio.get_event_loop()  
    
    def _new_client(self):
        """Initialisiert den Socket.IO-Client"""
        from app.managers.agent_manager import agent_manager
        agent_manager.send_agent_list()
        self.emit_event_sync('mode', config.get_current_mode().value)


        

    def _register_events(self):
        @self.sio.event
        async def connect(sid, environ):
            self.active_clients.add(sid)
            print(f"Client verbunden: {sid}")
            self._new_client()

        @self.sio.event
        async def disconnect(sid):
            self.active_clients.discard(sid)
            print(f"Client getrennt: {sid}")

    async def emit_event(self, event_name: str, data: Any, target_sid: Optional[str] = None):
        if target_sid:
            await self.sio.emit(event_name, data, room=target_sid)
        else:
            await self.sio.emit(event_name, data)

    def emit_event_sync(self, event_name, data, target_sid=None):
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.emit_event(event_name, data, target_sid),
                self.loop
            )
        else:
            print("Kein gültiger Event Loop zum Emittieren!")


socket_manager = SocketIOManager()