import socketio
import asyncio
import logging
import inspect
from typing import Any, Optional, Callable

# Absoluter Import, um ModuleNotFoundError zu vermeiden
from app.config import config 

logger = logging.getLogger("socketio")

class SocketIOManager:
    """
    Zentrale Verwaltung der WebSocket-Kommunikation.
    Hält die Verbindung zwischen dem FastAPI-Server und dem Elm-Frontend.
    """
    def __init__(self):
        self.sio = socketio.AsyncServer(
            async_mode="asgi", 
            cors_allowed_origins="*",
            ping_timeout=10,
            ping_interval=25
        )
        self.app_sio = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.active_clients = set()
        
        # Callback für Initialdaten (wird von main.py gesetzt)
        self.on_new_client_callback: Optional[Callable] = None
        
        self._register_events()

    def attach_fastapi(self, fastapi_app):
        """Verbindet Socket.IO mit der FastAPI-Instanz."""
        self.app_sio = socketio.ASGIApp(self.sio, other_asgi_app=fastapi_app)
        
        # Event-Loop für thread-sicheres Senden (wichtig für ROS-Callbacks)
        try:
            self.loop = asyncio.get_running_loop()
        except RuntimeError:
            self.loop = asyncio.get_event_loop()
        
        logger.info("✅ Socket.IO erfolgreich an FastAPI angehängt.")

    def set_on_new_client_callback(self, callback: Callable):
        """Registriert die Funktion für den initialen Datensync."""
        self.on_new_client_callback = callback

    async def _handle_new_connection(self, sid):
        """Ablauf bei neuer Verbindung eines Clients."""
        # 1. Aktuellen Modus sofort senden
        current_mode = config.get_current_mode().value
        await self.emit_event('mode', current_mode, target_sid=sid)
        
        # 2. Sync-Funktion aus main.py aufrufen (Logs & Agenten)
        if self.on_new_client_callback:
            if inspect.iscoroutinefunction(self.on_new_client_callback):
                await self.on_new_client_callback(sid)
            else:
                self.on_new_client_callback(sid)
        else:
            logger.warning(f"⚠️ Kein Sync-Callback für Client {sid} registriert.")

    def _register_events(self):
        """Registriert Standard-Events wie Connect und Disconnect."""
        @self.sio.event
        async def connect(sid, environ):
            self.active_clients.add(sid)
            logger.info(f"🔗 Client verbunden: {sid} (Online: {len(self.active_clients)})")
            await self._handle_new_connection(sid)

        @self.sio.event
        async def disconnect(sid):
            self.active_clients.discard(sid)
            logger.info(f"❌ Client getrennt: {sid}")

    async def emit_event(self, event_name: str, data: Any, target_sid: Optional[str] = None):
        """Asynchrones Senden von Events."""
        try:
            await self.sio.emit(event_name, data, room=target_sid)
        except Exception as e:
            logger.error(f"Fehler beim Senden von {event_name}: {e}")

    def emit_event_sync(self, event_name: str, data: Any, target_sid: Optional[str] = None):
        """Threadsicheres Senden für ROS-Threads."""
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.emit_event(event_name, data, target_sid),
                self.loop
            )

    def active_clients_count(self) -> int:
        """Gibt die Anzahl der verbundenen Browser-Tabs zurück."""
        return len(self.active_clients)

# Singleton Instanz
socket_manager = SocketIOManager()