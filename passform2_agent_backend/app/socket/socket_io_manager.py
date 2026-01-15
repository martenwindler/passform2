import socketio
import asyncio
import logging
from typing import Any, Optional, Callable
from app.config import config

# Logger konfigurieren
logger = logging.getLogger("socketio")

class SocketIOManager:
    def __init__(self):
        # Initialisierung des asynchronen Servers
        self.sio = socketio.AsyncServer(
            async_mode="asgi", 
            cors_allowed_origins="*",
            # Verhindert, dass langsame Clients den Server blockieren
            ping_timeout=5,
            ping_interval=10
        )
        self.app_sio = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.active_clients = set()
        
        # Callbacks für Initialisierungsdaten neuer Clients
        self.on_new_client_callback: Optional[Callable] = None
        
        self._register_events()

    def attach_fastapi(self, fastapi_app):
        """Verbindet Socket.IO mit der FastAPI Instanz"""
        self.app_sio = socketio.ASGIApp(self.sio, other_asgi_app=fastapi_app)
        # Wir greifen den Loop direkt hier ab, da attach_fastapi 
        # im Haupt-Thread während des App-Starts aufgerufen wird.
        try:
            self.loop = asyncio.get_running_loop()
        except RuntimeError:
            self.loop = asyncio.get_event_loop()
        
        logger.info("Socket.IO an FastAPI angehängt und Event-Loop registriert.")

    def set_on_new_client_callback(self, callback: Callable):
        """Erlaubt dem AgentManager, sich für neue Verbindungen zu registrieren"""
        self.on_new_client_callback = callback

    async def _handle_new_connection(self, sid):
        """Interner Handler für neue Clients"""
        # 1. Aktuellen System-Modus senden (Hardware/Simulation)
        await self.emit_event('mode', config.get_current_mode().value, target_sid=sid)
        
        # 2. Falls ein Callback (z.B. vom AgentManager) existiert, Initialdaten senden
        if self.on_new_client_callback:
            self.on_new_client_callback()
        else:
            # Fallback falls Callback noch nicht registriert
            logger.warning(f"Client {sid} verbunden, aber AgentManager Callback fehlt.")

    def _register_events(self):
        @self.sio.event
        async def connect(sid, environ):
            self.active_clients.add(sid)
            logger.info(f"Client verbunden: {sid} (Total: {len(self.active_clients)})")
            await self._handle_new_connection(sid)

        @self.sio.event
        async def disconnect(sid):
            self.active_clients.discard(sid)
            logger.info(f"Client getrennt: {sid} (Total: {len(self.active_clients)})")

    async def emit_event(self, event_name: str, data: Any, target_sid: Optional[str] = None):
        """Asynchrone Methode zum Senden von Events"""
        try:
            await self.sio.emit(event_name, data, room=target_sid)
        except Exception as e:
            logger.error(f"Fehler beim Emit ({event_name}): {e}")

    def emit_event_sync(self, event_name: str, data: Any, target_sid: Optional[str] = None):
        """
        Threadsichere, synchrone Methode zum Senden von Events.
        Kann aus ROS-Callbacks oder anderen Threads aufgerufen werden.
        """
        if self.loop and self.loop.is_running():
            # Übergibt die Koroutine an den Haupt-Event-Loop
            asyncio.run_coroutine_threadsafe(
                self.emit_event(event_name, data, target_sid),
                self.loop
            )
        else:
            # Während des Startvorgangs oder Shutdowns puffern wir nicht, 
            # sondern loggen nur, um Hänger zu vermeiden.
            logger.debug(f"Event {event_name} verworfen: Loop nicht bereit.")

# Singleton Instanz
socket_manager = SocketIOManager()