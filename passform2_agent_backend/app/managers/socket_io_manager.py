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
    Optimiert für SSoT-Synchronisation und stabile Brücken zwischen WSL2 und Browser.
    """
    def __init__(self):
        # Erhöhte Timeouts und Deaktivierung der CORS-Prüfung auf Engine.IO Ebene
        # check_cors=False verhindert Handshake-Abbrüche bei WSL2-Loopback-Adressen
        self.sio = socketio.AsyncServer(
            async_mode="asgi",
            cors_allowed_origins="*",
            check_cors=False,         # Wichtig für WSL2 <-> Windows Handshake
            always_connect=True,      # Erlaubt Handshake-Fortsetzung trotz Protokoll-Aushandlung
            ping_timeout=30,          # Zeit in Sek. bis Client als offline gilt
            ping_interval=10,         # Herzschlag-Intervall
            engineio_logger=False      # Auf True setzen für tiefes Debugging im Terminal
        )
        self.app_sio = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.active_clients = set()
        
        # Callback für Initialdaten (wird von main.py gesetzt)
        self.on_new_client_callback: Optional[Callable] = None
        
        self._register_events()

    def attach_fastapi(self, fastapi_app):
        """
        Verbindet Socket.IO mit der FastAPI-Instanz.
        Der Wrapper sorgt dafür, dass /socket.io Anfragen abgefangen werden.
        """
        self.app_sio = socketio.ASGIApp(
            socketio_server=self.sio, 
            other_asgi_app=fastapi_app,
            socketio_path="/socket.io"
        )
        
        # Sicherstellen, dass wir den korrekten Event-Loop für ROS-Threads haben
        try:
            self.loop = asyncio.get_running_loop()
        except RuntimeError:
            self.loop = asyncio.get_event_loop()
        
        logger.info("✅ Socket.IO erfolgreich an FastAPI angehängt (CORS: * | check_cors: False).")

    def set_on_new_client_callback(self, callback: Callable):
        """Registriert die Funktion für den initialen Datensync."""
        self.on_new_client_callback = callback

    async def _handle_new_connection(self, sid):
        """Ablauf bei neuer Verbindung eines Clients."""
        try:
            # 1. Aktuellen Modus sofort senden (für Navbar-Dots)
            current_mode = config.get_current_mode().value
            await self.emit_event('mode', current_mode, target_sid=sid)
            
            # 2. Sync-Funktion aus main.py aufrufen (Logs, Hardware & Agenten)
            if self.on_new_client_callback:
                if inspect.iscoroutinefunction(self.on_new_client_callback):
                    await self.on_new_client_callback(sid)
                else:
                    self.on_new_client_callback(sid)
            else:
                logger.warning(f"⚠️ Kein Sync-Callback für Client {sid} hinterlegt.")
        except Exception as e:
            logger.error(f"❌ Fehler bei Initial-Sync für SID {sid}: {e}")

    def _register_events(self):
        """Registriert Standard-Events mit verbessertem Logging."""
        @self.sio.event
        async def connect(sid, environ):
            self.active_clients.add(sid)
            origin = environ.get('HTTP_ORIGIN', 'unbekannt')
            logger.info(f"🔗 Client verbunden: {sid} | Origin: {origin} | Gesamt: {len(self.active_clients)}")
            
            # Grüne Lampe im Frontend triggern
            await self.sio.emit('socket_status', True, room=sid)
            await self._handle_new_connection(sid)

        @self.sio.event
        async def disconnect(sid):
            self.active_clients.discard(sid)
            logger.info(f"❌ Client getrennt: {sid} | Verbleibend: {len(self.active_clients)}")
            
            # Rote Lampe im Frontend triggern (Broadcast, da sid nicht mehr adressierbar)
            await self.emit_event('socket_status', False)

    async def emit_event(self, event_name: str, data: Any, target_sid: Optional[str] = None):
        """Asynchrones Senden von Events an Raum oder Einzelclient."""
        try:
            # Wenn target_sid None ist, wird an alle gesendet (Broadcast)
            await self.sio.emit(event_name, data, room=target_sid)
        except Exception as e:
            logger.error(f"❌ Fehler beim Senden von '{event_name}': {e}")

    def emit_event_sync(self, event_name: str, data: Any, target_sid: Optional[str] = None):
        """
        Threadsicheres Senden für ROS-Threads.
        Brücke zwischen synchroner ROS-Welt und asynchroner Socket-Welt.
        """
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.emit_event(event_name, data, target_sid),
                self.loop
            )
        else:
            logger.error(f"❌ Senden von '{event_name}' fehlgeschlagen: Event-Loop nicht aktiv.")

    def active_clients_count(self) -> int:
        """Gibt die Anzahl der verbundenen Browser-Tabs zurück."""
        return len(self.active_clients)

# Singleton Instanz
socket_manager = SocketIOManager()