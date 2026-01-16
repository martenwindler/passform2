import { Elm } from './src/Main.elm';
import { io, Socket } from 'socket.io-client';
import './src/ThreeGridScene'; // Registriert die Web Component

// --- KONFIGURATION & FLAGS ---

const savedConfig = localStorage.getItem('p2_agent_config');
const defaultBackend = 'http://localhost:8000';

// Falls noch "5000" im Speicher steht, überschreiben wir es mit dem Standard
let backendIP = localStorage.getItem('p2_backend_ip') || defaultBackend;
if (backendIP.includes(":5000")) {
    backendIP = defaultBackend;
}

const app = Elm.Main.init({
    node: document.getElementById('elm-app'),
    flags: {
        backendIP: backendIP,
        savedConfig: savedConfig
    }
});

let socket: Socket | null = null;

// --- HILFSFUNKTIONEN FÜR STABILITÄT ---

// Elm -> JS: Sicherstellen, dass der Port existiert, bevor wir ihn abonnieren
const subscribeSafe = (portName: string, callback: (data: any) => void) => {
    const port = (app.ports as any)[portName];
    if (port && port.subscribe) {
        port.subscribe(callback);
    } else {
        console.warn(`⚠️ Outgoing Port '${portName}' ist in Elm nicht aktiv.`);
    }
};

// JS -> Elm: Sicherstellen, dass der Port existiert, bevor wir Daten senden
const sendSafe = (portName: string, data: any) => {
    const port = (app.ports as any)[portName];
    if (port && port.send) {
        port.send(data);
    } else {
        console.warn(`📩 Incoming Port '${portName}' ist in Elm nicht aktiv. Daten verworfen.`);
    }
};

// --- OUTGOING PORTS (Elm -> JS) ---

subscribeSafe('connectToBackend', (url: string) => {
    if (socket) {
        socket.disconnect();
    }
    
    console.log("Versuche Verbindung zu:", url);
    localStorage.setItem('p2_backend_ip', url);
    socket = io(url);

    socket.on('connect', () => {
        console.log("✅ Socket.IO verbunden");
        sendSafe('socketStatusReceiver', true);
    });

    socket.on('disconnect', () => {
        console.log("❌ Socket.IO getrennt");
        sendSafe('socketStatusReceiver', false);
        sendSafe('systemLogReceiver', { 
            message: "Verbindung zum Backend verloren.", 
            level: "warning" 
        });
    });

    // --- INCOMING DATA (Socket -> Elm) ---

    socket.on('active_agents', (data) => {
        console.log("🤖 Daten werden an Elm gesendet...");
        
        if (app.ports.activeAgentsReceiver) {
            app.ports.activeAgentsReceiver.send(data);
        } else {
            console.error("❌ Port 'activeAgentsReceiver' wurde in Elm nicht gefunden!");
        }
    });

    socket.on('path_complete', (data: any) => {
        console.log("🛤 Pfad berechnet:", data);
        sendSafe('pathCompleteReceiver', data);
    });

    socket.on('system_log', (data: any) => {
        sendSafe('systemLogReceiver', data);
    });

    socket.on('rfid_scanned', (data: any) => {
        console.log("🎴 RFID Scan:", data);
        sendSafe('rfidReceiver', data);
    });
});

// --- STEUERUNGS-PORTS ---

subscribeSafe('setMode', (mode: string) => {
    if (socket?.connected) {
        socket.emit('set_mode', { mode: mode });
    }
});

subscribeSafe('triggerPlanning', (payload: any) => {
    if (socket?.connected) {
        socket.emit('plan_path', payload);
    } else {
        sendSafe('systemLogReceiver', { 
            message: "Fehler: Nicht mit Backend verbunden!", 
            level: "error" 
        });
    }
});

subscribeSafe('saveToLocalStorage', (jsonString: string) => {
    localStorage.setItem('p2_agent_config', jsonString);
});

// Datei Export
subscribeSafe('exportConfig', (jsonString: string) => {
    const blob = new Blob([jsonString], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'passform_config.json';
    a.click();
    URL.revokeObjectURL(url);
});

// Datei Import
subscribeSafe('importConfigTrigger', () => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.onchange = (e: any) => {
        const file = e.target.files[0];
        const reader = new FileReader();
        reader.onload = (readerEvent) => {
            const content = readerEvent.target?.result;
            if (typeof content === 'string') {
                sendSafe('configReceived', content);
            }
        };
        reader.readAsText(file);
    };
    input.click();
});