import { Elm } from './src/Main.elm';
import { io, Socket } from 'socket.io-client';
import './src/ThreeGridScene';
import "./src/assets/styles/main.scss";

// --- KONFIGURATION & FLAGS ---

const savedConfig = localStorage.getItem('p2_agent_config');
const defaultBackend = 'http://127.0.0.1:8080';

let backendIP = localStorage.getItem('p2_backend_ip') || defaultBackend;

// Bereinigung der IP-Adressen
if (backendIP.includes("localhost")) {
    backendIP = backendIP.replace("localhost", "127.0.0.1");
}
if (backendIP.includes(":8000")) {
    backendIP = backendIP.replace(":8000", ":8080");
}
if (backendIP.includes(":5000") || backendIP.includes("192.168")) {
    backendIP = defaultBackend;
}

const app = Elm.Main.init({
    node: document.getElementById('elm-app'),
    flags: {
        backendIP: backendIP,
        savedConfig: savedConfig
    }
});

// Suche diesen Block in index.ts und ersetze ihn komplett:
app.ports.pushConfig.subscribe((configData: any) => {
    console.log("🔌 JS PORT: pushConfig erhalten. Inhalt:", configData);
    
    if (socket && socket.connected) {
        console.log("📡 SOCKET: Emittiére 'push_config' an das Backend...");
        socket.emit('push_config', configData);
    } else {
        console.error("❌ SOCKET: Nicht verbunden! Senden abgebrochen.");
    }
});

let socket: Socket | null = null;
let rosSocket: Socket | null = null;
let currentBackendUrl = ""; 
let isConnecting = false;

// --- HILFSFUNKTIONEN ---

const subscribeSafe = (portName: string, callback: (data: any) => void) => {
    const port = (app.ports as any)[portName];
    if (port && port.subscribe) port.subscribe(callback);
};

const sendSafe = (portName: string, data: any) => {
    const port = (app.ports as any)[portName];
    if (port && port.send) port.send(data);
};

// --- FILE READER LOGIK (KORRIGIERT FÜR LANDING PAGE) ---

/**
 * Kernfunktion zum Einlesen von JSON-Dateien.
 * Wandelt das File-Objekt in einen String um und schickt ihn an Elm.
 */
const handleFileReading = (file: File) => {
    if (!file) return;
    
    // Validierung (Optional, aber empfohlen)
    if (!file.name.endsWith('.json') && file.type !== "application/json") {
        console.warn("⚠️ Nur .json Dateien sind erlaubt.");
        return;
    }

    const reader = new FileReader();
    reader.onload = (ev) => {
        const content = ev.target?.result;
        if (typeof content === 'string') {
            // Sende an beide möglichen Ports (Abwärtskompatibilität)
            sendSafe('fileContentRead', content);
            sendSafe('configReceived', content);
            console.log("✅ Konfigurationsdatei erfolgreich eingelesen.");
        }
    };
    reader.readAsText(file);
};

/**
 * Port für automatisiertes Einlesen (wird von der Landing-Page Dropzone getriggert)
 */
subscribeSafe('requestFileRead', (data: any) => {
    // Wenn data direkt ein File ist (aus dem Drop-Decoder)
    if (data instanceof File || (data && data.name)) {
        handleFileReading(data);
    } 
    // Falls ein Raw-Event oder ein Proxy-Objekt kommt
    else if (data && data.target && data.target.files) {
        handleFileReading(data.target.files[0]);
    }
});

// --- ROS-BRIDGE VERBINDUNG (Port 5000) ---

const connectToRosBridge = () => {
    if (rosSocket) {
        rosSocket.removeAllListeners();
        rosSocket.disconnect();
    }
    
    rosSocket = io('ws://127.0.0.1:5000', { 
        transports: ['websocket'],
        reconnection: true,
        reconnectionDelay: 2000
    });

    rosSocket.on('connect', () => sendSafe('rosStatusReceiver', true));
    rosSocket.on('connect_error', () => sendSafe('rosStatusReceiver', false));
    rosSocket.on('disconnect', () => sendSafe('rosStatusReceiver', false));
};

connectToRosBridge();

// --- HAUPT-BACKEND VERBINDUNG (Port 8080) ---

const setupMainSocket = (url: string) => {
    const socketUrl = url.replace("localhost", "127.0.0.1");

    if (socketUrl === currentBackendUrl && (isConnecting || socket?.connected)) return;

    if (socket) {
        socket.removeAllListeners();
        socket.disconnect();
    }
    
    currentBackendUrl = socketUrl;
    isConnecting = true;
    
    socket = io(socketUrl, {
        transports: ['polling', 'websocket'],
        reconnection: true,
        path: "/socket.io/",
        forceNew: true 
    });

    socket.on('connect', () => {
        isConnecting = false;
        sendSafe('socketStatusReceiver', true);
    });

    socket.on('active_agents', (data) => sendSafe('activeAgentsReceiver', data.agents || data));
    socket.on('path_complete', (data: any) => sendSafe('pathCompleteReceiver', data));
    socket.on('system_log', (data: any) => sendSafe('systemLogReceiver', data));
    socket.on('nfc_status', (data: any) => sendSafe('nfcStatusReceiver', data.status || data));
    socket.on('hardware_update', (data: any) => sendSafe('hardwareUpdateReceiver', data));

    socket.on('rfid_scanned', (data: any) => {
        sendSafe('rfidReceiver', data.id || data);
    });

    socket.on('connect_error', () => {
        isConnecting = false;
        sendSafe('socketStatusReceiver', false);
    });

    socket.on('disconnect', (reason) => {
        isConnecting = false;
        sendSafe('socketStatusReceiver', false);
        if (reason === "io server disconnect") socket?.connect();
    });
};

setupMainSocket(backendIP);

// --- ELM PORTS ---

subscribeSafe('connectToBackend', (url: string) => {
    const fixedUrl = url.replace(":8000", ":8080");
    localStorage.setItem('p2_backend_ip', fixedUrl);
    setupMainSocket(fixedUrl);
});

subscribeSafe('socketEmitPort', (payload: [string, any]) => {
    const [eventName, data] = payload;
    if (socket?.connected) socket.emit(eventName, data);
});

subscribeSafe('setMode', (mode: string) => {
    if (socket?.connected) socket.emit('set_mode', mode);
});

// Manueller Import-Trigger (Button-Klick)
subscribeSafe('importConfigTrigger', () => {
    const input = document.createElement('input');
    input.type = 'file'; 
    input.accept = '.json';
    input.onchange = (e: any) => {
        const file = e.target.files?.[0];
        handleFileReading(file);
    };
    input.click();
});

// --- STORAGE & EXPORT ---

subscribeSafe('saveToLocalStorage', (json: string) => localStorage.setItem('p2_agent_config', json));

subscribeSafe('exportConfig', (json: string) => {
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; 
    a.download = 'passform_config.json'; 
    a.click();
    URL.revokeObjectURL(url);
});

// Restliche Steuerungs-Ports
subscribeSafe('triggerPlanning', (payload: any) => {
    if (socket?.connected) socket.emit('plan_path', payload);
});

subscribeSafe('sendCnpRequest', (payload: any) => {
    if (socket?.connected) socket.emit('plan_path', payload);
});

subscribeSafe('savePlanningWeights', (weights: any) => {
    if (socket?.connected) socket.emit('update_planning_config', weights);
});

subscribeSafe('writeNfcTrigger', (text: string) => {
    if (socket?.connected) socket.emit('write_nfc', text);
});