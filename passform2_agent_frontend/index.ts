import { Elm } from './src/Main.elm';
import { io, Socket } from 'socket.io-client';
import './src/ThreeGridScene';
import "./src/assets/styles/main.scss";

// --- KONFIGURATION & FLAGS ---

const savedConfig = localStorage.getItem('p2_agent_config');
const defaultBackend = 'http://127.0.0.1:8080';

let backendIP = localStorage.getItem('p2_backend_ip') || defaultBackend;

// Strikte Bereinigung der IP-Adressen für die Kommunikation
const sanitizeUrl = (url: string) : string => {
    let clean = url.replace("localhost", "127.0.0.1");
    if (clean.includes(":8000")) clean = clean.replace(":8000", ":8080");
    if (!clean.startsWith("http")) clean = "http://" + clean;
    return clean;
};

backendIP = sanitizeUrl(backendIP);

const app = Elm.Main.init({
    node: document.getElementById('elm-app'),
    flags: {
        backendIP: backendIP,
        savedConfig: savedConfig
    }
});

// --- HILFSFUNKTIONEN FÜR PORTS ---

const subscribeSafe = (portName: string, callback: (data: any) => void) => {
    const port = (app.ports as any)[portName];
    if (port && port.subscribe) port.subscribe(callback);
};

const sendSafe = (portName: string, data: any) => {
    const port = (app.ports as any)[portName];
    if (port && port.send) {
        port.send(data);
    } else {
        console.warn(`⚠️ Port '${portName}' nicht in Elm gefunden oder nicht als Port definiert.`);
    }
};

// --- INITIAL DATA FETCH (REST API) ---

/**
 * Holt statische/initiale Daten vom Backend via HTTP
 */
/**
 * Holt statische/initiale Daten vom Backend via HTTP
 */
const fetchInitialData = async (url: string) => {
    const baseUrl = url.endsWith('/') ? url.slice(0, -1) : url;
    console.log("📥 JS: Starte Initial-Fetch von API Endpunkten auf", baseUrl);

    try {
        // 1. Buchten (Bays) laden
        const baysResponse = await fetch(`${baseUrl}/api/bays`);
        if (baysResponse.ok) {
            const bays = await baysResponse.json();
            console.log("📡 API: Bays empfangen:", bays);
            sendSafe('initialBaysReceiver', bays);
        }

        // 2. Agenten laden (SSoT)
        const agentsResponse = await fetch(`${baseUrl}/api/agents`);
        if (agentsResponse.ok) {
            const agents = await agentsResponse.json();
            console.log("📡 API: Agents empfangen:", agents);
            sendSafe('activeAgentsReceiver', agents);
        }

        // 3. Inventar laden
        const invResponse = await fetch(`${baseUrl}/api/inventory`);
        if (invResponse.ok) {
            const inventory = await invResponse.json();
            sendSafe('inventoryReceiver', inventory);
        }

        console.log("✅ JS: Alle Initial-Daten erfolgreich synchronisiert.");
    } catch (err) {
        console.error("❌ JS: Fehler beim Initial-Fetch:", err);
    } // <-- Hier muss der catch-Block sauber enden
}; // <-- Und hier endet die Pfeil-Funktion

// --- SOCKET.IO LOGIK ---

let socket: Socket | null = null;
let rosSocket: Socket | null = null;
let currentBackendUrl = ""; 
let isConnecting = false;

const setupMainSocket = (url: string) => {
    const socketUrl = sanitizeUrl(url);

    if (socketUrl === currentBackendUrl && (isConnecting || socket?.connected)) return;

    if (socket) {
        socket.removeAllListeners();
        socket.disconnect();
    }
    
    currentBackendUrl = socketUrl;
    isConnecting = true;
    
    console.log(`🔌 SOCKET: Verbindungsversuch zu ${socketUrl}...`);
    
    socket = io(socketUrl, {
        transports: ['websocket', 'polling'], // WebSocket bevorzugen
        reconnection: true,
        reconnectionAttempts: 5,
        path: "/socket.io/",
        forceNew: true,
        // Diese Option ist wichtig, wenn du Credentials (Cookies/Auth) nutzt, 
        // passt aber vor allem zum tower-http CORS Setup im Backend.
        withCredentials: true 
    });

    socket.on('connect', () => {
        console.log("✅ SOCKET: Verbunden mit Haupt-Backend");
        isConnecting = false;
        sendSafe('socketStatusReceiver', true);
        
        // Synchronisiere Daten erneut bei Reconnect
        fetchInitialData(socketUrl);
    });

    // --- LISTENERS ---

    socket.on('active_agents', (data) => sendSafe('activeAgentsReceiver', data.agents || data));
    socket.on('bay_update', (data) => {
        console.log("📍 SOCKET: Bay Update erhalten:", data);
        sendSafe('bayUpdateReceiver', data);
    });
    socket.on('inventory_update', (data) => sendSafe('inventoryReceiver', data.items || data));
    socket.on('world_state', (data) => sendSafe('worldStateReceiver', data));
    socket.on('hardware_update', (data) => sendSafe('hardwareUpdateReceiver', data));
    socket.on('system_log', (data) => sendSafe('systemLogReceiver', data));
    socket.on('path_complete', (data) => sendSafe('pathCompleteReceiver', data));
    socket.on('nfc_status', (data) => sendSafe('nfcStatusReceiver', data.status || data));
    socket.on('rfid_scanned', (data) => sendSafe('rfidReceiver', data.id || data));

    socket.on('connect_error', (error) => {
        isConnecting = false;
        console.error("❌ SOCKET: Verbindungsfehler:", error);
        sendSafe('socketStatusReceiver', false);
    });

    socket.on('disconnect', (reason) => {
        isConnecting = false;
        console.warn("🔌 SOCKET: Getrennt. Grund:", reason);
        sendSafe('socketStatusReceiver', false);
        if (reason === "io server disconnect") socket?.connect();
    });
};

// --- INITIALISIERUNG ---

// 1. Socket starten
setupMainSocket(backendIP);

// 2. REST Daten sofort laden (nicht nur auf Socket-Connect warten)
fetchInitialData(backendIP);

// --- FILE READER LOGIK ---

const handleFileReading = (file: File) => {
    if (!file) return;
    if (!file.name.endsWith('.json') && file.type !== "application/json") {
        console.warn("⚠️ Nur .json Dateien sind erlaubt.");
        return;
    }

    const reader = new FileReader();
    reader.onload = (ev) => {
        const content = ev.target?.result;
        if (typeof content === 'string') {
            sendSafe('fileContentRead', content);
            sendSafe('configReceived', content);
            console.log("✅ Konfigurationsdatei erfolgreich eingelesen.");
        }
    };
    reader.readAsText(file);
};

// --- ELM SUBSCRIPTIONS (JS -> Backend) ---

subscribeSafe('connectToBackend', (url: string) => {
    const fixedUrl = sanitizeUrl(url);
    localStorage.setItem('p2_backend_ip', fixedUrl);
    setupMainSocket(fixedUrl);
});

subscribeSafe('pushConfig', (fullConfig: any) => {
    console.log("📤 JS: Elm bittet um SSoT-Speicherung...", fullConfig);
    
    if (socket && socket.connected) {
        // Wir senden das gesamte Objekt an Rust
        socket.emit('push_config', fullConfig);
        console.log("🚀 JS: Daten erfolgreich an Socket 'push_config' übergeben.");
    } else {
        console.error("❌ JS: Speichern fehlgeschlagen - Socket ist nicht verbunden!");
        // Optional: Elm über den Fehler informieren
        sendSafe('systemLogReceiver', {
            message: "Verbindungsfehler: Konfiguration konnte nicht gespeichert werden.",
            level: "error"
        });
    }
});

subscribeSafe('socketEmitPort', (payload: [string, any]) => {
    const [eventName, data] = payload;
    if (socket?.connected) socket.emit(eventName, data);
});

subscribeSafe('setMode', (mode: string) => {
    if (socket?.connected) socket.emit('set_mode', mode);
});

subscribeSafe('triggerPlanning', (payload: any) => {
    if (socket?.connected) socket.emit('plan_path', payload);
});

subscribeSafe('writeNfcTrigger', (text: string) => {
    if (socket?.connected) socket.emit('write_nfc', text);
});

subscribeSafe('requestFileRead', (data: any) => {
    if (data instanceof File || (data && data.name)) {
        handleFileReading(data);
    } else if (data && data.target && data.target.files) {
        handleFileReading(data.target.files[0]);
    }
});

subscribeSafe('importConfigTrigger', () => {
    const input = document.createElement('input');
    input.type = 'file'; 
    input.accept = '.json';
    input.onchange = (e: any) => handleFileReading((e.target as HTMLInputElement).files?.[0] as File);
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

// --- ROS-BRIDGE (Port 5000 Support) ---

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