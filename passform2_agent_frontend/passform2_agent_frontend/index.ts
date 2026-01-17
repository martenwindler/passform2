import { Elm } from './src/Main.elm';
import { io, Socket } from 'socket.io-client';
import './src/ThreeGridScene';

// --- KONFIGURATION & FLAGS ---

const savedConfig = localStorage.getItem('p2_agent_config');
const defaultBackend = 'http://127.0.0.1:8080';
const rosBridgeUrl = 'http://127.0.0.1:5000';

let backendIP = localStorage.getItem('p2_backend_ip') || defaultBackend;

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

// --- ROS-BRIDGE VERBINDUNG (Port 5000) ---

const connectToRosBridge = () => {
    if (rosSocket) {
        rosSocket.removeAllListeners();
        rosSocket.disconnect();
    }
    
    console.log("🤖 Initialisiere Verbindung zu ROS-Bridge (Hardware)...");
    
    rosSocket = io(rosBridgeUrl, {
        transports: ['websocket'],
        reconnection: true,
        reconnectionDelay: 1000,
        timeout: 5000
    });

    rosSocket.on('connect', () => {
        console.log("✅ ROS-Bridge (5000) Online");
        sendSafe('rosStatusReceiver', true);
    });

    rosSocket.on('active_agents', (data) => {
        if (data) sendSafe('activeAgentsReceiver', data);
    });

    rosSocket.on('connect_error', (err) => {
        console.warn("❌ ROS-Bridge (5000) Fehler:", err.message);
        sendSafe('rosStatusReceiver', false);
    });

    // NEU: Disconnect Handler für ROS
    rosSocket.on('disconnect', (reason) => {
        console.warn("⚠️ ROS-Bridge getrennt:", reason);
        sendSafe('rosStatusReceiver', false);
    });
};

connectToRosBridge();

// --- HAUPT-BACKEND VERBINDUNG (Port 8080) ---

const setupMainSocket = (url: string) => {
    const socketUrl = url.replace("localhost", "127.0.0.1");

    if (socketUrl === currentBackendUrl && (isConnecting || socket?.connected)) {
        return;
    }

    if (socket) {
        console.log("🔄 Trenne bestehenden Socket für Neuverbindung...");
        socket.removeAllListeners();
        socket.disconnect();
        isConnecting = false;
    }
    
    currentBackendUrl = socketUrl;
    isConnecting = true;
    console.log("🔗 Verbindungsaufbau Haupt-Backend:", socketUrl);
    
    socket = io(socketUrl, {
        transports: ['websocket'], 
        upgrade: false,
        reconnection: true,
        reconnectionDelay: 2000, 
        timeout: 10000,
        path: "/socket.io/",
        forceNew: true 
    });

    socket.on('connect', () => {
        isConnecting = false;
        console.log("✅ Haupt-Backend (8080) Online via WebSocket");
        sendSafe('socketStatusReceiver', true);
    });

    socket.on('hardware_update', (data: any) => sendSafe('hardwareUpdateReceiver', data));

    socket.on('rfid_scanned', (data: any) => {
        sendSafe('rfidReceiver', data.id || data);
        sendSafe('systemLogReceiver', { 
            message: `RFID Scan [${data.pi_id || 'Pi'}]: ${data.id}`, 
            level: "info" 
        });
    });

    socket.on('active_agents', (data) => {
        const agents = data.agents ? data.agents : data;
        sendSafe('activeAgentsReceiver', agents);
    });

    socket.on('path_complete', (data: any) => sendSafe('pathCompleteReceiver', data));
    socket.on('system_log', (data: any) => sendSafe('systemLogReceiver', data));
    socket.on('nfc_status', (data: any) => sendSafe('nfcStatusReceiver', data.status || data));

    socket.on('connect_error', (err) => {
        isConnecting = false;
        console.error("❌ Haupt-Backend Fehler (8080):", err.message);
        sendSafe('socketStatusReceiver', false);
    });

    // VERBESSERT: Disconnect Handler mit Reason-Logging
    socket.on('disconnect', (reason) => {
        isConnecting = false;
        console.warn("❌ Haupt-Backend Verbindung verloren. Grund:", reason);
        sendSafe('socketStatusReceiver', false);
        
        // Automatischer Reconnect-Versuch bei serverseitigem Abbruch
        if (reason === "io server disconnect") {
            socket?.connect();
        }
    });
};

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

subscribeSafe('triggerPlanning', (payload: any) => {
    if (socket?.connected) socket.emit('plan_path', payload);
});

subscribeSafe('savePlanningWeights', (weights: any) => {
    if (socket?.connected) socket.emit('update_planning_config', weights);
});

subscribeSafe('writeNfcTrigger', (text: string) => {
    if (socket?.connected) socket.emit('write_nfc', text);
});

// --- STORAGE & EXPORT ---

subscribeSafe('saveToLocalStorage', (json: string) => localStorage.setItem('p2_agent_config', json));

subscribeSafe('exportConfig', (json: string) => {
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = 'passform_config.json'; a.click();
    URL.revokeObjectURL(url);
});

subscribeSafe('importConfigTrigger', () => {
    const input = document.createElement('input');
    input.type = 'file'; input.accept = '.json';
    input.onchange = (e: any) => {
        const file = (e.target as HTMLInputElement).files?.[0];
        if (!file) return;
        const reader = new FileReader();
        reader.onload = (ev) => {
            const content = ev.target?.result;
            if (typeof content === 'string') sendSafe('configReceived', content);
        };
        reader.readAsText(file);
    };
    input.click();
});