import { Elm } from './src/Main.elm';
import { io, Socket } from 'socket.io-client';
import './src/ThreeGridScene'; // Registriert die Web Component

// --- KONFIGURATION & FLAGS ---

const savedConfig = localStorage.getItem('p2_agent_config');
const defaultBackend = 'http://localhost:8000';
const rosBridgeUrl = 'http://localhost:5000';

let backendIP = localStorage.getItem('p2_backend_ip') || defaultBackend;

// Falls noch eine alte IP mit Port 5000 im Speicher ist, auf Standard zurücksetzen
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
let rosSocket: Socket | null = null;

// --- HILFSFUNKTIONEN FÜR STABILITÄT ---

const subscribeSafe = (portName: string, callback: (data: any) => void) => {
    const port = (app.ports as any)[portName];
    if (port && port.subscribe) {
        port.subscribe(callback);
    } else {
        console.warn(`⚠️ Outgoing Port '${portName}' ist in Elm nicht aktiv.`);
    }
};

const sendSafe = (portName: string, data: any) => {
    const port = (app.ports as any)[portName];
    if (port && port.send) {
        port.send(data);
    } else {
        console.error(`📩 Fehler: Incoming Port '${portName}' existiert nicht in Main.elm.`);
    }
};

// --- ROS-BRIDGE VERBINDUNG (Port 5000) ---

const connectToRosBridge = () => {
    if (rosSocket) rosSocket.disconnect();
    
    console.log("🤖 Initialisiere Verbindung zu ROS-Bridge (Hardware)...");
    
    // WICHTIG: WebSocket-Only verhindert den 400 Bad Request Fehler
    rosSocket = io(rosBridgeUrl, {
        transports: ['websocket'],
        upgrade: false,
        reconnection: true,
        reconnectionAttempts: Infinity
    });

    rosSocket.on('connect', () => {
        console.log("✅ ROS-Bridge (5000): WebSocket-Verbindung steht.");
    });

    rosSocket.on('active_agents', (data) => {
        console.log("📥 Hardware-Daten erhalten:", data);
        
        // Wir leiten die Daten an Elm weiter
        if (data && Object.keys(data).length > 0) {
            sendSafe('activeAgentsReceiver', data);
        }
    });

    rosSocket.on('connect_error', (err) => {
        console.error("❌ ROS-Bridge (5000) Fehler:", err.message);
    });

    rosSocket.on('disconnect', (reason) => {
        console.warn("⚠️ ROS-Bridge getrennt:", reason);
    });
};

// Startet die ROS-Verbindung sofort beim Laden
connectToRosBridge();

// --- HAUPT-BACKEND VERBINDUNG (Port 8000) ---

subscribeSafe('connectToBackend', (url: string) => {
    if (socket) socket.disconnect();
    
    console.log("🔗 Verbinde zu Haupt-Backend (Simulation):", url);
    localStorage.setItem('p2_backend_ip', url);
    socket = io(url);

    socket.on('connect', () => {
        console.log("✅ Haupt-Backend (8000) verbunden.");
        sendSafe('socketStatusReceiver', true);
        socket?.emit('get_nfc_status');
    });

    socket.on('active_agents', (data) => {
        console.log("📥 Simulations-Daten erhalten");
        // Falls die Simulation die Agenten in ein 'agents' Feld wickelt, entpacken wir sie
        const agents = data.agents ? data.agents : data;
        sendSafe('activeAgentsReceiver', agents);
    });

    socket.on('path_complete', (data: any) => {
        console.log("🛤 Pfad berechnet:", data);
        sendSafe('pathCompleteReceiver', data);
    });

    socket.on('system_log', (data: any) => {
        sendSafe('systemLogReceiver', data);
    });

    socket.on('rfid_found', (data: string) => {
        console.log("🎴 RFID Scan erkannt:", data);
        sendSafe('rfidReceiver', data);
    });

    socket.on('nfc_status', (data: any) => {
        const statusValue = (data && data.status) ? data.status : data;
        console.log("📡 NFC Hardware Status:", statusValue);
        sendSafe('nfcStatusReceiver', statusValue);
    });

    socket.on('disconnect', () => {
        console.log("❌ Haupt-Backend getrennt.");
        sendSafe('socketStatusReceiver', false);
    });
});

// --- STEUERUNGS-PORTS (Elm -> JS -> Backend) ---

subscribeSafe('socketEmitPort', (payload: [string, any]) => {
    const [eventName, data] = payload;
    
    // Befehle gehen primär an das Simulations-Backend
    if (socket?.connected) {
        socket.emit(eventName, data);
    }
    
    // Spezielle Befehle (wie Herzschlag-Takt) auch an Hardware senden
    if (rosSocket?.connected && eventName === 'set_heartbeat_rate') {
        console.log("⚙️ Sende Heartbeat-Rate an Hardware-Bridge");
        rosSocket.emit(eventName, data);
    }
});

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

subscribeSafe('savePlanningWeights', (weights: any) => {
    if (socket?.connected) {
        console.log("📤 Sende neue Planungs-Gewichte:", weights);
        socket.emit('update_planning_config', weights);
    }
});

subscribeSafe('writeNfcTrigger', (text: string) => {
    if (socket?.connected) {
        console.log("📤 Sende Schreibbefehl für NFC:", text);
        socket.emit('write_nfc', { text: text });
    }
});

// --- LOKALE DATEN-VERWALTUNG ---

subscribeSafe('saveToLocalStorage', (jsonString: string) => {
    localStorage.setItem('p2_agent_config', jsonString);
});

subscribeSafe('exportConfig', (jsonString: string) => {
    const blob = new Blob([jsonString], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = 'passform_config.json'; a.click();
    URL.revokeObjectURL(url);
});

subscribeSafe('importConfigTrigger', () => {
    const input = document.createElement('input');
    input.type = 'file'; input.accept = '.json';
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