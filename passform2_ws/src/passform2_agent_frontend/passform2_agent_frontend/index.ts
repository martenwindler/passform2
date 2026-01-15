import { Elm } from './src/Main.elm';
import { io, Socket } from 'socket.io-client';
import './src/ThreeGridScene'; // Registriert die Web Component

// --- KONFIGURATION & FLAGS ---

// Wir laden die gespeicherte Konfiguration beim Start
const savedConfig = localStorage.getItem('p2_agent_config');
const backendIP = localStorage.getItem('p2_backend_ip') || 'http://localhost:5000';

// Initialisierung der Elm App
const app = Elm.Main.init({
    node: document.getElementById('elm-app'),
    flags: {
        backendIP: backendIP,
        savedConfig: savedConfig
    }
});

let socket: Socket | null = null;

// --- OUTGOING PORTS (Elm -> JS) ---

// Verbindung zum Backend aufbauen
app.ports.connectToBackend.subscribe((url: string) => {
    if (socket) socket.disconnect();
    
    localStorage.setItem('p2_backend_ip', url);
    socket = io(url);

    socket.on('connect', () => {
        app.ports.socketStatusReceiver.send(true);
        app.ports.logReceivedReceiver.send("Verbunden mit Backend: " + url);
    });

    socket.on('disconnect', () => {
        app.ports.socketStatusReceiver.send(false);
        app.ports.logReceivedReceiver.send("Verbindung zum Backend verloren.");
    });

    // Empfange Daten vom Backend
    socket.on('active_agents', (data: any) => {
        app.ports.activeAgentsReceiver.send(data);
    });

    socket.on('path_complete', (data: any) => {
        app.ports.pathCompleteReceiver.send(data);
    });

    socket.on('system_log', (msg: string) => {
        app.ports.logReceivedReceiver.send(msg);
    });
});

// Modus wechseln (Simulation / Hardware)
app.ports.setMode.subscribe((mode: string) => {
    if (socket?.connected) {
        socket.emit('set_mode', { mode: mode });
    }
});

// Pfadplanung triggern
app.ports.triggerPlanning.subscribe((payload: any) => {
    if (socket?.connected) {
        socket.emit('plan_path', payload);
    } else {
        app.ports.logReceivedReceiver.send("Fehler: Nicht mit Backend verbunden!");
    }
});

// LocalStorage Persistenz
app.ports.saveToLocalStorage.subscribe((jsonString: string) => {
    localStorage.setItem('p2_agent_config', jsonString);
});

// Datei Export (Download)
app.ports.exportConfig.subscribe((jsonString: string) => {
    const blob = new Blob([jsonString], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'passform_config.json';
    a.click();
    URL.revokeObjectURL(url);
});

// Datei Import (Upload)
app.ports.importConfigTrigger.subscribe(() => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.onchange = (e: any) => {
        const file = e.target.files[0];
        const reader = new FileReader();
        reader.onload = (readerEvent) => {
            const content = readerEvent.target?.result;
            if (typeof content === 'string') {
                app.ports.configReceived.send(content);
            }
        };
        reader.readAsText(file);
    };
    input.click();
});