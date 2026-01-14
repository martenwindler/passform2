import { Elm } from './src/Main.elm';
import { io } from "socket.io-client";
import './src/ThreeGridScene';

const app = Elm.Main.init({
    node: document.getElementById('elm-app'),
    flags: { backendIP: "localhost" } 
});

let socket = null;

app.ports.connectToBackend.subscribe((ip) => {
    if (socket) socket.disconnect();
    
    socket = io(`http://${ip}:8000`, { transports: ["websocket"] });

    socket.on("connect", () => app.ports.socketStatusReceiver.send(true));
    socket.on("disconnect", () => app.ports.socketStatusReceiver.send(false));
    
    socket.on("active_agents", (data) => {
        app.ports.activeAgentsReceiver.send(data);
    });

    socket.on("path_complete", (data) => {
        app.ports.pathCompleteReceiver.send(data);
    });
});

app.ports.setViewMode.subscribe((is3D) => {
    const scene = document.querySelector('three-grid-scene');
    if (scene) scene.is3D = is3D;
});