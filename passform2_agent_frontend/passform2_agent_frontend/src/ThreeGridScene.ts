import * as THREE from 'three';

export class ThreeGridScene extends HTMLElement {
    private renderer: THREE.WebGLRenderer;
    private camera: THREE.PerspectiveCamera;
    private scene: THREE.Scene;
    private raycaster: THREE.Raycaster;
    private gridHelper: THREE.GridHelper | null = null;
    
    // Raster aus einzelnen Kacheln für die Boden-Einfärbung
    private gridCells: THREE.Mesh[][] = [];
    private agentMeshes: Map<string, THREE.Object3D> = new Map();
    private pathLine: THREE.Line | null = null;
    
    // Farben & Marker-Status
    private readonly defaultGroundColor = 0x2d3748;
    private readonly startColor = 0x48bb78; // Grün
    private readonly goalColor = 0x3182ce;  // Blau
    private startPos: {x: number, y: number} | null = null;
    private goalPos: {x: number, y: number} | null = null;

    // Kamera & Interaktions-Parameter
    private targetCameraPos = new THREE.Vector3(16, 25, 16);
    private lerpSpeed = 0.08;
    private isDragging = false;
    private isMouseDown = false;
    private draggedAgentKey: string | null = null;
    private mouseDownPos = { x: 0, y: 0 };
    private dragThreshold = 5; 

    constructor() {
        super();
        this.attachShadow({ mode: 'open' });
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a202c);
        
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
        this.scene.add(ambientLight);
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
        dirLight.position.set(10, 50, 10);
        this.scene.add(dirLight);

        this.camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.raycaster = new THREE.Raycaster();
    }

    static get observedAttributes() {
        return ['agents', 'is-3d', 'grid-width', 'grid-height', 'path', 'start-pos', 'goal-pos'];
    }

    attributeChangedCallback(name: string, oldVal: string, newVal: string) {
        if (oldVal === newVal) return;
        try {
            switch (name) {
                case 'grid-width':
                case 'grid-height': this.updateGrid(); break;
                case 'is-3d': this.toggleCamera(newVal === 'true'); break;
                case 'agents': if (!this.isDragging) this.updateAgents(JSON.parse(newVal)); break;
                case 'path': this.drawPath(JSON.parse(newVal)); break;
                case 'start-pos':
                case 'goal-pos': this.updateMarkers(); break;
            }
        } catch (e) { console.error(`Attribute update error (${name}):`, e); }
    }

    connectedCallback() {
        this.shadowRoot?.appendChild(this.renderer.domElement);
        this.resize();
        window.addEventListener('resize', () => this.resize());
        this.renderer.domElement.addEventListener('mousedown', (e) => this.onMouseDown(e));
        window.addEventListener('mousemove', (e) => this.onMouseMove(e));
        window.addEventListener('mouseup', (e) => this.onMouseUp(e));
        this.animate();
        this.updateGrid();
    }

    // Erstellt das Gitter aus Einzelkacheln
    private updateGrid() {
        const w = Math.max(1, parseInt(this.getAttribute('grid-width') || '10'));
        const h = Math.max(1, parseInt(this.getAttribute('grid-height') || '10'));

        // Alte Kacheln löschen
        this.gridCells.flat().forEach(cell => {
            this.scene.remove(cell);
            cell.geometry.dispose();
            (cell.material as THREE.Material).dispose();
        });
        this.gridCells = [];

        const geometry = new THREE.PlaneGeometry(1, 1);
        geometry.rotateX(-Math.PI / 2);

        for (let x = 0; x < w; x++) {
            this.gridCells[x] = [];
            for (let y = 0; y < h; y++) {
                const material = new THREE.MeshPhongMaterial({ color: this.defaultGroundColor });
                const cell = new THREE.Mesh(geometry, material);
                cell.position.set(x + 0.5, 0, y + 0.5);
                cell.userData = { gridX: x, gridY: y };
                this.scene.add(cell);
                this.gridCells[x][y] = cell;
            }
        }

        if (this.gridHelper) this.scene.remove(this.gridHelper);
        this.gridHelper = new THREE.GridHelper(Math.max(w, h), Math.max(w, h), 0x4a5568, 0x2d3748);
        this.gridHelper.position.set(w / 2, 0.01, h / 2);
        this.scene.add(this.gridHelper);

        this.updateCameraFocus(w, h);
        this.updateMarkers();
    }

    // Färbt die Kacheln für Start und Ziel ein
    private updateMarkers() {
        if (this.startPos) this.colorizeCell(this.startPos.x, this.startPos.y, this.defaultGroundColor);
        if (this.goalPos) this.colorizeCell(this.goalPos.x, this.goalPos.y, this.defaultGroundColor);

        try {
            this.startPos = JSON.parse(this.getAttribute('start-pos') || 'null');
            this.goalPos = JSON.parse(this.getAttribute('goal-pos') || 'null');
        } catch (e) { this.startPos = null; this.goalPos = null; }

        if (this.startPos) this.colorizeCell(this.startPos.x, this.startPos.y, this.startColor);
        if (this.goalPos) this.colorizeCell(this.goalPos.x, this.goalPos.y, this.goalColor);
    }

    private colorizeCell(x: number, y: number, color: number) {
        if (this.gridCells[x] && this.gridCells[x][y]) {
            (this.gridCells[x][y].material as THREE.MeshPhongMaterial).color.setHex(color);
        }
    }

    // Aktualisiert die Agenten-Meshes und deren Rotation
    private updateAgents(agents: any[]) {
        // Wir nutzen jetzt die agent_id als Key für eine flüssige Bewegung
        const currentIds = new Set(agents.map(a => a.agent_id));
        const toRemove = new Set([...this.agentMeshes.keys()].filter(id => !currentIds.has(id)));

        agents.forEach(agent => {
            const id = agent.agent_id;
            let mesh = this.agentMeshes.get(id);

            if (!mesh) {
                mesh = this.createAgentMesh(agent.module_type, agent.is_dynamic);
                this.scene.add(mesh);
                this.agentMeshes.set(id, mesh);
            }

            // Zielposition setzen
            const targetX = agent.position.x + 0.5;
            const targetZ = agent.position.y + 0.5;

            // Einfaches "Sliding" für dynamische Agenten (FTF)
            if (agent.is_dynamic) {
                mesh.position.lerp(new THREE.Vector3(targetX, 0.1, targetZ), 0.1);
            } else {
                mesh.position.set(targetX, 0.2, targetZ);
            }
            
            mesh.rotation.y = (agent.orientation || 0) * (Math.PI / 180);
            
            // Visualisierung der Beladung (Payload)
            if (agent.payload) {
                mesh.scale.set(1.1, 1.2, 1.1); // FTF "plustert" sich auf, wenn es trägt
            } else {
                mesh.scale.set(1, 1, 1);
            }
        });

        // Verschwundene Agenten löschen
        toRemove.forEach(id => {
            const mesh = this.agentMeshes.get(id);
            if (mesh) { this.scene.remove(mesh); this.agentMeshes.delete(id); }
        });
    }

    private createAgentMesh(type: string, isDynamic: boolean): THREE.Object3D {
        if (isDynamic || type === 'ftf') {
            // Das FTF als flache, gelbe "Flunder"
            const group = new THREE.Group();
            const bodyGeo = new THREE.BoxGeometry(0.7, 0.1, 0.8);
            const bodyMat = new THREE.MeshPhongMaterial({ color: 0xffd700, emissive: 0x443300 });
            const body = new THREE.Mesh(bodyGeo, bodyMat);
            
            // Kleine "Augen" oder Sensoren für die Fahrtrichtung
            const eyeGeo = new THREE.BoxGeometry(0.1, 0.05, 0.1);
            const eyeMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
            const eye = new THREE.Mesh(eyeGeo, eyeMat);
            eye.position.set(0, 0.05, 0.35);
            
            group.add(body);
            group.add(eye);
            return group;
        }

        // Statische Module (unverändert, aber optimiert)
        let geo, col;
        if (type.includes('rollen')) { geo = new THREE.BoxGeometry(0.85, 0.15, 0.85); col = 0x3182ce; }
        else if (type === 'greifer') { geo = new THREE.CylinderGeometry(0.35, 0.35, 0.6, 16); col = 0xed8936; }
        else if (type === 'conveyeur') { geo = new THREE.BoxGeometry(0.9, 0.1, 0.9); col = 0x38a169; }
        else { geo = new THREE.BoxGeometry(0.95, 0.3, 0.95); col = 0x718096; }
        
        return new THREE.Mesh(geo, new THREE.MeshPhongMaterial({ color: col }));
    }

    // Pfad-Zeichnung: Jetzt mit Leucht-Effekt für die Mission
    private drawPath(pathNodes: any[]) {
        if (this.pathLine) { this.scene.remove(this.pathLine); this.pathLine.geometry.dispose(); }
        if (!pathNodes || pathNodes.length < 2) return;

        const points = pathNodes.map(n => {
            // Wir nutzen n.position (für Sim) oder n direkt (Fallback)
            const pos = n.position || n;
            return new THREE.Vector3(pos.x + 0.5, 0.12, pos.y + 0.5);
        });

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ 
            color: 0x00f2ff, // Neon-Blau für Missionen
            linewidth: 3,
            transparent: true,
            opacity: 0.8
        });

        this.pathLine = new THREE.Line(geometry, material);
        this.scene.add(this.pathLine);
    }

    // --- INTERAKTIONSLOGIK ---

    private onMouseDown(event: MouseEvent) {
        this.isMouseDown = true;
        this.mouseDownPos = { x: event.clientX, y: event.clientY };
    }

    private onMouseMove(event: MouseEvent) {
        if (!this.isMouseDown) return;
        const delta = Math.hypot(event.clientX - this.mouseDownPos.x, event.clientY - this.mouseDownPos.y);

        if (!this.isDragging && delta > this.dragThreshold) {
            const coords = this.getMouseCoords(event);
            this.raycaster.setFromCamera(coords, this.camera);
            const intersects = this.raycaster.intersectObjects(Array.from(this.agentMeshes.values()));
            if (intersects.length > 0) {
                this.isDragging = true;
                const object = intersects[0].object;
                this.draggedAgentKey = [...this.agentMeshes.entries()].find(([_, v]) => v === object)?.[0] || null;
                if (this.draggedAgentKey) (this.agentMeshes.get(this.draggedAgentKey) as THREE.Mesh).material.opacity = 0.6;
            }
        }

        if (this.isDragging && this.draggedAgentKey) {
            const coords = this.getMouseCoords(event);
            this.raycaster.setFromCamera(coords, this.camera);
            const ground = this.raycaster.intersectObjects(this.gridCells.flat());
            if (ground.length > 0) {
                const mesh = this.agentMeshes.get(this.draggedAgentKey);
                const hit = ground[0].object;
                if (mesh) {
                    mesh.position.x = hit.userData.gridX + 0.5;
                    mesh.position.z = hit.userData.gridY + 0.5;
                    mesh.position.y = 0.5;
                }
            }
        }
    }

    private onMouseUp(event: MouseEvent) {
        const delta = Math.hypot(event.clientX - this.mouseDownPos.x, event.clientY - this.mouseDownPos.y);
        if (!this.isDragging && delta <= this.dragThreshold) {
            const coords = this.getMouseCoords(event);
            this.raycaster.setFromCamera(coords, this.camera);
            const ground = this.raycaster.intersectObjects(this.gridCells.flat());
            if (ground.length > 0) {
                this.dispatchEvent(new CustomEvent('cell-clicked', {
                    detail: { x: ground[0].object.userData.gridX, y: ground[0].object.userData.gridY },
                    bubbles: true, composed: true
                }));
            }
        } else if (this.isDragging && this.draggedAgentKey) {
            const mesh = this.agentMeshes.get(this.draggedAgentKey);
            if (mesh) {
                const oldPos = this.draggedAgentKey.split('_').map(Number);
                const newX = Math.floor(mesh.position.x), newY = Math.floor(mesh.position.z);
                (mesh as THREE.Mesh).material.opacity = 1.0;
                mesh.position.y = 0.2;
                this.dispatchEvent(new CustomEvent('agent-moved', {
                    detail: { oldX: oldPos[0], oldY: oldPos[1], newX, newY },
                    bubbles: true, composed: true
                }));
            }
        }
        this.isMouseDown = this.isDragging = false;
        this.draggedAgentKey = null;
    }

    private updateCameraFocus(w: number, h: number) {
        const is3D = this.getAttribute('is-3d') === 'true';
        const dist = Math.max(w, h);
        if (is3D) { this.targetCameraPos.set(dist * 1.2, dist * 0.8, dist * 1.2); this.camera.up.set(0, 1, 0); }
        else { this.targetCameraPos.set(w / 2, dist * 1.1, h / 2); this.camera.up.set(0, 0, -1); }
    }

    private toggleCamera(is3D: boolean) {
        this.updateCameraFocus(parseInt(this.getAttribute('grid-width') || '10'), parseInt(this.getAttribute('grid-height') || '10'));
    }

    private getMouseCoords(e: MouseEvent) {
        const r = this.renderer.domElement.getBoundingClientRect();
        return { x: ((e.clientX - r.left) / r.width) * 2 - 1, y: -((e.clientY - r.top) / r.height) * 2 + 1 };
    }

    private resize() {
        const p = this.parentElement;
        if (p) { this.renderer.setSize(p.clientWidth, p.clientHeight); this.camera.aspect = p.clientWidth / p.clientHeight; this.camera.updateProjectionMatrix(); }
    }

    private animate() {
        requestAnimationFrame(() => this.animate());
        this.camera.position.lerp(this.targetCameraPos, this.lerpSpeed);
        const w = parseInt(this.getAttribute('grid-width') || '10'), h = parseInt(this.getAttribute('grid-height') || '10');
        this.camera.lookAt(w / 2, 0, h / 2);
        this.renderer.render(this.scene, this.camera);
    }
}
customElements.define('three-grid-scene', ThreeGridScene);