import * as THREE from 'three';

export class ThreeGridScene extends HTMLElement {
    private renderer: THREE.WebGLRenderer;
    private camera: THREE.PerspectiveCamera;
    private scene: THREE.Scene;
    
    private helperScene: THREE.Scene;
    private helperCamera: THREE.PerspectiveCamera;
    private resizeObserver: ResizeObserver | null = null;

    private raycaster: THREE.Raycaster;
    private gridHelper: THREE.GridHelper | null = null;
    
    private gridCells: THREE.Mesh[][] = [];
    private agentMeshes: Map<string, THREE.Object3D> = new Map();
    private pathLine: THREE.Line | null = null;
    
    private readonly defaultGroundColor = 0x2d3748;
    private readonly startColor = 0x48bb78; 
    private readonly goalColor = 0x3182ce;  
    private startPos: {x: number, y: number} | null = null;
    private goalPos: {x: number, y: number} | null = null;

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
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.raycaster = new THREE.Raycaster();

        this.helperScene = new THREE.Scene();
        this.helperCamera = new THREE.PerspectiveCamera(75, 1, 0.1, 100);
        this.helperCamera.position.set(0, 0, 4); 

        const axes = new THREE.AxesHelper(2);
        (axes.material as THREE.Material).depthTest = false;
        axes.renderOrder = 1000;
        this.helperScene.add(axes);
    }

    static get observedAttributes() {
        return ['agents', 'is-3d', 'grid-width', 'grid-height', 'path', 'start-pos', 'goal-pos', 'allow-interaction'];
    }

    attributeChangedCallback(name: string, oldVal: string, newVal: string) {
        // DEBUG LOG: Jetzt sehen wir jede Änderung von Elm!
        console.log(`🔔 WebComponent: Attribut '${name}' geändert`, { oldVal: oldVal?.substring(0,20), newVal: newVal?.substring(0,20) });

        if (oldVal === newVal) return;
        try {
            switch (name) {
                case 'grid-width':
                case 'grid-height': this.updateGrid(); break;
                case 'is-3d': this.toggleCamera(newVal === 'true'); break;
                case 'agents': 
                    if (!this.isDragging && newVal) {
                        const parsed = JSON.parse(newVal);
                        // Falls Elm {"agents": [...]} schickt, extrahieren wir die Liste
                        const list = Array.isArray(parsed) ? parsed : (parsed.agents || []);
                        this.updateAgents(list);
                    }
                    break;
                case 'path': if(newVal) this.drawPath(JSON.parse(newVal)); break;
                case 'start-pos':
                case 'goal-pos': this.updateMarkers(); break;
                case 'allow-interaction': 
                    this.style.cursor = newVal === 'true' ? 'crosshair' : 'default'; 
                    break;
            }
        } catch (e) { console.error(`❌ Fehler beim Attribut-Update (${name}):`, e); }
    }

    connectedCallback() {
        const style = document.createElement('style');
        style.textContent = `
            canvas { display: block; width: 100%; height: 100%; outline: none; } 
            :host { display: block; width: 100%; height: 100%; overflow: hidden; }
        `;
        this.shadowRoot?.appendChild(style);
        this.shadowRoot?.appendChild(this.renderer.domElement);

        this.style.pointerEvents = 'auto';
        this.renderer.domElement.style.pointerEvents = 'auto';

        this.resizeObserver = new ResizeObserver(() => this.resize());
        this.resizeObserver.observe(this);

        this.renderer.domElement.addEventListener('mousedown', (e) => this.onMouseDown(e));
        this.renderer.domElement.addEventListener('mousemove', (e) => this.onMouseMove(e));
        this.renderer.domElement.addEventListener('mouseup', (e) => this.onMouseUp(e));
        
        this.animate();
        this.updateGrid();

        // INITIALER CHECK: Falls Elm die Daten schon gesetzt hat, bevor wir "connected" waren
        const initialAgents = this.getAttribute('agents');
        if (initialAgents) {
            console.log("🚀 Initialer Daten-Kickstart beim Connect!");
            this.attributeChangedCallback('agents', "", initialAgents);
        }
    }

    disconnectedCallback() {
        if (this.resizeObserver) this.resizeObserver.disconnect();
    }

    private resize() {
        const width = this.clientWidth;
        const height = this.clientHeight;
        if (width === 0 || height === 0) return;
        this.renderer.setSize(width, height, false);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
    }

    private isInteractionAllowed(): boolean {
        return this.getAttribute('allow-interaction') === 'true';
    }

    private onMouseDown(event: MouseEvent) {
        if (!this.isInteractionAllowed()) return;
        this.isMouseDown = true;
        this.mouseDownPos = { x: event.clientX, y: event.clientY };
    }

    private onMouseMove(event: MouseEvent) {
        if (!this.isInteractionAllowed()) return;
        const coords = this.getMouseCoords(event);
        this.raycaster.setFromCamera(coords, this.camera);
        const groundHits = this.raycaster.intersectObjects(this.gridCells.flat());
        if (!this.isDragging) this.style.cursor = groundHits.length > 0 ? 'crosshair' : 'default';

        if (!this.isMouseDown) return;
        const delta = Math.hypot(event.clientX - this.mouseDownPos.x, event.clientY - this.mouseDownPos.y);

        if (!this.isDragging && delta > this.dragThreshold) {
            const intersects = this.raycaster.intersectObjects(Array.from(this.agentMeshes.values()), true);
            if (intersects.length > 0) {
                this.isDragging = true;
                let target = intersects[0].object;
                while (target.parent && target.parent !== this.scene) target = target.parent;
                this.draggedAgentKey = [...this.agentMeshes.entries()].find(([_, v]) => v === target)?.[0] || null;
            }
        }

        if (this.isDragging && this.draggedAgentKey) {
            const ground = this.raycaster.intersectObjects(this.gridCells.flat());
            if (ground.length > 0) {
                const mesh = this.agentMeshes.get(this.draggedAgentKey);
                if (mesh) { 
                    mesh.position.x = ground[0].object.userData.gridX + 0.5; 
                    mesh.position.z = ground[0].object.userData.gridY + 0.5; 
                    mesh.position.y = 0.5; 
                }
            }
        }
    }

    private onMouseUp(event: MouseEvent) {
        console.log("🖱️ JS: MouseUp Event ausgelöst. isDragging:", this.isDragging);

        if (this.isDragging && this.draggedAgentKey) {
            const mesh = this.agentMeshes.get(this.draggedAgentKey);
            if (mesh) {
                // 1. Grid-Position berechnen
                const newX = Math.floor(mesh.position.x);
                const newY = Math.floor(mesh.position.z);
                
                console.log(`🎯 JS: Drag beendet. Agent: ${this.draggedAgentKey} -> neue Position: (${newX}, ${newY})`);

                // 2. Event an Elm feuern
                // WICHTIG: 'agent-moved' muss in Elm in Ports.elm/Main.elm als Sub oder Event-Listener gefangen werden
                const moveEvent = new CustomEvent('agent-moved', {
                    detail: { 
                        agentId: this.draggedAgentKey, 
                        newX: newX, 
                        newY: newY 
                    },
                    bubbles: true, 
                    composed: true
                });
                
                console.log("🚀 JS -> ELM: CustomEvent 'agent-moved' wird abgeschickt...");
                this.dispatchEvent(moveEvent);

                // 3. Optisches Einrasten (Snap-to-Grid)
                mesh.position.set(newX + 0.5, 0.1, newY + 0.5);
            }
        } else if (this.isMouseDown) {
            // Normale Klick-Logik für Zellen (Modal öffnen etc.)
            const coords = this.getMouseCoords(event);
            this.raycaster.setFromCamera(coords, this.camera);
            const groundHits = this.raycaster.intersectObjects(this.gridCells.flat());
            
            if (groundHits.length > 0) {
                const gridX = groundHits[0].object.userData.gridX;
                const gridY = groundHits[0].object.userData.gridY;
                
                console.log(`🖱️ JS: Klick auf Zelle (${gridX}, ${gridY})`);
                
                this.dispatchEvent(new CustomEvent('cell-clicked', {
                    detail: { x: gridX, y: gridY },
                    bubbles: true, 
                    composed: true
                }));
            }
        }

        // RESET der internen Zustände
        console.log("🧹 JS: Drag-Status zurückgesetzt.");
        this.isMouseDown = false;
        this.isDragging = false;
        this.draggedAgentKey = null;
        this.style.cursor = 'crosshair';
    }
    
    private updateGrid() {
        const w = Math.max(1, parseInt(this.getAttribute('grid-width') || '10'));
        const h = Math.max(1, parseInt(this.getAttribute('grid-height') || '10'));
        this.gridCells.flat().forEach(cell => { this.scene.remove(cell); cell.geometry.dispose(); (cell.material as THREE.Material).dispose(); });
        this.gridCells = [];
        const geometry = new THREE.PlaneGeometry(1, 1);
        geometry.rotateX(-Math.PI / 2);
        for (let x = 0; x < w; x++) {
            this.gridCells[x] = [];
            for (let y = 0; y < h; y++) {
                const cell = new THREE.Mesh(geometry, new THREE.MeshPhongMaterial({ color: this.defaultGroundColor }));
                cell.position.set(x + 0.5, 0, y + 0.5);
                cell.userData = { gridX: x, gridY: y };
                this.scene.add(cell);
                this.gridCells[x][y] = cell;
            }
        }
        if (this.gridHelper) this.scene.remove(this.gridHelper);
        const maxDim = Math.max(w, h);
        this.gridHelper = new THREE.GridHelper(maxDim, maxDim, 0x4a5568, 0x2d3748);
        this.gridHelper.position.set(maxDim / 2, 0.01, maxDim / 2);
        this.scene.add(this.gridHelper);
        this.updateCameraFocus(w, h);
        this.updateMarkers();
    }

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
        if (this.gridCells[x] && this.gridCells[x][y]) (this.gridCells[x][y].material as THREE.MeshPhongMaterial).color.setHex(color);
    }

    private updateAgents(agents: any[]) {
        console.log("📦 3D-Engine: Verarbeite Agenten-Liste", agents);
        const currentIds = new Set(agents.map((a: any) => a.agent_id));
        const toRemove = [...this.agentMeshes.keys()].filter(id => !currentIds.has(id));
        
        agents.forEach((agent: any) => {
            const id = agent.agent_id;
            let mesh = this.agentMeshes.get(id);
            const x = agent.x ?? (agent.position?.x || 0);
            const y = agent.y ?? (agent.position?.y || 0);
            const targetX = x + 0.5, targetZ = y + 0.5;
            const targetY = (agent.is_dynamic || agent.module_type === 'ftf') ? 0.05 : 0.1;

            if (!mesh) {
                console.log(`✨ Erstelle neuen Agenten: ${id}`);
                mesh = this.createAgentMesh(agent.module_type, agent.is_dynamic);
                this.scene.add(mesh);
                this.agentMeshes.set(id, mesh);
                mesh.position.set(targetX, targetY, targetZ);
            } else {
                if (agent.is_dynamic || agent.module_type === 'ftf') mesh.position.lerp(new THREE.Vector3(targetX, targetY, targetZ), 0.2);
                else mesh.position.set(targetX, targetY, targetZ);
            }
            mesh.rotation.y = (agent.orientation || 0) * (Math.PI / 180);
        });

        toRemove.forEach(id => {
            const mesh = this.agentMeshes.get(id);
            if (mesh) { this.scene.remove(mesh); this.agentMeshes.delete(id); }
        });
    }

    private createAgentMesh(type: string, isDynamic: boolean): THREE.Object3D {
        const group = new THREE.Group();
        if (isDynamic || type === 'ftf') {
            const body = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.2, 0.8), new THREE.MeshPhongMaterial({ color: 0xffd700 }));
            body.position.set(0, 0.1, 0); group.add(body);
            const eye = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.1, 0.1), new THREE.MeshBasicMaterial({ color: 0xff0000 }));
            eye.position.set(0, 0.2, 0.35); group.add(eye);
        } else {
            let h = 0.3, c = 0x718096;
            if (type.includes('rollen')) { h = 0.15; c = 0x3182ce; }
            else if (type === 'greifer') { h = 0.6; c = 0xed8936; }
            const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.85, h, 0.85), new THREE.MeshPhongMaterial({ color: c }));
            mesh.position.set(0, h/2, 0); group.add(mesh);
        }
        return group;
    }

    private drawPath(nodes: any[]) {
        if (this.pathLine) { this.scene.remove(this.pathLine); this.pathLine.geometry.dispose(); }
        if (!nodes || nodes.length < 2) return;
        const pts = nodes.map(n => new THREE.Vector3((n.position?.x || n.x) + 0.5, 0.12, (n.position?.y || n.y) + 0.5));
        this.pathLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x00f2ff }));
        this.scene.add(this.pathLine);
    }

    private updateCameraFocus(w: number, h: number) {
        const is3D = this.getAttribute('is-3d') === 'true';
        const d = Math.max(w, h);
        if (is3D) this.targetCameraPos.set(d * 1.2, d * 0.8, d * 1.2);
        else this.targetCameraPos.set(w / 2, d * 1.1, h / 2);
    }

    private toggleCamera(is3D: boolean) { this.updateCameraFocus(parseInt(this.getAttribute('grid-width') || '10'), parseInt(this.getAttribute('grid-height') || '10')); }
    private getMouseCoords(e: MouseEvent) { const r = this.renderer.domElement.getBoundingClientRect(); return { x: ((e.clientX - r.left) / r.width) * 2 - 1, y: -((e.clientY - r.top) / r.height) * 2 + 1 }; }

    private animate() {
        requestAnimationFrame(() => this.animate());
        this.camera.position.lerp(this.targetCameraPos, this.lerpSpeed);
        const w = parseInt(this.getAttribute('grid-width') || '10'), h = parseInt(this.getAttribute('grid-height') || '10');
        this.camera.lookAt(w / 2, 0, h / 2);
        this.renderer.setViewport(0, 0, this.clientWidth, this.clientHeight);
        this.renderer.render(this.scene, this.camera);
        this.renderAxesHelper();
    }

    private renderAxesHelper() {
        this.renderer.autoClear = false;
        this.renderer.setScissorTest(true);
        this.renderer.setScissor(10, 10, 100, 100);
        this.renderer.setViewport(10, 10, 100, 100);
        this.helperCamera.quaternion.copy(this.camera.quaternion);
        this.renderer.render(this.helperScene, this.helperCamera);
        this.renderer.setScissorTest(false);
    }
}
customElements.define('three-grid-scene', ThreeGridScene);