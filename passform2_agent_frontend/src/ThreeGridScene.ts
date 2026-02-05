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
    private bayMeshes: Map<string, THREE.LineSegments> = new Map(); // NEU: Speicher für die Buchten-Umrandungen
    private pathLine: THREE.Line | null = null;
    
    private readonly defaultGroundColor = 0x2d3748;
    private readonly startColor = 0x48bb78; 
    private readonly goalColor = 0x3182ce;  
    private readonly occupiedBayColor = 0x00f2ff; // Cyan für belegte Buchten
    private readonly vacantBayColor = 0x4a5568;   // Dunkelgrau für freie Buchten

    private startPos: {x: number, y: number} | null = null;
    private goalPos: {x: number, y: number} | null = null;

    private targetCameraPos = new THREE.Vector3(0, 20, 0);
    private lerpSpeed = 0.1; 
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
        
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.9);
        this.scene.add(ambientLight);
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.5);
        dirLight.position.set(10, 50, 10);
        this.scene.add(dirLight);

        this.camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
        this.camera.up.set(0, 0, -1); 

        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.raycaster = new THREE.Raycaster();

        this.helperScene = new THREE.Scene();
        this.helperCamera = new THREE.PerspectiveCamera(75, 1, 0.1, 100);
        this.helperCamera.position.set(0, 0, 4); 

        const axes = new THREE.AxesHelper(2);
        this.helperScene.add(axes);
    }

    static get observedAttributes() {
        // 'bays' wurde hier hinzugefügt
        return ['agents', 'is-3d', 'grid-width', 'grid-height', 'path', 'start-pos', 'goal-pos', 'allow-interaction', 'bays'];
    }

    attributeChangedCallback(name: string, oldVal: string, newVal: string) {
        if (oldVal === newVal) return;
        try {
            switch (name) {
                case 'grid-width':
                case 'grid-height': this.updateGrid(); break;
                case 'is-3d': this.updateCameraFocus(); break;
                case 'agents': 
                    if (!this.isDragging && newVal) {
                        const parsed = JSON.parse(newVal);
                        const list = Array.isArray(parsed) ? parsed : (parsed.agents || []);
                        this.updateAgents(list);
                    }
                    break;
                case 'bays': // NEU: Reagiert auf die Buchten-Daten aus Elm
                    if (newVal) this.updateBays(JSON.parse(newVal));
                    break;
                case 'path': if(newVal) this.drawPath(JSON.parse(newVal)); break;
                case 'start-pos':
                case 'goal-pos': this.updateMarkers(); break;
            }
        } catch (e) { console.error(`❌ Attribute Error: ${name}`, e); }
    }

    connectedCallback() {
        const style = document.createElement('style');
        style.textContent = `
            :host { display: block !important; width: 100% !important; height: 100% !important; position: relative; overflow: hidden; background: #1a202c; }
            canvas { display: block !important; width: 100% !important; height: 100% !important; }
        `;
        this.shadowRoot?.appendChild(style);
        this.shadowRoot?.appendChild(this.renderer.domElement);

        this.resizeObserver = new ResizeObserver(() => {
            requestAnimationFrame(() => this.resize());
        });
        this.resizeObserver.observe(this);

        this.renderer.domElement.addEventListener('mousedown', (e) => this.onMouseDown(e));
        this.renderer.domElement.addEventListener('mousemove', (e) => this.onMouseMove(e));
        this.renderer.domElement.addEventListener('mouseup', (e) => this.onMouseUp(e));
        
        this.updateGrid();
        this.animate();
    }

    private resize() {
        const width = this.clientWidth;
        const height = this.clientHeight;
        if (width <= 0 || height <= 0) return;
        this.renderer.setSize(width, height, false);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
    }

    private updateGrid() {
        const w = Math.max(1, parseInt(this.getAttribute('grid-width') || '6'));
        const h = Math.max(1, parseInt(this.getAttribute('grid-height') || '4'));
        
        this.gridCells.flat().forEach(cell => { this.scene.remove(cell); cell.geometry.dispose(); (cell.material as THREE.Material).dispose(); });
        this.gridCells = [];

        const geometry = new THREE.PlaneGeometry(0.95, 0.95);
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
        this.gridHelper = new THREE.GridHelper(Math.max(w,h)*2, Math.max(w,h)*2, 0x2d3748, 0x2d3748);
        this.gridHelper.position.set(w/2, -0.01, h/2);
        this.scene.add(this.gridHelper);

        this.updateCameraFocus();
    }

    private updateBays(bays: any[]) {
        const currentIds = new Set(bays.map(b => b.unique_id));
        
        // 1. Entferne Bays, die nicht mehr existieren
        [...this.bayMeshes.keys()].forEach(id => {
            if (!currentIds.has(id)) {
                this.scene.remove(this.bayMeshes.get(id)!);
                this.bayMeshes.delete(id);
            }
        });

        // 2. Erzeuge oder aktualisiere Bay-Umrandungen
        bays.forEach(bay => {
            let wireframe = this.bayMeshes.get(bay.unique_id);
            const color = bay.occupation ? this.occupiedBayColor : this.vacantBayColor;
            
            if (!wireframe) {
                // Erzeuge EdgesGeometry für eine Plane (Umrandung eines Quadrats)
                const geo = new THREE.EdgesGeometry(new THREE.PlaneGeometry(0.98, 0.98));
                const mat = new THREE.LineBasicMaterial({ color: color, linewidth: 2 });
                wireframe = new THREE.LineSegments(geo, mat);
                
                wireframe.rotateX(-Math.PI / 2);
                // Minimal über dem Boden (0.005), um Z-Fighting zu vermeiden
                wireframe.position.set(bay.x + 0.5, 0.005, bay.y + 0.5);
                
                this.scene.add(wireframe);
                this.bayMeshes.set(bay.unique_id, wireframe);
            } else {
                // Nur die Farbe anpassen, wenn die Bay bereits existiert
                (wireframe.material as THREE.LineBasicMaterial).color.setHex(color);
                wireframe.position.set(bay.x + 0.5, 0.005, bay.y + 0.5);
            }
        });
    }

    private updateCameraFocus() {
        const w = parseInt(this.getAttribute('grid-width') || '6');
        const h = parseInt(this.getAttribute('grid-height') || '4');
        const is3D = this.getAttribute('is-3d') === 'true';
        
        const centerX = w / 2;
        const centerZ = h / 2;
        const dist = Math.max(w, h);

        if (is3D) {
            this.camera.up.set(0, 1, 0); 
            this.targetCameraPos.set(centerX + dist, dist, centerZ + dist);
        } else {
            this.camera.up.set(0, 0, -1); 
            this.targetCameraPos.set(centerX, dist * 1.5, centerZ);
        }
    }

    private animate() {
        requestAnimationFrame(() => this.animate());
        const w = parseInt(this.getAttribute('grid-width') || '6');
        const h = parseInt(this.getAttribute('grid-height') || '4');
        this.camera.position.lerp(this.targetCameraPos, this.lerpSpeed);
        this.camera.lookAt(w / 2, 0, h / 2);
        this.renderer.render(this.scene, this.camera);
    }

    private getMouseCoords(e: MouseEvent) { 
        const r = this.renderer.domElement.getBoundingClientRect(); 
        return { x: ((e.clientX - r.left) / r.width) * 2 - 1, y: -((e.clientY - r.top) / r.height) * 2 + 1 }; 
    }

    private onMouseDown(event: MouseEvent) {
        this.isMouseDown = true;
        this.mouseDownPos = { x: event.clientX, y: event.clientY };
    }

    private onMouseMove(event: MouseEvent) {
        const coords = this.getMouseCoords(event);
        this.raycaster.setFromCamera(coords, this.camera);
        const intersects = this.raycaster.intersectObjects(Array.from(this.agentMeshes.values()), true);
        
        if (!this.isMouseDown) {
            this.style.cursor = intersects.length > 0 ? 'pointer' : 'default';
            return;
        }

        const delta = Math.hypot(event.clientX - this.mouseDownPos.x, event.clientY - this.mouseDownPos.y);
        if (!this.isDragging && delta > this.dragThreshold && intersects.length > 0) {
            this.isDragging = true;
            let target = intersects[0].object;
            while (target.parent && target.parent !== this.scene) target = target.parent;
            this.draggedAgentKey = [...this.agentMeshes.entries()].find(([_, v]) => v === target)?.[0] || null;
        }

        if (this.isDragging && this.draggedAgentKey) {
            const ground = this.raycaster.intersectObjects(this.gridCells.flat());
            if (ground.length > 0) {
                const mesh = this.agentMeshes.get(this.draggedAgentKey);
                if (mesh) {
                    mesh.position.x = ground[0].object.userData.gridX + 0.5;
                    mesh.position.z = ground[0].object.userData.gridY + 0.5;
                }
            }
        }
    }

    private onMouseUp(event: MouseEvent) {
        if (this.isDragging && this.draggedAgentKey) {
            const mesh = this.agentMeshes.get(this.draggedAgentKey);
            if (mesh) {
                const newX = Math.floor(mesh.position.x);
                const newY = Math.floor(mesh.position.z);
                this.dispatchEvent(new CustomEvent('agent-moved', {
                    detail: { agentId: this.draggedAgentKey, newX, newY },
                    bubbles: true, composed: true
                }));
            }
        } else if (this.isMouseDown) {
            const coords = this.getMouseCoords(event);
            this.raycaster.setFromCamera(coords, this.camera);
            const ground = this.raycaster.intersectObjects(this.gridCells.flat());
            if (ground.length > 0) {
                this.dispatchEvent(new CustomEvent('cell-clicked', {
                    detail: { x: ground[0].object.userData.gridX, y: ground[0].object.userData.gridY },
                    bubbles: true, composed: true
                }));
            }
        }
        this.isMouseDown = false;
        this.isDragging = false;
        this.draggedAgentKey = null;
    }

    private updateAgents(agents: any[]) {
        const currentIds = new Set(agents.map((a: any) => a.agent_id));
        [...this.agentMeshes.keys()].forEach(id => {
            if (!currentIds.has(id)) {
                this.scene.remove(this.agentMeshes.get(id)!);
                this.agentMeshes.delete(id);
            }
        });

        agents.forEach(agent => {
            let mesh = this.agentMeshes.get(agent.agent_id);
            const tx = (agent.x ?? agent.position?.x ?? 0) + 0.5;
            const tz = (agent.y ?? agent.position?.y ?? 0) + 0.5;

            if (!mesh) {
                mesh = this.createAgentMesh(agent.module_type, agent.is_dynamic);
                this.scene.add(mesh);
                this.agentMeshes.set(agent.agent_id, mesh);
                mesh.position.set(tx, 0.1, tz);
            } else {
                mesh.position.lerp(new THREE.Vector3(tx, mesh.position.y, tz), 0.2);
            }
            mesh.rotation.y = (agent.orientation || 0) * (Math.PI / 180);
        });
    }

    private createAgentMesh(type: string, isDynamic: boolean): THREE.Object3D {
        const group = new THREE.Group();
        type = type.toLowerCase();

        if (isDynamic || type === 'ftf' || type === 'ranger') {
            const body = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.2, 0.8), new THREE.MeshPhongMaterial({ color: 0xffd700 }));
            body.position.set(0, 0.1, 0); 
            group.add(body);
            const eye = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.1, 0.1), new THREE.MeshBasicMaterial({ color: 0xff0000 }));
            eye.position.set(0, 0.2, 0.35); 
            group.add(eye);
        } else if (type.includes('greifer')) {
            const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.6, 0.8), new THREE.MeshPhongMaterial({ color: 0xed8936 }));
            mesh.position.set(0, 0.3, 0);
            group.add(mesh);
        } else if (type.includes('rollen')) {
            const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.85, 0.15, 0.85), new THREE.MeshPhongMaterial({ color: 0x3182ce }));
            mesh.position.set(0, 0.075, 0);
            group.add(mesh);
        } else {
            const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.3, 0.8), new THREE.MeshPhongMaterial({ color: 0x718096 }));
            mesh.position.set(0, 0.15, 0);
            group.add(mesh);
        }
        return group;
    }

    private drawPath(nodes: any[]) {
        if (this.pathLine) { this.scene.remove(this.pathLine); }
        if (!nodes || nodes.length < 2) return;
        const pts = nodes.map(n => new THREE.Vector3((n.x ?? n.position.x) + 0.5, 0.1, (n.y ?? n.position.y) + 0.5));
        this.pathLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x00f2ff }));
        this.scene.add(this.pathLine);
    }

    private updateMarkers() {}
}
customElements.define('three-grid-scene', ThreeGridScene);