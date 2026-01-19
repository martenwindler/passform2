import * as THREE from 'three';

export class ThreeGridScene extends HTMLElement {
    private renderer: THREE.WebGLRenderer;
    private camera: THREE.PerspectiveCamera;
    private scene: THREE.Scene;
    
    private helperScene: THREE.Scene;
    private helperCamera: THREE.PerspectiveCamera;
    private resizeObserver: ResizeObserver | null = null; // HINZUGEFÜGT

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
                case 'allow-interaction': this.style.cursor = 'default'; break;
            }
        } catch (e) { console.error(`Attribute update error (${name}):`, e); }
    }

    connectedCallback() {
        const style = document.createElement('style');
        style.textContent = `
            canvas { display: block; width: 100%; height: 100%; outline: none; } 
            :host { display: block; width: 100%; height: 100%; overflow: hidden; }
        `;
        this.shadowRoot?.appendChild(style);
        this.shadowRoot?.appendChild(this.renderer.domElement);

        // KORREKTUR: ResizeObserver statt Window-Event
        // Er erkennt Größenänderungen durch die Sidebar-Animation in Echtzeit
        this.resizeObserver = new ResizeObserver(() => this.resize());
        this.resizeObserver.observe(this);

        this.renderer.domElement.addEventListener('mousedown', (e) => this.onMouseDown(e));
        window.addEventListener('mousemove', (e) => this.onMouseMove(e));
        window.addEventListener('mouseup', (e) => this.onMouseUp(e));
        
        this.animate();
        this.updateGrid();
    }

    // HINZUGEFÜGT: Cleanup um Memory Leaks zu vermeiden
    disconnectedCallback() {
        if (this.resizeObserver) {
            this.resizeObserver.disconnect();
        }
    }

    private resize() {
        // Wir nehmen die Maße direkt von der Komponente selbst
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
        const interactionAllowed = this.isInteractionAllowed();
        const coords = this.getMouseCoords(event);
        this.raycaster.setFromCamera(coords, this.camera);

        if (interactionAllowed) {
            const groundHits = this.raycaster.intersectObjects(this.gridCells.flat());
            this.style.cursor = groundHits.length > 0 ? 'crosshair' : 'default';
        } else {
            this.style.cursor = 'default';
            return; 
        }

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
        if (!this.isInteractionAllowed()) {
            this.isMouseDown = this.isDragging = false;
            return;
        }

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
                this.dispatchEvent(new CustomEvent('agent-moved', {
                    detail: { oldX: oldPos[0], oldY: oldPos[1], newX, newY },
                    bubbles: true, composed: true
                }));
            }
        }
        this.isMouseDown = this.isDragging = false;
        this.draggedAgentKey = null;
    }

    private updateGrid() {
        const w = Math.max(1, parseInt(this.getAttribute('grid-width') || '10'));
        const h = Math.max(1, parseInt(this.getAttribute('grid-height') || '10'));
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

    private updateAgents(agents: any[]) {
        const currentIds = new Set(agents.map((a: any) => a.agent_id));
        const toRemove = new Set([...this.agentMeshes.keys()].filter(id => !currentIds.has(id)));
        agents.forEach((agent: any) => {
            const id = agent.agent_id;
            let mesh = this.agentMeshes.get(id);
            if (!mesh) {
                mesh = this.createAgentMesh(agent.module_type, agent.is_dynamic);
                this.scene.add(mesh);
                this.agentMeshes.set(id, mesh);
            }
            const targetX = agent.position.x + 0.5;
            const targetZ = agent.position.y + 0.5;
            if (agent.is_dynamic || agent.module_type === 'ftf') {
                mesh.position.lerp(new THREE.Vector3(targetX, 0.1, targetZ), 0.1);
            } else {
                mesh.position.set(targetX, 0.2, targetZ);
            }
            mesh.rotation.y = (agent.orientation || 0) * (Math.PI / 180);
        });
        toRemove.forEach(id => {
            const mesh = this.agentMeshes.get(id);
            if (mesh) { this.scene.remove(mesh); this.agentMeshes.delete(id); }
        });
    }

    private createDirectionArrows(moduleType: string): THREE.Group {
        const arrowGroup = new THREE.Group();
        const arrowColor = 0x00ffff; 
        const arrowLength = 0.5;
        const headLength = 0.2;
        const headWidth = 0.15;
        const directionMap: Record<string, THREE.Vector3[]> = {
            'greifer': [new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, -1), new THREE.Vector3(1, 0, 0), new THREE.Vector3(-1, 0, 0)],
            'mensch': [new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, -1), new THREE.Vector3(1, 0, 0), new THREE.Vector3(-1, 0, 0)],
            'rollen_ns': [new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, -1)],
            'rollen_ow': [new THREE.Vector3(1, 0, 0), new THREE.Vector3(-1, 0, 0)]
        };
        const dirs = directionMap[moduleType] || [];
        dirs.forEach(dir => {
            const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0, 0), arrowLength, arrowColor, headLength, headWidth);
            [arrow.line, arrow.cone].forEach((obj: any) => {
                obj.material.transparent = true;
                obj.material.opacity = 0.9;
                obj.material.depthTest = false; 
                obj.renderOrder = 999;
            });
            arrowGroup.add(arrow);
        });
        return arrowGroup;
    }

    private createAgentMesh(type: string, isDynamic: boolean): THREE.Object3D {
        const group = new THREE.Group();
        let arrowsY = 0.2;
        if (isDynamic || type === 'ftf') {
            const bodyGeo = new THREE.BoxGeometry(0.7, 0.1, 0.8);
            const bodyMat = new THREE.MeshPhongMaterial({ color: 0xffd700, emissive: 0x443300 });
            const body = new THREE.Mesh(bodyGeo, bodyMat);
            group.add(body);
            const eyeGeo = new THREE.BoxGeometry(0.1, 0.05, 0.1);
            const eyeMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
            const eye = new THREE.Mesh(eyeGeo, eyeMat);
            eye.position.set(0, 0.05, 0.35);
            group.add(eye);
        } else {
            let geo, col, height = 0.3;
            if (type.includes('rollen')) { height = 0.15; geo = new THREE.BoxGeometry(0.85, height, 0.85); col = 0x3182ce; arrowsY = height / 2 + 0.05; }
            else if (type === 'greifer') { height = 0.6; geo = new THREE.CylinderGeometry(0.35, 0.35, height, 16); col = 0xed8936; arrowsY = height / 2 + 0.1; }
            else if (type === 'conveyeur') { height = 0.1; geo = new THREE.BoxGeometry(0.9, height, 0.9); col = 0x38a169; arrowsY = height / 2 + 0.05; }
            else { geo = new THREE.BoxGeometry(0.95, 0.3, 0.95); col = 0x718096; arrowsY = 0.3 / 2 + 0.1; }
            const mesh = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({ color: col }));
            group.add(mesh);
            const arrows = this.createDirectionArrows(type);
            arrows.position.y = arrowsY; 
            group.add(arrows);
        }
        return group;
    }

    private drawPath(pathNodes: any[]) {
        if (this.pathLine) { this.scene.remove(this.pathLine); this.pathLine.geometry.dispose(); }
        if (!pathNodes || pathNodes.length < 2) return;
        const points = pathNodes.map(n => {
            const pos = n.position || n;
            return new THREE.Vector3(pos.x + 0.5, 0.12, pos.y + 0.5);
        });
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ color: 0x00f2ff, transparent: true, opacity: 0.8 });
        this.pathLine = new THREE.Line(geometry, material);
        this.scene.add(this.pathLine);
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
        return { 
            x: ((e.clientX - r.left) / r.width) * 2 - 1, 
            y: -((e.clientY - r.top) / r.height) * 2 + 1 
        };
    }

    private animate() {
        requestAnimationFrame(() => this.animate());
        this.camera.position.lerp(this.targetCameraPos, this.lerpSpeed);
        const w = parseInt(this.getAttribute('grid-width') || '10');
        const h = parseInt(this.getAttribute('grid-height') || '10');
        this.camera.lookAt(w / 2, 0, h / 2);

        this.renderer.setScissorTest(false);
        this.renderer.setViewport(0, 0, this.clientWidth, this.clientHeight);
        this.renderer.render(this.scene, this.camera);
        this.renderAxesHelper();
    }

    private renderAxesHelper() {
        this.renderer.autoClear = false;
        this.renderer.setScissorTest(true);
        const size = 100;
        // Helper unten links fixieren
        this.renderer.setScissor(10, 10, size, size);
        this.renderer.setViewport(10, 10, size, size);
        this.renderer.clearDepth();

        this.helperCamera.quaternion.copy(this.camera.quaternion);
        this.renderer.render(this.helperScene, this.helperCamera);
    }
}
customElements.define('three-grid-scene', ThreeGridScene);