import * as THREE from 'three';

export class ThreeGridScene extends HTMLElement {
    private renderer: THREE.WebGLRenderer;
    private camera: THREE.PerspectiveCamera;
    private scene: THREE.Scene;
    private raycaster: THREE.Raycaster;
    private groundPlane: THREE.Mesh;
    
    // Map, um existierende Agenten-Meshes zu speichern (Key: x_y)
    private agentMeshes: Map<string, THREE.Object3D> = new Map();

    constructor() {
        super();
        this.attachShadow({ mode: 'open' });
        
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x000000);
        
        this.camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
        this.camera.position.set(16, 20, 16); // Blick von oben
        this.camera.lookAt(16, 0, 16);

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.raycaster = new THREE.Raycaster();

        // Licht hinzufügen, damit man etwas sieht
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
        this.scene.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
        directionalLight.position.set(10, 20, 10);
        this.scene.add(directionalLight);

        // Bodenplatte für Raycasting und Visualisierung
        const geometry = new THREE.PlaneGeometry(32, 32);
        const material = new THREE.MeshPhongMaterial({ color: 0x1a202c });
        this.groundPlane = new THREE.Mesh(geometry, material);
        this.groundPlane.rotation.x = -Math.PI / 2;
        this.groundPlane.position.set(16, 0, 16); // Zentriert auf 0-32 Bereich
        this.scene.add(this.groundPlane);

        // Ein Hilfsgitter (GridHelper)
        const gridHelper = new THREE.GridHelper(32, 32, 0x4a5568, 0x2d3748);
        gridHelper.position.set(16, 0.01, 16);
        this.scene.add(gridHelper);
    }

    // Diese Attribute überwacht die Komponente
    static get observedAttributes() {
        return ['agents', 'is-3d'];
    }

    // Wird aufgerufen, wenn Elm ein Attribut ändert
    attributeChangedCallback(name: string, oldVal: string, newVal: string) {
        if (oldVal === newVal) return;

        if (name === 'agents') {
            try {
                const agentsList = JSON.parse(newVal);
                this.updateAgents(agentsList);
            } catch (e) {
                console.error("Fehler beim Parsen der Agenten-Daten:", e);
            }
        }

        if (name === 'is-3d') {
            this.toggleCamera(newVal === 'true');
        }
    }

    connectedCallback() {
        this.shadowRoot?.appendChild(this.renderer.domElement);
        this.resize();
        window.addEventListener('resize', () => this.resize());
        this.animate();
        
        this.renderer.domElement.addEventListener('click', (e) => this.handleCanvasClick(e));
    }

    private updateAgents(agents: any[]) {
        // 1. Markiere alle aktuellen Meshes als "zu löschen"
        const toRemove = new Set(this.agentMeshes.keys());

        // 2. Erstelle oder aktualisiere Agenten aus der Elm-Liste
        agents.forEach(agent => {
            const key = `${agent.position.x}_${agent.position.y}`;
            toRemove.delete(key);

            if (!this.agentMeshes.has(key)) {
                const mesh = this.createAgentMesh(agent.module_type);
                // Weltkoordinaten: Gittermitte ist x + 0.5
                mesh.position.set(agent.position.x + 0.5, 0.5, agent.position.y + 0.5);
                this.scene.add(mesh);
                this.agentMeshes.set(key, mesh);
            }
        });

        // 3. Lösche Agenten, die nicht mehr in der Elm-Liste sind
        toRemove.forEach(key => {
            const mesh = this.agentMeshes.get(key);
            if (mesh) {
                this.scene.remove(mesh);
                this.agentMeshes.delete(key);
            }
        });
    }

    private createAgentMesh(type: string): THREE.Object3D {
        let geometry;
        let color;

        if (type.includes('rollen')) {
            geometry = new THREE.BoxGeometry(0.8, 0.2, 0.8);
            color = 0x4299e1; // Blau
        } else if (type === 'greifer') {
            geometry = new THREE.CylinderGeometry(0.3, 0.3, 1, 16);
            color = 0xed8936; // Orange
        } else {
            geometry = new THREE.SphereGeometry(0.4);
            color = 0xa0aec0; // Grau
        }

        const material = new THREE.MeshPhongMaterial({ color });
        return new THREE.Mesh(geometry, material);
    }

    private toggleCamera(is3D: boolean) {
        if (is3D) {
            this.camera.position.set(25, 15, 25);
        } else {
            this.camera.position.set(16, 25, 16);
        }
        this.camera.lookAt(16, 0, 16);
    }

    private handleCanvasClick(event: MouseEvent) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        const x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        const y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

        this.raycaster.setFromCamera({ x, y }, this.camera);
        const intersects = this.raycaster.intersectObject(this.groundPlane);

        if (intersects.length > 0) {
            const point = intersects[0].point;
            // Wir nutzen Math.floor, um auf die Gitter-ID zu kommen
            const gridX = Math.floor(point.x);
            const gridY = Math.floor(point.z);

            this.dispatchEvent(new CustomEvent('cell-clicked', {
                detail: { x: gridX, y: gridY },
                bubbles: true,
                composed: true
            }));
        }
    }

    private resize() {
        const width = this.parentElement?.clientWidth || 800;
        const height = this.parentElement?.clientHeight || 600;
        this.renderer.setSize(width, height);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
    }

    private animate() {
        requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
    }
}

customElements.define('three-grid-scene', ThreeGridScene);