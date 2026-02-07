import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'; // NEU

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
    private bayMeshes: Map<string, THREE.LineSegments> = new Map();
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

    private lastViewMode: string | null = null;
    private isWaitingForSync = false;

    private isTransitioning = false;
    private readonly transitionSpeed = 0.07; // Geschwindigkeit des Eindrehens

    public controls: OrbitControls;    

    constructor() {
        super();
        this.attachShadow({ mode: 'open' });
        
        // 1. Szene & Hintergrund
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a202c);
        
        // 2. Licht-Setup
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.9);
        this.scene.add(ambientLight);
        
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.5);
        dirLight.position.set(10, 50, 10);
        this.scene.add(dirLight);

        // 3. Kamera & Renderer
        this.camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
        
        // WICHTIG: OrbitControls brauchen Y=Oben (0,1,0), um stabil um das Gitter zu kreisen
        this.camera.up.set(0, 1, 0); 
        
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true, 
            alpha: true 
        });

        // 4. OrbitControls Initialisierung
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        
        // --- MAUS-KONFIGURATION ---
        // Wir lassen die LINKE Taste für Elm (Agenten ziehen/klicken) frei!
        this.controls.mouseButtons = {
            LEFT: null,                 // Keine Kamera-Drehung mit links
            MIDDLE: THREE.MOUSE.DOLLY,  // Zoom mit Mausrad
            RIGHT: THREE.MOUSE.PAN      // Verschieben mit rechts
        };

        // --- FEEL & PHYSICS ---
        this.controls.enableDamping = true;   // Sanftes Nachgleiten
        this.controls.dampingFactor = 0.05;
        this.controls.screenSpacePanning = false; // Schiebt auf der X/Z Ebene
        
        // Verhindert, dass man unter das Gitter gucken kann (optional, aber empfohlen)
        this.controls.maxPolarAngle = Math.PI / 2.1; 

        // 5. Interaktion & Hilfs-Systeme
        this.raycaster = new THREE.Raycaster();
        
        this.helperScene = new THREE.Scene();
        this.helperCamera = new THREE.PerspectiveCamera(75, 1, 0.1, 100);
        this.helperCamera.position.set(0, 0, 4); 

        const axes = new THREE.AxesHelper(2);
        this.helperScene.add(axes);

        // Initialer Sync
        this.controls.update();
    }

    set agents(val: any) {
        if (!val) return;
        const list = Array.isArray(val) ? val : Object.values(val);
        
        // Sobald neue Daten kommen, geben wir die Sperre frei
        this.isWaitingForSync = false; 
        this.draggedAgentKey = null; 

        this.updateAgents(list);
    }

    set bays(val: any) {
        if (!val) return;
        this.updateBays(Array.isArray(val) ? val : Object.values(val));
    }

    set currentPath(val: any) {
        if (!val) return;
        this.drawPath(val.path || []);
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
                        
                        // KORREKTUR: Wir stellen sicher, dass wir immer ein Array von Agenten haben,
                        // egal ob Elm ein Dict (Objekt) oder eine Liste (Array) schickt.
                        let list: any[] = [];
                        if (Array.isArray(parsed)) {
                            list = parsed;
                        } else if (typeof parsed === 'object') {
                            // Falls es das Haupt-Modell ist, nimm .agents, sonst das Objekt selbst
                            const agentsSource = parsed.agents || parsed;
                            list = Object.values(agentsSource);
                        }

                        console.log("3D Scene: Verarbeite", list.length, "Agenten");
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
        if (!bays) return;
        
        const currentIds = new Set(bays.map(b => b.unique_id));
        
        // 1. Aufräumen: Entferne gelöschte Bays
        for (const [id, mesh] of this.bayMeshes.entries()) {
            if (!currentIds.has(id)) {
                this.scene.remove(mesh);
                this.bayMeshes.delete(id);
            }
        }

        // 2. Aktualisieren oder Erstellen
        bays.forEach(bay => {
            let wireframe = this.bayMeshes.get(bay.unique_id);
            // Wir nehmen die Farbe basierend auf dem belegt-Status
            const color = bay.occupation ? this.occupiedBayColor : this.vacantBayColor;
            
            // WICHTIG: Koordinaten-Mapping prüfen (bay.x/y oder bay.origin.x/y?)
            const bx = (Number(bay.x ?? bay.origin?.x) || 0) + 0.5;
            const bz = (Number(bay.y ?? bay.origin?.y) || 0) + 0.5;

            if (!wireframe) {
                const geo = new THREE.EdgesGeometry(new THREE.PlaneGeometry(0.99, 0.99));
                const mat = new THREE.LineBasicMaterial({ color: color, linewidth: 4 });
                wireframe = new THREE.LineSegments(geo, mat);
                wireframe.rotateX(-Math.PI / 2);
                wireframe.position.set(bx, 0.02, bz); // 0.02 um über dem Boden zu liegen
                
                this.scene.add(wireframe);
                this.bayMeshes.set(bay.unique_id, wireframe);
            } else {
                // FARB-UPDATE ERZWINGEN:
                (wireframe.material as THREE.LineBasicMaterial).color.setHex(color);
                wireframe.position.set(bx, 0.02, bz);
            }
        });
    }

    private updateCameraFocus() {
        const w = parseInt(this.getAttribute('grid-width') || '6');
        const h = parseInt(this.getAttribute('grid-height') || '4');
        const is3DStr = this.getAttribute('is-3d');
        const is3D = is3DStr === 'true';
        
        const centerX = w / 2;
        const centerZ = h / 2;
        const dist = Math.max(w, h);

        if (this.controls) {
            // Fokuspunkt ist immer die Mitte
            this.controls.target.set(centerX, 0, centerZ);

            // --- 1. SCHRITT: Standard-Ziele definieren ---
            if (is3D) {
                // (-) ISO-ANSICHT (+45 Grad Versatz)
                // Wir starten "frontal" und drehen den Vektor dann um die Y-Achse
                const isoOffset = new THREE.Vector3(0, dist * 0.8, dist * 1.2);
                isoOffset.applyAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 4); // Exakt +45 Grad
                this.targetCameraPos.copy(this.controls.target).add(isoOffset);
            } else {
                // 2D-ANSICHT (Flach von oben)
                this.targetCameraPos.set(centerX, dist * 1.5, centerZ);
            }

            // --- 2. SCHRITT: Initial-Sonderfall vs. Transition ---
            if (this.lastViewMode === null) {
                // BEIM ALLERERSTEN LADEN: 
                // Falls 3D aktiv ist, erzwingen wir HIER die horizontale Ausrichtung
                if (is3D) {
                    this.targetCameraPos.set(centerX, dist * 0.8, centerZ + dist * 1.2);
                }
                
                this.camera.position.copy(this.targetCameraPos);
                this.lastViewMode = is3DStr;
                this.controls.update();
                console.log("3D Scene: Initialzustand horizontal fixiert.");
            } 
            else if (this.lastViewMode !== is3DStr) {
                // BEIM KLICK AUF 2D/3D BUTTON:
                // Jetzt wird das Ziel (inkl. der -45 Grad für 3D) sanft angefahren
                this.isTransitioning = true;
                this.lastViewMode = is3DStr;
                console.log("3D Scene: Transition in neue Perspektive gestartet.");
            }
        }
    }

    private animate() {
        requestAnimationFrame(() => this.animate());

        if (this.controls) {
            if (this.isTransitioning) {
                // Sanftes Gleiten zum Ziel
                this.camera.position.lerp(this.targetCameraPos, this.transitionSpeed);

                // Wenn wir nah genug am Ziel sind, stoppen wir das Gleiten, 
                // damit die manuelle Steuerung nicht mehr beeinflusst wird.
                if (this.camera.position.distanceTo(this.targetCameraPos) < 0.1) {
                    this.isTransitioning = false;
                    console.log("3D Scene: Transition beendet.");
                }
            }
            
            this.controls.update(); 
        }

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
            // Wir casten NUR gegen die Boden-Zellen, um die X/Z Ebene zu bestimmen
            const ground = this.raycaster.intersectObjects(this.gridCells.flat());
            if (ground.length > 0) {
                const mesh = this.agentMeshes.get(this.draggedAgentKey);
                if (mesh) {
                    // FIX: Wir ändern NUR x und z. 
                    // Die Höhe (y) darf während des Ziehens NICHT verändert werden,
                    // damit das Element nicht auf den Boden "klatscht" oder hüpft.
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
                // 1. Wir nutzen NICHT den Raycaster (der trifft oft die falsche Stelle/den Agenten selbst)
                // Stattdessen nehmen wir die Position, an die das Mesh in onMouseMove bereits "gesnappt" ist.
                // Da das Mesh immer auf .5 liegt (z.B. 1.5, 2.5), liefert Math.floor den exakten Integer.
                const finalX = Math.floor(mesh.position.x);
                const finalY = Math.floor(mesh.position.z);

                // 2. Mesh-Position final zentrieren (Sicherheits-Snap)
                mesh.position.x = finalX + 0.5;
                mesh.position.z = finalY + 0.5;

                // 3. Level berechnen
                const level = Math.round(mesh.position.y / 0.4); 

                // 4. Update-Sperre aktivieren
                this.isWaitingForSync = true;

                console.log(`✅ Drop abgeschlossen: Agent ${this.draggedAgentKey} fest auf (${finalX}, ${finalY})`);
                
                // 5. Event an Elm schicken
                this.dispatchEvent(new CustomEvent('agent-moved', {
                    detail: { 
                        agentId: this.draggedAgentKey, 
                        newX: finalX, 
                        newY: finalY,
                        level: level,
                        oldX: 0, 
                        oldY: 0
                    },
                    bubbles: true, 
                    composed: true
                }));
            }
        } else if (this.isMouseDown) {
            // Einfacher Klick (Selection)
            const coords = this.getMouseCoords(event);
            this.raycaster.setFromCamera(coords, this.camera);
            const ground = this.raycaster.intersectObjects(this.gridCells.flat());
            if (ground.length > 0) {
                this.dispatchEvent(new CustomEvent('cell-clicked', {
                    detail: { 
                        x: ground[0].object.userData.gridX, 
                        y: ground[0].object.userData.gridY 
                    },
                    bubbles: true, 
                    composed: true
                }));
            }
        }

        // Status zurücksetzen
        this.isMouseDown = false;
        this.isDragging = false;
        // WICHTIG: draggedAgentKey bleibt gesetzt, bis Elm per Property-Update (set agents) 
        // neue Daten schickt und isWaitingForSync auf false setzt!
    }

    private updateAgents(agents: any[]) {
        const getSafeId = (a: any): string => {
            const rawId = a.agent_id?.a || a.agent_id || a.unique_id || a.id;
            return String(rawId || "unknown");
        };

        const currentIds = new Set(agents.map(getSafeId));
        
        // --- DIESER TEIL LÖSCHT DIE 3D-OBJEKTE ---
        for (const [id, mesh] of this.agentMeshes.entries()) {
            if (!currentIds.has(id)) {
                this.scene.remove(mesh);
                this.agentMeshes.delete(id);
                console.log(`🗑️ 3D: Agent ${id} entfernt.`);
            }
        }
        // ----------------------------------------
        
        // 1. Aufräumen
        for (const [id, mesh] of this.agentMeshes.entries()) {
            if (!currentIds.has(id)) {
                this.scene.remove(mesh);
                this.agentMeshes.delete(id);
            }
        }

        // 2. Verarbeiten
        agents.forEach(agent => {
            const id = getSafeId(agent);
            if (id === this.draggedAgentKey) return;

            // --- FIX: Koordinaten strikt auf Gittermitte zwingen ---
            const gx = Number(agent.position?.x ?? agent.x ?? 0);
            const gy = Number(agent.position?.y ?? agent.y ?? 0);
            const glvl = Number(agent.position?.level ?? agent.lvl ?? 0);

            // Math.floor stellt sicher, dass wir bei 1.99 oder 1.01 immer die Zelle 1 treffen
            const tx = Math.floor(gx) + 0.5;
            const tz = Math.floor(gy) + 0.5; 
            const ty = (glvl * 0.4) + 0.05; 

            let mesh = this.agentMeshes.get(id);

            if (!mesh) {
                mesh = this.createAgentMesh(agent);
                this.scene.add(mesh);
                this.agentMeshes.set(id, mesh);
                mesh.position.set(tx, ty, tz);
            } else {
                // FIX GEGEN HÜPFEN & VERSATZ:
                const currentY = mesh.position.y;
                if (Math.abs(currentY - ty) > 0.1) {
                    mesh.position.y = ty; 
                }

                // Ziel-Vektor immer auf die .5 Mitte setzen
                const targetPos = new THREE.Vector3(tx, mesh.position.y, tz);
                mesh.position.lerp(targetPos, 0.2);
                
                mesh.position.y = THREE.MathUtils.lerp(mesh.position.y, ty, 0.1);
            }

            mesh.rotation.y = (Number(agent.orientation) || 0) * (Math.PI / 180);
        });
    }

    private createAgentMesh(agent: any): THREE.Object3D {
        const group = new THREE.Group();
        // Sicherstellen, dass wir einen String haben und Kleinschreibung nutzen
        const type = String(agent.module_type || "").toLowerCase();
        const isDynamic = !!agent.is_dynamic;

        // --- MATERIAL-HELPER (Etwas Glanz macht 3D schöner) ---
        const material = (color: number) => new THREE.MeshPhongMaterial({ 
            color: color, 
            flatShading: true,
            shininess: 30 
        });

        if (isDynamic || type === 'ftf' || type === 'ranger') {
            // FTF / Roboter (Gelb)
            const body = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.2, 0.8), material(0xffd700));
            body.position.set(0, 0.1, 0); 
            group.add(body);
            
            const eye = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.1, 0.1), new THREE.MeshBasicMaterial({ color: 0xff0000 }));
            eye.position.set(0, 0.2, 0.35); 
            group.add(eye);

        } else if (type.includes('greifer')) {
            // Greifer (Orange)
            const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.85, 0.15, 0.85), material(0xed8936));
            mesh.position.set(0, 0.075, 0);
            group.add(mesh);

        } else if (type.includes('rollen')) {
            // Rollenmodul (Blau)
            const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.85, 0.15, 0.85), material(0x3182ce));
            mesh.position.set(0, 0.075, 0);
            group.add(mesh);

        } else if (type.includes('tisch')) {
            // TISCH / STATION (Hellgrau statt Bodengrau!)
            // Wir nehmen 0x718096 (ein bläuliches Hellgrau), damit man ihn auf dem dunklen Boden sieht
            const top = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.2, 0.9), material(0x718096));
            top.position.set(0, 0.1, 0);
            group.add(top);

            // Kleine Füße an den Ecken, damit er wie ein Tisch aussieht
            const legGeo = new THREE.BoxGeometry(0.1, 0.1, 0.1);
            const legMat = material(0x2d3748);
            [[-0.4, -0.4], [0.4, -0.4], [-0.4, 0.4], [0.4, 0.4]].forEach(p => {
                const leg = new THREE.Mesh(legGeo, legMat);
                leg.position.set(p[0], 0.05, p[1]);
                group.add(leg);
            });

        } else {
            // Fallback für Unbekanntes (Grau)
            const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.3, 0.8), material(0x4a5568));
            mesh.position.set(0, 0.15, 0);
            group.add(mesh);
        }

        return group;
    }

    public rotateCamera(angle: number) {
        if (this.controls) {
            // Sobald der User eingreift, brechen wir die automatische Transition ab
            this.isTransitioning = false;

            const offset = new THREE.Vector3().copy(this.camera.position).sub(this.controls.target);
            offset.applyAxisAngle(new THREE.Vector3(0, 1, 0), angle);
            this.camera.position.copy(this.controls.target).add(offset);
            this.controls.update();
        }
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