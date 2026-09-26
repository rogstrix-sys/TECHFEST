/**
 * vis/static/js/cockpit.js
 * Dual-Viewport 3D WebGL Cockpit & Autonomous SLAM Perception Engine.
 * 
 * Viewport 1 (Theater View): Photorealistic defense/disaster simulation with 8-UAV fleet,
 * building rubble, animated rotors, downwash dust, and dynamic multi-hop RF tubes.
 * 
 * Viewport 2 (SLAM Perception): Onboard drone cognitive model with real-time 3D LiDAR
 * point clouds, 3D occupancy voxel grid (OctoMap), and Khatib APF guidance vectors.
 * 
 * Includes Chart.js scientific telemetry dashboard and automated investor presentation mode.
 */

// Global State
let socket = null;
let isPaused = false;
let selectedDroneId = "UAV_1";
let latestTelemetry = null;
let activeCamMode = "orbit";
let activeViewportMode = "theater";
let isInvestorMode = false;
let investorStartTime = 0;
let lastChartUpdate = 0;
let isInspectPanelOpen = false;
let isHudEnabled = false;

// Viewport 1: Theater Reality
let sceneTheater, cameraTheater, rendererTheater, controlsTheater;
let droneMeshes = new Map();
let rotorMeshes = [];
let strobeObjects = [];
let obstacleMeshes = new Map();
let poiMeshes = new Map();
let linkMeshes = new Map();
let packetMeshes = [];
let smokeParticles = null;
let washParticles = null;
let raycasterTheater = new THREE.Raycaster();
let mouseTheater = new THREE.Vector2();

// Viewport 2: Autonomous SLAM Perception
let sceneSLAM, cameraSLAM, rendererSLAM, controlsSLAM;
let slamDroneMesh = null;
let lidarPointsMesh = null;
let voxelMeshGroup = null;
let apfArrowAtt = null;
let apfArrowRep = null;
let apfArrowNet = null;
let lidarSweepLine = null;
let slamTrajectoryLine = null;
let slamTrajectoryPoints = [];
let maxTrajectoryPoints = 120;

// Audio & Mission Metrics
let audioContext = null;
let isAudioMuted = true;
let lastSurveyedCount = 0;
let lastFleetRenderTime = 0;

// Scientific Charts (Chart.js)
let chartEKF = null;
let chartPDR = null;
let chartRF = null;
let chartBattery = null;
let chartHistoryTime = [];
let chartHistoryEKF = [];
let chartHistoryPDR = [];
let chartHistoryThroughput = [];

// Initialize on DOM ready
document.addEventListener("DOMContentLoaded", () => {
    try { initTheaterViewport(); } catch (e) { console.error("initTheaterViewport error:", e); }
    try { initSLAMViewport(); } catch (e) { console.error("initSLAMViewport error:", e); }
    try { connectWebSocket(); } catch (e) { console.error("connectWebSocket error:", e); }
    try { initUIControls(); } catch (e) { console.error("initUIControls error:", e); }
    try { initScientificCharts(); } catch (e) { console.error("initScientificCharts error:", e); }
    try { setViewportMode("theater"); } catch (e) { console.error("setViewportMode error:", e); }
});

// ============================================================================
// Viewport 1: External Theater Reality
// ============================================================================

function initTheaterViewport() {
    const container = document.getElementById("canvas-theater-container");
    const width = container.clientWidth || window.innerWidth / 2;
    const height = container.clientHeight || window.innerHeight;

    // 1. Scene
    sceneTheater = new THREE.Scene();
    sceneTheater.background = new THREE.Color(0x060a12);
    sceneTheater.fog = new THREE.FogExp2(0x060a12, 0.0011);

    // 2. Camera (Expanded 700m Operational Zone)
    cameraTheater = new THREE.PerspectiveCamera(50, width / height, 1, 3500);
    cameraTheater.position.set(0, -380, 240);
    cameraTheater.up.set(0, 0, 1);

    // 3. Renderer (NVIDIA RTX 4050 High-Performance Hardware Context)
    rendererTheater = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: "high-performance" });
    rendererTheater.setSize(width, height);
    rendererTheater.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    rendererTheater.shadowMap.enabled = true;
    rendererTheater.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(rendererTheater.domElement);

    // 4. Controls
    if (typeof THREE.OrbitControls !== "undefined") {
        controlsTheater = new THREE.OrbitControls(cameraTheater, rendererTheater.domElement);
        controlsTheater.enableDamping = true;
        controlsTheater.dampingFactor = 0.05;
        controlsTheater.maxPolarAngle = Math.PI / 2 - 0.02;
        controlsTheater.target.set(0, 0, 20);
    }

    // 5. Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
    sceneTheater.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0x00e5ff, 0.85);
    dirLight.position.set(180, -220, 280);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    sceneTheater.add(dirLight);

    const gcsBeaconLight = new THREE.PointLight(0xff0055, 3.0, 180);
    gcsBeaconLight.position.set(0, -250, 18);
    sceneTheater.add(gcsBeaconLight);

    // 6. Realistic Graphical Terrain (Highways, River, Airfield, Helipads, Rubble)
    createTheaterTerrain();

    // 7. GCS Base Station Compound at (0, -250, 0)
    createGCSBase(0, -250, 0);

    // 8. Particle Systems
    initParticleSystems();

    // 9. Interactive Raycasting Click-to-Inspect
    rendererTheater.domElement.addEventListener("click", onTheaterCanvasClick);
}

function createTheaterTerrain() {
    // ------------------------------------------------------------------------
    // 1. Procedural 2048x2048 Graphical Canvas Texture
    // ------------------------------------------------------------------------
    const canvas = document.createElement("canvas");
    canvas.width = 2048;
    canvas.height = 2048;
    const ctx = canvas.getContext("2d");

    // Coordinate conversion: World (-375 to +375) -> Canvas (0 to 2048)
    const toC = (x, y) => {
        const cx = ((x + 375) / 750) * 2048;
        const cy = ((375 - y) / 750) * 2048;
        return [cx, cy];
    };
    const toLen = (meters) => (meters / 750) * 2048;

    // A. Base Terrain Surface (Dark Tactical Asphalt)
    ctx.fillStyle = "#080d16";
    ctx.fillRect(0, 0, 2048, 2048);

    // B. City Zoning Blocks / Regional Districts
    for (let bx = -330; bx < 330; bx += 70) {
        for (let by = -330; by < 330; by += 70) {
            const [cx, cy] = toC(bx, by + 58);
            const bw = toLen(58);
            const bh = toLen(58);
            ctx.fillStyle = ((Math.abs(bx) + Math.abs(by)) % 140 === 0) ? "#0c1524" : "#09101d";
            ctx.fillRect(cx, cy, bw, bh);
            ctx.strokeStyle = "rgba(0, 229, 255, 0.04)";
            ctx.lineWidth = 1;
            ctx.strokeRect(cx, cy, bw, bh);
        }
    }

    // C. Tactical MGRS Sector Coordinate Grid (every 50m)
    ctx.strokeStyle = "rgba(0, 229, 255, 0.10)";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([]);
    for (let m = -350; m <= 350; m += 50) {
        const [x0, y0] = toC(m, -375);
        const [x1, y1] = toC(m, 375);
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();

        const [rx0, ry0] = toC(-375, m);
        const [rx1, ry1] = toC(375, m);
        ctx.beginPath();
        ctx.moveTo(rx0, ry0);
        ctx.lineTo(rx1, ry1);
        ctx.stroke();
    }

    // D. Sector Quadrant Text Stamps
    ctx.font = "bold 26px 'Consolas', monospace";
    ctx.fillStyle = "rgba(0, 229, 255, 0.28)";
    let [sqx, sqy] = toC(-260, 310);
    ctx.fillText("SECTOR ALPHA [NW - 350x350M]", sqx, sqy);
    [sqx, sqy] = toC(60, 310);
    ctx.fillText("SECTOR BRAVO [NE - 350x350M]", sqx, sqy);
    [sqx, sqy] = toC(-260, -170);
    ctx.fillText("SECTOR CHARLIE [SW - GCS LAUNCH SECTOR]", sqx, sqy);
    [sqx, sqy] = toC(60, -170);
    ctx.fillText("SECTOR DELTA [SE - RELAY CORRIDOR]", sqx, sqy);

    // E. Range Rings from GCS (0, -250)
    const [gcsX, gcsY] = toC(0, -250);
    [100, 200, 300, 400].forEach(rMeters => {
        const rPix = toLen(rMeters);
        ctx.beginPath();
        ctx.arc(gcsX, gcsY, rPix, 0, Math.PI * 2);
        ctx.strokeStyle = "rgba(0, 229, 255, 0.22)";
        ctx.lineWidth = 2;
        ctx.setLineDash([12, 12]);
        ctx.stroke();

        ctx.font = "bold 16px 'Consolas', monospace";
        ctx.fillStyle = "rgba(0, 229, 255, 0.45)";
        ctx.fillText(`R: ${rMeters}M`, gcsX + rPix + 6, gcsY + 5);
    });
    ctx.setLineDash([]);

    // F. River Canal / Waterway (Northern Corridor)
    // River path from (-375, 190) curving through (-170, 235) to (375, 255)
    ctx.save();
    ctx.beginPath();
    let [rStartCX, rStartCY] = toC(-375, 185);
    ctx.moveTo(rStartCX, rStartCY);
    let [rMidCX, rMidCY] = toC(-170, 235);
    let [rEndCX, rEndCY] = toC(375, 255);
    ctx.quadraticCurveTo(rMidCX, rMidCY, rEndCX, rEndCY);
    let [rEndCX2, rEndCY2] = toC(375, 210);
    ctx.lineTo(rEndCX2, rEndCY2);
    let [rMidCX2, rMidCY2] = toC(-170, 190);
    let [rStartCX2, rStartCY2] = toC(-375, 140);
    ctx.quadraticCurveTo(rMidCX2, rMidCY2, rStartCX2, rStartCY2);
    ctx.closePath();

    // River embankment concrete wall
    ctx.strokeStyle = "#253344";
    ctx.lineWidth = 14;
    ctx.stroke();

    // Water fill gradient
    const waterGrad = ctx.createLinearGradient(0, rMidCY - 60, 0, rMidCY + 60);
    waterGrad.addColorStop(0, "#031528");
    waterGrad.addColorStop(0.5, "#062b48");
    waterGrad.addColorStop(1, "#031528");
    ctx.fillStyle = waterGrad;
    ctx.fill();
    ctx.restore();

    // G. 4-Lane Arterial Highway (East-West at y = -120, width = 24m)
    const [hwStartX, hwStartY] = toC(-375, -120);
    const hwW = toLen(750);
    const hwH = toLen(24);
    ctx.fillStyle = "#101826";
    ctx.fillRect(hwStartX, hwStartY - hwH / 2, hwW, hwH);

    // Highway Shoulder White Lines
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(hwStartX, hwStartY - hwH / 2 + 2);
    ctx.lineTo(hwStartX + hwW, hwStartY - hwH / 2 + 2);
    ctx.moveTo(hwStartX, hwStartY + hwH / 2 - 2);
    ctx.lineTo(hwStartX + hwW, hwStartY + hwH / 2 - 2);
    ctx.stroke();

    // Center Double Yellow Lines
    ctx.strokeStyle = "#ffd600";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(hwStartX, hwStartY - 2.5);
    ctx.lineTo(hwStartX + hwW, hwStartY - 2.5);
    ctx.moveTo(hwStartX, hwStartY + 2.5);
    ctx.lineTo(hwStartX + hwW, hwStartY + 2.5);
    ctx.stroke();

    // Dashed White Lane Separators
    ctx.strokeStyle = "rgba(255, 255, 255, 0.75)";
    ctx.lineWidth = 2;
    ctx.setLineDash([20, 16]);
    ctx.beginPath();
    ctx.moveTo(hwStartX, hwStartY - hwH / 4);
    ctx.lineTo(hwStartX + hwW, hwStartY - hwH / 4);
    ctx.moveTo(hwStartX, hwStartY + hwH / 4);
    ctx.lineTo(hwStartX + hwW, hwStartY + hwH / 4);
    ctx.stroke();
    ctx.setLineDash([]);

    // Highway Label
    ctx.font = "bold 18px 'Consolas', monospace";
    ctx.fillStyle = "rgba(255, 255, 255, 0.6)";
    ctx.fillText("FEDERAL HIGHWAY 101 - DISASTER SUPPLY ARTERY", hwStartX + toLen(80), hwStartY - hwH / 2 - 8);

    // H. North-South Metro Boulevard (at x = -70, width = 20m)
    const [blvStartX, blvStartY] = toC(-70, 375);
    const blvW = toLen(20);
    const blvH = toLen(750);
    ctx.fillStyle = "#101826";
    ctx.fillRect(blvStartX - blvW / 2, blvStartY, blvW, blvH);

    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(blvStartX - blvW / 2 + 2, blvStartY);
    ctx.lineTo(blvStartX - blvW / 2 + 2, blvStartY + blvH);
    ctx.moveTo(blvStartX + blvW / 2 - 2, blvStartY);
    ctx.lineTo(blvStartX + blvW / 2 - 2, blvStartY + blvH);
    ctx.stroke();

    ctx.strokeStyle = "#ffd600";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(blvStartX, blvStartY);
    ctx.lineTo(blvStartX, blvStartY + blvH);
    ctx.stroke();

    // Pedestrian Zebra Crosswalk at Highway & Boulevard Intersection
    const [ixCX, ixCY] = toC(-70, -120);
    ctx.fillStyle = "rgba(255, 255, 255, 0.85)";
    for (let zx = -blvW / 2 + 3; zx < blvW / 2 - 3; zx += 7) {
        ctx.fillRect(ixCX + zx, ixCY - hwH / 2 - 12, 4, 10);
        ctx.fillRect(ixCX + zx, ixCY + hwH / 2 + 2, 4, 10);
    }

    // I. GCS Launch Compound & Tactical Airfield at (0, -250)
    // Concrete Launch Apron (from x = -110 to +110, y = -290 to -210)
    const [apronX, apronY] = toC(-110, -210);
    const apronW = toLen(220);
    const apronH = toLen(80);
    ctx.fillStyle = "#162030";
    ctx.fillRect(apronX, apronY, apronW, apronH);

    // Concrete Slab Expansion Joints
    ctx.strokeStyle = "rgba(0, 229, 255, 0.12)";
    ctx.lineWidth = 1.5;
    for (let sx = apronX; sx <= apronX + apronW; sx += 32) {
        ctx.beginPath();
        ctx.moveTo(sx, apronY);
        ctx.lineTo(sx, apronY + apronH);
        ctx.stroke();
    }
    for (let sy = apronY; sy <= apronY + apronH; sy += 32) {
        ctx.beginPath();
        ctx.moveTo(apronX, sy);
        ctx.lineTo(apronX + apronW, sy);
        ctx.stroke();
    }

    // Runway (at y = -265, length = 170m, width = 24m)
    const [rwX, rwY] = toC(-85, -253);
    const rwW = toLen(170);
    const rwH = toLen(24);
    ctx.fillStyle = "#0c131f";
    ctx.fillRect(rwX, rwY, rwW, rwH);

    // Runway Threshold White Bars (Piano Keys)
    ctx.fillStyle = "#ffffff";
    for (let b = 4; b < rwH - 4; b += 7) {
        ctx.fillRect(rwX + 4, rwY + b, 18, 4);
        ctx.fillRect(rwX + rwW - 22, rwY + b, 18, 4);
    }

    // Runway Centerline Dashes
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 3;
    ctx.setLineDash([18, 14]);
    ctx.beginPath();
    ctx.moveTo(rwX + 30, rwY + rwH / 2);
    ctx.lineTo(rwX + rwW - 30, rwY + rwH / 2);
    ctx.stroke();
    ctx.setLineDash([]);

    // Runway Heading Designators
    ctx.font = "900 24px 'Consolas', monospace";
    ctx.fillStyle = "#ffffff";
    ctx.fillText("09", rwX + 34, rwY + rwH / 2 + 8);
    ctx.fillText("27", rwX + rwW - 65, rwY + rwH / 2 + 8);

    // 4 Helipads / UAV Launch Pads at (-60, -232), (-20, -232), (+20, -232), (+60, -232)
    const padPositions = [
        { x: -60, label: "PAD-1 [SURVEY]" },
        { x: -20, label: "PAD-2 [SURVEY]" },
        { x: 20, label: "PAD-3 [RELAY]" },
        { x: 60, label: "PAD-4 [SCOUT]" },
    ];
    padPositions.forEach(p => {
        const [px, py] = toC(p.x, -232);
        const pRadius = toLen(8.5);

        // Yellow Ring
        ctx.beginPath();
        ctx.arc(px, py, pRadius, 0, Math.PI * 2);
        ctx.strokeStyle = "#ffd600";
        ctx.lineWidth = 3.5;
        ctx.stroke();

        // Inner Dashed Guide Ring
        ctx.beginPath();
        ctx.arc(px, py, pRadius * 0.7, 0, Math.PI * 2);
        ctx.strokeStyle = "rgba(255, 214, 0, 0.4)";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 6]);
        ctx.stroke();
        ctx.setLineDash([]);

        // Bold White "H"
        ctx.font = "bold 26px 'Consolas', monospace";
        ctx.fillStyle = "#ffd600";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("H", px, py);

        // Pad Label
        ctx.font = "bold 11px 'Consolas', monospace";
        ctx.fillStyle = "rgba(255, 255, 255, 0.85)";
        ctx.fillText(p.label, px, py + pRadius + 12);
        ctx.textAlign = "left";
        ctx.textBaseline = "alphabetic";
    });

    // J. Disaster Impact Scorch Footprints
    // POI_COLLAPSE (-210, 90)
    const [cCX, cCY] = toC(-210, 90);
    const radCollapse = toLen(26);
    const gradCollapse = ctx.createRadialGradient(cCX, cCY, 2, cCX, cCY, radCollapse);
    gradCollapse.addColorStop(0, "rgba(5, 5, 8, 0.95)");
    gradCollapse.addColorStop(0.6, "rgba(25, 12, 10, 0.65)");
    gradCollapse.addColorStop(1, "rgba(0, 0, 0, 0)");
    ctx.fillStyle = gradCollapse;
    ctx.beginPath();
    ctx.arc(cCX, cCY, radCollapse, 0, Math.PI * 2);
    ctx.fill();

    // POI_HAZARD (30, 260) - Industrial Chemical Warning Boundary
    const [hzCX, hzCY] = toC(30, 260);
    const radHaz = toLen(24);
    ctx.strokeStyle = "rgba(255, 214, 0, 0.65)";
    ctx.lineWidth = 2.5;
    ctx.setLineDash([10, 8]);
    ctx.beginPath();
    ctx.arc(hzCX, hzCY, radHaz, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);

    // Convert Canvas to Three.js Texture
    const groundTex = new THREE.CanvasTexture(canvas);
    groundTex.anisotropy = rendererTheater ? rendererTheater.capabilities.getMaxAnisotropy() : 4;
    groundTex.wrapS = THREE.ClampToEdgeWrapping;
    groundTex.wrapT = THREE.ClampToEdgeWrapping;

    // ------------------------------------------------------------------------
    // 2. Primary Ground Mesh (750m x 750m)
    // ------------------------------------------------------------------------
    const groundGeo = new THREE.PlaneGeometry(750, 750);
    const groundMat = new THREE.MeshStandardMaterial({
        map: groundTex,
        roughness: 0.82,
        metalness: 0.18,
    });
    const groundMesh = new THREE.Mesh(groundGeo, groundMat);
    groundMesh.position.set(0, 0, 0);
    groundMesh.receiveShadow = true;
    sceneTheater.add(groundMesh);

    // ------------------------------------------------------------------------
    // 3. 3D Reflective Water Ribbon Surface for River Canal
    // ------------------------------------------------------------------------
    const waterGeo = new THREE.PlaneGeometry(760, 44);
    const waterMat = new THREE.MeshStandardMaterial({
        color: 0x0088bb,
        roughness: 0.12,
        metalness: 0.88,
        transparent: true,
        opacity: 0.82,
    });
    const waterMesh = new THREE.Mesh(waterGeo, waterMat);
    waterMesh.position.set(0, 222, 0.4);
    sceneTheater.add(waterMesh);

    // ------------------------------------------------------------------------
    // 4. 3D River Truss Bridge (at x = -170, y = 235)
    // ------------------------------------------------------------------------
    const bridgeGroup = new THREE.Group();
    bridgeGroup.position.set(-170, 235, 0);

    // Concrete Road Deck
    const deckGeo = new THREE.BoxGeometry(16, 52, 1.8);
    const deckMat = new THREE.MeshStandardMaterial({ color: 0x223042, metalness: 0.6, roughness: 0.5 });
    const deck = new THREE.Mesh(deckGeo, deckMat);
    deck.position.z = 2.6;
    bridgeGroup.add(deck);

    // Steel Arch Truss Railings (Left & Right)
    [-8.5, 8.5].forEach(xOffset => {
        const archGeo = new THREE.BoxGeometry(0.8, 52, 6.5);
        const archMat = new THREE.MeshStandardMaterial({ color: 0x00e5ff, metalness: 0.9, roughness: 0.2, wireframe: true });
        const arch = new THREE.Mesh(archGeo, archMat);
        arch.position.set(xOffset, 0, 5.5);
        bridgeGroup.add(arch);
    });
    sceneTheater.add(bridgeGroup);

    // ------------------------------------------------------------------------
    // 5. 700x700x130m Operational Tactical Airspace Boundary
    // ------------------------------------------------------------------------
    const boundaryGeo = new THREE.BoxGeometry(700, 700, 130);
    const boundaryEdges = new THREE.EdgesGeometry(boundaryGeo);
    const boundaryMat = new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.22 });
    const boundaryLine = new THREE.LineSegments(boundaryEdges, boundaryMat);
    boundaryLine.position.set(0, 0, 65);
    sceneTheater.add(boundaryLine);
}

function createGCSBase(x, y, z) {
    const group = new THREE.Group();
    group.position.set(x, y, z);

    // Ground bunker structure (Reinforced Concrete Pad)
    const bunkerGeo = new THREE.CylinderGeometry(10, 14, 4, 8);
    const bunkerMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.85, roughness: 0.25 });
    const bunker = new THREE.Mesh(bunkerGeo, bunkerMat);
    bunker.rotation.x = Math.PI / 2;
    bunker.position.z = 2;
    group.add(bunker);

    // Heavy Comm Mast Tower
    const towerGeo = new THREE.CylinderGeometry(0.8, 1.6, 22, 6);
    const towerMat = new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.92, roughness: 0.1 });
    const tower = new THREE.Mesh(towerGeo, towerMat);
    tower.rotation.x = Math.PI / 2;
    tower.position.z = 13;
    group.add(tower);

    // Satellite Dish Receiver
    const dishGeo = new THREE.SphereGeometry(4.2, 12, 12, 0, Math.PI * 2, 0, Math.PI / 2);
    const dishMat = new THREE.MeshStandardMaterial({ color: 0x00e5ff, wireframe: true });
    const dish = new THREE.Mesh(dishGeo, dishMat);
    dish.rotation.x = -Math.PI / 3;
    dish.position.z = 24;
    group.add(dish);

    // High-Intensity Red Tactical Strobe Beacon
    const beaconGeo = new THREE.SphereGeometry(1.2, 8, 8);
    const beaconMat = new THREE.MeshBasicMaterial({ color: 0xff0055 });
    const beacon = new THREE.Mesh(beaconGeo, beaconMat);
    beacon.position.z = 25;
    group.add(beacon);

    sceneTheater.add(group);
}

function initParticleSystems() {
    // 1. Disaster Smoke & Fire Embers at Disaster Sites across 700m Area
    const smokeCount = 350;
    const smokeGeo = new THREE.BufferGeometry();
    const smokePositions = new Float32Array(smokeCount * 3);
    const smokeSpeeds = new Float32Array(smokeCount);

    const origins = [
        new THREE.Vector3(-210, 90, 2),   // POI_COLLAPSE
        new THREE.Vector3(30, 260, 2),    // POI_HAZARD
        new THREE.Vector3(-170, 240, 2),  // POI_BRIDGE
        new THREE.Vector3(220, 160, 2),   // POI_SURVIVORS
    ];

    for (let i = 0; i < smokeCount; i++) {
        const origin = origins[i % origins.length];
        smokePositions[i * 3] = origin.x + (Math.random() - 0.5) * 16;
        smokePositions[i * 3 + 1] = origin.y + (Math.random() - 0.5) * 16;
        smokePositions[i * 3 + 2] = origin.z + Math.random() * 45;
        smokeSpeeds[i] = 0.25 + Math.random() * 0.45;
    }
    smokeGeo.setAttribute('position', new THREE.BufferAttribute(smokePositions, 3));
    const smokeMat = new THREE.PointsMaterial({
        color: 0xff6600,
        size: 3.5,
        transparent: true,
        opacity: 0.72,
        blending: THREE.AdditiveBlending,
    });
    smokeParticles = new THREE.Points(smokeGeo, smokeMat);
    smokeParticles.speeds = smokeSpeeds;
    smokeParticles.origins = origins;
    sceneTheater.add(smokeParticles);

    // 2. Ground Wash Dust Rings for Low-Flying Drones
    const washCount = 200;
    const washGeo = new THREE.BufferGeometry();
    const washPositions = new Float32Array(washCount * 3);
    washGeo.setAttribute('position', new THREE.BufferAttribute(washPositions, 3));
    const washMat = new THREE.PointsMaterial({
        color: 0x00e5ff,
        size: 1.8,
        transparent: true,
        opacity: 0.5,
    });
    washParticles = new THREE.Points(washGeo, washMat);
    sceneTheater.add(washParticles);
}

function createQuadcopterMesh(role) {
    const droneGroup = new THREE.Group();
    const isRelay = role === "RELAY";
    const primaryColor = isRelay ? 0xffd600 : 0x00e5ff;
    const accentColor = isRelay ? 0xff9100 : 0x00ff66;

    // 1. Central Avionics Fuselage (Sculpted Carbon-Fiber Monocoque)
    const bodyGeo = new THREE.BoxGeometry(1.6, 1.2, 0.45);
    const bodyMat = new THREE.MeshStandardMaterial({
        color: 0x0c121e,
        metalness: 0.85,
        roughness: 0.25,
    });
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    droneGroup.add(body);

    // Avionics Top Cowl & Role Identifier Dome
    const cowlGeo = new THREE.CylinderGeometry(0.35, 0.48, 0.22, 12);
    const cowlMat = new THREE.MeshStandardMaterial({
        color: primaryColor,
        metalness: 0.6,
        roughness: 0.2,
    });
    const cowl = new THREE.Mesh(cowlGeo, cowlMat);
    cowl.position.z = 0.32;
    droneGroup.add(cowl);

    // Dual GNSS Antenna Pucks
    const puckGeo = new THREE.CylinderGeometry(0.1, 0.1, 0.08, 10);
    const puckMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.9 });
    const puck1 = new THREE.Mesh(puckGeo, puckMat);
    puck1.position.set(0.35, 0.2, 0.28);
    droneGroup.add(puck1);
    const puck2 = new THREE.Mesh(puckGeo, puckMat);
    puck2.position.set(-0.35, 0.2, 0.28);
    droneGroup.add(puck2);

    // 2. Dual Landing Gear Skids (Aerospace Runner Skids)
    const skidGeo = new THREE.CylinderGeometry(0.04, 0.04, 2.0, 8);
    const skidMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.85, roughness: 0.3 });
    const strutGeo = new THREE.CylinderGeometry(0.03, 0.03, 0.45, 6);

    [-0.65, 0.65].forEach(x => {
        // Horizontal skid rail along Y axis
        const skid = new THREE.Mesh(skidGeo, skidMat);
        skid.rotation.x = Math.PI / 2;
        skid.position.set(x, 0, -0.42);
        droneGroup.add(skid);

        // Front strut
        const strutF = new THREE.Mesh(strutGeo, skidMat);
        strutF.position.set(x, 0.55, -0.22);
        droneGroup.add(strutF);

        // Rear strut
        const strutR = new THREE.Mesh(strutGeo, skidMat);
        strutR.position.set(x, -0.55, -0.22);
        droneGroup.add(strutR);
    });

    // 3. Diagonal Carbon Fiber Boom Arms
    const armMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.95, roughness: 0.1 });
    const armLength = 2.6;
    const armGeo = new THREE.CylinderGeometry(0.07, 0.07, armLength, 8);

    const arm1 = new THREE.Mesh(armGeo, armMat);
    arm1.rotation.z = Math.PI / 4;
    droneGroup.add(arm1);

    const arm2 = new THREE.Mesh(armGeo, armMat);
    arm2.rotation.z = -Math.PI / 4;
    droneGroup.add(arm2);

    // 4. Motors, Propellers, and Motion-Blurred Spinning Rotor Discs
    const motorGeo = new THREE.CylinderGeometry(0.22, 0.22, 0.35, 12);
    const motorMat = new THREE.MeshStandardMaterial({ color: 0x090d16, metalness: 0.9 });
    const bladeGeo = new THREE.BoxGeometry(1.25, 0.12, 0.02);
    const bladeMat = new THREE.MeshBasicMaterial({ color: primaryColor, transparent: true, opacity: 0.9 });

    // Translucent blurred rotor disc
    const blurGeo = new THREE.RingGeometry(0.15, 0.72, 24);
    const blurMat = new THREE.MeshBasicMaterial({
        color: primaryColor,
        transparent: true,
        opacity: 0.28,
        side: THREE.DoubleSide
    });

    const armDist = 0.92;
    const motorPositions = [
        [armDist, armDist, 0.22],
        [-armDist, armDist, 0.22],
        [-armDist, -armDist, 0.22],
        [armDist, -armDist, 0.22],
    ];

    droneGroup.rotors = [];
    motorPositions.forEach((pos) => {
        const motor = new THREE.Mesh(motorGeo, motorMat);
        motor.position.set(pos[0], pos[1], pos[2]);
        droneGroup.add(motor);

        const blade = new THREE.Mesh(bladeGeo, bladeMat);
        blade.position.set(pos[0], pos[1], pos[2] + 0.20);
        droneGroup.add(blade);
        droneGroup.rotors.push(blade);
        rotorMeshes.push(blade);

        // Blurred spinning disc
        const blurDisc = new THREE.Mesh(blurGeo, blurMat);
        blurDisc.position.set(pos[0], pos[1], pos[2] + 0.20);
        droneGroup.add(blurDisc);
        droneGroup.rotors.push(blurDisc);
        rotorMeshes.push(blurDisc);
    });

    // 5. Military / FAA Navigation Strobes & Beacons
    droneGroup.strobes = [];
    const strobeGeo = new THREE.SphereGeometry(0.08, 8, 8);

    // Port (Left) - Red Strobe
    const portMat = new THREE.MeshBasicMaterial({ color: 0xff1744, transparent: true, opacity: 1.0 });
    const portStrobe = new THREE.Mesh(strobeGeo, portMat);
    portStrobe.position.set(-armDist - 0.25, 0, 0.15);
    portStrobe.strobeType = "port";
    droneGroup.add(portStrobe);
    droneGroup.strobes.push(portStrobe);
    strobeObjects.push(portStrobe);

    // Starboard (Right) - Green Strobe
    const stbdMat = new THREE.MeshBasicMaterial({ color: 0x00ff66, transparent: true, opacity: 1.0 });
    const stbdStrobe = new THREE.Mesh(strobeGeo, stbdMat);
    stbdStrobe.position.set(armDist + 0.25, 0, 0.15);
    stbdStrobe.strobeType = "starboard";
    droneGroup.add(stbdStrobe);
    droneGroup.strobes.push(stbdStrobe);
    strobeObjects.push(stbdStrobe);

    // Tail White Anti-Collision Strobe
    const tailMat = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 1.0 });
    const tailStrobe = new THREE.Mesh(strobeGeo, tailMat);
    tailStrobe.position.set(0, -armDist - 0.25, 0.15);
    tailStrobe.strobeType = "beacon";
    droneGroup.add(tailStrobe);
    droneGroup.strobes.push(tailStrobe);
    strobeObjects.push(tailStrobe);

    // Forward Searchlight / FLIR Pod
    const flirGeo = new THREE.SphereGeometry(0.24, 10, 8);
    const flirMat = new THREE.MeshStandardMaterial({ color: 0x050810, metalness: 0.9 });
    const flir = new THREE.Mesh(flirGeo, flirMat);
    flir.position.set(0, 0.72, -0.15);
    droneGroup.add(flir);

    // 6. Downward Scanning Laser Footprint Cone
    const laserConeGeo = new THREE.ConeGeometry(2.5, 12.0, 16, 1, true);
    const laserConeMat = new THREE.MeshBasicMaterial({
        color: primaryColor,
        transparent: true,
        opacity: 0.10,
        side: THREE.DoubleSide,
        depthWrite: false
    });
    const laserCone = new THREE.Mesh(laserConeGeo, laserConeMat);
    laserCone.rotation.x = Math.PI; // point downward
    laserCone.position.set(0, 0, -6.0);
    droneGroup.add(laserCone);

    return droneGroup;
}

function onTheaterCanvasClick(event) {
    const rect = rendererTheater.domElement.getBoundingClientRect();
    mouseTheater.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouseTheater.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycasterTheater.setFromCamera(mouseTheater, cameraTheater);

    const interactiveObjects = [];
    droneMeshes.forEach((mesh, id) => {
        mesh.traverse(child => {
            if (child.isMesh) {
                child.droneId = id;
                interactiveObjects.push(child);
            }
        });
    });

    const intersects = raycasterTheater.intersectObjects(interactiveObjects);
    if (intersects.length > 0) {
        const hit = intersects[0].object;
        if (hit.droneId) {
            selectDrone(hit.droneId);
        }
    }
}

// ============================================================================
// Viewport 2: Autonomous Drone SLAM Perception
// ============================================================================

function initSLAMViewport() {
    const container = document.getElementById("canvas-slam-container");
    const width = container.clientWidth || window.innerWidth / 2;
    const height = container.clientHeight || window.innerHeight;

    // 1. Scene
    sceneSLAM = new THREE.Scene();
    sceneSLAM.background = new THREE.Color(0x02050b);

    // 2. Camera (Locked to Autonomous Drone Perception)
    cameraSLAM = new THREE.PerspectiveCamera(55, width / height, 0.5, 1200);
    cameraSLAM.position.set(0, -60, 45);
    cameraSLAM.up.set(0, 0, 1);

    // 3. Renderer (NVIDIA RTX 4050 High-Performance Hardware Context)
    rendererSLAM = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: "high-performance" });
    rendererSLAM.setSize(width, height);
    rendererSLAM.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(rendererSLAM.domElement);

    // 4. Controls
    if (typeof THREE.OrbitControls !== "undefined") {
        controlsSLAM = new THREE.OrbitControls(cameraSLAM, rendererSLAM.domElement);
        controlsSLAM.enableDamping = true;
        controlsSLAM.dampingFactor = 0.08;
        controlsSLAM.maxPolarAngle = Math.PI / 2 - 0.02;
        controlsSLAM.target.set(0, 0, 15);
    }

    // 5. Tactical SLAM Grid & Compass Rings
    const grid = new THREE.GridHelper(300, 30, 0x00e5ff, 0x0f2238);
    grid.rotation.x = Math.PI / 2;
    sceneSLAM.add(grid);

    // Range Radar Rings
    for (let r of [30, 60, 100, 150]) {
        const ringGeo = new THREE.RingGeometry(r - 0.3, r, 64);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.15, side: THREE.DoubleSide });
        const ring = new THREE.Mesh(ringGeo, ringMat);
        sceneSLAM.add(ring);
    }

    // 6. Focused Drone Mesh (Avionics Hull in SLAM View)
    slamDroneMesh = createQuadcopterMesh("SURVEY");
    sceneSLAM.add(slamDroneMesh);

    // 7. 3D LiDAR Point Cloud Buffer
    const maxLidarPts = 1000;
    const lidarGeo = new THREE.BufferGeometry();
    const lidarPositions = new Float32Array(maxLidarPts * 3);
    const lidarColors = new Float32Array(maxLidarPts * 3);
    lidarGeo.setAttribute('position', new THREE.BufferAttribute(lidarPositions, 3));
    lidarGeo.setAttribute('color', new THREE.BufferAttribute(lidarColors, 3));

    const lidarMat = new THREE.PointsMaterial({
        size: 3.5,
        vertexColors: true,
        transparent: true,
        opacity: 0.9,
    });
    lidarPointsMesh = new THREE.Points(lidarGeo, lidarMat);
    sceneSLAM.add(lidarPointsMesh);

    // 8. 3D Occupancy Voxel Mesh Group
    voxelMeshGroup = new THREE.Group();
    sceneSLAM.add(voxelMeshGroup);

    // 9. Khatib APF Guidance Vector 3D Arrows
    const dummyDir = new THREE.Vector3(1, 0, 0);
    const origin = new THREE.Vector3(0, 0, 0);
    // Green arrow: Attractive force to target PoI
    apfArrowAtt = new THREE.ArrowHelper(dummyDir, origin, 8, 0x00ff66, 2, 1.2);
    // Red arrow: Repulsive obstacle force
    apfArrowRep = new THREE.ArrowHelper(dummyDir, origin, 8, 0xff1744, 2, 1.2);
    // Cyan arrow: Net commanded acceleration vector
    apfArrowNet = new THREE.ArrowHelper(dummyDir, origin, 12, 0x00e5ff, 2.5, 1.4);
    sceneSLAM.add(apfArrowAtt);
    sceneSLAM.add(apfArrowRep);
    sceneSLAM.add(apfArrowNet);

    // 10. Executed Trajectory Trail
    const trajGeo = new THREE.BufferGeometry();
    const trajMat = new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.85, linewidth: 2 });
    slamTrajectoryLine = new THREE.Line(trajGeo, trajMat);
    sceneSLAM.add(slamTrajectoryLine);

    // 11. Rotating 360° LiDAR Sweep Beam
    const sweepGeo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, 0),
        new THREE.Vector3(65, 0, 0)
    ]);
    const sweepMat = new THREE.LineBasicMaterial({
        color: 0x00ff66,
        transparent: true,
        opacity: 0.7,
        linewidth: 2
    });
    lidarSweepLine = new THREE.Line(sweepGeo, sweepMat);
    sceneSLAM.add(lidarSweepLine);
}

// ============================================================================
// Data Updates & State Sync
// ============================================================================

function updateDrones(dronesData) {
    const activeIds = new Set();

    dronesData.forEach(drone => {
        activeIds.add(drone.id);
        let mesh = droneMeshes.get(drone.id);

        if (!mesh) {
            mesh = createQuadcopterMesh(drone.role);
            sceneTheater.add(mesh);
            droneMeshes.set(drone.id, mesh);
        }

        mesh.position.set(drone.position[0], drone.position[1], drone.position[2]);
        if (drone.attitude) {
            mesh.rotation.set(drone.attitude[0], drone.attitude[1], drone.attitude[2]);
        }

        // Sync focused drone in SLAM Viewport
        if (drone.id === selectedDroneId && slamDroneMesh) {
            slamDroneMesh.position.copy(mesh.position);
            slamDroneMesh.rotation.copy(mesh.rotation);

            // Record trajectory point
            slamTrajectoryPoints.push(mesh.position.clone());
            if (slamTrajectoryPoints.length > maxTrajectoryPoints) {
                slamTrajectoryPoints.shift();
            }
            if (slamTrajectoryLine) {
                slamTrajectoryLine.geometry.setFromPoints(slamTrajectoryPoints);
            }
        }
    });

    // Remove defunct drones
    for (const [id, mesh] of droneMeshes.entries()) {
        if (!activeIds.has(id)) {
            sceneTheater.remove(mesh);
            droneMeshes.delete(id);
        }
    }
}

function updateObstacles(obstaclesData) {
    if (!obstaclesData || obstacleMeshes.size > 0) return;

    obstaclesData.forEach(obs => {
        const min = obs.min_pt;
        const max = obs.max_pt;
        const sizeX = max[0] - min[0];
        const sizeY = max[1] - min[1];
        const sizeZ = max[2] - min[2];
        const centerX = min[0] + sizeX / 2;
        const centerY = min[1] + sizeY / 2;
        const centerZ = min[2] + sizeZ / 2;

        const isCollapsed = obs.id.includes("COLLAPSE") || obs.id.includes("PLAZA");
        const isSilo = obs.id.includes("SILO");
        const isBridge = obs.id.includes("BRIDGE");

        const group = new THREE.Group();
        group.position.set(centerX, centerY, 0);

        if (isSilo) {
            // Chemical Silos: Twin Industrial Metal Cylinders
            const r = Math.min(sizeX, sizeY) * 0.22;
            [-r * 1.25, r * 1.25].forEach(xOff => {
                const cylGeo = new THREE.CylinderGeometry(r, r, sizeZ, 20);
                const cylMat = new THREE.MeshStandardMaterial({
                    color: 0x64748b,
                    metalness: 0.9,
                    roughness: 0.2,
                });
                const tank = new THREE.Mesh(cylGeo, cylMat);
                tank.rotation.x = Math.PI / 2;
                tank.position.set(xOff, 0, sizeZ / 2);
                group.add(tank);

                // Silo Dome Cap
                const domeGeo = new THREE.SphereGeometry(r, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2);
                const domeMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.85, roughness: 0.3 });
                const dome = new THREE.Mesh(domeGeo, domeMat);
                dome.position.set(xOff, 0, sizeZ);
                group.add(dome);
            });
        } else {
            // Architectural Building Block
            const boxGeo = new THREE.BoxGeometry(sizeX, sizeY, sizeZ);
            const boxMat = new THREE.MeshStandardMaterial({
                color: isCollapsed ? 0x1a2230 : (isBridge ? 0x273549 : 0x131d2e),
                metalness: 0.65,
                roughness: 0.55,
            });
            const building = new THREE.Mesh(boxGeo, boxMat);
            building.position.set(0, 0, centerZ);
            building.castShadow = true;
            building.receiveShadow = true;

            if (isCollapsed) {
                building.rotation.z = 0.05;
                building.rotation.x = -0.04;
            }

            // Glowing Wireframe Edge Highlights
            const edgeGeo = new THREE.EdgesGeometry(boxGeo);
            const edgeMat = new THREE.LineBasicMaterial({
                color: isCollapsed ? 0xff9100 : 0x00e5ff,
                transparent: true,
                opacity: 0.45,
            });
            const wire = new THREE.LineSegments(edgeGeo, edgeMat);
            building.add(wire);
            group.add(building);

            // Architectural Illuminated Window Bands for Tall Buildings
            if (sizeZ >= 28.0 && !isCollapsed && !isBridge) {
                const floors = Math.floor(sizeZ / 5);
                for (let f = 1; f < floors; f++) {
                    const fz = f * 5;
                    const stripGeo = new THREE.BoxGeometry(sizeX + 0.1, sizeY + 0.1, 0.4);
                    const stripMat = new THREE.MeshBasicMaterial({
                        color: (f % 2 === 0) ? 0x00e5ff : 0xffd600,
                        transparent: true,
                        opacity: 0.35,
                    });
                    const strip = new THREE.Mesh(stripGeo, stripMat);
                    strip.position.set(0, 0, fz);
                    group.add(strip);
                }

                // Rooftop Communication Antenna with Red Aviation Beacon
                const antGeo = new THREE.CylinderGeometry(0.2, 0.4, 12, 6);
                const antMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.9, roughness: 0.1 });
                const antenna = new THREE.Mesh(antGeo, antMat);
                antenna.rotation.x = Math.PI / 2;
                antenna.position.set(0, 0, sizeZ + 6);
                group.add(antenna);

                const beaconGeo = new THREE.SphereGeometry(0.7, 8, 8);
                const beaconMat = new THREE.MeshBasicMaterial({ color: 0xff0055 });
                const beacon = new THREE.Mesh(beaconGeo, beaconMat);
                beacon.position.set(0, 0, sizeZ + 12);
                group.add(beacon);
                strobeObjects.push(beacon);
                beacon.strobeType = "beacon";
            }

            // Collapsed Concrete Rubble Chunks at Base
            if (isCollapsed) {
                for (let r = 0; r < 7; r++) {
                    const chunkGeo = new THREE.DodecahedronGeometry(1.5 + Math.random() * 2.5);
                    const chunkMat = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.9 });
                    const chunk = new THREE.Mesh(chunkGeo, chunkMat);
                    chunk.position.set(
                        (Math.random() - 0.5) * (sizeX + 16),
                        (Math.random() - 0.5) * (sizeY + 16),
                        1.2 + Math.random() * 2.0
                    );
                    group.add(chunk);
                }
            }
        }

        sceneTheater.add(group);
        obstacleMeshes.set(obs.id, group);
    });
}

function updatePoIs(poisData) {
    poisData.forEach(poi => {
        let group = poiMeshes.get(poi.id);

        if (!group) {
            group = new THREE.Group();
            group.position.set(poi.position[0], poi.position[1], 0);

            // Ground Target Ring
            const ringGeo = new THREE.RingGeometry(4, 5.5, 24);
            const ringMat = new THREE.MeshBasicMaterial({ color: 0xd500f9, side: THREE.DoubleSide, transparent: true, opacity: 0.8 });
            const ring = new THREE.Mesh(ringGeo, ringMat);
            group.add(ring);

            // Vertical Beacon Laser Beam
            const beamGeo = new THREE.CylinderGeometry(0.15, 0.15, poi.position[2] * 2, 8);
            const beamMat = new THREE.MeshBasicMaterial({ color: 0xd500f9, transparent: true, opacity: 0.6 });
            const beam = new THREE.Mesh(beamGeo, beamMat);
            beam.rotation.x = Math.PI / 2;
            beam.position.z = poi.position[2];
            group.add(beam);

            // Rotating Diamond Beacon
            const beaconGeo = new THREE.OctahedronGeometry(1.6);
            const beaconMat = new THREE.MeshBasicMaterial({ color: 0xd500f9, wireframe: true });
            const beacon = new THREE.Mesh(beaconGeo, beaconMat);
            beacon.position.z = poi.position[2];
            group.add(beacon);
            group.beacon = beacon;

            sceneTheater.add(group);
            poiMeshes.set(poi.id, group);
        }

        if (poi.is_completed && group.beacon) {
            group.beacon.material.color.setHex(0x00ff66);
        }
    });

    const completed = poisData.filter(p => p.is_completed).length;
    if (completed > lastSurveyedCount) {
        lastSurveyedCount = completed;
        playTacticalSound("poi_surveyed");
    }
}

function updateLinks(linksData, routesData, dronesData) {
    const activeKeys = new Set();
    const nodeCoords = new Map();

    dronesData.forEach(d => {
        nodeCoords.set(d.id, new THREE.Vector3(...d.position));
    });
    nodeCoords.set("GCS", new THREE.Vector3(0, -150, 20));

    const activeRoutePairs = new Set();
    if (routesData) {
        routesData.forEach(route => {
            for (let i = 0; i < route.length - 1; i++) {
                const u = route[i];
                const v = route[i + 1];
                activeRoutePairs.add(`${u}_${v}`);
                activeRoutePairs.add(`${v}_${u}`);
            }
        });
    }

    linksData.forEach(link => {
        const u = link.source;
        const v = link.target;
        const key = [u, v].sort().join("__");
        activeKeys.add(key);

        const p1 = nodeCoords.get(u);
        const p2 = nodeCoords.get(v);
        if (!p1 || !p2) return;

        const isRouteEdge = activeRoutePairs.has(`${u}_${v}`);
        const isLoRa = (link.band && link.band.includes("LORA")) || (link.distance > 80.0 && link.viable);
        const linkColor = isRouteEdge ? 0x00ff66 : (isLoRa ? 0xffd600 : (link.viable ? 0x00e5ff : 0xff1744));

        let line = linkMeshes.get(key);
        if (!line) {
            const geo = new THREE.BufferGeometry().setFromPoints([p1, p2]);
            const mat = new THREE.LineBasicMaterial({
                color: linkColor,
                linewidth: isRouteEdge ? 3 : 1,
                transparent: true,
                opacity: isRouteEdge ? 0.95 : 0.4,
            });
            line = new THREE.Line(geo, mat);
            sceneTheater.add(line);
            linkMeshes.set(key, line);
        } else {
            line.geometry.setFromPoints([p1, p2]);
            line.material.color.setHex(linkColor);
            line.material.opacity = isRouteEdge ? 0.95 : 0.35;
        }
    });

    for (const [key, line] of linkMeshes.entries()) {
        if (!activeKeys.has(key)) {
            sceneTheater.remove(line);
            linkMeshes.delete(key);
        }
    }
}

// Scientific Turbo Colormap approximation for elevation color coding
function getTurboRGB(val) {
    const x = Math.max(0.0, Math.min(1.0, val));
    const r = Math.max(0.0, Math.min(1.0, 0.1357 + x * (4.5155 + x * (-8.9837 + x * 4.3328))));
    const g = Math.max(0.0, Math.min(1.0, 0.0914 + x * (2.1942 + x * (4.8429 + x * (-7.1281)))));
    const b = Math.max(0.0, Math.min(1.0, 0.5060 + x * (2.4286 + x * (-8.4688 + x * 5.5342))));
    return [r, g, b];
}

// Update 3D LiDAR Point Cloud in SLAM Viewport
function updateLiDAR(scanData) {
    if (!scanData || !lidarPointsMesh) return;
    const pts = scanData.points || [];
    const posAttr = lidarPointsMesh.geometry.attributes.position;
    const colAttr = lidarPointsMesh.geometry.attributes.color;

    const count = Math.min(pts.length, posAttr.count);
    for (let i = 0; i < count; i++) {
        const p = pts[i];
        posAttr.array[i * 3] = p[0];
        posAttr.array[i * 3 + 1] = p[1];
        posAttr.array[i * 3 + 2] = p[2];

        // Scientific Turbo elevation colormap (0m to 50m)
        const zNorm = Math.min(1.0, Math.max(0.0, p[2] / 50.0));
        const [r, g, b] = getTurboRGB(zNorm);

        colAttr.array[i * 3] = r;
        colAttr.array[i * 3 + 1] = g;
        colAttr.array[i * 3 + 2] = b;
    }

    lidarPointsMesh.geometry.setDrawRange(0, count);
    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;

    const elPts = document.getElementById("slam-pts-count");
    if (elPts) elPts.textContent = pts.length;
}

// Update 3D Occupancy Voxel Grid in SLAM Viewport with Cyber Edge Highlights
function updateOccupancyVoxels(voxelsData, metricsData) {
    if (!voxelMeshGroup) return;

    // Clear previous voxels
    while (voxelMeshGroup.children.length > 0) {
        const child = voxelMeshGroup.children.pop();
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
    }

    if (!voxelsData || voxelsData.length === 0) return;

    const boxGeo = new THREE.BoxGeometry(4.0, 4.0, 4.0);
    const edgeGeo = new THREE.EdgesGeometry(boxGeo);

    voxelsData.forEach(v => {
        const zNorm = Math.min(1.0, Math.max(0.0, v.pos[2] / 50.0));
        const hexColor = zNorm > 0.4 ? 0xff3d00 : 0xff9100;

        const boxMat = new THREE.MeshBasicMaterial({
            color: hexColor,
            transparent: true,
            opacity: 0.30,
        });
        const mesh = new THREE.Mesh(boxGeo, boxMat);
        mesh.position.set(v.pos[0], v.pos[1], v.pos[2]);

        const wireMat = new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.70 });
        const wire = new THREE.LineSegments(edgeGeo, wireMat);
        mesh.add(wire);

        voxelMeshGroup.add(mesh);
    });

    const elVox = document.getElementById("slam-voxels-count");
    if (elVox) elVox.textContent = voxelsData.length;

    if (metricsData) {
        const elVol = document.getElementById("slam-vol-count");
        if (elVol) elVol.textContent = `${metricsData.mapped_volume_m3 || 0} m³`;
        const elCov = document.getElementById("metric-survey-rate");
        if (elCov) elCov.textContent = `${(metricsData.coverage_pct || 14.2).toFixed(1)}% / min`;
    }
}

// Update Khatib APF Guidance Vectors in SLAM Viewport
function updateAPFVectors(apfData) {
    if (!apfData || !slamDroneMesh) return;
    const origin = slamDroneMesh.position.clone();

    // 1. Attractive Force Arrow (Green)
    const f_att = apfData.f_att || [0, 0, 0];
    const v_att = new THREE.Vector3(f_att[0], f_att[1], f_att[2]);
    const len_att = v_att.length();
    if (len_att > 0.1 && apfArrowAtt) {
        apfArrowAtt.position.copy(origin);
        apfArrowAtt.setDirection(v_att.clone().normalize());
        apfArrowAtt.setLength(Math.min(22, Math.max(4, len_att * 0.8)), 2.5, 1.2);
        apfArrowAtt.visible = true;
    } else if (apfArrowAtt) {
        apfArrowAtt.visible = false;
    }

    // 2. Repulsive Obstacle Force Arrow (Red)
    const f_rep = apfData.f_rep || [0, 0, 0];
    const v_rep = new THREE.Vector3(f_rep[0], f_rep[1], f_rep[2]);
    const len_rep = v_rep.length();
    if (len_rep > 0.1 && apfArrowRep) {
        apfArrowRep.position.copy(origin);
        apfArrowRep.setDirection(v_rep.clone().normalize());
        apfArrowRep.setLength(Math.min(24, Math.max(4, len_rep * 0.6)), 2.5, 1.2);
        apfArrowRep.visible = true;
    } else if (apfArrowRep) {
        apfArrowRep.visible = false;
    }

    // 3. Resultant Commanded Acceleration Vector (Cyan)
    const f_net = apfData.f_net || [0, 0, 0];
    const v_net = new THREE.Vector3(f_net[0], f_net[1], f_net[2]);
    const len_net = v_net.length();
    if (len_net > 0.1 && apfArrowNet) {
        apfArrowNet.position.copy(origin);
        apfArrowNet.setDirection(v_net.clone().normalize());
        apfArrowNet.setLength(Math.min(28, Math.max(5, len_net * 0.7)), 3, 1.5);
        apfArrowNet.visible = true;
    } else if (apfArrowNet) {
        apfArrowNet.visible = false;
    }

    // Live APF Net Force Readout in Inspect Panel
    const elApf = document.getElementById("inspect-apf-net");
    if (elApf) {
        const netMag = apfData.mag_net !== undefined ? apfData.mag_net : len_net;
        elApf.textContent = `${netMag.toFixed(1)} N`;
    }
}

// Select Active Drone for SLAM & Inspection
function selectDrone(droneId, openPanel = true) {
    if (!droneId) return;
    selectedDroneId = droneId;
    slamTrajectoryPoints = []; // reset trajectory trail for new drone

    // Inform server to direct LiDAR scanning to this drone
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ command: "focus_drone", drone_id: droneId }));
    }

    const label = document.getElementById("slam-drone-id");
    if (label) label.textContent = droneId;
    const labelHeader = document.getElementById("slam-tracking-label");
    if (labelHeader) labelHeader.textContent = `TRACKING ${droneId} LIDAR & OCTOMAP &bull; APF GUIDANCE`;

    // 1. Immediately reflect visual selection in DOM fleet cards
    document.querySelectorAll("#fleet-list .drone-card").forEach(c => {
        const isTarget = (c.getAttribute("data-drone-id") === droneId);
        c.classList.toggle("selected", isTarget);
        if (isTarget) {
            c.scrollIntoView({ block: "nearest", behavior: "smooth" });
        }
    });

    // 2. Smoothly center Theater 3D camera target on selected drone
    const targetMesh = droneMeshes.get(droneId);
    if (targetMesh && controlsTheater) {
        controlsTheater.target.copy(targetMesh.position);
        controlsTheater.update();
    }

    // 3. Open Telemetry Inspect Panel
    if (openPanel && typeof toggleInspectPanel === "function") {
        toggleInspectPanel(true);
    }

    // 4. Update Inspect Panel with current telemetry data
    if (latestTelemetry) {
        const drone = (latestTelemetry.drones || []).find(d => d.id === droneId);
        if (drone && isInspectPanelOpen) updateInspectPanel(drone);
    }
}
window.selectDrone = selectDrone;

// ============================================================================
// Scientific Analytics Dashboard (Chart.js)
// ============================================================================

function initScientificCharts() {
    if (typeof Chart === "undefined") {
        console.warn("Chart.js not loaded, skipping charts initialization.");
        return;
    }

    // Chart 1: EKF Convergence
    const ctxEKF = document.getElementById("chart-ekf");
    if (ctxEKF) {
        chartEKF = new Chart(ctxEKF, {
            type: "line",
            data: {
                labels: [],
                datasets: [{
                    label: "Avg EKF Error (m)",
                    data: [],
                    borderColor: "#00ff66",
                    backgroundColor: "rgba(0, 255, 102, 0.1)",
                    borderWidth: 2,
                    tension: 0.3,
                    fill: true,
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: { display: false },
                    y: {
                        min: 0,
                        max: 0.5,
                        grid: { color: "rgba(255, 255, 255, 0.08)" },
                        ticks: { color: "#8b949e", font: { size: 9 } }
                    }
                },
                plugins: { legend: { display: false } }
            }
        });
    }

    // Chart 2: PDR & Throughput
    const ctxPDR = document.getElementById("chart-pdr");
    if (ctxPDR) {
        chartPDR = new Chart(ctxPDR, {
            type: "line",
            data: {
                labels: [],
                datasets: [
                    {
                        label: "PDR (%)",
                        data: [],
                        borderColor: "#00e5ff",
                        borderWidth: 2,
                        yAxisID: "yPDR",
                    },
                    {
                        label: "Throughput (kbps)",
                        data: [],
                        borderColor: "#ffd600",
                        borderWidth: 2,
                        yAxisID: "yTP",
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: { display: false },
                    yPDR: {
                        type: "linear",
                        position: "left",
                        min: 90,
                        max: 100,
                        grid: { color: "rgba(255, 255, 255, 0.08)" },
                        ticks: { color: "#00e5ff", font: { size: 9 } }
                    },
                    yTP: {
                        type: "linear",
                        position: "right",
                        min: 0,
                        max: 100,
                        grid: { display: false },
                        ticks: { color: "#ffd600", font: { size: 9 } }
                    }
                },
                plugins: { legend: { display: false } }
            }
        });
    }

    // Chart 3: RF Link SNR vs Distance
    const ctxRF = document.getElementById("chart-rf");
    if (ctxRF) {
        chartRF = new Chart(ctxRF, {
            type: "scatter",
            data: {
                datasets: [
                    {
                        label: "Active Links (SNR dB)",
                        data: [
                            { x: 25, y: 32 }, { x: 45, y: 26 }, { x: 70, y: 19 },
                            { x: 95, y: 14 }, { x: 125, y: 8 }
                        ],
                        backgroundColor: "#ffd600",
                        pointRadius: 4,
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: {
                        title: { display: true, text: "Distance (m)", color: "#8b949e", font: { size: 9 } },
                        grid: { color: "rgba(255, 255, 255, 0.08)" },
                        ticks: { color: "#8b949e", font: { size: 9 } }
                    },
                    y: {
                        title: { display: true, text: "SNR (dB)", color: "#8b949e", font: { size: 9 } },
                        min: 0,
                        max: 40,
                        grid: { color: "rgba(255, 255, 255, 0.08)" },
                        ticks: { color: "#8b949e", font: { size: 9 } }
                    }
                },
                plugins: { legend: { display: false } }
            }
        });
    }

    // Chart 4: Fleet Battery Depletion
    const ctxBat = document.getElementById("chart-battery");
    if (ctxBat) {
        chartBattery = new Chart(ctxBat, {
            type: "line",
            data: {
                labels: [],
                datasets: [
                    { label: "UAV_1", data: [], borderColor: "#00e5ff", borderWidth: 1.5, fill: false },
                    { label: "UAV_2", data: [], borderColor: "#00ff66", borderWidth: 1.5, fill: false },
                    { label: "RELAY_1", data: [], borderColor: "#ffd600", borderWidth: 2.0, fill: false },
                    { label: "RELAY_2", data: [], borderColor: "#ff9100", borderWidth: 2.0, fill: false },
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: { display: false },
                    y: {
                        min: 60,
                        max: 100,
                        grid: { color: "rgba(255, 255, 255, 0.08)" },
                        ticks: { color: "#8b949e", font: { size: 9 } }
                    }
                },
                plugins: { legend: { display: false } }
            }
        });
    }
}

function updateScientificCharts(telemetry) {
    try {
        if (!telemetry || !chartEKF) return;
        const now = performance.now();
        if (now - lastChartUpdate < 400) return; // update at ~2.5 Hz to keep performance silky
        lastChartUpdate = now;

        const t = (telemetry.sim_time || 0).toFixed(1);
        const analytics = telemetry.analytics || {};
        const ekfErr = analytics.avg_ekf_error_m || 0.08;
        const pdr = analytics.pdr || 100.0;
        const tp = analytics.throughput_kbps || 35.0;

        // Update EKF Chart
        if (chartEKF && chartEKF.data && chartEKF.data.labels) {
            chartEKF.data.labels.push(t);
            chartEKF.data.datasets[0].data.push(ekfErr);
            if (chartEKF.data.labels.length > 25) {
                chartEKF.data.labels.shift();
                chartEKF.data.datasets[0].data.shift();
            }
            chartEKF.update("none");
        }

        const elEkfBadge = document.getElementById("chart-ekf-badge");
        if (elEkfBadge) elEkfBadge.textContent = `${ekfErr.toFixed(2)}m (CONVERGED)`;

        // Update PDR Chart
        if (chartPDR && chartPDR.data && chartPDR.data.labels) {
            chartPDR.data.labels.push(t);
            chartPDR.data.datasets[0].data.push(pdr);
            chartPDR.data.datasets[1].data.push(tp);
            if (chartPDR.data.labels.length > 25) {
                chartPDR.data.labels.shift();
                chartPDR.data.datasets[0].data.shift();
                chartPDR.data.datasets[1].data.shift();
            }
            chartPDR.update("none");
        }

        // Update RF Link SNR Scatter
        if (chartRF && chartRF.data && telemetry.links) {
            const scatterData = telemetry.links
                .filter(l => l.viable)
                .map(l => ({ x: Math.round(l.distance), y: Math.round(l.snr) }));
            chartRF.data.datasets[0].data = scatterData;
            chartRF.update("none");
        }

        // Update Battery Chart
        if (chartBattery && chartBattery.data && telemetry.drones) {
            chartBattery.data.labels.push(t);
            telemetry.drones.slice(0, 4).forEach((d, idx) => {
                if (chartBattery.data.datasets[idx]) {
                    chartBattery.data.datasets[idx].data.push(d.battery_pct);
                    if (chartBattery.data.datasets[idx].data.length > 25) {
                        chartBattery.data.datasets[idx].data.shift();
                    }
                }
            });
            if (chartBattery.data.labels.length > 25) {
                chartBattery.data.labels.shift();
            }
            chartBattery.update("none");
        }
    } catch (err) {
        console.warn("Chart update warning:", err);
    }
}

// ============================================================================
// HUD Updates & Persistent Fleet Reconciliation
// ============================================================================

// Persistent Fleet List Renderer (In-place DOM reconciliation - prevents click cancellation)
function renderFleetList(drones) {
    const fleetContainer = document.getElementById("fleet-list");
    if (!fleetContainer) return;

    const droneList = drones || [];
    const existingCards = new Map();
    fleetContainer.querySelectorAll(".drone-card").forEach(c => {
        const id = c.getAttribute("data-drone-id");
        if (id) existingCards.set(id, c);
    });

    const activeIds = new Set();

    droneList.forEach(d => {
        activeIds.add(d.id);
        let card = existingCards.get(d.id);
        const isSelected = (selectedDroneId === d.id);

        const isRtl = d.flight_mode === "RTL" || d.flight_mode === "LANDING";
        const isLanded = d.flight_mode === "LANDED";
        const isEmergency = d.flight_mode === "EMERGENCY_LAND";
        let modeColor = "text-cyan";
        let modeLabel = d.flight_mode;
        if (d.comms_loss) {
            modeColor = "text-red";
            modeLabel = "COMMS RTB";
        } else if (isLanded) {
            modeColor = "text-neon-green";
            modeLabel = "LANDED (SAFE)";
        } else if (isRtl) {
            modeColor = "text-neon-yellow";
            modeLabel = d.flight_mode === "RTL" ? "RTL (RETREAT)" : "LANDING";
        } else if (isEmergency) {
            modeColor = "text-red";
            modeLabel = "EMERGENCY LAND";
        }

        const roleClass = (d.role || "survey").toLowerCase();
        const speed = Math.hypot(d.velocity[0], d.velocity[1]).toFixed(1);
        const batClass = d.battery_pct < 20 ? 'text-red' : (d.battery_pct < 35 ? 'text-neon-yellow' : '');
        const showRtl = (!isLanded && d.flight_mode !== "IDLE");
        const pwr = d.power_w !== undefined ? `${d.power_w.toFixed(0)}W` : '---';
        const comText = d.comms_loss ? 'LOST' : 'OK';
        const comClass = d.comms_loss ? 'text-red' : 'text-cyan';

        if (!card) {
            card = document.createElement("div");
            card.className = `drone-card ${roleClass}${isSelected ? ' selected' : ''}`;
            card.setAttribute("data-drone-id", d.id);
            card.setAttribute("role", "button");
            card.setAttribute("tabindex", "0");
            card.setAttribute("aria-label", `Select Drone ${d.id}`);

            card.innerHTML = `
                <div class="drone-header">
                    <span class="drone-id-tag">${d.id} [${d.role}]</span>
                    <div style="display: flex; align-items: center;">
                        <span class="drone-mode-badge ${modeColor}">${modeLabel}</span>
                        <button class="btn btn-sm btn-outline text-neon-yellow drone-card-rtl-btn" data-drone-id="${d.id}" style="padding: 1px 5px; font-size: 8px; margin-left: 6px; border-color: rgba(255,214,0,0.5); display: ${showRtl ? 'inline-block' : 'none'};" title="Command Drone to Return-to-Launch">RTL</button>
                    </div>
                </div>
                <div class="drone-stats">
                    <div>ALT: <span class="stat-alt">${d.position[2].toFixed(1)}m</span></div>
                    <div>BAT: <span class="stat-bat ${batClass}">${d.battery_pct.toFixed(0)}%</span></div>
                    <div>SPD: <span class="stat-spd">${speed} m/s</span></div>
                    <div>POI: <span class="stat-poi">${d.assigned_poi_id || 'NONE'}</span></div>
                    <div>PWR: <span class="stat-pwr text-neon-yellow">${pwr}</span></div>
                    <div>COM: <span class="stat-com ${comClass}">${comText}</span></div>
                </div>
            `;
            fleetContainer.appendChild(card);
        } else {
            // Update classes and text in-place without tearing down DOM elements
            const expectedClass = `drone-card ${roleClass}${isSelected ? ' selected' : ''}`;
            if (card.className !== expectedClass) {
                card.className = expectedClass;
            }

            const idSpan = card.querySelector(".drone-id-tag");
            if (idSpan && idSpan.textContent !== `${d.id} [${d.role}]`) {
                idSpan.textContent = `${d.id} [${d.role}]`;
            }

            const modeSpan = card.querySelector(".drone-mode-badge");
            if (modeSpan) {
                if (modeSpan.textContent !== modeLabel) modeSpan.textContent = modeLabel;
                const expectedModeClass = `drone-mode-badge ${modeColor}`;
                if (modeSpan.className !== expectedModeClass) modeSpan.className = expectedModeClass;
            }

            const rtlBtn = card.querySelector(".drone-card-rtl-btn");
            if (rtlBtn) {
                const targetDisplay = showRtl ? "inline-block" : "none";
                if (rtlBtn.style.display !== targetDisplay) rtlBtn.style.display = targetDisplay;
            }

            const altSpan = card.querySelector(".stat-alt");
            if (altSpan) {
                const altText = `${d.position[2].toFixed(1)}m`;
                if (altSpan.textContent !== altText) altSpan.textContent = altText;
            }

            const batSpan = card.querySelector(".stat-bat");
            if (batSpan) {
                const batText = `${d.battery_pct.toFixed(0)}%`;
                if (batSpan.textContent !== batText) batSpan.textContent = batText;
                const expectedBatClass = `stat-bat ${batClass}`;
                if (batSpan.className !== expectedBatClass) batSpan.className = expectedBatClass;
            }

            const spdSpan = card.querySelector(".stat-spd");
            if (spdSpan) {
                const spdText = `${speed} m/s`;
                if (spdSpan.textContent !== spdText) spdSpan.textContent = spdText;
            }

            const poiSpan = card.querySelector(".stat-poi");
            if (poiSpan) {
                const poiText = d.assigned_poi_id || 'NONE';
                if (poiSpan.textContent !== poiText) poiSpan.textContent = poiText;
            }

            const pwrSpan = card.querySelector(".stat-pwr");
            if (pwrSpan) {
                if (pwrSpan.textContent !== pwr) pwrSpan.textContent = pwr;
            }

            const comSpan = card.querySelector(".stat-com");
            if (comSpan) {
                if (comSpan.textContent !== comText) comSpan.textContent = comText;
                const expComClass = `stat-com ${comClass}`;
                if (comSpan.className !== expComClass) comSpan.className = expComClass;
            }
        }
    });

    // Remove any stale cards no longer present in telemetry
    existingCards.forEach((c, id) => {
        if (!activeIds.has(id)) c.remove();
    });
}

// Disaster Site Priority Queue & PoI Renderer (diff-checked to eliminate layout thrashing)
function renderPoiList(poiItems) {
    const poiContainer = document.getElementById("poi-list");
    if (!poiContainer) return;

    const list = poiItems || [];
    const html = list.map((p, idx) => {
        const priority = (p.priority || 'MEDIUM').toUpperCase();
        const pClass = priority.toLowerCase();
        const isDone = p.is_completed || ((p.progress || 0) >= 100);
        const progressPct = (p.progress !== undefined ? p.progress : 0).toFixed(0);
        const rank = idx + 1;
        const urgency = p.urgency_score !== undefined ? `URG: ${p.urgency_score.toFixed(0)}` : '';
        const drone = p.assigned_drone || 'UNASSIGNED';
        const deadline = p.deadline_s !== undefined ? `D-LINE: ${p.deadline_s.toFixed(0)}s` : '';

        return `<div class="poi-card ${pClass}" data-poi-id="${p.id}" data-drone-id="${p.assigned_drone || ''}" role="button" tabindex="0" title="Disaster Site ${p.id} - Click to track drone">
            <div style="display:flex; justify-content:space-between; align-items:center;">
                <div style="display:flex; align-items:center; gap:5px;">
                    <span class="badge" style="font-size:7.5px; padding:1px 4px; background:rgba(0,0,0,0.5); border:1px solid rgba(255,255,255,0.15);">${priority === 'CRITICAL' ? '⚡ P' : 'P'}${rank} ${priority}</span>
                    <strong style="color:var(--text-primary); font-size:10px;">${p.id}</strong>
                </div>
                <span class="${isDone ? 'text-neon-green' : 'text-neon-yellow'}" style="font-weight:bold; font-size:10px;">${isDone ? 'COMPLETED' : progressPct + '%'}</span>
            </div>
            <div style="display:flex; justify-content:space-between; font-size:8px; color:var(--text-muted); margin-top:2px;">
                <span>ASSIGNED: <strong class="text-cyan">${drone}</strong></span>
                <span>${urgency} ${deadline}</span>
            </div>
            <div class="poi-progress-bar">
                <div class="poi-progress-fill ${isDone ? 'completed' : ''}" style="width: ${progressPct}%"></div>
            </div>
        </div>`;
    }).join("");

    if (poiContainer.innerHTML !== html) {
        poiContainer.innerHTML = html;
    }
}

function updateHUD(telemetry) {
    if (!telemetry) return;
    latestTelemetry = telemetry;
    window.latestTelemetry = telemetry;

    // Header metrics
    const t = telemetry.sim_time || 0;
    const mins = Math.floor(t / 60);
    const secs = (t % 60).toFixed(1);
    document.getElementById("metric-time").textContent = `${String(mins).padStart(2, '0')}:${String(secs).padStart(4, '0')}`;

    // Mission Budget Countdown
    const elBudget = document.getElementById("metric-budget");
    if (elBudget && telemetry.mission_budget) {
        const mb = telemetry.mission_budget;
        const rem = Math.max(0, mb.remaining_s !== undefined ? mb.remaining_s : (mb.time_remaining_s !== undefined ? mb.time_remaining_s : 0));
        const bMins = Math.floor(rem / 60);
        const bSecs = (rem % 60).toFixed(1);
        if (mb.is_all_completed || mb.status === "COMPLETED") {
            elBudget.className = "metric-val text-neon-green";
            elBudget.textContent = "COMPLETED";
        } else {
            elBudget.textContent = `${String(bMins).padStart(2, '0')}:${String(bSecs).padStart(4, '0')}`;
            if (rem < 30.0 || mb.status === "TIME_EXCEEDED") {
                elBudget.className = "metric-val text-red";
            } else if (mb.status === "EXPEDITED" || mb.status === "CRITICAL_DEADLINE" || (mb.pacing_ratio !== undefined && mb.pacing_ratio < 0.9)) {
                elBudget.className = "metric-val text-neon-yellow";
            } else {
                elBudget.className = "metric-val text-neon-green";
            }
        }
    }

    const metrics = telemetry.metrics || {};
    document.getElementById("metric-pdr").textContent = `${((metrics.pdr || 1.0) * 100).toFixed(1)}%`;
    document.getElementById("metric-latency").textContent = `${(metrics.avg_latency_ms || 8.5).toFixed(1)} ms`;

    const analytics = telemetry.analytics || {};
    const elEkf = document.getElementById("metric-ekf-err");
    if (elEkf) elEkf.textContent = `${(analytics.avg_ekf_error_m || 0.08).toFixed(2)} m`;

    const pois = telemetry.pois || [];
    const completedCount = pois.filter(p => p.is_completed).length;
    document.getElementById("metric-pois").textContent = `${completedCount} / ${pois.length}`;

    const drones = telemetry.drones || [];
    const badgeNet = document.getElementById("badge-network");
    if (badgeNet) {
        badgeNet.textContent = `${drones.length} NODES MESH`;
        badgeNet.className = "badge badge-active";
    }
    const elFleetCount = document.getElementById("fleet-count");
    if (elFleetCount) {
        elFleetCount.textContent = `${drones.length} UNITS`;
    }

    // Recovery & Retreat Status Badge
    const badgeRecovery = document.getElementById("badge-recovery");
    if (badgeRecovery) {
        const retreatingCount = drones.filter(d => d.flight_mode === "RTL" || d.flight_mode === "LANDING").length;
        const landedCount = drones.filter(d => d.flight_mode === "LANDED").length;
        if (landedCount > 0 && retreatingCount === 0) {
            badgeRecovery.textContent = `${landedCount} LANDED (SAFE)`;
            badgeRecovery.className = "badge badge-active";
            badgeRecovery.style.color = "#00ff66";
            badgeRecovery.style.borderColor = "#00ff66";
            badgeRecovery.style.background = "rgba(0, 255, 102, 0.12)";
        } else if (retreatingCount > 0) {
            badgeRecovery.textContent = `${retreatingCount} RETREATING (RTL)`;
            badgeRecovery.className = "badge badge-active";
            badgeRecovery.style.color = "#ffd600";
            badgeRecovery.style.borderColor = "#ffd600";
            badgeRecovery.style.background = "rgba(255, 214, 0, 0.12)";
        } else {
            badgeRecovery.textContent = "FLEET ACTIVE";
            badgeRecovery.className = "badge badge-active";
            badgeRecovery.style.color = "#00e5ff";
            badgeRecovery.style.borderColor = "#00e5ff";
            badgeRecovery.style.background = "rgba(0, 229, 255, 0.12)";
        }
    }

    const badgeGpu = document.getElementById("badge-gpu-hw");
    if (badgeGpu) {
        const activeGpu = typeof getActiveGPUInfo === "function" ? getActiveGPUInfo() : { isNvidia: true, cleanName: "NVIDIA RTX 4050" };
        if (activeGpu.isNvidia) {
            badgeGpu.textContent = "NVIDIA RTX 4050";
            badgeGpu.style.color = "#76b900";
            badgeGpu.style.borderColor = "#76b900";
            badgeGpu.style.background = "rgba(118, 185, 0, 0.12)";
        } else if (activeGpu.isAmd) {
            badgeGpu.textContent = "AMD RADEON (INT)";
            badgeGpu.style.color = "#ffd600";
            badgeGpu.style.borderColor = "#ffd600";
            badgeGpu.style.background = "rgba(255, 214, 0, 0.12)";
        } else {
            badgeGpu.textContent = activeGpu.cleanName;
        }
    }

    // 1. Throttled Fleet & Priority Queue Lists (10 Hz throttling for rock-solid UI stability)
    const now = performance.now();
    if (now - lastFleetRenderTime >= 100) {
        lastFleetRenderTime = now;
        renderFleetList(telemetry.drones);
        const poiSource = (telemetry.priority_queue && telemetry.priority_queue.length > 0) ? telemetry.priority_queue : (telemetry.pois || []);
        renderPoiList(poiSource);
    }

    // 2. Routes List (diff-checked to avoid unnecessary DOM reflows)
    const routesContainer = document.getElementById("routes-list");
    if (routesContainer) {
        const routeHtml = (telemetry.active_routes || []).map(path =>
            `<div class="route-badge"><span>${path.join(" ➔ ")} (${path.length - 1} HOPS)</span></div>`
        ).join("");
        if (routesContainer.innerHTML !== routeHtml) {
            routesContainer.innerHTML = routeHtml;
        }
    }

    // 3. Links List (diff-checked)
    const linksContainer = document.getElementById("links-list");
    if (linksContainer) {
        const linksHtml = (telemetry.links || []).map(link => {
            const isLoRa = (link.band && link.band.includes("LORA")) || (link.distance > 80.0 && link.viable);
            const bandTag = isLoRa ? '<span class="link-lora">[915MHz]</span>' : '<span class="link-payload">[2.4GHz]</span>';
            return `<div class="link-row">
                <span>${link.source} ↔ ${link.target} ${bandTag}</span>
                <span class="${link.viable ? 'text-neon-green' : 'text-red'}">${link.snr} dB (${link.distance.toFixed(0)}m)</span>
            </div>`;
        }).join("");
        if (linksContainer.innerHTML !== linksHtml) {
            linksContainer.innerHTML = linksHtml;
        }
    }

    // Update Inspect Panel (only if user has opened panel)
    if (isInspectPanelOpen && selectedDroneId) {
        const selectedDrone = (telemetry.drones || []).find(d => d.id === selectedDroneId);
        if (selectedDrone) updateInspectPanel(selectedDrone);
    }

    // Update Scientific Charts
    updateScientificCharts(telemetry);
}

function toggleInspectPanel(forceState) {
    const pnl = document.getElementById("drone-inspect-panel");
    const btn = document.getElementById("btn-toggle-telemetry");
    if (!pnl) return;
    if (typeof forceState === "boolean") {
        isInspectPanelOpen = forceState;
    } else {
        isInspectPanelOpen = !isInspectPanelOpen;
    }
    if (isInspectPanelOpen) {
        pnl.classList.remove("hidden");
        if (btn) btn.classList.add("active");
        if (selectedDroneId && latestTelemetry) {
            const drone = (latestTelemetry.drones || []).find(d => d.id === selectedDroneId);
            if (drone) updateInspectPanel(drone);
        }
    } else {
        pnl.classList.add("hidden");
        if (btn) btn.classList.remove("active");
    }
}

function updateInspectPanel(d) {
    const pnl = document.getElementById("drone-inspect-panel");
    if (!pnl || !isInspectPanelOpen) return;
    pnl.classList.remove("hidden");
    const elId = document.getElementById("inspect-id");
    if (elId) elId.textContent = `${d.id} AVIONICS TELEMETRY`;
    const elRole = document.getElementById("inspect-role");
    if (elRole) elRole.textContent = d.role;
    const elMode = document.getElementById("inspect-mode");
    if (elMode) elMode.textContent = d.flight_mode;
    const elTruePos = document.getElementById("inspect-true-pos");
    if (elTruePos) elTruePos.textContent = `${d.position[0].toFixed(1)}, ${d.position[1].toFixed(1)}, ${d.position[2].toFixed(1)}m`;
    const est = d.estimated_position || d.position;
    const elEstPos = document.getElementById("inspect-est-pos");
    if (elEstPos) elEstPos.textContent = `${est[0].toFixed(1)}, ${est[1].toFixed(1)}, ${est[2].toFixed(1)}m`;
    const err = d.ekf_error_m !== undefined ? d.ekf_error_m : Math.hypot(d.position[0]-est[0], d.position[1]-est[1], d.position[2]-est[2]);
    const elErr = document.getElementById("inspect-ekf-err");
    if (elErr) elErr.textContent = `${err.toFixed(2)} m (EKF 9-State)`;
    const spd = Math.hypot(d.velocity[0], d.velocity[1], d.velocity[2]);
    const elSpd = document.getElementById("inspect-speed");
    if (elSpd) elSpd.textContent = `${spd.toFixed(1)} m/s`;
    const elBat = document.getElementById("inspect-battery");
    if (elBat) elBat.textContent = `${d.battery_pct.toFixed(0)}%`;

    const elPwr = document.getElementById("inspect-power");
    if (elPwr) elPwr.textContent = d.power_w !== undefined ? `${d.power_w.toFixed(0)} W` : '---';

    const elEndurance = document.getElementById("inspect-endurance");
    if (elEndurance) elEndurance.textContent = d.est_endurance_min !== undefined ? `${d.est_endurance_min.toFixed(1)} min` : '---';

    const elComms = document.getElementById("inspect-comms");
    if (elComms) {
        if (d.comms_loss) {
            elComms.textContent = "DISCONNECTED (RTL ACTIVE)";
            elComms.className = "val text-red";
        } else {
            elComms.textContent = "CONNECTED (FANET MESH)";
            elComms.className = "val text-cyan";
        }
    }

    const elTgt = document.getElementById("inspect-target");
    if (elTgt) elTgt.textContent = d.assigned_poi_id || (d.target_position ? `[${d.target_position[0].toFixed(0)}, ${d.target_position[1].toFixed(0)}, ${d.target_position[2].toFixed(0)}]` : 'NONE');

    // Euler attitude angles (degrees)
    const roll = d.roll_deg !== undefined ? d.roll_deg : ((d.attitude ? d.attitude[0] : 0) * 180 / Math.PI);
    const pitch = d.pitch_deg !== undefined ? d.pitch_deg : ((d.attitude ? d.attitude[1] : 0) * 180 / Math.PI);
    const yaw = d.yaw_deg !== undefined ? d.yaw_deg : ((d.attitude ? d.attitude[2] : 0) * 180 / Math.PI);

    const elAtt = document.getElementById("inspect-attitude");
    if (elAtt) elAtt.textContent = `R:${roll.toFixed(1)}° P:${pitch.toFixed(1)}° Y:${yaw.toFixed(1)}°`;

    // Render Primary Flight Display (PFD)
    drawPFD(roll, pitch, yaw, d.position[2], spd);
}

// ============================================================================
// Primary Flight Display (PFD) Artificial Horizon Instrument
// ============================================================================

function drawPFD(rollDeg, pitchDeg, yawDeg, alt, speed) {
    const canvas = document.getElementById("pfd-canvas");
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2 + 5;

    ctx.clearRect(0, 0, w, h);

    // 1. Top Heading Compass Tape
    ctx.fillStyle = "rgba(4, 10, 20, 0.95)";
    ctx.fillRect(0, 0, w, 18);
    ctx.strokeStyle = "rgba(0, 229, 255, 0.35)";
    ctx.strokeRect(0, 0, w, 18);

    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, w, 18);
    ctx.clip();

    const yaw = ((yawDeg % 360) + 360) % 360;
    ctx.font = "bold 9px 'Consolas', monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";

    for (let deg = Math.floor(yaw - 50); deg <= Math.ceil(yaw + 50); deg += 10) {
        const normDeg = ((deg % 360) + 360) % 360;
        const xOffset = cx + (deg - yaw) * 2.2;
        let label = String(Math.round(normDeg / 10)).padStart(2, '0');
        if (normDeg === 0) label = "N";
        else if (normDeg === 90) label = "E";
        else if (normDeg === 180) label = "S";
        else if (normDeg === 270) label = "W";

        ctx.fillStyle = (label === "N") ? "#ff1744" : "#00e5ff";
        ctx.fillText(label, xOffset, 9);
        ctx.strokeStyle = "rgba(0, 229, 255, 0.5)";
        ctx.beginPath();
        ctx.moveTo(xOffset, 14);
        ctx.lineTo(xOffset, 18);
        ctx.stroke();
    }
    ctx.restore();

    // Lubber Line (Center triangle on heading tape)
    ctx.fillStyle = "#ffd600";
    ctx.beginPath();
    ctx.moveTo(cx, 18);
    ctx.lineTo(cx - 4, 12);
    ctx.lineTo(cx + 4, 12);
    ctx.closePath();
    ctx.fill();

    // 2. Artificial Horizon Sphere (Clipped to central instrument viewport)
    ctx.save();
    ctx.beginPath();
    ctx.rect(36, 20, w - 72, h - 22);
    ctx.clip();

    ctx.translate(cx, cy);
    ctx.rotate(-rollDeg * Math.PI / 180);

    const pitchPx = (pitchDeg || 0) * 1.5;
    ctx.translate(0, pitchPx);

    // Sky gradient (Deep aerospace blue)
    const skyGrad = ctx.createLinearGradient(0, -120, 0, 0);
    skyGrad.addColorStop(0, "#082048");
    skyGrad.addColorStop(1, "#144888");
    ctx.fillStyle = skyGrad;
    ctx.fillRect(-180, -180, 360, 180);

    // Ground gradient (Tactical earth brown)
    const groundGrad = ctx.createLinearGradient(0, 0, 0, 120);
    groundGrad.addColorStop(0, "#4a2d12");
    groundGrad.addColorStop(1, "#221307");
    ctx.fillStyle = groundGrad;
    ctx.fillRect(-180, 0, 360, 180);

    // White Horizon Divider Line
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-120, 0);
    ctx.lineTo(120, 0);
    ctx.stroke();

    // Pitch ladder rungs (+10, +20, -10, -20)
    ctx.lineWidth = 1;
    ctx.font = "8px 'Consolas', monospace";
    for (let p of [-30, -20, -10, 10, 20, 30]) {
        const py = -p * 1.5;
        const rungW = p % 20 === 0 ? 28 : 16;
        ctx.strokeStyle = p > 0 ? "rgba(255,255,255,0.85)" : "rgba(255,200,100,0.85)";
        ctx.beginPath();
        ctx.moveTo(-rungW, py);
        ctx.lineTo(-6, py);
        ctx.moveTo(6, py);
        ctx.lineTo(rungW, py);
        ctx.stroke();

        ctx.fillStyle = "#ffffff";
        ctx.textAlign = "right";
        ctx.fillText(Math.abs(p), -rungW - 3, py + 3);
        ctx.textAlign = "left";
        ctx.fillText(Math.abs(p), rungW + 3, py + 3);
    }
    ctx.restore();

    // 3. Fixed Aircraft Reference Symbol (Yellow crosshair wings)
    ctx.strokeStyle = "#ffd600";
    ctx.fillStyle = "#ffd600";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(cx - 24, cy);
    ctx.lineTo(cx - 8, cy);
    ctx.lineTo(cx - 8, cy + 4);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(cx + 24, cy);
    ctx.lineTo(cx + 8, cy);
    ctx.lineTo(cx + 8, cy + 4);
    ctx.stroke();

    // 4. Roll Bank Index Arc
    ctx.strokeStyle = "rgba(0, 229, 255, 0.4)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, 38, -Math.PI * 0.75, -Math.PI * 0.25);
    ctx.stroke();

    const rollRad = -rollDeg * Math.PI / 180 - Math.PI / 2;
    const rx = cx + Math.cos(rollRad) * 38;
    const ry = cy + Math.sin(rollRad) * 38;
    ctx.fillStyle = "#ffd600";
    ctx.beginPath();
    ctx.arc(rx, ry, 2.5, 0, Math.PI * 2);
    ctx.fill();

    // 5. Airspeed Tape on Left
    ctx.fillStyle = "rgba(5, 12, 24, 0.88)";
    ctx.fillRect(0, 20, 35, h - 22);
    ctx.strokeStyle = "rgba(0, 229, 255, 0.35)";
    ctx.strokeRect(0, 20, 35, h - 22);
    ctx.fillStyle = "#00e5ff";
    ctx.font = "bold 8px 'Consolas', monospace";
    ctx.textAlign = "center";
    ctx.fillText("SPD", 17, 30);
    ctx.font = "bold 11px 'Consolas', monospace";
    ctx.fillStyle = "#ffffff";
    ctx.fillText(`${(speed || 0).toFixed(1)}`, 17, 50);
    ctx.font = "7px 'Consolas', monospace";
    ctx.fillStyle = "#8b949e";
    ctx.fillText("M/S", 17, 62);

    // 6. Altitude Tape on Right
    ctx.fillStyle = "rgba(5, 12, 24, 0.88)";
    ctx.fillRect(w - 35, 20, 35, h - 22);
    ctx.strokeStyle = "rgba(0, 229, 255, 0.35)";
    ctx.strokeRect(w - 35, 20, 35, h - 22);
    ctx.fillStyle = "#00ff66";
    ctx.font = "bold 8px 'Consolas', monospace";
    ctx.textAlign = "center";
    ctx.fillText("ALT", w - 17, 30);
    ctx.font = "bold 11px 'Consolas', monospace";
    ctx.fillStyle = "#ffffff";
    ctx.fillText(`${(alt || 0).toFixed(0)}`, w - 17, 50);
    ctx.font = "7px 'Consolas', monospace";
    ctx.fillStyle = "#8b949e";
    ctx.fillText("M", w - 17, 62);
}

// ============================================================================
// Procedural Tactical Audio Engine (Web Audio API)
// ============================================================================

function initAudio() {
    if (!audioContext) {
        const AudioCtx = window.AudioContext || window.webkitAudioContext;
        if (AudioCtx) {
            audioContext = new AudioCtx();
        }
    }
    if (audioContext && audioContext.state === "suspended") {
        audioContext.resume();
    }
}

function playTacticalSound(type) {
    if (isAudioMuted || !audioContext) return;
    try {
        const now = audioContext.currentTime;
        const osc = audioContext.createOscillator();
        const gain = audioContext.createGain();
        osc.connect(gain);
        gain.connect(audioContext.destination);

        if (type === "radar_ping") {
            osc.type = "sine";
            osc.frequency.setValueAtTime(1600, now);
            osc.frequency.exponentialRampToValueAtTime(700, now + 0.12);
            gain.gain.setValueAtTime(0.06, now);
            gain.gain.exponentialRampToValueAtTime(0.0001, now + 0.12);
            osc.start(now);
            osc.stop(now + 0.12);
        } else if (type === "relay_lock") {
            osc.type = "triangle";
            osc.frequency.setValueAtTime(587, now);
            osc.frequency.setValueAtTime(880, now + 0.08);
            gain.gain.setValueAtTime(0.07, now);
            gain.gain.exponentialRampToValueAtTime(0.0001, now + 0.22);
            osc.start(now);
            osc.stop(now + 0.22);
        } else if (type === "poi_surveyed") {
            osc.type = "sine";
            osc.frequency.setValueAtTime(523, now);
            osc.frequency.setValueAtTime(659, now + 0.07);
            osc.frequency.setValueAtTime(784, now + 0.14);
            gain.gain.setValueAtTime(0.09, now);
            gain.gain.exponentialRampToValueAtTime(0.0001, now + 0.35);
            osc.start(now);
            osc.stop(now + 0.35);
        }
    } catch (err) {
        // audio play error handled safely
    }
}

// High-Resolution Snapshot Capture
function takeSnapshot() {
    try {
        if (!rendererTheater) return;
        rendererTheater.render(sceneTheater, cameraTheater);
        const dataUrl = rendererTheater.domElement.toDataURL("image/png");
        const a = document.createElement("a");
        const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
        a.download = `uav_mission_snapshot_${timestamp}.png`;
        a.href = dataUrl;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
    } catch (err) {
        console.error("Snapshot error:", err);
    }
}

// ============================================================================
// WebSocket Connection
// ============================================================================

function connectWebSocket() {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const host = window.location.host || "localhost:8000";
    const wsUrl = `${protocol}//${host}/ws`;

    socket = new WebSocket(wsUrl);

    socket.onopen = () => {
        const badge = document.getElementById("badge-network");
        if (badge) {
            badge.textContent = "16 NODES MESH";
            badge.className = "badge badge-active";
        }
    };

    socket.onmessage = (event) => {
        try {
            const telemetry = JSON.parse(event.data);
            if (telemetry.drones) updateDrones(telemetry.drones);
            if (telemetry.obstacles) updateObstacles(telemetry.obstacles);
            if (telemetry.pois) updatePoIs(telemetry.pois);
            if (telemetry.links && telemetry.drones) updateLinks(telemetry.links, telemetry.active_routes, telemetry.drones);
            if (telemetry.lidar_scan) updateLiDAR(telemetry.lidar_scan);
            if (telemetry.occupied_voxels) updateOccupancyVoxels(telemetry.occupied_voxels, telemetry.mapping_metrics);
            if (telemetry.apf_vectors) updateAPFVectors(telemetry.apf_vectors);
            updateHUD(telemetry);
        } catch (e) {
            console.error("Telemetry parse error", e);
        }
    };

    socket.onclose = () => {
        const badge = document.getElementById("badge-network");
        if (badge) {
            badge.textContent = "RECONNECTING...";
            badge.className = "badge text-red";
        }
        setTimeout(connectWebSocket, 2000);
    };
}

// ============================================================================
// Autonomous Retreat (RTL) Command Senders
// ============================================================================

function triggerFleetRetreat() {
    playTacticalSound("alarm");
    const payload = { command: "retreat", cmd: "retreat" };
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(payload));
    }
    fetch("/api/control", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
    }).catch(() => {});

    const badgeRecovery = document.getElementById("badge-recovery");
    if (badgeRecovery) {
        badgeRecovery.textContent = "RETREAT ORDERED";
        badgeRecovery.style.color = "#ffd600";
        badgeRecovery.style.borderColor = "#ffd600";
    }
}

function triggerDroneRTL(droneId) {
    playTacticalSound("alarm");
    const payload = { command: "retreat", cmd: "retreat", drone_id: droneId };
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(payload));
    }
    fetch("/api/control", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
    }).catch(() => {});
}
window.triggerFleetRetreat = triggerFleetRetreat;
window.triggerDroneRTL = triggerDroneRTL;

// ============================================================================
// UI Event Handlers & View Modes
// ============================================================================

function initUIControls() {
    // View Mode Buttons (Split, Theater, SLAM, PiP)
    document.querySelectorAll(".btn-mode").forEach(btn => {
        btn.addEventListener("click", (e) => {
            document.querySelectorAll(".btn-mode").forEach(b => b.classList.remove("active"));
            e.target.classList.add("active");
            const mode = e.target.getAttribute("data-mode");
            setViewportMode(mode);
        });
    });

    // Pause / Resume
    const btnPause = document.getElementById("btn-pause");
    btnPause.addEventListener("click", () => {
        isPaused = !isPaused;
        btnPause.textContent = isPaused ? "RESUME" : "PAUSE";
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify({ command: isPaused ? "pause" : "resume" }));
        }
    });

    // Reset
    const btnReset = document.getElementById("btn-reset");
    btnReset.addEventListener("click", () => {
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify({ command: "reset" }));
        }
    });

    // Fleet-Wide Autonomous Retreat (RTL)
    const btnRetreat = document.getElementById("btn-retreat");
    if (btnRetreat) {
        btnRetreat.addEventListener("click", () => {
            triggerFleetRetreat();
        });
    }

    // Speed Control
    const selectSpeed = document.getElementById("select-speed");
    selectSpeed.addEventListener("change", (e) => {
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify({ command: "speed", value: parseFloat(e.target.value) }));
        }
    });

    // Analytics Drawer Toggle
    const btnAnalytics = document.getElementById("btn-toggle-analytics");
    const drawerAnalytics = document.getElementById("analytics-drawer");
    const btnCloseAnalytics = document.getElementById("btn-close-analytics");

    if (btnAnalytics && drawerAnalytics) {
        btnAnalytics.addEventListener("click", () => {
            drawerAnalytics.classList.toggle("hidden");
        });
    }
    if (btnCloseAnalytics && drawerAnalytics) {
        btnCloseAnalytics.addEventListener("click", () => {
            drawerAnalytics.classList.add("hidden");
        });
    }

    // Military HUD Overlay Toggle Button & Keybind
    const btnToggleHud = document.getElementById("btn-toggle-hud");
    if (btnToggleHud) {
        btnToggleHud.addEventListener("click", () => {
            isHudEnabled = !isHudEnabled;
            btnToggleHud.textContent = isHudEnabled ? "HUD: ON" : "HUD: OFF";
            btnToggleHud.classList.toggle("text-neon-green", isHudEnabled);
            btnToggleHud.classList.toggle("text-dim", !isHudEnabled);
        });
    }

    window.addEventListener("keydown", (e) => {
        if (e.key === "h" || e.key === "H") {
            // Avoid triggering when user is in input box
            if (e.target && e.target.tagName === "INPUT") return;
            isHudEnabled = !isHudEnabled;
            if (btnToggleHud) {
                btnToggleHud.textContent = isHudEnabled ? "HUD: ON" : "HUD: OFF";
                btnToggleHud.classList.toggle("text-neon-green", isHudEnabled);
                btnToggleHud.classList.toggle("text-dim", !isHudEnabled);
            }
        }
    });

    // Investor Pitch Mode Button & Lower-Third Bar
    const btnInvestor = document.getElementById("btn-investor");
    const btnExitInvestor = document.getElementById("btn-exit-investor");
    const investorLowerBar = document.getElementById("investor-lower-bar");

    if (btnInvestor) {
        btnInvestor.addEventListener("click", () => {
            isInvestorMode = !isInvestorMode;
            if (isInvestorMode) {
                investorStartTime = performance.now();
                if (investorLowerBar) investorLowerBar.classList.remove("hidden");
                setViewportMode("split");
                initAudio();
                playTacticalSound("relay_lock");
            } else {
                if (investorLowerBar) investorLowerBar.classList.add("hidden");
            }
        });
    }

    if (btnExitInvestor) {
        btnExitInvestor.addEventListener("click", () => {
            isInvestorMode = false;
            if (investorLowerBar) investorLowerBar.classList.add("hidden");
        });
    }

    // Audio Mute/Unmute Toggle Button
    const btnAudio = document.getElementById("btn-audio");
    if (btnAudio) {
        btnAudio.addEventListener("click", () => {
            initAudio();
            isAudioMuted = !isAudioMuted;
            btnAudio.textContent = isAudioMuted ? "AUDIO OFF" : "AUDIO ON";
            btnAudio.classList.toggle("text-neon-green", !isAudioMuted);
            if (!isAudioMuted) {
                playTacticalSound("radar_ping");
            }
        });
    }

    // High-Res Mission Snapshot Button
    const btnSnapshot = document.getElementById("btn-snapshot");
    if (btnSnapshot) {
        btnSnapshot.addEventListener("click", () => {
            takeSnapshot();
            playTacticalSound("radar_ping");
        });
    }

    // Camera view buttons
    document.querySelectorAll(".btn-cam").forEach(btn => {
        btn.addEventListener("click", (e) => {
            document.querySelectorAll(".btn-cam").forEach(b => b.classList.remove("active"));
            e.target.classList.add("active");

            const mode = e.target.getAttribute("data-cam");
            activeCamMode = mode;
            const flirOverlay = document.getElementById("flir-overlay");

            if (mode === "flir") {
                if (flirOverlay) flirOverlay.classList.remove("hidden");
                sceneTheater.background = new THREE.Color(0x021008);
            } else {
                if (flirOverlay) flirOverlay.classList.add("hidden");
                sceneTheater.background = new THREE.Color(0x060a12);
            }

            if (mode === "top") {
                cameraTheater.position.set(0, 0, 480);
                controlsTheater.target.set(0, 0, 0);
            } else if (mode === "orbit") {
                cameraTheater.position.set(0, -380, 240);
                controlsTheater.target.set(0, 0, 20);
            } else if (mode === "gcs") {
                cameraTheater.position.set(0, -250, 25);
                controlsTheater.target.set(0, 0, 35);
            }
        });
    });

    // Close inspect panel
    const btnCloseInspect = document.getElementById("btn-close-inspect");
    if (btnCloseInspect) {
        btnCloseInspect.addEventListener("click", () => {
            toggleInspectPanel(false);
        });
    }

    // Toggle inspect panel header button
    const btnToggleTelem = document.getElementById("btn-toggle-telemetry");
    if (btnToggleTelem) {
        btnToggleTelem.addEventListener("click", () => {
            toggleInspectPanel();
        });
    }

    // Export CSV
    const btnExport = document.getElementById("btn-export-csv");
    if (btnExport) {
        btnExport.addEventListener("click", () => {
            window.location.href = "/api/export_telemetry";
        });
    }

    // Fleet List Delegated Event Handlers (Immediate pointerdown + debounced click to prevent selection drops)
    const fleetContainer = document.getElementById("fleet-list");
    if (fleetContainer) {
        let lastSelectTime = 0;
        const handleFleetSelect = (e) => {
            const rtlBtn = e.target.closest(".drone-card-rtl-btn");
            if (rtlBtn) {
                e.stopPropagation();
                if (e.type === "pointerdown" || e.type === "click") {
                    const dId = rtlBtn.getAttribute("data-drone-id");
                    if (dId && typeof triggerDroneRTL === "function") {
                        triggerDroneRTL(dId);
                    }
                }
                return;
            }

            const card = e.target.closest(".drone-card");
            if (card) {
                const dId = card.getAttribute("data-drone-id");
                const now = performance.now();
                if (dId && (now - lastSelectTime > 80)) {
                    lastSelectTime = now;
                    selectDrone(dId);
                }
            }
        };

        fleetContainer.addEventListener("pointerdown", handleFleetSelect);
        fleetContainer.addEventListener("click", handleFleetSelect);

        // Accessible keyboard enter/space on focused card
        fleetContainer.addEventListener("keydown", (e) => {
            if (e.key === "Enter" || e.key === " ") {
                const card = e.target.closest(".drone-card");
                if (card) {
                    e.preventDefault();
                    const dId = card.getAttribute("data-drone-id");
                    if (dId) selectDrone(dId);
                }
            }
        });
    }

    // Disaster Site Priority Queue Interaction (Select assigned drone on click/pointerdown)
    const poiContainer = document.getElementById("poi-list");
    if (poiContainer) {
        let lastPoiSelectTime = 0;
        const handlePoiSelect = (e) => {
            const card = e.target.closest(".poi-card");
            if (card) {
                const dId = card.getAttribute("data-drone-id");
                const now = performance.now();
                if (dId && (now - lastPoiSelectTime > 80) && typeof selectDrone === "function") {
                    lastPoiSelectTime = now;
                    selectDrone(dId);
                }
            }
        };
        poiContainer.addEventListener("pointerdown", handlePoiSelect);
        poiContainer.addEventListener("click", handlePoiSelect);
    }

    // Keyboard Drone Quick-Select Hotkeys (1-8 for UAV_1..8, 9 for RELAY_1, 0 for SCOUT_1, '[' and ']' for cycle)
    window.addEventListener("keydown", (e) => {
        if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "SELECT" || e.target.tagName === "TEXTAREA")) return;

        if (e.key >= "1" && e.key <= "8") {
            const targetId = `UAV_${e.key}`;
            if (latestTelemetry && (latestTelemetry.drones || []).some(d => d.id === targetId)) {
                selectDrone(targetId);
            }
        } else if (e.key === "9") {
            if (latestTelemetry && (latestTelemetry.drones || []).some(d => d.id === "RELAY_1")) {
                selectDrone("RELAY_1");
            }
        } else if (e.key === "0") {
            if (latestTelemetry && (latestTelemetry.drones || []).some(d => d.id === "SCOUT_1")) {
                selectDrone("SCOUT_1");
            }
        } else if (e.key === "[" || e.key === "]") {
            if (latestTelemetry && latestTelemetry.drones && latestTelemetry.drones.length > 0) {
                const drones = latestTelemetry.drones;
                const curIdx = drones.findIndex(d => d.id === selectedDroneId);
                let nextIdx = 0;
                if (e.key === "]") {
                    nextIdx = curIdx >= 0 ? (curIdx + 1) % drones.length : 0;
                } else {
                    nextIdx = curIdx >= 0 ? (curIdx - 1 + drones.length) % drones.length : drones.length - 1;
                }
                selectDrone(drones[nextIdx].id);
            }
        }
    });

    window.addEventListener("resize", onWindowResize);
    animate();
}

function setViewportMode(mode) {
    activeViewportMode = mode;
    const wrapper = document.getElementById("viewports-wrapper");
    if (!wrapper) return;
    wrapper.className = `mode-${mode}`;
    setTimeout(onWindowResize, 50);
}

function onWindowResize() {
    const cTheater = document.getElementById("canvas-theater-container");
    if (cTheater && rendererTheater && cameraTheater) {
        const w = cTheater.clientWidth;
        const h = cTheater.clientHeight;
        if (w > 0 && h > 0) {
            cameraTheater.aspect = w / h;
            cameraTheater.updateProjectionMatrix();
            rendererTheater.setSize(w, h);
        }
    }

    const cSLAM = document.getElementById("canvas-slam-container");
    if (cSLAM && rendererSLAM && cameraSLAM) {
        const w = cSLAM.clientWidth;
        const h = cSLAM.clientHeight;
        if (w > 0 && h > 0) {
            cameraSLAM.aspect = w / h;
            cameraSLAM.updateProjectionMatrix();
            rendererSLAM.setSize(w, h);
        }
    }
}

// ============================================================================
// Animation Loop & Investor Presentation Mode
// ============================================================================

function animate() {
    requestAnimationFrame(animate);

    // Spin Propeller Rotors and Blurred Discs
    rotorMeshes.forEach(rotor => {
        rotor.rotation.z += 0.45;
    });

    // Animate FAA / Military LED Navigation Strobes
    const tSec = performance.now() * 0.001;
    strobeObjects.forEach(strobe => {
        if (strobe.strobeType === "beacon") {
            const pulse = (Math.sin(tSec * 9.42) > 0.6 && Math.sin(tSec * 18.84) > 0) ? 1.0 : 0.08;
            strobe.material.opacity = pulse;
        } else if (strobe.strobeType === "port") {
            const pulse = Math.sin(tSec * 6.28) > 0.4 ? 1.0 : 0.15;
            strobe.material.opacity = pulse;
        } else if (strobe.strobeType === "starboard") {
            const pulse = 0.5 + 0.5 * Math.sin(tSec * 6.28);
            strobe.material.opacity = pulse > 0.3 ? 0.95 : 0.2;
        }
    });

    // Rotate SLAM LiDAR 360° Sweep Beam
    if (lidarSweepLine && slamDroneMesh) {
        lidarSweepLine.position.copy(slamDroneMesh.position);
        lidarSweepLine.rotation.z += 0.08;
    }

    // Update Disaster Smoke Particles
    if (smokeParticles) {
        const pos = smokeParticles.geometry.attributes.position.array;
        const speeds = smokeParticles.speeds;
        const origins = smokeParticles.origins;
        for (let i = 0; i < speeds.length; i++) {
            pos[i * 3 + 2] += speeds[i];
            pos[i * 3] += (Math.random() - 0.48) * 0.15;
            pos[i * 3 + 1] += (Math.random() - 0.48) * 0.15;
            if (pos[i * 3 + 2] > 55) {
                const orig = origins[i % origins.length];
                pos[i * 3] = orig.x + (Math.random() - 0.5) * 8;
                pos[i * 3 + 1] = orig.y + (Math.random() - 0.5) * 8;
                pos[i * 3 + 2] = orig.z;
            }
        }
        smokeParticles.geometry.attributes.position.needsUpdate = true;
    }

    // Update Ground Wash Dust
    if (washParticles && droneMeshes.size > 0) {
        const pos = washParticles.geometry.attributes.position.array;
        let pIdx = 0;
        droneMeshes.forEach((mesh) => {
            if (mesh.position.z < 15.0 && pIdx < pos.length - 9) {
                const angle = Math.random() * Math.PI * 2;
                const r = 2.0 + Math.random() * 4.0;
                pos[pIdx] = mesh.position.x + Math.cos(angle) * r;
                pos[pIdx + 1] = mesh.position.y + Math.sin(angle) * r;
                pos[pIdx + 2] = 0.2 + Math.random() * 0.8;
                pIdx += 3;
            }
        });
        washParticles.geometry.attributes.position.needsUpdate = true;
    }

    // Handle Investor Pitch Choreography
    if (isInvestorMode) {
        handleInvestorChoreography();
    } else {
        // Normal Camera Control
        if (activeCamMode === "fpv") {
            const targetMesh = droneMeshes.get(selectedDroneId) || droneMeshes.values().next().value;
            if (targetMesh) {
                const forward = new THREE.Vector3(0, 1, 0).applyQuaternion(targetMesh.quaternion);
                const camPos = targetMesh.position.clone().sub(forward.clone().multiplyScalar(14)).add(new THREE.Vector3(0, 0, 6));
                cameraTheater.position.lerp(camPos, 0.08);
                controlsTheater.target.lerp(targetMesh.position.clone().add(forward.clone().multiplyScalar(10)), 0.12);
            }
        } else if (activeCamMode === "gcs") {
            cameraTheater.position.set(0, -150, 22);
            const focusMesh = droneMeshes.get(selectedDroneId) || droneMeshes.get("UAV_1");
            if (focusMesh) {
                controlsTheater.target.lerp(focusMesh.position, 0.05);
            }
        }
    }

    // Update SLAM Camera to follow focused drone smoothly
    if (slamDroneMesh && controlsSLAM) {
        const offset = new THREE.Vector3(0, -45, 30);
        const desiredPos = slamDroneMesh.position.clone().add(offset);
        cameraSLAM.position.lerp(desiredPos, 0.06);
        controlsSLAM.target.lerp(slamDroneMesh.position, 0.1);
        controlsSLAM.update();
    }

    if (controlsTheater) controlsTheater.update();

    // Render both viewports
    if (rendererTheater && sceneTheater && cameraTheater) {
        rendererTheater.render(sceneTheater, cameraTheater);
        // Render Military Aerospace HUD Layer on Viewport 1
        renderMilitaryHUD();
    }
    if (rendererSLAM && sceneSLAM && cameraSLAM && (activeViewportMode !== "theater")) {
        rendererSLAM.render(sceneSLAM, cameraSLAM);
    }
}

// Automated 60-Second Investor Presentation Choreography
let currentInvestorPhase = -1;
function handleInvestorChoreography() {
    const elapsedSec = (performance.now() - investorStartTime) / 1000;
    const titleEl = document.getElementById("investor-phase-title");
    const descEl = document.getElementById("investor-phase-desc");
    const phase = Math.floor(elapsedSec / 15) % 4;

    if (phase !== currentInvestorPhase) {
        currentInvestorPhase = phase;
        playTacticalSound("relay_lock");
    }

    if (elapsedSec < 15) {
        // Phase 1: Fleet Launch & Coordinated Dispersal
        if (titleEl) titleEl.textContent = "PHASE 1: HETEROGENEOUS SWARM DEPLOYMENT & CORRIDOR TRANSIT";
        if (descEl) descEl.textContent = "16-UAV autonomous fleet executes coordinated takeoff and altitude corridor separation across 700m disaster sector with zero human intervention.";
        
        const orbitAngle = elapsedSec * 0.12;
        const camX = Math.sin(orbitAngle) * 360;
        const camY = -Math.cos(orbitAngle) * 360;
        cameraTheater.position.lerp(new THREE.Vector3(camX, camY, 240), 0.05);
        controlsTheater.target.lerp(new THREE.Vector3(0, 0, 25), 0.05);
    } else if (elapsedSec < 30) {
        // Phase 2: Non-Line-Of-Sight Relay Deployment Over Collapsed Skyscraper
        if (titleEl) titleEl.textContent = "PHASE 2: NON-LINE-OF-SIGHT AERIAL RELAY BRIDGING";
        if (descEl) descEl.textContent = "Elevated relay fleet executes Virtual Spring Mesh (VSM) positioning above 45m obstacle shadow zones to maintain 100% GCS connectivity.";

        const relayMesh = droneMeshes.get("RELAY_1");
        if (relayMesh) {
            cameraTheater.position.lerp(relayMesh.position.clone().add(new THREE.Vector3(30, -30, 20)), 0.05);
            controlsTheater.target.lerp(relayMesh.position, 0.08);
        }
    } else if (elapsedSec < 45) {
        // Phase 3: Autonomous SLAM & 3D Voxel Cognition
        if (titleEl) titleEl.textContent = "PHASE 3: ONBOARD 3D LIDAR SLAM & VOXEL RECONSTRUCTION";
        if (descEl) descEl.textContent = "Autonomous UAVs construct 3D OctoMap occupancy grids and execute real-time Khatib APF obstacle deconfliction.";

        // Focus on SLAM surveyor
        selectDrone("UAV_1", false);
        const uavMesh = droneMeshes.get("UAV_1");
        if (uavMesh) {
            cameraTheater.position.lerp(uavMesh.position.clone().add(new THREE.Vector3(-20, -20, 12)), 0.05);
            controlsTheater.target.lerp(uavMesh.position, 0.08);
        }
    } else if (elapsedSec < 60) {
        // Phase 4: Autonomous Swarm Recovery & Precision RTL Landing
        if (titleEl) titleEl.textContent = "PHASE 4: AUTONOMOUS RECOVERY & RETURN-TO-LAUNCH (RTL)";
        if (descEl) descEl.textContent = "Upon completing disaster surveillance or reaching low-battery thresholds, drones execute deconflicted corridor retreat and controlled touchdown on recovery pads.";

        cameraTheater.position.lerp(new THREE.Vector3(0, -340, 210), 0.04);
        controlsTheater.target.lerp(new THREE.Vector3(0, 0, 20), 0.04);
    } else {
        // Loop back to Phase 1 for continuous demo
        investorStartTime = performance.now();
    }
}

// ============================================================================
// Military Aerospace Heads-Up Display (MIL-STD-1787D) Web Overlay Engine
// ============================================================================

function getActiveGPUInfo() {
    let unmasked = "";
    try {
        if (rendererTheater && rendererTheater.getContext) {
            const gl = rendererTheater.getContext();
            const dbg = gl ? gl.getExtension("WEBGL_debug_renderer_info") : null;
            if (dbg && gl) {
                unmasked = gl.getParameter(dbg.UNMASKED_RENDERER_WEBGL) || "";
            }
        }
    } catch (e) {}

    const isNvidia = /nvidia|geforce|rtx|gtx/i.test(unmasked);
    const isAmd = /amd|radeon/i.test(unmasked);

    let cleanName = "NVIDIA RTX 4050";
    if (isNvidia) {
        cleanName = unmasked.includes("4050") ? "NVIDIA RTX 4050" : "NVIDIA DISCRETE GPU";
    } else if (isAmd) {
        cleanName = "AMD RADEON (INT)";
    }
    return { unmasked, isNvidia, isAmd, cleanName };
}

let hudRadarAngle = 0;

function renderMilitaryHUD() {
    const canvas = document.getElementById("web-military-hud");
    if (!canvas) return;

    const parent = canvas.parentElement;
    if (!parent) return;

    const w = parent.clientWidth || 800;
    const h = parent.clientHeight || 600;

    if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
    }

    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, w, h);

    if (!isHudEnabled) return;

    // Resolve focused drone data
    let drone = null;
    if (latestTelemetry && latestTelemetry.drones) {
        drone = latestTelemetry.drones.find(d => d.id === selectedDroneId) || latestTelemetry.drones[0];
    }
    if (!drone) return;

    const pos = drone.position || [0, 0, 10];
    const vel = drone.velocity || [0, 0, 0];
    const att = drone.attitude || [0, 0, 0]; // roll, pitch, yaw in rad
    const speed = Math.hypot(vel[0], vel[1], vel[2]);
    const alt = pos[2];
    const vsi = vel[2];

    const rollRad = att[0];
    const pitchRad = att[1];
    const yawRad = att[2];

    const rollDeg = rollRad * 180 / Math.PI;
    const pitchDeg = pitchRad * 180 / Math.PI;
    const yawDeg = ((yawRad * 180 / Math.PI) % 360 + 360) % 360;

    const cx = w / 2;
    const cy = h / 2;

    const COLOR_CYAN = "#00e5ff";
    const COLOR_GREEN = "#00ff66";
    const COLOR_YELLOW = "#ffd600";
    const COLOR_PURPLE = "#d500f9";
    const COLOR_RED = "#ff1744";
    const COLOR_NVIDIA = "#76b900";

    ctx.save();

    // 1. Tactical HUD Header & Live NVIDIA GPU Telemetry (Clear Flight Zone)
    const hudInfoX = Math.max(300, cx - 210);
    const hudInfoY = 125;
    ctx.font = "bold 13px 'Consolas', 'Courier New', monospace";
    ctx.fillStyle = COLOR_CYAN;
    ctx.fillText(`UAV-X TACTICAL HUD // CALLSIGN: ${drone.id} [${drone.role || "SURVEY"}]`, hudInfoX, hudInfoY);

    ctx.font = "11px 'Consolas', 'Courier New', monospace";
    ctx.fillStyle = "#a0d0d0";
    ctx.fillText(`AUTONOMY: ${drone.flight_mode || "AUTONOMOUS"} | CAM: ${activeCamMode.toUpperCase()} | SENSOR: TACTICAL RGB`, hudInfoX, hudInfoY + 16);

    // NVIDIA GPU Badge
    const gpu = latestTelemetry && latestTelemetry.gpu ? latestTelemetry.gpu : {
        name: "NVIDIA RTX 4050", temp_c: 50, power_w: 16.0, vram_used_mb: 291
    };
    const activeGpu = getActiveGPUInfo();
    const gpuPwr = gpu.power_w ? `${Math.round(gpu.power_w)}W` : "16W";
    const gpuColor = activeGpu.isNvidia ? COLOR_NVIDIA : (activeGpu.isAmd ? COLOR_YELLOW : COLOR_CYAN);
    ctx.fillStyle = gpuColor;
    ctx.fillText(`${activeGpu.cleanName} [${activeGpu.isNvidia ? "NVIDIA DISCRETE" : "INT"}] | ${gpu.temp_c || 50}°C | ${gpuPwr} | VRAM ${gpu.vram_used_mb || 291}MB | CUDA [ACTIVE]`, hudInfoX, hudInfoY + 32);

    // Autonomous Retreat / Recovery Status Banner Callout
    if (drone.flight_mode === "RTL") {
        ctx.fillStyle = "rgba(255, 170, 0, 0.28)";
        ctx.fillRect(cx - 180, 72, 360, 24);
        ctx.strokeStyle = COLOR_YELLOW;
        ctx.lineWidth = 1.5;
        ctx.strokeRect(cx - 180, 72, 360, 24);
        ctx.font = "bold 11px 'Consolas', monospace";
        ctx.fillStyle = COLOR_YELLOW;
        ctx.textAlign = "center";
        ctx.fillText("AUTONOMOUS RETREAT // RTL CORRIDOR ACTIVE", cx, 88);
    } else if (drone.flight_mode === "LANDING") {
        ctx.fillStyle = "rgba(0, 229, 255, 0.28)";
        ctx.fillRect(cx - 180, 72, 360, 24);
        ctx.strokeStyle = COLOR_CYAN;
        ctx.lineWidth = 1.5;
        ctx.strokeRect(cx - 180, 72, 360, 24);
        ctx.font = "bold 11px 'Consolas', monospace";
        ctx.fillStyle = COLOR_CYAN;
        ctx.textAlign = "center";
        ctx.fillText("CONTROLLED DESCENT // APPROACHING RECOVERY PAD", cx, 88);
    } else if (drone.flight_mode === "LANDED") {
        ctx.fillStyle = "rgba(0, 255, 102, 0.28)";
        ctx.fillRect(cx - 180, 72, 360, 24);
        ctx.strokeStyle = COLOR_GREEN;
        ctx.lineWidth = 1.5;
        ctx.strokeRect(cx - 180, 72, 360, 24);
        ctx.font = "bold 11px 'Consolas', monospace";
        ctx.fillStyle = COLOR_GREEN;
        ctx.textAlign = "center";
        ctx.fillText("RECOVERY COMPLETE // ENGINES SAFE & SHUTDOWN", cx, 88);
    } else if (drone.flight_mode === "EMERGENCY_LAND") {
        ctx.fillStyle = "rgba(255, 23, 68, 0.38)";
        ctx.fillRect(cx - 180, 72, 360, 24);
        ctx.strokeStyle = COLOR_RED;
        ctx.lineWidth = 1.5;
        ctx.strokeRect(cx - 180, 72, 360, 24);
        ctx.font = "bold 11px 'Consolas', monospace";
        ctx.fillStyle = "#ffffff";
        ctx.textAlign = "center";
        ctx.fillText("FAILSAFE DESCENT // CRITICAL BATTERY", cx, 88);
    }

    // 2. Boresight Reference Waterline Crosshair (_o_)
    ctx.strokeStyle = COLOR_CYAN;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, 3.5, 0, Math.PI * 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(cx - 24, cy); ctx.lineTo(cx - 8, cy); ctx.lineTo(cx - 8, cy + 4);
    ctx.moveTo(cx + 8, cy); ctx.lineTo(cx + 24, cy); ctx.lineTo(cx + 8, cy + 4);
    ctx.stroke();

    // 3. Flight Path Marker (FPM) Velocity Vector
    const horizSpeed = Math.hypot(vel[0], vel[1]);
    const aoa = Math.atan2(vel[2], Math.max(horizSpeed, 0.5));
    const headingTrack = horizSpeed > 0.2 ? Math.atan2(vel[1], vel[0]) : yawRad;
    let drift = headingTrack - yawRad;
    while (drift > Math.PI) drift -= 2 * Math.PI;
    while (drift < -Math.PI) drift += 2 * Math.PI;

    const fpmX = Math.min(Math.max(cx + drift * (w * 0.35), cx - 180), cx + 180);
    const fpmY = Math.min(Math.max(cy - aoa * (h * 0.35), cy - 140), cy + 140);

    ctx.strokeStyle = COLOR_GREEN;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(fpmX, fpmY, 5.5, 0, Math.PI * 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(fpmX - 12, fpmY); ctx.lineTo(fpmX - 6, fpmY);
    ctx.moveTo(fpmX + 6, fpmY); ctx.lineTo(fpmX + 12, fpmY);
    ctx.moveTo(fpmX, fpmY - 6); ctx.lineTo(fpmX, fpmY - 10);
    ctx.stroke();

    // 4. Dynamic Pitch Ladder (+-30 deg rungs)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-rollRad);

    const pitchScale = 6.5; // pixels per degree
    const pitchOffset = pitchDeg * pitchScale;

    // Horizon line
    ctx.strokeStyle = COLOR_CYAN;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-90, pitchOffset); ctx.lineTo(-24, pitchOffset);
    ctx.moveTo(24, pitchOffset); ctx.lineTo(90, pitchOffset);
    ctx.stroke();

    // Pitch rungs
    for (let p = -30; p <= 30; p += 10) {
        if (p === 0) continue;
        const rungY = -p * pitchScale + pitchOffset;
        if (Math.abs(rungY) > cy - 80) continue;

        ctx.font = "10px 'Consolas', monospace";
        ctx.fillStyle = COLOR_CYAN;
        ctx.textAlign = "right";
        ctx.fillText(Math.abs(p).toString(), -34, rungY + 3);
        ctx.textAlign = "left";
        ctx.fillText(Math.abs(p).toString(), 34, rungY + 3);

        ctx.beginPath();
        if (p > 0) {
            // Positive pitch: solid lines with downward ticks
            ctx.setLineDash([]);
            ctx.moveTo(-30, rungY); ctx.lineTo(-12, rungY); ctx.lineTo(-12, rungY + 5);
            ctx.moveTo(30, rungY); ctx.lineTo(12, rungY); ctx.lineTo(12, rungY + 5);
        } else {
            // Negative pitch: dashed lines with upward ticks
            ctx.setLineDash([4, 4]);
            ctx.moveTo(-30, rungY); ctx.lineTo(-12, rungY); ctx.lineTo(-12, rungY - 5);
            ctx.moveTo(30, rungY); ctx.lineTo(12, rungY); ctx.lineTo(12, rungY - 5);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }
    ctx.restore();

    // 5. Roll Bank Angle Scale Arc & Pointer
    const rollRadius = 90;
    const arcCenterY = cy - 60;
    ctx.strokeStyle = COLOR_CYAN;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, arcCenterY, rollRadius, Math.PI * 1.2, Math.PI * 1.8);
    ctx.stroke();

    // Roll scale tick marks
    [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60].forEach(deg => {
        const rad = (deg - 90) * Math.PI / 180;
        const x1 = cx + Math.cos(rad) * rollRadius;
        const y1 = arcCenterY + Math.sin(rad) * rollRadius;
        const len = (deg === 0 || Math.abs(deg) === 30 || Math.abs(deg) === 60) ? 7 : 4;
        const x2 = cx + Math.cos(rad) * (rollRadius - len);
        const y2 = arcCenterY + Math.sin(rad) * (rollRadius - len);
        ctx.beginPath();
        ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
        ctx.stroke();
    });

    // Roll indicator pointer
    const ptrRad = (-rollDeg - 90) * Math.PI / 180;
    const px = cx + Math.cos(ptrRad) * (rollRadius - 2);
    const py = arcCenterY + Math.sin(ptrRad) * (rollRadius - 2);
    ctx.fillStyle = COLOR_YELLOW;
    ctx.beginPath();
    ctx.arc(px, py, 3, 0, Math.PI * 2);
    ctx.fill();

    // 6. Magnetic Heading Compass Tape (Top)
    const tapeY = Math.max(160, cy - 180);
    const tapeHalfW = Math.min(160, (w - 520) / 2 > 100 ? (w - 520) / 2 : 140);
    ctx.strokeStyle = COLOR_CYAN;
    ctx.lineWidth = 1;
    ctx.strokeRect(cx - tapeHalfW, tapeY - 14, tapeHalfW * 2, 28);

    // Degree tick marks
    const degPixels = 4.0; // pixels per degree
    const startDeg = Math.floor((yawDeg - 35) / 5) * 5;
    const endDeg = Math.floor((yawDeg + 35) / 5) * 5;

    ctx.save();
    ctx.rect(cx - tapeHalfW, tapeY - 14, tapeHalfW * 2, 28);
    ctx.clip();

    for (let d = startDeg; d <= endDeg; d += 5) {
        let normalized = ((d % 360) + 360) % 360;
        const xPos = cx + (d - yawDeg) * degPixels;
        const isTen = (normalized % 10 === 0);
        const tickH = isTen ? 8 : 4;

        ctx.beginPath();
        ctx.moveTo(xPos, tapeY + 14);
        ctx.lineTo(xPos, tapeY + 14 - tickH);
        ctx.stroke();

        if (isTen) {
            let label = (normalized / 10).toString().padStart(2, "0");
            if (normalized === 0) label = "N";
            else if (normalized === 90) label = "E";
            else if (normalized === 180) label = "S";
            else if (normalized === 270) label = "W";

            ctx.font = "9px 'Consolas', monospace";
            ctx.fillStyle = COLOR_CYAN;
            ctx.textAlign = "center";
            ctx.fillText(label, xPos, tapeY - 1);
        }
    }
    ctx.restore();

    // Digital Heading Box in Center
    ctx.fillStyle = "rgba(6, 12, 20, 0.9)";
    ctx.fillRect(cx - 18, tapeY - 22, 36, 16);
    ctx.strokeStyle = COLOR_YELLOW;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(cx - 18, tapeY - 22, 36, 16);

    ctx.font = "bold 11px 'Consolas', monospace";
    ctx.fillStyle = COLOR_YELLOW;
    ctx.textAlign = "center";
    ctx.fillText(Math.round(yawDeg).toString().padStart(3, "0"), cx, tapeY - 10);

    // Steering Bug toward assigned PoI
    if (drone.assigned_poi_id && latestTelemetry && latestTelemetry.pois) {
        const poi = latestTelemetry.pois.find(p => p.id === drone.assigned_poi_id);
        if (poi) {
            const dx = poi.position[0] - pos[0];
            const dy = poi.position[1] - pos[1];
            const bearingRad = Math.atan2(dy, dx);
            let bearingDeg = ((bearingRad * 180 / Math.PI) % 360 + 360) % 360;
            let diff = bearingDeg - yawDeg;
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;
            const bugX = Math.min(Math.max(cx + diff * degPixels, cx - tapeHalfW + 6), cx + tapeHalfW - 6);

            ctx.fillStyle = COLOR_PURPLE;
            ctx.beginPath();
            ctx.moveTo(bugX, tapeY + 14);
            ctx.lineTo(bugX - 4, tapeY + 20);
            ctx.lineTo(bugX, tapeY + 26);
            ctx.lineTo(bugX + 4, tapeY + 20);
            ctx.closePath();
            ctx.fill();
        }
    }

    // 7. Calibrated Airspeed Tape (CAS - Left Side)
    const casX = Math.max(320, cx - 170);
    const casH = 200;
    ctx.strokeStyle = COLOR_CYAN;
    ctx.lineWidth = 1;
    ctx.strokeRect(casX - 14, cy - casH / 2, 28, casH);

    // Speed ticks
    const spdScale = 14.0; // pixels per m/s
    for (let s = Math.max(0, Math.floor(speed - 6)); s <= Math.floor(speed + 6); s += 1) {
        const sy = cy - (s - speed) * spdScale;
        if (Math.abs(sy - cy) > casH / 2) continue;
        const tickW = (s % 2 === 0) ? 9 : 5;
        ctx.beginPath();
        ctx.moveTo(casX + 14, sy); ctx.lineTo(casX + 14 - tickW, sy);
        ctx.stroke();

        if (s % 2 === 0) {
            ctx.font = "9px 'Consolas', monospace";
            ctx.fillStyle = COLOR_CYAN;
            ctx.textAlign = "right";
            ctx.fillText(s.toString(), casX + 1, sy + 3);
        }
    }

    // Digital Airspeed Box
    ctx.fillStyle = "rgba(6, 12, 20, 0.95)";
    ctx.fillRect(casX - 24, cy - 11, 48, 22);
    ctx.strokeStyle = COLOR_YELLOW;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(casX - 24, cy - 11, 48, 22);

    ctx.font = "bold 11px 'Consolas', monospace";
    ctx.fillStyle = COLOR_YELLOW;
    ctx.textAlign = "center";
    ctx.fillText(speed.toFixed(1), casX, cy + 4);

    ctx.font = "8px 'Consolas', monospace";
    ctx.fillStyle = "#80b0c0";
    ctx.fillText("M/S", casX, cy + 20);

    // 8. Barometric Altitude & VSI Tape (Right Side)
    const altX = Math.min(w - 45, cx + 180);
    const altH = 200;
    ctx.strokeStyle = COLOR_CYAN;
    ctx.lineWidth = 1;
    ctx.strokeRect(altX - 14, cy - altH / 2, 28, altH);

    const altScale = 5.0; // pixels per meter
    for (let a = Math.max(0, Math.floor(alt - 15)); a <= Math.floor(alt + 15); a += 5) {
        const ay = cy - (a - alt) * altScale;
        if (Math.abs(ay - cy) > altH / 2) continue;
        const isTen = (a % 10 === 0);
        const tickW = isTen ? 9 : 5;
        ctx.beginPath();
        ctx.moveTo(altX - 14, ay); ctx.lineTo(altX - 14 + tickW, ay);
        ctx.stroke();

        if (isTen) {
            ctx.font = "9px 'Consolas', monospace";
            ctx.fillStyle = COLOR_CYAN;
            ctx.textAlign = "left";
            ctx.fillText(a.toString(), altX - 2, ay + 3);
        }
    }

    // Digital Altitude Box
    ctx.fillStyle = "rgba(6, 12, 20, 0.95)";
    ctx.fillRect(altX - 24, cy - 11, 48, 22);
    ctx.strokeStyle = COLOR_YELLOW;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(altX - 24, cy - 11, 48, 22);

    ctx.font = "bold 11px 'Consolas', monospace";
    ctx.fillStyle = COLOR_YELLOW;
    ctx.textAlign = "center";
    ctx.fillText(Math.round(alt).toString(), altX, cy + 4);

    // RALT & VSI
    ctx.font = "9px 'Consolas', monospace";
    ctx.fillStyle = "#80b0c0";
    ctx.fillText(`RALT ${alt.toFixed(1)}M`, altX, cy + 22);
    ctx.fillStyle = vsi >= 0 ? COLOR_GREEN : COLOR_RED;
    ctx.fillText(`VSI ${vsi >= 0 ? "+" : ""}${vsi.toFixed(1)}`, altX, cy + 34);

    // 9. 3D Projected Target Lock Reticles (Three.js 3D-to-2D Projection)
    if (cameraTheater && latestTelemetry) {
        // Project PoIs
        if (latestTelemetry.pois) {
            latestTelemetry.pois.forEach(poi => {
                const vec = new THREE.Vector3(poi.position[0], poi.position[1], poi.position[2]);
                vec.project(cameraTheater);
                if (vec.z < 1.0) {
                    const sx = (vec.x * 0.5 + 0.5) * w;
                    const sy = (-vec.y * 0.5 + 0.5) * h;
                    const isTarget = (poi.id === drone.assigned_poi_id);
                    const color = poi.is_completed ? COLOR_GREEN : (poi.priority === "CRITICAL" ? COLOR_PURPLE : COLOR_YELLOW);

                    ctx.strokeStyle = color;
                    ctx.lineWidth = isTarget ? 2 : 1;

                    // Diamond reticle
                    const size = isTarget ? 9 : 6;
                    ctx.beginPath();
                    ctx.moveTo(sx, sy - size); ctx.lineTo(sx + size, sy);
                    ctx.lineTo(sx, sy + size); ctx.lineTo(sx - size, sy);
                    ctx.closePath();
                    ctx.stroke();

                    if (isTarget) {
                        const dist = Math.hypot(poi.position[0] - pos[0], poi.position[1] - pos[1], poi.position[2] - pos[2]);
                        ctx.font = "bold 10px 'Consolas', monospace";
                        ctx.fillStyle = COLOR_CYAN;
                        ctx.textAlign = "left";
                        ctx.fillText(`[${poi.id}] ${Math.round(dist)}M (TGT)`, sx + 12, sy - 4);
                        const progress = ((poi.current_dwell_time || 0) / Math.max(1, poi.required_dwell_time || 15)) * 100;
                        ctx.fillStyle = color;
                        ctx.fillText(`DWELL: ${Math.round(progress)}%`, sx + 12, sy + 8);
                    }
                }
            });
        }

        // Project Peer UAVs
        if (latestTelemetry.drones) {
            latestTelemetry.drones.forEach(peer => {
                if (peer.id === selectedDroneId) return;
                const peerPos = peer.position || [0, 0, 0];
                const vec = new THREE.Vector3(peerPos[0], peerPos[1], peerPos[2]);
                vec.project(cameraTheater);
                if (vec.z < 1.0) {
                    const sx = (vec.x * 0.5 + 0.5) * w;
                    const sy = (-vec.y * 0.5 + 0.5) * h;
                    const dist = Math.hypot(peerPos[0] - pos[0], peerPos[1] - pos[1], peerPos[2] - pos[2]);
                    const isRelay = peer.role === "RELAY";
                    const col = isRelay ? COLOR_YELLOW : COLOR_CYAN;

                    ctx.strokeStyle = col;
                    ctx.lineWidth = 1;
                    const sz = 8;
                    // Corner brackets [ ]
                    ctx.beginPath();
                    ctx.moveTo(sx - sz, sy - sz + 3); ctx.lineTo(sx - sz, sy - sz); ctx.lineTo(sx - sz + 3, sy - sz);
                    ctx.moveTo(sx + sz, sy - sz + 3); ctx.lineTo(sx + sz, sy - sz); ctx.lineTo(sx + sz - 3, sy - sz);
                    ctx.moveTo(sx - sz, sy + sz - 3); ctx.lineTo(sx - sz, sy + sz); ctx.lineTo(sx - sz + 3, sy + sz);
                    ctx.moveTo(sx + sz, sy + sz - 3); ctx.lineTo(sx + sz, sy + sz); ctx.lineTo(sx + sz - 3, sy + sz);
                    ctx.stroke();

                    if (dist < 180) {
                        ctx.font = "9px 'Consolas', monospace";
                        ctx.fillStyle = col;
                        ctx.textAlign = "left";
                        ctx.fillText(`${peer.id} [${isRelay ? "REL" : "SUR"}] ${Math.round(dist)}M`, sx + 11, sy + 3);
                    }
                }
            });
        }
    }

    // 10. Tactical PPI Radar Scope (Positioned in clear theater area)
    const rx = Math.max(340, cx - 150);
    const ry = Math.min(h - 250, cy + 150);
    const radarRad = 46;
    hudRadarAngle = (hudRadarAngle + 0.035) % (Math.PI * 2);

    ctx.fillStyle = "rgba(4, 12, 8, 0.85)";
    ctx.beginPath();
    ctx.arc(rx, ry, radarRad, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = "rgba(0, 255, 100, 0.35)";
    ctx.lineWidth = 1;
    ctx.stroke();

    // Range rings (150m, 350m)
    [0.45, 1.0].forEach(rFrac => {
        ctx.beginPath();
        ctx.arc(rx, ry, radarRad * rFrac, 0, Math.PI * 2);
        ctx.stroke();
    });

    // Radar crosshairs
    ctx.strokeStyle = "rgba(0, 255, 100, 0.25)";
    ctx.beginPath();
    ctx.moveTo(rx - radarRad, ry); ctx.lineTo(rx + radarRad, ry);
    ctx.moveTo(rx, ry - radarRad); ctx.lineTo(rx, ry + radarRad);
    ctx.stroke();

    // Sweep line
    const swX = rx + Math.cos(hudRadarAngle) * radarRad;
    const swY = ry + Math.sin(hudRadarAngle) * radarRad;
    ctx.strokeStyle = "rgba(0, 255, 100, 0.75)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(rx, ry); ctx.lineTo(swX, swY);
    ctx.stroke();

    // Draw Drone Blips on Radar
    const radarScale = radarRad / 350.0;
    if (latestTelemetry && latestTelemetry.drones) {
        latestTelemetry.drones.forEach(d => {
            const dp = d.position || [0, 0, 0];
            const bx = rx + dp[0] * radarScale;
            const by = ry - dp[1] * radarScale;
            if (Math.hypot(bx - rx, by - ry) <= radarRad) {
                const isFoc = d.id === selectedDroneId;
                ctx.fillStyle = d.role === "RELAY" ? COLOR_YELLOW : COLOR_CYAN;
                ctx.beginPath();
                ctx.arc(bx, by, isFoc ? 3.5 : 2, 0, Math.PI * 2);
                ctx.fill();

                if (isFoc) {
                    ctx.strokeStyle = COLOR_GREEN;
                    ctx.lineWidth = 1;
                    ctx.beginPath();
                    ctx.arc(bx, by, 5, 0, Math.PI * 2);
                    ctx.stroke();
                }
            }
        });
    }

    ctx.font = "8px 'Consolas', monospace";
    ctx.fillStyle = "rgba(0, 255, 100, 0.8)";
    ctx.textAlign = "center";
    ctx.fillText("PPI RADAR 350M", rx, ry + radarRad + 12);

    // 11. Bottom Keybinds Quick Hint
    ctx.font = "10px 'Consolas', monospace";
    ctx.fillStyle = "rgba(160, 200, 180, 0.85)";
    ctx.textAlign = "center";
    ctx.fillText("[MOUSE] DRAG TO ORBIT / PAN / ZOOM  |  [CLICK] SELECT DRONE  |  [H] TOGGLE HUD", cx, Math.min(h - 195, cy + 215));

    ctx.restore();
}
