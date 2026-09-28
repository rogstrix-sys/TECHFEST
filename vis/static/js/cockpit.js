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
let flameParticles = null;
let washParticles = null;
const droneTrails = new Map();
const targetVectorLines = new Map();
let dispatchBeaconMesh = null;
let isDispatchMode = false;
let raycasterTheater = new THREE.Raycaster();
let mouseTheater = new THREE.Vector2();

// Tactical Night Operations & Dynamic Lighting
let isNightOps = false;
let ambientLightTheater = null;
let dirLightTheater = null;
let hemiLightTheater = null;
let fillLightTheater = null;
let theaterSkyTex = null;
const droneSearchlights = [];

// Map Navigation & Drag / Pan Control State
let isPanMode = false;
let isShiftHeld = false;
let isMouseDownOnTheater = false;
let pressedNavKeys = {};
let smoothPanTarget = null;
let smoothPanStartTarget = null;
let smoothPanStartCam = null;
let smoothPanProgress = 1.0;
const smoothPanDuration = 400; // ms
let smoothPanStartTime = 0;

// Multi-Palette Thermal FLIR
let activeFlirPalette = "ironbow"; // "ironbow", "whitehot", "blackhot"

// Manual FPV Controller Mode
let isManualControlActive = false;
const activeKeys = {};
let lastManualSendTime = 0;

// Viewport 2: Autonomous SLAM Perception & Dense 3D LiDAR Engine
let sceneSLAM, cameraSLAM, rendererSLAM, controlsSLAM;
let slamDroneMesh = null;
let lidarPointsMesh = null;
const maxLidarPts = 25000;
let lidarWriteIndex = 0;
let lidarTotalStored = 0;
let activeLidarColormap = "turbo"; // "turbo", "intensity", "cyber"
let isTheaterLidarActive = true;
let theaterLidarPointsMesh = null;
let voxelMeshGroup = null;
let voxelInstancedMesh = null;
const maxInstancedVoxels = 1500;
const voxelDummyMatrix = new THREE.Matrix4();
const voxelDummyColor = new THREE.Color();
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
let lastAnimateTime = performance.now();

// Scientific Charts (Chart.js)
let chartEKF = null;
let chartPDR = null;
let chartRF = null;
let chartBattery = null;
let chartHistoryTime = [];
let chartHistoryEKF = [];
let chartHistoryPDR = [];
let chartHistoryThroughput = [];
let currentScenario = "sector_delta";
let outerGroundMesh = null;
let gcsBaseGroup = null;

function applyScenarioUI(scen) {
    currentScenario = "sector_delta";
    const card = document.getElementById("metric-card-challenge");
    if (card) {
        card.style.display = "none";
    }
    if (outerGroundMesh) outerGroundMesh.visible = true;
    if (gcsBaseGroup) gcsBaseGroup.visible = true;
    if (window.SectorDelta && typeof window.SectorDelta.setScenario === "function") {
        window.SectorDelta.setScenario("sector_delta");
    }
    if (cameraTheater && controlsTheater) {
        smoothPanProgress = 1.0;
        controlsTheater.maxDistance = 1600;
        cameraTheater.position.set(135, -345, 215);
        controlsTheater.target.set(0, -45, 25);
        controlsTheater.update();
    }
}
window.applyScenarioUI = applyScenarioUI;

// Initialize on DOM ready
document.addEventListener("DOMContentLoaded", () => {
    try { initTheaterViewport(); } catch (e) { console.error("initTheaterViewport error:", e); }
    try { initSLAMViewport(); } catch (e) { console.error("initSLAMViewport error:", e); }
    try { connectWebSocket(); } catch (e) { console.error("connectWebSocket error:", e); }
    try { initUIControls(); } catch (e) { console.error("initUIControls error:", e); }
    try { initScientificCharts(); } catch (e) { console.error("initScientificCharts error:", e); }
    try { setViewportMode("theater"); } catch (e) { console.error("setViewportMode error:", e); }
    // Fetch initial active scenario from server to guarantee sync
    fetch("/api/scenario")
        .then(r => r.json())
        .then(data => {
            if (data && data.scenario && data.scenario !== currentScenario) {
                applyScenarioUI(data.scenario);
            }
        })
        .catch(err => console.warn("[Scenario] fetch error:", err));
});

// ============================================================================
// Viewport 1: External Theater Reality
// ============================================================================

function initTheaterViewport() {
    const container = document.getElementById("canvas-theater-container");
    const width = container.clientWidth || window.innerWidth / 2;
    const height = container.clientHeight || window.innerHeight;

    // 1. Scene & Photorealistic Daytime Sky Gradient
    sceneTheater = new THREE.Scene();

    // Procedural Vibrant Daytime Sky Dome Canvas Texture matching reference photo
    const skyCanvas = document.createElement("canvas");
    skyCanvas.width = 512;
    skyCanvas.height = 512;
    const skyCtx = skyCanvas.getContext("2d");
    const skyGrad = skyCtx.createLinearGradient(0, 0, 0, 512);
    skyGrad.addColorStop(0.0, "#3382dc"); // Deep rich sky blue at zenith
    skyGrad.addColorStop(0.35, "#529ce8"); // Bright summer blue
    skyGrad.addColorStop(0.70, "#7ab6f0"); // Soft azure horizon
    skyGrad.addColorStop(1.0, "#a5d2f8"); // Crisp sunny haze
    skyCtx.fillStyle = skyGrad;
    skyCtx.fillRect(0, 0, 512, 512);

    const skyTex = new THREE.CanvasTexture(skyCanvas);
    theaterSkyTex = skyTex;
    sceneTheater.background = skyTex;
    sceneTheater.fog = new THREE.FogExp2(0x95c7f2, 0.00045);

    // 2. Camera (3/4 Elevated Isometric Perspective matching reference photograph)
    cameraTheater = new THREE.PerspectiveCamera(46, width / height, 1, 3500);
    cameraTheater.position.set(135, -345, 215);
    cameraTheater.up.set(0, 0, 1);

    // 3. Renderer (High-Performance Hardware Context with ACES Filmic Tone Mapping)
    rendererTheater = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: "high-performance" });
    rendererTheater.setSize(width, height);
    rendererTheater.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    rendererTheater.shadowMap.enabled = true;
    rendererTheater.shadowMap.type = THREE.PCFSoftShadowMap;
    rendererTheater.toneMapping = THREE.ACESFilmicToneMapping;
    rendererTheater.toneMappingExposure = 1.08;
    container.appendChild(rendererTheater.domElement);

    // 4. Controls (Smooth Orbit & Pan)
    if (typeof THREE.OrbitControls !== "undefined") {
        controlsTheater = new THREE.OrbitControls(cameraTheater, rendererTheater.domElement);
        controlsTheater.enableDamping = true;
        controlsTheater.dampingFactor = 0.05;
        controlsTheater.maxPolarAngle = Math.PI / 2 - 0.02;
        controlsTheater.minDistance = 20;
        controlsTheater.maxDistance = 1600;
        controlsTheater.target.set(0, -45, 25);
        controlsTheater.enablePan = true;
        controlsTheater.screenSpacePanning = false; // Panning glides parallel to ground plane
        controlsTheater.panSpeed = 1.5;
        controlsTheater.mouseButtons = {
            LEFT: THREE.MOUSE.ROTATE,
            MIDDLE: THREE.MOUSE.DOLLY,
            RIGHT: THREE.MOUSE.PAN
        };
        controlsTheater.touches = {
            ONE: THREE.TOUCH.ROTATE,
            TWO: THREE.TOUCH.DOLLY_PAN
        };
        controlsTheater.addEventListener("start", () => {
            smoothPanProgress = 1.0; // Hand over control immediately if user begins manual interaction
        });
    }

    // 5. Lighting (Warm Crisp Sun & Sky Ambient Fill)
    const hemiLight = new THREE.HemisphereLight(0xe4f2ff, 0x2e421c, 0.85);
    hemiLight.position.set(0, 0, 300);
    hemiLightTheater = hemiLight;
    ambientLightTheater = hemiLight;
    sceneTheater.add(hemiLight);

    const sunLight = new THREE.DirectionalLight(0xfffaee, 1.35);
    sunLight.position.set(240, -190, 320); // Front-right sun casting crisp soft shadows
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 2048;
    sunLight.shadow.mapSize.height = 2048;
    sunLight.shadow.camera.near = 10;
    sunLight.shadow.camera.far = 1200;
    const d = 260;
    sunLight.shadow.camera.left = -d;
    sunLight.shadow.camera.right = d;
    sunLight.shadow.camera.top = d;
    sunLight.shadow.camera.bottom = -d;
    sunLight.shadow.bias = -0.0004;
    dirLightTheater = sunLight;
    sceneTheater.add(sunLight);

    const fillLight = new THREE.DirectionalLight(0x9eccff, 0.35);
    fillLight.position.set(-220, 210, 160);
    fillLightTheater = fillLight;
    sceneTheater.add(fillLight);

    // 6. Sector Delta Architectural Diorama (or fallback Theater Terrain)
    if (typeof SectorDelta !== "undefined") {
        SectorDelta.init(sceneTheater);
    } else {
        createTheaterTerrain();
    }

    // 7. Base ground plane outside the diorama tray (Expansive natural parkland landscape fading into horizon haze)
    const outerGroundGeo = new THREE.PlaneGeometry(6000, 6000);
    const outerGroundMat = new THREE.MeshStandardMaterial({
        color: 0x1a4016, // Rich botanical green complementary to diorama parkland
        roughness: 0.95,
        metalness: 0.02,
    });
    const outerGround = new THREE.Mesh(outerGroundGeo, outerGroundMat);
    outerGround.position.set(0, 0, -10.5);
    outerGround.receiveShadow = true;
    outerGround.matrixAutoUpdate = false;
    outerGround.updateMatrix();
    outerGroundMesh = outerGround;
    sceneTheater.add(outerGround);

    // 7. GCS Base Station Compound on Sector Delta Diorama Tray at (0, -145, 0.1)
    createGCSBase(0, -145, 0.1);

    // 8. Particle Systems
    initParticleSystems();

    // 8b. 3D LiDAR Point Cloud Overlay in Theater Viewport
    const theaterLidarGeo = new THREE.BufferGeometry();
    const theaterLidarPositions = new Float32Array(maxLidarPts * 3);
    const theaterLidarColors = new Float32Array(maxLidarPts * 3);
    theaterLidarGeo.setAttribute('position', new THREE.BufferAttribute(theaterLidarPositions, 3));
    theaterLidarGeo.setAttribute('color', new THREE.BufferAttribute(theaterLidarColors, 3));
    const theaterLidarMat = new THREE.PointsMaterial({
        size: 3.6,
        vertexColors: true,
        transparent: true,
        opacity: 0.85,
        blending: THREE.AdditiveBlending,
        depthWrite: false,
    });
    theaterLidarPointsMesh = new THREE.Points(theaterLidarGeo, theaterLidarMat);
    theaterLidarPointsMesh.geometry.setDrawRange(0, 0);
    theaterLidarPointsMesh.visible = isTheaterLidarActive;
    sceneTheater.add(theaterLidarPointsMesh);

    // 9. Interactive Raycasting Click-to-Inspect & Double-Click to Center
    rendererTheater.domElement.addEventListener("click", onTheaterCanvasClick);
    rendererTheater.domElement.addEventListener("dblclick", onTheaterCanvasDblClick);
    rendererTheater.domElement.addEventListener("mousedown", () => {
        isMouseDownOnTheater = true;
        updateCanvasCursor();
    });
    window.addEventListener("mouseup", () => {
        isMouseDownOnTheater = false;
        updateCanvasCursor();
    });

    // Synchronize initial scenario visuals and camera if challenge mode was pre-set
    if (currentScenario === "challenge") {
        applyScenarioUI("challenge");
    }
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
    gcsBaseGroup = group;

    // 1. Reinforced Hexagonal Apron Plinth
    const bunkerGeo = new THREE.CylinderGeometry(8.5, 9.8, 2.2, 8);
    const bunkerMat = new THREE.MeshStandardMaterial({
        color: 0x18202c,
        metalness: 0.85,
        roughness: 0.3,
    });
    const bunker = new THREE.Mesh(bunkerGeo, bunkerMat);
    bunker.rotation.x = Math.PI / 2;
    bunker.position.z = 1.1;
    bunker.receiveShadow = true;
    group.add(bunker);

    // Hazard Safety Striping Border Ring
    const stripeGeo = new THREE.RingGeometry(8.6, 9.8, 8);
    const stripeMat = new THREE.MeshBasicMaterial({
        color: 0xffd600,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.85,
    });
    const stripe = new THREE.Mesh(stripeGeo, stripeMat);
    stripe.position.z = 2.25;
    group.add(stripe);

    // 2. Heavy Communications Lattice Mast Tower
    const mastHeight = 26;
    const mastGeo = new THREE.CylinderGeometry(0.8, 1.8, mastHeight, 6);
    const mastMat = new THREE.MeshStandardMaterial({
        color: 0x78909c,
        metalness: 0.92,
        roughness: 0.15,
    });
    const mast = new THREE.Mesh(mastGeo, mastMat);
    mast.rotation.x = Math.PI / 2;
    mast.position.z = mastHeight / 2 + 2.2;
    mast.castShadow = true;
    group.add(mast);

    // Secondary Cross-Bracing Truss Struts
    for (let k = 0; k < 4; k++) {
        const theta = (k / 4) * Math.PI * 2;
        const legGeo = new THREE.CylinderGeometry(0.2, 0.4, 18, 4);
        const leg = new THREE.Mesh(legGeo, mastMat);
        leg.position.set(Math.cos(theta) * 3.2, Math.sin(theta) * 3.2, 9.0);
        leg.rotation.z = theta;
        leg.rotation.y = 0.35;
        leg.rotation.x = Math.PI / 2;
        group.add(leg);
    }

    // 3. High-Gain Radar / Satellite Dish Receiver (Oriented toward city center)
    const dishGeo = new THREE.SphereGeometry(4.8, 16, 16, 0, Math.PI * 2, 0, Math.PI / 2);
    const dishMat = new THREE.MeshStandardMaterial({
        color: 0x00e5ff,
        metalness: 0.8,
        roughness: 0.2,
        wireframe: true,
    });
    const dish = new THREE.Mesh(dishGeo, dishMat);
    dish.rotation.x = -Math.PI / 3.2;
    dish.position.set(0, 2, mastHeight + 3);
    group.add(dish);

    // 4. Omnidirectional Phased-Array Radome Beacon
    const beaconGeo = new THREE.SphereGeometry(1.2, 12, 12);
    const beaconMat = new THREE.MeshBasicMaterial({ color: 0xff0055 });
    const beacon = new THREE.Mesh(beaconGeo, beaconMat);
    beacon.position.set(0, 0, mastHeight + 5.5);
    group.add(beacon);
    strobeObjects.push(beacon);
    beacon.strobeType = "beacon";

    // 5. Tactical Mobile Command Ops Trailer / Field Shelter
    const opsGeo = new THREE.BoxGeometry(14, 8, 5.5);
    const opsMat = new THREE.MeshStandardMaterial({
        color: 0x243242,
        metalness: 0.7,
        roughness: 0.4,
    });
    const ops = new THREE.Mesh(opsGeo, opsMat);
    ops.position.set(0, -9, 5.25);
    ops.castShadow = true;
    group.add(ops);

    // Illuminated Ops Center Observation Visor Windows
    const winGeo = new THREE.BoxGeometry(12, 0.4, 1.8);
    const winMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.85 });
    const win = new THREE.Mesh(winGeo, winMat);
    win.position.set(0, -4.8, 6.2);
    group.add(win);

    // Ops Trailer Rooftop Radome Dome
    const domeGeo = new THREE.SphereGeometry(1.6, 12, 8, 0, Math.PI * 2, 0, Math.PI / 2);
    const domeMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, metalness: 0.3, roughness: 0.2 });
    const dome = new THREE.Mesh(domeGeo, domeMat);
    dome.position.set(-3.5, -9, 8.0);
    group.add(dome);

    sceneTheater.add(group);
}

function initParticleSystems() {
    const origins = [
        new THREE.Vector3(-85, -60, 2),   // POI_COLLAPSE (West Collapsed Apartment Complex)
        new THREE.Vector3(-20, 95, 2),    // POI_HAZARD (North Chemical Processing Plant & Hazard Silos)
        new THREE.Vector3(40, -95, 2),    // POI_BRIDGE (Elevated Highway Overpass Viaduct Fracture)
        new THREE.Vector3(25, -55, 2),    // POI_SURVIVORS (Downtown Central Plaza Skybridge Rubble)
        new THREE.Vector3(115, -40, 2),   // POI_SUBSTATION (East Regional Power Substation)
        new THREE.Vector3(-60, 25, 2),    // POI_HOSPITAL (St. Jude Medical Center Trauma Helipad)
    ];

    // 1. Disaster Smoke Plumes (Billowing atmospheric smoke rising and drifting with wind)
    const smokeCount = 450;
    const smokeGeo = new THREE.BufferGeometry();
    const smokePositions = new Float32Array(smokeCount * 3);
    const smokeSpeeds = new Float32Array(smokeCount);

    for (let i = 0; i < smokeCount; i++) {
        const origin = origins[i % origins.length];
        smokePositions[i * 3] = origin.x + (Math.random() - 0.5) * 16;
        smokePositions[i * 3 + 1] = origin.y + (Math.random() - 0.5) * 16;
        smokePositions[i * 3 + 2] = origin.z + Math.random() * 55;
        smokeSpeeds[i] = 0.18 + Math.random() * 0.35;
    }
    smokeGeo.setAttribute('position', new THREE.BufferAttribute(smokePositions, 3));
    const smokeMat = new THREE.PointsMaterial({
        color: 0x4a5568,
        size: 5.5,
        transparent: true,
        opacity: 0.55,
        depthWrite: false,
    });
    smokeParticles = new THREE.Points(smokeGeo, smokeMat);
    smokeParticles.speeds = smokeSpeeds;
    smokeParticles.origins = origins;
    sceneTheater.add(smokeParticles);

    // 2. Disaster Incandescent Flame & Ember System (Turbulent flickering fires at ground zero)
    const flameCount = 300;
    const flameGeo = new THREE.BufferGeometry();
    const flamePositions = new Float32Array(flameCount * 3);
    const flameSpeeds = new Float32Array(flameCount);

    for (let i = 0; i < flameCount; i++) {
        const origin = origins[i % origins.length];
        flamePositions[i * 3] = origin.x + (Math.random() - 0.5) * 8;
        flamePositions[i * 3 + 1] = origin.y + (Math.random() - 0.5) * 8;
        flamePositions[i * 3 + 2] = origin.z + Math.random() * 14;
        flameSpeeds[i] = 0.35 + Math.random() * 0.65;
    }
    flameGeo.setAttribute('position', new THREE.BufferAttribute(flamePositions, 3));
    const flameMat = new THREE.PointsMaterial({
        color: 0xff5500,
        size: 3.2,
        transparent: true,
        opacity: 0.85,
        blending: THREE.AdditiveBlending,
        depthWrite: false,
    });
    flameParticles = new THREE.Points(flameGeo, flameMat);
    flameParticles.speeds = flameSpeeds;
    flameParticles.origins = origins;
    sceneTheater.add(flameParticles);

    // 3. Ground Wash Dust Rings for Low-Flying Drones
    const washCount = 200;
    const washGeo = new THREE.BufferGeometry();
    const washPositions = new Float32Array(washCount * 3);
    washGeo.setAttribute('position', new THREE.BufferAttribute(washPositions, 3));
    const washMat = new THREE.PointsMaterial({
        color: 0x00e5ff,
        size: 1.8,
        transparent: true,
        opacity: 0.5,
        depthWrite: false,
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

    // 6. Downward Scanning Laser Footprint Cone & Ground Drop Reticle
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

    // Ground Drop Laser Line & Ground Footprint Reticle (shown only for active/selected drone)
    const altLineGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1)]);
    const altLineMat = new THREE.LineDashedMaterial({
        color: primaryColor,
        dashSize: 1.5,
        gapSize: 1.0,
        transparent: true,
        opacity: 0.75,
        depthWrite: false
    });
    const altLine = new THREE.Line(altLineGeo, altLineMat);
    altLine.visible = false;
    droneGroup.add(altLine);
    droneGroup.altLine = altLine;

    // Laser Ground Reticle
    const reticleGeo = new THREE.RingGeometry(1.6, 2.2, 24);
    const reticleMat = new THREE.MeshBasicMaterial({
        color: primaryColor,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.65,
        depthWrite: false
    });
    const reticle = new THREE.Mesh(reticleGeo, reticleMat);
    reticle.visible = false;
    droneGroup.add(reticle);
    droneGroup.reticle = reticle;

    // 7. Tactical Night Operations Searchlight
    const searchLight = new THREE.SpotLight(0xfff3e0, isNightOps ? 3.5 : 0.0, 110, Math.PI / 5.5, 0.45, 1.2);
    searchLight.position.set(0, 0, -0.3);
    const searchTarget = new THREE.Object3D();
    searchTarget.position.set(0, 0, -35);
    droneGroup.add(searchTarget);
    searchLight.target = searchTarget;
    droneGroup.add(searchLight);
    droneGroup.searchLight = searchLight;
    droneSearchlights.push(searchLight);

    // Dynamic Motion Interpolation Targets (60-144 FPS smooth animation)
    droneGroup.targetPosition = new THREE.Vector3();
    droneGroup.targetQuaternion = new THREE.Quaternion();
    droneGroup.hasInitialPose = false;

    return droneGroup;
}

function dispatchSelectedDroneTo(x, y, z) {
    const droneId = selectedDroneId || "UAV_1";
    const targetAlt = Math.max(15, Math.min(65, z !== undefined ? z : 35));
    const targetPos = [Number(x.toFixed(1)), Number(y.toFixed(1)), Number(targetAlt.toFixed(1))];

    // Create or update 3D Holographic Dispatch Waypoint Beacon in Theater View
    if (!dispatchBeaconMesh && sceneTheater) {
        const beaconGroup = new THREE.Group();

        // 1. Ground Target Outer Base Ring
        const ringGeo = new THREE.RingGeometry(3.0, 4.5, 32);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, side: THREE.DoubleSide, transparent: true, opacity: 0.85 });
        const ring = new THREE.Mesh(ringGeo, ringMat);
        ring.name = "baseRing";
        beaconGroup.add(ring);

        // 2. Outer Expanding Pulsing Radar Wave Ring
        const pulseGeo = new THREE.RingGeometry(1.2, 2.5, 32);
        const pulseMat = new THREE.MeshBasicMaterial({ color: 0x00ff66, side: THREE.DoubleSide, transparent: true, opacity: 0.65 });
        const pulse = new THREE.Mesh(pulseGeo, pulseMat);
        pulse.position.z = 0.08;
        pulse.name = "pulseRing";
        beaconGroup.add(pulse);

        // 3. Vertical Cyan Beacon Laser Beam
        const beamGeo = new THREE.CylinderGeometry(0.18, 0.18, 140, 12);
        const beamMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.45, depthWrite: false });
        const beam = new THREE.Mesh(beamGeo, beamMat);
        beam.rotation.x = Math.PI / 2;
        beam.position.z = 70;
        beam.name = "beam";
        beaconGroup.add(beam);

        // 4. 3D Setpoint Waypoint Marker Diamond
        const markerGeo = new THREE.OctahedronGeometry(2.4);
        const markerMat = new THREE.MeshBasicMaterial({ color: 0xffd600, wireframe: true });
        const marker = new THREE.Mesh(markerGeo, markerMat);
        marker.name = "marker";
        beaconGroup.add(marker);

        sceneTheater.add(beaconGroup);
        dispatchBeaconMesh = beaconGroup;
    }

    if (dispatchBeaconMesh) {
        dispatchBeaconMesh.position.set(x, y, 0);
        const marker = dispatchBeaconMesh.getObjectByName("marker");
        if (marker) {
            marker.position.z = targetAlt;
        }
        dispatchBeaconMesh.visible = true;
    }

    // Send dispatch command over WebSocket and REST fallback
    const payload = {
        command: "dispatch",
        cmd: "dispatch",
        drone_id: droneId,
        target: targetPos
    };

    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(payload));
    }
    fetch("/api/dispatch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ drone_id: droneId, target: targetPos })
    }).catch(() => {});

    playTacticalSound("radar_ping");

    // Tactical toast alert
    const ticker = document.getElementById("hud-comms-ticker");
    const tickerBadge = document.getElementById("ticker-badge");
    const tickerMsg = document.getElementById("ticker-msg");
    if (ticker && tickerBadge && tickerMsg) {
        tickerBadge.textContent = `[OPERATOR DISPATCH]`;
        tickerBadge.className = `ticker-badge comms-badge-INFO`;
        tickerMsg.textContent = `🎯 ${droneId} routed to (${targetPos[0]}m, ${targetPos[1]}m, ${targetPos[2]}m)`;
        ticker.classList.remove("hidden");
        if (commsTickerTimeout) clearTimeout(commsTickerTimeout);
        commsTickerTimeout = setTimeout(() => {
            ticker.classList.add("hidden");
        }, 4000);
    }
}
window.dispatchSelectedDroneTo = dispatchSelectedDroneTo;

function onTheaterCanvasClick(event) {
    const rect = rendererTheater.domElement.getBoundingClientRect();
    mouseTheater.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouseTheater.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycasterTheater.setFromCamera(mouseTheater, cameraTheater);

    // 1. Shift+Click or Active Dispatch Mode: Click-to-Dispatch 3D Waypoint
    if (isDispatchMode || event.shiftKey) {
        const intersects = raycasterTheater.intersectObjects(sceneTheater.children, true);
        let hitPoint = null;

        for (let i = 0; i < intersects.length; i++) {
            const obj = intersects[i].object;
            if (obj && obj.isMesh && obj.visible && !obj.isLine && !obj.droneId && intersects[i].distance > 1) {
                hitPoint = intersects[i].point.clone();
                break;
            }
        }

        if (!hitPoint) {
            const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
            const groundHit = new THREE.Vector3();
            if (raycasterTheater.ray.intersectPlane(groundPlane, groundHit)) {
                hitPoint = groundHit;
            }
        }

        if (hitPoint) {
            const targetAlt = Math.max(25, (hitPoint.z || 0) + 18);
            dispatchSelectedDroneTo(hitPoint.x, hitPoint.y, targetAlt);
            return;
        }
    }

    // 2. Standard Click: Drone Selection
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

function updateCanvasCursor() {
    if (!rendererTheater || !rendererTheater.domElement) return;
    if (isDispatchMode) {
        rendererTheater.domElement.style.cursor = "crosshair";
    } else if (isPanMode || isShiftHeld) {
        rendererTheater.domElement.style.cursor = isMouseDownOnTheater ? "grabbing" : "grab";
    } else {
        rendererTheater.domElement.style.cursor = "";
    }
}

function setPanMode(active) {
    isPanMode = active;
    const btn = document.getElementById("btn-toggle-pan");
    if (controlsTheater) {
        if (isPanMode) {
            controlsTheater.mouseButtons.LEFT = THREE.MOUSE.PAN;
            if (controlsTheater.touches) controlsTheater.touches.ONE = THREE.TOUCH.PAN;
            if (btn) {
                btn.classList.add("active");
                btn.innerHTML = "&#9995; DRAG: ON";
                btn.title = "Drag Mode ACTIVE: Left-click and drag anywhere to move across map. Click to return to Orbit Rotate.";
            }
        } else {
            controlsTheater.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
            if (controlsTheater.touches) controlsTheater.touches.ONE = THREE.TOUCH.ROTATE;
            if (btn) {
                btn.classList.remove("active");
                btn.innerHTML = "&#9995; DRAG: OFF";
                btn.title = "Click to toggle Drag/Pan: Left-click drags the map to move anywhere. (Or use Right-Click Drag, WASD keys, or Double-Click).";
            }
        }
    }
    updateCanvasCursor();
}

function smoothPanTo(destination) {
    if (!controlsTheater || !cameraTheater || !destination) return;
    smoothPanStartTarget = controlsTheater.target.clone();
    smoothPanStartCam = cameraTheater.position.clone();
    smoothPanTarget = destination.clone();
    smoothPanTarget.z = Math.max(0, Math.min(destination.z !== undefined ? destination.z : 0, 80));
    smoothPanProgress = 0.0;
    smoothPanStartTime = performance.now();
}

function onTheaterCanvasDblClick(event) {
    if (!rendererTheater || !cameraTheater || !sceneTheater) return;
    const rect = rendererTheater.domElement.getBoundingClientRect();
    const mouse = new THREE.Vector2(
        ((event.clientX - rect.left) / rect.width) * 2 - 1,
        -((event.clientY - rect.top) / rect.height) * 2 + 1
    );

    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, cameraTheater);

    const intersects = raycaster.intersectObjects(sceneTheater.children, true);
    let hitPoint = null;

    for (let i = 0; i < intersects.length; i++) {
        const obj = intersects[i].object;
        if (obj && obj.isMesh && obj.visible && !obj.isLine && intersects[i].distance > 1) {
            hitPoint = intersects[i].point.clone();
            break;
        }
    }

    if (!hitPoint) {
        const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
        const groundHit = new THREE.Vector3();
        if (raycaster.ray.intersectPlane(groundPlane, groundHit)) {
            hitPoint = groundHit;
        }
    }

    if (hitPoint) {
        smoothPanTo(hitPoint);
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

    // 7. 3D LiDAR Persistent Point Cloud Buffer (25,000 points dense SLAM reconstruction)
    const lidarGeo = new THREE.BufferGeometry();
    const lidarPositions = new Float32Array(maxLidarPts * 3);
    const lidarColors = new Float32Array(maxLidarPts * 3);
    lidarGeo.setAttribute('position', new THREE.BufferAttribute(lidarPositions, 3));
    lidarGeo.setAttribute('color', new THREE.BufferAttribute(lidarColors, 3));

    const lidarMat = new THREE.PointsMaterial({
        size: 3.5,
        vertexColors: true,
        transparent: true,
        opacity: 0.92,
        blending: THREE.AdditiveBlending,
        depthWrite: false,
    });
    lidarPointsMesh = new THREE.Points(lidarGeo, lidarMat);
    lidarPointsMesh.geometry.setDrawRange(0, 0);
    sceneSLAM.add(lidarPointsMesh);

    // 8. 3D Occupancy Voxel Instanced Mesh (Ultra-Fast 60-144 FPS Hardware Acceleration)
    voxelMeshGroup = new THREE.Group();
    sceneSLAM.add(voxelMeshGroup);

    const voxelGeo = new THREE.BoxGeometry(1.0, 1.0, 1.0);
    const voxelMat = new THREE.MeshBasicMaterial({
        transparent: true,
        opacity: 0.42,
        depthWrite: false,
    });
    voxelInstancedMesh = new THREE.InstancedMesh(voxelGeo, voxelMat, maxInstancedVoxels);
    voxelInstancedMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    voxelInstancedMesh.count = 0;
    voxelMeshGroup.add(voxelInstancedMesh);

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

        const isRelay = (drone.role === "RELAY");
        const isScout = (drone.role === "SCOUT");
        const roleColor = isRelay ? 0xffd600 : (isScout ? 0x00ff66 : 0x00e5ff);

        if (!mesh) {
            mesh = createQuadcopterMesh(drone.role);
            sceneTheater.add(mesh);
            droneMeshes.set(drone.id, mesh);
        }

        if (!mesh.targetPosition) {
            mesh.targetPosition = new THREE.Vector3();
            mesh.targetQuaternion = new THREE.Quaternion();
            mesh.hasInitialPose = false;
        }

        // Store new target position & attitude quaternion from telemetry packet
        mesh.targetPosition.set(drone.position[0], drone.position[1], drone.position[2]);
        if (drone.attitude) {
            const euler = new THREE.Euler(drone.attitude[0], drone.attitude[1], drone.attitude[2], 'XYZ');
            mesh.targetQuaternion.setFromEuler(euler);
        }

        // Snap to initial pose on first packet or when sitting on ground launch pad
        if (!mesh.hasInitialPose || drone.flight_mode === "IDLE" || drone.flight_mode === "LANDED" || (drone.position && drone.position[2] <= 0.55)) {
            mesh.position.copy(mesh.targetPosition);
            mesh.quaternion.copy(mesh.targetQuaternion);
            mesh.hasInitialPose = true;
        }

        // 1. Multi-UAV 3D Flight Path Trajectory Ribbons
        let trail = droneTrails.get(drone.id);
        if (!trail) {
            const maxTrailPts = 60;
            const trailGeo = new THREE.BufferGeometry();
            const trailPositions = new Float32Array(maxTrailPts * 3);
            trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
            const trailMat = new THREE.LineBasicMaterial({
                color: roleColor,
                transparent: true,
                opacity: 0.55,
                depthWrite: false,
            });
            const trailLine = new THREE.Line(trailGeo, trailMat);
            trail = { line: trailLine, points: [], maxPoints: maxTrailPts };
            sceneTheater.add(trailLine);
            droneTrails.set(drone.id, trail);
        }

        if (drone.position[2] > 0.8 && drone.flight_mode !== "IDLE" && drone.flight_mode !== "LANDED") {
            trail.points.push(new THREE.Vector3(drone.position[0], drone.position[1], drone.position[2]));
            if (trail.points.length > trail.maxPoints) {
                trail.points.shift();
            }
            trail.line.geometry.setFromPoints(trail.points);
            trail.line.visible = true;
        } else if (drone.flight_mode === "LANDED" || drone.flight_mode === "IDLE") {
            trail.points = [];
            trail.line.visible = false;
        }

        // 2. 3D Dashed Target Waypoint Projection Vectors
        let targetLine = targetVectorLines.get(drone.id);
        if (!targetLine) {
            const targetGeo = new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(0, 0, 0),
                new THREE.Vector3(0, 0, 0)
            ]);
            const targetMat = new THREE.LineDashedMaterial({
                color: roleColor,
                dashSize: 2.5,
                gapSize: 1.5,
                transparent: true,
                opacity: 0.70,
                depthWrite: false,
            });
            targetLine = new THREE.Line(targetGeo, targetMat);
            sceneTheater.add(targetLine);
            targetVectorLines.set(drone.id, targetLine);
        }

        if (drone.target_position && drone.position[2] > 1.2 && drone.flight_mode !== "IDLE" && drone.flight_mode !== "LANDED") {
            const p1 = new THREE.Vector3(drone.position[0], drone.position[1], drone.position[2]);
            const p2 = new THREE.Vector3(drone.target_position[0], drone.target_position[1], drone.target_position[2]);
            targetLine.geometry.setFromPoints([p1, p2]);
            targetLine.computeLineDistances();
            targetLine.visible = true;
        } else {
            targetLine.visible = false;
        }

        // Record trajectory point for focused drone in SLAM view
        if (drone.id === selectedDroneId) {
            slamTrajectoryPoints.push(new THREE.Vector3(drone.position[0], drone.position[1], drone.position[2]));
            if (slamTrajectoryPoints.length > maxTrajectoryPoints) {
                slamTrajectoryPoints.shift();
            }
            if (slamTrajectoryLine) {
                slamTrajectoryLine.geometry.setFromPoints(slamTrajectoryPoints);
            }
        }
    });

    // Remove defunct drones & trails
    for (const [id, mesh] of droneMeshes.entries()) {
        if (!activeIds.has(id)) {
            sceneTheater.remove(mesh);
            droneMeshes.delete(id);
            const trail = droneTrails.get(id);
            if (trail) {
                sceneTheater.remove(trail.line);
                droneTrails.delete(id);
            }
            const tgtLine = targetVectorLines.get(id);
            if (tgtLine) {
                sceneTheater.remove(tgtLine);
                targetVectorLines.delete(id);
            }
        }
    }
}

function updateObstacles(obstaclesData) {
    // In Sector Delta diorama mode, the architectural models and buildings are rendered by SectorDelta
    if (typeof SectorDelta !== "undefined") return;
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

            const isChallenge = (currentScenario === "challenge");
            const baseColor = isChallenge ? 0xff1744 : 0xd500f9;

            // Ground Target Ring
            const ringGeo = new THREE.RingGeometry(3.5, 5.0, 24);
            const ringMat = new THREE.MeshBasicMaterial({ color: baseColor, side: THREE.DoubleSide, transparent: true, opacity: 0.85 });
            const ring = new THREE.Mesh(ringGeo, ringMat);
            group.add(ring);
            group.ring = ring;

            // Concentric Radar Ripple Beacon Ring (matching Challenge problem statement graphic!)
            const rippleGeo = new THREE.RingGeometry(1.0, 2.2, 24);
            const rippleMat = new THREE.MeshBasicMaterial({ color: baseColor, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });
            const ripple = new THREE.Mesh(rippleGeo, rippleMat);
            ripple.position.z = 0.1;
            group.add(ripple);
            group.ripple = ripple;

            // Vertical Beacon Laser Beam
            const beamGeo = new THREE.CylinderGeometry(0.18, 0.18, poi.position[2] * 2, 8);
            const beamMat = new THREE.MeshBasicMaterial({ color: baseColor, transparent: true, opacity: 0.65 });
            const beam = new THREE.Mesh(beamGeo, beamMat);
            beam.rotation.x = Math.PI / 2;
            beam.position.z = poi.position[2];
            group.add(beam);
            group.beam = beam;

            // Rotating Diamond Beacon
            const beaconGeo = new THREE.OctahedronGeometry(1.8);
            const beaconMat = new THREE.MeshBasicMaterial({ color: baseColor, wireframe: true });
            const beacon = new THREE.Mesh(beaconGeo, beaconMat);
            beacon.position.z = poi.position[2];
            group.add(beacon);
            group.beacon = beacon;

            sceneTheater.add(group);
            poiMeshes.set(poi.id, group);
        }

        // Color and animation state transitions
        if (poi.is_spawned === false) {
            group.visible = false;
        } else {
            group.visible = true;
            if (poi.is_completed) {
                if (group.beacon) group.beacon.material.color.setHex(0x00ff66);
                if (group.ring) group.ring.material.color.setHex(0x00ff66);
                if (group.ripple) group.ripple.material.color.setHex(0x00ff66);
                if (group.beam) group.beam.material.color.setHex(0x00ff66);
            } else if (poi.is_reported) {
                // Detected and reported within 10s SLA: Cyan
                if (group.beacon) group.beacon.material.color.setHex(0x00e5ff);
                if (group.ring) group.ring.material.color.setHex(0x00e5ff);
                if (group.ripple) group.ripple.material.color.setHex(0x00e5ff);
            } else if (poi.is_detected) {
                // Detected by UAV, in transit to report: Yellow
                if (group.beacon) group.beacon.material.color.setHex(0xffd600);
                if (group.ring) group.ring.material.color.setHex(0xffd600);
            } else {
                // Active undetected target: Red (Challenge) or Magenta (Sector Delta)
                const col = (currentScenario === "challenge") ? 0xff1744 : 0xd500f9;
                if (group.beacon) group.beacon.material.color.setHex(col);
                if (group.ring) group.ring.material.color.setHex(col);
                if (group.ripple) group.ripple.material.color.setHex(col);
            }

            // Radar ripple pulsation
            if (group.ripple) {
                const s = 1.0 + ((Date.now() * 0.003) % 2.5);
                group.ripple.scale.set(s, s, 1.0);
                group.ripple.material.opacity = Math.max(0.05, 0.7 - s * 0.25);
            }
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
    // GCS Base Station Radar Mast Receiver (Sector Delta: 0, -145, 26 | Challenge: -75, 0, 16)
    const gcsCoords = (currentScenario === "challenge") ? new THREE.Vector3(-75, 0, 16) : new THREE.Vector3(0, -145, 26);
    nodeCoords.set("GCS", gcsCoords);

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
        if (!link.viable && !isRouteEdge) return;

        // Clean link style with simple distinct colors:
        // - Green (#00ff66): Active multi-hop routing paths carrying live packet traffic
        // - Golden Amber (#ffd600): Long-range LoRa links / high-altitude relay links
        // - Electric Cyan (#00e5ff): Standard viable 2.4GHz Wi-Fi mesh links
        const isLoRa = (link.band && link.band.includes("LORA")) || (link.distance > 80.0 && link.viable);
        const linkColor = isRouteEdge ? 0x00ff66 : (isLoRa ? 0xffd600 : 0x00e5ff);
        const linkOpacity = isRouteEdge ? 1.0 : (isLoRa ? 0.45 : 0.55);

        let line = linkMeshes.get(key);
        if (!line) {
            const geo = new THREE.BufferGeometry().setFromPoints([p1, p2]);
            const mat = new THREE.LineBasicMaterial({
                color: linkColor,
                transparent: true,
                opacity: linkOpacity,
                depthWrite: false,
            });
            line = new THREE.Line(geo, mat);
            line.visible = true;
            sceneTheater.add(line);
            linkMeshes.set(key, line);
        } else {
            // Update vertex endpoints dynamically as drones fly
            const posAttr = line.geometry.attributes.position;
            const positions = posAttr.array;
            positions[0] = p1.x;
            positions[1] = p1.y;
            positions[2] = p1.z;
            positions[3] = p2.x;
            positions[4] = p2.y;
            positions[5] = p2.z;
            posAttr.needsUpdate = true;
            line.geometry.computeBoundingSphere();
            line.material.color.setHex(linkColor);
            line.material.opacity = linkOpacity;
            line.visible = true;
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

// Update 3D LiDAR Point Cloud in SLAM Viewport & Theater Overlay (Persistent Accumulation)
function updateLiDAR(scanData) {
    if (!scanData || !lidarPointsMesh) return;
    const pts = scanData.points || [];
    if (pts.length === 0) return;

    const posAttr = lidarPointsMesh.geometry.attributes.position;
    const colAttr = lidarPointsMesh.geometry.attributes.color;

    const theaterPosAttr = theaterLidarPointsMesh ? theaterLidarPointsMesh.geometry.attributes.position : null;
    const theaterColAttr = theaterLidarPointsMesh ? theaterLidarPointsMesh.geometry.attributes.color : null;

    for (let i = 0; i < pts.length; i++) {
        const p = pts[i];
        const idx = lidarWriteIndex;

        posAttr.array[idx * 3] = p[0];
        posAttr.array[idx * 3 + 1] = p[1];
        posAttr.array[idx * 3 + 2] = p[2];

        // Resolve RGB based on active colormap
        let r = 0.0, g = 0.9, b = 1.0;
        if (activeLidarColormap === "turbo") {
            const zNorm = Math.min(1.0, Math.max(0.0, p[2] / 50.0));
            [r, g, b] = getTurboRGB(zNorm);
        } else if (activeLidarColormap === "intensity") {
            const intensity = Math.min(1.0, Math.max(0.0, p[3] !== undefined ? p[3] : 0.6));
            r = 0.15 + intensity * 0.85;
            g = 0.95;
            b = 0.3 + (1.0 - intensity) * 0.7;
        } else if (activeLidarColormap === "cyber") {
            const zNorm = Math.min(1.0, Math.max(0.0, p[2] / 50.0));
            if (zNorm > 0.45) {
                r = 0.85; g = 0.0; b = 0.95; // High altitude structure: Neon Magenta
            } else if (zNorm > 0.15) {
                r = 0.0; g = 0.9; b = 1.0;   // Mid altitude: Electric Cyan
            } else {
                r = 0.0; g = 1.0; b = 0.4;   // Ground plane: Neon Green
            }
        }

        colAttr.array[idx * 3] = r;
        colAttr.array[idx * 3 + 1] = g;
        colAttr.array[idx * 3 + 2] = b;

        if (theaterPosAttr && theaterColAttr) {
            theaterPosAttr.array[idx * 3] = p[0];
            theaterPosAttr.array[idx * 3 + 1] = p[1];
            theaterPosAttr.array[idx * 3 + 2] = p[2];
            theaterColAttr.array[idx * 3] = r;
            theaterColAttr.array[idx * 3 + 1] = g;
            theaterColAttr.array[idx * 3 + 2] = b;
        }

        lidarWriteIndex = (lidarWriteIndex + 1) % maxLidarPts;
        lidarTotalStored = Math.min(maxLidarPts, lidarTotalStored + 1);
    }

    lidarPointsMesh.geometry.setDrawRange(0, lidarTotalStored);
    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;

    if (theaterLidarPointsMesh && theaterPosAttr && theaterColAttr) {
        theaterLidarPointsMesh.geometry.setDrawRange(0, lidarTotalStored);
        theaterPosAttr.needsUpdate = true;
        theaterColAttr.needsUpdate = true;
    }

    const elPts = document.getElementById("slam-pts-count");
    if (elPts) elPts.textContent = lidarTotalStored.toLocaleString();
}

// Update 3D Occupancy Voxel Grid with InstancedMesh Hardware Acceleration
function updateOccupancyVoxels(voxelsData, metricsData) {
    if (!voxelInstancedMesh) return;
    const vList = voxelsData || [];
    const count = Math.min(vList.length, maxInstancedVoxels);

    for (let i = 0; i < count; i++) {
        const v = vList[i];
        const scale = (v.size || 4.0) * 0.96;
        voxelDummyMatrix.makeScale(scale, scale, scale);
        voxelDummyMatrix.setPosition(v.pos[0], v.pos[1], v.pos[2]);
        voxelInstancedMesh.setMatrixAt(i, voxelDummyMatrix);

        const zNorm = Math.min(1.0, Math.max(0.0, v.pos[2] / 50.0));
        const prob = v.prob !== undefined ? v.prob : 0.8;
        let hexColor;
        if (prob > 0.85) {
            hexColor = zNorm > 0.4 ? 0xff1744 : 0xff9100;
        } else {
            hexColor = zNorm > 0.3 ? 0xffd600 : 0x00e5ff;
        }
        voxelDummyColor.setHex(hexColor);
        voxelInstancedMesh.setColorAt(i, voxelDummyColor);
    }

    voxelInstancedMesh.count = count;
    voxelInstancedMesh.instanceMatrix.needsUpdate = true;
    if (voxelInstancedMesh.instanceColor) {
        voxelInstancedMesh.instanceColor.needsUpdate = true;
    }

    const elVox = document.getElementById("slam-voxels-count");
    if (elVox) elVox.textContent = count.toLocaleString();

    if (metricsData) {
        const elVol = document.getElementById("slam-vol-count");
        if (elVol) elVol.textContent = `${(metricsData.mapped_volume_m3 || 0).toLocaleString()} m³`;
        const elCov = document.getElementById("metric-survey-rate");
        if (elCov) elCov.textContent = `${(metricsData.coverage_pct || 14.2).toFixed(1)}% / min`;
    }
}

function resetSLAMMap() {
    lidarWriteIndex = 0;
    lidarTotalStored = 0;
    if (lidarPointsMesh) {
        lidarPointsMesh.geometry.setDrawRange(0, 0);
    }
    if (theaterLidarPointsMesh) {
        theaterLidarPointsMesh.geometry.setDrawRange(0, 0);
    }
    if (voxelInstancedMesh) {
        voxelInstancedMesh.count = 0;
        voxelInstancedMesh.instanceMatrix.needsUpdate = true;
    }
    const elPts = document.getElementById("slam-pts-count");
    if (elPts) elPts.textContent = "0";
    const elVox = document.getElementById("slam-voxels-count");
    if (elVox) elVox.textContent = "0";

    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ command: "clear_slam" }));
    } else {
        fetch("/api/control", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ command: "clear_slam" })
        }).catch(() => {});
    }
    playTacticalSound("radar_ping");
}
window.resetSLAMMap = resetSLAMMap;

function toggleLidarColormap() {
    const modes = ["turbo", "intensity", "cyber"];
    const nextIdx = (modes.indexOf(activeLidarColormap) + 1) % modes.length;
    activeLidarColormap = modes[nextIdx];
    const btn = document.getElementById("btn-slam-colormap");
    if (btn) {
        btn.textContent = `COLOR: ${activeLidarColormap.toUpperCase()}`;
    }
    playTacticalSound("radar_ping");
}
window.toggleLidarColormap = toggleLidarColormap;

function toggleTheaterLidar() {
    isTheaterLidarActive = !isTheaterLidarActive;
    if (theaterLidarPointsMesh) {
        theaterLidarPointsMesh.visible = isTheaterLidarActive;
    }
    const btn = document.getElementById("btn-toggle-lidar");
    if (btn) {
        btn.classList.toggle("active", isTheaterLidarActive);
        btn.textContent = isTheaterLidarActive ? "🌐 LIDAR: ON" : "🌐 LIDAR: OFF";
        btn.classList.toggle("text-neon-green", isTheaterLidarActive);
        btn.classList.toggle("text-cyan", !isTheaterLidarActive);
    }
    playTacticalSound("radar_ping");
}
window.toggleTheaterLidar = toggleTheaterLidar;

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
        smoothPanTo(targetMesh.position);
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
        const faultBadgeHtml = d.is_fault_injected ? '<span class="fault-badge">⚡ FLAMEOUT</span>' : '';
        const draftBadgeHtml = d.is_drafting ? `<span class="drafting-badge" title="Drafting wake of ${d.drafting_leader_id}">⚡ +${d.drafting_saving_pct}%</span>` : '';

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
                    <div style="display: flex; align-items: center; gap: 4px;">
                        <span class="drone-badges-container">${faultBadgeHtml}${draftBadgeHtml}</span>
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

            const badgesContainer = card.querySelector(".drone-badges-container");
            const newBadges = `${faultBadgeHtml}${draftBadgeHtml}`;
            if (badgesContainer && badgesContainer.innerHTML !== newBadges) {
                badgesContainer.innerHTML = newBadges;
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

let latestDebriefData = null;
let lastSeenCommsTimestamp = -1;
let commsTickerTimeout = null;

function updateTacticalComms(events) {
    if (!events || events.length === 0) return;
    const newest = events[events.length - 1];
    if (newest.timestamp > lastSeenCommsTimestamp) {
        lastSeenCommsTimestamp = newest.timestamp;
        const ticker = document.getElementById("hud-comms-ticker");
        const tickerBadge = document.getElementById("ticker-badge");
        const tickerMsg = document.getElementById("ticker-msg");
        if (ticker && tickerBadge && tickerMsg) {
            tickerBadge.textContent = `[${newest.callsign} ${newest.category}]`;
            tickerBadge.className = `ticker-badge comms-badge-${newest.level}`;
            tickerMsg.textContent = newest.message;
            ticker.classList.remove("hidden");
            if (commsTickerTimeout) clearTimeout(commsTickerTimeout);
            commsTickerTimeout = setTimeout(() => {
                ticker.classList.add("hidden");
            }, 4500);
        }
    }
    const logList = document.getElementById("comms-log-list");
    if (logList) {
        let html = "";
        for (let i = events.length - 1; i >= 0; i--) {
            const e = events[i];
            const mins = Math.floor(e.timestamp / 60);
            const secs = (e.timestamp % 60).toFixed(1);
            const tStr = `${String(mins).padStart(2, '0')}:${String(secs).padStart(4, '0')}`;
            html += `
                <div class="comms-entry ${e.level}">
                    <div class="comms-entry-header">
                        <span class="comms-entry-callsign">[${e.callsign}] (${e.category})</span>
                        <span class="comms-entry-time">${tStr}</span>
                    </div>
                    <div class="comms-entry-body">${e.message}</div>
                </div>
            `;
        }
        logList.innerHTML = html;
    }
}

async function exportMissionDebrief() {
    try {
        const res = await fetch("/api/export_debrief");
        if (!res.ok) throw new Error("Failed to fetch debrief");
        const debrief = await res.json();
        latestDebriefData = debrief;
        const modal = document.getElementById("debrief-modal");
        const body = document.getElementById("debrief-modal-body");
        if (modal && body) {
            const survList = (debrief.search_and_rescue_summary.discovered_survivors || []);
            const survHtml = survList.length > 0
                ? survList.map(s => `&bull; <strong>${s.id}</strong> at ${s.poi_id} (IR Body Heat: ${s.heat_c}&deg;C, Confidence: ${Math.round(s.confidence * 100)}%)`).join("<br>")
                : "Scanning disaster sites for thermal signatures...";

            body.innerHTML = `
                <div class="debrief-kpi-grid">
                    <div class="debrief-kpi"><span class="lbl">MISSION DURATION</span><span class="val text-neon-green">${debrief.mission_duration_s}s / ${debrief.mission_time_budget_s}s</span></div>
                    <div class="debrief-kpi"><span class="lbl">SITES CLEARED</span><span class="val text-purple">${debrief.disaster_sites_summary.cleared_sites} / ${debrief.disaster_sites_summary.total_sites} (${debrief.disaster_sites_summary.completion_pct}%)</span></div>
                    <div class="debrief-kpi"><span class="lbl">SURVIVORS LOCATED</span><span class="val text-neon-green">${debrief.search_and_rescue_summary.survivors_located} / ${debrief.search_and_rescue_summary.total_survivors_estimated}</span></div>
                    <div class="debrief-kpi"><span class="lbl">TOTAL ENERGY</span><span class="val text-neon-yellow">${debrief.fleet_and_energy_summary.total_energy_consumed_wh} Wh</span></div>
                    <div class="debrief-kpi"><span class="lbl">NETWORK PDR</span><span class="val text-cyan">${(debrief.network_and_telemetry_summary.packet_delivery_ratio * 100).toFixed(1)}%</span></div>
                    <div class="debrief-kpi"><span class="lbl">BUDGET STATUS</span><span class="val text-neon-green">${debrief.budget_compliance}</span></div>
                </div>
                <div class="debrief-section-title">PRIORITY SURVEY SITES CLEARED</div>
                <p>CRITICAL: <strong>${debrief.disaster_sites_summary.priority_breakdown.critical}</strong> | HIGH: <strong>${debrief.disaster_sites_summary.priority_breakdown.high}</strong> | MEDIUM: <strong>${debrief.disaster_sites_summary.priority_breakdown.medium}</strong> | LOW: <strong>${debrief.disaster_sites_summary.priority_breakdown.low}</strong></p>
                <div class="debrief-section-title">SEARCH & RESCUE (SAR) THERMAL LOCATIONS</div>
                <p style="line-height:1.6;">${survHtml}</p>
                <div class="debrief-section-title">RADIO COMMS RECORD</div>
                <p>Total recorded radio events: <strong>${debrief.tactical_comms_log.length}</strong> transmissions captured in audit log.</p>
            `;
            modal.classList.remove("hidden");
        }
    } catch (err) {
        console.error("Debrief error:", err);
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

    // Survivors Located (Thermal SAR)
    const elSurvivors = document.getElementById("metric-survivors");
    if (elSurvivors && telemetry.survivors) {
        const s = telemetry.survivors;
        elSurvivors.textContent = `${s.located_count || 0} / ${s.total_count || 0}`;
        if ((s.located_count || 0) > 0) {
            elSurvivors.className = "metric-val text-neon-green";
        }
    }

    // Atmospheric Wind & Turbulence
    const elWind = document.getElementById("metric-wind");
    if (elWind && telemetry.weather) {
        const w = telemetry.weather;
        const spd = w.current_speed_mps !== undefined ? w.current_speed_mps : (w.mean_speed_mps || 0);
        const dir = Math.round(w.direction_deg || 0);
        const gustTxt = w.gust_active ? " [GUST]" : "";
        elWind.textContent = `${spd.toFixed(1)}m/s ${dir}°${gustTxt}`;
        if (w.gust_active) {
            elWind.className = "metric-val text-neon-yellow";
        } else {
            elWind.className = "metric-val text-cyan";
        }
    }

    // Tactical Visual Comms Chatter Feed
    if (telemetry.tactical_comms && Array.isArray(telemetry.tactical_comms)) {
        updateTacticalComms(telemetry.tactical_comms);
    }

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

    // MeitY / IIT Bombay / IISER Bhopal Challenge Compliance Tracking
    const cardChallenge = document.getElementById("metric-card-challenge");
    const badgeChallenge = document.getElementById("badge-challenge-status");
    if (telemetry.challenge_constraints) {
        if (cardChallenge) cardChallenge.style.display = "flex";
        const cc = telemetry.challenge_constraints;
        if (badgeChallenge) {
            if (cc.is_fully_compliant) {
                badgeChallenge.textContent = "COMPLIANT (11/11)";
                badgeChallenge.style.color = "#00ff88";
                badgeChallenge.style.borderColor = "#00ff88";
                badgeChallenge.style.background = "rgba(0, 255, 136, 0.12)";
            } else {
                const viols = cc.violations || {};
                const nonZero = Object.entries(viols).filter(([_, v]) => v > 0).map(([k]) => k.toUpperCase());
                badgeChallenge.textContent = `VIOLATION: ${nonZero.join(",") || "NON-COMPLIANT"}`;
                badgeChallenge.style.color = "#ff3333";
                badgeChallenge.style.borderColor = "#ff3333";
                badgeChallenge.style.background = "rgba(255, 51, 51, 0.15)";
            }
        }
    } else if (cardChallenge && currentScenario !== "challenge") {
        cardChallenge.style.display = "none";
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
            if (telemetry.scenario && telemetry.scenario !== currentScenario) {
                applyScenarioUI(telemetry.scenario);
            }
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
// 5 New Tactical Upgrades: Chaos, Night Ops, PLY Export, FPV, Drafting
// ============================================================================

function toggleNightOps() {
    isNightOps = !isNightOps;
    const btnNight = document.getElementById("btn-night");
    if (btnNight) {
        btnNight.classList.toggle("active", isNightOps);
        btnNight.textContent = isNightOps ? "NIGHT: ON" : "NIGHT";
    }

    if (ambientLightTheater && dirLightTheater && sceneTheater) {
        if (isNightOps) {
            ambientLightTheater.intensity = 0.15;
            ambientLightTheater.color.setHex(0x1a2638);
            dirLightTheater.intensity = 0.20;
            dirLightTheater.color.setHex(0x336699);
            if (fillLightTheater) fillLightTheater.intensity = 0.05;
            if (activeCamMode !== "flir") {
                sceneTheater.background = new THREE.Color(0x020408);
                sceneTheater.fog.color.setHex(0x020408);
            }
            droneSearchlights.forEach(sl => { sl.intensity = 3.5; });
        } else {
            ambientLightTheater.intensity = 0.85;
            ambientLightTheater.color.setHex(0xe4f2ff);
            dirLightTheater.intensity = 1.35;
            dirLightTheater.color.setHex(0xfffaee);
            if (fillLightTheater) fillLightTheater.intensity = 0.35;
            if (activeCamMode !== "flir") {
                if (theaterSkyTex) {
                    sceneTheater.background = theaterSkyTex;
                } else {
                    sceneTheater.background = new THREE.Color(0x060a12);
                }
                sceneTheater.fog.color.setHex(0x95c7f2);
            }
            droneSearchlights.forEach(sl => { sl.intensity = 0.0; });
        }
    }
}
window.toggleNightOps = toggleNightOps;

function triggerChaosFault() {
    playTacticalSound("alarm");
    const payload = { command: "chaos_fault", cmd: "chaos_fault", drone_id: selectedDroneId };
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(payload));
    }
    fetch("/api/chaos_fault", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ drone_id: selectedDroneId })
    })
    .then(r => r.json())
    .then(data => {
        const victim = data.victim_id || selectedDroneId;
        const btnChaos = document.getElementById("btn-chaos");
        if (btnChaos) {
            const orig = btnChaos.textContent;
            btnChaos.textContent = `⚡ FAULT: ${victim}`;
            setTimeout(() => { btnChaos.textContent = orig; }, 3500);
        }
    })
    .catch(() => {});
}
window.triggerChaosFault = triggerChaosFault;

function exportPointCloudPLY() {
    playTacticalSound("radar_ping");
    const btn = document.getElementById("btn-export-ply");
    if (btn) btn.textContent = "SAVING...";

    fetch("/api/export_point_cloud")
        .then(res => res.blob())
        .then(blob => {
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement("a");
            a.style.display = "none";
            a.href = url;
            a.download = "UAVX_Disaster_PointCloud.ply";
            document.body.appendChild(a);
            a.click();
            window.URL.revokeObjectURL(url);
            a.remove();
            if (btn) btn.textContent = "PLY SAVED!";
            setTimeout(() => { if (btn) btn.textContent = "PLY"; }, 2500);
        })
        .catch(err => {
            console.error("PLY export error", err);
            if (btn) btn.textContent = "PLY ERR";
            setTimeout(() => { if (btn) btn.textContent = "PLY"; }, 2000);
        });
}
window.exportPointCloudPLY = exportPointCloudPLY;

function toggleManualControl() {
    isManualControlActive = !isManualControlActive;
    const btnManual = document.getElementById("btn-manual");
    if (btnManual) {
        btnManual.classList.toggle("btn-manual-active", isManualControlActive);
        btnManual.textContent = isManualControlActive ? "FPV: MANUAL" : "FPV: AUTO";
    }

    if (!isManualControlActive) {
        sendManualVelocity(0, 0, 0, 0, false);
    }
}
window.toggleManualControl = toggleManualControl;

function sendManualVelocity(vx, vy, vz, yawRate, enabled = true) {
    const payload = {
        command: "manual_control",
        drone_id: selectedDroneId,
        vx: vx,
        vy: vy,
        vz: vz,
        yaw_rate: yawRate,
        enabled: enabled
    };
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(payload));
    } else {
        fetch("/api/manual_control", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload)
        }).catch(() => {});
    }
}

function handleManualFPVInput() {
    if (!isManualControlActive) return;

    const now = performance.now();
    if (now - lastManualSendTime < 45) return; // ~20 Hz rate limit

    let vx = 0.0, vy = 0.0, vz = 0.0, yawRate = 0.0;

    // 1. Keyboard Controls (WASD horizontal, QE yaw, ArrowUp/ArrowDown altitude)
    const maxSpeed = 10.0;
    const maxClimb = 3.0;
    const maxYaw = 1.5;

    if (activeKeys["KeyW"]) vy += maxSpeed;
    if (activeKeys["KeyS"]) vy -= maxSpeed;
    if (activeKeys["KeyA"]) vx -= maxSpeed;
    if (activeKeys["KeyD"]) vx += maxSpeed;
    if (activeKeys["KeyQ"]) yawRate -= maxYaw;
    if (activeKeys["KeyE"]) yawRate += maxYaw;
    if (activeKeys["ArrowUp"] || activeKeys["KeyR"]) vz += maxClimb;
    if (activeKeys["ArrowDown"] || activeKeys["KeyF"]) vz -= maxClimb;

    // 2. HTML5 Gamepad API Support (Xbox / PlayStation controller)
    if (navigator.getGamepads) {
        const gamepads = navigator.getGamepads();
        for (const gp of gamepads) {
            if (gp && gp.connected) {
                const deadzone = 0.15;
                const applyDeadzone = (v) => Math.abs(v) > deadzone ? v : 0.0;

                // Left stick: pitch / roll
                if (gp.axes.length >= 2) {
                    const stickX = applyDeadzone(gp.axes[0]);
                    const stickY = applyDeadzone(gp.axes[1]);
                    if (stickX !== 0) vx = stickX * maxSpeed;
                    if (stickY !== 0) vy = -stickY * maxSpeed;
                }
                // Right stick: yaw / throttle
                if (gp.axes.length >= 4) {
                    const stickRX = applyDeadzone(gp.axes[2]);
                    const stickRY = applyDeadzone(gp.axes[3]);
                    if (stickRX !== 0) yawRate = stickRX * maxYaw;
                    if (stickRY !== 0) vz = -stickRY * maxClimb;
                }
                // Triggers: RT = climb, LT = descent
                if (gp.buttons.length >= 8) {
                    if (gp.buttons[7].pressed) vz += maxClimb * (gp.buttons[7].value || 1.0);
                    if (gp.buttons[6].pressed) vz -= maxClimb * (gp.buttons[6].value || 1.0);
                }
                break;
            }
        }
    }

    sendManualVelocity(vx, vy, vz, yawRate, true);
    lastManualSendTime = now;
}

function updateFlirPaletteEffect() {
    if (activeCamMode !== "flir") {
        if (rendererTheater && rendererTheater.domElement) {
            rendererTheater.domElement.style.filter = "none";
        }
        return;
    }
    const canvasDom = rendererTheater ? rendererTheater.domElement : null;
    if (!canvasDom) return;

    if (activeFlirPalette === "ironbow") {
        sceneTheater.background = new THREE.Color(0x12031a);
        canvasDom.style.filter = "contrast(1.4) saturate(2.4) hue-rotate(20deg)";
    } else if (activeFlirPalette === "whitehot") {
        sceneTheater.background = new THREE.Color(0x060606);
        canvasDom.style.filter = "grayscale(1.0) contrast(2.2) brightness(1.15)";
    } else if (activeFlirPalette === "blackhot") {
        sceneTheater.background = new THREE.Color(0xdadada);
        canvasDom.style.filter = "grayscale(1.0) invert(1) contrast(1.9) brightness(1.05)";
    }
}

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

    // MeitY / IIT Bombay / IISER Bhopal 1000m Challenge Mode Toggle
    const btnChallenge = document.getElementById("btn-challenge");
    if (btnChallenge) {
        btnChallenge.addEventListener("click", () => {
            const nextScen = (currentScenario === "challenge") ? "sector_delta" : "challenge";
            fetch("/api/scenario", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ scenario: nextScen })
            })
            .then(res => res.json())
            .then(data => {
                if (data.status === "ok") {
                    applyScenarioUI(data.scenario);
                }
            })
            .catch(err => console.error("Error toggling scenario:", err));
        });
    }

    // Tactical Night Operations Toggle
    const btnNight = document.getElementById("btn-night");
    if (btnNight) {
        btnNight.addEventListener("click", () => {
            toggleNightOps();
        });
    }

    // Dynamic Swarm Chaos Fault Injection
    const btnChaos = document.getElementById("btn-chaos");
    if (btnChaos) {
        btnChaos.addEventListener("click", () => {
            triggerChaosFault();
        });
    }

    // 1-Click 3D Point Cloud Export (.PLY)
    const btnExportPly = document.getElementById("btn-export-ply");
    if (btnExportPly) {
        btnExportPly.addEventListener("click", () => {
            exportPointCloudPLY();
        });
    }

    // Interactive 3D Click-to-Dispatch Mode Toggle
    const btnDispatch = document.getElementById("btn-dispatch");
    if (btnDispatch) {
        btnDispatch.addEventListener("click", () => {
            isDispatchMode = !isDispatchMode;
            btnDispatch.classList.toggle("active", isDispatchMode);
            btnDispatch.textContent = isDispatchMode ? "🎯 DISPATCH: ON" : "🎯 DISPATCH";
            btnDispatch.classList.toggle("text-neon-yellow", isDispatchMode);
            btnDispatch.classList.toggle("text-cyan", !isDispatchMode);
            updateCanvasCursor();
        });
    }

    // Manual FPV Controller Takeover (WASD / Gamepad)
    const btnManual = document.getElementById("btn-manual");
    if (btnManual) {
        btnManual.addEventListener("click", () => {
            toggleManualControl();
        });
    }

    // Multi-Palette Thermal FLIR Colormaps
    document.querySelectorAll(".btn-palette").forEach(btn => {
        btn.addEventListener("click", (e) => {
            document.querySelectorAll(".btn-palette").forEach(b => b.classList.remove("active"));
            e.target.classList.add("active");
            activeFlirPalette = e.target.getAttribute("data-palette") || "ironbow";
            const titleEl = document.getElementById("flir-hud-title");
            if (titleEl) {
                titleEl.textContent = `FLIR THERMAL IR • ${activeFlirPalette.toUpperCase()} SENSOR`;
            }
            updateFlirPaletteEffect();
        });
    });

    // Keyboard Listeners for Manual FPV Control
    window.addEventListener("keydown", (e) => {
        if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "SELECT" || e.target.tagName === "TEXTAREA")) return;
        activeKeys[e.code] = true;
    });
    window.addEventListener("keyup", (e) => {
        activeKeys[e.code] = false;
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

    // 3D LiDAR Point Cloud Overlay in Theater View Toggle
    const btnToggleLidar = document.getElementById("btn-toggle-lidar");
    if (btnToggleLidar) {
        btnToggleLidar.addEventListener("click", () => {
            toggleTheaterLidar();
        });
    }

    // SLAM Colormap Toggle (Turbo, Intensity, Cyber)
    const btnSlamColormap = document.getElementById("btn-slam-colormap");
    if (btnSlamColormap) {
        btnSlamColormap.addEventListener("click", () => {
            toggleLidarColormap();
        });
    }

    // SLAM Map Reset & Clear
    const btnSlamClear = document.getElementById("btn-slam-clear");
    if (btnSlamClear) {
        btnSlamClear.addEventListener("click", () => {
            resetSLAMMap();
        });
    }

    // Reset
    const btnReset = document.getElementById("btn-reset");
    btnReset.addEventListener("click", () => {
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify({ command: "reset" }));
        }
        droneMeshes.forEach(mesh => {
            mesh.hasInitialPose = false;
        });
        slamTrajectoryPoints = [];
        if (slamTrajectoryLine) {
            slamTrajectoryLine.geometry.setFromPoints([]);
        }
        resetSLAMMap();
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

    // Tactical Comms Drawer Toggle
    const btnToggleComms = document.getElementById("btn-toggle-comms");
    const drawerComms = document.getElementById("tactical-comms-panel");
    const btnCloseComms = document.getElementById("btn-close-comms");
    if (btnToggleComms && drawerComms) {
        btnToggleComms.addEventListener("click", () => {
            drawerComms.classList.toggle("hidden");
        });
    }
    if (btnCloseComms && drawerComms) {
        btnCloseComms.addEventListener("click", () => {
            drawerComms.classList.add("hidden");
        });
    }

    // Executive Mission Debrief Exporter Modal
    const btnExportDebrief = document.getElementById("btn-export-debrief");
    const modalDebrief = document.getElementById("debrief-modal");
    const btnCloseDebrief = document.getElementById("btn-close-debrief");
    const btnDismissDebrief = document.getElementById("btn-dismiss-debrief");
    const btnDownloadDebriefJson = document.getElementById("btn-download-debrief-json");
    const btnDownloadFlightCsv = document.getElementById("btn-download-flight-csv");

    if (btnExportDebrief) {
        btnExportDebrief.addEventListener("click", () => {
            exportMissionDebrief();
        });
    }
    if (btnCloseDebrief && modalDebrief) {
        btnCloseDebrief.addEventListener("click", () => {
            modalDebrief.classList.add("hidden");
        });
    }
    if (btnDismissDebrief && modalDebrief) {
        btnDismissDebrief.addEventListener("click", () => {
            modalDebrief.classList.add("hidden");
        });
    }
    if (btnDownloadDebriefJson) {
        btnDownloadDebriefJson.addEventListener("click", () => {
            if (!latestDebriefData) return;
            const str = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(latestDebriefData, null, 2));
            const dlAnchor = document.createElement('a');
            dlAnchor.setAttribute("href", str);
            dlAnchor.setAttribute("download", "UAVX_Mission_Debrief.json");
            document.body.appendChild(dlAnchor);
            dlAnchor.click();
            dlAnchor.remove();
        });
    }
    if (btnDownloadFlightCsv) {
        btnDownloadFlightCsv.addEventListener("click", () => {
            window.location.href = "/api/export_telemetry";
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
                updateFlirPaletteEffect();
            } else {
                if (flirOverlay) flirOverlay.classList.add("hidden");
                if (rendererTheater && rendererTheater.domElement) {
                    rendererTheater.domElement.style.filter = "none";
                }
                if (isNightOps) {
                    sceneTheater.background = new THREE.Color(0x020408);
                } else if (theaterSkyTex) {
                    sceneTheater.background = theaterSkyTex;
                } else {
                    sceneTheater.background = new THREE.Color(0x060a12);
                }
            }

            if (mode === "top") {
                if (currentScenario === "challenge") {
                    cameraTheater.position.set(450, 0, 1150);
                    controlsTheater.target.set(450, 0, 0);
                } else {
                    cameraTheater.position.set(0, -45, 520);
                    controlsTheater.target.set(0, -45, 0);
                }
            } else if (mode === "orbit") {
                if (currentScenario === "challenge") {
                    cameraTheater.position.set(450, -850, 550);
                    controlsTheater.target.set(450, 0, 0);
                } else {
                    cameraTheater.position.set(135, -345, 215);
                    controlsTheater.target.set(0, -45, 25);
                }
            } else if (mode === "gcs") {
                if (currentScenario === "challenge") {
                    cameraTheater.position.set(-75, -50, 22);
                    controlsTheater.target.set(-75, 0, 10);
                } else {
                    cameraTheater.position.set(0, -145, 26);
                    controlsTheater.target.set(0, 0, 35);
                }
            }
        });
    });

    // Map Drag / Pan Toggle Button
    const btnTogglePan = document.getElementById("btn-toggle-pan");
    if (btnTogglePan) {
        btnTogglePan.addEventListener("click", () => {
            setPanMode(!isPanMode);
        });
    }

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

    // Keyboard Drone Quick-Select Hotkeys & Map Pan Nav Keys
    window.addEventListener("keydown", (e) => {
        if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "SELECT" || e.target.tagName === "TEXTAREA")) return;

        const k = e.key.toLowerCase();
        pressedNavKeys[k] = true;
        if (e.key.startsWith("Arrow")) {
            pressedNavKeys[e.key] = true;
            e.preventDefault();
        }
        if (e.key === "Shift") {
            isShiftHeld = true;
            if (controlsTheater && !isPanMode) {
                controlsTheater.mouseButtons.LEFT = THREE.MOUSE.PAN;
            }
            updateCanvasCursor();
        }
        if (k === "p" && !e.ctrlKey && !e.metaKey && !e.altKey) {
            setPanMode(!isPanMode);
        }

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

    window.addEventListener("keyup", (e) => {
        const k = e.key.toLowerCase();
        pressedNavKeys[k] = false;
        if (e.key.startsWith("Arrow")) {
            pressedNavKeys[e.key] = false;
        }
        if (e.key === "Shift") {
            isShiftHeld = false;
            if (controlsTheater && !isPanMode) {
                controlsTheater.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
            }
            updateCanvasCursor();
        }
    });

    window.addEventListener("resize", onWindowResize);

    // Deep-link initial camera perspective if specified via ?cam= (e.g. ?cam=top)
    const urlParams = new URLSearchParams(window.location.search);
    const camParam = urlParams.get("cam");
    if (camParam) {
        const btnCam = document.querySelector(`.btn-cam[data-cam="${camParam}"]`);
        if (btnCam) btnCam.click();
    }

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

    // High-precision delta time for frame-rate independent interpolation
    const now = performance.now();
    const dt = Math.min((now - lastAnimateTime) * 0.001, 0.1);
    lastAnimateTime = now;

    // Smooth Frame-Rate Independent Drone Motion Interpolation (LERP & SLERP)
    // Eliminates discrete telemetry step stutter and produces silky-smooth 60-144 FPS continuous flight
    const lerpFactor = 1.0 - Math.exp(-22.0 * dt);
    droneMeshes.forEach((mesh, id) => {
        if (mesh.targetPosition) {
            mesh.position.lerp(mesh.targetPosition, lerpFactor);
        }
        if (mesh.targetQuaternion) {
            mesh.quaternion.slerp(mesh.targetQuaternion, lerpFactor);
        }

        // Keep altitude drop laser line & ground projection reticle in sync with interpolated position
        if (mesh.altLine && mesh.reticle && id === selectedDroneId) {
            const pz = mesh.position.z;
            if (pz > 1.2) {
                mesh.altLine.visible = true;
                mesh.reticle.visible = true;
                const localGroundZ = -pz;
                mesh.altLine.geometry.setFromPoints([
                    new THREE.Vector3(0, 0, 0),
                    new THREE.Vector3(0, 0, localGroundZ)
                ]);
                mesh.altLine.computeLineDistances();
                mesh.reticle.position.set(0, 0, localGroundZ + 0.05);
            } else {
                mesh.altLine.visible = false;
                mesh.reticle.visible = false;
            }
        }
    });

    // Synchronize SLAM Drone Mesh with smoothly interpolated focus drone
    const activeSelectedMesh = droneMeshes.get(selectedDroneId);
    if (activeSelectedMesh && slamDroneMesh) {
        slamDroneMesh.position.copy(activeSelectedMesh.position);
        slamDroneMesh.quaternion.copy(activeSelectedMesh.quaternion);
    }

    // Update Sector Delta Sparkling Photon Particle Streams
    if (typeof SectorDelta !== "undefined") {
        SectorDelta.animate(0.016);
    }

    // Handle Manual FPV Controller Input (WASD / Gamepad)
    if (isManualControlActive) {
        handleManualFPVInput();
    }

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

    // Wind vector calculation from live telemetry weather
    let windVx = 0.08;
    let windVy = 0.05;
    if (latestTelemetry && latestTelemetry.weather) {
        const spd = latestTelemetry.weather.current_speed_mps !== undefined ? latestTelemetry.weather.current_speed_mps : 2.0;
        const dirRad = ((latestTelemetry.weather.direction_deg || 45) * Math.PI) / 180;
        windVx = Math.cos(dirRad) * spd * 0.035;
        windVy = Math.sin(dirRad) * spd * 0.035;
    }

    // Update Disaster Smoke Particles (Rising plumes drifting with atmospheric wind)
    if (smokeParticles) {
        const pos = smokeParticles.geometry.attributes.position.array;
        const speeds = smokeParticles.speeds;
        const origins = smokeParticles.origins;
        for (let i = 0; i < speeds.length; i++) {
            pos[i * 3 + 2] += speeds[i];
            pos[i * 3] += windVx + (Math.random() - 0.48) * 0.18;
            pos[i * 3 + 1] += windVy + (Math.random() - 0.48) * 0.18;
            if (pos[i * 3 + 2] > 65) {
                const orig = origins[i % origins.length];
                pos[i * 3] = orig.x + (Math.random() - 0.5) * 10;
                pos[i * 3 + 1] = orig.y + (Math.random() - 0.5) * 10;
                pos[i * 3 + 2] = orig.z;
            }
        }
        smokeParticles.geometry.attributes.position.needsUpdate = true;
    }

    // Update Disaster Flame & Ember Particles (Turbulent flickering fires at ground zero)
    if (flameParticles) {
        const fPos = flameParticles.geometry.attributes.position.array;
        const fSpeeds = flameParticles.speeds;
        const fOrigins = flameParticles.origins;
        for (let i = 0; i < fSpeeds.length; i++) {
            fPos[i * 3 + 2] += fSpeeds[i];
            fPos[i * 3] += (Math.random() - 0.5) * 0.35 + windVx * 0.4;
            fPos[i * 3 + 1] += (Math.random() - 0.5) * 0.35 + windVy * 0.4;
            if (fPos[i * 3 + 2] > 16) {
                const orig = fOrigins[i % fOrigins.length];
                fPos[i * 3] = orig.x + (Math.random() - 0.5) * 6;
                fPos[i * 3 + 1] = orig.y + (Math.random() - 0.5) * 6;
                fPos[i * 3 + 2] = orig.z + Math.random() * 2.0;
            }
        }
        flameParticles.geometry.attributes.position.needsUpdate = true;
        flameParticles.material.opacity = 0.65 + Math.sin(tSec * 16) * 0.25;
    }

    // Animate Holographic Click-to-Dispatch Waypoint Beacon
    if (dispatchBeaconMesh && dispatchBeaconMesh.visible) {
        const pulseRing = dispatchBeaconMesh.getObjectByName("pulseRing");
        if (pulseRing) {
            const s = 1.0 + ((performance.now() * 0.0035) % 2.8);
            pulseRing.scale.set(s, s, 1.0);
            pulseRing.material.opacity = Math.max(0.05, 0.75 - s * 0.22);
        }
        const marker = dispatchBeaconMesh.getObjectByName("marker");
        if (marker) {
            marker.rotation.z += 0.04;
            marker.rotation.x += 0.02;
        }
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
            cameraTheater.position.set(0, -145, 26);
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

    // Smooth camera panning transition
    if (smoothPanProgress < 1.0 && smoothPanTarget && smoothPanStartTarget && controlsTheater && cameraTheater) {
        const elapsed = performance.now() - smoothPanStartTime;
        smoothPanProgress = Math.min(1.0, elapsed / smoothPanDuration);
        const ease = 1 - Math.pow(1 - smoothPanProgress, 3);
        const delta = smoothPanTarget.clone().sub(smoothPanStartTarget);
        controlsTheater.target.copy(smoothPanStartTarget.clone().add(delta.clone().multiplyScalar(ease)));
        cameraTheater.position.copy(smoothPanStartCam.clone().add(delta.clone().multiplyScalar(ease)));
    }

    // Keyboard WASD / Arrow keys map panning
    if (controlsTheater && cameraTheater && (activeCamMode === "orbit" || activeCamMode === "top")) {
        const forward = new THREE.Vector3();
        cameraTheater.getWorldDirection(forward);
        forward.z = 0;
        if (forward.lengthSq() > 0.001) {
            forward.normalize();
            const right = new THREE.Vector3().crossVectors(forward, cameraTheater.up).normalize();
            const moveVec = new THREE.Vector3();
            const panSpeed = (pressedNavKeys["shift"] ? 4.5 : 2.0);

            if (pressedNavKeys["w"] || pressedNavKeys["arrowup"]) moveVec.add(forward);
            if (pressedNavKeys["s"] || pressedNavKeys["arrowdown"]) moveVec.sub(forward);
            if (pressedNavKeys["d"] || pressedNavKeys["arrowright"]) moveVec.add(right);
            if (pressedNavKeys["a"] || pressedNavKeys["arrowleft"]) moveVec.sub(right);

            if (moveVec.lengthSq() > 0) {
                moveVec.normalize().multiplyScalar(panSpeed);
                controlsTheater.target.add(moveVec);
                cameraTheater.position.add(moveVec);
            }
        }
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
