/**
 * vis/static/js/sector_delta_diorama.js
 * High-Fidelity Architectural Tabletop Diorama ("Sector Delta -27")
 * Replicates the reference model with:
 * - Sunny blue sky gradient & warm daylighting with soft shadows
 * - Beveled presentation display tray with raised carbon rim
 * - Vibrant natural green land plinth texture with grassy parkland zoning, elevation contours, subtle grid, and markings
 * - Diverse architectural skyscrapers (faceted glass tower, hexagonal tower, beige punch-window high-rises, terracotta red-roofed low-rises)
 * - Stylized miniature green trees along curbs, overpass ramps, and red courtyard plazas
 * - 3D elevated curved highway overpasses with concrete piers, guardrails, and miniature cars
 * - Luminous 3D parabolic communication arcs (golden-yellow and electric cyan) with streaming sparkling photon particles
 */

const SectorDelta = (() => {
    let dioramaGroup = null;
    let challengeGroup = null;
    let arcStreams = []; // Array of active parabolic arc streams
    let rooftopRelayNodes = []; // Key rooftop relay coordinates

    // ------------------------------------------------------------------------
    // 1. Procedural Texture Generators
    // ------------------------------------------------------------------------

    // Glass Curtain Wall Facade Texture (Teal-blue reflective glass with white mullion grid)
    function createGlassFacadeTexture() {
        const c = document.createElement("canvas");
        c.width = 512;
        c.height = 1024;
        const ctx = c.getContext("2d");

        // Base glass gradient: Bright architectural reflective teal-blue
        const grad = ctx.createLinearGradient(0, 0, 512, 1024);
        grad.addColorStop(0.0, "#3a7f9e"); // Reflective sky-teal at upper floors
        grad.addColorStop(0.35, "#4c96b8"); // Clear cyan glass
        grad.addColorStop(0.70, "#2c6885"); // Rich architectural azure
        grad.addColorStop(1.0, "#214e66");
        ctx.fillStyle = grad;
        ctx.fillRect(0, 0, 512, 1024);

        // Window grid: Dense reflective panes with crisp white mullion borders
        const cols = 14;
        const rows = 46;
        const gw = 512 / cols;
        const gh = 1024 / rows;

        for (let r = 0; r < rows; r++) {
            for (let col = 0; col < cols; col++) {
                // Glass pane with realistic sky reflection variation
                const tint = (Math.sin(col * 2.3 + r * 1.7) * 0.5 + 0.5) * 0.35;
                ctx.fillStyle = `rgba(195, 238, 255, ${0.28 + tint})`;
                ctx.fillRect(col * gw + 1.2, r * gh + 1.2, gw - 2.4, gh - 2.4);

                // Thin crisp white mullion frame
                ctx.strokeStyle = "rgba(255, 255, 255, 0.95)";
                ctx.lineWidth = 1.6;
                ctx.strokeRect(col * gw, r * gh, gw, gh);
            }
            // Major horizontal floor spandrel
            ctx.fillStyle = "#ffffff";
            ctx.fillRect(0, r * gh - 1, 512, 3);
        }

        const tex = new THREE.CanvasTexture(c);
        tex.wrapS = THREE.RepeatWrapping;
        tex.wrapT = THREE.RepeatWrapping;
        return tex;
    }

    // Beige Sandstone Punch-Window Facade Texture
    function createBeigeFacadeTexture() {
        const c = document.createElement("canvas");
        c.width = 512;
        c.height = 1024;
        const ctx = c.getContext("2d");

        // Warm beige/sandstone background
        ctx.fillStyle = "#dfd5c2";
        ctx.fillRect(0, 0, 512, 1024);

        // Subtle limestone texture noise
        for (let i = 0; i < 6000; i++) {
            const x = Math.random() * 512;
            const y = Math.random() * 1024;
            ctx.fillStyle = Math.random() > 0.5 ? "rgba(255,255,255,0.08)" : "rgba(180,165,145,0.08)";
            ctx.fillRect(x, y, 2, 2);
        }

        const cols = 10;
        const rows = 36;
        const gw = 512 / cols;
        const gh = 1024 / rows;

        for (let r = 0; r < rows; r++) {
            for (let col = 0; col < cols; col++) {
                const wx = col * gw + 8;
                const wy = r * gh + 7;
                const ww = gw - 16;
                const wh = gh - 14;

                // Window recess shadow
                ctx.fillStyle = "#2c261e";
                ctx.fillRect(wx - 1, wy - 1, ww + 2, wh + 2);

                // Dark reflective window glass
                const winGrad = ctx.createLinearGradient(wx, wy, wx, wy + wh);
                winGrad.addColorStop(0.0, "#1a2530");
                winGrad.addColorStop(1.0, "#0c131a");
                ctx.fillStyle = winGrad;
                ctx.fillRect(wx, wy, ww, wh);

                // Window sill highlight
                ctx.fillStyle = "#f5eee0";
                ctx.fillRect(wx - 2, wy + wh, ww + 4, 3);
            }
            // Architectural horizontal cornice line every 3 floors
            if (r % 3 === 0) {
                ctx.fillStyle = "#c7baa5";
                ctx.fillRect(0, r * gh - 2, 512, 4);
                ctx.fillStyle = "#ffffff";
                ctx.fillRect(0, r * gh - 3, 512, 1);
            }
        }

        const tex = new THREE.CanvasTexture(c);
        tex.wrapS = THREE.RepeatWrapping;
        tex.wrapT = THREE.RepeatWrapping;
        return tex;
    }

    // Modern Slate Grey Ribbon Window Texture
    function createModernSlateTexture() {
        const c = document.createElement("canvas");
        c.width = 512;
        c.height = 512;
        const ctx = c.getContext("2d");

        ctx.fillStyle = "#263238";
        ctx.fillRect(0, 0, 512, 512);

        const rows = 16;
        const rh = 512 / rows;
        for (let r = 0; r < rows; r++) {
            // Dark ribbon window band
            ctx.fillStyle = "#0d1822";
            ctx.fillRect(0, r * rh + 6, 512, rh - 12);

            // Glass specular stripe
            ctx.fillStyle = "rgba(100, 180, 240, 0.25)";
            ctx.fillRect(0, r * rh + 8, 512, (rh - 12) * 0.4);

            // White metal mullion vertical dividers
            ctx.strokeStyle = "rgba(240, 245, 250, 0.6)";
            ctx.lineWidth = 1.5;
            for (let x = 0; x < 512; x += 32) {
                ctx.beginPath();
                ctx.moveTo(x, r * rh + 6);
                ctx.lineTo(x, r * rh + rh - 6);
                ctx.stroke();
            }

            // Concrete spandrel band
            ctx.fillStyle = "#455a64";
            ctx.fillRect(0, r * rh, 512, 6);
        }

        const tex = new THREE.CanvasTexture(c);
        tex.wrapS = THREE.RepeatWrapping;
        tex.wrapT = THREE.RepeatWrapping;
        return tex;
    }

    // Terracotta Roof Tile Texture
    function createTerracottaRoofTexture() {
        const c = document.createElement("canvas");
        c.width = 256;
        c.height = 256;
        const ctx = c.getContext("2d");

        ctx.fillStyle = "#b83828";
        ctx.fillRect(0, 0, 256, 256);

        // Tile ridges
        for (let y = 0; y < 256; y += 12) {
            ctx.fillStyle = "#942517";
            ctx.fillRect(0, y, 256, 3);
            ctx.fillStyle = "#d44a39";
            ctx.fillRect(0, y + 3, 256, 3);
        }

        const tex = new THREE.CanvasTexture(c);
        tex.wrapS = THREE.RepeatWrapping;
        tex.wrapT = THREE.RepeatWrapping;
        return tex;
    }

    // Road Boulevard Control Points and Smooth Spline
    const roadControlPoints = [
        [-155, -80],
        [-105, -114],
        [-20, -122],
        [55, -112],
        [115, -75],
        [134, 5],
        [95, 78],
        [15, 102],
        [-55, 92],
        [-125, 50],
        [-155, -80]
    ];

    function getCatmullRomSpline(points, numSamples = 120, closed = true) {
        const pts = points.map(p => ({ x: p[0], y: p[1] }));
        const n = pts.length;
        const curvePts = [];
        const getPt = (idx) => {
            if (closed) return pts[((idx % n) + n) % n];
            return pts[Math.max(0, Math.min(n - 1, idx))];
        };
        for (let i = 0; i < n; i++) {
            const p0 = getPt(i - 1);
            const p1 = getPt(i);
            const p2 = getPt(i + 1);
            const p3 = getPt(i + 2);
            const segSamples = Math.max(2, Math.floor(numSamples / n));
            for (let s = 0; s < segSamples; s++) {
                const t = s / segSamples;
                const t2 = t * t;
                const t3 = t2 * t;
                const x = 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
                const y = 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
                curvePts.push([x, y]);
            }
        }
        return curvePts;
    }

    const smoothRoadPoints = getCatmullRomSpline(roadControlPoints, 120, true);

    // ------------------------------------------------------------------------
    // Vibrant Natural Green Land Plinth Texture ("Sector Delta -27")
    // Replaces blue tactical blueprint with rich parkland terrain, subtle
    // field zoning, topographic contour lines, soft grid, smooth roads, and
    // unobstructed Sector Delta -27 typography.
    // ------------------------------------------------------------------------
    function createGreenLandPlinthTexture() {
        const c = document.createElement("canvas");
        c.width = 2048;
        c.height = 2048;
        const ctx = c.getContext("2d");

        // Coordinate helper: World [-500, +500] -> Canvas [0, 2048]
        const toC = (x, y) => [
            ((x + 500) / 1000) * 2048,
            ((500 - y) / 1000) * 2048
        ];
        const toLen = (m) => (m / 1000) * 2048;

        // 1. Desaturated Earthquake/Disaster Base Terrain (earthy olive-drab/khaki)
        const bgGrad = ctx.createRadialGradient(1024, 1024, 200, 1024, 1024, 1440);
        bgGrad.addColorStop(0.0, "#4a5344"); // Distressed dry grass/soil
        bgGrad.addColorStop(0.4, "#414b3d"); // Olive-drab field
        bgGrad.addColorStop(0.7, "#384134"); // Darker perimeter terrain
        bgGrad.addColorStop(1.0, "#293126"); // Distant edge
        ctx.fillStyle = bgGrad;
        ctx.fillRect(0, 0, 2048, 2048);

        // 2. Northeast Landslide Mud & Clay Debris Fan ([80, 80] to [240, 250])
        const [lsX, lsY] = toC(80, 250);
        const lsW = toLen(160);
        const lsH = toLen(170);
        const mudGrad = ctx.createRadialGradient(lsX + lsW * 0.6, lsY + lsH * 0.3, toLen(20), lsX + lsW * 0.5, lsY + lsH * 0.5, toLen(100));
        mudGrad.addColorStop(0.0, "#7d5d42"); // Reddish-brown exposed raw clay
        mudGrad.addColorStop(0.4, "#694d36"); // Mud slide apron
        mudGrad.addColorStop(0.8, "#553e2c"); // Debris edge
        mudGrad.addColorStop(1.0, "rgba(85, 62, 44, 0.0)");
        ctx.fillStyle = mudGrad;
        ctx.beginPath();
        ctx.ellipse(lsX + lsW * 0.5, lsY + lsH * 0.5, lsW * 0.6, lsH * 0.5, 0.3, 0, Math.PI * 2);
        ctx.fill();

        // 3. Central Liquefaction & Concrete Dust Dispersion Zone
        const [centX, centY] = toC(-20, 20);
        const dustGrad = ctx.createRadialGradient(centX, centY, toLen(30), centX, centY, toLen(180));
        dustGrad.addColorStop(0.0, "rgba(110, 114, 110, 0.45)");
        dustGrad.addColorStop(0.6, "rgba(90, 95, 90, 0.25)");
        dustGrad.addColorStop(1.0, "rgba(70, 75, 70, 0.0)");
        ctx.fillStyle = dustGrad;
        ctx.beginPath();
        ctx.arc(centX, centY, toLen(180), 0, Math.PI * 2);
        ctx.fill();

        // 4. River Gorge Channel Footprint (North to South around X = 40)
        ctx.save();
        ctx.beginPath();
        const riverPts = [
            [48, 260], [44, 180], [38, 110], [42, 40], [39, -40], [44, -120], [40, -200], [46, -260]
        ];
        const [r0x, r0y] = toC(riverPts[0][0], riverPts[0][1]);
        ctx.moveTo(r0x, r0y);
        for (let i = 1; i < riverPts.length; i++) {
            const [rx, ry] = toC(riverPts[i][0], riverPts[i][1]);
            ctx.lineTo(rx, ry);
        }
        // Riverbed rocky margins
        ctx.strokeStyle = "#38322a";
        ctx.lineWidth = toLen(24);
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        ctx.stroke();
        // Cyan-blue water channel
        ctx.strokeStyle = "#1b5a75";
        ctx.lineWidth = toLen(16);
        ctx.stroke();
        ctx.strokeStyle = "#247ba0";
        ctx.lineWidth = toLen(10);
        ctx.stroke();
        ctx.restore();

        // 5. Urban Asphalt Road Boulevard Grid
        ctx.save();
        // Outer loop road
        ctx.beginPath();
        const [spX, spY] = toC(smoothRoadPoints[0][0], smoothRoadPoints[0][1]);
        ctx.moveTo(spX, spY);
        for (let i = 1; i < smoothRoadPoints.length; i++) {
            const [px, py] = toC(smoothRoadPoints[i][0], smoothRoadPoints[i][1]);
            ctx.lineTo(px, py);
        }
        ctx.closePath();
        ctx.strokeStyle = "#252b33";
        ctx.lineWidth = toLen(13.5);
        ctx.lineJoin = "round";
        ctx.stroke();

        // White road edges
        ctx.strokeStyle = "rgba(220, 225, 230, 0.7)";
        ctx.lineWidth = 2.0;
        ctx.stroke();

        // Yellow dashed center line
        ctx.strokeStyle = "#d4af37";
        ctx.lineWidth = 1.8;
        ctx.setLineDash([14, 12]);
        ctx.stroke();
        ctx.setLineDash([]);

        // Internal cross boulevards
        const avenues = [
            [[0, 40], [25, 95], [30, 135], [120, 145]], // Connects to Northeast Mountain Road
            [[45, 0], [105, -30], [145, -50]],           // East Substation road
            [[-40, -115], [-40, 20], [-25, 90]],         // North-South spine
            [[-130, 20], [30, 20]],                       // West-East boulevard
        ];
        avenues.forEach(av => {
            ctx.beginPath();
            const [a0x, a0y] = toC(av[0][0], av[0][1]);
            ctx.moveTo(a0x, a0y);
            for (let i = 1; i < av.length; i++) {
                const [ax, ay] = toC(av[i][0], av[i][1]);
                ctx.lineTo(ax, ay);
            }
            ctx.strokeStyle = "#252b33";
            ctx.lineWidth = toLen(11.5);
            ctx.stroke();
            ctx.strokeStyle = "rgba(220, 225, 230, 0.6)";
            ctx.lineWidth = 1.6;
            ctx.stroke();
            ctx.strokeStyle = "#d4af37";
            ctx.lineWidth = 1.6;
            ctx.setLineDash([12, 10]);
            ctx.stroke();
            ctx.setLineDash([]);
        });
        ctx.restore();

        // 6. Jagged Earthquake Fault Rupture Fissures (Dark ground cracks slicing across city & roads)
        ctx.save();
        const faultLines = [
            // Fault 1: West-East through collapse zone and hospital
            [[-150, -45], [-120, -52], [-85, -60], [-55, -58], [-20, -65], [10, -50], [40, -95], [75, -90], [115, -110]],
            // Fault 2: Northwest fissure branching to central district
            [[-70, 120], [-60, 85], [-45, 45], [-15, 20], [15, -15], [35, -45]],
            // Fault 3: East fault near substation
            [[80, -20], [105, -38], [115, -40], [145, -42], [165, -60]],
            // Fault 4: Northeast slide fault
            [[100, 130], [130, 142], [140, 150], [170, 165]]
        ];

        faultLines.forEach(fl => {
            // Main dark rupture fissure
            ctx.beginPath();
            const [f0x, f0y] = toC(fl[0][0], fl[0][1]);
            ctx.moveTo(f0x, f0y);
            for (let i = 1; i < fl.length; i++) {
                const [fx, fy] = toC(fl[i][0], fl[i][1]);
                const prev = toC(fl[i - 1][0], fl[i - 1][1]);
                const midX = (prev[0] + fx) / 2 + (Math.sin(i * 3.7) * 8);
                const midY = (prev[1] + fy) / 2 + (Math.cos(i * 4.1) * 8);
                ctx.lineTo(midX, midY);
                ctx.lineTo(fx, fy);
            }
            ctx.strokeStyle = "#0e1114";
            ctx.lineWidth = 3.5;
            ctx.stroke();

            // Broken asphalt edges along the fissure
            ctx.strokeStyle = "rgba(70, 50, 35, 0.85)";
            ctx.lineWidth = 6.0;
            ctx.stroke();
            ctx.strokeStyle = "#0e1114";
            ctx.lineWidth = 2.5;
            ctx.stroke();
        });
        ctx.restore();

        // 7. Outer Plinth Frame & Corner Registration Marks
        ctx.strokeStyle = "rgba(180, 200, 180, 0.35)";
        ctx.lineWidth = 3;
        ctx.strokeRect(24, 24, 2000, 2000);

        const cross = (x, y) => {
            ctx.beginPath();
            ctx.moveTo(x - 14, y); ctx.lineTo(x + 14, y);
            ctx.moveTo(x, y - 14); ctx.lineTo(x + 14, y);
            ctx.stroke();
        };
        cross(24, 24);
        cross(2024, 24);
        cross(24, 2024);
        cross(2024, 2024);

        const tex = new THREE.CanvasTexture(c);
        tex.anisotropy = 4;
        return tex;
    }

    // Backwards compatibility alias
    function createBlueprintPlinthTexture() {
        return createGreenLandPlinthTexture();
    }

    // ------------------------------------------------------------------------
    // 2. Beveled Presentation Display Tray (Plinth)
    // ------------------------------------------------------------------------
    function createPlinthTray(size, height) {
        const trayGroup = new THREE.Group();
        const half = size / 2;
        const rimWidth = 12;

        // Dark matte charcoal tray material with metallic edge sheen
        const rimMat = new THREE.MeshStandardMaterial({
            color: 0x161a22,
            metalness: 0.8,
            roughness: 0.35,
        });

        // 4 beveled border rims with 45-degree chamfers
        const rimGeoX = new THREE.BoxGeometry(size + rimWidth * 2, rimWidth, height);
        const rimGeoY = new THREE.BoxGeometry(rimWidth, size, height);

        // North Rim
        const rimN = new THREE.Mesh(rimGeoX, rimMat);
        rimN.position.set(0, half + rimWidth / 2, height / 2 - 1);
        trayGroup.add(rimN);

        // South Rim
        const rimS = new THREE.Mesh(rimGeoX, rimMat);
        rimS.position.set(0, -half - rimWidth / 2, height / 2 - 1);
        trayGroup.add(rimS);

        // East Rim
        const rimE = new THREE.Mesh(rimGeoY, rimMat);
        rimE.position.set(half + rimWidth / 2, 0, height / 2 - 1);
        trayGroup.add(rimE);

        // West Rim
        const rimW = new THREE.Mesh(rimGeoY, rimMat);
        rimW.position.set(-half - rimWidth / 2, 0, height / 2 - 1);
        trayGroup.add(rimW);

        // Chamfered outside lip
        const lipGeo = new THREE.CylinderGeometry(rimWidth * 0.7, rimWidth * 0.7, height, 4);
        const corners = [
            [half + rimWidth / 2, half + rimWidth / 2],
            [-half - rimWidth / 2, half + rimWidth / 2],
            [half + rimWidth / 2, -half - rimWidth / 2],
            [-half - rimWidth / 2, -half - rimWidth / 2],
        ];
        corners.forEach(([cx, cy]) => {
            const corner = new THREE.Mesh(lipGeo, rimMat);
            corner.position.set(cx, cy, height / 2 - 1);
            corner.rotation.y = Math.PI / 4;
            trayGroup.add(corner);
        });

        // Vibrant Green Land Tabletop Surface Plate
        const groundGeo = new THREE.PlaneGeometry(size, size);
        const groundTex = createGreenLandPlinthTexture();
        const groundMat = new THREE.MeshStandardMaterial({
            map: groundTex,
            roughness: 0.88,
            metalness: 0.04,
        });
        const ground = new THREE.Mesh(groundGeo, groundMat);
        ground.position.set(0, 0, 0.1);
        ground.receiveShadow = true;
        trayGroup.add(ground);

        return trayGroup;
    }

    // ------------------------------------------------------------------------
    // 3. Elevated Curved Highway Overpasses & Miniature Cars
    // ------------------------------------------------------------------------
    function createElevatedOverpass(group) {
        // Curve: Elevated overpass curving around the south & east city
        // Segment from X: -140 to +125
        const curvePoints = [
            new THREE.Vector3(-140, -80, 2.5),
            new THREE.Vector3(-100, -100, 7.5), // POI_HIGHWAY: fault rupture break
            new THREE.Vector3(-30, -105, 12.0),
            new THREE.Vector3(18, -98, 13.5),   // Approach to river bridge
            new THREE.Vector3(62, -92, 13.5),   // Exit from river bridge
            new THREE.Vector3(95, -60, 11.0),
            new THREE.Vector3(120, -10, 6.0),
            new THREE.Vector3(125, 40, 2.0),
        ];
        const curve = new THREE.CatmullRomCurve3(curvePoints);
        const samples = 48;
        const pts = curve.getPoints(samples);

        const roadWidth = 14;
        const deckThickness = 1.2;
        const deckMat = new THREE.MeshStandardMaterial({
            color: 0x2e3440,
            roughness: 0.85,
            flatShading: true,
        });
        const barrierMat = new THREE.MeshStandardMaterial({
            color: 0x828b94,
            roughness: 0.75,
            flatShading: true,
        });
        const pierMat = new THREE.MeshStandardMaterial({
            color: 0x6c747c,
            roughness: 0.8,
            flatShading: true,
        });

        // Loop through segments
        for (let i = 0; i < pts.length - 1; i++) {
            const p1 = pts[i];
            const p2 = pts[i + 1];
            const segLen = p1.distanceTo(p2);
            const mid = p1.clone().add(p2).multiplyScalar(0.5);

            // 1. POI_HIGHWAY Fracture Zone (around i in [7, 9], x ~ -100, y ~ -100)
            const isHighwayFault = (i >= 7 && i <= 8);

            // 2. River Gorge Span (X in [20, 60]):
            // Entirely handled as the iconic severed bridge in createRiverAndBridges!
            if (mid.x >= 20 && mid.x <= 60) {
                continue;
            }

            const segGeo = new THREE.BoxGeometry(roadWidth, segLen, deckThickness);
            const seg = new THREE.Mesh(segGeo, deckMat);
            seg.position.copy(mid);

            const dir = p2.clone().sub(p1).normalize();
            const up = new THREE.Vector3(0, 0, 1);
            const right = new THREE.Vector3().crossVectors(dir, up).normalize();
            const rotMat = new THREE.Matrix4().makeBasis(right, dir, up);
            seg.rotation.setFromRotationMatrix(rotMat);

            if (isHighwayFault) {
                seg.position.z -= 1.8;
                seg.rotation.x += 0.22;
                seg.rotation.y += 0.08;
            }

            seg.castShadow = true;
            seg.receiveShadow = true;
            group.add(seg);

            // Concrete crash barriers
            if (i % 6 !== 2) {
                [-roadWidth / 2 + 0.4, roadWidth / 2 - 0.4].forEach(offset => {
                    const barGeo = new THREE.BoxGeometry(0.8, segLen, 1.2);
                    const bar = new THREE.Mesh(barGeo, barrierMat);
                    bar.position.copy(mid).add(right.clone().multiplyScalar(offset));
                    bar.position.z += 0.8;
                    bar.rotation.setFromRotationMatrix(rotMat);
                    if (isHighwayFault) bar.position.z -= 1.8;
                    group.add(bar);
                });
            }

            // Concrete support piers
            if (i % 5 === 0 && mid.z > 3.0 && !isHighwayFault) {
                const pierGeo = new THREE.CylinderGeometry(1.6, 2.0, mid.z, 8);
                const pier = new THREE.Mesh(pierGeo, pierMat);
                pier.rotation.x = Math.PI / 2;
                pier.position.set(mid.x, mid.y, mid.z / 2);
                pier.castShadow = true;
                group.add(pier);
            }
        }

        // Stranded / Emergency Vehicles on overpass
        const vehicles = [
            { idx: 6, side: 2.5, col: 0x1d4ed8, type: "patrol" },
            { idx: 18, side: -2.0, col: 0xd97706, type: "car" },
            { idx: 35, side: 1.8, col: 0xe2e8f0, type: "truck" },
        ];
        vehicles.forEach(v => {
            const pt = pts[v.idx];
            const dir = pts[v.idx + 1].clone().sub(pt).normalize();
            const up = new THREE.Vector3(0, 0, 1);
            const right = new THREE.Vector3().crossVectors(dir, up).normalize();
            const vGroup = new THREE.Group();

            const bodyGeo = v.type === "truck" ? new THREE.BoxGeometry(2.4, 5.0, 2.2) : new THREE.BoxGeometry(2.0, 3.8, 1.3);
            const body = new THREE.Mesh(bodyGeo, new THREE.MeshStandardMaterial({ color: v.col, roughness: 0.6, flatShading: true }));
            vGroup.add(body);

            const roofGeo = new THREE.BoxGeometry(1.7, 1.8, 0.7);
            const roof = new THREE.Mesh(roofGeo, new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.5, flatShading: true }));
            roof.position.set(0, -0.2, 0.9);
            vGroup.add(roof);

            vGroup.position.copy(pt).add(right.multiplyScalar(v.side));
            vGroup.position.z += 1.3;
            const rotMat = new THREE.Matrix4().makeBasis(right, dir, up);
            vGroup.rotation.setFromRotationMatrix(rotMat);
            group.add(vGroup);
        });
    }

    // ------------------------------------------------------------------------
    // 4. Stylized Miniature Architectural Trees
    // ------------------------------------------------------------------------
    // ------------------------------------------------------------------------
    // 4. Stylized Miniature Architectural Trees
    // ------------------------------------------------------------------------
    function createTree(x, y, scale = 1.0, type = 0) {
        const treeGroup = new THREE.Group();
        treeGroup.position.set(x, y, 0);

        const trunkMat = new THREE.MeshStandardMaterial({ color: 0x3d2b1f, roughness: 0.9, flatShading: true });
        const pineGreens = [0x1e4620, 0x245424, 0x163818];
        const deciduousGreens = [0x2e6b27, 0x3b7d33, 0x488a3e];

        if (type === 1) {
            // Stylized Low-Poly Pine Tree (Matching Reference Image 1)
            const trunkH = 3.6 * scale;
            const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.24 * scale, 0.38 * scale, trunkH, 6), trunkMat);
            trunk.rotation.x = Math.PI / 2;
            trunk.position.z = trunkH / 2;
            trunk.castShadow = true;
            treeGroup.add(trunk);

            for (let k = 0; k < 3; k++) {
                const cRad = (2.2 - k * 0.55) * scale;
                const cH = (3.0 - k * 0.4) * scale;
                const cone = new THREE.Mesh(
                    new THREE.ConeGeometry(cRad, cH, 6),
                    new THREE.MeshStandardMaterial({
                        color: pineGreens[k % pineGreens.length],
                        roughness: 0.8,
                        flatShading: true,
                    })
                );
                cone.rotation.x = Math.PI / 2;
                cone.position.z = (2.0 + k * 1.6) * scale;
                cone.castShadow = true;
                cone.receiveShadow = true;
                treeGroup.add(cone);
            }
        } else {
            // Faceted Low-Poly Deciduous Tree
            const trunkH = 2.4 * scale;
            const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.25 * scale, 0.4 * scale, trunkH, 6), trunkMat);
            trunk.rotation.x = Math.PI / 2;
            trunk.position.z = trunkH / 2;
            trunk.castShadow = true;
            treeGroup.add(trunk);

            for (let c = 0; c < 2; c++) {
                const folGeo = new THREE.DodecahedronGeometry((1.5 + c * 0.4) * scale, 0);
                const folMat = new THREE.MeshStandardMaterial({
                    color: deciduousGreens[c % deciduousGreens.length],
                    roughness: 0.82,
                    flatShading: true,
                });
                const fol = new THREE.Mesh(folGeo, folMat);
                fol.position.set(0, 0, (2.2 + c * 1.2) * scale);
                fol.castShadow = true;
                fol.receiveShadow = true;
                treeGroup.add(fol);
            }
        }
        return treeGroup;
    }

    function createFallenTree(x, y, scale = 1.0) {
        const treeGroup = new THREE.Group();
        treeGroup.position.set(x, y, 0.4);

        const trunkMat = new THREE.MeshStandardMaterial({ color: 0x3d2b1f, roughness: 0.9, flatShading: true });
        const pineMat = new THREE.MeshStandardMaterial({ color: 0x224920, roughness: 0.85, flatShading: true });

        // Toppled trunk lying on ground
        const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.3 * scale, 0.4 * scale, 7.0 * scale, 6), trunkMat);
        trunk.rotation.z = Math.random() * Math.PI;
        trunk.rotation.y = Math.PI / 2;
        treeGroup.add(trunk);

        // Crushed pine cone on ground
        const cone = new THREE.Mesh(new THREE.ConeGeometry(1.8 * scale, 3.5 * scale, 6), pineMat);
        cone.rotation.z = trunk.rotation.z;
        cone.rotation.y = Math.PI / 2;
        cone.position.set(2.5 * scale, 0, 0.2);
        treeGroup.add(cone);

        return treeGroup;
    }

    function isTreePositionBlocked(x, y) {
        // Outside plinth
        if (Math.abs(x) > 175 || Math.abs(y) > 175) return true;
        // GCS flight apron clearance
        if (y < -124 && Math.abs(x) < 92) return true;
        // River channel clearance
        if (x >= 28 && x <= 56) return true;
        // Landslide mountain clearance
        if (x >= 80 && y >= 80) return true;
        // Major building blocks clearance
        const blocks = [
            { minX: -85, maxX: -35, minY: 0, maxY: 45 },    // West towers
            { minX: -140, maxX: -85, minY: -45, maxY: 10 },
            { minX: -5, maxX: 35, minY: 30, maxY: 75 },     // Center towers
            { minX: -105, maxX: -65, minY: -80, maxY: -45 },// Collapse & shelter
            { minX: 100, maxX: 140, minY: -60, maxY: -20 }, // Substation
        ];
        for (let b of blocks) {
            if (x >= b.minX && x <= b.maxX && y >= b.minY && y <= b.maxY) return true;
        }
        return false;
    }

    function createLandscapingTrees(group) {
        // Stylized low-poly trees scattered in parks, along riverbanks, and slopes
        const clusters = [
            { cx: -100, cy: -20, count: 12, radius: 22, type: 0 },
            { cx: -60, cy: -95, count: 10, radius: 18, type: 0 },
            { cx: -20, cy: 15, count: 14, radius: 20, type: 0 },
            { cx: 70, cy: -65, count: 12, radius: 20, type: 1 },
            { cx: 20, cy: -110, count: 8, radius: 16, type: 1 },
            { cx: -130, cy: 40, count: 10, radius: 18, type: 1 },
            { cx: -100, cy: 110, count: 16, radius: 24, type: 1 },
            { cx: 15, cy: 130, count: 14, radius: 22, type: 1 },
            { cx: 22, cy: 30, count: 8, radius: 15, type: 1 },   // River west bank
            { cx: 62, cy: 20, count: 8, radius: 15, type: 1 },   // River east bank
        ];

        clusters.forEach(({ cx, cy, count, radius, type }) => {
            let placed = 0;
            for (let attempt = 0; attempt < count * 3 && placed < count; attempt++) {
                const angle = Math.random() * Math.PI * 2;
                const dist = Math.random() * radius;
                const tx = cx + Math.cos(angle) * dist;
                const ty = cy + Math.sin(angle) * dist;

                if (!isTreePositionBlocked(tx, ty)) {
                    const scale = 0.85 + Math.random() * 0.45;
                    group.add(createTree(tx, ty, scale, type));
                    placed++;
                }
            }
        });

        // Several toppled / fallen trees along the landslide fringe and river gorge
        const fallenCoords = [
            [72, 95], [68, 125], [26, -45], [24, -80], [58, -70], [75, 75]
        ];
        fallenCoords.forEach(([fx, fy]) => {
            group.add(createFallenTree(fx, fy, 1.0));
        });
    }

    // ------------------------------------------------------------------------
    // ------------------------------------------------------------------------
    // 4b. Northeast Landslide Mountain with Buried Tunnels & Massive Rockslide
    // (Directly matches Reference Image 1)
    // ------------------------------------------------------------------------
    function createLandslideMountain(group) {
        const mountainGroup = new THREE.Group();
        mountainGroup.name = "landslide_mountain_complex";

        // A. Faceted Low-Poly Mountain Terrain with Landslide Chute
        // Grid: X from 75 to 250 (14 cols), Y from 125 to 255 (13 rows)
        const xMin = 75, xMax = 250, cols = 15;
        const yMin = 125, yMax = 255, rows = 14;
        const dx = (xMax - xMin) / (cols - 1);
        const dy = (yMax - yMin) / (rows - 1);

        const vertices = [];
        const colors = [];

        // Elevation function
        const getElevation = (x, y) => {
            if (y <= 134) return 0.0; // Road base
            const tY = (y - 134) / (yMax - 134);
            // Main mountain ridge rising to peak at (185, 230)
            const dPeak = Math.hypot(x - 185, y - 230);
            let z = tY * 48.0 + Math.max(0, 26.0 - dPeak * 0.28);

            // Landslide chute scooped depression (between x: 120 and 170, y: 135 to 215)
            const inChute = (x >= 115 && x <= 175 && y <= 215);
            if (inChute) {
                const chuteCenter = 145;
                const chuteDist = Math.abs(x - chuteCenter) / 30;
                const scoop = (1.0 - chuteDist * chuteDist) * 7.5;
                z = Math.max(0.5, z - scoop);
            }

            // Pseudo-random low-poly surface jitter
            const jitter = Math.sin(x * 0.35 + y * 0.28) * 1.8 + Math.cos(x * 0.15 - y * 0.3) * 1.4;
            return Math.max(0, z + jitter);
        };

        // Vertex color function (Lush low-poly green for forest, clay orange/brown for landslide scarp)
        const getColor = (x, y) => {
            const inChute = (x >= 115 && x <= 175 && y <= 215);
            if (inChute) {
                // Landslide raw earth / clay slope (matches Reference Image 1)
                const cVar = (Math.sin(x * 1.3) + Math.cos(y * 1.1)) * 0.5 + 0.5;
                // Blend from terracotta brown to raw clay orange
                return new THREE.Color().lerpColors(
                    new THREE.Color(0x995733),
                    new THREE.Color(0xb86c43),
                    cVar
                );
            } else {
                // Low-poly green forest mountain
                const cVar = (Math.sin(x * 0.8) + Math.cos(y * 0.9)) * 0.5 + 0.5;
                return new THREE.Color().lerpColors(
                    new THREE.Color(0x3e7735),
                    new THREE.Color(0x4c8f42),
                    cVar
                );
            }
        };

        // Build faceted triangles
        for (let j = 0; j < rows - 1; j++) {
            for (let i = 0; i < cols - 1; i++) {
                const x0 = xMin + i * dx;
                const x1 = xMin + (i + 1) * dx;
                const y0 = yMin + j * dy;
                const y1 = yMin + (j + 1) * dy;

                const z00 = getElevation(x0, y0);
                const z10 = getElevation(x1, y0);
                const z01 = getElevation(x0, y1);
                const z11 = getElevation(x1, y1);

                const c00 = getColor(x0, y0);
                const c10 = getColor(x1, y0);
                const c01 = getColor(x0, y1);
                const c11 = getColor(x1, y1);

                // Triangle 1: (x0, y0) -> (x1, y0) -> (x1, y1)
                vertices.push(x0, y0, z00,  x1, y0, z10,  x1, y1, z11);
                colors.push(c00.r, c00.g, c00.b,  c10.r, c10.g, c10.b,  c11.r, c11.g, c11.b);

                // Triangle 2: (x0, y0) -> (x1, y1) -> (x0, y1)
                vertices.push(x0, y0, z00,  x1, y1, z11,  x0, y1, z01);
                colors.push(c00.r, c00.g, c00.b,  c11.r, c11.g, c11.b,  c01.r, c01.g, c01.b);
            }
        }

        const mountainGeo = new THREE.BufferGeometry();
        mountainGeo.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
        mountainGeo.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
        mountainGeo.computeVertexNormals();

        const mountainMat = new THREE.MeshStandardMaterial({
            vertexColors: true,
            roughness: 0.85,
            metalness: 0.05,
            flatShading: true,
        });
        const mountainMesh = new THREE.Mesh(mountainGeo, mountainMat);
        mountainMesh.castShadow = true;
        mountainMesh.receiveShadow = true;
        mountainGroup.add(mountainMesh);

        // B. Dual Mountain Highway Concrete Tunnels at Base (Y ~ 136)
        const tunnelConcreteMat = new THREE.MeshStandardMaterial({
            color: 0x8a9299,
            roughness: 0.75,
            flatShading: true,
        });

        // 1. West Tunnel (Open clear lane into mountain)
        const tWest = new THREE.Group();
        tWest.position.set(116, 134, 0);
        // Concrete portal box
        const portalW = new THREE.Mesh(new THREE.BoxGeometry(15, 6, 11), tunnelConcreteMat);
        portalW.position.set(0, 0, 5.5);
        portalW.castShadow = true;
        tWest.add(portalW);
        // Tunnel arch opening void
        const voidW = new THREE.Mesh(new THREE.BoxGeometry(9, 6.2, 7.5), new THREE.MeshBasicMaterial({ color: 0x090c10 }));
        voidW.position.set(0, 0, 4.0);
        tWest.add(voidW);
        mountainGroup.add(tWest);

        // 2. East Tunnel (Cracked, crushed, and buried under rockslide!)
        const tEast = new THREE.Group();
        tEast.position.set(138, 137, 0);
        const portalE = new THREE.Mesh(new THREE.BoxGeometry(15, 6, 10), tunnelConcreteMat);
        portalE.position.set(0, 0, 5.0);
        portalE.rotation.z = -0.12;
        portalE.castShadow = true;
        tEast.add(portalE);
        mountainGroup.add(tEast);

        // C. Massive Rockslide Boulder Cascade (Directly matches Reference Image 1!)
        // Light tan and stone grey faceted boulders spilling down the chute
        const boulderColors = [0xc4af98, 0xb09c85, 0x8f7f6e, 0xa59480, 0x6e6355];
        const boulderMats = boulderColors.map(c => new THREE.MeshStandardMaterial({
            color: c,
            roughness: 0.85,
            flatShading: true,
        }));

        const boulderGeos = [
            new THREE.DodecahedronGeometry(1.0, 0),
            new THREE.TetrahedronGeometry(1.0, 0),
        ];

        // 1. Choke pile directly inside and over the East Tunnel entrance
        for (let b = 0; b < 24; b++) {
            const rad = 2.0 + (b % 4) * 0.9;
            const geo = boulderGeos[b % 2];
            const mat = boulderMats[b % boulderMats.length];
            const boulder = new THREE.Mesh(geo, mat);
            boulder.scale.set(rad, rad * (0.8 + Math.random() * 0.4), rad * (0.7 + Math.random() * 0.5));
            boulder.position.set(
                138 + (Math.sin(b * 1.7) * 8.0),
                135 + (Math.cos(b * 2.1) * 7.0),
                1.2 + (b * 0.5)
            );
            boulder.rotation.set(b * 0.7, b * 1.3, b * 0.5);
            boulder.castShadow = true;
            boulder.receiveShadow = true;
            mountainGroup.add(boulder);
        }

        // 2. Cascade streaming down the chute (from Y: 210, X: 165 down to Y: 130, X: 135)
        for (let b = 0; b < 60; b++) {
            const t = b / 60; // 0 (top of slide) to 1 (road bottom)
            const rad = 1.6 + (b % 5) * 0.85;
            const geo = boulderGeos[b % 2];
            const mat = boulderMats[b % boulderMats.length];
            const boulder = new THREE.Mesh(geo, mat);
            boulder.scale.set(rad, rad * (0.75 + Math.random() * 0.5), rad * (0.75 + Math.random() * 0.4));

            const bx = 168 - t * 34 + (Math.sin(b * 2.8) * 14.0);
            const by = 210 - t * 76 + (Math.cos(b * 3.1) * 8.0);
            const bz = Math.max(0.8, (1.0 - t) * 36.0 + (Math.sin(b * 4.1) * 2.2));

            boulder.position.set(bx, by, bz);
            boulder.rotation.set(b * 1.1, b * 0.8, b * 1.5);
            boulder.castShadow = true;
            boulder.receiveShadow = true;
            mountainGroup.add(boulder);
        }

        // D. Stylized Low-Poly Pine Trees on Mountain Crests & Undisturbed Slopes
        const pineMat = new THREE.MeshStandardMaterial({
            color: 0x225520,
            roughness: 0.8,
            flatShading: true,
        });
        const trunkMat = new THREE.MeshStandardMaterial({
            color: 0x3d2b1f,
            roughness: 0.9,
            flatShading: true,
        });

        const pineCrests = [
            // North-East High Ridge
            [195, 235], [180, 240], [215, 225], [230, 210], [170, 245],
            // East slope
            [220, 185], [235, 160], [210, 155], [230, 135],
            // West mountain shoulder (above open tunnel)
            [95, 210], [90, 180], [100, 155], [105, 140],
            [110, 225], [130, 240], [145, 245],
        ];

        pineCrests.forEach(([px, py]) => {
            const pz = getElevation(px, py);
            const pTree = new THREE.Group();
            pTree.position.set(px, py, pz);

            const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.35, 0.55, 3.5, 6), trunkMat);
            trunk.rotation.x = Math.PI / 2;
            trunk.position.z = 1.75;
            pTree.add(trunk);

            for (let t = 0; t < 3; t++) {
                const cGeo = new THREE.ConeGeometry(3.0 - t * 0.65, 3.8 - t * 0.5, 6);
                const cone = new THREE.Mesh(cGeo, pineMat);
                cone.rotation.x = Math.PI / 2;
                cone.position.z = 3.2 + t * 2.0;
                cone.castShadow = true;
                pTree.add(cone);
            }
            mountainGroup.add(pTree);
        });

        group.add(mountainGroup);
    }

    // ------------------------------------------------------------------------
    // 4c. River Gorge Channel & Fractured Bridge (POI_BRIDGE)
    // (Directly matches Reference Image 2)
    // ------------------------------------------------------------------------
    function createRiverAndBridges(group) {
        const riverGroup = new THREE.Group();
        riverGroup.name = "river_and_fractured_bridges";

        const waterMat = new THREE.MeshStandardMaterial({
            color: 0x22809e, // Vibrant turquoise / river cyan-blue (Matching Reference Image 1 & 2)
            roughness: 0.15,
            metalness: 0.1,
            flatShading: true,
            transparent: true,
            opacity: 0.92,
        });
        const canyonMat1 = new THREE.MeshStandardMaterial({
            color: 0x73695d, // Weathered rocky canyon block
            roughness: 0.9,
            flatShading: true,
        });
        const canyonMat2 = new THREE.MeshStandardMaterial({
            color: 0x8c8275, // Sunlit rock ledge
            roughness: 0.88,
            flatShading: true,
        });
        const bridgeDeckMat = new THREE.MeshStandardMaterial({
            color: 0x2e3540,
            roughness: 0.85,
            flatShading: true,
        });
        const bridgePierMat = new THREE.MeshStandardMaterial({
            color: 0x7c858e,
            roughness: 0.75,
            flatShading: true,
        });
        const barrierMat = new THREE.MeshStandardMaterial({
            color: 0x949da6,
            roughness: 0.7,
            flatShading: true,
        });
        const riverRockMat = new THREE.MeshStandardMaterial({
            color: 0x5a5247,
            roughness: 0.88,
            flatShading: true,
        });

        // 1. River Water Surface Plane (Running North to South at X ~ 40)
        const waterGeo = new THREE.PlaneGeometry(26, 520, 4, 32);
        const waterMesh = new THREE.Mesh(waterGeo, waterMat);
        waterMesh.position.set(40, 0, 0.4);
        waterMesh.receiveShadow = true;
        riverGroup.add(waterMesh);

        // 2. Stepped Polygonal Rocky Canyon Cliffs along both banks (Matching Reference Image 1 & 2)
        for (let y = -230; y <= 230; y += 28) {
            // West Canyon Bank: Tiered rocky blocks
            const wX = 24 + Math.sin(y * 0.06) * 3.5;
            const wBlock1 = new THREE.Mesh(new THREE.BoxGeometry(8, 28, 4.5), canyonMat1);
            wBlock1.position.set(wX, y, 2.25);
            wBlock1.rotation.set(Math.sin(y * 0.1) * 0.08, 0, Math.cos(y * 0.08) * 0.12);
            wBlock1.castShadow = true;
            wBlock1.receiveShadow = true;
            riverGroup.add(wBlock1);

            const wBlock2 = new THREE.Mesh(new THREE.BoxGeometry(6, 24, 7.5), canyonMat2);
            wBlock2.position.set(wX - 4.0, y + 2, 3.75);
            wBlock2.castShadow = true;
            riverGroup.add(wBlock2);

            // East Canyon Bank
            const eX = 56 + Math.cos(y * 0.06) * 3.5;
            const eBlock1 = new THREE.Mesh(new THREE.BoxGeometry(8, 28, 4.5), canyonMat1);
            eBlock1.position.set(eX, y, 2.25);
            eBlock1.rotation.set(Math.cos(y * 0.1) * 0.08, 0, -Math.sin(y * 0.08) * 0.12);
            eBlock1.castShadow = true;
            eBlock1.receiveShadow = true;
            riverGroup.add(eBlock1);

            const eBlock2 = new THREE.Mesh(new THREE.BoxGeometry(6, 24, 7.5), canyonMat2);
            eBlock2.position.set(eX + 4.0, y - 2, 3.75);
            eBlock2.castShadow = true;
            riverGroup.add(eBlock2);
        }

        // Riverbed Boulders in water
        for (let r = 0; r < 36; r++) {
            const rad = 1.2 + (r % 4) * 0.6;
            const rock = new THREE.Mesh(new THREE.DodecahedronGeometry(rad, 0), riverRockMat);
            rock.position.set(40 + (Math.sin(r * 2.3) * 8.5), -220 + r * 12.5, rad * 0.45);
            rock.rotation.set(r * 0.5, r * 1.2, r * 0.8);
            rock.castShadow = true;
            riverGroup.add(rock);
        }

        // --------------------------------------------------------------------
        // 3. ICONIC SEVERED BRIDGE at POI_BRIDGE [40.0, -95.0, 16.0]
        // (Directly matches Reference Image 2 & Reference Image 1 foreground!)
        // --------------------------------------------------------------------
        const bridgeGroup = new THREE.Group();
        bridgeGroup.position.set(40, -95, 0);

        const bWidth = 14.0;
        const deckThick = 1.3;

        // West Abutment Pier (Intact approach connecting from West Overpass at X = 18)
        const pierW = new THREE.Mesh(new THREE.BoxGeometry(4.5, 14.0, 13.5), bridgePierMat);
        pierW.position.set(-18, 0, 6.75);
        pierW.castShadow = true;
        bridgeGroup.add(pierW);

        const deckW = new THREE.Mesh(new THREE.BoxGeometry(16.0, bWidth, deckThick), bridgeDeckMat);
        deckW.position.set(-18, 0, 13.5);
        deckW.castShadow = true;
        deckW.receiveShadow = true;
        bridgeGroup.add(deckW);

        [-bWidth / 2 + 0.4, bWidth / 2 - 0.4].forEach(by => {
            const bar = new THREE.Mesh(new THREE.BoxGeometry(16.0, 0.8, 1.2), barrierMat);
            bar.position.set(-18, by, 14.5);
            bridgeGroup.add(bar);
        });

        // East Abutment Pier (Intact approach connecting to East Overpass at X = 62)
        const pierE = new THREE.Mesh(new THREE.BoxGeometry(4.5, 14.0, 13.5), bridgePierMat);
        pierE.position.set(18, 0, 6.75);
        pierE.castShadow = true;
        bridgeGroup.add(pierE);

        const deckE = new THREE.Mesh(new THREE.BoxGeometry(16.0, bWidth, deckThick), bridgeDeckMat);
        deckE.position.set(18, 0, 13.5);
        deckE.castShadow = true;
        bridgeGroup.add(deckE);

        [-bWidth / 2 + 0.4, bWidth / 2 - 0.4].forEach(by => {
            const bar = new THREE.Mesh(new THREE.BoxGeometry(16.0, 0.8, 1.2), barrierMat);
            bar.position.set(18, by, 14.5);
            bridgeGroup.add(bar);
        });

        // THE COLLAPSED DECK SPAN (Sheared in half, tilted down ~22 deg into riverbed rocks!)
        const colSpanLen = 22.0;
        const colSpan = new THREE.Mesh(new THREE.BoxGeometry(colSpanLen, bWidth, deckThick), bridgeDeckMat);
        colSpan.position.set(1.5, 0, 7.2);
        colSpan.rotation.y = 0.38;
        colSpan.castShadow = true;
        colSpan.receiveShadow = true;
        bridgeGroup.add(colSpan);

        [-bWidth / 2 + 0.4, bWidth / 2 - 0.4].forEach(by => {
            const cBar = new THREE.Mesh(new THREE.BoxGeometry(colSpanLen * 0.8, 0.8, 1.2), barrierMat);
            cBar.position.set(1.5, by, 8.0);
            cBar.rotation.y = 0.38;
            bridgeGroup.add(cBar);
        });

        // Exposed twisted rebar rods at fracture points
        for (let rb = 0; rb < 6; rb++) {
            const rebar = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 3.2, 4), new THREE.MeshStandardMaterial({ color: 0xb45309 }));
            rebar.position.set(-10.0, -5.0 + rb * 2.0, 13.2);
            rebar.rotation.set(Math.random() * 0.4, 0.6 + Math.random() * 0.3, 0);
            bridgeGroup.add(rebar);
        }

        // Riverbed Boulders piled at the base of the fallen bridge slab
        for (let b = 0; b < 12; b++) {
            const rad = 1.6 + Math.random() * 1.6;
            const rock = new THREE.Mesh(new THREE.DodecahedronGeometry(rad, 0), canyonMat1);
            rock.position.set(6.0 + Math.sin(b * 1.8) * 7.0, -5.0 + Math.cos(b * 2.2) * 8.0, 1.0 + Math.random() * 1.0);
            rock.rotation.set(b * 0.7, b * 1.1, b * 0.4);
            rock.castShadow = true;
            bridgeGroup.add(rock);
        }

        // Stranded Cars at the bridge fracture (Matching Reference Image 2)
        const car1Group = new THREE.Group();
        car1Group.position.set(-12.5, -2.5, 14.3);
        const c1Body = new THREE.Mesh(new THREE.BoxGeometry(3.6, 2.0, 1.2), new THREE.MeshStandardMaterial({ color: 0xd97706, roughness: 0.6, flatShading: true }));
        car1Group.add(c1Body);
        const c1Roof = new THREE.Mesh(new THREE.BoxGeometry(1.8, 1.7, 0.7), new THREE.MeshStandardMaterial({ color: 0x1e293b, flatShading: true }));
        c1Roof.position.set(-0.2, 0, 0.85);
        car1Group.add(c1Roof);
        bridgeGroup.add(car1Group);

        const car2Group = new THREE.Group();
        car2Group.position.set(2.5, 2.0, 7.5);
        car2Group.rotation.y = 0.38;
        const c2Body = new THREE.Mesh(new THREE.BoxGeometry(3.6, 2.0, 1.2), new THREE.MeshStandardMaterial({ color: 0xd32f2f, roughness: 0.6, flatShading: true }));
        car2Group.add(c2Body);
        const c2Roof = new THREE.Mesh(new THREE.BoxGeometry(1.8, 1.7, 0.7), new THREE.MeshStandardMaterial({ color: 0x1e293b, flatShading: true }));
        c2Roof.position.set(-0.2, 0, 0.85);
        car2Group.add(c2Roof);
        bridgeGroup.add(car2Group);

        riverGroup.add(bridgeGroup);

        // 4. Secondary Intact Bridge upstream at Y = 20 (Matching background bridges in Image 1)
        const midBridge = new THREE.Group();
        midBridge.position.set(40, 20, 0);
        const mbDeck = new THREE.Mesh(new THREE.BoxGeometry(34, 10, 1.2), bridgeDeckMat);
        mbDeck.position.set(0, 0, 5.0);
        mbDeck.castShadow = true;
        midBridge.add(mbDeck);
        [-4.5, 4.5].forEach(by => {
            const bar = new THREE.Mesh(new THREE.BoxGeometry(34, 0.8, 1.0), barrierMat);
            bar.position.set(0, by, 6.0);
            midBridge.add(bar);
        });
        const mbPier = new THREE.Mesh(new THREE.BoxGeometry(4, 10, 5.0), bridgePierMat);
        mbPier.position.set(0, 0, 2.5);
        midBridge.add(mbPier);
        riverGroup.add(midBridge);

        // 5. Far North Bridge at Y = 135
        const northBridge = new THREE.Group();
        northBridge.position.set(40, 135, 0);
        const nbDeck = new THREE.Mesh(new THREE.BoxGeometry(32, 10, 1.2), bridgeDeckMat);
        nbDeck.position.set(0, 0, 4.5);
        nbDeck.castShadow = true;
        northBridge.add(nbDeck);
        riverGroup.add(northBridge);

        group.add(riverGroup);
    }

    // 5. Architectural Skyscraper City Cluster & Low-Rise Village
    // ------------------------------------------------------------------------
    
    // ------------------------------------------------------------------------
    // Massive Corner Mountains to Enclose the 1000x1000 City
    // ------------------------------------------------------------------------
    function createCornerMountains(group) {
        const rockMat = new THREE.MeshStandardMaterial({
            color: 0x8b5a33, // Lighter mid-tone mountain brown
            roughness: 0.88,
            flatShading: true
        });
        const dirtMat = new THREE.MeshStandardMaterial({
            color: 0x6a4020, // Lighter dirt base
            roughness: 0.92,
            flatShading: true
        });
        const peakMat = new THREE.MeshStandardMaterial({
            color: 0x8b5a33, // Match rockMat (no white tips)
            roughness: 0.88,
            flatShading: true
        });

        // 4 Corners: NW, NE, SW, SE
        const corners = [
            { x: -450, y: 450, scale: 1.2 }, // NW
            { x: 450, y: 450, scale: 1.5 },  // NE
            { x: -450, y: -450, scale: 1.1 },// SW
            { x: 450, y: -450, scale: 1.3 }  // SE
        ];

        // Seeded random for consistent mountain generation
        let seed = 7777;
        function random() {
            seed = (seed * 9301 + 49297) % 233280;
            return seed / 233280;
        }

        corners.forEach(corner => {
            const mGroup = new THREE.Group();
            mGroup.position.set(corner.x, corner.y, 0);

            // Generate a cluster of peaks for each corner
            const numPeaks = 8 + Math.floor(random() * 5);
            for (let i = 0; i < numPeaks; i++) {
                const px = (random() - 0.5) * 200;
                const py = (random() - 0.5) * 200;
                // Exclude any mountain anywhere near GCS Hub corridor (|X| < 160, Y < -420)
                if (Math.abs(corner.x + px) < 160 && (corner.y + py) < -420) continue;

                const pRadius = 60 + random() * 100;
                const pHeight = (80 + random() * 140) * corner.scale;
                
                // Base dirt mound
                const baseGeo = new THREE.ConeGeometry(pRadius, pHeight * 0.4, 5 + Math.floor(random()*3));
                const baseMesh = new THREE.Mesh(baseGeo, dirtMat);
                baseMesh.rotation.x = Math.PI / 2;
                baseMesh.position.set(px, py, pHeight * 0.2);
                baseMesh.castShadow = true;
                baseMesh.receiveShadow = true;
                mGroup.add(baseMesh);

                // Main rock peak
                const peakGeo = new THREE.ConeGeometry(pRadius * 0.7, pHeight, 4 + Math.floor(random()*4));
                const peakMesh = new THREE.Mesh(peakGeo, rockMat);
                peakMesh.rotation.x = Math.PI / 2;
                peakMesh.rotation.y = random() * Math.PI;
                peakMesh.position.set(px, py, pHeight * 0.5);
                peakMesh.castShadow = true;
                peakMesh.receiveShadow = true;
                mGroup.add(peakMesh);
            }
            group.add(mGroup);
        });
        
        // Connect corners with border mountains (leaving a wide 320m open corridor for the GCS Hub at Y = -575)
        const edges = [
            // North edge
            { x: -200, y: 480 }, { x: 0, y: 490 }, { x: 200, y: 480 },
            // South edge (Keep corridor wide open for GCS at y=-575)
            { x: -280, y: -480 }, { x: 280, y: -480 },
            // West edge
            { x: -480, y: -200 }, { x: -490, y: 0 }, { x: -480, y: 200 },
            // East edge
            { x: 480, y: -200 }, { x: 490, y: 0 }, { x: 480, y: 200 },
        ];

        edges.forEach(edge => {
            const numPeaks = 3 + Math.floor(random() * 3);
            for (let i = 0; i < numPeaks; i++) {
                const px = edge.x + (random() - 0.5) * 80;
                const py = edge.y + (random() - 0.5) * 80;
                // Exclude any mountain anywhere near the GCS Hub corridor
                if (Math.abs(px) < 160 && py < -400) continue;

                const pRadius = 50 + random() * 80;
                const pHeight = 60 + random() * 80;
                
                const peakGeo = new THREE.ConeGeometry(pRadius, pHeight, 5);
                const peakMesh = new THREE.Mesh(peakGeo, rockMat);
                peakMesh.rotation.x = Math.PI / 2;
                peakMesh.position.set(px, py, pHeight * 0.5);
                peakMesh.castShadow = true;
                peakMesh.receiveShadow = true;
                group.add(peakMesh);
            }
        });
    }

    function createArchitecturalCity(group) {
        // Flat-shaded low-poly materials with high color contrast & rich saturation
        const terracottaMat = new THREE.MeshStandardMaterial({
            color: 0xbf360c, // Rich deep burnt terracotta red / rust
            roughness: 0.75,
            flatShading: true,
        });
        const ochreMat = new THREE.MeshStandardMaterial({
            color: 0xd97706, // Deep golden amber / warm sandstone
            roughness: 0.75,
            flatShading: true,
        });
        const slateMat = new THREE.MeshStandardMaterial({
            color: 0xe53935, // Vibrant architectural red
            roughness: 0.70,
            flatShading: true,
        });
        const concreteMat = new THREE.MeshStandardMaterial({
            color: 0xfbc02d, // Vibrant bright canary yellow
            roughness: 0.65,
            flatShading: true,
        });
        const creamWallMat = new THREE.MeshStandardMaterial({
            color: 0xd6ad78, // Warm golden sand stucco (not washed-out white)
            roughness: 0.80,
            flatShading: true,
        });
        const rubbleMat = new THREE.MeshStandardMaterial({
            color: 0xbf360c, // Rich red-brick / terracotta rubble
            roughness: 0.85,
            flatShading: true,
        });
        const hazardYellowMat = new THREE.MeshStandardMaterial({
            color: 0xffb300, // Vibrant hazard amber-yellow
            roughness: 0.55,
            flatShading: true,
        });
        const redCrossMat = new THREE.MeshBasicMaterial({ color: 0xff1744 });
        const tentMat = new THREE.MeshStandardMaterial({
            color: 0x1b5e20, // Deep forest green
            roughness: 0.8,
            flatShading: true,
        });

        // --------------------------------------------------------------------
        // TOWER 1: West Iconic High-Rise (Leaning 14 deg with damaged corner)
        // Center: (-60, 20)
        // --------------------------------------------------------------------
        const t1Group = new THREE.Group();
        t1Group.position.set(-60, 20, 0);
        // Earthquake tilt (pitch & roll)
        t1Group.rotation.set(0.18, 0.14, -0.06);

        const t1W = 36, t1D = 38, t1H = 68;
        const t1Mesh = new THREE.Mesh(new THREE.BoxGeometry(t1W, t1D, t1H), slateMat);
        t1Mesh.position.z = t1H / 2;
        t1Mesh.castShadow = true;
        t1Mesh.receiveShadow = true;
        t1Group.add(t1Mesh);

        // Angled sheared rooftop wedge
        const t1CrownGeo = new THREE.BoxGeometry(t1W * 0.8, t1D * 0.8, 12);
        const t1Crown = new THREE.Mesh(t1CrownGeo, terracottaMat);
        t1Crown.position.set(2, 2, t1H + 6);
        t1Crown.rotation.x = 0.2;
        t1Crown.castShadow = true;
        t1Group.add(t1Crown);

        // Communication antenna tilted
        const antGeo = new THREE.CylinderGeometry(0.3, 0.5, 16, 6);
        const ant = new THREE.Mesh(antGeo, concreteMat);
        ant.rotation.set(Math.PI / 2 + 0.25, 0.2, 0);
        ant.position.set(10, 0, t1H + 16);
        t1Group.add(ant);

        // Beacon tip
        const beacon = new THREE.Mesh(new THREE.SphereGeometry(1.0, 8, 8), new THREE.MeshBasicMaterial({ color: 0xff1744 }));
        beacon.position.set(14, 0, t1H + 24);
        t1Group.add(beacon);

        group.add(t1Group);
        rooftopRelayNodes.push(new THREE.Vector3(-60, 20, t1H + 2));

        // --------------------------------------------------------------------
        // TOWER 2: Hexagonal / Cylindrical High-Rise (Sheared & Leaning)
        // Center: (-110, -18)
        // --------------------------------------------------------------------
        const t2Group = new THREE.Group();
        t2Group.position.set(-110, -18, 0);
        t2Group.rotation.set(-0.16, 0.12, 0.1);

        const t2Rad = 18, t2H = 75;
        const t2Mesh = new THREE.Mesh(new THREE.CylinderGeometry(t2Rad, t2Rad, t2H, 10), ochreMat);
        t2Mesh.rotation.x = Math.PI / 2;
        t2Mesh.position.z = t2H / 2;
        t2Mesh.castShadow = true;
        t2Mesh.receiveShadow = true;
        t2Group.add(t2Mesh);

        // Tilted observation crown
        const t2Ring = new THREE.Mesh(new THREE.CylinderGeometry(14, 14, 6, 10), slateMat);
        t2Ring.rotation.x = Math.PI / 2;
        t2Ring.position.z = t2H + 3;
        t2Group.add(t2Ring);

        group.add(t2Group);
        rooftopRelayNodes.push(new THREE.Vector3(-110, -18, t2H + 2));

        // --------------------------------------------------------------------
        // TOWER 3: Stepped Corporate High-Rise (Leaning 12 deg)
        // Center: (10, 52)
        // --------------------------------------------------------------------
        const t3Group = new THREE.Group();
        t3Group.position.set(10, 52, 0);
        t3Group.rotation.set(-0.14, 0.10, 0.05);

        // Tier 1 Base
        const t3_1 = new THREE.Mesh(new THREE.BoxGeometry(42, 38, 44), concreteMat);
        t3_1.position.z = 22;
        t3_1.castShadow = true;
        t3Group.add(t3_1);

        // Tier 2 Setback
        const t3_2 = new THREE.Mesh(new THREE.BoxGeometry(32, 28, 22), terracottaMat);
        t3_2.position.z = 44 + 11;
        t3_2.castShadow = true;
        t3Group.add(t3_2);

        // Tier 3 Penthouse
        const t3_3 = new THREE.Mesh(new THREE.BoxGeometry(22, 18, 14), ochreMat);
        t3_3.position.z = 66 + 7;
        t3_3.castShadow = true;
        t3Group.add(t3_3);

        group.add(t3Group);
        rooftopRelayNodes.push(new THREE.Vector3(10, 52, 85));

        // --------------------------------------------------------------------
        // TOWER 4: Rear Spire High-Rise (Spire collapsed onto roof)
        // Center: (-25, 88)
        // --------------------------------------------------------------------
        const t4Group = new THREE.Group();
        t4Group.position.set(-25, 88, 0);
        t4Group.rotation.set(0.12, -0.08, 0.02);

        const t4H = 88;
        const t4Mesh = new THREE.Mesh(new THREE.BoxGeometry(26, 26, t4H), slateMat);
        t4Mesh.position.z = t4H / 2;
        t4Mesh.castShadow = true;
        t4Group.add(t4Mesh);

        // Collapsed fallen spire lying horizontally across roof
        const t4Spire = new THREE.Mesh(new THREE.ConeGeometry(4.0, 24, 6), concreteMat);
        t4Spire.rotation.set(0.2, 0.8, 1.2);
        t4Spire.position.set(0, 0, t4H + 4);
        t4Group.add(t4Spire);

        group.add(t4Group);
        rooftopRelayNodes.push(new THREE.Vector3(-25, 88, t4H + 4));

        // --------------------------------------------------------------------
        // LEANING TWIN TOWERS (Leaning toward each other and touching!)
        // Matching Reference Image 3 iconic earthquake diorama visual
        // --------------------------------------------------------------------
        // Left Twin at (-135, 20)
        const twinL = new THREE.Group();
        twinL.position.set(-135, 20, 0);
        twinL.rotation.set(0, -0.16, 0); // Leans east towards twinR
        const twinLMesh = new THREE.Mesh(new THREE.BoxGeometry(24, 26, 62), terracottaMat);
        twinLMesh.position.z = 31;
        twinLMesh.castShadow = true;
        twinL.add(twinLMesh);
        group.add(twinL);

        // Right Twin at (-110, 50)
        const twinR = new THREE.Group();
        twinR.position.set(-110, 50, 0);
        twinR.rotation.set(0.12, 0.14, 0); // Leans southwest towards twinL
        const twinRMesh = new THREE.Mesh(new THREE.BoxGeometry(24, 26, 64), ochreMat);
        twinRMesh.position.z = 32;
        twinRMesh.castShadow = true;
        twinR.add(twinRMesh);
        group.add(twinR);

        // --------------------------------------------------------------------
        // POI_COLLAPSE [-85.0, -60.0, 18.0]: 5-STORY PANCAKE COLLAPSE
        // (Directly matches Reference Images 3 & 4)
        // --------------------------------------------------------------------
        const pancakeGroup = new THREE.Group();
        pancakeGroup.position.set(-85, -60, 0);

        const slabGeo = new THREE.BoxGeometry(28, 22, 0.9);
        const slabConfigs = [
            { z: 0.6, rx: 0.02, ry: -0.04 },
            { z: 2.8, rx: 0.08, ry: 0.06 },
            { z: 5.2, rx: -0.12, ry: 0.10 },
            { z: 8.0, rx: 0.16, ry: -0.14 },
            { z: 11.2, rx: -0.22, ry: 0.18 }, // Top tilted floor plate
        ];

        slabConfigs.forEach((cfg, idx) => {
            const slab = new THREE.Mesh(slabGeo, concreteMat);
            slab.position.z = cfg.z;
            slab.rotation.set(cfg.rx, cfg.ry, idx * 0.08);
            slab.castShadow = true;
            slab.receiveShadow = true;
            pancakeGroup.add(slab);

            // Broken vertical support columns & rebar between slabs
            if (idx < slabConfigs.length - 1) {
                for (let col = 0; col < 6; col++) {
                    const colX = ((col % 3) - 1) * 9.0 + (Math.random() - 0.5) * 2;
                    const colY = ((col < 3 ? -1 : 1)) * 7.0 + (Math.random() - 0.5) * 2;
                    const colH = 2.4 + Math.random() * 0.8;
                    const column = new THREE.Mesh(new THREE.BoxGeometry(1.2, 1.2, colH), rubbleMat);
                    column.position.set(colX, colY, cfg.z + colH / 2);
                    column.rotation.set(Math.random() * 0.3, Math.random() * 0.3, 0);
                    pancakeGroup.add(column);
                }
            }
        });

        // Exposed twisted rebar rods projecting from top slab
        for (let r = 0; r < 8; r++) {
            const rebar = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 3.5, 4), new THREE.MeshStandardMaterial({ color: 0xb45309 }));
            rebar.position.set(-10 + r * 2.8, (Math.random() - 0.5) * 16, 12.0);
            rebar.rotation.set(Math.random() * 0.8, Math.random() * 0.8, 0);
            pancakeGroup.add(rebar);
        }

        // Surrounding concrete rubble mound & boulders
        for (let i = 0; i < 16; i++) {
            const chunkRad = 1.5 + Math.random() * 2.2;
            const chunk = new THREE.Mesh(new THREE.DodecahedronGeometry(chunkRad, 0), rubbleMat);
            const ang = (i / 16) * Math.PI * 2;
            const dist = 16 + Math.random() * 6;
            chunk.position.set(Math.cos(ang) * dist, Math.sin(ang) * dist, chunkRad * 0.5);
            chunk.rotation.set(Math.random() * 3, Math.random() * 3, Math.random() * 3);
            chunk.castShadow = true;
            pancakeGroup.add(chunk);
        }
        group.add(pancakeGroup);

        // --------------------------------------------------------------------
        // POI_SURVIVORS [25.0, -55.0, 20.0]: SKELETAL CONCRETE RUINS
        // (Matches Reference Image 4)
        // --------------------------------------------------------------------
        const survGroup = new THREE.Group();
        survGroup.position.set(25, -55, 0);

        // Jagged standing corner walls
        const wallA = new THREE.Mesh(new THREE.BoxGeometry(22, 2.5, 18), concreteMat);
        wallA.position.set(0, -9, 9);
        wallA.rotation.z = 0.08;
        wallA.castShadow = true;
        survGroup.add(wallA);

        const wallB = new THREE.Mesh(new THREE.BoxGeometry(2.5, 20, 16), concreteMat);
        wallB.position.set(-10, 0, 8);
        wallB.rotation.z = -0.05;
        wallB.castShadow = true;
        survGroup.add(wallB);

        // Exposed collapsed floor slab dipping into center
        const survSlab = new THREE.Mesh(new THREE.BoxGeometry(18, 16, 0.8), concreteMat);
        survSlab.position.set(2, 0, 4.5);
        survSlab.rotation.set(0.24, -0.18, 0.1);
        survGroup.add(survSlab);

        // Rubble mound in center
        for (let k = 0; k < 12; k++) {
            const rad = 1.4 + Math.random() * 1.8;
            const rChunk = new THREE.Mesh(new THREE.DodecahedronGeometry(rad, 0), rubbleMat);
            rChunk.position.set((Math.random() - 0.5) * 14, (Math.random() - 0.5) * 14, rad * 0.6);
            rChunk.rotation.set(Math.random() * 3, Math.random() * 3, Math.random() * 3);
            survGroup.add(rChunk);
        }

        // Survivor triage search marker
        const beaconPin = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, 6.0, 6), new THREE.MeshBasicMaterial({ color: 0xff1744 }));
        beaconPin.position.set(4, 4, 3.0);
        survGroup.add(beaconPin);
        const beaconOrb = new THREE.Mesh(new THREE.SphereGeometry(1.2, 8, 8), new THREE.MeshBasicMaterial({ color: 0xff1744 }));
        beaconOrb.position.set(4, 4, 6.5);
        survGroup.add(beaconOrb);

        group.add(survGroup);

        // --------------------------------------------------------------------
        // POI_HAZARD [-20.0, 95.0, 26.0]: CHEMICAL PROCESSING FACILITY
        // --------------------------------------------------------------------
        const hazGroup = new THREE.Group();
        hazGroup.position.set(-20, 95, 0);

        // Main industrial facility block (partially collapsed roof)
        const hazMain = new THREE.Mesh(new THREE.BoxGeometry(34, 24, 16), slateMat);
        hazMain.position.z = 8;
        hazMain.rotation.z = -0.05;
        hazMain.castShadow = true;
        hazGroup.add(hazMain);

        // Ruptured & Dented Silos
        [-11, 11].forEach((sx, idx) => {
            const siloH = 20;
            const silo = new THREE.Mesh(new THREE.CylinderGeometry(5.2, 5.2, siloH, 12), concreteMat);
            silo.rotation.x = Math.PI / 2;
            silo.position.set(sx, 0, siloH / 2);
            // Dented tilt on the right silo
            if (idx === 1) {
                silo.rotation.z = -0.22;
                silo.rotation.y = 0.15;
            }
            silo.castShadow = true;
            hazGroup.add(silo);

            // Hazard warning stripes
            const band = new THREE.Mesh(new THREE.CylinderGeometry(5.3, 5.3, 2.5, 12), hazardYellowMat);
            band.rotation.x = Math.PI / 2;
            band.position.set(sx, 0, 13);
            if (idx === 1) {
                band.rotation.z = -0.22;
                band.rotation.y = 0.15;
            }
            hazGroup.add(band);
        });

        // Chemical spill stain on ground
        const spillGeo = new THREE.CircleGeometry(12, 12);
        const spillMat = new THREE.MeshBasicMaterial({ color: 0x854d0e, transparent: true, opacity: 0.7 });
        const spill = new THREE.Mesh(spillGeo, spillMat);
        spill.position.set(6, 6, 0.15);
        hazGroup.add(spill);

        group.add(hazGroup);

        // --------------------------------------------------------------------
        // POI_HOSPITAL [-60.0, 25.0, 30.0]: ST. JUDE TRAUMA HOSPITAL
        // --------------------------------------------------------------------
        const hospGroup = new THREE.Group();
        hospGroup.position.set(-60, 25, 0);

        // Main hospital wing (Standing)
        const hospMain = new THREE.Mesh(new THREE.BoxGeometry(32, 40, 28), creamWallMat);
        hospMain.position.set(-8, 0, 14);
        hospMain.castShadow = true;
        hospGroup.add(hospMain);

        // Collapsed East wing into rubble mound
        const hospEast = new THREE.Mesh(new THREE.BoxGeometry(18, 30, 8), rubbleMat);
        hospEast.position.set(16, 0, 4);
        hospEast.rotation.set(0.12, -0.18, 0);
        hospEast.castShadow = true;
        hospGroup.add(hospEast);

        // Rooftop Helipad on standing main wing
        const hospPad = new THREE.Mesh(new THREE.CylinderGeometry(8.0, 8.5, 1.0, 16), slateMat);
        hospPad.rotation.x = Math.PI / 2;
        hospPad.position.set(-8, 0, 28.5);
        hospGroup.add(hospPad);

        const hospCrossH = new THREE.Mesh(new THREE.BoxGeometry(6, 2, 0.1), redCrossMat);
        hospCrossH.position.set(-8, 0, 29.1);
        hospGroup.add(hospCrossH);
        const hospCrossV = new THREE.Mesh(new THREE.BoxGeometry(2, 6, 0.1), redCrossMat);
        hospCrossV.position.set(-8, 0, 29.1);
        hospGroup.add(hospCrossV);

        const hospRing = new THREE.Mesh(new THREE.RingGeometry(7.0, 7.8, 16), new THREE.MeshBasicMaterial({ color: 0x00ff66, side: THREE.DoubleSide }));
        hospRing.position.set(-8, 0, 29.1);
        hospGroup.add(hospRing);

        group.add(hospGroup);
        rooftopRelayNodes.push(new THREE.Vector3(-68, 25, 30));

        // --------------------------------------------------------------------
        // POI_SUBSTATION [115.0, -40.0, 24.0]: EAST REGIONAL SUBSTATION
        // --------------------------------------------------------------------
        const subGroup = new THREE.Group();
        subGroup.position.set(115, -40, 0);

        const subBldg = new THREE.Mesh(new THREE.BoxGeometry(24, 22, 14), slateMat);
        subBldg.position.z = 7;
        subBldg.castShadow = true;
        subGroup.add(subBldg);

        // Bent & Collapsed Transmission Pylon (Tilted 38 deg)
        const subPylon = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 1.0, 26, 4), concreteMat);
        subPylon.position.set(0, 0, 12);
        subPylon.rotation.set(0.65, 0.2, 0.3);
        subPylon.castShadow = true;
        subGroup.add(subPylon);

        // Broken transformers & rubble
        [[-7, -6], [-7, 6], [7, -6], [7, 6]].forEach(([tx, ty], idx) => {
            const tr = new THREE.Mesh(new THREE.BoxGeometry(4.5, 4.5, 6), hazardYellowMat);
            tr.position.set(tx, ty, 3.0);
            if (idx === 1) tr.rotation.set(0.2, 0.3, 0);
            subGroup.add(tr);
        });

        group.add(subGroup);

        // --------------------------------------------------------------------
        // POI_SHELTER [-50.0, -85.0, 16.0]: METRO CIVIC EMERGENCY SHELTER
        // --------------------------------------------------------------------
        const shelterGroup = new THREE.Group();
        shelterGroup.position.set(-50, -85, 0);

        // Main shelter pavilion
        const shelterPav = new THREE.Mesh(new THREE.BoxGeometry(26, 18, 9), creamWallMat);
        shelterPav.position.z = 4.5;
        shelterPav.castShadow = true;
        shelterGroup.add(shelterPav);

        // Relief tents array
        const tentCoords = [[-12, 14], [0, 14], [12, 14], [-12, -14], [0, -14], [12, -14]];
        tentCoords.forEach(([tx, ty]) => {
            const tent = new THREE.Mesh(new THREE.ConeGeometry(3.2, 4.0, 4), tentMat);
            tent.rotation.y = Math.PI / 4;
            tent.rotation.x = Math.PI / 2;
            tent.position.set(tx, ty, 2.0);
            tent.castShadow = true;
            shelterGroup.add(tent);
        });

        // Communication antenna
        const sAnt = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.4, 14, 6), concreteMat);
        sAnt.rotation.x = Math.PI / 2;
        sAnt.position.set(10, 0, 14);
        shelterGroup.add(sAnt);

        group.add(shelterGroup);

        // --------------------------------------------------------------------
        // LOW-RISE RESIDENTIAL & COMMERCIAL DISASTER BLOCKS
        // --------------------------------------------------------------------
        const lowRiseBuildings = [
            { x: -85, y: -85, w: 26, d: 18, h: 12, rot: 0.12, mat: terracottaMat },
            { x: -115, y: -75, w: 24, d: 20, h: 14, rot: -0.15, mat: ochreMat },
            { x: -50, y: -60, w: 22, d: 16, h: 11, rot: 0.08, mat: creamWallMat },
            { x: 85, y: -80, w: 22, d: 18, h: 12, rot: -0.1, mat: terracottaMat },
            { x: 145, y: -65, w: 28, d: 24, h: 16, rot: 0.05, mat: slateMat },
            { x: 145, y: 40, w: 26, d: 26, h: 52, rot: -0.08, mat: ochreMat },
            { x: 85, y: 85, w: 28, d: 26, h: 48, rot: 0.12, mat: concreteMat },
            { x: -70, y: 115, w: 28, d: 28, h: 56, rot: -0.1, mat: slateMat },
        ];

        lowRiseBuildings.forEach(b => {
            const bGroup = new THREE.Group();
            bGroup.position.set(b.x, b.y, 0);
            bGroup.rotation.z = b.rot;

            const bMesh = new THREE.Mesh(new THREE.BoxGeometry(b.w, b.d, b.h), b.mat);
            bMesh.position.z = b.h / 2;
            bMesh.castShadow = true;
            bMesh.receiveShadow = true;
            bGroup.add(bMesh);

            // Tilted gable or shed roof
            const roof = new THREE.Mesh(new THREE.ConeGeometry(b.w * 0.7, 5, 4), terracottaMat);
            roof.rotation.y = Math.PI / 4;
            roof.rotation.x = Math.PI / 2;
            roof.position.set(0, 0, b.h + 2.5);
            roof.castShadow = true;
            bGroup.add(roof);

            group.add(bGroup);
        });

        // --------------------------------------------------------------------
        // PERVASIVE STREET DEBRIS, CONCRETE CHUNKS & ABANDONED VEHICLES
        // (Matching Reference Images 3 & 4)
        // --------------------------------------------------------------------
        const debrisCoords = [
            [-75, 5], [-50, 10], [-35, 30], [-10, 40], [5, 25],
            [-95, -40], [-70, -45], [-45, -50], [-25, -70], [10, -75],
            [90, -45], [105, -55], [130, -35], [-125, 5], [-105, 35],
            [-65, 75], [-40, 70], [-15, 65], [75, 40], [105, 30],
        ];

        debrisCoords.forEach(([dx, dy]) => {
            const chunkCount = 2 + Math.floor(Math.random() * 3);
            for (let c = 0; c < chunkCount; c++) {
                const rad = 1.0 + Math.random() * 1.6;
                const chunk = new THREE.Mesh(new THREE.DodecahedronGeometry(rad, 0), rubbleMat);
                chunk.position.set(dx + (Math.random() - 0.5) * 8, dy + (Math.random() - 0.5) * 8, rad * 0.5);
                chunk.rotation.set(Math.random() * 3, Math.random() * 3, Math.random() * 3);
                chunk.castShadow = true;
                group.add(chunk);
            }
        });

        // Emergency Vehicles scattered in intersections
        // --------------------------------------------------------------------
        // MASSIVE EARTHQUAKE RUINS & CITY FLOOD
        // --------------------------------------------------------------------
        // 1. Massive Choppy Flood Water Plane
        const floodMat = new THREE.MeshStandardMaterial({
            color: 0x3892d3, // Rich vibrant natural sky-blue water
            transparent: true,
            opacity: 0.85, // Proper water translucency
            roughness: 0.14, // Subtle surface gloss
            metalness: 0.08, // Liquid dielectric reflectivity
            flatShading: true
        });
        
        // High-segment plane for choppy waves (covers the 1000m city from Y=-480 to Y=+510, leaving GCS Hub at Y=-575 dry & clear)
        const floodGeo = new THREE.PlaneGeometry(1040, 990, 80, 80);
        
        // Perturb vertices to create a low-poly turbulent flood surface
        const posAttribute = floodGeo.attributes.position;
        for (let i = 0; i < posAttribute.count; i++) {
            const z = posAttribute.getZ(i);
            // Add wave noise
            const noise = (Math.random() - 0.5) * 3.5; 
            posAttribute.setZ(i, z + noise);
        }
        floodGeo.computeVertexNormals();

        const floodMesh = new THREE.Mesh(floodGeo, floodMat);
        floodMesh.position.set(0, 15, 15); // Floods the city up to 15m; terminates at Y = -480, keeping GCS Hub dry
        
        floodMesh.receiveShadow = true;
        group.add(floodMesh);
        
        // 2. Displaced Earthquake Plates (Ground Ruptures)
        const plateMat = new THREE.MeshStandardMaterial({
            color: 0x11161d, // Deep dark asphalt/earth
            roughness: 0.92,
            flatShading: true
        });
        for (let i = 0; i < 40; i++) {
            const px = (Math.random() - 0.5) * 800;
            const py = (Math.random() - 0.5) * 800;
            if (Math.abs(px) < 100 && Math.abs(py) < 100) continue;
            
            const plateW = 40 + Math.random() * 80;
            const plateD = 20 + Math.random() * 60;
            const plateMesh = new THREE.Mesh(new THREE.BoxGeometry(plateW, plateD, 6), plateMat);
            plateMesh.position.set(px, py, 2 + Math.random() * 5);
            plateMesh.rotation.set(
                (Math.random() - 0.5) * 0.4,
                (Math.random() - 0.5) * 0.4,
                Math.random() * Math.PI
            );
            group.add(plateMesh);
        }

        // Additional high-contrast material palette
        const brickCrimsonMat = new THREE.MeshStandardMaterial({ color: 0x8b1e1e, roughness: 0.72, flatShading: true });
        const darkGraphiteMat = new THREE.MeshStandardMaterial({ color: 0x1c2128, roughness: 0.85, flatShading: true });
        const deepTealMat = new THREE.MeshStandardMaterial({ color: 0x0f4c5c, roughness: 0.75, flatShading: true });
        const oxideAmberMat = new THREE.MeshStandardMaterial({ color: 0xd96b14, roughness: 0.75, flatShading: true });

        // 3. Massive Destroyed Buildings
        const hugeHouseGeometries = [
            new THREE.BoxGeometry(45, 55, 60),
            new THREE.BoxGeometry(50, 45, 90),
            new THREE.BoxGeometry(60, 60, 45),
            new THREE.BoxGeometry(35, 45, 50),
            new THREE.BoxGeometry(65, 40, 75)
        ];
        
        const glassTex = createGlassFacadeTexture();
        const beigeTex = createBeigeFacadeTexture();
        const roofTex = createTerracottaRoofTexture();
        
        const glassFacadeMat = new THREE.MeshStandardMaterial({ map: glassTex, roughness: 0.2, metalness: 0.6 });
        const beigeFacadeMat = new THREE.MeshStandardMaterial({ map: beigeTex, roughness: 0.8 });
        const roofMat = new THREE.MeshStandardMaterial({ map: roofTex, roughness: 0.9, flatShading: true });

        // Diverse, non-dark architectural palette (White, Sky Blue, Canary Yellow, Turquoise, Mint, Coral, Peach, Lilac, Orange)
        const crispWhiteMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.55, flatShading: true });
        const sunnyYellowMat = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.55, flatShading: true });
        const skyBlueMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, roughness: 0.55, flatShading: true });
        const turquoiseMat = new THREE.MeshStandardMaterial({ color: 0x2dd4bf, roughness: 0.55, flatShading: true });
        const coralRedMat = new THREE.MeshStandardMaterial({ color: 0xf43f5e, roughness: 0.60, flatShading: true });
        const warmOrangeMat = new THREE.MeshStandardMaterial({ color: 0xfb923c, roughness: 0.55, flatShading: true });
        const mintGreenMat = new THREE.MeshStandardMaterial({ color: 0x4ade80, roughness: 0.60, flatShading: true });
        const lilacPurpleMat = new THREE.MeshStandardMaterial({ color: 0xc084fc, roughness: 0.55, flatShading: true });
        const peachMat = new THREE.MeshStandardMaterial({ color: 0xfda4af, roughness: 0.60, flatShading: true });

        // Multi-colored roofs (Mediterranean Blue, Spanish Orange, Golden Ochre, Mint Copper, White, Crimson, Terracotta)
        const roofMaterials = [
            new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.70, flatShading: true }), // Santorini Blue
            new THREE.MeshStandardMaterial({ color: 0xf97316, roughness: 0.70, flatShading: true }), // Spanish Orange
            new THREE.MeshStandardMaterial({ color: 0xeab308, roughness: 0.65, flatShading: true }), // Golden Tile
            new THREE.MeshStandardMaterial({ color: 0x059669, roughness: 0.70, flatShading: true }), // Verdigris Mint
            new THREE.MeshStandardMaterial({ color: 0xe11d48, roughness: 0.70, flatShading: true }), // Rose Crimson
            new THREE.MeshStandardMaterial({ color: 0x8b5cf6, roughness: 0.65, flatShading: true }), // Lilac Tile
            new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.60, flatShading: true }), // Modern Clean White
            roofMat // Classic Textured Terracotta
        ];

        // Diverse colorful materials for building walls & bases
        const houseMaterials = [
            crispWhiteMat,
            sunnyYellowMat,
            skyBlueMat,
            turquoiseMat,
            coralRedMat,
            warmOrangeMat,
            mintGreenMat,
            lilacPurpleMat,
            peachMat,
            beigeFacadeMat,
            glassFacadeMat
        ];

        let seed = 9999;
        function random() {
            seed = (seed * 9301 + 49297) % 233280;
            return seed / 233280;
        }

        const safeDistanceCore = 170;

        for (let gx = -480; gx <= 480; gx += 60) {
            for (let gy = -480; gy <= 480; gy += 60) {
                // Randomly skip some plots to create voids
                if (random() > 0.8) continue;

                // Add slight organic jitter but keep them roughly grid-aligned
                const rx = gx + (random() - 0.5) * 15;
                const ry = gy + (random() - 0.5) * 15;
                
                // Avoid central core, mountain, and river areas
                if (Math.abs(rx) < safeDistanceCore && Math.abs(ry) < safeDistanceCore) continue;
                // Avoid GCS hub area entirely
                if (ry < -400) continue;

                const hGroup = new THREE.Group();
                hGroup.position.set(rx, ry, 0);

                // Grid alignment (0, 90, 180, 270 degrees)
                const yaw = Math.floor(random() * 4) * (Math.PI / 2);

                // Create shattered/collapsed buildings with high visual contrast
                const geo = hugeHouseGeometries[Math.floor(random() * hugeHouseGeometries.length)];
                const baseMat = houseMaterials[Math.floor(random() * houseMaterials.length)];
                
                // Base remaining intact structure
                const intactRatio = 0.2 + random() * 0.4;
                const baseMesh = new THREE.Mesh(
                    new THREE.BoxGeometry(geo.parameters.width, geo.parameters.height, geo.parameters.depth * intactRatio),
                    baseMat
                );
                baseMesh.position.z = (geo.parameters.depth * intactRatio) / 2;
                baseMesh.castShadow = true;
                baseMesh.receiveShadow = true;
                hGroup.add(baseMesh);

                // Collapsed upper half - diverse non-dark colors
                const upperMat = houseMaterials[Math.floor(random() * houseMaterials.length)];
                
                if (random() > 0.6) {
                    // Sheared standing top section
                    const upperHeight = geo.parameters.depth * (1 - intactRatio);
                    const upperMesh = new THREE.Mesh(
                        new THREE.BoxGeometry(geo.parameters.width, geo.parameters.height, upperHeight),
                        upperMat
                    );
                    
                    // Add a multi-colored pitched roof (Santorini blue, golden, mint, orange, white, etc.)
                    const chosenRoofMat = roofMaterials[Math.floor(random() * roofMaterials.length)];
                    const roofGeo = new THREE.ConeGeometry(Math.max(geo.parameters.width, geo.parameters.height) * 0.8, 15, 4);
                    roofGeo.rotateX(Math.PI / 2);
                    roofGeo.rotateZ(Math.PI / 4);
                    const roof = new THREE.Mesh(roofGeo, chosenRoofMat);
                    roof.position.z = upperHeight / 2 + 7.5;
                    roof.castShadow = true;
                    upperMesh.add(roof);

                    upperMesh.position.set(
                        (random() - 0.5) * 15,
                        (random() - 0.5) * 15,
                        (geo.parameters.depth * intactRatio) + (upperHeight / 2) - 2
                    );
                    // Slight lean, no crazy rotation
                    upperMesh.rotation.set(
                        (random() - 0.5) * 0.4,
                        (random() - 0.5) * 0.4,
                        (random() - 0.5) * 0.2
                    );
                    upperMesh.castShadow = true;
                    upperMesh.receiveShadow = true;
                    hGroup.add(upperMesh);
                } else {
                    // Pancake collapse (flat rubble slabs piled up in diverse light colors)
                    const numSlabs = 2 + Math.floor(random() * 3);
                    for (let s = 0; s < numSlabs; s++) {
                        const slabMat = houseMaterials[(s + Math.floor(random() * houseMaterials.length)) % houseMaterials.length];
                        const slab = new THREE.Mesh(
                            new THREE.BoxGeometry(geo.parameters.width * 1.05, geo.parameters.height * 1.05, 4),
                            slabMat
                        );
                        slab.position.set(
                            (random() - 0.5) * 8,
                            (random() - 0.5) * 8,
                            (geo.parameters.depth * intactRatio) + (s * 5) + 2
                        );
                        slab.rotation.set(
                            (random() - 0.5) * 0.25,
                            (random() - 0.5) * 0.25,
                            (random() - 0.5) * 0.25
                        );
                        slab.castShadow = true;
                        slab.receiveShadow = true;
                        hGroup.add(slab);
                    }
                }

                // Massive rubble mound at the base
                const numRubble = 4 + Math.floor(random() * 5);
                for(let r=0; r<numRubble; r++) {
                    const rubbleSize = 8 + random() * 12;
                    const rMesh = new THREE.Mesh(new THREE.IcosahedronGeometry(rubbleSize, 0), rubbleMat);
                    rMesh.position.set(
                        (random() - 0.5) * (geo.parameters.width + 10),
                        (random() - 0.5) * (geo.parameters.height + 10),
                        rubbleSize * 0.4
                    );
                    rMesh.rotation.set(random(), random(), random());
                    rMesh.castShadow = true;
                    hGroup.add(rMesh);
                }
                
                // Set yaw alignment
                hGroup.rotation.z = yaw;
                
                // Add extreme earthquake tilt to the entire plot occasionally
                if (random() > 0.5) {
                    hGroup.rotation.x = (random() - 0.5) * 0.3;
                    hGroup.rotation.y = (random() - 0.5) * 0.3;
                    hGroup.position.z -= random() * 8;
                }

                group.add(hGroup);
            }
        }
        
        // Massive dead/uprooted trees
        const treeMat = new THREE.MeshStandardMaterial({ color: 0x2e3b32, roughness: 0.9, flatShading: true });
        const trunkMat = new THREE.MeshStandardMaterial({ color: 0x3d2817, roughness: 1.0, flatShading: true });
        
        for (let i = 0; i < 150; i++) {
            const rx = (random() - 0.5) * 960;
            const ry = (random() - 0.5) * 960;
            if (Math.abs(rx) < safeDistanceCore && Math.abs(ry) < safeDistanceCore) continue;
            if (ry < -450) continue;
            
            const tGroup = new THREE.Group();
            tGroup.position.set(rx, ry, 0);
            
            // Almost all trees fallen in a flood/earthquake
            tGroup.rotation.set(1.4 + random()*0.4, (random()-0.5)*3, 0);
            tGroup.position.z = 8.0; // Bobbing in water or on rubble

            const trunk = new THREE.Mesh(new THREE.CylinderGeometry(1.5, 2.5, 20, 5), trunkMat);
            trunk.rotation.x = Math.PI / 2;
            trunk.position.z = 10;
            tGroup.add(trunk);
            
            const foliage = new THREE.Mesh(new THREE.ConeGeometry(8, 25, 5), treeMat);
            foliage.rotation.x = Math.PI / 2;
            foliage.position.z = 25;
            foliage.castShadow = true;
            tGroup.add(foliage);
            
            group.add(tGroup);
        }


        const emVehicles = [
            // Fire Engine near West High-Rise
            { x: -45, y: 15, col: 0xd32f2f, rot: 0.4, type: "fire" },
            // Ambulance near Hospital
            { x: -45, y: 35, col: 0xffffff, rot: -0.6, type: "amb" },
            // Civilian car near Central Plaza
            { x: 5, y: -35, col: 0xd97706, rot: 1.2, type: "car" },
            // Tilted car near Shelter
            { x: -35, y: -75, col: 0x3b82f6, rot: 0.8, type: "car" },
        ];

        emVehicles.forEach(v => {
            const vG = new THREE.Group();
            vG.position.set(v.x, v.y, 0.8);
            vG.rotation.z = v.rot;

            const isBig = (v.type === "fire");
            const vBody = new THREE.Mesh(
                new THREE.BoxGeometry(isBig ? 3.0 : 2.0, isBig ? 6.5 : 4.0, isBig ? 2.2 : 1.3),
                new THREE.MeshStandardMaterial({ color: v.col, roughness: 0.6, flatShading: true })
            );
            vG.add(vBody);

            if (v.type === "amb") {
                // Red cross on ambulance roof
                const rx = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.25, 0.05), redCrossMat);
                rx.position.set(0, 0, 0.7);
                vG.add(rx);
                const ry = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.8, 0.05), redCrossMat);
                ry.position.set(0, 0, 0.7);
                vG.add(ry);
            }

            group.add(vG);
        });

        // Tilted street lamp posts
        const lamps = [[-35, 10], [-35, -20], [-10, 10], [5, -15], [-70, -35]];
        lamps.forEach(([lx, ly]) => {
            const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.15, 6.0, 6), concreteMat);
            pole.position.set(lx, ly, 3.0);
            pole.rotation.set(0.35 + Math.random() * 0.2, Math.random() * 0.3, 0);
            group.add(pole);
        });

        // --------------------------------------------------------------------
        // Big Rectangular High-Rise Buildings in Central Empty Water Space
        // --------------------------------------------------------------------
        const centerBuildings = [
            // Center-West Grand Monolith (Crisp White with Sky Blue Crown)
            { x: -18, y: -15, w: 46, d: 32, h: 76, mat: crispWhiteMat, crownMat: skyBlueMat, rot: 0.08 },
            // Center-East Financial Slab (Canary Yellow with Coral Red Trim)
            { x: 38, y: 16, w: 34, d: 52, h: 84, mat: sunnyYellowMat, crownMat: coralRedMat, rot: -0.12 },
            // North-Central Corporate Complex (Glass Facade with Turquoise Parapet)
            { x: -22, y: 46, w: 48, d: 28, h: 68, mat: glassFacadeMat, crownMat: turquoiseMat, rot: 0.15 },
            // North-East High-Rise Tower (Mint Green with Clean White Steps)
            { x: 26, y: 68, w: 36, d: 44, h: 80, mat: mintGreenMat, crownMat: crispWhiteMat, rot: -0.05 },
            // South-West Massive Rectangular Block (Lilac Purple with Warm Orange Accents)
            { x: -42, y: -42, w: 44, d: 36, h: 62, mat: lilacPurpleMat, crownMat: warmOrangeMat, rot: 0.10 },
            // South-East Waterfront Commercial High-Rise (Sky Blue & Peach Setbacks)
            { x: 22, y: -26, w: 50, d: 30, h: 72, mat: skyBlueMat, crownMat: peachMat, rot: -0.18 },
        ];

        centerBuildings.forEach(b => {
            const bGroup = new THREE.Group();
            bGroup.position.set(b.x, b.y, 0);
            bGroup.rotation.z = b.rot;
            // Subtle earthquake tilt in the flood water
            bGroup.rotation.x = (b.x % 2 === 0 ? 0.04 : -0.04);
            bGroup.rotation.y = (b.y % 2 === 0 ? 0.03 : -0.03);

            // Main Rectangular Body
            const bodyMesh = new THREE.Mesh(
                new THREE.BoxGeometry(b.w, b.d, b.h),
                b.mat
            );
            bodyMesh.position.z = b.h / 2;
            bodyMesh.castShadow = true;
            bodyMesh.receiveShadow = true;
            bGroup.add(bodyMesh);

            // Stepped Upper Rectangular Penthouse / Crown
            const crownH = 14;
            const crownMesh = new THREE.Mesh(
                new THREE.BoxGeometry(b.w * 0.75, b.d * 0.75, crownH),
                b.crownMat
            );
            crownMesh.position.set(0, 0, b.h + crownH / 2);
            crownMesh.castShadow = true;
            crownMesh.receiveShadow = true;
            bGroup.add(crownMesh);

            // Rooftop Parapet Rim / Mechanical Enclosure
            const rimMesh = new THREE.Mesh(
                new THREE.BoxGeometry(b.w * 0.5, b.d * 0.5, 4),
                b.crownMat
            );
            rimMesh.position.set(0, 0, b.h + crownH + 2);
            rimMesh.castShadow = true;
            bGroup.add(rimMesh);

            group.add(bGroup);
        });

        // --------------------------------------------------------------------
        // TOWER 5: Warm Sandstone / Golden Amber High-Rise (Relay Hub at 70, -12)
        // --------------------------------------------------------------------
        const t5Group = new THREE.Group();
        t5Group.position.set(70, -12, 0);
        const t5W = 40, t5D = 42, t5H = 76;
        const t5Mesh = new THREE.Mesh(new THREE.BoxGeometry(t5W, t5D, t5H), sunnyYellowMat);
        t5Mesh.position.z = t5H / 2;
        t5Mesh.castShadow = true;
        t5Mesh.receiveShadow = true;
        t5Group.add(t5Mesh);

        // Elevator penthouse on roof
        const t5Pent = new THREE.Mesh(new THREE.BoxGeometry(16, 18, 9), warmOrangeMat);
        t5Pent.position.set(-8, 6, t5H + 4.5);
        t5Group.add(t5Pent);

        // Glowing Circular Rooftop Relay Beacon Pad
        const relayPad = new THREE.Mesh(
            new THREE.RingGeometry(2, 9, 32),
            new THREE.MeshBasicMaterial({ color: 0xffd600, side: THREE.DoubleSide, transparent: true, opacity: 0.85 })
        );
        relayPad.position.set(8, -6, t5H + 0.3);
        t5Group.add(relayPad);

        const relayInnerRing = new THREE.Mesh(
            new THREE.RingGeometry(0.5, 3.5, 32),
            new THREE.MeshBasicMaterial({ color: 0x00e5ff, side: THREE.DoubleSide })
        );
        relayInnerRing.position.set(8, -6, t5H + 0.4);
        t5Group.add(relayInnerRing);

        group.add(t5Group);
        rooftopRelayNodes.push(new THREE.Vector3(78, -18, t5H + 1));

        // --------------------------------------------------------------------
        // TOWER 6: Coral-Terracotta High-Rise East at (118, -42)
        // --------------------------------------------------------------------
        const t6Group = new THREE.Group();
        t6Group.position.set(118, -42, 0);
        const t6H = 82;
        const t6Mesh = new THREE.Mesh(new THREE.BoxGeometry(32, 36, t6H), coralRedMat);
        t6Mesh.position.z = t6H / 2;
        t6Mesh.castShadow = true;
        t6Mesh.receiveShadow = true;
        t6Group.add(t6Mesh);
        group.add(t6Group);
        rooftopRelayNodes.push(new THREE.Vector3(118, -42, t6H + 1));

        // --------------------------------------------------------------------
        // TOWER 7: Modern Curved White High-Rise at (135, 20)
        // --------------------------------------------------------------------
        const t7Group = new THREE.Group();
        t7Group.position.set(135, 20, 0);
        const t7H = 66;
        const t7Mesh = new THREE.Mesh(new THREE.BoxGeometry(30, 32, t7H), crispWhiteMat);
        t7Mesh.position.z = t7H / 2;
        t7Mesh.castShadow = true;
        t7Mesh.receiveShadow = true;
        t7Group.add(t7Mesh);
        group.add(t7Group);

        // --------------------------------------------------------------------
        // TOWER 8: North Telecom Megatower at (30, 110)
        // --------------------------------------------------------------------
        const t8Group = new THREE.Group();
        t8Group.position.set(30, 110, 0);
        const t8Mesh = new THREE.Mesh(new THREE.BoxGeometry(36, 36, 120), glassFacadeMat);
        t8Mesh.position.z = 60;
        t8Mesh.castShadow = true;
        t8Group.add(t8Mesh);

        // Slender Telecom Spire & Red Flashing Beacon
        const t8Spire = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 1.2, 28, 6), new THREE.MeshStandardMaterial({ color: 0xe2e8f0, metalness: 0.8 }));
        t8Spire.rotation.x = Math.PI / 2;
        t8Spire.position.z = 120 + 14;
        t8Group.add(t8Spire);

        const t8Beacon = new THREE.Mesh(new THREE.SphereGeometry(1.4, 8, 8), new THREE.MeshBasicMaterial({ color: 0xff1744 }));
        t8Beacon.position.z = 135;
        t8Group.add(t8Beacon);

        group.add(t8Group);
        rooftopRelayNodes.push(new THREE.Vector3(30, 110, 122));

        // --------------------------------------------------------------------
        // TOWER 9: Northwest Cyan High-Rise at (-70, 115)
        // --------------------------------------------------------------------
        const t9Group = new THREE.Group();
        t9Group.position.set(-70, 115, 0);
        const t9Mesh = new THREE.Mesh(new THREE.BoxGeometry(32, 34, 98), turquoiseMat);
        t9Mesh.position.z = 49;
        t9Mesh.castShadow = true;
        t9Group.add(t9Mesh);
        group.add(t9Group);
        rooftopRelayNodes.push(new THREE.Vector3(-70, 115, 100));

        // --------------------------------------------------------------------
        // TOWER 10: North Commerce Center at (85, 95)
        // --------------------------------------------------------------------
        const t10Group = new THREE.Group();
        t10Group.position.set(85, 95, 0);
        const t10Mesh = new THREE.Mesh(new THREE.BoxGeometry(34, 32, 88), mintGreenMat);
        t10Mesh.position.z = 44;
        t10Mesh.castShadow = true;
        t10Group.add(t10Mesh);
        group.add(t10Group);
        rooftopRelayNodes.push(new THREE.Vector3(85, 95, 90));

        // --------------------------------------------------------------------
        // TOWER 11: Stepped Tech High-Rise at (120, 110)
        // --------------------------------------------------------------------
        const t11Group = new THREE.Group();
        t11Group.position.set(120, 110, 0);
        const t11_1 = new THREE.Mesh(new THREE.BoxGeometry(30, 30, 52), lilacPurpleMat);
        t11_1.position.z = 26;
        t11_1.castShadow = true;
        t11Group.add(t11_1);
        const t11_2 = new THREE.Mesh(new THREE.BoxGeometry(22, 22, 26), crispWhiteMat);
        t11_2.position.z = 52 + 13;
        t11_2.castShadow = true;
        t11Group.add(t11_2);
        group.add(t11Group);

        // --------------------------------------------------------------------
        // TOWER 14: East Horizon Corporate Spire at (145, 45)
        // --------------------------------------------------------------------
        const t14Group = new THREE.Group();
        t14Group.position.set(145, 45, 0);
        const t14Mesh = new THREE.Mesh(new THREE.BoxGeometry(32, 30, 84), skyBlueMat);
        t14Mesh.position.z = 42;
        t14Mesh.castShadow = true;
        t14Group.add(t14Mesh);
        const t14Spire = new THREE.Mesh(new THREE.ConeGeometry(8, 20, 4), peachMat);
        t14Spire.rotation.x = Math.PI / 2;
        t14Spire.position.z = 84 + 10;
        t14Group.add(t14Spire);
        group.add(t14Group);
        rooftopRelayNodes.push(new THREE.Vector3(145, 45, 86));
    }

    // ------------------------------------------------------------------------
    // 6. Dedicated 3D Physical Launch Pads on GCS Apron
    // ------------------------------------------------------------------------
    function create3DLaunchPads(group) {
        const padsGroup = new THREE.Group();

        const padBaseGeo = new THREE.CylinderGeometry(4.2, 4.4, 0.25, 6);
        const padBaseMat = new THREE.MeshStandardMaterial({
            color: 0x141a24,
            metalness: 0.85,
            roughness: 0.25,
        });

        const crossGeoH = new THREE.BoxGeometry(3.6, 0.8, 0.05);
        const crossGeoV = new THREE.BoxGeometry(0.8, 3.6, 0.05);

        const padsConfig = [
            // Surveyors (UAV_1 to UAV_8) along y = -575 (75m south of black boundary)
            ...[-70, -50, -30, -10, 10, 30, 50, 70].map((x, i) => ({
                id: `UAV_${i + 1}`,
                x, y: -575,
                color: 0x00e5ff,
            })),
            // Relays (RELAY_1 to RELAY_4) along y = -566
            ...[-45, -15, 15, 45].map((x, i) => ({
                id: `RELAY_${i + 1}`,
                x, y: -566,
                color: 0xffd600,
            })),
            // Scouts (SCOUT_1 to SCOUT_4) along y = -583
            ...[-45, -15, 15, 45].map((x, i) => ({
                id: `SCOUT_${i + 1}`,
                x, y: -583,
                color: 0x00ff66,
            })),
        ];

        padsConfig.forEach(cfg => {
            const pGroup = new THREE.Group();
            pGroup.position.set(cfg.x, cfg.y, 0);

            // 1. Raised hexagonal carbon base slab
            const baseMesh = new THREE.Mesh(padBaseGeo, padBaseMat);
            baseMesh.rotation.x = Math.PI / 2;
            baseMesh.position.z = 0.22;
            baseMesh.receiveShadow = true;
            pGroup.add(baseMesh);

            // 2. Glowing perimeter LED accent ring
            const ringGeo = new THREE.RingGeometry(3.6, 4.1, 6);
            const ringMat = new THREE.MeshBasicMaterial({
                color: cfg.color,
                side: THREE.DoubleSide,
                transparent: true,
                opacity: 0.9,
            });
            const ringMesh = new THREE.Mesh(ringGeo, ringMat);
            ringMesh.position.z = 0.33;
            pGroup.add(ringMesh);

            // 3. Central landing cross
            const crossMat = new THREE.MeshBasicMaterial({ color: cfg.color, transparent: true, opacity: 0.75 });
            const ch = new THREE.Mesh(crossGeoH, crossMat);
            ch.position.z = 0.33;
            pGroup.add(ch);
            const cv = new THREE.Mesh(crossGeoV, crossMat);
            cv.position.z = 0.33;
            pGroup.add(cv);

            // 4. Perimeter safety navigation corner LED pucks
            const puckGeo = new THREE.CylinderGeometry(0.18, 0.18, 0.15, 8);
            const puckMat = new THREE.MeshBasicMaterial({ color: cfg.color });
            for (let k = 0; k < 6; k++) {
                const angle = (k / 6) * Math.PI * 2;
                const puck = new THREE.Mesh(puckGeo, puckMat);
                puck.rotation.x = Math.PI / 2;
                puck.position.set(Math.cos(angle) * 3.8, Math.sin(angle) * 3.8, 0.36);
                pGroup.add(puck);
            }

            padsGroup.add(pGroup);
        });

        group.add(padsGroup);
    }

    // ------------------------------------------------------------------------
    // Relocated GCS Flight Apron Platform (75m Outside South Boundary at Y = -575)
    // ------------------------------------------------------------------------
    function createHubApronPlatform(group) {
        const hubGroup = new THREE.Group();
        hubGroup.name = "gcs_hub_apron_platform";

        // 1. Concrete Sub-Foundation Slab (Width 184m x Depth 48m x Height 0.4m)
        const subBaseMat = new THREE.MeshStandardMaterial({
            color: 0x64748b, // High-contrast bright architectural concrete
            roughness: 0.65,
            metalness: 0.1,
        });
        const subBase = new THREE.Mesh(new THREE.BoxGeometry(184, 48, 0.4), subBaseMat);
        subBase.position.set(0, -575, -0.05); // Z from -0.25 to +0.15
        subBase.receiveShadow = true;
        hubGroup.add(subBase);

        // 2. Primary Tarmac Deck (Width 180m x Depth 44m x Height 0.2m)
        const tarmacMat = new THREE.MeshStandardMaterial({
            color: 0x334155, // Clean, crisp runway asphalt
            roughness: 0.6,
            metalness: 0.2,
        });
        const tarmac = new THREE.Mesh(new THREE.BoxGeometry(180, 44, 0.2), tarmacMat);
        tarmac.position.set(0, -575, 0.10); // Z from 0.0 to +0.20
        tarmac.receiveShadow = true;
        hubGroup.add(tarmac);

        // 3. Safety Yellow Perimeter Border Rails
        const borderMat = new THREE.MeshBasicMaterial({ color: 0xffd600 });
        const hw = 90, hd = 22; // half-width and half-depth
        const borderThickness = 0.5;
        // North & South borders
        [-hd, hd].forEach(dy => {
            const bMesh = new THREE.Mesh(new THREE.BoxGeometry(180, borderThickness, 0.05), borderMat);
            bMesh.position.set(0, -575 + dy, 0.22);
            hubGroup.add(bMesh);
        });
        // West & East borders
        [-hw, hw].forEach(dx => {
            const bMesh = new THREE.Mesh(new THREE.BoxGeometry(borderThickness, 44, 0.05), borderMat);
            bMesh.position.set(dx, -575, 0.22);
            hubGroup.add(bMesh);
        });

        // 4. White Runway Threshold Piano-Key Stripes
        const whiteMat = new THREE.MeshBasicMaterial({ color: 0xf1f5f9 });
        for (let y = -575 - 18; y <= -575 + 18; y += 4.5) {
            // West threshold keys
            const wKey = new THREE.Mesh(new THREE.BoxGeometry(5.0, 1.8, 0.04), whiteMat);
            wKey.position.set(-84, y, 0.22);
            hubGroup.add(wKey);
            // East threshold keys
            const eKey = new THREE.Mesh(new THREE.BoxGeometry(5.0, 1.8, 0.04), whiteMat);
            eKey.position.set(84, y, 0.22);
            hubGroup.add(eKey);
        }

        // 5. Centerline Yellow Dashed Taxiway along Y = -575 (excluding center GCS pad |X| < 14)
        for (let x = -75; x <= 75; x += 6) {
            if (Math.abs(x) < 14) continue;
            const dash = new THREE.Mesh(new THREE.BoxGeometry(3.6, 0.6, 0.04), borderMat);
            dash.position.set(x, -575, 0.22);
            hubGroup.add(dash);
        }

        // 6. Central GCS Ground Pad Base Ring at (0, -575)
        const gcsRingMat = new THREE.MeshBasicMaterial({ color: 0xffd600, side: THREE.DoubleSide });
        const gcsRing = new THREE.Mesh(new THREE.RingGeometry(10.6, 11.4, 36), gcsRingMat);
        gcsRing.position.set(0, -575, 0.23);
        hubGroup.add(gcsRing);

        const gcsInnerRingMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.6, side: THREE.DoubleSide });
        const gcsInnerRing = new THREE.Mesh(new THREE.RingGeometry(6.8, 7.2, 36), gcsInnerRingMat);
        gcsInnerRing.position.set(0, -575, 0.23);
        hubGroup.add(gcsInnerRing);

        // 7. Helipads: PAD-A (-65, -566) and PAD-B (+65, -566)
        [-65, 65].forEach(hx => {
            const hRing = new THREE.Mesh(new THREE.RingGeometry(5.2, 5.8, 32), borderMat);
            hRing.position.set(hx, -566, 0.23);
            hubGroup.add(hRing);

            const hInnerDashed = new THREE.Mesh(new THREE.RingGeometry(3.4, 3.8, 24), gcsInnerRingMat);
            hInnerDashed.position.set(hx, -566, 0.23);
            hubGroup.add(hInnerDashed);

            // "H" Letter Crossbars
            const hBarMat = new THREE.MeshBasicMaterial({ color: 0xffd600 });
            const v1 = new THREE.Mesh(new THREE.BoxGeometry(0.5, 3.8, 0.05), hBarMat);
            v1.position.set(hx - 1.2, -566, 0.24);
            hubGroup.add(v1);

            const v2 = new THREE.Mesh(new THREE.BoxGeometry(0.5, 3.8, 0.05), hBarMat);
            v2.position.set(hx + 1.2, -566, 0.24);
            hubGroup.add(v2);

            const horiz = new THREE.Mesh(new THREE.BoxGeometry(2.4, 0.5, 0.05), hBarMat);
            horiz.position.set(hx, -566, 0.24);
            hubGroup.add(horiz);
        });

        // 8. 75m Standoff Access Runway Connector (Connecting Y = -500 black city boundary to Y = -553 hub)
        const roadMat = new THREE.MeshStandardMaterial({
            color: 0x1c2430,
            roughness: 0.8,
            metalness: 0.2,
        });
        const road = new THREE.Mesh(new THREE.BoxGeometry(16, 52, 0.15), roadMat);
        road.position.set(0, -526, 0.10); // span Y = -500 to -552
        road.receiveShadow = true;
        hubGroup.add(road);

        // Road yellow boundary curbs
        [-8, 8].forEach(rx => {
            const curb = new THREE.Mesh(new THREE.BoxGeometry(0.4, 52, 0.06), borderMat);
            curb.position.set(rx, -526, 0.22);
            hubGroup.add(curb);
        });

        // Road dashed center line
        for (let ry = -548; ry <= -504; ry += 6) {
            const rDash = new THREE.Mesh(new THREE.BoxGeometry(0.5, 3.5, 0.05), whiteMat);
            rDash.position.set(0, ry, 0.22);
            hubGroup.add(rDash);
        }

        // 9. Visual 75m Standoff Measurement Bracket Line (West Flank at X = -80)
        const guideMat = new THREE.LineDashedMaterial({
            color: 0x00e5ff,
            dashSize: 3,
            gapSize: 2,
            transparent: true,
            opacity: 0.85,
        });
        const guideGeo = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(-80, -500, 0.5),
            new THREE.Vector3(-80, -575, 0.5),
        ]);
        const guideLine = new THREE.Line(guideGeo, guideMat);
        guideLine.computeLineDistances();
        hubGroup.add(guideLine);

        // Boundary tick at Y = -500
        const tick1 = new THREE.Mesh(new THREE.BoxGeometry(6, 0.6, 0.1), borderMat);
        tick1.position.set(-80, -500, 0.5);
        hubGroup.add(tick1);

        // Hub tick at Y = -575
        const tick2 = new THREE.Mesh(new THREE.BoxGeometry(6, 0.6, 0.1), borderMat);
        tick2.position.set(-80, -575, 0.5);
        hubGroup.add(tick2);

        // 10. High-Visibility Corner Floodlight Towers (4 corners of apron, 22m tall)
        const poleGeo = new THREE.CylinderGeometry(0.35, 0.6, 22.0, 8);
        const poleMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, metalness: 0.7, roughness: 0.2 });
        const lightGeo = new THREE.SphereGeometry(1.2, 10, 10);
        const lightMat = new THREE.MeshBasicMaterial({ color: 0xffea00 });

        [[-90, -597], [90, -597], [-90, -553], [90, -553]].forEach(([px, py]) => {
            const pole = new THREE.Mesh(poleGeo, poleMat);
            pole.rotation.x = Math.PI / 2;
            pole.position.set(px, py, 11.0);
            hubGroup.add(pole);

            // Double high-output floodlight lanterns
            const light = new THREE.Mesh(lightGeo, lightMat);
            light.position.set(px, py, 22.0);
            hubGroup.add(light);
            
            // Halo glow ring
            const halo = new THREE.Mesh(new THREE.RingGeometry(1.4, 2.4, 16), new THREE.MeshBasicMaterial({ color: 0x00f0ff, side: THREE.DoubleSide }));
            halo.position.set(px, py, 22.2);
            hubGroup.add(halo);
        });

        // 11. Main High-Visibility GCS Operations Tower & Beacon (Center-South at X=0, Y=-600)
        const gcsTowerGeo = new THREE.CylinderGeometry(0.8, 1.8, 36.0, 8);
        const gcsTowerMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, metalness: 0.85, roughness: 0.15 });
        const gcsTower = new THREE.Mesh(gcsTowerGeo, gcsTowerMat);
        gcsTower.rotation.x = Math.PI / 2;
        gcsTower.position.set(0, -600, 18.0);
        hubGroup.add(gcsTower);

        // Giant Red Aircraft Warning Strobe Beacon on top of GCS Tower
        const gcsBeacon = new THREE.Mesh(new THREE.SphereGeometry(2.2, 16, 16), new THREE.MeshBasicMaterial({ color: 0xff0033 }));
        gcsBeacon.position.set(0, -600, 36.5);
        hubGroup.add(gcsBeacon);

        // Cyan Comm Ring Halo
        const gcsCommRing = new THREE.Mesh(new THREE.TorusGeometry(3.5, 0.4, 8, 24), new THREE.MeshBasicMaterial({ color: 0x00e5ff }));
        gcsCommRing.position.set(0, -600, 34.0);
        hubGroup.add(gcsCommRing);

        group.add(hubGroup);
    }

    // ------------------------------------------------------------------------
    // Challenge Arena: 1000m x 1000m Operational Area + 75m West Operational Center
    // ------------------------------------------------------------------------
    function createChallengeArena(scene) {
        challengeGroup = new THREE.Group();
        challengeGroup.name = "challenge_arena";
        challengeGroup.visible = false; // Default inactive, toggled via setScenario
        scene.add(challengeGroup);

        // 1. Terrain Canvas: 1200m x 1100m tactical arena grid
        const c = document.createElement("canvas");
        c.width = 1024;
        c.height = 1024;
        const ctx = c.getContext("2d");

        // Dark tactical surface background
        ctx.fillStyle = "#101826";
        ctx.fillRect(0, 0, 1024, 1024);

        // Map dimensions: arena [0, 1000] x [-500, 500] plus Operational Center at X = -75
        function toCanv(x, y) {
            const cx = ((x - (-150)) / 1200) * 1024;
            const cy = ((550 - y) / 1100) * 1024;
            return [cx, cy];
        }

        // Minor grid lines every 50m
        ctx.strokeStyle = "rgba(40, 75, 110, 0.35)";
        ctx.lineWidth = 1;
        for (let gx = -150; gx <= 1050; gx += 50) {
            const [x1, y1] = toCanv(gx, -550);
            const [x2, y2] = toCanv(gx, 550);
            ctx.beginPath();
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
        }
        for (let gy = -550; gy <= 550; gy += 50) {
            const [x1, y1] = toCanv(-150, gy);
            const [x2, y2] = toCanv(1050, gy);
            ctx.beginPath();
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
        }

        // Major grid lines every 100m inside 1000m x 1000m Arena
        ctx.strokeStyle = "rgba(0, 229, 255, 0.4)";
        ctx.lineWidth = 1.5;
        for (let gx = 0; gx <= 1000; gx += 100) {
            const [x1, y1] = toCanv(gx, -500);
            const [x2, y2] = toCanv(gx, 500);
            ctx.beginPath();
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
            // Label
            ctx.font = "bold 10px 'Consolas', monospace";
            ctx.fillStyle = "rgba(0, 229, 255, 0.7)";
            ctx.fillText(`${gx}m`, x1 + 2, y1 - 4);
        }
        for (let gy = -500; gy <= 500; gy += 100) {
            const [x1, y1] = toCanv(0, gy);
            const [x2, y2] = toCanv(1000, gy);
            ctx.beginPath();
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
            // Label
            ctx.font = "bold 10px 'Consolas', monospace";
            ctx.fillStyle = "rgba(0, 229, 255, 0.7)";
            ctx.fillText(`${gy > 0 ? '+' : ''}${gy}m`, x1 + 4, y1 - 2);
        }

        // Operational Center Apron background at X = -75, Y in [-160, 160]
        const [apx1, apy1] = toCanv(-120, 160);
        const [apx2, apy2] = toCanv(-20, -160);
        ctx.fillStyle = "#1e293b";
        ctx.fillRect(apx1, apy1, apx2 - apx1, apy2 - apy1);
        ctx.strokeStyle = "#ffd600";
        ctx.lineWidth = 2.5;
        ctx.strokeRect(apx1, apy1, apx2 - apx1, apy2 - apy1);

        // Operational Center Header Stencil
        ctx.font = "bold 12px 'Consolas', monospace";
        ctx.fillStyle = "#ffd600";
        ctx.fillText("OPERATIONAL CENTER (75m WEST OF ARENA)", apx1 + 8, apy1 + 20);
        ctx.font = "bold 10px 'Consolas', monospace";
        ctx.fillStyle = "#00e5ff";
        ctx.fillText("MEITY / IIT BOMBAY / IISER BHOPAL SWARM CHALLENGE", apx1 + 8, apy1 + 36);

        // 1000m x 1000m Operational Arena Boundary Border
        const [bx1, by1] = toCanv(0, 500);
        const [bx2, by2] = toCanv(1000, -500);
        ctx.strokeStyle = "#00e5ff";
        ctx.lineWidth = 4;
        ctx.strokeRect(bx1, by1, bx2 - bx1, by2 - by1);

        // Hazard corner hatches
        ctx.strokeStyle = "#ffd600";
        ctx.lineWidth = 2;
        [[0, 500], [1000, 500], [1000, -500], [0, -500]].forEach(([cx, cy]) => {
            const [px, py] = toCanv(cx, cy);
            ctx.strokeRect(px - 14, py - 14, 28, 28);
        });

        // Main Arena Center Title
        const [tcx, tcy] = toCanv(500, 0);
        ctx.save();
        ctx.font = "bold 24px 'Consolas', monospace";
        ctx.fillStyle = "rgba(0, 229, 255, 0.4)";
        ctx.textAlign = "center";
        ctx.fillText("1000m x 1000m OPERATIONAL ARENA", tcx, tcy - 12);
        ctx.font = "bold 13px 'Consolas', monospace";
        ctx.fillStyle = "rgba(255, 214, 0, 0.5)";
        ctx.fillText("MAX ALTITUDE: 100m | MAX SPEED: 5 m/s | MIN SEPARATION: 20m | COMM RANGE: 100m", tcx, tcy + 16);
        ctx.restore();

        const terrainTex = new THREE.CanvasTexture(c);
        const groundMat = new THREE.MeshStandardMaterial({
            map: terrainTex,
            roughness: 0.8,
            metalness: 0.15,
        });

        // Ground mesh: centered at (450, 0, 0) covering [-150, 1050] x [-550, 550]
        const groundGeo = new THREE.PlaneGeometry(1200, 1100);
        const groundMesh = new THREE.Mesh(groundGeo, groundMat);
        groundMesh.position.set(450, 0, -0.05);
        groundMesh.receiveShadow = true;
        challengeGroup.add(groundMesh);

        // 2. 3D Glowing Perimeter Ribbon around the 1000m x 1000m Arena
        const fenceMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.85 });
        const yellowFenceMat = new THREE.MeshBasicMaterial({ color: 0xffd600, transparent: true, opacity: 0.85 });

        // 4 boundary wall segments
        const sWall = new THREE.Mesh(new THREE.BoxGeometry(1000, 1.5, 1.5), fenceMat);
        sWall.position.set(500, -500, 0.75);
        challengeGroup.add(sWall);

        const nWall = new THREE.Mesh(new THREE.BoxGeometry(1000, 1.5, 1.5), fenceMat);
        nWall.position.set(500, 500, 0.75);
        challengeGroup.add(nWall);

        const eWall = new THREE.Mesh(new THREE.BoxGeometry(1.5, 1000, 1.5), fenceMat);
        eWall.position.set(1000, 0, 0.75);
        challengeGroup.add(eWall);

        const wWall = new THREE.Mesh(new THREE.BoxGeometry(1.5, 1000, 1.5), yellowFenceMat);
        wWall.position.set(0, 0, 0.75);
        challengeGroup.add(wWall);

        // 3. Four 25m Tall Boundary Pylon Towers at Arena Corners
        const pylonGeo = new THREE.CylinderGeometry(0.8, 1.6, 25, 8);
        const pylonMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.8, roughness: 0.3 });
        const beaconGeo = new THREE.SphereGeometry(1.2, 12, 12);
        const beaconMat = new THREE.MeshBasicMaterial({ color: 0xff1744 });

        [[0, -500], [1000, -500], [1000, 500], [0, 500]].forEach(([cx, cy]) => {
            const pylon = new THREE.Mesh(pylonGeo, pylonMat);
            pylon.rotation.x = Math.PI / 2;
            pylon.position.set(cx, cy, 12.5);
            challengeGroup.add(pylon);

            const b = new THREE.Mesh(beaconGeo, beaconMat);
            b.position.set(cx, cy, 25.5);
            challengeGroup.add(b);
        });

        // 4. Operational Center Base Command Complex at X = -75.0
        const commandBuilding = new THREE.Mesh(
            new THREE.BoxGeometry(32, 45, 14),
            new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.6, roughness: 0.4 })
        );
        commandBuilding.position.set(-135, 0, 7);
        commandBuilding.castShadow = true;
        challengeGroup.add(commandBuilding);

        // Radome on Command Building roof
        const radome = new THREE.Mesh(
            new THREE.SphereGeometry(5, 16, 16),
            new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.2 })
        );
        radome.position.set(-135, 0, 18);
        challengeGroup.add(radome);

        // Communication Mast with Dish Antenna at GCS node coordinate (-75, 0)
        const mast = new THREE.Mesh(
            new THREE.CylinderGeometry(0.5, 0.9, 22, 8),
            new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.85 })
        );
        mast.rotation.x = Math.PI / 2;
        mast.position.set(-75, 0, 11);
        challengeGroup.add(mast);

        const dish = new THREE.Mesh(
            new THREE.CylinderGeometry(3.0, 0.4, 0.8, 16),
            new THREE.MeshStandardMaterial({ color: 0x00e5ff, metalness: 0.8 })
        );
        dish.rotation.z = Math.PI / 3;
        dish.position.set(-75, 0, 20);
        challengeGroup.add(dish);

        // 5. 16 Physical 3D Launch Pads at Operational Center (>20m separation guaranteed)
        const fleetPads = [
            // Surveyors (Column X = -65, Y spaced by 50m)
            { id: "UAV_1", label: "UAV-1", pos: [-65, -125, 0.2], col: 0x00e5ff },
            { id: "UAV_2", label: "UAV-2", pos: [-65, -75, 0.2], col: 0x00e5ff },
            { id: "UAV_3", label: "UAV-3", pos: [-65, -25, 0.2], col: 0x00e5ff },
            { id: "UAV_4", label: "UAV-4", pos: [-65, 25, 0.2], col: 0x00e5ff },
            { id: "UAV_5", label: "UAV-5", pos: [-65, 75, 0.2], col: 0x00e5ff },
            { id: "UAV_6", label: "UAV-6", pos: [-65, 125, 0.2], col: 0x00e5ff },
            // Relays (Column X = -95, Y spaced by 50m)
            { id: "RELAY_1", label: "REL-1", pos: [-95, -125, 0.2], col: 0xffd600 },
            { id: "RELAY_2", label: "REL-2", pos: [-95, -75, 0.2], col: 0xffd600 },
            { id: "RELAY_3", label: "REL-3", pos: [-95, -25, 0.2], col: 0xffd600 },
            { id: "RELAY_4", label: "REL-4", pos: [-95, 25, 0.2], col: 0xffd600 },
            { id: "RELAY_5", label: "REL-5", pos: [-95, 75, 0.2], col: 0xffd600 },
            { id: "RELAY_6", label: "REL-6", pos: [-95, 125, 0.2], col: 0xffd600 },
            // Scouts (Column X = -35, Y spaced by 50m)
            { id: "SCOUT_1", label: "SCT-1", pos: [-35, -75, 0.2], col: 0x00ff88 },
            { id: "SCOUT_2", label: "SCT-2", pos: [-35, -25, 0.2], col: 0x00ff88 },
            { id: "SCOUT_3", label: "SCT-3", pos: [-35, 25, 0.2], col: 0x00ff88 },
            { id: "SCOUT_4", label: "SCT-4", pos: [-35, 75, 0.2], col: 0x00ff88 },
        ];

        fleetPads.forEach(pad => {
            const padGroup = new THREE.Group();
            padGroup.position.set(pad.pos[0], pad.pos[1], pad.pos[2]);

            // Octagonal platform base
            const plat = new THREE.Mesh(
                new THREE.CylinderGeometry(6.5, 7.0, 0.4, 8),
                new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.6 })
            );
            plat.rotation.x = Math.PI / 2;
            padGroup.add(plat);

            // Glowing target ring
            const rMat = new THREE.MeshBasicMaterial({ color: pad.col, transparent: true, opacity: 0.85, side: THREE.DoubleSide });
            const rMesh = new THREE.Mesh(new THREE.RingGeometry(4.2, 5.0, 24), rMat);
            rMesh.position.z = 0.22;
            padGroup.add(rMesh);

            // Crosshair
            const crossMat = new THREE.MeshBasicMaterial({ color: pad.col, transparent: true, opacity: 0.7 });
            const ch = new THREE.Mesh(new THREE.PlaneGeometry(8.0, 0.6), crossMat);
            ch.position.z = 0.23;
            padGroup.add(ch);
            const cv = new THREE.Mesh(new THREE.PlaneGeometry(0.6, 8.0), crossMat);
            cv.position.z = 0.23;
            padGroup.add(cv);

            challengeGroup.add(padGroup);
        });

        // 6. 100m Altitude Ceiling Virtual Wireframe
        const ceilingLines = new THREE.LineSegments(
            new THREE.EdgesGeometry(new THREE.BoxGeometry(1000, 1000, 0.1)),
            new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.25 })
        );
        ceilingLines.position.set(500, 0, 100);
        challengeGroup.add(ceilingLines);
    }

    // ------------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------------
    return {
        // Initialize the complete Sector Delta diorama in the Three.js scene
        init(sceneTheater) {
            dioramaGroup = new THREE.Group();
            sceneTheater.add(dioramaGroup);

            // 1. Beveled Tabletop Tray (Plinth) 1000m x 1000m
            const plinth = createPlinthTray(1000, 10);
            dioramaGroup.add(plinth);

            // 1b. 1000m x 1000m x 130m Holographic Tactical Airspace Boundary Wireframe Box
            const boundaryGeo = new THREE.BoxGeometry(1000, 1000, 130);
            const boundaryEdges = new THREE.EdgesGeometry(boundaryGeo);
            const boundaryMat = new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.28 });
            const boundaryLine = new THREE.LineSegments(boundaryEdges, boundaryMat);
            boundaryLine.position.set(0, 0, 65);
            dioramaGroup.add(boundaryLine);

            // 2. Elevated Curved Highway Overpass & Miniature Cars
            createElevatedOverpass(dioramaGroup);

            // 2b. Northeast Landslide Mountain with Buried Tunnels & Rockslide
            createLandslideMountain(dioramaGroup);

            // 2c. River Gorge Channel & Fractured Bridge (POI_BRIDGE)
            createRiverAndBridges(dioramaGroup);

            // 3. Stylized Miniature Green Trees
            createLandscapingTrees(dioramaGroup);

            // 4. Architectural Skyscraper City Cluster & Low-Rise Village (Earthquake Destroyed)
            createArchitecturalCity(dioramaGroup);

            // Enclose the city with massive corner mountains
            createCornerMountains(dioramaGroup);

            // 5. Dedicated 3D Physical Launch Pads on GCS Apron
            create3DLaunchPads(dioramaGroup);

            // 5b. Relocated 3D GCS Flight Apron Platform (75m south of boundary at Y = -575)
            createHubApronPlatform(dioramaGroup);

            // 6. Initialize MeitY / IIT Bombay / IISER Bhopal 1000m Challenge Arena
            createChallengeArena(sceneTheater);

            // Optimize render performance: static diorama elements never move
            dioramaGroup.traverse(child => {
                if (child.isMesh) {
                    child.matrixAutoUpdate = false;
                    child.updateMatrix();
                }
            });
            dioramaGroup.matrixAutoUpdate = false;
            dioramaGroup.updateMatrix();

            console.log("[SectorDelta] Diorama initialized with 3D buildings, plinth, overpasses, and trees.");
        },

        // Maintain 3D City Diorama as the single unified visual environment
        setScenario(scenarioName) {
            if (dioramaGroup) dioramaGroup.visible = true;
            if (challengeGroup) challengeGroup.visible = false;
            console.log(`[SectorDelta] 3D City Diorama active`);
        },

        // Links are rendered as simple clean colored lines in cockpit.js
        updateArcs(linksData, nodeCoordsMap, sceneTheater) {
        },

        animate(dt = 0.016) {
        },

        getRooftopNodes() {
            return rooftopRelayNodes;
        }
    };
})();
window.SectorDelta = SectorDelta;
