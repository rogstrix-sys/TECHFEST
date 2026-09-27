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

        // Coordinate helper: World [-180, +180] -> Canvas [0, 2048]
        const toC = (x, y) => [
            ((x + 180) / 360) * 2048,
            ((180 - y) / 360) * 2048
        ];
        const toLen = (m) => (m / 360) * 2048;

        // 1. Rich Natural Parkland Green Base Gradient
        // Deep, saturated botanical greens that stay lush under bright directional sunlight
        const bgGrad = ctx.createRadialGradient(1024, 1024, 160, 1024, 1024, 1440);
        bgGrad.addColorStop(0.0, "#286620"); // Vibrant lively meadow green in central parkland
        bgGrad.addColorStop(0.30, "#22581a"); // Lush parkland green
        bgGrad.addColorStop(0.65, "#1a4614"); // Deep rich emerald lawn
        bgGrad.addColorStop(0.90, "#13380e"); // Perimeter park turf
        bgGrad.addColorStop(1.0, "#0e290a"); // Shaded perimeter forest turf
        ctx.fillStyle = bgGrad;
        ctx.fillRect(0, 0, 2048, 2048);

        // 2. Procedural Multi-Frequency Grass Turf Micro-Texture (Stippling)
        // High-end architectural tabletop model flock/felt turf simulation
        let seed = 42819;
        const rnd = () => {
            seed = (seed * 9301 + 49297) % 233280;
            return seed / 233280;
        };

        const grassColors = [
            "rgba(90, 175, 50, 0.20)",  // Sunlit rich blade tips
            "rgba(145, 190, 60, 0.14)", // Warm meadow golden-green flecks
            "rgba(32, 90, 20, 0.24)",   // Rich clover green
            "rgba(12, 35, 8, 0.28)",    // Deep thatch shadow
            "rgba(185, 210, 75, 0.10)"  // Warm clover blossom flecks
        ];
        for (let i = 0; i < 38000; i++) {
            const gx = rnd() * 2048;
            const gy = rnd() * 2048;
            const gSize = 1.2 + rnd() * 2.4;
            ctx.fillStyle = grassColors[Math.floor(rnd() * grassColors.length)];
            ctx.fillRect(gx, gy, gSize, gSize);
        }

        // 3. Subtle Field Zoning & Landscape Architecture Parcels
        // A. Northwest Arboretum & Botanical Park ([ -165, 30 ] to [ -35, 165 ])
        const [nwX, nwY] = toC(-165, 165);
        const nwW = toLen(130);
        const nwH = toLen(135);
        ctx.fillStyle = "rgba(45, 125, 35, 0.16)";
        ctx.fillRect(nwX, nwY, nwW, nwH);

        // Northwest Park Mowing Stripes (subtle lawn stripes at 45°)
        ctx.save();
        ctx.beginPath();
        ctx.rect(nwX, nwY, nwW, nwH);
        ctx.clip();
        for (let s = -nwH; s < nwW + nwH; s += 28) {
            ctx.fillStyle = "rgba(255, 255, 255, 0.04)";
            ctx.fillRect(nwX + s, nwY, 14, nwH * 2);
        }
        ctx.restore();
        ctx.strokeStyle = "rgba(20, 75, 18, 0.40)";
        ctx.lineWidth = 3;
        ctx.strokeRect(nwX, nwY, nwW, nwH);

        // Northwest Garden plot divider borders
        ctx.strokeStyle = "rgba(220, 255, 205, 0.18)";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(nwX + nwW * 0.45, nwY);
        ctx.lineTo(nwX + nwW * 0.45, nwY + nwH);
        ctx.moveTo(nwX, nwY + nwH * 0.55);
        ctx.lineTo(nwX + nwW, nwY + nwH * 0.55);
        ctx.stroke();

        // B. Northeast Civic Meadow & Forest Grounds ([ 35, 35 ] to [ 165, 165 ])
        const [neX, neY] = toC(35, 165);
        const neW = toLen(130);
        const neH = toLen(130);
        ctx.fillStyle = "rgba(28, 105, 24, 0.18)";
        ctx.fillRect(neX, neY, neW, neH);
        ctx.strokeStyle = "rgba(18, 70, 16, 0.40)";
        ctx.lineWidth = 3;
        ctx.strokeRect(neX, neY, neW, neH);

        // Concentric ornamental botanical rose-garden rings in Northeast Meadow
        ctx.strokeStyle = "rgba(225, 255, 210, 0.22)";
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        ctx.arc(neX + neW * 0.5, neY + neH * 0.48, toLen(24), 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(neX + neW * 0.5, neY + neH * 0.48, toLen(12), 0, Math.PI * 2);
        ctx.stroke();

        // C. Central Skyscraper Parkland & Promenade ([ -45, -25 ] to [ 50, 65 ])
        const [cParkX, cParkY] = toC(-45, 65);
        const cParkW = toLen(95);
        const cParkH = toLen(90);
        ctx.fillStyle = "rgba(50, 140, 45, 0.15)";
        ctx.fillRect(cParkX, cParkY, cParkW, cParkH);
        ctx.strokeStyle = "rgba(215, 245, 200, 0.28)";
        ctx.lineWidth = 2.5;
        ctx.strokeRect(cParkX, cParkY, cParkW, cParkH);

        // D. East Recreational Grounds & Sports Oval ([ 60, -85 ] to [ 165, 15 ])
        const [eX, eY] = toC(60, 15);
        const eW = toLen(105);
        const eH = toLen(100);
        ctx.fillStyle = "rgba(40, 120, 35, 0.15)";
        ctx.fillRect(eX, eY, eW, eH);
        ctx.strokeStyle = "rgba(215, 245, 195, 0.28)";
        ctx.lineWidth = 2.5;
        ctx.strokeRect(eX, eY, eW, eH);

        // Running track oval with terracotta clay turf
        ctx.beginPath();
        ctx.ellipse(eX + eW / 2, eY + eH / 2, eW * 0.38, eH * 0.32, 0, 0, Math.PI * 2);
        ctx.strokeStyle = "#a84832"; // Clay running track
        ctx.lineWidth = toLen(7.0);
        ctx.stroke();
        ctx.strokeStyle = "rgba(255, 255, 255, 0.65)";
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // E. Pedestrian Sandstone Promenade Walkways through the Greenery
        ctx.save();
        ctx.strokeStyle = "#cfc2a0";
        ctx.lineWidth = toLen(3.5);
        ctx.lineCap = "round";
        ctx.lineJoin = "round";

        // Trail 1: Northwest Arboretum to Skyscraper Park
        ctx.beginPath();
        const [p1x, p1y] = toC(-140, 120);
        const [p2x, p2y] = toC(-70, 70);
        const [p3x, p3y] = toC(-10, 30);
        ctx.moveTo(p1x, p1y);
        ctx.quadraticCurveTo(p2x, p2y, p3x, p3y);
        ctx.stroke();

        // Trail 2: Skyscraper Park to Terracotta Village Plaza
        ctx.beginPath();
        const [p4x, p4y] = toC(-30, 20);
        const [p5x, p5y] = toC(-70, -20);
        const [p6x, p6y] = toC(-80, -35);
        ctx.moveTo(p4x, p4y);
        ctx.quadraticCurveTo(p5x, p5y, p6x, p6y);
        ctx.stroke();

        // Trail 3: Skyscraper Park to East Recreational Oval
        ctx.beginPath();
        const [p7x, p7y] = toC(30, 20);
        const [p8x, p8y] = toC(80, 0);
        const [p9x, p9y] = toC(115, -40);
        ctx.moveTo(p7x, p7y);
        ctx.quadraticCurveTo(p8x, p8y, p9x, p9y);
        ctx.stroke();

        // Trail 4: Northeast Gardens to Skyscraper Park
        ctx.beginPath();
        const [p10x, p10y] = toC(100, 110);
        const [p11x, p11y] = toC(50, 70);
        const [p12x, p12y] = toC(10, 40);
        ctx.moveTo(p10x, p10y);
        ctx.quadraticCurveTo(p11x, p11y, p12x, p12y);
        ctx.stroke();
        ctx.restore();

        // 4. Subtle Topographic Elevation Contour Lines
        ctx.save();
        const contours = [
            { el: "+10.0m", pts: [[-180, 130], [-80, 150], [40, 110], [140, 140], [180, 160]] },
            { el: "+14.0m", pts: [[-180, 70], [-70, 95], [30, 60], [120, 90], [180, 110]] },
            { el: "+18.0m", pts: [[-180, 10], [-60, 35], [25, 0], [110, 30], [180, 50]] },
            { el: "+22.0m", pts: [[-180, -50], [-70, -25], [20, -55], [105, -30], [180, -10]] },
            { el: "+26.0m", pts: [[-180, -110], [-65, -85], [15, -115], [100, -90], [180, -70]] },
        ];

        contours.forEach(({ el, pts }) => {
            ctx.beginPath();
            pts.forEach((pt, idx) => {
                const [cx, cy] = toC(pt[0], pt[1]);
                if (idx === 0) ctx.moveTo(cx, cy);
                else {
                    const [prevX, prevY] = toC(pts[idx - 1][0], pts[idx - 1][1]);
                    const midX = (prevX + cx) / 2;
                    const midY = (prevY + cy) / 2;
                    ctx.quadraticCurveTo(prevX, prevY, midX, midY);
                }
            });
            const [lastX, lastY] = toC(pts[pts.length - 1][0], pts[pts.length - 1][1]);
            ctx.lineTo(lastX, lastY);
            ctx.strokeStyle = "rgba(235, 255, 220, 0.24)";
            ctx.lineWidth = 1.8;
            ctx.stroke();

            // Elevation label badge
            const labelPt = pts[2];
            const [lx, ly] = toC(labelPt[0], labelPt[1]);
            ctx.font = "bold 12px 'Consolas', monospace";
            ctx.fillStyle = "rgba(10, 35, 12, 0.85)";
            ctx.fillRect(lx - 3, ly - 9, 58, 16);
            ctx.fillStyle = "rgba(240, 255, 220, 0.95)";
            ctx.fillText(el, lx + 2, ly + 4);
        });
        ctx.restore();

        // 5. Subtle Architectural Coordinate Grid
        // Fine grid (every 32px / 5.6m)
        ctx.strokeStyle = "rgba(215, 255, 200, 0.055)";
        ctx.lineWidth = 1;
        for (let x = 0; x <= 2048; x += 32) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, 2048);
            ctx.stroke();
        }
        for (let y = 0; y <= 2048; y += 32) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(2048, y);
            ctx.stroke();
        }

        // Major grid lines (every 160px / 28m)
        ctx.strokeStyle = "rgba(235, 255, 215, 0.15)";
        ctx.lineWidth = 1.6;
        for (let x = 0; x <= 2048; x += 160) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, 2048);
            ctx.stroke();
        }
        for (let y = 0; y <= 2048; y += 160) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(2048, y);
            ctx.stroke();
        }

        // Major grid intersection crosshairs
        ctx.strokeStyle = "rgba(245, 255, 230, 0.35)";
        ctx.lineWidth = 1.6;
        for (let x = 160; x < 2048; x += 160) {
            for (let y = 160; y < 2048; y += 160) {
                ctx.beginPath();
                ctx.moveTo(x - 6, y);
                ctx.lineTo(x + 6, y);
                ctx.moveTo(x, y - 6);
                ctx.lineTo(x, y + 6);
                ctx.stroke();
            }
        }

        // 6. Terracotta Courtyard Plazas (Under the low-rise village)
        // High-contrast warm terracotta paving surrounded by vibrant green parkland
        const [pz1X, pz1Y] = toC(-105, -35);
        ctx.fillStyle = "#b84232";
        ctx.fillRect(pz1X, pz1Y, toLen(70), toLen(75));
        ctx.strokeStyle = "#e8d8c2";
        ctx.lineWidth = 3.5;
        ctx.strokeRect(pz1X, pz1Y, toLen(70), toLen(75));

        // Paving stone joints inside Plaza 1
        ctx.strokeStyle = "rgba(255, 210, 195, 0.22)";
        ctx.lineWidth = 1.2;
        for (let px = pz1X + 16; px < pz1X + toLen(70); px += 18) {
            ctx.beginPath();
            ctx.moveTo(px, pz1Y);
            ctx.lineTo(px, pz1Y + toLen(75));
            ctx.stroke();
        }

        const [pz2X, pz2Y] = toC(20, -55);
        ctx.fillStyle = "#b84232";
        ctx.fillRect(pz2X, pz2Y, toLen(55), toLen(50));
        ctx.strokeStyle = "#e8d8c2";
        ctx.lineWidth = 3.5;
        ctx.strokeRect(pz2X, pz2Y, toLen(55), toLen(50));

        for (let px = pz2X + 16; px < pz2X + toLen(55); px += 18) {
            ctx.beginPath();
            ctx.moveTo(px, pz2Y);
            ctx.lineTo(px, pz2Y + toLen(50));
            ctx.stroke();
        }

        // 7. Smooth Catmull-Rom Asphalt Boulevard with Stone Curbs & Yellow Dashes
        ctx.lineCap = "round";
        ctx.lineJoin = "round";

        // Stone curb shoulder bed (light stone verge separating road from lawn)
        ctx.beginPath();
        smoothRoadPoints.forEach((pt, idx) => {
            const [cx, cy] = toC(pt[0], pt[1]);
            if (idx === 0) ctx.moveTo(cx, cy);
            else ctx.lineTo(cx, cy);
        });
        ctx.closePath();
        ctx.strokeStyle = "#788880";
        ctx.lineWidth = toLen(18.0);
        ctx.stroke();

        // Dark charcoal asphalt roadway
        ctx.beginPath();
        smoothRoadPoints.forEach((pt, idx) => {
            const [cx, cy] = toC(pt[0], pt[1]);
            if (idx === 0) ctx.moveTo(cx, cy);
            else ctx.lineTo(cx, cy);
        });
        ctx.closePath();
        ctx.strokeStyle = "#1e252d";
        ctx.lineWidth = toLen(14.5);
        ctx.stroke();

        // Crisp white road shoulder edges
        ctx.strokeStyle = "rgba(255, 255, 255, 0.88)";
        ctx.lineWidth = 2.2;
        ctx.stroke();

        // Yellow dashed center line
        ctx.strokeStyle = "#ffd600";
        ctx.lineWidth = 2.2;
        ctx.setLineDash([16, 12]);
        ctx.stroke();
        ctx.setLineDash([]);

        // Pedestrian crosswalks near plazas
        const [zw1X, zw1Y] = toC(-88, -114);
        ctx.fillStyle = "rgba(255, 255, 255, 0.88)";
        for (let k = -8; k <= 8; k += 4) {
            ctx.fillRect(zw1X + k, zw1Y - 14, 2.5, 28);
        }

        // 8. Tactical GCS Flight Apron & 16-UAV Launch Markings
        // Positioned cleanly from x = -88 to +88, y = -130 to -165 (depth 35m)
        const [apronX, apronY] = toC(-88, -130);
        const apW = toLen(176);
        const apH = toLen(35);

        // Concrete tarmac apron slab
        ctx.fillStyle = "#242e37";
        ctx.fillRect(apronX, apronY, apW, apH);

        // Reinforced concrete slab expansion joint grid
        ctx.strokeStyle = "rgba(255, 255, 255, 0.10)";
        ctx.lineWidth = 1.2;
        for (let sx = apronX; sx <= apronX + apW; sx += 28) {
            ctx.beginPath();
            ctx.moveTo(sx, apronY);
            ctx.lineTo(sx, apronY + apH);
            ctx.stroke();
        }
        for (let sy = apronY; sy <= apronY + apH; sy += 28) {
            ctx.beginPath();
            ctx.moveTo(apronX, sy);
            ctx.lineTo(apronX + apW, sy);
            ctx.stroke();
        }

        // Safety yellow border outline
        ctx.strokeStyle = "#ffd600";
        ctx.lineWidth = 3.0;
        ctx.strokeRect(apronX, apronY, apW, apH);

        // Apron threshold white piano-key stripes
        ctx.fillStyle = "#ffffff";
        for (let b = 6; b < apH - 6; b += 10) {
            ctx.fillRect(apronX + 4, apronY + b, 16, 5);
            ctx.fillRect(apronX + apW - 20, apronY + b, 16, 5);
        }

        // Apron centerline dashed yellow taxiway
        ctx.strokeStyle = "#ffd600";
        ctx.lineWidth = 2;
        ctx.setLineDash([14, 10]);
        ctx.beginPath();
        ctx.moveTo(apronX + 28, apronY + apH / 2);
        ctx.lineTo(apronX + apW - 28, apronY + apH / 2);
        ctx.stroke();
        ctx.setLineDash([]);

        // Central GCS Command Bunker Pad (x = 0, y = -145)
        const [gcsCenterX, gcsCenterY] = toC(0, -145);
        ctx.fillStyle = "#18202c";
        ctx.beginPath();
        ctx.arc(gcsCenterX, gcsCenterY, toLen(10.5), 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "#ffd600";
        ctx.lineWidth = 2.5;
        ctx.stroke();

        // 2 Primary Helipads: Pad Alpha (-65, -136) and Pad Bravo (+65, -136)
        // Positioned cleanly on the Relay line with zero overlap on any UAV spawn bay
        [ { x: -65, y: -136, lbl: "PAD-A [SURVEY]" }, { x: 65, y: -136, lbl: "PAD-B [SURVEY]" } ].forEach(hp => {
            const [hx, hy] = toC(hp.x, hp.y);
            const hRadius = toLen(6.5);

            ctx.strokeStyle = "#ffd600";
            ctx.lineWidth = 2.8;
            ctx.beginPath();
            ctx.arc(hx, hy, hRadius, 0, Math.PI * 2);
            ctx.stroke();

            ctx.strokeStyle = "rgba(255, 214, 0, 0.4)";
            ctx.lineWidth = 1.4;
            ctx.setLineDash([5, 5]);
            ctx.beginPath();
            ctx.arc(hx, hy, hRadius * 0.7, 0, Math.PI * 2);
            ctx.stroke();
            ctx.setLineDash([]);

            ctx.font = "bold 20px 'Consolas', monospace";
            ctx.fillStyle = "#ffd600";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.fillText("H", hx, hy);

            ctx.font = "bold 9px 'Consolas', monospace";
            ctx.fillStyle = "rgba(255, 255, 255, 0.85)";
            ctx.fillText(hp.lbl, hx, hy + hRadius + 8);
            ctx.textAlign = "left";
            ctx.textBaseline = "alphabetic";
        });

        // 16 UAV Spawn Marking Bays matching vis/server.py:
        // Surveyors row (y = -145, x in [-70, -50, -30, -10, 10, 30, 50, 70])
        const surveyorXs = [-70, -50, -30, -10, 10, 30, 50, 70];
        surveyorXs.forEach((sx, idx) => {
            const [bx, by] = toC(sx, -145);
            ctx.strokeStyle = "rgba(0, 229, 255, 0.75)";
            ctx.lineWidth = 1.4;
            const sz = toLen(3.8);
            ctx.strokeRect(bx - sz, by - sz, sz * 2, sz * 2);

            ctx.font = "bold 9px 'Consolas', monospace";
            ctx.fillStyle = "rgba(0, 229, 255, 0.85)";
            ctx.fillText(`U${idx + 1}`, bx - 6, by - sz - 2);
        });

        // Relays row (y = -136, x in [-45, -15, 15, 45])
        const relayXs = [-45, -15, 15, 45];
        relayXs.forEach((rx, idx) => {
            const [bx, by] = toC(rx, -136);
            ctx.strokeStyle = "rgba(255, 214, 0, 0.75)";
            ctx.lineWidth = 1.4;
            const sz = toLen(3.4);
            ctx.strokeRect(bx - sz, by - sz, sz * 2, sz * 2);

            ctx.font = "bold 9px 'Consolas', monospace";
            ctx.fillStyle = "rgba(255, 214, 0, 0.85)";
            ctx.fillText(`R${idx + 1}`, bx - 5, by - sz - 2);
        });

        // Scouts row (y = -153, x in [-45, -15, 15, 45])
        const scoutXs = [-45, -15, 15, 45];
        scoutXs.forEach((scx, idx) => {
            const [bx, by] = toC(scx, -153);
            ctx.strokeStyle = "rgba(100, 255, 120, 0.75)";
            ctx.lineWidth = 1.4;
            const sz = toLen(3.4);
            ctx.strokeRect(bx - sz, by - sz, sz * 2, sz * 2);

            ctx.font = "bold 9px 'Consolas', monospace";
            ctx.fillStyle = "rgba(100, 255, 120, 0.85)";
            ctx.fillText(`S${idx + 1}`, bx - 5, by + sz + 9);
        });

        // Apron Header Title
        ctx.font = "bold 13px 'Consolas', monospace";
        ctx.fillStyle = "#ffd600";
        ctx.fillText("GCS FLIGHT APRON // 16-UAV FANET COMMAND POST", apronX + 28, apronY - 8);

        // 9. SECTOR DELTA -27 Main Typography & Title Plate
        // Positioned in the open South-East parkland lawn corridor (x = 94 to 166, y = -132 to -162)
        // Completely clear of overpasses, buildings, roads, and GCS apron. 100% visible in top and orbit views!
        const [txtX, txtY] = toC(94, -134);

        ctx.save();
        ctx.shadowColor = "rgba(10, 35, 12, 0.95)";
        ctx.shadowBlur = 12;

        // Tactical classification badge
        ctx.font = "bold 11px 'Consolas', monospace";
        ctx.fillStyle = "#ffd600";
        ctx.fillText("TACTICAL THEATER // AUTONOMOUS SWARM GRID", txtX, txtY);

        // Main Sector Name
        ctx.font = "bold 44px -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif";
        ctx.fillStyle = "#ffffff";
        ctx.fillText("SECTOR DELTA", txtX, txtY + 38);

        // Subtitle
        ctx.font = "bold 20px -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif";
        ctx.fillStyle = "#c8f0d0";
        ctx.fillText("(SE - RELAY CORRIDOR)", txtX, txtY + 64);

        // Prominent numeric "-27"
        ctx.font = "900 76px -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif";
        ctx.fillStyle = "#ffffff";
        ctx.fillText("-27", txtX + 260, txtY + 54);

        // Graphic scale bar: 0m to 50m
        ctx.strokeStyle = "rgba(240, 255, 225, 0.85)";
        ctx.lineWidth = 2.5;
        const scaleW = toLen(50);
        ctx.beginPath();
        ctx.moveTo(txtX, txtY + 84);
        ctx.lineTo(txtX + scaleW, txtY + 84);
        ctx.moveTo(txtX, txtY + 78);
        ctx.lineTo(txtX, txtY + 88);
        ctx.moveTo(txtX + scaleW / 2, txtY + 80);
        ctx.lineTo(txtX + scaleW / 2, txtY + 86);
        ctx.moveTo(txtX + scaleW, txtY + 78);
        ctx.lineTo(txtX + scaleW, txtY + 88);
        ctx.stroke();

        ctx.font = "bold 10px 'Consolas', monospace";
        ctx.fillStyle = "rgba(235, 255, 215, 0.90)";
        ctx.fillText("0m", txtX - 4, txtY + 98);
        ctx.fillText("25m", txtX + scaleW / 2 - 8, txtY + 98);
        ctx.fillText("50m SCALE", txtX + scaleW - 14, txtY + 98);

        // North compass indicator badge
        ctx.fillStyle = "rgba(10, 35, 12, 0.85)";
        ctx.fillRect(txtX + 348, txtY + 68, 38, 34);
        ctx.strokeStyle = "#ffd600";
        ctx.lineWidth = 1.5;
        ctx.strokeRect(txtX + 348, txtY + 68, 38, 34);

        ctx.font = "bold 14px 'Consolas', monospace";
        ctx.fillStyle = "#ffd600";
        ctx.fillText("▲ N", txtX + 354, txtY + 90);

        ctx.restore();

        // 10. Northwest Arboretum Metadata Typography
        const [nwBadgeX, nwBadgeY] = toC(-162, 162);
        ctx.font = "bold 11px 'Consolas', monospace";
        ctx.fillStyle = "rgba(240, 255, 230, 0.85)";
        ctx.fillText("SECTOR DELTA // NORTH ARBORETUM & BOTANICAL COMMONS", nwBadgeX, nwBadgeY);
        ctx.fillStyle = "rgba(200, 240, 190, 0.70)";
        ctx.fillText("GRID REF: 48Q-DE 28914 // DATUM: WGS84 // ELEV: +12.0m", nwBadgeX, nwBadgeY + 16);

        // 11. Outer Plinth Border Frame & Corner Registration Crosshairs
        ctx.strokeStyle = "rgba(235, 255, 220, 0.40)";
        ctx.lineWidth = 3;
        ctx.strokeRect(24, 24, 2000, 2000);

        // Corner crosshairs
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
        tex.anisotropy = 8;
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
        // Curve 1: Elevated overpass curving around the skyscraper district
        const curvePoints = [
            new THREE.Vector3(-140, -80, 2.5),
            new THREE.Vector3(-100, -100, 7.5),
            new THREE.Vector3(-30, -105, 12.0),
            new THREE.Vector3(40, -95, 13.5),
            new THREE.Vector3(95, -60, 11.0),
            new THREE.Vector3(120, -10, 6.0),
            new THREE.Vector3(125, 40, 2.0),
        ];
        const curve = new THREE.CatmullRomCurve3(curvePoints);
        const samples = 48;
        const pts = curve.getPoints(samples);

        // Road deck geometry
        const roadWidth = 14;
        const deckThickness = 1.2;
        const deckMat = new THREE.MeshStandardMaterial({
            color: 0x222a36,
            roughness: 0.7,
            metalness: 0.3,
        });
        const barrierMat = new THREE.MeshStandardMaterial({
            color: 0x90a4ae,
            roughness: 0.5,
            metalness: 0.5,
        });
        const pierMat = new THREE.MeshStandardMaterial({
            color: 0x78909c,
            roughness: 0.6,
        });

        for (let i = 0; i < pts.length - 1; i++) {
            const p1 = pts[i];
            const p2 = pts[i + 1];
            const segLen = p1.distanceTo(p2);
            const mid = p1.clone().add(p2).multiplyScalar(0.5);

            const segGeo = new THREE.BoxGeometry(roadWidth, segLen, deckThickness);
            const seg = new THREE.Mesh(segGeo, deckMat);
            seg.position.copy(mid);

            // Orient along segment
            const dir = p2.clone().sub(p1).normalize();
            const up = new THREE.Vector3(0, 0, 1);
            const right = new THREE.Vector3().crossVectors(dir, up).normalize();
            const rotMat = new THREE.Matrix4().makeBasis(right, dir, up);
            seg.rotation.setFromRotationMatrix(rotMat);
            seg.castShadow = true;
            seg.receiveShadow = true;
            group.add(seg);

            // Concrete crash barriers on left and right
            [-roadWidth / 2 + 0.4, roadWidth / 2 - 0.4].forEach(offset => {
                const barGeo = new THREE.BoxGeometry(0.8, segLen, 1.2);
                const bar = new THREE.Mesh(barGeo, barrierMat);
                bar.position.copy(mid).add(right.clone().multiplyScalar(offset));
                bar.position.z += 0.8;
                bar.rotation.setFromRotationMatrix(rotMat);
                group.add(bar);
            });

            // Concrete support piers every 4 segments
            if (i % 5 === 0 && mid.z > 3.0) {
                const pierGeo = new THREE.CylinderGeometry(1.6, 2.0, mid.z, 12);
                const pier = new THREE.Mesh(pierGeo, pierMat);
                pier.rotation.x = Math.PI / 2;
                pier.position.set(mid.x, mid.y, mid.z / 2);
                pier.castShadow = true;
                group.add(pier);
            }
        }

        // Miniature Cars on the Overpass & Ground
        const carColors = [0xffffff, 0xd32f2f, 0xfbc02d, 0x1976d2, 0x37474f, 0x388e3c];
        for (let i = 2; i < pts.length - 2; i += 3) {
            const pt = pts[i];
            const dir = pts[i + 1].clone().sub(pt).normalize();
            const carGroup = new THREE.Group();

            const cBodyGeo = new THREE.BoxGeometry(2.0, 3.8, 1.3);
            const cColor = carColors[i % carColors.length];
            const cBodyMat = new THREE.MeshStandardMaterial({ color: cColor, metalness: 0.6, roughness: 0.3 });
            const cBody = new THREE.Mesh(cBodyGeo, cBodyMat);
            carGroup.add(cBody);

            // Cabin / windshield
            const cRoofGeo = new THREE.BoxGeometry(1.7, 2.0, 0.8);
            const cRoofMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.2 });
            const cRoof = new THREE.Mesh(cRoofGeo, cRoofMat);
            cRoof.position.set(0, -0.2, 0.9);
            carGroup.add(cRoof);

            const sideOffset = (i % 2 === 0 ? 1 : -1) * 3.5;
            const up = new THREE.Vector3(0, 0, 1);
            const right = new THREE.Vector3().crossVectors(dir, up).normalize();
            carGroup.position.copy(pt).add(right.multiplyScalar(sideOffset));
            carGroup.position.z += 1.4;

            const rotMat = new THREE.Matrix4().makeBasis(right, dir, up);
            carGroup.rotation.setFromRotationMatrix(rotMat);
            group.add(carGroup);
        }
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

        // Trunk
        const trunkH = (type === 1 ? 3.8 : 2.6) * scale;
        const trunkGeo = new THREE.CylinderGeometry(0.28 * scale, 0.42 * scale, trunkH, 8);
        const trunkMat = new THREE.MeshStandardMaterial({ color: 0x423126, roughness: 0.92 });
        const trunk = new THREE.Mesh(trunkGeo, trunkMat);
        trunk.rotation.x = Math.PI / 2;
        trunk.position.z = (trunkH / 2) * scale;
        trunk.castShadow = true;
        treeGroup.add(trunk);

        const greens = [0x2e7d32, 0x388e3c, 0x43a047, 0x1b5e20, 0x558b2f, 0x2e6b27];

        if (type === 1) {
            // Conical Cypress / Pine Tree: 3 stacked tapered cones
            const pineGreens = [0x1b501d, 0x226225, 0x174419];
            for (let k = 0; k < 3; k++) {
                const cRad = (2.4 - k * 0.6) * scale;
                const cH = (3.2 - k * 0.4) * scale;
                const cGeo = new THREE.ConeGeometry(cRad, cH, 7);
                const cMat = new THREE.MeshStandardMaterial({
                    color: pineGreens[k % pineGreens.length],
                    roughness: 0.75,
                    metalness: 0.04,
                    flatShading: true,
                });
                const cone = new THREE.Mesh(cGeo, cMat);
                cone.rotation.x = Math.PI / 2;
                cone.position.z = (2.2 + k * 1.8) * scale;
                cone.castShadow = true;
                cone.receiveShadow = true;
                treeGroup.add(cone);
            }
        } else {
            // Deciduous / Meadow Canopy Tree: 2-3 faceted dodecahedron clusters
            const numClusters = 2 + Math.floor(Math.random() * 2);
            for (let c = 0; c < numClusters; c++) {
                const rad = (1.6 + Math.random() * 0.8) * scale;
                const folGeo = new THREE.DodecahedronGeometry(rad, 1);
                const folMat = new THREE.MeshStandardMaterial({
                    color: greens[(c + Math.floor(Math.abs(x) + Math.abs(y))) % greens.length],
                    roughness: 0.72,
                    metalness: 0.04,
                    flatShading: true,
                });
                const foliage = new THREE.Mesh(folGeo, folMat);
                foliage.position.set(
                    (Math.random() - 0.5) * 0.7 * scale,
                    (Math.random() - 0.5) * 0.7 * scale,
                    (2.8 + c * 1.1 + Math.random() * 0.4) * scale
                );
                foliage.castShadow = true;
                foliage.receiveShadow = true;
                treeGroup.add(foliage);
            }
        }

        return treeGroup;
    }

    function isTreePositionBlocked(x, y) {
        // 1. Outside plinth limits
        if (Math.abs(x) > 170 || Math.abs(y) > 170) return true;
        // 2. GCS flight apron clearance
        if (y < -124 && Math.abs(x) < 92) return true;
        // 3. SE title plate corridor clearance
        if (y < -124 && x > 86) return true;
        // 4. Smooth road boulevard clearance
        for (let i = 0; i < smoothRoadPoints.length; i += 2) {
            const dx = x - smoothRoadPoints[i][0];
            const dy = y - smoothRoadPoints[i][1];
            if (dx * dx + dy * dy < 11.5 * 11.5) return true;
        }
        // 5. Overpass curve clearance
        const overpassSamples = [
            [-140, -80], [-100, -100], [-30, -105], [40, -95], [95, -60], [120, -10], [125, 40]
        ];
        for (let p of overpassSamples) {
            const dx = x - p[0];
            const dy = y - p[1];
            if (dx * dx + dy * dy < 13.5 * 13.5) return true;
        }
        // 6. Architectural building footprints
        const buildings = [
            { minX: -84, maxX: -36, minY: -4, maxY: 44 },   // Tower 1
            { minX: -132, maxX: -88, minY: -40, maxY: 4 },  // Tower 2
            { minX: -16, maxX: 36, minY: 28, maxY: 76 },    // Tower 3
            { minX: -42, maxX: -8, minY: 72, maxY: 104 },   // Tower 4
            { minX: 46, maxX: 94, minY: -36, maxY: 12 },    // Tower 5
            { minX: 98, maxX: 138, minY: -62, maxY: -22 },  // Tower 6
            { minX: 116, maxX: 154, minY: 4, maxY: 38 },    // Tower 7
            { minX: -6, maxX: 30, minY: -60, maxY: -24 },   // Midrise 1
            { minX: -32, maxX: -4, minY: -70, maxY: -40 },  // Midrise 2
            { minX: -125, maxX: -40, minY: -100, maxY: -45 }, // Village West
            { minX: 20, maxX: 98, minY: -95, maxY: -65 },   // Village East
        ];
        for (let b of buildings) {
            if (x >= b.minX && x <= b.maxX && y >= b.minY && y <= b.maxY) return true;
        }
        return false;
    }

    function createLandscapingTrees(group) {
        // Place tree clusters along open lawns, parklands, plazas, and landscaped green belts
        const clusters = [
            // Plaza 1 (West low-rise village gardens)
            { cx: -95, cy: -25, count: 18, radius: 24, type: 0 },
            // Plaza 2 (South-west corridor)
            { cx: -60, cy: -95, count: 16, radius: 22, type: 0 },
            // Skyscraper base park (Center-North promenade)
            { cx: -15, cy: 15, count: 22, radius: 26, type: 0 },
            // East courtyard (Near sandstone tower)
            { cx: 75, cy: -65, count: 18, radius: 22, type: 1 },
            // Overpass embankment greenery
            { cx: 35, cy: -110, count: 14, radius: 18, type: 0 },
            // Far East street fringe
            { cx: 125, cy: -75, count: 14, radius: 18, type: 1 },
            // North-west boulevard median & verge
            { cx: -130, cy: 30, count: 15, radius: 20, type: 0 },
            // Northwest Arboretum parkland grove
            { cx: -105, cy: 115, count: 26, radius: 30, type: 1 },
            // Northeast Botanical Meadow gardens
            { cx: 95, cy: 110, count: 26, radius: 30, type: 0 },
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
                    const treeType = (type + (placed % 2 === 0 ? 0 : 1)) % 2;
                    group.add(createTree(tx, ty, scale, treeType));
                    placed++;
                }
            }
        });
    }

    // ------------------------------------------------------------------------
    // 5. Architectural Skyscraper City Cluster & Low-Rise Village
    // ------------------------------------------------------------------------
    function createArchitecturalCity(group) {
        const glassTex = createGlassFacadeTexture();
        const beigeTex = createBeigeFacadeTexture();
        const slateTex = createModernSlateTexture();
        const redRoofTex = createTerracottaRoofTexture();

        const glassMat = new THREE.MeshStandardMaterial({
            map: glassTex,
            color: 0x489ec7,
            roughness: 0.22,
            metalness: 0.35,
            emissive: 0x163e54,
            emissiveIntensity: 0.4,
        });

        const beigeMat = new THREE.MeshStandardMaterial({
            map: beigeTex,
            roughness: 0.65,
            metalness: 0.12,
        });

        const slateMat = new THREE.MeshStandardMaterial({
            map: slateTex,
            metalness: 0.5,
            roughness: 0.45,
        });

        const creamWallMat = new THREE.MeshStandardMaterial({
            color: 0xf0e6d6,
            roughness: 0.75,
        });

        const redRoofMat = new THREE.MeshStandardMaterial({
            map: redRoofTex,
            roughness: 0.6,
        });

        // --------------------------------------------------------------------
        // TOWER 1: Iconic Faceted Glass Skyscraper with Angled Diagonal Crown
        // (Matching prominent angled skyscraper in reference photo)
        // --------------------------------------------------------------------
        const t1Group = new THREE.Group();
        t1Group.position.set(-60, 20, 0);

        const t1W = 38;
        const t1D = 40;
        const t1H = 80;

        // Base tower body
        const t1Geo = new THREE.BoxGeometry(t1W, t1D, t1H);
        const t1Mesh = new THREE.Mesh(t1Geo, glassMat);
        t1Mesh.position.z = t1H / 2;
        t1Mesh.castShadow = true;
        t1Mesh.receiveShadow = true;
        t1Group.add(t1Mesh);

        // Angled/chamfered sloped roof crown (35 degree wedge cut)
        const crownShape = new THREE.Shape();
        crownShape.moveTo(-t1W / 2, 0);
        crownShape.lineTo(t1W / 2, 0);
        crownShape.lineTo(t1W / 2, 22);
        crownShape.closePath();

        const crownExtrudeSettings = { depth: t1D, bevelEnabled: false };
        const crownGeo = new THREE.ExtrudeGeometry(crownShape, crownExtrudeSettings);
        const crownMesh = new THREE.Mesh(crownGeo, glassMat);
        crownMesh.rotation.x = Math.PI / 2;
        crownMesh.position.set(0, t1D / 2, t1H);
        crownMesh.castShadow = true;
        t1Group.add(crownMesh);

        // Communication antenna mast on top
        const antGeo = new THREE.CylinderGeometry(0.3, 0.6, 16, 8);
        const antMat = new THREE.MeshStandardMaterial({ color: 0xcfd8dc, metalness: 0.9 });
        const ant = new THREE.Mesh(antGeo, antMat);
        ant.rotation.x = Math.PI / 2;
        ant.position.set(t1W / 2 - 4, 0, t1H + 28);
        t1Group.add(ant);

        // Rooftop glowing beacon tip
        const beaconGeo = new THREE.SphereGeometry(1.0, 12, 12);
        const beaconMat = new THREE.MeshBasicMaterial({ color: 0xff1744 });
        const beacon = new THREE.Mesh(beaconGeo, beaconMat);
        beacon.position.set(t1W / 2 - 4, 0, t1H + 36);
        t1Group.add(beacon);

        group.add(t1Group);
        rooftopRelayNodes.push(new THREE.Vector3(-60, 20, t1H + 2));

        // --------------------------------------------------------------------
        // TOWER 2: Hexagonal / Cylindrical Glass High-Rise
        // (Far-left tower in reference photo)
        // --------------------------------------------------------------------
        const t2Group = new THREE.Group();
        t2Group.position.set(-110, -18, 0);
        const t2Radius = 18;
        const t2H = 92;

        const t2Geo = new THREE.CylinderGeometry(t2Radius, t2Radius, t2H, 16);
        const t2Mesh = new THREE.Mesh(t2Geo, glassMat);
        t2Mesh.rotation.x = Math.PI / 2;
        t2Mesh.position.z = t2H / 2;
        t2Mesh.castShadow = true;
        t2Mesh.receiveShadow = true;
        t2Group.add(t2Mesh);

        // Vertical architectural fin ribs
        const ribMat = new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.8, roughness: 0.2 });
        for (let i = 0; i < 16; i++) {
            const theta = (i / 16) * Math.PI * 2;
            const rx = Math.cos(theta) * (t2Radius + 0.3);
            const ry = Math.sin(theta) * (t2Radius + 0.3);
            const ribGeo = new THREE.BoxGeometry(0.5, 0.5, t2H);
            const rib = new THREE.Mesh(ribGeo, ribMat);
            rib.position.set(rx, ry, t2H / 2);
            t2Group.add(rib);
        }

        // Recessed top observation ring
        const ringGeo = new THREE.CylinderGeometry(t2Radius * 0.8, t2Radius * 0.8, 6, 16);
        const ring = new THREE.Mesh(ringGeo, slateMat);
        ring.rotation.x = Math.PI / 2;
        ring.position.z = t2H + 3;
        t2Group.add(ring);

        group.add(t2Group);
        rooftopRelayNodes.push(new THREE.Vector3(-110, -18, t2H + 5));

        // --------------------------------------------------------------------
        // TOWER 3: Stepped Corporate High-Rise
        // (Center tower in reference photo)
        // --------------------------------------------------------------------
        const t3Group = new THREE.Group();
        t3Group.position.set(10, 52, 0);

        // Tier 1 Base
        const t3_1 = new THREE.Mesh(new THREE.BoxGeometry(44, 40, 48), slateMat);
        t3_1.position.z = 24;
        t3_1.castShadow = true;
        t3_1.receiveShadow = true;
        t3Group.add(t3_1);

        // Tier 2 Setback
        const t3_2 = new THREE.Mesh(new THREE.BoxGeometry(36, 32, 24), slateMat);
        t3_2.position.z = 48 + 12;
        t3_2.castShadow = true;
        t3Group.add(t3_2);

        // Tier 3 Crown Penthouse
        const t3_3 = new THREE.Mesh(new THREE.BoxGeometry(26, 22, 16), glassMat);
        t3_3.position.z = 72 + 8;
        t3_3.castShadow = true;
        t3Group.add(t3_3);

        // Rooftop mechanical HVAC chillers
        for (let k = 0; k < 3; k++) {
            const hvac = new THREE.Mesh(new THREE.BoxGeometry(6, 4, 3), slateMat);
            hvac.position.set(-6 + k * 6, 0, 97.5);
            t3Group.add(hvac);
        }

        group.add(t3Group);
        rooftopRelayNodes.push(new THREE.Vector3(10, 52, 98));

        // --------------------------------------------------------------------
        // TOWER 4: Rear Slender Spire Tower
        // --------------------------------------------------------------------
        const t4Group = new THREE.Group();
        t4Group.position.set(-25, 88, 0);
        const t4H = 110;

        const t4Mesh = new THREE.Mesh(new THREE.BoxGeometry(26, 26, t4H), glassMat);
        t4Mesh.position.z = t4H / 2;
        t4Mesh.castShadow = true;
        t4Group.add(t4Mesh);

        // Faceted spire crown
        const spireGeo = new THREE.ConeGeometry(8, 28, 4);
        const spire = new THREE.Mesh(spireGeo, slateMat);
        spire.rotation.x = Math.PI / 2;
        spire.position.z = t4H + 14;
        t4Group.add(spire);

        group.add(t4Group);
        rooftopRelayNodes.push(new THREE.Vector3(-25, 88, t4H + 28));

        // --------------------------------------------------------------------
        // TOWER 5: Warm Beige / Sandstone Tower with Rooftop Relay Pad
        // (Prominent sandstone tower on the right in reference photo)
        // --------------------------------------------------------------------
        const t5Group = new THREE.Group();
        t5Group.position.set(70, -12, 0);
        const t5W = 40;
        const t5D = 42;
        const t5H = 76;

        const t5Mesh = new THREE.Mesh(new THREE.BoxGeometry(t5W, t5D, t5H), beigeMat);
        t5Mesh.position.z = t5H / 2;
        t5Mesh.castShadow = true;
        t5Mesh.receiveShadow = true;
        t5Group.add(t5Mesh);

        // Elevator penthouse on roof
        const t5Pent = new THREE.Mesh(new THREE.BoxGeometry(16, 18, 9), beigeMat);
        t5Pent.position.set(-8, 6, t5H + 4.5);
        t5Group.add(t5Pent);

        // Glowing Circular Rooftop Relay Beacon Pad
        const padRadius = 9;
        const padGeo = new THREE.RingGeometry(2, padRadius, 32);
        const padMat = new THREE.MeshBasicMaterial({
            color: 0xffd600,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.85,
        });
        const relayPad = new THREE.Mesh(padGeo, padMat);
        relayPad.position.set(8, -6, t5H + 0.3);
        t5Group.add(relayPad);

        // Concentric inner cyan ring
        const innerRingGeo = new THREE.RingGeometry(0.5, 3.5, 32);
        const innerRingMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, side: THREE.DoubleSide });
        const innerRing = new THREE.Mesh(innerRingGeo, innerRingMat);
        innerRing.position.set(8, -6, t5H + 0.4);
        t5Group.add(innerRing);

        group.add(t5Group);
        rooftopRelayNodes.push(new THREE.Vector3(78, -18, t5H + 1));

        // --------------------------------------------------------------------
        // TOWER 6: Sandstone High-Rise 2 (Far-right)
        // --------------------------------------------------------------------
        const t6Group = new THREE.Group();
        t6Group.position.set(118, -42, 0);
        const t6H = 82;
        const t6Mesh = new THREE.Mesh(new THREE.BoxGeometry(32, 36, t6H), beigeMat);
        t6Mesh.position.z = t6H / 2;
        t6Mesh.castShadow = true;
        t6Group.add(t6Mesh);
        group.add(t6Group);
        rooftopRelayNodes.push(new THREE.Vector3(118, -42, t6H + 1));

        // --------------------------------------------------------------------
        // TOWER 7: Modern Curved White High-Rise
        // --------------------------------------------------------------------
        const t7Group = new THREE.Group();
        t7Group.position.set(135, 20, 0);
        const t7H = 66;
        const t7Mesh = new THREE.Mesh(new THREE.BoxGeometry(30, 32, t7H), slateMat);
        t7Mesh.position.z = t7H / 2;
        t7Mesh.castShadow = true;
        t7Group.add(t7Mesh);
        group.add(t7Group);

        // --------------------------------------------------------------------
        // MID-RISE BUILDINGS
        // --------------------------------------------------------------------
        // Mid-rise A (Center-front)
        const mr1 = new THREE.Mesh(new THREE.BoxGeometry(30, 32, 36), slateMat);
        mr1.position.set(12, -42, 18);
        mr1.castShadow = true;
        group.add(mr1);

        // Mid-rise B (West street)
        const mr2 = new THREE.Mesh(new THREE.BoxGeometry(24, 26, 30), beigeMat);
        mr2.position.set(-18, -55, 15);
        mr2.castShadow = true;
        group.add(mr2);

        // --------------------------------------------------------------------
        // LOW-RISE RESIDENTIAL VILLAGE (Terracotta Pitch-Roofed Buildings)
        // (Matching foreground low-rises with red roofs in reference photo)
        // --------------------------------------------------------------------
        const villageBuildings = [
            { x: -85, y: -60, w: 32, d: 20, h: 14, roofType: "gable", rot: 0.1 },
            { x: -50, y: -82, w: 24, d: 18, h: 12, roofType: "gable", rot: -0.15 },
            { x: 38, y: -75, w: 26, d: 20, h: 16, roofType: "gable", rot: 0.05 },
            { x: -15, y: -82, w: 28, d: 16, h: 11, roofType: "gable", rot: 0.0 },
            { x: 85, y: -85, w: 22, d: 16, h: 13, roofType: "gable", rot: -0.1 },
            { x: -115, y: -50, w: 20, d: 16, h: 12, roofType: "gable", rot: 0.2 },
        ];

        // Level Hip Roof Generator (BufferGeometry without tilted Euler rotations)
        function createHipRoofGeometry(w, d, h, overhang = 1.2) {
            const w2 = w / 2 + overhang;
            const d2 = d / 2 + overhang;
            const geom = new THREE.BufferGeometry();
            const vertices = [];

            function addTri(a, b, c) {
                vertices.push(...a, ...b, ...c);
            }

            if (w2 >= d2) {
                const ridgeHalf = Math.max(0, w2 - d2);
                const p0 = [-w2, -d2, 0];
                const p1 = [ w2, -d2, 0];
                const p2 = [ w2,  d2, 0];
                const p3 = [-w2,  d2, 0];
                const r0 = [-ridgeHalf, 0, h];
                const r1 = [ ridgeHalf, 0, h];

                addTri(p0, p1, r1);
                addTri(p0, r1, r0);
                addTri(p1, p2, r1);
                addTri(p2, p3, r0);
                addTri(p2, r0, r1);
                addTri(p3, p0, r0);
                addTri(p0, p3, p2);
                addTri(p0, p2, p1);
            } else {
                const ridgeHalf = Math.max(0, d2 - w2);
                const p0 = [-w2, -d2, 0];
                const p1 = [ w2, -d2, 0];
                const p2 = [ w2,  d2, 0];
                const p3 = [-w2,  d2, 0];
                const r0 = [0, -ridgeHalf, h];
                const r1 = [0,  ridgeHalf, h];

                addTri(p0, p1, r0);
                addTri(p1, p2, r1);
                addTri(p1, r1, r0);
                addTri(p2, p3, r1);
                addTri(p3, p0, r0);
                addTri(p3, r0, r1);
                addTri(p0, p3, p2);
                addTri(p0, p2, p1);
            }

            geom.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
            geom.computeVertexNormals();
            return geom;
        }

        // Level Shed Roof Generator (BufferGeometry)
        function createShedRoofGeometry(w, d, h, overhang = 1.2) {
            const w2 = w / 2 + overhang;
            const d2 = d / 2 + overhang;
            const geom = new THREE.BufferGeometry();
            const vertices = [];

            function addTri(a, b, c) {
                vertices.push(...a, ...b, ...c);
            }

            const p0 = [-w2, -d2, 0];
            const p1 = [ w2, -d2, 0];
            const p2 = [ w2,  d2, h];
            const p3 = [-w2,  d2, h];
            const p2_low = [ w2,  d2, 0];
            const p3_low = [-w2,  d2, 0];

            addTri(p0, p1, p2);
            addTri(p0, p2, p3);
            addTri(p2_low, p2, p3);
            addTri(p2_low, p3, p3_low);
            addTri(p1, p2, p2_low);
            addTri(p3_low, p3, p0);
            addTri(p0, p3_low, p2_low);
            addTri(p0, p2_low, p1);

            geom.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
            geom.computeVertexNormals();
            return geom;
        }

        villageBuildings.forEach(b => {
            const vGroup = new THREE.Group();
            vGroup.position.set(b.x, b.y, 0);
            vGroup.rotation.z = b.rot;

            // Cream stucco walls
            const wallGeo = new THREE.BoxGeometry(b.w, b.d, b.h);
            const wall = new THREE.Mesh(wallGeo, creamWallMat);
            wall.position.z = b.h / 2;
            wall.castShadow = true;
            wall.receiveShadow = true;
            vGroup.add(wall);

            // Small rectangular punch windows on walls
            const winMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.3 });
            [-b.w / 4, b.w / 4].forEach(wx => {
                const win = new THREE.Mesh(new THREE.BoxGeometry(2.5, 0.4, 3.0), winMat);
                win.position.set(wx, -b.d / 2 - 0.2, b.h * 0.55);
                vGroup.add(win);
            });

            // Terracotta red pitched roof (level, non-tilted, highway-cleared)
            const roofH = 7.0;
            if (b.roofType === "hip") {
                const hipGeo = createHipRoofGeometry(b.w, b.d, roofH, 1.2);
                const roof = new THREE.Mesh(hipGeo, redRoofMat);
                roof.position.set(0, 0, b.h);
                roof.castShadow = true;
                vGroup.add(roof);
            } else if (b.roofType === "shed") {
                const shedGeo = createShedRoofGeometry(b.w, b.d, roofH, 1.2);
                const roof = new THREE.Mesh(shedGeo, redRoofMat);
                roof.position.set(0, 0, b.h);
                roof.castShadow = true;
                vGroup.add(roof);
            } else {
                // Default & standard: Level Gable roof
                const roofShape = new THREE.Shape();
                roofShape.moveTo(-b.w / 2 - 1.2, 0);
                roofShape.lineTo(0, roofH);
                roofShape.lineTo(b.w / 2 + 1.2, 0);
                roofShape.closePath();

                const roofGeo = new THREE.ExtrudeGeometry(roofShape, { depth: b.d + 2.4, bevelEnabled: false });
                const roof = new THREE.Mesh(roofGeo, redRoofMat);
                roof.rotation.x = Math.PI / 2;
                roof.position.set(0, (b.d + 2.4) / 2, b.h);
                roof.castShadow = true;
                vGroup.add(roof);
            }

            group.add(vGroup);
        });
    }

    // ------------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------------
    return {
        // Initialize the complete Sector Delta diorama in the Three.js scene
        init(sceneTheater) {
            dioramaGroup = new THREE.Group();
            sceneTheater.add(dioramaGroup);

            // 1. Beveled Tabletop Tray (Plinth) 360m x 360m
            const plinth = createPlinthTray(360, 10);
            dioramaGroup.add(plinth);

            // 2. Elevated Curved Highway Overpass & Miniature Cars
            createElevatedOverpass(dioramaGroup);

            // 3. Stylized Miniature Green Trees
            createLandscapingTrees(dioramaGroup);

            // 4. Architectural Skyscraper City Cluster & Low-Rise Village
            createArchitecturalCity(dioramaGroup);

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
