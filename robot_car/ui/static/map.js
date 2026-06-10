/* Map canvas rendering + WebSocket client (F-18).
 *
 * Layers (drawn in order): base occupancy grid, forbidden zones, planned path,
 * named waypoints, robot arrow. The base grid lives on an offscreen 1px-per-cell
 * canvas that incremental `cell_update` events patch in place; the visible canvas is
 * redrawn each animation frame by scaling that offscreen buffer up and overlaying the
 * dynamic layers. Exposes a small `RobotMap` API used by ui.js.
 */
(function () {
  const socket = io();
  const canvas = document.getElementById("mapCanvas");
  const ctx = canvas.getContext("2d");

  let meta = { width: 200, height: 200, resolution: 0.05, origin_col: 100, origin_row: 100 };
  let scale = canvas.width / meta.width;          // visible px per cell

  // Offscreen base grid (1px per cell).
  const base = document.createElement("canvas");
  const bctx = base.getContext("2d");

  let pose = { x: 0, y: 0, theta: 0 };
  let path = [];                                   // [{x:col,y:row}]
  let waypoints = {};                              // name -> [wx, wy]
  let zones = [];                                  // [{x1,y1,x2,y2}] grid cells

  function colorForValue(v) {
    if (v < 35) return [240, 240, 240];           // free  -> white
    if (v > 65) return [20, 24, 32];              // wall  -> dark
    return [120, 128, 140];                       // unknown -> grey
  }

  function setMeta(m) {
    meta = m;
    base.width = m.width;
    base.height = m.height;
    scale = canvas.width / m.width;
  }

  function loadBasePng(b64) {
    const img = new Image();
    img.onload = () => { bctx.drawImage(img, 0, 0, base.width, base.height); };
    img.src = "data:image/png;base64," + b64;
  }

  function patchCells(cells) {
    cells.forEach((c) => {
      const [r, g, b] = colorForValue(c.value);
      bctx.fillStyle = `rgb(${r},${g},${b})`;
      bctx.fillRect(c.x, c.y, 1, 1);
    });
  }

  // -- coordinate helpers ---------------------------------------------------
  function worldToCell(wx, wy) {
    return {
      col: Math.round(wx / meta.resolution) + meta.origin_col,
      row: meta.origin_row - Math.round(wy / meta.resolution),
    };
  }
  function pixelToCell(px, py) {
    return { x: Math.floor(px / scale), y: Math.floor(py / scale) };
  }

  // -- rendering ------------------------------------------------------------
  function draw() {
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(base, 0, 0, canvas.width, canvas.height);

    // Forbidden zones (semi-transparent red).
    ctx.fillStyle = "rgba(255,60,60,0.30)";
    zones.forEach((z) => {
      const x = Math.min(z.x1, z.x2) * scale;
      const y = Math.min(z.y1, z.y2) * scale;
      const w = (Math.abs(z.x2 - z.x1) + 1) * scale;
      const h = (Math.abs(z.y2 - z.y1) + 1) * scale;
      ctx.fillRect(x, y, w, h);
    });

    // Planned path (dashed blue).
    if (path.length > 1) {
      ctx.strokeStyle = "#3aa0ff";
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      path.forEach((c, i) => {
        const px = (c.x + 0.5) * scale, py = (c.y + 0.5) * scale;
        i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Waypoints.
    Object.entries(waypoints).forEach(([name, w]) => {
      const cell = worldToCell(w[0], w[1]);
      const px = (cell.col + 0.5) * scale, py = (cell.row + 0.5) * scale;
      ctx.fillStyle = "#ffcf4e";
      ctx.beginPath(); ctx.arc(px, py, 4, 0, 7); ctx.fill();
      ctx.fillStyle = "#ffe9a8"; ctx.font = "11px sans-serif";
      ctx.fillText(name, px + 6, py - 6);
    });

    // Robot arrow.
    const cell = worldToCell(pose.x, pose.y);
    const px = (cell.col + 0.5) * scale, py = (cell.row + 0.5) * scale;
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-pose.theta);                       // canvas y is down -> negate
    ctx.fillStyle = "#5fd28a";
    ctx.beginPath();
    ctx.moveTo(10, 0); ctx.lineTo(-6, -6); ctx.lineTo(-6, 6); ctx.closePath();
    ctx.fill();
    ctx.restore();

    requestAnimationFrame(draw);
  }
  requestAnimationFrame(draw);

  // -- socket events --------------------------------------------------------
  socket.on("map_base", (m) => { setMeta(m); loadBasePng(m.png); });
  socket.on("cell_update", patchCells);
  socket.on("pose_update", (p) => { pose = p; });
  socket.on("path_update", (p) => { path = p; });
  socket.on("mode_change", (m) => window.RobotUI && window.RobotUI.onMode(m.mode));
  socket.on("status_log", (l) => window.RobotUI && window.RobotUI.onLog(l.level, l.msg));

  // -- public API (used by ui.js) ------------------------------------------
  window.RobotMap = {
    socket,
    canvas,
    pixelToCell,
    setWaypoints: (w) => { waypoints = w; },
    addZone: (z) => { zones.push(z); },
    clearZones: () => { zones = []; },
    getPose: () => pose,
  };
})();
