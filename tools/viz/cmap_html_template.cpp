// f4flight - tools/viz/cmap_html_template.cpp
//
// HTML template definitions for the campaign visualizer.

namespace f4flight {
namespace campaign {

const char* kCampaignHtmlHead = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Falcon 4 Campaign Visualizer</title>
<style>
:root {
  --bg: #0f111a;
  --panel: #161925;
  --panel2: #1e2235;
  --border: #2c314c;
  --text: #e6e8ee;
  --muted: #8a90a6;
  --accent: #60a5fa;
  --blue: #3b82f6;
  --red: #ef4444;
  --neutral: #94a3b8;
  --green: #10b981;
  --yellow: #f59e0b;
}
* { box-sizing: border-box; }
html, body {
  margin: 0; padding: 0; background: var(--bg); color: var(--text);
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  font-size: 14px; overflow: hidden; height: 100vh;
}
.app-container {
  display: flex; flex-direction: column; height: 100vh;
}
header.navbar {
  background: var(--panel); border-bottom: 1px solid var(--border);
  padding: 10px 20px; display: flex; justify-content: space-between; align-items: center;
}
header.navbar h1 { margin: 0; font-size: 18px; font-weight: 600; color: var(--text); }
.main-layout {
  display: flex; flex: 1; overflow: hidden;
}
.sidebar {
  width: 320px; background: var(--panel); border-right: 1px solid var(--border);
  display: flex; flex-direction: column; overflow-y: auto; padding: 15px; gap: 15px;
}
.map-viewport {
  flex: 1; background: #0b0c13; position: relative; overflow: hidden;
  display: flex; align-items: center; justify-content: center;
}
.canvas-container {
  position: absolute; cursor: grab;
}
.canvas-container:active { cursor: grabbing; }
canvas { display: block; box-shadow: 0 4px 20px rgba(0,0,0,0.5); border-radius: 4px; }
.bottom-panel {
  background: var(--panel); border-top: 1px solid var(--border);
  padding: 15px 20px; display: flex; flex-direction: column; gap: 10px;
}
.timeline-controls {
  display: flex; align-items: center; gap: 15px;
}
button {
  background: var(--panel2); border: 1px solid var(--border); color: var(--text);
  padding: 6px 12px; border-radius: 4px; cursor: pointer; transition: all 0.15s;
  font-size: 13px; font-weight: 500; display: flex; align-items: center; gap: 6px;
}
button:hover { border-color: var(--accent); background: var(--border); }
button.active { background: var(--accent); color: #0f111a; border-color: var(--accent); }
.scrubber {
  flex: 1; -webkit-appearance: none; appearance: none; height: 6px;
  background: var(--panel2); border-radius: 3px; outline: none;
}
.scrubber::-webkit-slider-thumb {
  -webkit-appearance: none; width: 16px; height: 16px; border-radius: 50%;
  background: var(--accent); cursor: pointer; border: 2px solid var(--bg);
}
.time-readout {
  font-family: monospace; font-size: 13px; color: var(--muted); min-width: 80px; text-align: right;
}
.legend-grid {
  display: grid; grid-template-columns: repeat(auto-fill, minmax(110px, 1fr)); gap: 8px; font-size: 11px;
}
.legend-item { display: flex; align-items: center; gap: 6px; }
.legend-box { width: 14px; height: 14px; border-radius: 3px; border: 1px solid rgba(255,255,255,0.1); }
.section-title {
  font-size: 11px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.5px;
  font-weight: 600; border-bottom: 1px solid var(--border); padding-bottom: 4px; margin-bottom: 8px;
}
.obj-card, .unit-card {
  background: var(--panel2); border: 1px solid var(--border); border-radius: 6px;
  padding: 10px; cursor: pointer; transition: border-color 0.15s;
}
.obj-card:hover, .unit-card:hover { border-color: var(--accent); }
.obj-card.active, .unit-card.active { border-color: var(--accent); background: rgba(96,165,250,0.05); }
.card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 4px; }
.card-name { font-weight: 600; font-size: 13px; }
.faction-badge {
  font-size: 9px; font-weight: 700; padding: 1px 5px; border-radius: 10px; text-transform: uppercase;
}
.faction-badge.blue { background: rgba(59,130,246,0.15); color: var(--accent); border: 1px solid rgba(59,130,246,0.3); }
.faction-badge.red { background: rgba(239,68,68,0.15); color: var(--red); border: 1px solid rgba(239,68,68,0.3); }
.faction-badge.neutral { background: rgba(148,163,184,0.15); color: var(--neutral); border: 1px solid rgba(148,163,184,0.3); }
.health-bar-container { background: rgba(0,0,0,0.3); height: 4px; border-radius: 2px; overflow: hidden; margin-top: 4px; }
.health-bar { height: 100%; transition: width 0.2s; }
.health-bar.good { background: var(--green); }
.health-bar.warning { background: var(--yellow); }
.health-bar.critical { background: var(--red); }
.meta-row { display: flex; justify-content: space-between; font-size: 11px; color: var(--muted); margin-top: 2px; }
.meta-val { color: var(--text); }
.structure-item {
  display: flex; justify-content: space-between; align-items: center; font-size: 12px; margin-bottom: 3px;
}
</style>
</head>
<body>
<div class="app-container">
  <header class="navbar">
    <h1 id="app-title">Falcon 4 Campaign Visualizer</h1>
    <div style="display: flex; gap: 10px; align-items: center;">
      <button id="btn-reset-view">Recenter Map</button>
      <span style="font-size: 11px; color: var(--muted);">Click objectives or units to inspect</span>
    </div>
  </header>

  <div class="main-layout">
    <aside class="sidebar">
      <div id="inspector-section" style="display: flex; flex-direction: column; gap: 15px;">
        <div>
          <div class="section-title">Selected Entity</div>
          <div id="inspector-content" style="color: var(--muted); text-align: center; padding: 20px 0; font-style: italic;">
            No objective or unit selected. Click any marker on the map to inspect its health, runways, and sub-vehicles.
          </div>
        </div>
      </div>

      <div style="margin-top: auto;">
        <div class="section-title">Active Objectives</div>
        <div id="objective-list" style="display: flex; flex-direction: column; gap: 8px; max-height: 250px; overflow-y: auto;">
        </div>
      </div>
    </aside>

    <main class="map-viewport" id="viewport">
      <div class="canvas-container" id="canvas-container">
        <canvas id="map-canvas"></canvas>
      </div>
    </main>
  </div>

  <footer class="bottom-panel">
    <div class="timeline-controls">
      <button id="btn-play">▶ Play</button>
      <button id="btn-prev">◀ Step Back</button>
      <button id="btn-next">Step Forward ▶</button>
      <input type="range" class="scrubber" id="timeline-scrubber" min="0" max="59" value="0">
      <div class="time-readout" id="time-readout">00:00:00</div>
    </div>

    <div style="display: flex; justify-content: space-between; align-items: center; border-top: 1px solid var(--border); padding-top: 10px;">
      <div class="legend-grid">
        <div class="legend-item"><div class="legend-box" style="background: #2563eb;"></div><span>Blue Territory</span></div>
        <div class="legend-item"><div class="legend-box" style="background: #dc2626;"></div><span>Red Territory</span></div>
        <div class="legend-item"><div class="legend-box" style="background: #4b5563;"></div><span>Neutral</span></div>
        <div class="legend-item"><div class="legend-box" style="background: #14532d;"></div><span>Forest</span></div>
        <div class="legend-item"><div class="legend-box" style="background: #1e3a8a;"></div><span>Ocean / Water</span></div>
        <div class="legend-item"><div class="legend-box" style="background: #78350f;"></div><span>Hills / Peaks</span></div>
      </div>
      <div style="font-size: 11px; color: var(--muted);">
        Interactive Wang Tile and Tactical Column Overlay Layer &bull; C++17 Map Data Engine
      </div>
    </div>
  </footer>
</div>

<script>
)HTML";

const char* kCampaignHtmlTail = R"HTML(
// JavaScript visualization implementation of Campaign State

document.getElementById('app-title').textContent = CAMPAIGN_TITLE;

const frames = CAMPAIGN_FRAMES;
let currentFrameIdx = 0;
let isPlaying = false;
let playInterval = null;

// Selected Entity State
let selectedEntity = null; // { type: 'obj'|'unit', id: number }

const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');
const container = document.getElementById('canvas-container');
const viewport = document.getElementById('viewport');

// Map dimensions and view transform
const TILE_SIZE = 40; // pixel size per tile in HTML
let mapCols = 32;
let mapRows = 32;
let mapWidth = mapCols * TILE_SIZE;
let mapHeight = mapRows * TILE_SIZE;

canvas.width = mapWidth;
canvas.height = mapHeight;

let panX = (viewport.clientWidth - mapWidth) / 2;
let panY = (viewport.clientHeight - mapHeight) / 2;
let zoom = 1.0;

function updateTransform() {
  container.style.transform = `translate(${panX}px, ${panY}px) scale(${zoom})`;
}
updateTransform();

// Dragging / Panning
let isDragging = false;
let startX, startY;

viewport.addEventListener('mousedown', (e) => {
  if (e.target.closest('.timeline-controls') || e.target.closest('.sidebar')) return;
  isDragging = true;
  startX = e.clientX - panX;
  startY = e.clientY - panY;
});

window.addEventListener('mousemove', (e) => {
  if (!isDragging) return;
  panX = e.clientX - startX;
  panY = e.clientY - startY;
  updateTransform();
});

window.addEventListener('mouseup', () => { isDragging = false; });

viewport.addEventListener('wheel', (e) => {
  e.preventDefault();
  const rect = container.getBoundingClientRect();
  const mouseX = e.clientX - rect.left;
  const mouseY = e.clientY - rect.top;

  const zoomFactor = 1.1;
  const nextZoom = e.deltaY < 0 ? zoom * zoomFactor : zoom / zoomFactor;
  if (nextZoom < 0.2 || nextZoom > 5.0) return;

  // Zoom to mouse position
  panX = e.clientX - (mouseX / zoom) * nextZoom;
  panY = e.clientY - (mouseY / zoom) * nextZoom;
  zoom = nextZoom;
  updateTransform();
}, { passive: false });

document.getElementById('btn-reset-view').addEventListener('click', () => {
  zoom = 1.0;
  panX = (viewport.clientWidth - mapWidth) / 2;
  panY = (viewport.clientHeight - mapHeight) / 2;
  updateTransform();
});

// Cover Type Coloring and Patterns
function getCoverColor(cover) {
  switch(cover) {
    case 'Water': return '#1a2e40';
    case 'Coast': return '#2c4a3e';
    case 'Plains': return '#243e30';
    case 'Forest': return '#122c1b';
    case 'Hills': return '#3c3528';
    case 'Mountains': return '#504838';
    case 'Swamp': return '#1e2d28';
    case 'Desert': return '#4c4228';
    case 'City': return '#2e3540';
  }
  return '#222';
}

function getFactionColor(faction) {
  if (faction === 1) return '#3b82f6'; // Blue
  if (faction === 2) return '#ef4444'; // Red
  return '#94a3b8'; // Neutral
}

// Convert world coordinates (ft, ENU where origin is bottom left) to screen pixel coordinates on canvas
// The procedural map is 32 NM x 32 NM. 1 NM = 6076.12 ft.
const NM_FT = 6076.12;
const MAP_SIZE_FT = 32 * NM_FT;

function worldToCanvas(pos) {
  // x goes from 0 to MAP_SIZE_FT -> mapWidth
  // y goes from 0 to MAP_SIZE_FT -> mapHeight.
  // In screen canvas, (0,0) is top-left, so we flip Y:
  const px = (pos[0] / MAP_SIZE_FT) * mapWidth;
  const py = mapHeight - (pos[1] / MAP_SIZE_FT) * mapHeight;
  return { x: px, y: py };
}

// Draw the current campaign frame on canvas
function render() {
  const frame = frames[currentFrameIdx];
  if (!frame) return;

  ctx.clearRect(0, 0, mapWidth, mapHeight);

  // 1. Draw Terrain Tiles
  const ter = frame.terrain;
  mapCols = ter.cols;
  mapRows = ter.rows;

  for (let r = 0; r < mapRows; ++r) {
    for (let c = 0; c < mapCols; ++c) {
      const idx = r * mapCols + c;
      const tile = ter.tiles[idx];

      const x = c * TILE_SIZE;
      const y = (mapRows - 1 - r) * TILE_SIZE; // Flip Y row so row 0 is at the bottom

      // Base color
      ctx.fillStyle = getCoverColor(tile.c);
      ctx.fillRect(x, y, TILE_SIZE, TILE_SIZE);

      // Border lines (grid)
      ctx.strokeStyle = 'rgba(44, 49, 76, 0.15)';
      ctx.strokeRect(x, y, TILE_SIZE, TILE_SIZE);

      // Draw road/rail overlays
      if (tile.road) {
        ctx.strokeStyle = '#4b5563';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(x + TILE_SIZE/2, y);
        ctx.lineTo(x + TILE_SIZE/2, y + TILE_SIZE);
        ctx.moveTo(x, y + TILE_SIZE/2);
        ctx.lineTo(x + TILE_SIZE, y + TILE_SIZE/2);
        ctx.stroke();
      }
    }
  }

  // 2. Draw Road Node connections as highways
  const rn = frame.roadNetwork;
  ctx.strokeStyle = '#4b5563';
  ctx.lineWidth = 2.5;
  // Let's connect objectives procedurally since edges aren't fully listed in json serialization
  const objPosMap = {};
  frame.objectives.forEach(obj => {
    objPosMap[obj.id] = worldToCanvas(obj.pos);
  });

  // Roads connections based on Generator setup:
  const roadEdges = [
    [2, 10], [3, 10], [1, 10], [1, 11], [4, 10], [5, 11], [4, 5], [5, 6], [7, 6], [4, 7]
  ];

  // Custom junction positions
  objPosMap[10] = worldToCanvas([15 * NM_FT, 11 * NM_FT, 80]);
  objPosMap[11] = worldToCanvas([22 * NM_FT, 14 * NM_FT, 150]);

  ctx.beginPath();
  roadEdges.forEach(([from, to]) => {
    const p1 = objPosMap[from];
    const p2 = objPosMap[to];
    if (p1 && p2) {
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
    }
  });
  ctx.stroke();

  // 3. Draw Flightpaths of Air Squadrons
  frame.units.forEach(unit => {
    if (!unit.isActive || unit.type !== 'AirSquadron') return;
    if (unit.path && unit.path.length > 0) {
      ctx.strokeStyle = getFactionColor(unit.faction);
      ctx.setLineDash([5, 5]);
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      const pStart = worldToCanvas(unit.pos);
      ctx.moveTo(pStart.x, pStart.y);
      unit.path.forEach(wp => {
        const pwp = worldToCanvas(wp);
        ctx.lineTo(pwp.x, pwp.y);
      });
      ctx.stroke();
      ctx.setLineDash([]); // Reset
    }
  });

  // 4. Draw Objectives
  frame.objectives.forEach(obj => {
    const sc = worldToCanvas(obj.pos);

    // Draw stylized circle/square
    ctx.fillStyle = getFactionColor(obj.faction);
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;

    ctx.beginPath();
    if (obj.type === 'Airbase') {
      // Triangle/wings for Airbase
      ctx.moveTo(sc.x, sc.y - 12);
      ctx.lineTo(sc.x + 12, sc.y + 8);
      ctx.lineTo(sc.x - 12, sc.y + 8);
      ctx.closePath();
    } else if (obj.type === 'Headquarters') {
      // Star/shield for Headquarters
      ctx.rect(sc.x - 10, sc.y - 10, 20, 20);
    } else {
      // Standard Circle
      ctx.arc(sc.x, sc.y, 8, 0, Math.PI * 2);
    }
    ctx.fill();
    ctx.stroke();

    // Objective Name
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 11px sans-serif';
    ctx.shadowColor = 'black';
    ctx.shadowBlur = 4;
    ctx.textAlign = 'center';
    ctx.fillText(obj.name, sc.x, sc.y - 16);
    ctx.shadowBlur = 0; // Reset

    // Objective Health Bar overlay
    ctx.fillStyle = '#10b981';
    ctx.fillRect(sc.x - 10, sc.y + 12, 20 * obj.health, 3);
    ctx.strokeStyle = 'rgba(0,0,0,0.5)';
    ctx.lineWidth = 0.5;
    ctx.strokeRect(sc.x - 10, sc.y + 12, 20, 3);
  });

  // 5. Draw Units and Column Vehicles
  frame.units.forEach(unit => {
    if (!unit.isActive) return;

    const uPos = worldToCanvas(unit.pos);

    // Render individual vehicles in column/wedge
    unit.subUnits.forEach(su => {
      const suPos = worldToCanvas(su.pos);
      ctx.fillStyle = getFactionColor(unit.faction);
      ctx.strokeStyle = '#000';
      ctx.lineWidth = 1;

      ctx.beginPath();
      if (unit.type === 'GroundBattalion') {
        // Draw Tiny tanks/APC dots
        ctx.arc(suPos.x, suPos.y, 4, 0, Math.PI * 2);
      } else {
        // Draw Tiny airplanes
        ctx.moveTo(suPos.x, suPos.y - 6);
        ctx.lineTo(suPos.x + 5, suPos.y + 4);
        ctx.lineTo(suPos.x - 5, suPos.y + 4);
        ctx.closePath();
      }
      ctx.fill();
      ctx.stroke();
    });

    // Draw unit aggregate icon
    ctx.fillStyle = getFactionColor(unit.faction);
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(uPos.x, uPos.y, 11, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Text Label inside unit
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(unit.type === 'GroundBattalion' ? 'XX' : '++', uPos.x, uPos.y + 3);

    // Unit Name above/below
    ctx.fillStyle = getFactionColor(unit.faction);
    ctx.font = '10px sans-serif';
    ctx.fillText(unit.name, uPos.x, uPos.y + 20);
  });
}

// Update the timeline UI controls
function updateTimeline() {
  const frame = frames[currentFrameIdx];
  if (!frame) return;

  document.getElementById('timeline-scrubber').value = currentFrameIdx;

  // Format time (simulated seconds since start)
  const totalSeconds = frame.time;
  const hrs = Math.floor(totalSeconds / 3600).toString().padStart(2, '0');
  const mins = Math.floor((totalSeconds % 3600) / 60).toString().padStart(2, '0');
  const secs = Math.floor(totalSeconds % 60).toString().padStart(2, '0');
  document.getElementById('time-readout').textContent = `${hrs}:${mins}:${secs}`;

  render();
  updateSidebar();
}

// Sidebars & Lists
function populateObjectiveList() {
  const frame = frames[currentFrameIdx];
  const list = document.getElementById('objective-list');
  list.innerHTML = '';

  frame.objectives.forEach(obj => {
    const card = document.createElement('div');
    card.className = `obj-card ${selectedEntity && selectedEntity.type === 'obj' && selectedEntity.id === obj.id ? 'active' : ''}`;

    let badgeClass = 'neutral';
    if (obj.faction === 1) badgeClass = 'blue';
    if (obj.faction === 2) badgeClass = 'red';

    card.innerHTML = `
      <div class="card-header">
        <div class="card-name">${obj.name}</div>
        <span class="faction-badge ${badgeClass}">${obj.faction === 1 ? 'Blue' : obj.faction === 2 ? 'Red' : 'Neut'}</span>
      </div>
      <div class="health-bar-container">
        <div class="health-bar ${obj.health > 0.6 ? 'good' : obj.health > 0.3 ? 'warning' : 'critical'}" style="width: ${obj.health * 100}%"></div>
      </div>
    `;

    card.addEventListener('click', () => {
      selectedEntity = { type: 'obj', id: obj.id };
      updateTimeline();
    });

    list.appendChild(card);
  });
}

function updateSidebar() {
  populateObjectiveList();

  const content = document.getElementById('inspector-content');
  if (!selectedEntity) {
    content.innerHTML = `
      <div style="color: var(--muted); text-align: center; padding: 20px 0; font-style: italic;">
        No objective or unit selected. Click any marker on the map to inspect its health, runways, and sub-vehicles.
      </div>
    `;
    return;
  }

  const frame = frames[currentFrameIdx];

  if (selectedEntity.type === 'obj') {
    const obj = frame.objectives.find(o => o.id === selectedEntity.id);
    if (!obj) return;

    let badgeClass = 'neutral';
    if (obj.faction === 1) badgeClass = 'blue';
    if (obj.faction === 2) badgeClass = 'red';

    let html = `
      <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:10px;">
        <h3 style="margin:0; font-size:16px;">${obj.name}</h3>
        <span class="faction-badge ${badgeClass}">${obj.faction === 1 ? 'Blue' : obj.faction === 2 ? 'Red' : 'Neut'}</span>
      </div>
      <div class="meta-row"><span>Type:</span><span class="meta-val">${obj.type}</span></div>
      <div class="meta-row"><span>Overall Integrity:</span><span class="meta-val">${(obj.health * 100).toFixed(0)}%</span></div>

      <div class="health-bar-container" style="height: 6px; margin: 8px 0 15px;">
        <div class="health-bar ${obj.health > 0.6 ? 'good' : obj.health > 0.3 ? 'warning' : 'critical'}" style="width: ${obj.health * 100}%"></div>
      </div>

      <div class="section-title">Sub-Structures</div>
    `;

    obj.structures.forEach(st => {
      html += `
        <div class="structure-item">
          <span>${st.name}</span>
          <span style="font-weight:600; color:${st.h > 0.6 ? 'var(--green)' : st.h > 0.3 ? 'var(--yellow)' : 'var(--red)'}">${(st.h * 100).toFixed(0)}%</span>
        </div>
      `;
    });

    if (obj.type === 'Airbase' && obj.runways && obj.runways.length > 0) {
      html += `<div class="section-title" style="margin-top:15px;">Active Runways</div>`;
      obj.runways.forEach(r => {
        html += `
          <div class="structure-item">
            <span>${r.name} (${(r.width).toFixed(0)}ft wide)</span>
            <span style="font-weight:600; color:${r.h > 0.6 ? 'var(--green)' : 'var(--red)'}">${r.h > 0.6 ? 'Active' : 'BOMBED'}</span>
          </div>
        `;
      });
    }

    content.innerHTML = html;
  } else if (selectedEntity.type === 'unit') {
    const unit = frame.units.find(u => u.id === selectedEntity.id);
    if (!unit) return;

    let badgeClass = 'neutral';
    if (unit.faction === 1) badgeClass = 'blue';
    if (unit.faction === 2) badgeClass = 'red';

    let html = `
      <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:10px;">
        <h3 style="margin:0; font-size:16px;">${unit.name}</h3>
        <span class="faction-badge ${badgeClass}">${unit.faction === 1 ? 'Blue' : 'Red'}</span>
      </div>
      <div class="meta-row"><span>Type:</span><span class="meta-val">${unit.type}</span></div>
      <div class="meta-row"><span>Combat Stance:</span><span class="meta-val" style="color:var(--accent); font-weight:600;">${unit.formation}</span></div>
      <div class="meta-row"><span>March Speed:</span><span class="meta-val">${(unit.speed * 0.592).toFixed(1)} kts</span></div>
      <div class="meta-row"><span>Force Strength:</span><span class="meta-val">${(unit.health * 100).toFixed(0)}%</span></div>

      <div class="health-bar-container" style="height: 6px; margin: 8px 0 15px;">
        <div class="health-bar ${unit.health > 0.6 ? 'good' : unit.health > 0.3 ? 'warning' : 'critical'}" style="width: ${unit.health * 100}%"></div>
      </div>

      <div class="section-title">Constituent Vehicles</div>
    `;

    unit.subUnits.forEach(su => {
      html += `
        <div class="structure-item">
          <span>${su.name}</span>
          <span style="font-weight:600; color:${su.h > 0.6 ? 'var(--green)' : 'var(--red)'}">${(su.h * 100).toFixed(0)}%</span>
        </div>
      `;
    });

    content.innerHTML = html;
  }
}

// Click Canvas to Select
canvas.addEventListener('click', (e) => {
  const rect = canvas.getBoundingClientRect();
  const clickX = (e.clientX - rect.left) / zoom;
  const clickY = (e.clientY - rect.top) / zoom;

  const frame = frames[currentFrameIdx];
  if (!frame) return;

  // 1. Try selecting a Unit
  let found = false;
  for (let unit of frame.units) {
    if (!unit.isActive) continue;
    const uPos = worldToCanvas(unit.pos);
    const dist = Math.hypot(clickX - uPos.x, clickY - uPos.y);
    if (dist <= 15) {
      selectedEntity = { type: 'unit', id: unit.id };
      found = true;
      break;
    }
  }

  // 2. Try selecting an Objective
  if (!found) {
    for (let obj of frame.objectives) {
      const sc = worldToCanvas(obj.pos);
      const dist = Math.hypot(clickX - sc.x, clickY - sc.y);
      if (dist <= 15) {
        selectedEntity = { type: 'obj', id: obj.id };
        found = true;
        break;
      }
    }
  }

  if (!found) {
    selectedEntity = null;
  }

  updateTimeline();
});

// Playback Logic
const playBtn = document.getElementById('btn-play');
const nextBtn = document.getElementById('btn-next');
const prevBtn = document.getElementById('btn-prev');
const scrubber = document.getElementById('timeline-scrubber');

scrubber.max = frames.length - 1;

scrubber.addEventListener('input', (e) => {
  currentFrameIdx = parseInt(e.target.value);
  updateTimeline();
});

function togglePlay() {
  isPlaying = !isPlaying;
  if (isPlaying) {
    playBtn.textContent = '⏸ Pause';
    playInterval = setInterval(() => {
      currentFrameIdx++;
      if (currentFrameIdx >= frames.length) {
        currentFrameIdx = 0;
      }
      updateTimeline();
    }, 500);
  } else {
    playBtn.textContent = '▶ Play';
    clearInterval(playInterval);
  }
}

playBtn.addEventListener('click', togglePlay);

nextBtn.addEventListener('click', () => {
  currentFrameIdx = (currentFrameIdx + 1) % frames.length;
  updateTimeline();
});

prevBtn.addEventListener('click', () => {
  currentFrameIdx = (currentFrameIdx - 1 + frames.length) % frames.length;
  updateTimeline();
});

// Initialization
updateTimeline();

</script>
</body>
</html>
)HTML";

}
}
