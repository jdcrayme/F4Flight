// f4flight - tools/viz/html_report.cpp
//
// Self-contained interactive HTML report generator.
//
// Architecture:
//   - C++ side: reads Trace structs, serializes them to compact JSON, and
//     embeds the JSON inline inside a <script> tag in a static HTML template.
//   - JS side (vanilla, no dependencies): renders an index dashboard (grid of
//     trace cards with mini flight-path thumbnails) and a detail view (large
//     top-down flight path + altitude/speed/G time-series with a scrubber,
//     play/pause, and color-by controls).
//
// The HTML file is fully self-contained — no web server, no external files,
// no network. Open it in any browser via file://.

#include "html_report.h"
#include "trace.h"

#include <sstream>
#include <string>
#include <vector>

namespace f4flight {

// Escape a JSON string for safe embedding inside an HTML <script> tag.
// The only character that can break out of <script> is the sequence "</".
// We escape '<' as the JSON unicode escape \u003c, which is valid in both
// JSON and JS string literals and decodes back to '<' on parse.
static std::string escapeForScript(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 16);
    for (char c : s) {
        if (c == '<') out += "\\u003c";
        else out += c;
    }
    return out;
}

// Produce a JSON string literal (with quotes) from a C++ string, applying
// both JSON escaping (for " and \) and the </script> safety escape.
static std::string jsonString(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    out += "\"";
    return escapeForScript(out);
}

// Downsample a trace's frames for HTML embedding. Long scenarios (e.g. a
// 6-phase / 10-minute ai_basic run) produce 30k+ frames; embedding all of
// them makes the HTML file huge and the SVG rendering slow. For
// visualization purposes 3000 points per trace is more than enough — that's
// 50 Hz resolution for a 60-second scenario, or ~5 Hz for a 10-minute one,
// either of which renders a smooth flight path.
//
// Phase boundaries are preserved by always keeping the first frame of each
// phase (so the phase-start marker lands exactly right) and the last frame
// overall. Phase start_s/end_s times are kept verbatim, so the JS
// phaseRanges() matcher still groups frames into the correct phases.
static const size_t kMaxFramesPerTrace = 3000;
static Trace downsampleForHtml(const Trace& in) {
    Trace out = in;  // copy metadata + phases
    out.frames.clear();
    const size_t n = in.frames.size();
    if (n <= kMaxFramesPerTrace) {
        out.frames = in.frames;
        return out;
    }
    // Uniform stride, plus guaranteed phase-start frames.
    const size_t stride = (n + kMaxFramesPerTrace - 1) / kMaxFramesPerTrace;
    // Indices we must keep: every stride-th frame, plus the first frame of
    // each phase (closest frame at or after start_s).
    std::vector<bool> keep(n, false);
    for (size_t i = 0; i < n; i += stride) keep[i] = true;
    keep[n - 1] = true;
    for (const auto& ph : in.phases) {
        // Find first frame with t >= start_s.
        size_t fi = 0;
        while (fi < n && in.frames[fi].t < ph.start_s) ++fi;
        if (fi < n) keep[fi] = true;
    }
    out.frames.reserve(kMaxFramesPerTrace + in.phases.size() + 2);
    for (size_t i = 0; i < n; ++i) {
        if (keep[i]) out.frames.push_back(in.frames[i]);
    }
    return out;
}

// ===========================================================================
// HTML template — part 1 (head, CSS, HTML structure, up to the TRACES array)
// ===========================================================================
// Uses a raw string literal. The delimiter is )HTML" which will not appear
// in normal HTML/JS/CSS content.
static const char* kHtmlHead = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>F4Flight Flight Test Report</title>
<style>
:root{
  --bg:#0f1117; --panel:#161922; --panel2:#1d2130; --border:#2a2f42;
  --text:#e6e8ee; --muted:#8a90a6; --dim:#5a6076;
  --pass:#4ade80; --fail:#f87171; --skip:#94a3b8; --warn:#fbbf24;
  --accent:#60a5fa; --accent2:#a78bfa;
  --alt:#4ade80; --spd:#60a5fa; --gcol:#f59e0b;
  --grid:#222637; --grid2:#2d3247;
}
*{box-sizing:border-box}
html,body{margin:0;padding:0;background:var(--bg);color:var(--text);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
  font-size:14px;line-height:1.5}
a{color:var(--accent);text-decoration:none}
a:hover{text-decoration:underline}
button{font-family:inherit;font-size:13px;cursor:pointer}
.hidden{display:none!important}

/* ---------- Layout ---------- */
.app{max-width:1400px;margin:0 auto;padding:16px 20px 60px}
header.app-header{padding:8px 0 16px;border-bottom:1px solid var(--border);margin-bottom:20px}
header.app-header h1{margin:0;font-size:20px;font-weight:600}
header.app-header .sub{color:var(--muted);font-size:13px;margin-top:2px}

/* ---------- Index view ---------- */
.index-summary{display:flex;gap:24px;flex-wrap:wrap;margin-bottom:20px;align-items:center}
.index-summary .stat{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:10px 16px}
.index-summary .stat .v{font-size:20px;font-weight:600}
.index-summary .stat .l{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.5px}
.filter-bar{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:16px;align-items:center}
.filter-bar select,.filter-bar input{background:var(--panel);border:1px solid var(--border);color:var(--text);
  border-radius:6px;padding:6px 10px;font-size:13px}
.filter-bar label{display:flex;align-items:center;gap:6px;font-size:13px;color:var(--muted)}
.card-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:14px}
.card{background:var(--panel);border:1px solid var(--border);border-radius:10px;padding:14px;cursor:pointer;
  transition:border-color .15s,transform .15s;overflow:hidden}
.card:hover{border-color:var(--accent);transform:translateY(-2px)}
.card .thumb{background:var(--bg);border-radius:6px;margin-bottom:10px;border:1px solid var(--border)}
.card .thumb svg{display:block;width:100%;height:auto}
.card .title{font-weight:600;font-size:13px;margin-bottom:2px;word-break:break-word}
.card .meta{font-size:11px;color:var(--muted)}
.badge{display:inline-block;padding:1px 8px;border-radius:10px;font-size:11px;font-weight:600;letter-spacing:.3px}
.badge.pass{background:rgba(74,222,128,.15);color:var(--pass);border:1px solid rgba(74,222,128,.3)}
.badge.fail{background:rgba(248,113,113,.15);color:var(--fail);border:1px solid rgba(248,113,113,.3)}
.badge.skip{background:rgba(148,163,184,.15);color:var(--skip);border:1px solid rgba(148,163,184,.3)}
.badge.mixed{background:rgba(251,191,36,.15);color:var(--warn);border:1px solid rgba(251,191,36,.3)}
.empty{color:var(--muted);text-align:center;padding:60px 20px}

/* ---------- Detail view ---------- */
.detail-header{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:16px}
.detail-header h2{margin:0;font-size:18px;font-weight:600}
.back-btn{background:var(--panel);border:1px solid var(--border);color:var(--text);border-radius:6px;
  padding:6px 12px}
.back-btn:hover{border-color:var(--accent)}
.detail-summary{display:flex;gap:16px;flex-wrap:wrap;color:var(--muted);font-size:13px;margin-bottom:16px}
.detail-summary span b{color:var(--text)}

.phase-chips{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:14px}
.chip{background:var(--panel);border:1px solid var(--border);border-radius:14px;padding:3px 12px;
  font-size:12px;cursor:pointer;white-space:nowrap;transition:all .12s}
.chip:hover{border-color:var(--accent)}
.chip.active{background:var(--panel2);border-color:var(--accent)}
.chip .n{color:var(--dim);margin-right:4px}
.chip.pass{border-left:3px solid var(--pass)}
.chip.fail{border-left:3px solid var(--fail)}
.chip.skip{border-left:3px solid var(--skip)}

.detail-grid{display:grid;grid-template-columns:1fr;gap:16px}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:10px;padding:14px}
.panel h3{margin:0 0 10px;font-size:13px;color:var(--muted);text-transform:uppercase;letter-spacing:.5px;font-weight:600}
.panel svg{display:block;width:100%;height:auto}

/* color-by controls */
.controls{display:flex;gap:14px;flex-wrap:wrap;align-items:center;margin-bottom:14px}
.controls .group{display:flex;align-items:center;gap:6px}
.controls .group>span{font-size:12px;color:var(--muted)}
.controls .btn{background:var(--panel);border:1px solid var(--border);color:var(--text);border-radius:6px;
  padding:6px 12px;font-size:13px}
.controls .btn:hover{border-color:var(--accent)}
.controls .btn.active{background:var(--accent);color:#0f1117;border-color:var(--accent)}
.controls .btn.play{min-width:80px}
.seg{display:inline-flex;border:1px solid var(--border);border-radius:6px;overflow:hidden}
.seg button{background:var(--panel);border:none;color:var(--muted);padding:6px 10px;font-size:12px;
  border-right:1px solid var(--border)}
.seg button:last-child{border-right:none}
.seg button.active{background:var(--panel2);color:var(--text)}

/* scrubber */
.scrubber-row{display:flex;align-items:center;gap:12px;margin-top:10px}
.scrubber{flex:1;-webkit-appearance:none;appearance:none;height:6px;background:var(--panel2);
  border-radius:3px;outline:none}
.scrubber::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;border-radius:50%;
  background:var(--accent);cursor:pointer;border:2px solid var(--bg)}
.scrubber::-moz-range-thumb{width:16px;height:16px;border-radius:50%;background:var(--accent);
  cursor:pointer;border:2px solid var(--bg)}
.time-label{font-family:ui-monospace,"SF Mono",Menlo,monospace;font-size:12px;color:var(--muted);
  min-width:64px;text-align:right}

/* frame readout */
.readout{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px;margin-top:12px}
.readout .cell{background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:8px 10px}
.readout .cell .l{font-size:10px;color:var(--muted);text-transform:uppercase;letter-spacing:.5px}
.readout .cell .v{font-family:ui-monospace,"SF Mono",Menlo,monospace;font-size:15px;font-weight:600}
.readout .cell.mode .v,.readout .cell.phase .v{font-size:12px}

/* SVG styles */
.axis-text{font-family:ui-monospace,"SF Mono",Menlo,monospace;font-size:10px;fill:var(--dim)}
.grid-line{stroke:var(--grid);stroke-width:.5;fill:none}
.grid-line-major{stroke:var(--grid2);stroke-width:.75;fill:none}
.legend-text{font-size:11px;fill:var(--muted)}

/* criteria panel */
.criteria-panel{background:var(--panel);border:1px solid var(--border);border-radius:10px;
  padding:14px;margin-bottom:14px}
.criteria-panel h3{margin:0 0 8px;font-size:13px;color:var(--muted);text-transform:uppercase;
  letter-spacing:.5px;font-weight:600}
.criteria-table{width:100%;border-collapse:collapse;font-size:12px}
.criteria-table th{text-align:left;padding:4px 8px;color:var(--muted);border-bottom:1px solid var(--border);
  font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.3px}
.criteria-table td{padding:5px 8px;border-bottom:1px solid var(--border);vertical-align:top}
.criteria-table td.name{font-weight:600;white-space:nowrap}
.criteria-table td.status{width:60px;text-align:center}
.criteria-table td.criteria-text{color:var(--muted);font-size:11px}

/* 3D view */
#topdown-host svg,#threed-host svg{cursor:grab}
#threed-host svg:active{cursor:grabbing}
</style>
</head>
<body>
<div class="app">
  <header class="app-header">
    <h1 id="app-title">F4Flight Flight Test Report</h1>
    <div class="sub" id="app-sub"></div>
  </header>

  <!-- Index view -->
  <section id="index-view">
    <div class="index-summary" id="index-summary"></div>
    <div class="filter-bar" id="filter-bar">
      <label>Filter
        <input type="search" id="filter-input" placeholder="aircraft / scenario..." style="min-width:200px">
      </label>
      <label>Status
        <select id="filter-status">
          <option value="">all</option>
          <option value="pass">passing</option>
          <option value="fail">failing</option>
          <option value="mixed">mixed</option>
        </select>
      </label>
    </div>
    <div class="card-grid" id="card-grid"></div>
  </section>

  <!-- Detail view -->
  <section id="detail-view" class="hidden">
    <div class="detail-header">
      <button class="back-btn" id="back-btn">&larr; Index</button>
      <h2 id="detail-title"></h2>
      <span class="badge" id="detail-badge"></span>
    </div>
    <div class="detail-summary" id="detail-summary"></div>
    <div class="criteria-panel hidden" id="criteria-panel">
      <h3>Pass / Fail Criteria</h3>
      <table class="criteria-table" id="criteria-table"></table>
    </div>
    <div class="phase-chips" id="phase-chips"></div>
    <div class="controls">
      <div class="group">
        <button class="btn play" id="play-btn">&#9654; Play</button>
      </div>
      <div class="group">
        <span>View</span>
        <div class="seg" id="view-seg">
          <button data-view="2d" class="active">2D top-down</button>
          <button data-view="3d">3D orbit</button>
        </div>
      </div>
      <div class="group">
        <span>Speed</span>
        <div class="seg" id="speed-seg">
          <button data-speed="0.25">0.25&times;</button>
          <button data-speed="0.5">0.5&times;</button>
          <button data-speed="1" class="active">1&times;</button>
          <button data-speed="2">2&times;</button>
          <button data-speed="4">4&times;</button>
        </div>
      </div>
      <div class="group">
        <span>Color by</span>
        <div class="seg" id="color-seg">
          <button data-color="phase" class="active">phase</button>
          <button data-color="passfail">pass/fail</button>
          <button data-color="altitude">altitude</button>
          <button data-color="speed">speed</button>
          <button data-color="g">G-load</button>
          <button data-color="mode">AI mode</button>
        </div>
      </div>
    </div>
    <div class="scrubber-row">
      <span class="time-label" id="time-now">0.0s</span>
      <input type="range" class="scrubber" id="scrubber" min="0" max="1000" value="0">
      <span class="time-label" id="time-end">0.0s</span>
    </div>
    <div class="detail-grid" style="margin-top:14px">
      <div class="panel">
        <h3 id="path-panel-title">Top-Down Flight Path</h3>
        <div id="topdown-host"></div>
        <div id="threed-host" class="hidden"></div>
        <div class="readout" id="readout"></div>
      </div>
      <div class="panel">
        <h3>Altitude / Speed / G-load vs Time</h3>
        <div id="timeseries-host"></div>
      </div>
    </div>
  </section>
</div>

<script>
)HTML";

// ===========================================================================
// HTML template — part 2 (all the JavaScript rendering logic + close)
// ===========================================================================
// The JS is split across kHtmlTail1..kHtmlTail4 (rather than one literal)
// because MSVC caps a single string literal at 16380 chars (error C2026).
// Each piece is ~8-12 KB, well under the limit. GCC/Clang have no such cap.
//
// kHtmlTail1 starts with ";" to terminate the `const TRACES = [...]` array
// that generateHtmlReport emits between kHtmlHead and the tail pieces. The
// `"use strict"` directive and REPORT_TITLE are emitted by C++ before
// TRACES so that "use strict" is the first statement in the script.
static const char* kHtmlTail1 = R"HTML(;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
const state = {
  traceIdx: -1,       // -1 = index view
  t: 0,               // playhead time (seconds)
  playing: false,
  speed: 1,
  colorBy: 'phase',
  viewMode: '2d',     // '2d' or '3d'
  segmentIdx: 0,      // which segment (discontinuity group) is active, 0-based
  zoom: 1.0,          // mouse-wheel zoom factor (1.0 = fit)
  panX: 0, panY: 0,   // pan offset in screen px
  orbit: {yaw: 0.6, pitch: 0.4, dragging: false, lastX: 0, lastY: 0},
  playTimer: null,
  filter: '',
  filterStatus: '',
};

// ---------------------------------------------------------------------------
// Color utilities
// ---------------------------------------------------------------------------
function hsvToRgb(h, s, v) {
  // h in [0,1)
  const i = Math.floor(h * 6);
  const f = h * 6 - i;
  const p = v * (1 - s);
  const q = v * (1 - f * s);
  const t = v * (1 - (1 - f) * s);
  let r, g, b;
  switch (i % 6) {
    case 0: r=v; g=t; b=p; break;
    case 1: r=q; g=v; b=p; break;
    case 2: r=p; g=v; b=t; break;
    case 3: r=p; g=q; b=v; break;
    case 4: r=t; g=p; b=v; break;
    default: r=v; g=p; b=q; break;
  }
  return [Math.round(r*255), Math.round(g*255), Math.round(b*255)];
}
function rgbHex(r,g,b){return '#'+[r,g,b].map(x=>Math.max(0,Math.min(255,Math.round(x))).toString(16).padStart(2,'0')).join('');}
function hashStr(s){let h=5381;for(let i=0;i<s.length;i++)h=((h<<5)+h)+s.charCodeAt(i);return Math.abs(h);}
function phaseColor(i){return rgbHex(...hsvToRgb(((i*67)%360)/360,0.65,0.95));}
function modeColor(name){const h=hashStr(name||'')%360;return rgbHex(...hsvToRgb(h/360,0.6,0.9));}

// Gradient: stops = [[pos,[r,g,b]],...], t in [0,1]
function gradColor(stops, t){
  if(t<=0)return rgbHex(...stops[0][1]);
  if(t>=1)return rgbHex(...stops[stops.length-1][1]);
  for(let i=0;i<stops.length-1;i++){
    if(t>=stops[i][0]&&t<=stops[i+1][0]){
      const f=(t-stops[i][0])/(stops[i+1][0]-stops[i][0]);
      return rgbHex(
        stops[i][1][0]+(stops[i+1][1][0]-stops[i][1][0])*f,
        stops[i][1][1]+(stops[i+1][1][1]-stops[i][1][1])*f,
        stops[i][1][2]+(stops[i+1][1][2]-stops[i][1][2])*f);
    }
  }
  return rgbHex(...stops[stops.length-1][1]);
}
const ALT_STOPS=[[0,[40,20,120]],[0.2,[40,90,200]],[0.4,[40,180,170]],[0.6,[120,200,80]],[0.8,[230,200,50]],[1,[220,70,40]]];
const SPD_STOPS=[[0,[60,40,140]],[0.25,[60,120,220]],[0.5,[80,200,120]],[0.75,[240,200,60]],[1,[220,70,40]]];
const G_STOPS=[[0,[60,140,80]],[0.3,[120,200,60]],[0.6,[240,200,40]],[1,[220,50,40]]];

// ---------------------------------------------------------------------------
// Trace helpers
// ---------------------------------------------------------------------------
function traceDuration(tr){return tr.duration_s||(tr.frames.length?tr.frames[tr.frames.length-1].t:0);}
function traceStatus(tr){
  if(!tr.phases||!tr.phases.length)return 'pass';
  let p=0,f=0,s=0;
  for(const ph of tr.phases){if(ph.skipped)s++;else if(ph.passed)p++;else f++;}
  if(f>0&&p>0)return 'mixed';
  if(f>0)return 'fail';
  if(p>0)return 'pass';
  return 'skip';
}
function tracePassedCount(tr){return(tr.phases||[]).filter(p=>p.passed).length;}
function traceTotalCount(tr){return(tr.phases||[]).length;}

// Compute bounds with padding
function computeBounds(tr){
  let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
  let minAlt=Infinity,maxAlt=-Infinity,minSpd=Infinity,maxSpd=-Infinity,minG=Infinity,maxG=-Infinity;
  let minT=Infinity,maxT=-Infinity;
  for(const f of tr.frames){
    if(f.x<minX)minX=f.x; if(f.x>maxX)maxX=f.x;
    if(f.y<minY)minY=f.y; if(f.y>maxY)maxY=f.y;
    const alt=-f.z;
    if(alt<minAlt)minAlt=alt; if(alt>maxAlt)maxAlt=alt;
    if(f.vcas<minSpd)minSpd=f.vcas; if(f.vcas>maxSpd)maxSpd=f.vcas;
    if(f.nzcgs<minG)minG=f.nzcgs; if(f.nzcgs>maxG)maxG=f.nzcgs;
    if(f.t<minT)minT=f.t; if(f.t>maxT)maxT=f.t;
    if(f.threats){for(const t of f.threats){
      if(t.x<minX)minX=t.x; if(t.x>maxX)maxX=t.x;
      if(t.y<minY)minY=t.y; if(t.y>maxY)maxY=t.y;
    }}
  }
  if(!isFinite(minX)){minX=maxX=minY=maxY=0;minAlt=maxAlt=0;minSpd=maxSpd=0;minG=maxG=0;minT=maxT=0;}
  const padX=(maxX-minX)*0.08+200, padY=(maxY-minY)*0.08+200;
  const padAlt=(maxAlt-minAlt)*0.1+200;
  minX-=padX;maxX+=padX;minY-=padY;maxY+=padY;
  minAlt=Math.max(0,minAlt-padAlt);maxAlt+=padAlt;
  if(maxT<=minT)maxT=minT+1;
  if(maxSpd<=minSpd)maxSpd=minSpd+1;
  if(maxG<=minG)maxG=minG+1;
  return{minX,maxX,minY,maxY,minAlt,maxAlt,minSpd,maxSpd,minG,maxG,minT,maxT};
}

// Phase ranges: [startFrame, endFrame] per phase (frames split at boundaries)
function phaseRanges(tr){
  if(!tr.phases||!tr.phases.length){
    return tr.frames.length?[[0,tr.frames.length-1]]:[];
  }
  const ranges=[];
  let fi=0;
  for(const p of tr.phases){
    const start=fi;
    while(fi<tr.frames.length&&tr.frames[fi].t<=p.end_s)fi++;
    let end=fi-1; if(end<start)end=start;
    ranges.push([start,end]);
  }
  return ranges;
}
function hasGroundOps(tr){
  for(const p of tr.phases||[]){if(/takeoff|landing|groundops/i.test(p.name))return true;}
  for(const f of tr.frames){if(f.mode==='Takeoff'||f.mode==='Landing')return true;}
  return false;
}
// Compute segments — groups of phases that form a continuous flight path.
// A new segment starts when a phase's `reinitializes` flag is true (the
// runner detected that fm.init() was called, resetting body rates). This is
// the most reliable discontinuity detection — position jumps alone miss
// cases where the aircraft is re-initialized to nearly the same position.
function computeSegments(tr){
  const ranges=phaseRanges(tr);
  if(ranges.length<=1)return [[0]];
  const segs=[[0]];
  for(let i=1;i<ranges.length;i++){
    const phase=tr.phases[i];
    // A phase that reinitializes starts a new segment.
    if(phase&&phase.reinitializes){
      segs.push([i]);
    }else{
      segs[segs.length-1].push(i);
    }
  }
  return segs;
}
// Get the last valid frame index in a range
function rangeLastFrame(tr,range){
  const [s,e]=range;
  for(let i=Math.min(e,tr.frames.length-1);i>=s;i--){
    return i;
  }
  return -1;
}
// Compute bounds for a single segment (only frames in that segment's phases)
function computeSegmentBounds(tr,segPhaseIndices){
  let b={minX:Infinity,maxX:-Infinity,minY:Infinity,maxY:-Infinity,
    minAlt:Infinity,maxAlt:-Infinity,minSpd:Infinity,maxSpd:-Infinity,
    minG:Infinity,maxG:-Infinity,minT:Infinity,maxT:-Infinity};
  const ranges=phaseRanges(tr);
  for(const pi of segPhaseIndices){
    if(pi>=ranges.length)continue;
    const [s,e]=ranges[pi];
    for(let fi=s;fi<=e&&fi<tr.frames.length;fi++){
      const f=tr.frames[fi];
      if(f.x<b.minX)b.minX=f.x; if(f.x>b.maxX)b.maxX=f.x;
      if(f.y<b.minY)b.minY=f.y; if(f.y>b.maxY)b.maxY=f.y;
      const alt=-f.z;
      if(alt<b.minAlt)b.minAlt=alt; if(alt>b.maxAlt)b.maxAlt=alt;
      if(f.vcas<b.minSpd)b.minSpd=f.vcas; if(f.vcas>b.maxSpd)b.maxSpd=f.vcas;
      if(f.nzcgs<b.minG)b.minG=f.nzcgs; if(f.nzcgs>b.maxG)b.maxG=f.nzcgs;
      if(f.t<b.minT)b.minT=f.t; if(f.t>b.maxT)b.maxT=f.t;
      if(f.threats){for(const t of f.threats){
        if(t.x<b.minX)b.minX=t.x; if(t.x>b.maxX)b.maxX=t.x;
        if(t.y<b.minY)b.minY=t.y; if(t.y>b.maxY)b.maxY=t.y;
      }}
    }
  }
  if(!isFinite(b.minX)){b.minX=b.maxX=b.minY=b.maxY=0;b.minAlt=b.maxAlt=0;b.minSpd=b.maxSpd=0;b.minG=b.maxG=0;b.minT=b.maxT=0;}
  // Include waypoint positions for this segment
  if(tr.waypoints){
    for(const w of tr.waypoints){
      if(w.x<b.minX)b.minX=w.x; if(w.x>b.maxX)b.maxX=w.x;
      if(w.y<b.minY)b.minY=w.y; if(w.y>b.maxY)b.maxY=w.y;
    }
  }
  // Include scene line endpoints
  if(tr.sceneLines){
    for(const sl of tr.sceneLines){
      b.minX=Math.min(b.minX,sl.x1,sl.x2); b.maxX=Math.max(b.maxX,sl.x1,sl.x2);
      b.minY=Math.min(b.minY,sl.y1,sl.y2); b.maxY=Math.max(b.maxY,sl.y1,sl.y2);
    }
  }
  const padX=(b.maxX-b.minX)*0.08+200, padY=(b.maxY-b.minY)*0.08+200;
  const padAlt=(b.maxAlt-b.minAlt)*0.1+200;
  b.minX-=padX;b.maxX+=padX;b.minY-=padY;b.maxY+=padY;
  b.minAlt=Math.max(0,b.minAlt-padAlt);b.maxAlt+=padAlt;
  if(b.maxT<=b.minT)b.maxT=b.minT+1;
  if(b.maxSpd<=b.minSpd)b.maxSpd=b.minSpd+1;
  if(b.maxG<=b.minG)b.maxG=b.minG+1;
  return equalizeAspect(b);
}
// Equalize the X/Y aspect ratio of bounds so the smaller axis is at least
// `minRatio` of the larger. Prevents the plot and grid from collapsing to a
// line when the aircraft flies straight (e.g. a 60 NM north leg with only
// 200 ft of X drift). Mutates and returns `b`.
function equalizeAspect(b, minRatio){
  minRatio = minRatio || 0.2;
  const spanX=b.maxX-b.minX, spanY=b.maxY-b.minY;
  const spanMax=Math.max(spanX,spanY,1);
  const minSpan=spanMax*minRatio;
  if(spanX<minSpan){
    const cx=(b.minX+b.maxX)/2;
    b.minX=cx-minSpan/2; b.maxX=cx+minSpan/2;
  }
  if(spanY<minSpan){
    const cy=(b.minY+b.maxY)/2;
    b.minY=cy-minSpan/2; b.maxY=cy+minSpan/2;
  }
  return b;
}

// Color for a frame given the active color-by mode
function frameColor(tr,frameIdx,bounds){
  const f=tr.frames[frameIdx]; if(!f)return '#888';
  switch(state.colorBy){
    case 'phase':{
      // find phase index for this frame
      const ranges=phaseRanges(tr);
      for(let i=0;i<ranges.length;i++){if(frameIdx>=ranges[i][0]&&frameIdx<=ranges[i][1])return phaseColor(i);}
      return '#888';
    }
    case 'passfail':{
      const ranges=phaseRanges(tr);
      for(let i=0;i<ranges.length;i++){
        if(frameIdx>=ranges[i][0]&&frameIdx<=ranges[i][1]){
          const ph=tr.phases[i];
          if(!ph)return '#888';
          if(ph.skipped)return '#94a3b8';
          return ph.passed?'#4ade80':'#f87171';
        }
      }
      return '#888';
    }
    case 'altitude':{
      const t=(-f.z-bounds.minAlt)/(bounds.maxAlt-bounds.minAlt);
      return gradColor(ALT_STOPS,Math.max(0,Math.min(1,t)));
    }
    case 'speed':{
      const t=(f.vcas-bounds.minSpd)/(bounds.maxSpd-bounds.minSpd);
      return gradColor(SPD_STOPS,Math.max(0,Math.min(1,t)));
    }
    case 'g':{
      const t=(f.nzcgs-0)/Math.max(1,bounds.maxG);
      return gradColor(G_STOPS,Math.max(0,Math.min(1,t)));
    }
    case 'mode':return modeColor(f.mode||'');
    default:return '#888';
  }
}

// ---------------------------------------------------------------------------
// SVG helpers
// ---------------------------------------------------------------------------
function el(name,attrs,content){
  let s='<'+name;
  for(const k in attrs){if(attrs[k]!==undefined&&attrs[k]!==null)s+=' '+k+'="'+attrs[k]+'"';}
  if(content!==undefined){s+='>'+content+'</'+name+'>';}else{s+='/>';}
  return s;
}
function fmt(n,d){d=d===undefined?0:d;if(!isFinite(n))return'—';return n.toFixed(d);}
function fmtInt(n){if(!isFinite(n))return'—';return Math.round(n).toString();}

// ---------------------------------------------------------------------------
// Mini top-down thumbnail (for index cards)
// ---------------------------------------------------------------------------
function miniTopdown(tr){
  if(!tr.frames.length)return '<svg viewBox="0 0 200 120" width="200" height="120"></svg>';
  const b=equalizeAspect(computeBounds(tr));
  const W=200,H=120,pad=10;
  const span=Math.max(b.maxX-b.minX,b.maxY-b.minY,1);
  const scale=Math.min((W-2*pad)/span,(H-2*pad)/span);
  const ox=(W-span*scale)/2, oy=(H-span*scale)/2;
  const sx=x=>ox+(x-b.minX)*scale;
  const sy=y=>oy+(b.maxY-y)*scale;
  // Scene lines (runway etc.) — drawn under the path
  let scene='';
  if(tr.sceneLines){
    for(const sl of tr.sceneLines){
      const col=sl.color||'#FFD700';
      const w=sl.width?Math.max(0.5,sl.width*scale*0.5):1;
      scene+=el('line',{x1:sx(sl.x1),y1:sy(sl.y1),x2:sx(sl.x2),y2:sy(sl.y2),
        stroke:col,'stroke-width':w.toFixed(1),opacity:0.6});
    }
  }
  const ranges=phaseRanges(tr);
  let paths='';
  for(let i=0;i<ranges.length;i++){
    const [s,e]=ranges[i]; if(e<=s)continue;
    let pts='';
    for(let fi=s;fi<=e&&fi<tr.frames.length;fi++){
      const f=tr.frames[fi];
      pts+=(pts?' ':'')+sx(f.x).toFixed(1)+','+sy(f.y).toFixed(1);
    }
    const col=phaseColor(i);
    paths+=el('polyline',{points:pts,fill:'none',stroke:col,'stroke-width':1.5,'stroke-linejoin':'round','stroke-linecap':'round',opacity:0.9});
  }
  // Waypoint markers
  let wps='';
  if(tr.waypoints){
    for(let i=0;i<tr.waypoints.length;i++){
      const w=tr.waypoints[i];
      wps+=el('polygon',{points: diamondPts(sx(w.x),sy(w.y),4),fill:'#fff',stroke:'#000','stroke-width':.5,opacity:0.8});
    }
  }
  return '<svg viewBox="0 0 '+W+' '+H+'" width="'+W+'" height="'+H+'">'+
    el('rect',{x:0,y:0,width:W,height:H,fill:'#0f1117'})+scene+paths+wps+'</svg>';
}
// Helper: diamond polygon points
function diamondPts(cx,cy,r){
  return cx+','+(cy-r)+' '+(cx+r)+','+cy+' '+cx+','+(cy+r)+' '+(cx-r)+','+cy;
}
)HTML";
static const char* kHtmlTail1b = R"HTML(
// Helper: build SVG for scene lines (shared by top-down and 3D)
function sceneLinesSvg(tr,sx,sy,scale){
  if(!tr.sceneLines||!tr.sceneLines.length)return '';
  let svg='<g class="scene-lines">';
  for(const sl of tr.sceneLines){
    const col=sl.color||'#3a3a4a';
    // Width: use the scene line's specified width (in ft), scaled to pixels.
    // Ensure a minimum of 4px so the runway is always visible even when
    // zoomed way out (e.g. a 10000ft runway on a 50000ft plot).
    const w=sl.width?Math.max(4,sl.width*scale):4;
    svg+=el('line',{x1:sx(sl.x1),y1:sy(sl.y1),x2:sx(sl.x2),y2:sy(sl.y2),
      stroke:col,'stroke-width':w.toFixed(1),opacity:0.8});
    // Label at midpoint
    const mx=(sx(sl.x1)+sx(sl.x2))/2, my=(sy(sl.y1)+sy(sl.y2))/2;
    if(sl.label)svg+=el('text',{x:mx,y:my-4,fill:col,'font-size':9,'text-anchor':'middle',opacity:0.8},esc(sl.label));
  }
  return svg+'</g>';
}
// Helper: build SVG for waypoint markers
function waypointsSvg(tr,sx,sy){
  if(!tr.waypoints||!tr.waypoints.length)return '';
  let svg='<g class="waypoints">';
  for(let i=0;i<tr.waypoints.length;i++){
    const w=tr.waypoints[i];
    const x=sx(w.x), y=sy(w.y);
    svg+=el('polygon',{points:diamondPts(x,y,7),fill:'#fff',stroke:'#000','stroke-width':1.5});
    svg+=el('text',{x:x+10,y:y-8,fill:'#fff','font-weight':'bold','font-size':11},esc(w.name||('WP'+(i+1))));
  }
  return svg+'</g>';
}
// Helper: build SVG for threat tracks + bearing lines at current time
function threatsSvg(tr,sx,sy,currentT){
  if(!tr.frames)return '';
  // Collect unique missile tracks (type=missile moves; draw full trail)
  // and static bearing lines (type=guns/target; draw line from aircraft to threat at currentT)
  const missileTracks={}; // key=type -> [{x,y,t}]
  const fi=findFrameIndex(tr,currentT);
  const acFrame=tr.frames[fi];
  let bearingLines='';
  for(let i=0;i<tr.frames.length;i++){
    const f=tr.frames[i];
    if(!f.threats)continue;
    for(const th of f.threats){
      if(th.type==='missile'){
        if(!missileTracks['missile'])missileTracks['missile']=[];
        missileTracks['missile'].push({x:th.x,y:th.y,t:f.t});
      }
    }
  }
  let svg='<g class="threats">';
  // Missile trail
  for(const key in missileTracks){
    const track=missileTracks[key];
    if(track.length<2)continue;
    let pts='';
    for(const p of track){pts+=(pts?' ':'')+sx(p.x).toFixed(1)+','+sy(p.y).toFixed(1);}
    svg+=el('polyline',{points:pts,fill:'none',stroke:'#f44336','stroke-width':1.5,
      'stroke-dasharray':'4,3',opacity:0.5});
    // Current missile position
    const cur=track[Math.min(track.length-1,fi)];
    if(cur){
      svg+=el('circle',{cx:sx(cur.x),cy:sy(cur.y),r:5,fill:'#f44336',stroke:'#fff','stroke-width':1,opacity:0.9});
      svg+=el('text',{x:sx(cur.x)+7,y:sy(cur.y)+4,fill:'#f44336','font-size':10},'missile');
    }
  }
  // Bearing lines to guns threats / targets at current frame
  if(acFrame&&acFrame.threats){
    for(const th of acFrame.threats){
      if(th.type==='guns'){
        svg+=el('line',{x1:sx(acFrame.x),y1:sy(acFrame.y),x2:sx(th.x),y2:sy(th.y),
          stroke:'#ff9800','stroke-width':1,'stroke-dasharray':'2,2',opacity:0.7});
        svg+=el('circle',{cx:sx(th.x),cy:sy(th.y),r:5,fill:'#ff9800',opacity:0.6,stroke:'#ff9800','stroke-width':1});
        svg+=el('text',{x:sx(th.x)+7,y:sy(th.y)+4,fill:'#ff9800','font-size':10},'guns');
      }else if(th.type==='target'){
        svg+=el('line',{x1:sx(acFrame.x),y1:sy(acFrame.y),x2:sx(th.x),y2:sy(th.y),
          stroke:'#9c27b0','stroke-width':1,'stroke-dasharray':'2,2',opacity:0.7});
        svg+=el('rect',{x:sx(th.x)-5,y:sy(th.y)-5,width:10,height:10,fill:'#9c27b0',opacity:0.6,stroke:'#9c27b0'});
        svg+=el('text',{x:sx(th.x)+7,y:sy(th.y)+4,fill:'#9c27b0','font-size':10},'target');
      }
    }
  }
  return svg+'</g>';
}
)HTML";
static const char* kHtmlTail2 = R"HTML(
// ---------------------------------------------------------------------------
// Index view
// ---------------------------------------------------------------------------
function renderIndex(){
  const summary=document.getElementById('index-summary');
  const total=TRACES.length;
  let passN=0,failN=0,mixedN=0,phaseP=0,phaseT=0;
  for(const tr of TRACES){
    const st=traceStatus(tr);
    if(st==='pass')passN++;else if(st==='fail')failN++;else if(st==='mixed')mixedN++;
    phaseP+=tracePassedCount(tr);phaseT+=traceTotalCount(tr);
  }
  summary.innerHTML=
    statCard(total,'Traces')+
    statCard(passN,'Passing')+
    statCard(failN,'Failing')+
    statCard(mixedN,'Mixed')+
    statCard(phaseP+'/'+phaseT,'Phases passed');

  const grid=document.getElementById('card-grid');
  const filt=state.filter.toLowerCase();
  const filtStatus=state.filterStatus;
  let html='';
  let shown=0;
  for(let i=0;i<TRACES.length;i++){
    const tr=TRACES[i];
    const st=traceStatus(tr);
    if(filtStatus&&st!==filtStatus&&!(filtStatus==='pass'&&st==='pass'))continue;
    if(filt){
      const hay=(tr.aircraft+' '+tr.scenario+' '+(tr.phases||[]).map(p=>p.name).join(' ')).toLowerCase();
      if(hay.indexOf(filt)<0)continue;
    }
    shown++;
    const badgeCls=st==='pass'?'pass':st==='fail'?'fail':st==='mixed'?'mixed':'skip';
    const badgeTxt=st==='pass'?'PASS':st==='fail'?'FAIL':st==='mixed'?'MIXED':'SKIP';
    html+='<div class="card" data-idx="'+i+'">'+
      '<div class="thumb">'+miniTopdown(tr)+'</div>'+
      '<div class="title">'+esc(tr.aircraft)+' &mdash; '+esc(tr.scenario)+'</div>'+
      '<div class="meta">'+tracePassedCount(tr)+'/'+traceTotalCount(tr)+' phases &middot; '+
        fmt(traceDuration(tr),0)+'s &middot; '+tr.frames.length+' frames</div>'+
      '<div style="margin-top:6px"><span class="badge '+badgeCls+'">'+badgeTxt+'</span></div>'+
      '</div>';
  }
  if(!shown)html='<div class="empty">No traces match the filter.</div>';
  grid.innerHTML=html;
  grid.querySelectorAll('.card').forEach(c=>{
    c.addEventListener('click',()=>{location.hash='#trace/'+c.dataset.idx;});
  });
}
function statCard(v,l){return '<div class="stat"><div class="v">'+v+'</div><div class="l">'+l+'</div></div>';}
function esc(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}

// ---------------------------------------------------------------------------
// Detail view: top-down flight path
// ---------------------------------------------------------------------------
let detailBounds=null, detailScales=null, detail3DProject=null;

function renderTopDown(tr){
  const host=document.getElementById('topdown-host');
  // Use segment-specific bounds so discontinuous phases aren't overlaid
  const segs=computeSegments(tr);
  const si=Math.min(state.segmentIdx,segs.length-1);
  const segPhases=segs[si];
  const b=computeSegmentBounds(tr,segPhases);
  detailBounds=b;
  // Dynamic height: size the SVG to the actual plot aspect ratio, not a
  // fixed 620 px. This avoids large empty areas when the path is narrow.
  const W=1000,padL=30,padR=20,padT=20,padB=20;
  const plotW=W-padL-padR;
  const spanX=b.maxX-b.minX, spanY=b.maxY-b.minY;
  const scBase=Math.min(plotW/spanX,plotW/spanY);
  const sc=scBase*state.zoom;
  const plotH=spanY*sc;
  const H=padT+plotH+padB;
  const ox=padL+state.panX+(plotW-spanX*sc)/2;
  const oy=padT+state.panY;
  const sx=x=>ox+(x-b.minX)*sc;
  const sy=y=>oy+(b.maxY-y)*sc;
  detailScales={sx,sy,ox,oy,padL,padT,plotW,plotH:Math.max(plotH,100),W,H};

  let svg='<svg viewBox="0 0 '+W+' '+H+'" preserveAspectRatio="xMidYMid meet">';
  // Grid (no axis labels — they overlapped on narrow plots)
  const spanTD=Math.max(spanX,spanY);
  let gs=10000;
  if(spanTD<3000)gs=500;else if(spanTD<10000)gs=1000;else if(spanTD<50000)gs=5000;else gs=10000;
  svg+='<g>';
  for(let gx=Math.ceil(b.minX/gs)*gs;gx<=b.maxX;gx+=gs){
    svg+=el('line',{x1:sx(gx),y1:oy,x2:sx(gx),y2:oy+spanY*sc,class:'grid-line'});
  }
  for(let gy=Math.ceil(b.minY/gs)*gs;gy<=b.maxY;gy+=gs){
    svg+=el('line',{x1:ox,y1:sy(gy),x2:ox+spanX*sc,y2:sy(gy),class:'grid-line'});
  }
  svg+=el('rect',{x:ox,y:oy,width:spanX*sc,height:spanY*sc,fill:'none',stroke:'var(--border)','stroke-width':1});
  svg+='</g>';

  // Scene lines + waypoints
  svg+=sceneLinesSvg(tr,sx,sy,sc);
  svg+=waypointsSvg(tr,sx,sy);

  // Paths — only draw phases in the current segment
  const ranges=phaseRanges(tr);
  if(state.colorBy==='altitude'||state.colorBy==='speed'||state.colorBy==='g'||state.colorBy==='mode'){
    for(const pi of segPhases){
      if(pi>=ranges.length)continue;
      const [s,e]=ranges[pi];
      for(let fi=s;fi<e&&fi<tr.frames.length-1;fi++){
        const f1=tr.frames[fi], f2=tr.frames[fi+1];
        const col=frameColor(tr,fi,b);
        svg+=el('line',{x1:sx(f1.x),y1:sy(f1.y),x2:sx(f2.x),y2:sy(f2.y),
          stroke:col,'stroke-width':2,'stroke-linecap':'round',opacity:0.9});
      }
    }
  }else{
    for(const pi of segPhases){
      if(pi>=ranges.length)continue;
      const [s,e]=ranges[pi]; if(e<=s)continue;
      let pts='';
      for(let fi=s;fi<=e&&fi<tr.frames.length;fi++){
        const f=tr.frames[fi];
        pts+=(pts?' ':'')+sx(f.x).toFixed(1)+','+sy(f.y).toFixed(1);
      }
      let col;
      if(state.colorBy==='phase')col=phaseColor(pi);
      else{ const ph=tr.phases[pi]; col=ph?(ph.skipped?'#94a3b8':ph.passed?'#4ade80':'#f87171'):'#888'; }
      svg+=el('polyline',{points:pts,fill:'none',stroke:col,'stroke-width':2.5,
        'stroke-linejoin':'round','stroke-linecap':'round',opacity:0.9});
    }
  }

  // Phase start markers
  for(const pi of segPhases){
    if(pi>=ranges.length)continue;
    const [s,e]=ranges[pi]; if(s>=tr.frames.length)continue;
    const f=tr.frames[s];
    const col=state.colorBy==='passfail'?(tr.phases[pi]?(tr.phases[pi].skipped?'#94a3b8':tr.phases[pi].passed?'#4ade80':'#f87171'):'#888'):phaseColor(pi);
    svg+=el('circle',{cx:sx(f.x),cy:sy(f.y),r:2,fill:col,stroke:'#fff','stroke-width':1});
    svg+=el('text',{x:sx(f.x)+10,y:sy(f.y)-8,fill:'#fff','font-weight':'bold','font-size':11},
      esc((pi+1)+'. '+(tr.phases[pi]?tr.phases[pi].name:'P'+(pi+1))));
  }

  // Threat tracks
  svg+=threatsSvg(tr,sx,sy,state.t);

  // Playhead group
  svg+='<g id="td-playhead"></g>';

  // Zoom indicator
  if(state.zoom!==1.0){
    svg+=el('text',{x:W-padR,y:14,fill:'var(--muted)','font-size':11,'text-anchor':'end'},
      'zoom '+state.zoom.toFixed(1)+'x');
  }

  svg+='</svg>';
  host.innerHTML=svg;
}

// ---------------------------------------------------------------------------
// Detail view: time series (altitude / speed / G)
// ---------------------------------------------------------------------------
function renderTimeSeries(tr){
  const host=document.getElementById('timeseries-host');
  const b=computeBounds(tr);
  const W=1000,H=560,padL=60,padR=40,padT=20,padB=36;
  const plotW=W-padL-padR;
  const panelH=(H-padT-padB)/3;
  const dur=traceDuration(tr);
  const sx=t=>padL+(t/dur)*plotW;
  // Each panel: [field, lo, hi, unit, color, fmtFn]
  const panels=[
    {name:'Altitude',get:f=>-f.z,lo:b.minAlt,hi:b.maxAlt,unit:'ft',col:'#4ade80'},
    {name:'Speed (VCAS)',get:f=>f.vcas,lo:b.minSpd,hi:b.maxSpd,unit:'kts',col:'#60a5fa'},
    {name:'G-load',get:f=>f.nzcgs,lo:0,hi:Math.max(2,b.maxG),unit:'G',col:'#f59e0b'},
  ];
  let svg='<svg viewBox="0 0 '+W+' '+H+'" preserveAspectRatio="xMidYMid meet">';
  // Time grid spacing
  let ts=10;
  if(dur>300)ts=60;else if(dur>120)ts=30;else if(dur>60)ts=15;else if(dur>30)ts=10;else ts=5;

  for(let pi=0;pi<panels.length;pi++){
    const p=panels[pi];
    const top=padT+pi*panelH;
    const bottom=top+panelH-16; // leave gap
    const ph=bottom-top;
    const sy=v=>top+(p.hi-v)/(p.hi-p.lo)*ph;
    // Panel label
    svg+=el('text',{x:padL,y:top-4,class:'axis-text','font-weight':'bold'},p.name+' ('+p.unit+')');
    // Grid + Y labels
    let vs=p.lo;
    const vrange=p.hi-p.lo;
    let vstep=vrange/4;
    // nice step
    const mag=Math.pow(10,Math.floor(Math.log10(vstep)));
    vstep=Math.ceil(vstep/mag)*mag;
    for(let v=Math.ceil(p.lo/vstep)*vstep;v<=p.hi;v+=vstep){
      svg+=el('line',{x1:padL,y1:sy(v),x2:padL+plotW,y2:sy(v),class:'grid-line'});
      svg+=el('text',{x:padL-5,y:sy(v)+3,class:'axis-text','text-anchor':'end'},fmtInt(v));
    }
    // Phase bands (colored by pass/fail)
    if(tr.phases&&tr.phases.length){
      for(let i=0;i<tr.phases.length;i++){
        const ph=tr.phases[i];
        const x1=sx(ph.start_s), x2=sx(ph.end_s);
        const col=ph.skipped?'rgba(148,163,184,0.07)':ph.passed?'rgba(74,222,128,0.07)':'rgba(248,113,113,0.12)';
        svg+=el('rect',{x:x1,y:top,width:Math.max(1,x2-x1),height:ph,fill:col});
        // phase number at top
        svg+=el('text',{x:(x1+x2)/2,y:bottom+12,class:'axis-text','text-anchor':'middle','font-size':9,fill:ph.passed?'var(--pass)':ph.skipped?'var(--skip)':'var(--fail)'},(i+1));
      }
    }
    // Data line (split by phase)
    const ranges=phaseRanges(tr);
    for(const [s,e] of ranges){
      if(e<=s)continue;
      let pts='';
      for(let fi=s;fi<=e&&fi<tr.frames.length;fi++){
        const f=tr.frames[fi];
        pts+=(pts?' ':'')+sx(f.t).toFixed(1)+','+sy(p.get(f)).toFixed(1);
      }
      svg+=el('polyline',{points:pts,fill:'none',stroke:p.col,'stroke-width':1.5,opacity:0.9});
    }
    // Border
    svg+=el('rect',{x:padL,y:top,width:plotW,height:ph,fill:'none',stroke:'var(--border)','stroke-width':1});
    // Playhead line (updated separately)
    svg+=el('line',{id:'ts-play-'+pi,x1:padL,y1:top,x2:padL,y2:bottom,stroke:'#fff','stroke-width':1,opacity:0.5});
  }
  // Time axis labels
  for(let t=Math.ceil(0/ts)*ts;t<=dur;t+=ts){
    svg+=el('text',{x:sx(t),y:H-6,class:'axis-text','text-anchor':'middle'},fmtInt(t)+'s');
  }
  svg+='</svg>';
  host.innerHTML=svg;
}
)HTML";
static const char* kHtmlTail3 = R"HTML(
// ---------------------------------------------------------------------------
// Playhead update (cheap — no full re-render)
// ---------------------------------------------------------------------------
function findFrameIndex(tr,t){
  // binary search for frame with time <= t
  let lo=0,hi=tr.frames.length-1,best=0;
  while(lo<=hi){
    const mid=(lo+hi)>>1;
    if(tr.frames[mid].t<=t){best=mid;lo=mid+1;}
    else hi=mid-1;
  }
  return best;
}
function updatePlayhead(){
  const tr=TRACES[state.traceIdx]; if(!tr||!tr.frames.length)return;
  if(state.t<0)state.t=0;
  const dur=traceDuration(tr);
  if(state.t>dur)state.t=dur;
  const fi=findFrameIndex(tr,state.t);
  const f=tr.frames[fi];
  // In 3D mode, re-render the view so it stays centered on the aircraft
  // as the playhead moves. (In 2D mode, only the playhead marker updates.)
  if(state.viewMode==='3d'){
    render3D(tr);
  }
  // 2D top-down playhead: aircraft icon
  // psi is body yaw in radians, measured counter-clockwise from +X (East).
  // (Confirmed empirically: psi=0 when flying east, psi=π/2 when flying north.)
  // The polygon points up (= North = +Y on screen). To align it with psi:
  //   psi=0 (East)   → rotate 90° clockwise (up→right)
  //   psi=π/2 (North)→ rotate 0° (stays up)
  // So SVG rotation = 90 - psi_deg.
  const td=document.getElementById('td-playhead');
  if(td&&detailScales){
    const x=detailScales.sx(f.x), y=detailScales.sy(f.y);
    const deg=90-f.psi*180/Math.PI;
    td.innerHTML=el('g',{transform:'translate('+x.toFixed(1)+','+y.toFixed(1)+') rotate('+deg.toFixed(1)+')'},
      el('polygon',{points:'0,-9 6,7 0,3 -6,7',fill:'#fff',stroke:'#000','stroke-width':1,opacity:0.95})+
      el('circle',{r:2,fill:'#000'}));
  }
  // 3D playhead: aircraft as a 3D pyramid (4 triangular faces) pointing in
  // the direction of travel (psi). Built from 5 points: nose, tail, left
  // wing, right wing, top. All projected through the 3D projection.
  const td3=document.getElementById('td3-playhead');
  if(td3&&detail3DProject&&detailBounds){
    const span3D=Math.max(detailBounds.maxX-detailBounds.minX,
                          detailBounds.maxY-detailBounds.minY,
                          detailBounds.maxAlt-detailBounds.minAlt,1);
    const size=span3D*0.01; // aircraft size relative to data span
    const cosP=Math.cos(f.psi), sinP=Math.sin(f.psi);
    // 5 points of the aircraft shape (in world coords, relative to ac pos):
    //   nose: +X (psi direction), at altitude
    //   tail: -X, slightly below
    //   lWing: -X, -Y perpendicular, at altitude
    //   rWing: -X, +Y perpendicular, at altitude
    //   top: center, above
    const noseX=f.x+cosP*size*2, noseY=f.y+sinP*size*2;
    const tailX=f.x-cosP*size*0.5, tailY=f.y-sinP*size*0.5;
    const lwx=f.x-cosP*size*0.5-sinP*size, lwy=f.y-sinP*size*0.5+cosP*size;
    const rwx=f.x-cosP*size*0.5+sinP*size, rwy=f.y-sinP*size*0.5-cosP*size;
    const topX=f.x-cosP*size*0.13, topY=f.y-sinP*size*0.13;
    const alt=-f.z;
    // Project all 5 points
    const pn=detail3DProject(noseX,noseY,alt);
    const pt=detail3DProject(tailX,tailY,alt-size*0.1);
    const plw=detail3DProject(lwx,lwy,alt);
    const prw=detail3DProject(rwx,rwy,alt);
    const ptop=detail3DProject(topX,topY,alt+size*0.4);
    // Draw 4 triangular faces (nose-tail-lwing, nose-tail-rwing, nose-lwing-top, nose-rwing-top)
    // Use different opacities to give a 3D shaded look
    let model='';
    // Bottom faces (darker)
    model+=el('polygon',{points:pn[0].toFixed(1)+','+pn[1].toFixed(1)+' '+pt[0].toFixed(1)+','+pt[1].toFixed(1)+' '+plw[0].toFixed(1)+','+plw[1].toFixed(1),fill:'#888',stroke:'#000','stroke-width':0.0,opacity:1.0});
    model+=el('polygon',{points:pn[0].toFixed(1)+','+pn[1].toFixed(1)+' '+pt[0].toFixed(1)+','+pt[1].toFixed(1)+' '+prw[0].toFixed(1)+','+prw[1].toFixed(1),fill:'#aaa',stroke:'#000','stroke-width':0.0,opacity:1.0});
    model+=el('polygon',{points:plw[0].toFixed(1)+','+plw[1].toFixed(1)+' '+pt[0].toFixed(1)+','+pt[1].toFixed(1)+' '+prw[0].toFixed(1)+','+prw[1].toFixed(1),fill:'#aaa',stroke:'#000','stroke-width':0.0,opacity:1.0});
    // Top faces (lighter)
    model+=el('polygon',{points:pn[0].toFixed(1)+','+pn[1].toFixed(1)+' '+plw[0].toFixed(1)+','+plw[1].toFixed(1)+' '+ptop[0].toFixed(1)+','+ptop[1].toFixed(1),fill:'#ddd',stroke:'#000','stroke-width':0.0,opacity:1.0});
    model+=el('polygon',{points:pn[0].toFixed(1)+','+pn[1].toFixed(1)+' '+prw[0].toFixed(1)+','+prw[1].toFixed(1)+' '+ptop[0].toFixed(1)+','+ptop[1].toFixed(1),fill:'#ccc',stroke:'#000','stroke-width':0.0,opacity:1.0});
    model+=el('polygon',{points:plw[0].toFixed(1)+','+plw[1].toFixed(1)+' '+ptop[0].toFixed(1)+','+ptop[1].toFixed(1)+' '+prw[0].toFixed(1)+','+prw[1].toFixed(1),fill:'#ccc',stroke:'#000','stroke-width':0.0,opacity:1.0});
    td3.innerHTML=model;
  }
  // Time-series playhead lines
  for(let pi=0;pi<3;pi++){
    const ln=document.getElementById('ts-play-'+pi);
    if(ln){
      const x=detailScales?detailScales.padL+(state.t/dur)*detailScales.plotW:0;
      ln.setAttribute('x1',x);ln.setAttribute('x2',x);
    }
  }
  // Scrubber
  const scr=document.getElementById('scrubber');
  if(scr){scr.value=Math.round((state.t/dur)*1000);}
  document.getElementById('time-now').textContent=fmt(state.t,1)+'s';
  // Frame readout
  const ro=document.getElementById('readout');
  const alt=-f.z;
  const phaseIdx=phaseRanges(tr).findIndex(([s,e])=>fi>=s&&fi<=e);
  const phaseName=phaseIdx>=0&&tr.phases[phaseIdx]?tr.phases[phaseIdx].name:'—';
  ro.innerHTML=
    cell('Time',fmt(f.t,1)+'s')+
    cell('Altitude',fmtInt(alt)+' ft')+
    cell('VCAS',fmt(f.vcas,0)+' kts')+
    cell('G-load',fmt(f.nzcgs,2)+' G')+
    cell('Bank',fmt(f.phi*180/Math.PI,0)+'\u00b0')+
    cell('Pitch',fmt(f.theta*180/Math.PI,0)+'\u00b0')+
    cell('Heading',fmt(f.psi*180/Math.PI,0)+'\u00b0')+
    cell('Throttle',fmt(f.throttle,2))+
    cell('AI Mode',esc(f.mode||'—'),'mode')+
    cell('Phase',esc(phaseName),'phase');
}
function cell(l,v,cls){return '<div class="cell '+(cls||'')+'"><div class="l">'+l+'</div><div class="v">'+v+'</div></div>';}

// ---------------------------------------------------------------------------
// Path view dispatcher (2D or 3D based on state.viewMode)
// ---------------------------------------------------------------------------
function renderPathView(tr){
  const tdHost=document.getElementById('topdown-host');
  const td3Host=document.getElementById('threed-host');
  const title=document.getElementById('path-panel-title');
  if(state.viewMode==='3d'){
    tdHost.classList.add('hidden');
    td3Host.classList.remove('hidden');
    title.textContent='3D Flight Path (drag to orbit)';
    render3D(tr);
  }else{
    td3Host.classList.add('hidden');
    tdHost.classList.remove('hidden');
    title.textContent='Top-Down Flight Path';
    renderTopDown(tr);
  }
}

)HTML";
static const char* kHtmlTail3b = R"HTML(
// ---------------------------------------------------------------------------
// 3D orbit view — projects the 3D flight path (X, Y, altitude) to 2D SVG
// using yaw/pitch rotation + perspective. Mouse drag orbits the camera.
// No WebGL/Three.js needed — pure SVG projection.
// ---------------------------------------------------------------------------
function render3D(tr){
  const host=document.getElementById('threed-host');
  // Segment-specific bounds
  const segs=computeSegments(tr);
  const si=Math.min(state.segmentIdx,segs.length-1);
  const segPhases=segs[si];
  const b=computeSegmentBounds(tr,segPhases);
  detailBounds=b;
  const W=1000,H=620;
  // Center the 3D view on the current aircraft position (at the playhead
  // time) so the aircraft stays in view as it moves. Fall back to the data
  // centroid if no frames exist.
  const dur=traceDuration(tr);
  const acFi=findFrameIndex(tr,Math.min(state.t,dur));
  const acFrame=tr.frames[acFi];
  const cx=acFrame?acFrame.x:(b.minX+b.maxX)/2;
  const cy=acFrame?acFrame.y:(b.minY+b.maxY)/2;
  const cz=acFrame?-acFrame.z:(b.minAlt+b.maxAlt)/2;
  const span=Math.max(b.maxX-b.minX,b.maxY-b.minY,b.maxAlt-b.minAlt,1);
  const scale=(W*0.35)/span*state.zoom;
  const yaw=state.orbit.yaw, pitch=state.orbit.pitch;
  // 3D projection centered on the aircraft
  function project(x,y,z){
    let px=x-cx, py=y-cy, pz=z-cz;
    const xr=px*Math.cos(yaw)-py*Math.sin(yaw);
    const yr=px*Math.sin(yaw)+py*Math.cos(yaw);
    const yr2=yr*Math.cos(pitch)-pz*Math.sin(pitch);
    const zr=yr*Math.sin(pitch)+pz*Math.cos(pitch);
    const d=span*3;
    const f=d/(d+zr);
    return [W/2+xr*f*scale+state.panX, H/2-yr2*f*scale+state.panY, zr];
  }
  let svg='<svg viewBox="0 0 '+W+' '+H+'" preserveAspectRatio="xMidYMid meet">';
  svg+=el('rect',{x:0,y:0,width:W,height:H,fill:'#0a0c12'});
  // Ground grid — centered on aircraft, extends ±span/2
  const gs=span>50000?10000:span>10000?5000:1000;
  const gridR=span; // grid extends this far from center
  svg+='<g opacity="0.3">';
  for(let gx=Math.ceil((cx-gridR)/gs)*gs;gx<=cx+gridR;gx+=gs){
    const p1=project(gx,cy-gridR,0), p2=project(gx,cy+gridR,0);
    svg+=el('line',{x1:p1[0],y1:p1[1],x2:p2[0],y2:p2[1],stroke:'#2a2f42','stroke-width':.5});
  }
  for(let gy=Math.ceil((cy-gridR)/gs)*gs;gy<=cy+gridR;gy+=gs){
    const p1=project(cx-gridR,gy,0), p2=project(cx+gridR,gy,0);
    svg+=el('line',{x1:p1[0],y1:p1[1],x2:p2[0],y2:p2[1],stroke:'#2a2f42','stroke-width':.5});
  }
  svg+='</g>';
  // Scene lines
  if(tr.sceneLines){
    for(const sl of tr.sceneLines){
      const col=sl.color||'#3a3a4a';
      const p1=project(sl.x1,sl.y1,-sl.z1), p2=project(sl.x2,sl.y2,-sl.z2);
      svg+=el('line',{x1:p1[0],y1:p1[1],x2:p2[0],y2:p2[1],stroke:col,'stroke-width':3,opacity:0.7});
    }
  }
  // Flight path — drawn as per-segment colored lines (like the 2D view) but
  // projected to 3D. We subsample frames to keep the SVG manageable (max ~500
  // segments per phase) and ensure each segment is at least 2px on screen.
  const ranges=phaseRanges(tr);
  for(const pi of segPhases){
    if(pi>=ranges.length)continue;
    const [s,e]=ranges[pi]; if(e-s<2)continue;
    // Subsample: take every Nth frame, targeting ~300 segments
    const total=e-s;
    const stride=Math.max(1,Math.floor(total/300));
    let prevP=null;
    for(let fi=s;fi<=e&&fi<tr.frames.length;fi+=stride){
      const f=tr.frames[fi];
      const p=project(f.x,f.y,-f.z);
      if(prevP){
        const col=frameColor(tr,fi,b);
        // Line width scales with zoom so it stays visible
        const lw=Math.max(1.5,2*state.zoom);
        svg+=el('line',{x1:prevP[0].toFixed(1),y1:prevP[1].toFixed(1),
          x2:p[0].toFixed(1),y2:p[1].toFixed(1),
          stroke:col,'stroke-width':lw,'stroke-linecap':'round',opacity:0.8});
      }
      prevP=p;
    }
  }
  // Waypoints
  if(tr.waypoints){
    for(let i=0;i<tr.waypoints.length;i++){
      const w=tr.waypoints[i];
      const p=project(w.x,w.y,-w.z);
      svg+=el('polygon',{points:diamondPts(p[0],p[1],7),fill:'#fff',stroke:'#000','stroke-width':1.5});
      svg+=el('text',{x:p[0]+10,y:p[1]-8,fill:'#fff','font-weight':'bold','font-size':11},esc(w.name||('WP'+(i+1))));
    }
  }
  // Playhead marker (updated by updatePlayhead to include 3D aircraft model)
  svg+='<g id="td3-playhead"></g>';
  svg+=el('text',{x:12,y:H-8,fill:'#5a6076','font-size':11},'drag to orbit \u00b7 scroll to zoom \u00b7 centered on aircraft');
  svg+='</svg>';
  host.innerHTML=svg;
  detail3DProject=project;
}

// ---------------------------------------------------------------------------
// Criteria panel — shows what each phase checks and its pass/fail status.
// Rows are clickable: clicking a row jumps the playhead to that phase and
// switches to the segment containing it. This replaces the separate phase
// chips bar, saving vertical space.
// ---------------------------------------------------------------------------
function renderCriteria(tr){
  const panel=document.getElementById('criteria-panel');
  const tbl=document.getElementById('criteria-table');
  if(!tr.phases||!tr.phases.length){
    panel.classList.add('hidden');
    return;
  }
  panel.classList.remove('hidden');
  // Segment tabs (only show if >1 segment)
  const segs=computeSegments(tr);
  let segHtml='';
  if(segs.length>1){
    segHtml='<div class="segment-tabs" style="margin-bottom:8px;display:flex;gap:6px;flex-wrap:wrap">';
    for(let si=0;si<segs.length;si++){
      const active=si===state.segmentIdx;
      const phases=segs[si].map(pi=>tr.phases[pi]?tr.phases[pi].name:'?').join(', ');
      segHtml+='<span class="chip '+(active?'active':'')+'" data-seg="'+si+'" style="cursor:pointer">Segment '+(si+1)+'</span>';
    }
    segHtml+='</div>';
  }
  let html='<thead><tr><th>#</th><th>Phase</th><th>Status</th><th>Criteria</th></tr></thead><tbody>';
  for(let i=0;i<tr.phases.length;i++){
    const p=tr.phases[i];
    const cls=p.passed?'pass':p.skipped?'skip':'fail';
    const res=p.skipped?'SKIP':p.passed?'PASS':'FAIL';
    // Find which segment this phase belongs to
    let segOf=-1;
    for(let si=0;si<segs.length;si++){if(segs[si].indexOf(i)>=0){segOf=si;break;}}
    const inActiveSeg=segOf===state.segmentIdx;
    html+='<tr data-phase="'+i+'" data-seg="'+segOf+'" style="cursor:pointer;'+(inActiveSeg?'':'opacity:0.5')+'">'+
      '<td>'+(i+1)+'</td><td class="name">'+esc(p.name)+'</td>'+
      '<td class="status"><span class="badge '+cls+'">'+res+'</span></td>'+
      '<td class="criteria-text">'+esc(p.criteria||'\u2014')+'</td></tr>';
  }
  html+='</tbody>';
  tbl.innerHTML=segHtml+html;
  // Wire up row clicks
  tbl.querySelectorAll('tbody tr').forEach(row=>{
    row.addEventListener('click',()=>{
      const pi=+row.dataset.phase;
      const seg=+row.dataset.seg;
      state.segmentIdx=seg;
      const p=tr.phases[pi];
      state.t=(p.start_s+p.end_s)/2;
      renderCriteria(tr);
      renderPathView(tr);
      renderTimeSeries(tr);
      updatePlayhead();
    });
  });
  // Wire up segment tabs
  tbl.querySelectorAll('[data-seg]').forEach(tab=>{
    if(tab.tagName==='SPAN'){
      tab.addEventListener('click',()=>{
        state.segmentIdx=+tab.dataset.seg;
        renderCriteria(tr);
        renderPathView(tr);
        updatePlayhead();
      });
    }
  });
}

// ---------------------------------------------------------------------------
// Detail view assembly
// ---------------------------------------------------------------------------
function showDetail(i){
  state.traceIdx=i;state.t=0;
  state.segmentIdx=0;state.zoom=1.0;state.panX=0;state.panY=0;
  const tr=TRACES[i]; if(!tr)return;
  document.getElementById('index-view').classList.add('hidden');
  document.getElementById('detail-view').classList.remove('hidden');
  document.getElementById('detail-title').textContent=tr.aircraft+' / '+tr.scenario;
  const st=traceStatus(tr);
  const badge=document.getElementById('detail-badge');
  badge.className='badge '+(st==='pass'?'pass':st==='fail'?'fail':st==='mixed'?'mixed':'skip');
  badge.textContent=st==='pass'?'PASS':st==='fail'?'FAIL':st==='mixed'?'MIXED':'SKIP';
  // Count segments for the summary
  const segs=computeSegments(tr);
  document.getElementById('detail-summary').innerHTML=
    '<span>Duration: <b>'+fmt(traceDuration(tr),0)+'s</b></span>'+
    '<span>Phases: <b>'+tracePassedCount(tr)+'/'+traceTotalCount(tr)+'</b> passed</span>'+
    '<span>Segments: <b>'+segs.length+'</b></span>'+
    '<span>Frames: <b>'+tr.frames.length+'</b></span>'+
    '<span>Aircraft: <b>'+esc(tr.aircraft)+'</b></span>';
  // Criteria panel (now includes clickable rows + segment tabs)
  renderCriteria(tr);
  // Hide the old phase-chips bar (replaced by clickable criteria rows)
  document.getElementById('phase-chips').classList.add('hidden');
  document.getElementById('time-end').textContent=fmt(traceDuration(tr),1)+'s';
  renderPathView(tr);
  renderTimeSeries(tr);
  updatePlayhead();
}
function showIndex(){
  state.traceIdx=-1;
  stopPlay();
  document.getElementById('detail-view').classList.add('hidden');
  document.getElementById('index-view').classList.remove('hidden');
}

// ---------------------------------------------------------------------------
// Playback
// ---------------------------------------------------------------------------
function togglePlay(){
  if(state.playing)stopPlay();else startPlay();
}
function startPlay(){
  if(state.traceIdx<0)return;
  state.playing=true;
  const btn=document.getElementById('play-btn');
  btn.innerHTML='&#10073;&#10073; Pause';
  const dt=1/60;
  if(state.playTimer)clearInterval(state.playTimer);
  state.playTimer=setInterval(()=>{
    const tr=TRACES[state.traceIdx]; if(!tr)return;
    state.t+=dt*state.speed;
    if(state.t>=traceDuration(tr)){state.t=traceDuration(tr);stopPlay();}
    updatePlayhead();
  },1000/60);
}
function stopPlay(){
  state.playing=false;
  const btn=document.getElementById('play-btn');
  if(btn)btn.innerHTML='&#9654; Play';
  if(state.playTimer){clearInterval(state.playTimer);state.playTimer=null;}
}

// ---------------------------------------------------------------------------
// Routing
// ---------------------------------------------------------------------------
function route(){
  const h=location.hash;
  if(h.indexOf('#trace/')===0){
    const idx=parseInt(h.slice(7),10);  // '#trace/' is 7 chars
    if(idx>=0&&idx<TRACES.length){showDetail(idx);return;}
  }
  showIndex();
}
window.addEventListener('hashchange',route);

// ---------------------------------------------------------------------------
// Wire up controls
// ---------------------------------------------------------------------------
document.getElementById('back-btn').addEventListener('click',()=>{location.hash='#';});
document.getElementById('play-btn').addEventListener('click',togglePlay);
document.getElementById('scrubber').addEventListener('input',e=>{
  if(state.traceIdx<0)return;
  const tr=TRACES[state.traceIdx];
  state.t=(+e.target.value/1000)*traceDuration(tr);
  updatePlayhead();
});
document.querySelectorAll('#speed-seg button').forEach(b=>{
  b.addEventListener('click',()=>{
    state.speed=+b.dataset.speed;
    document.querySelectorAll('#speed-seg button').forEach(x=>x.classList.toggle('active',x===b));
  });
});
document.querySelectorAll('#color-seg button').forEach(b=>{
  b.addEventListener('click',()=>{
    state.colorBy=b.dataset.color;
    document.querySelectorAll('#color-seg button').forEach(x=>x.classList.toggle('active',x===b));
    if(state.traceIdx>=0){
      renderPathView(TRACES[state.traceIdx]);
      updatePlayhead();
    }
  });
});
// View toggle (2D / 3D)
document.querySelectorAll('#view-seg button').forEach(b=>{
  b.addEventListener('click',()=>{
    state.viewMode=b.dataset.view;
    document.querySelectorAll('#view-seg button').forEach(x=>x.classList.toggle('active',x===b));
    if(state.traceIdx>=0){
      renderPathView(TRACES[state.traceIdx]);
      updatePlayhead();
    }
  });
});
// 3D orbit: mouse drag to rotate
document.addEventListener('mousedown',e=>{
  if(state.traceIdx<0)return;
  const tdHost=document.getElementById('topdown-host');
  const td3Host=document.getElementById('threed-host');
  if(state.viewMode==='3d'){
    if(!td3Host.contains(e.target))return;
    state.orbit.dragging=true;
    state.orbit.lastX=e.clientX;
    state.orbit.lastY=e.clientY;
  }else{
    // 2D pan: drag to pan
    if(!tdHost.contains(e.target))return;
    state.orbit.dragging=true;
    state.orbit.lastX=e.clientX;
    state.orbit.lastY=e.clientY;
  }
});
document.addEventListener('mousemove',e=>{
  if(!state.orbit.dragging)return;
  const dx=e.clientX-state.orbit.lastX;
  const dy=e.clientY-state.orbit.lastY;
  if(state.viewMode==='3d'){
    state.orbit.yaw+=dx*0.01;
    state.orbit.pitch+=dy*0.01;
    state.orbit.pitch=Math.max(-1.4,Math.min(1.4,state.orbit.pitch));
    render3D(TRACES[state.traceIdx]);
  }else{
    state.panX+=dx;
    state.panY+=dy;
    renderTopDown(TRACES[state.traceIdx]);
  }
  state.orbit.lastX=e.clientX;
  state.orbit.lastY=e.clientY;
  updatePlayhead();
});
document.addEventListener('mouseup',()=>{state.orbit.dragging=false;});
// Mouse wheel zoom (both 2D and 3D)
document.addEventListener('wheel',e=>{
  if(state.traceIdx<0)return;
  const tdHost=document.getElementById('topdown-host');
  const td3Host=document.getElementById('threed-host');
  if(!tdHost.contains(e.target)&&!td3Host.contains(e.target))return;
  e.preventDefault();
  const factor=e.deltaY<0?1.15:1/1.15;
  state.zoom=Math.max(0.2,Math.min(20,state.zoom*factor));
  renderPathView(TRACES[state.traceIdx]);
  updatePlayhead();
},{passive:false});
document.getElementById('filter-input').addEventListener('input',e=>{state.filter=e.target.value;renderIndex();});
document.getElementById('filter-status').addEventListener('change',e=>{state.filterStatus=e.target.value;renderIndex();});

// Keyboard shortcuts (when in detail view)
document.addEventListener('keydown',e=>{
  if(state.traceIdx<0)return;
  if(e.target.tagName==='INPUT')return;
  if(e.code==='Space'){e.preventDefault();togglePlay();}
  else if(e.code==='ArrowLeft'){
    const tr=TRACES[state.traceIdx];
    state.t=Math.max(0,state.t-1);updatePlayhead();
  }else if(e.code==='ArrowRight'){
    const tr=TRACES[state.traceIdx];
    state.t=Math.min(traceDuration(tr),state.t+1);updatePlayhead();
  }else if(e.code==='Escape'){location.hash='#';}
});
)HTML";
static const char* kHtmlTail4 = R"HTML(
// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
function init(){
  document.title = REPORT_TITLE;
  document.getElementById('app-title').textContent = REPORT_TITLE;
  const total=TRACES.length;
  let passN=0;
  for(const tr of TRACES)if(traceStatus(tr)==='pass')passN++;
  document.getElementById('app-sub').textContent=
    total+' traces &middot; '+passN+' fully passing &middot; click a card to inspect';
  // Render index first (default view)
  renderIndex();
  route();
}
init();
</script>
</body>
</html>
)HTML";

// ===========================================================================
// generateHtmlReport — assemble the document
// ===========================================================================
void generateHtmlReport(const std::vector<Trace>& traces,
                        std::ostream& out,
                        const HtmlReportOptions& opts) {
    out << kHtmlHead;

    // Emit the script prologue: "use strict" must be the first statement.
    // Then the report title (JSON-escaped) and the TRACES array opener.
    out << "\"use strict\";\nconst REPORT_TITLE=" << jsonString(opts.title)
        << ";\nconst TRACES=";

    // Embed traces as a compact JSON array. Each trace is downsampled (to
    // keep the file size and SVG node count reasonable), serialized via
    // traceToJson (compact), and escaped for safe embedding in <script>.
    out << "[";
    for (size_t i = 0; i < traces.size(); ++i) {
        if (i > 0) out << ",";
        Trace ds = downsampleForHtml(traces[i]);
        std::string json;
        traceToJson(ds, json);
        out << escapeForScript(json);
    }
    out << "]";

    // Write the JS template in chunks (MSVC caps a single string literal
    // at 16380 chars — see the kHtmlTail1 comment above).
    out << kHtmlTail1 << kHtmlTail1b << kHtmlTail2 << kHtmlTail3 << kHtmlTail3b << kHtmlTail4;
}

} // namespace f4flight
