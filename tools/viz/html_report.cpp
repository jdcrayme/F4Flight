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
/* per-frame samples (range, heading error, fuel, TTGO, etc.) — visually
   distinct from the built-in telemetry cells: accent-tinted background +
   left border to mark them as scenario-published extras. */
.readout .cell.sample{background:rgba(96,165,250,0.06);border-color:rgba(96,165,250,0.4);
  border-left:3px solid var(--accent)}
.readout .cell.sample .l{color:var(--accent)}
.readout .cell.sample .v{font-size:13px;color:var(--text)}

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
/* failure reason row inside criteria-text cell */
.failure-reason{color:#ff6b6b;font-style:italic;font-size:11px;margin-top:3px;
  padding:2px 6px;border-left:2px solid #ff6b6b;background:rgba(255,107,107,0.06);
  border-radius:2px;line-height:1.4}
.failure-reason span{color:#ffb3b3}

/* event log panel */
.event-log-panel{background:var(--panel);border:1px solid var(--border);border-radius:10px;
  padding:14px;margin-bottom:14px}
.event-log-panel h3{margin:0 0 8px;font-size:13px;color:var(--muted);text-transform:uppercase;
  letter-spacing:.5px;font-weight:600}
.event-log{max-height:200px;overflow-y:auto;background:var(--bg);border:1px solid var(--border);
  border-radius:6px;padding:4px;font-family:ui-monospace,"SF Mono",Menlo,monospace;font-size:12px}
.event-row{display:flex;gap:8px;padding:3px 6px;border-bottom:1px solid rgba(42,47,66,0.5);
  align-items:baseline;transition:background .1s}
.event-row:last-child{border-bottom:none}
.event-row:hover{background:rgba(255,255,255,0.03)}
.event-row.sev-info{color:var(--muted)}
.event-row.sev-warn{color:var(--warn)}
.event-row.sev-fail{color:var(--fail)}
.event-row.current{background:rgba(96,165,250,0.15);box-shadow:inset 2px 0 0 var(--accent)}
.ev-time{min-width:56px;color:var(--dim);font-size:11px}
.ev-cat{min-width:64px;font-size:10px;text-transform:uppercase;letter-spacing:.3px;
  padding:1px 5px;border-radius:8px;background:var(--panel2);color:var(--text);
  text-align:center;font-weight:600}
.event-row.sev-warn .ev-cat{background:rgba(251,191,36,0.18);color:var(--warn)}
.event-row.sev-fail .ev-cat{background:rgba(248,113,113,0.18);color:var(--fail)}
.event-row.sev-info .ev-cat{background:rgba(138,144,166,0.18);color:var(--muted)}
.ev-msg{flex:1;word-break:break-word}
.event-log-empty{color:var(--dim);text-align:center;padding:20px;font-style:italic}

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
    <div class="event-log-panel hidden" id="event-log-panel">
      <h3>Event Log <span id="event-log-count" style="color:var(--dim);font-weight:400;text-transform:none;letter-spacing:0"></span></h3>
      <div class="event-log" id="event-log"></div>
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

// Phase ranges: [startFrame, endFrame] per phase (frames split at boundaries).
// Uses a strict `<` on end_s so that a frame recorded at exactly the phase
// boundary time (which is the FIRST frame of the next phase, recorded after
// Init() repositioned the aircraft) is assigned to the NEXT phase, not the
// current one. This prevents a straight "repositioning" line from being
// drawn at the tail of the previous phase's polyline.
function phaseRanges(tr){
  if(!tr.phases||!tr.phases.length){
    return tr.frames.length?[[0,tr.frames.length-1]]:[];
  }
  const ranges=[];
  let fi=0;
  for(const p of tr.phases){
    const start=fi;
    while(fi<tr.frames.length&&tr.frames[fi].t<p.end_s)fi++;
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
// Return [minT, maxT] for the active segment — the time window the
// scrubber, time-series X-axis, and playback loop are scoped to.
function segmentTimeRange(tr,segIdx){
  const segs=computeSegments(tr);
  const si=Math.min(segIdx,segs.length-1);
  const segPhases=segs[si];
  const ranges=phaseRanges(tr);
  let minT=Infinity,maxT=-Infinity;
  for(const pi of segPhases){
    if(pi>=ranges.length)continue;
    const [s,e]=ranges[pi];
    if(s<tr.frames.length){const t=tr.frames[s].t; if(t<minT)minT=t;}
    if(e<tr.frames.length){const t=tr.frames[e].t; if(t>maxT)maxT=t;}
  }
  if(!isFinite(minT))minT=0;
  if(!isFinite(maxT))maxT=traceDuration(tr);
  if(maxT<=minT)maxT=minT+1;
  return [minT,maxT];
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
  // Collect tracks for moving entities (missile, lead, wingman)
  // and bearing lines (guns/target) and static markers (slot)
  const tracks={}; // key=type -> [{x,y,t}]
  const fi=findFrameIndex(tr,currentT);
  const acFrame=tr.frames[fi];
  let bearingLines='';
  for(let i=0;i<tr.frames.length;i++){
    const f=tr.frames[i];
    if(!f.threats)continue;
    for(const th of f.threats){
      if(th.type==='missile'||th.type==='lead'||th.type==='wingman'){
        if(!tracks[th.type])tracks[th.type]=[];
        tracks[th.type].push({x:th.x,y:th.y,t:f.t});
      }
    }
  }
  // Color/style config per entity type
  const style={
    missile:{color:'#f44336',label:'missile',r:5},
    lead:{color:'#4caf50',label:'lead',r:6},
    wingman:{color:'#00e5ff',label:'wingman',r:5},
    slot:{color:'#2196f3',label:'slot',r:5},
    guns:{color:'#ff9800',label:'guns',r:5},
    target:{color:'#9c27b0',label:'target',r:5},
    airbase:{color:'#ffc107',label:'airbase',r:6},
  };
  let svg='<g class="threats">';
  // Draw trails + current positions for moving entities (missile, lead, wingman)
  for(const key in tracks){
    const track=tracks[key];
    if(track.length<2)continue;
    const st=style[key]||style.missile;
    let pts='';
    for(const p of track){pts+=(pts?' ':'')+sx(p.x).toFixed(1)+','+sy(p.y).toFixed(1);}
    svg+=el('polyline',{points:pts,fill:'none',stroke:st.color,'stroke-width':1.5,
      'stroke-dasharray':key==='missile'?'4,3':'none',opacity:0.5});
    // Current position
    const cur=track[Math.min(track.length-1,fi)];
    if(cur){
      svg+=el('circle',{cx:sx(cur.x),cy:sy(cur.y),r:st.r,fill:st.color,stroke:'#fff','stroke-width':1,opacity:0.9});
      svg+=el('text',{x:sx(cur.x)+7,y:sy(cur.y)+4,fill:st.color,'font-size':10},st.label);
    }
  }
  // Draw bearing lines (guns, target) + slot markers + airbase squares at current frame
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
      }else if(th.type==='slot'){
        // Blue diamond marker for desired formation slot
        const cx=sx(th.x),cy=sy(th.y);
        svg+=el('polygon',{points:cx+','+(cy-6)+' '+(cx+6)+','+cy+' '+cx+','+(cy+6)+' '+(cx-6)+','+cy,
          fill:'none',stroke:'#2196f3','stroke-width':1.5,opacity:0.8});
        svg+=el('text',{x:cx+8,y:cy+4,fill:'#2196f3','font-size':9},'slot');
      }else if(th.type==='airbase'){
        // Amber square marker for a friendly airbase (runway box) — larger
        // than the target square, with a hollow fill so multiple airbases
        // in close proximity are distinguishable. No bearing line is drawn
        // (airbases are not engagement targets).
        const cx=sx(th.x),cy=sy(th.y),r=7;
        svg+=el('rect',{x:cx-r,y:cy-r,width:r*2,height:r*2,
          fill:'rgba(255,193,7,0.35)',stroke:'#ffc107','stroke-width':1.5,opacity:0.95});
        // Inner runway strip (thin amber rectangle, rotated to suggest a runway)
        svg+=el('rect',{x:cx-r*0.7,y:cy-1.5,width:r*1.4,height:3,
          fill:'#ffc107',opacity:0.9});
        svg+=el('text',{x:cx+r+3,y:cy+4,fill:'#ffc107','font-size':10,'font-weight':'bold'},'airbase');
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
let detailBounds=null, detailScales=null, detail3DProject=null, tsScales=null;

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

  // Paths — only draw phases in the current segment.
  // Gradient modes use per-frame colored ribbon quads (filled polygons)
  // instead of stroked lines. Filled quads render as a flat colored strip
  // with no stroke caps or bulging at joints — a ribbon, not a tube.
  const ranges=phaseRanges(tr);
  if(state.colorBy==='altitude'||state.colorBy==='speed'||state.colorBy==='g'||state.colorBy==='mode'){
    const rw=1.5; // ribbon half-width in px
    for(const pi of segPhases){
      if(pi>=ranges.length)continue;
      const [s,e]=ranges[pi];
      for(let fi=s;fi<e&&fi<tr.frames.length-1;fi++){
        const f1=tr.frames[fi], f2=tr.frames[fi+1];
        const col=frameColor(tr,fi,b);
        const x1=sx(f1.x), y1=sy(f1.y), x2=sx(f2.x), y2=sy(f2.y);
        const ddx=x2-x1, ddy=y2-y1;
        const dlen=Math.sqrt(ddx*ddx+ddy*ddy)||1;
        const nx=-ddy/dlen, ny=ddx/dlen;
        const pts=
          (x1+nx*rw).toFixed(1)+','+(y1+ny*rw).toFixed(1)+' '+
          (x2+nx*rw).toFixed(1)+','+(y2+ny*rw).toFixed(1)+' '+
          (x2-nx*rw).toFixed(1)+','+(y2-ny*rw).toFixed(1)+' '+
          (x1-nx*rw).toFixed(1)+','+(y1-ny*rw).toFixed(1);
        svg+=el('polygon',{points:pts,fill:col,stroke:'none',opacity:0.9});
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
  // Scope to the active segment: bounds, time window, and phase set all
  // come from segPhases so discontinuities in other segments don't bleed
  // into the graphs (no phantom connecting lines, no empty time gaps).
  const segs=computeSegments(tr);
  const si=Math.min(state.segmentIdx,segs.length-1);
  const segPhases=segs[si];
  const b=computeSegmentBounds(tr,segPhases);
  const W=1000,H=560,padL=60,padR=40,padT=20,padB=36;
  const plotW=W-padL-padR;
  const panelH=(H-padT-padB)/3;
  const [segMinT,segMaxT]=segmentTimeRange(tr,si);
  const segDur=segMaxT-segMinT;
  const sx=t=>padL+((t-segMinT)/segDur)*plotW;
  // Each panel: [field, lo, hi, unit, color, fmtFn]
  const panels=[
    {name:'Altitude',get:f=>-f.z,lo:b.minAlt,hi:b.maxAlt,unit:'ft',col:'#4ade80'},
    {name:'Speed (VCAS)',get:f=>f.vcas,lo:b.minSpd,hi:b.maxSpd,unit:'kts',col:'#60a5fa'},
    {name:'G-load',get:f=>f.nzcgs,lo:0,hi:Math.max(2,b.maxG),unit:'G',col:'#f59e0b'},
  ];
  let svg='<svg viewBox="0 0 '+W+' '+H+'" preserveAspectRatio="xMidYMid meet">';
  // Time grid spacing (based on segment duration, not full trace)
  let ts=10;
  if(segDur>300)ts=60;else if(segDur>120)ts=30;else if(segDur>60)ts=15;else if(segDur>30)ts=10;else ts=5;

  for(let pi=0;pi<panels.length;pi++){
    const p=panels[pi];
    const top=padT+pi*panelH;
    const bottom=top+panelH-16; // leave gap
    const ph=bottom-top;
    const sy=v=>top+(p.hi-v)/(p.hi-p.lo)*ph;
    // Panel label
    svg+=el('text',{x:padL,y:top-4,class:'axis-text','font-weight':'bold'},p.name+' ('+p.unit+')');
    // Grid + Y labels
    const vrange=p.hi-p.lo;
    let vstep=vrange/4;
    // nice step
    const mag=Math.pow(10,Math.floor(Math.log10(vstep)));
    vstep=Math.ceil(vstep/mag)*mag;
    for(let v=Math.ceil(p.lo/vstep)*vstep;v<=p.hi;v+=vstep){
      svg+=el('line',{x1:padL,y1:sy(v),x2:padL+plotW,y2:sy(v),class:'grid-line'});
      svg+=el('text',{x:padL-5,y:sy(v)+3,class:'axis-text','text-anchor':'end'},fmtInt(v));
    }
    // Phase bands — only for phases in the active segment
    const ranges=phaseRanges(tr);
    if(tr.phases&&tr.phases.length){
      for(const phaseIdx of segPhases){
        if(phaseIdx>=tr.phases.length)continue;
        const ph=tr.phases[phaseIdx];
        const x1=sx(ph.start_s), x2=sx(ph.end_s);
        const col=ph.skipped?'rgba(148,163,184,0.07)':ph.passed?'rgba(74,222,128,0.07)':'rgba(248,113,113,0.12)';
        svg+=el('rect',{x:x1,y:top,width:Math.max(1,x2-x1),height:ph,fill:col});
        // phase number at top
        svg+=el('text',{x:(x1+x2)/2,y:bottom+12,class:'axis-text','text-anchor':'middle','font-size':9,fill:ph.passed?'var(--pass)':ph.skipped?'var(--skip)':'var(--fail)'},(phaseIdx+1));
      }
    }
    // Data line — only phases in the active segment, each as a separate
    // polyline so repositioning breaks (within the segment, if any slipped
    // through) don't draw a connecting line.
    for(const phaseIdx of segPhases){
      if(phaseIdx>=ranges.length)continue;
      const [s,e]=ranges[phaseIdx];
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
    // Event tick marks — small vertical lines at event times, colored by
    // severity. Only events within the active segment's time window are
    // shown. The tick spans the full panel height with low opacity so it
    // reads as a faint marker, not a solid bar — the data line stays
    // visible underneath.
    if(tr.events){
      for(const e of tr.events){
        if(e.t<segMinT||e.t>segMaxT)continue;
        const sev=e.severity||'info';
        const col=sev==='fail'?'#f87171':sev==='warn'?'#fbbf24':'#94a3b8';
        const ex=sx(e.t);
        if(ex<padL||ex>padL+plotW)continue;
        svg+=el('line',{x1:ex.toFixed(1),y1:top,x2:ex.toFixed(1),y2:bottom,
          stroke:col,'stroke-width':1,opacity:0.55,'stroke-dasharray':'2,2'});
        // Small triangle marker at the top of the panel so events are
        // visible even when the panel's data line is far from the event.
        svg+=el('polygon',{points:
          ex.toFixed(1)+','+(top-1)+' '+
          (ex-3).toFixed(1)+','+(top-6)+' '+
          (ex+3).toFixed(1)+','+(top-6),
          fill:col,opacity:0.8});
      }
    }
    // Playhead line (updated separately)
    svg+=el('line',{id:'ts-play-'+pi,x1:padL,y1:top,x2:padL,y2:bottom,stroke:'#fff','stroke-width':1,opacity:0.5});
  }
  // Time axis labels — relative to segment start, shown as absolute time
  for(let t=Math.ceil(segMinT/ts)*ts;t<=segMaxT;t+=ts){
    svg+=el('text',{x:sx(t),y:H-6,class:'axis-text','text-anchor':'middle'},fmtInt(t)+'s');
  }
  svg+='</svg>';
  host.innerHTML=svg;
  // Stash segment time range + plot geometry so updatePlayhead can position
  // the playhead line without re-deriving it every frame.
  tsScales={padL:padL,plotW:plotW,segMinT:segMinT,segMaxT:segMaxT};
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
  // Clamp the playhead to the active segment's time window so the
  // scrubber, graphs, and 3D view all stay in sync with the selected
  // segment (no data from other segments leaks in).
  const [segMinT,segMaxT]=segmentTimeRange(tr,state.segmentIdx);
  if(state.t<segMinT)state.t=segMinT;
  if(state.t>segMaxT)state.t=segMaxT;
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
  // 3D playhead: aircraft as a 3D model with FULL yaw/pitch/roll orientation.
  //
  // Uses the EXACT body-to-world DCM from the flight model
  // (dcmFromEuler in flight/core/types.h), which is the standard ZYX
  // rotation: R = Rz(psi) * Ry(theta) * Rx(phi).
  //
  // Frame conventions (from aircraft_state.h):
  //   World (NED): X=East, Y=North, Z=Down (altitude = -z)
  //   Body:        X=forward, Y=right, Z=down
  //
  // DCM columns = body axes expressed in world (NED, Z-down):
  //   Col 0 (body X, forward) = (cp*ct, sp*ct, -st)
  //   Col 1 (body Y, right)   = (cp*st*sf - sp*cf, sp*st*sf + cp*cf, ct*sf)
  //   Col 2 (body Z, down)    = (cp*st*cf + sp*sf, sp*st*cf - cp*sf, ct*cf)
  //
  // IMPORTANT: This codebase's world frame is X=East, Y=North (NOT standard
  // NED's X=North, Y=East). The DCM formula is standard ZYX which assumes
  // X=North, Y=East. When applied to X=East, Y=North, the DCM's "body Y"
  // column actually points LEFT in world space (e.g., when psi=0 / facing
  // East, the DCM says body Y = +Y = North, but right = South when facing
  // East). So we NEGATE the body Y column to get true body right.
  //
  // The viz frame uses Z=UP (altitude positive), so we also negate the Z
  // component of each column (NED Z-down → viz Z-up).
  //
  // Final viz body axes (X=East, Y=North, Z=Up):
  //   X3 (forward) = (cp*ct, sp*ct, st)
  //   Y3 (right)   = (sp*cf - cp*st*sf, -cp*cf - sp*st*sf, ct*sf)
  //                = -(DCM col 1 with Z negated)
  //   Z3 (down)    = (cp*st*cf + sp*sf, sp*st*cf - cp*sf, -ct*cf)
  //                = (DCM col 2 with Z negated)
  //
  // Roll convention: positive phi = right wing down (right bank), matching
  // the rstick convention (+1 = full right). Verified: at phi=-59° the
  // aircraft is in a right turn (psi decreasing) and the model correctly
  // shows right wing down.
  //
  // Faces are depth-sorted (painter's algorithm) so the near side is drawn
  // on top of the far side — critical when banked/inverted.
  const td3=document.getElementById('td3-playhead');
  if(td3&&detail3DProject&&detailBounds){
    const span3D=Math.max(detailBounds.maxX-detailBounds.minX,
                          detailBounds.maxY-detailBounds.minY,
                          detailBounds.maxAlt-detailBounds.minAlt,1);
    const size=span3D*0.005; // aircraft size relative to data span (large enough to see bank/pitch)
    const psi=f.psi, theta=f.theta, phi=f.phi;
    const cP=Math.cos(psi), sP=Math.sin(psi);
    const cT=Math.cos(theta), sT=Math.sin(theta);
    const cF=Math.cos(phi), sF=Math.sin(phi);
    // Body axes in viz world (X=East, Y=North, Z=Up).
    // Derived from dcmFromEuler columns: negate Z (NED→viz) and negate
    // body-Y column (DCM body-Y points left in this world frame).
    const X3=[cP*cT, sP*cT, sT];
    const Y3=[-(cP*sT*sF - sP*cF), -(sP*sT*sF + cP*cF), -(-cT*sF)];
    const Z3=[cP*sT*cF + sP*sF, sP*sT*cF - cP*sF, -cT*cF];
    // Body-frame points: [forward, right, down]
    //   0: nose       — forward
    //   1: tail       — back, slightly below (belly fin)
    //   2: left wing  — back, left (-Y body = -right)
    //   3: right wing — back, right (+Y body)
    //   4: top        — back slightly, up (-Z body = -down = canopy)
    const bodyPts=[
      [size*2,     0,          0       ],
      [-size*0.5,  0,          size*0.1],
      [-size*0.5, -size,       0       ],
      [-size*0.5,  size,       0       ],
      [-size*0.13, 0,         -size*0.4],
    ];
    const alt=-f.z;
    // Transform each body point to world ENU, then project to screen.
    // proj[i] = [screenX, screenY, depthZr]
    const proj=bodyPts.map(bp=>{
      const wx=f.x+bp[0]*X3[0]+bp[1]*Y3[0]+bp[2]*Z3[0];
      const wy=f.y+bp[0]*X3[1]+bp[1]*Y3[1]+bp[2]*Z3[1];
      const wz=alt+bp[0]*X3[2]+bp[1]*Y3[2]+bp[2]*Z3[2];
      return detail3DProject(wx,wy,wz);
    });
    // Faces (vertex indices) with base color. Bottom faces are darker
    // (belly), top faces are lighter (canopy) — so when the aircraft rolls
    // inverted you see the dark belly on top, which is visually correct.
    const faces=[
      {idx:[0,1,2], color:'#7a7a7a'}, // bottom-left  (nose-tail-LW)
      {idx:[0,1,3], color:'#9a9a9a'}, // bottom-right (nose-tail-RW)
      {idx:[2,1,3], color:'#8a8a8a'}, // bottom       (LW-tail-RW)
      {idx:[0,2,4], color:'#dadada'}, // top-left     (nose-LW-top)
      {idx:[0,3,4], color:'#cacaca'}, // top-right    (nose-RW-top)
      {idx:[2,4,3], color:'#bababa'}, // top-rear     (LW-top-RW)
    ];
    // Depth-sort (painter's algorithm): larger zr = further away = draw first.
    const facesSorted=faces.map(face=>{
      const avgZr=(proj[face.idx[0]][2]+proj[face.idx[1]][2]+proj[face.idx[2]][2])/3;
      return {idx:face.idx, color:face.color, avgZr};
    }).sort((a,b)=>b.avgZr-a.avgZr);
    let model='';
    for(const face of facesSorted){
      const pts=face.idx.map(i=>proj[i][0].toFixed(1)+','+proj[i][1].toFixed(1)).join(' ');
      model+=el('polygon',{points:pts,fill:face.color,stroke:'#000','stroke-width':0.4,opacity:0.96});
    }
    // Tadpole: dashed vertical drop-line from the aircraft straight down to
    // the ground grid (z=0), ending in a ring + dot. This gives a constant
    // visual reference for altitude and ground track position while
    // orbiting — without it the aircraft model floats in empty space.
    // The drop-line uses the aircraft's POSITION (not orientation) so it
    // always points straight down regardless of bank/pitch.
    const pAcTip=detail3DProject(f.x,f.y,alt);
    const pGndTip=detail3DProject(f.x,f.y,0);
    let tadpole='';
    tadpole+=el('line',{x1:pAcTip[0].toFixed(1),y1:pAcTip[1].toFixed(1),
      x2:pGndTip[0].toFixed(1),y2:pGndTip[1].toFixed(1),
      stroke:'#e2e8f0','stroke-width':1,'stroke-dasharray':'4,3',opacity:0.55});
    tadpole+=el('circle',{cx:pGndTip[0].toFixed(1),cy:pGndTip[1].toFixed(1),
      r:5,fill:'none',stroke:'#e2e8f0','stroke-width':1.5,opacity:0.6});
    tadpole+=el('circle',{cx:pGndTip[0].toFixed(1),cy:pGndTip[1].toFixed(1),
      r:1.5,fill:'#e2e8f0',opacity:0.75});
    // Prepend so the aircraft model draws on top of the drop-line.
    model=tadpole+model;
    // Velocity vector arrow: shows the direction of travel as a bold
    // arrow extending forward from the nose. The length scales with
    // airspeed (vt) so faster flight = longer arrow — giving an instant
    // visual read on energy state. The arrow is drawn in body-X
    // (forward) direction using the same rotation as the model, so it
    // follows the nose through yaw/pitch/roll. A bright accent color
    // (cyan) distinguishes it from the gray model.
    //
    // The arrow is built from 3 line segments: a shaft (nose → tip)
    // and two arrowhead lines (tip → back-left, tip → back-right),
    // using the body Y axis for the arrowhead spread.
    {
      const vScale=span3D*0.000012; // arrow length per ft/s of airspeed
      const arrowLen=size*1.5+f.vt*vScale; // base size + speed-scaled
      // Arrow tip: forward from the nose by arrowLen.
      const tipX=f.x+X3[0]*arrowLen*1.5, tipY=f.y+X3[1]*arrowLen*1.5, tipZ=alt+X3[2]*arrowLen*1.5;
      // Arrow base: at the nose position.
      const baseX=f.x+X3[0]*size*2, baseY=f.y+X3[1]*size*2, baseZ=alt+X3[2]*size*2;
      const pTip=detail3DProject(tipX,tipY,tipZ);
      const pBase=detail3DProject(baseX,baseY,baseZ);
      // Arrowhead: two points back from the tip, offset along body Y.
      const headLen=arrowLen*0.35;
      const headBackX=tipX-X3[0]*headLen, headBackY=tipY-X3[1]*headLen, headBackZ=tipZ-X3[2]*headLen;
      const pHeadL=detail3DProject(headBackX+Y3[0]*headLen*0.5, headBackY+Y3[1]*headLen*0.5, headBackZ+Y3[2]*headLen*0.5);
      const pHeadR=detail3DProject(headBackX-Y3[0]*headLen*0.5, headBackY-Y3[1]*headLen*0.5, headBackZ-Y3[2]*headLen*0.5);
      let vel='';
      // Shaft (thick, bright cyan)
      vel+=el('line',{x1:pBase[0].toFixed(1),y1:pBase[1].toFixed(1),
        x2:pTip[0].toFixed(1),y2:pTip[1].toFixed(1),
        stroke:'#22d3ee','stroke-width':2.5,'stroke-linecap':'round',opacity:0.95});
      // Arrowhead (two lines forming a V at the tip)
      vel+=el('line',{x1:pTip[0].toFixed(1),y1:pTip[1].toFixed(1),
        x2:pHeadL[0].toFixed(1),y2:pHeadL[1].toFixed(1),
        stroke:'#22d3ee','stroke-width':2.5,'stroke-linecap':'round',opacity:0.95});
      vel+=el('line',{x1:pTip[0].toFixed(1),y1:pTip[1].toFixed(1),
        x2:pHeadR[0].toFixed(1),y2:pHeadR[1].toFixed(1),
        stroke:'#22d3ee','stroke-width':2.5,'stroke-linecap':'round',opacity:0.95});
      model+=vel;
    }
    td3.innerHTML=model;
  }
  // Time-series playhead lines — positioned within the segment's time
  // window (tsScales is set by renderTimeSeries).
  for(let pi=0;pi<3;pi++){
    const ln=document.getElementById('ts-play-'+pi);
    if(ln&&tsScales){
      const frac=(state.t-tsScales.segMinT)/(tsScales.segMaxT-tsScales.segMinT);
      const x=tsScales.padL+Math.max(0,Math.min(1,frac))*tsScales.plotW;
      ln.setAttribute('x1',x);ln.setAttribute('x2',x);
    }
  }
  // Scrubber — spans only the active segment's time window.
  const scr=document.getElementById('scrubber');
  if(scr){
    const frac=(state.t-segMinT)/(segMaxT-segMinT);
    scr.value=Math.round(Math.max(0,Math.min(1,frac))*1000);
  }
  document.getElementById('time-now').textContent=fmt(state.t,1)+'s';
  const te=document.getElementById('time-end');
  if(te)te.textContent=fmt(segMaxT,1)+'s';
  // Frame readout
  const ro=document.getElementById('readout');
  const alt=-f.z;
  const phaseIdx=phaseRanges(tr).findIndex(([s,e])=>fi>=s&&fi<=e);
  const phaseName=phaseIdx>=0&&tr.phases[phaseIdx]?tr.phases[phaseIdx].name:'—';
  // Per-frame samples (range, heading error, fuel, TTGO, etc.) — appended
  // after the standard cells. Rendered with the 'sample' class so they can
  // be styled differently (lighter background, accent border) from the
  // built-in telemetry cells.
  let samplesHtml='';
  if(f.samples&&f.samples.length){
    for(const s of f.samples){
      const v=fmt(s.value,1)+(s.unit?' '+s.unit:'');
      samplesHtml+=cell(esc(s.key),v,'sample');
    }
  }
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
    cell('Phase',esc(phaseName),'phase')+
    samplesHtml;
  // Update the event log panel (auto-scrolls to the event closest to the
  // playhead). Skipped if the trace has no events — renderEventLog hides
  // the panel in that case.
  renderEventLog(tr);
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
  // 3D projection centered on the aircraft.
  //
  // Convention: pitch=0 looks straight DOWN (camera directly above target),
  // pitch=π/2 looks horizontally (camera at target's altitude, off to the
  // side set by yaw). The pitch rotation is applied to the (yr, pz) pair
  // after yaw, mapping:
  //   yr2 (screen-up before perspective) = yr*cos(pitch) + pz*sin(pitch)
  //   zr  (depth relative to target)     = yr*sin(pitch) - pz*cos(pitch)
  //
  // The +sin(pitch) on pz in yr2 ensures HIGHER altitude → UP on screen
  // (previously this was `- pz*sin(pitch)`, which inverted altitude: the
  // ground appeared ABOVE the aircraft when looking down).
  //
  // The -cos(pitch) on pz in zr ensures that at pitch=0 (top-down), points
  // ABOVE the target are CLOSER to the camera (smaller zr → larger f),
  // which matches a camera looking down from above.
  function project(x,y,z){
    let px=x-cx, py=y-cy, pz=z-cz;
    const xr=px*Math.cos(yaw)-py*Math.sin(yaw);
    const yr=px*Math.sin(yaw)+py*Math.cos(yaw);
    const yr2=yr*Math.cos(pitch)+pz*Math.sin(pitch);
    const zr=yr*Math.sin(pitch)-pz*Math.cos(pitch);
    const d=span*3;
    const f=d/(d+zr);
    return [W/2+xr*f*scale+state.panX, H/2-yr2*f*scale+state.panY, zr];
  }
  let svg='<svg viewBox="0 0 '+W+' '+H+'" preserveAspectRatio="xMidYMid meet">';
  svg+=el('rect',{x:0,y:0,width:W,height:H,fill:'#0a0c12'});
  // Ground grid — centered on aircraft, extends ±span/2. Brightened so
  // the ground plane is clearly visible (was too faint to see, making the
  // 3D view feel disorienting). The two axis lines passing through the
  // aircraft's ground position are drawn brighter as a reference cross.
  const gs=span>50000?10000:span>10000?5000:1000;
  const gridR=span; // grid extends this far from center
  svg+='<g opacity="0.5">';
  for(let gx=Math.ceil((cx-gridR)/gs)*gs;gx<=cx+gridR;gx+=gs){
    const p1=project(gx,cy-gridR,0), p2=project(gx,cy+gridR,0);
    const isAxis=Math.abs(gx-cx)<gs*0.5;
    svg+=el('line',{x1:p1[0],y1:p1[1],x2:p2[0],y2:p2[1],stroke:isAxis?'#5a6275':'#3a4258','stroke-width':isAxis?1.0:0.7});
  }
  for(let gy=Math.ceil((cy-gridR)/gs)*gs;gy<=cy+gridR;gy+=gs){
    const p1=project(cx-gridR,gy,0), p2=project(cx+gridR,gy,0);
    const isAxis=Math.abs(gy-cy)<gs*0.5;
    svg+=el('line',{x1:p1[0],y1:p1[1],x2:p2[0],y2:p2[1],stroke:isAxis?'#5a6275':'#3a4258','stroke-width':isAxis?1.0:0.7});
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
  // Flight path — drawn as a depth-sorted 3D ribbon trail with ground shadow
  // and bank-tilted orientation.
  //
  // The ribbon is a continuous strip that follows the 3D flight path. At
  // each sample, the ribbon's normal is computed as the horizontal
  // perpendicular to the path tangent, then TILTED by the aircraft's roll
  // angle (phi) so the ribbon banks with the aircraft. This makes the
  // ribbon look like a contrail carved by the wings — when the aircraft
  // banks right, the ribbon's top leans right, matching the bank.
  //
  // Bank math:
  //   N_h = horizontal normal = normalize(cross(path_tangent, world_up))
  //   B   = binormal = normalize(cross(N_h, path_tangent))  ≈ world_up
  //   N_banked = N_h * cos(phi) + B * sin(phi)
  //
  // In this codebase's convention, NEGATIVE phi = right bank (right wing
  // down). With N_banked = N_h*cos(phi) + B*sin(phi):
  //   phi < 0 (right bank) → sin(phi) < 0 → N_banked tilts DOWNWARD
  //   → the right edge of the ribbon drops → correct right-bank lean ✓
  //
  // The ground shadow is the projection of the banked ribbon edges onto
  // z=0. When the ribbon banks, its shadow on the ground widens or
  // narrows accordingly (a vertical ribbon at 90° bank projects to a
  // line). This gives correct visual feedback for the bank angle even
  // when looking at just the shadow.
  //
  // Other features:
  //   1. DEPTH SORTING: all quads (ribbon + shadow) across all phases are
  //      collected and sorted by average depth, drawn far-to-near.
  //   2. SHARED VERTICES: normals use averaged tangents at interior
  //      vertices → no gaps at corners.
  //   3. ANTI-ALIASING: same-color stroke on each quad prevents 1px gaps.
  const ranges=phaseRanges(tr);
  const rw3D=span*0.004; // 3D half-width in world feet (wide enough to see at all zoom levels)
  const allQuads=[]; // {pts, color, opacity, avgZr} — collected, then depth-sorted
  for(const pi of segPhases){
    if(pi>=ranges.length)continue;
    const [s,e]=ranges[pi]; if(e-s<2)continue;
    const total=e-s;
    const stride=Math.max(1,Math.floor(total/400));
    // Sample path points in 3D world space (x, y, z=altitude) + roll angle.
    const samples=[];
    for(let fi=s;fi<=e&&fi<tr.frames.length;fi+=stride){
      const f=tr.frames[fi];
      samples.push({x:f.x, y:f.y, z:-f.z, phi:f.phi, fi:fi});
    }
    if(samples.length<2)continue;
    // Compute horizontal normal and 3D tangent at each sample using
    // averaged incoming+outgoing direction (for gap-free joints).
    const normals=[];  // horizontal normal [nx, ny]
    const tangents=[];  // 3D tangent [tx, ty, tz] (for binormal)
    for(let i=0;i<samples.length;i++){
      const prev=samples[Math.max(0,i-1)];
      const next=samples[Math.min(samples.length-1,i+1)];
      const dx=next.x-prev.x, dy=next.y-prev.y, dz=next.z-prev.z;
      const hlen=Math.sqrt(dx*dx+dy*dy)||1;
      let nx=-dy/hlen, ny=dx/hlen;
      const nlen=Math.sqrt(nx*nx+ny*ny)||1;
      normals.push([nx/nlen, ny/nlen]);
      const tlen=Math.sqrt(dx*dx+dy*dy+dz*dz)||1;
      tangents.push([dx/tlen, dy/tlen, dz/tlen]);
    }
    // Project left/right edge points for the ribbon (at altitude, banked)
    // and for the ground shadow (at z=0, projected from banked edges).
    const left=[], right=[], sLeft=[], sRight=[];
    for(let i=0;i<samples.length;i++){
      const sm=samples[i], n=normals[i], T=tangents[i];
      // Binormal B = cross(N_h, T) — approximately world-up when path
      // is horizontal. N_h = [n[0], n[1], 0].
      // cross([nx,ny,0], [tx,ty,tz]) = [ny*tz, -nx*tz, nx*ty - ny*tx]
      const bx=n[1]*T[2], by=-n[0]*T[2], bz=n[0]*T[1]-n[1]*T[0];
      const blen=Math.sqrt(bx*bx+by*by+bz*bz)||1;
      const Bx=bx/blen, By=by/blen, Bz=bz/blen;
      // Banked normal: N_h * cos(phi) + B * sin(phi).
      // phi < 0 (right bank) → sin(phi) < 0 → normal tilts down →
      // right edge drops, matching the aircraft's right bank.
      const phi=sm.phi;
      const cPhi=Math.cos(phi), sPhi=Math.sin(phi);
      const bnx=n[0]*cPhi+Bx*sPhi;
      const bny=n[1]*cPhi+By*sPhi;
      const bnz=Bz*sPhi; // N_h has z=0, so only B contributes to z
      // Ribbon edges at altitude (banked).
      left.push(project(sm.x+bnx*rw3D, sm.y+bny*rw3D, sm.z+bnz*rw3D));
      right.push(project(sm.x-bnx*rw3D, sm.y-bny*rw3D, sm.z-bnz*rw3D));
      // Ground shadow: project the same 3D edge points onto z=0.
      // When the ribbon banks, the shadow width changes to match the
      // projected width of the tilted strip.
      sLeft.push(project(sm.x+bnx*rw3D, sm.y+bny*rw3D, 0));
      sRight.push(project(sm.x-bnx*rw3D, sm.y-bny*rw3D, 0));
    }
    // Build ribbon quads + shadow quads. Both participate in the same
    // depth sort so they interleave correctly with each other and with
    // quads from other phases at crossings.
    for(let i=0;i<samples.length-1;i++){
      const col=frameColor(tr,samples[i].fi,b);
      // Ribbon quad (at altitude).
      const rAvgZr=(left[i][2]+left[i+1][2]+right[i+1][2]+right[i][2])/4;
      const rPts=
        left[i][0].toFixed(1)+','+left[i][1].toFixed(1)+' '+
        left[i+1][0].toFixed(1)+','+left[i+1][1].toFixed(1)+' '+
        right[i+1][0].toFixed(1)+','+right[i+1][1].toFixed(1)+' '+
        right[i][0].toFixed(1)+','+right[i][1].toFixed(1);
      allQuads.push({pts:rPts, color:col, opacity:0.9, avgZr:rAvgZr});
      // Ground shadow quad (at z=0). Same color, low opacity — reads as
      // a cast shadow. The shadow's depth is computed from its own
      // projected vertices (it's on the ground, so generally further
      // from the camera than the ribbon above it).
      const sAvgZr=(sLeft[i][2]+sLeft[i+1][2]+sRight[i+1][2]+sRight[i][2])/4;
      const sPts=
        sLeft[i][0].toFixed(1)+','+sLeft[i][1].toFixed(1)+' '+
        sLeft[i+1][0].toFixed(1)+','+sLeft[i+1][1].toFixed(1)+' '+
        sRight[i+1][0].toFixed(1)+','+sRight[i+1][1].toFixed(1)+' '+
        sRight[i][0].toFixed(1)+','+sRight[i][1].toFixed(1);
      allQuads.push({pts:sPts, color:col, opacity:0.2, avgZr:sAvgZr});
    }
  }
  // Depth-sort: far (large zr) first, near (small/negative zr) last.
  // This is the painter's algorithm — draw far things first so near
  // things paint over them. Critical for path crossings.
  allQuads.sort(function(a,b){return b.avgZr-a.avgZr;});
  for(const q of allQuads){
    svg+=el('polygon',{points:q.pts,fill:q.color,stroke:q.color,'stroke-width':0.5,opacity:q.opacity});
  }
  // Waypoints — rendered with the SAME tadpole drop-line + ground circle
  // as the aircraft playhead, so waypoint altitude and ground position are
  // readable at a glance while orbiting. Without the tadpole, waypoints
  // float in space and it's impossible to tell whether they're at ground
  // level or at altitude.
  //
  // Style matches the aircraft playhead tadpole: dashed vertical line from
  // the waypoint position straight down to z=0, ending in a ring + dot.
  // The waypoint diamond marker sits at the waypoint's true altitude.
  if(tr.waypoints){
    for(let i=0;i<tr.waypoints.length;i++){
      const w=tr.waypoints[i];
      const alt=-w.z; // altitude (positive up); NED z is negative for alt
      const pWp=project(w.x,w.y,alt);
      const pGnd=project(w.x,w.y,0);
      // Tadpole: dashed drop-line from waypoint to ground
      svg+=el('line',{x1:pWp[0].toFixed(1),y1:pWp[1].toFixed(1),
        x2:pGnd[0].toFixed(1),y2:pGnd[1].toFixed(1),
        stroke:'#e2e8f0','stroke-width':1,'stroke-dasharray':'4,3',opacity:0.5});
      // Ground ring + dot (matches aircraft tadpole)
      svg+=el('circle',{cx:pGnd[0].toFixed(1),cy:pGnd[1].toFixed(1),
        r:5,fill:'none',stroke:'#e2e8f0','stroke-width':1.5,opacity:0.55});
      svg+=el('circle',{cx:pGnd[0].toFixed(1),cy:pGnd[1].toFixed(1),
        r:1.5,fill:'#e2e8f0',opacity:0.7});
      // Waypoint marker (diamond) at altitude
      svg+=el('polygon',{points:diamondPts(pWp[0],pWp[1],7),fill:'#fff',stroke:'#000','stroke-width':1.5});
      // Label
      svg+=el('text',{x:pWp[0]+10,y:pWp[1]-8,fill:'#fff','font-weight':'bold','font-size':11},esc(w.name||('WP'+(i+1))));
    }
  }
  // Threats — render the current frame's threats at their projected 3D
  // position. Moving entities (missile, lead, wingman) get a sphere + a
  // dashed tadpole drop-line to the ground (matching the waypoint tadpole).
  // Static markers (slot, target, guns, airbase) get their own marker shape.
  // The 3D view re-renders every frame during playback, so only the current
  // frame's threats are projected — no trail collection across frames.
  // Color scheme matches threatsSvg() (2D top-down) for consistency.
  if(acFrame&&acFrame.threats&&acFrame.threats.length){
    const tStyle={
      missile:{color:'#f44336',r:6,label:'missile'},
      lead:{color:'#4caf50',r:6,label:'lead'},
      wingman:{color:'#00e5ff',r:5,label:'wingman'},
      slot:{color:'#2196f3',r:5,label:'slot'},
      guns:{color:'#ff9800',r:5,label:'guns'},
      target:{color:'#9c27b0',r:5,label:'target'},
      airbase:{color:'#ffc107',r:6,label:'airbase'},
    };
    for(const th of acFrame.threats){
      const st=tStyle[th.type]; if(!st)continue;
      const alt=-th.z; // NED z is negative for altitude
      const pTh=project(th.x,th.y,alt);
      const pGnd=project(th.x,th.y,0);
      // Tadpole drop-line for all threats (so altitude is readable in 3D)
      svg+=el('line',{x1:pTh[0].toFixed(1),y1:pTh[1].toFixed(1),
        x2:pGnd[0].toFixed(1),y2:pGnd[1].toFixed(1),
        stroke:st.color,'stroke-width':1,'stroke-dasharray':'3,2',opacity:0.5});
      // Ground ring + dot (color-coded by threat type)
      svg+=el('circle',{cx:pGnd[0].toFixed(1),cy:pGnd[1].toFixed(1),
        r:4,fill:'none',stroke:st.color,'stroke-width':1.5,opacity:0.55});
      svg+=el('circle',{cx:pGnd[0].toFixed(1),cy:pGnd[1].toFixed(1),
        r:1.2,fill:st.color,opacity:0.7});
      // Marker at the projected 3D position. Use distinct shapes per type
      // so they're visually distinguishable even when colors overlap:
      //   missile/lead/wingman → filled circle (moving entities)
      //   target → square (target box)
      //   slot  → diamond (formation slot)
      //   guns  → inverted triangle (ground gun)
      //   airbase → larger square (runway box)
      if(th.type==='target'||th.type==='airbase'){
        const s=st.r;
        svg+=el('rect',{x:(pTh[0]-s).toFixed(1),y:(pTh[1]-s).toFixed(1),
          width:(s*2).toFixed(1),height:(s*2).toFixed(1),
          fill:st.color,stroke:'#fff','stroke-width':1,opacity:0.9});
      }else if(th.type==='slot'){
        svg+=el('polygon',{points:diamondPts(pTh[0],pTh[1],st.r),
          fill:'none',stroke:st.color,'stroke-width':1.5,opacity:0.9});
      }else if(th.type==='guns'){
        const s=st.r;
        svg+=el('polygon',{points:
          (pTh[0]).toFixed(1)+','+(pTh[1]-s).toFixed(1)+' '+
          (pTh[0]+s).toFixed(1)+','+(pTh[1]+s).toFixed(1)+' '+
          (pTh[0]-s).toFixed(1)+','+(pTh[1]+s).toFixed(1),
          fill:st.color,stroke:'#fff','stroke-width':1,opacity:0.9});
      }else{
        // missile, lead, wingman: filled circle
        svg+=el('circle',{cx:pTh[0].toFixed(1),cy:pTh[1].toFixed(1),
          r:st.r,fill:st.color,stroke:'#fff','stroke-width':1,opacity:0.9});
      }
      // Label (offset to the right of the marker)
      svg+=el('text',{x:(pTh[0]+st.r+3).toFixed(1),y:(pTh[1]+4).toFixed(1),
        fill:st.color,'font-size':10,'font-weight':'bold'},st.label);
    }
  }
  // Playhead marker (updated by updatePlayhead to include 3D aircraft model)
  svg+='<g id="td3-playhead"></g>';
  svg+=el('text',{x:12,y:H-8,fill:'#5a6076','font-size':11},'drag to orbit \u00b7 scroll to zoom \u00b7 centered on aircraft');
  svg+='</svg>';
  host.innerHTML=svg;
  detail3DProject=project;
}

)HTML";
// kHtmlTail3c — split from kHtmlTail3b so each raw string literal stays
// under MSVC's 16380-char limit (error C2026). This piece holds the
// criteria panel, detail view assembly, playback, routing, and controls.
static const char* kHtmlTail3c = R"HTML(
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
    // Failure reason — shown only for failed phases with a non-empty reason.
    // Rendered as a second line inside the criteria-text cell with a red
    // accent (left border + tinted background) so it stands out.
    let critCell='<td class="criteria-text">'+esc(p.criteria||'\u2014');
    if(!p.passed&&!p.skipped&&p.failureReason){
      critCell+='<div class="failure-reason">\u26a0 <span>'+esc(p.failureReason)+'</span></div>';
    }
    critCell+='</td>';
    html+='<tr data-phase="'+i+'" data-seg="'+segOf+'" style="cursor:pointer;'+(inActiveSeg?'':'opacity:0.5')+'">'+
      '<td>'+(i+1)+'</td><td class="name">'+esc(p.name)+'</td>'+
      '<td class="status"><span class="badge '+cls+'">'+res+'</span></td>'+
      critCell+'</tr>';
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
        // Clamp the playhead into the newly selected segment's time
        // window so the scrubber and graphs stay in sync.
        const [sMin,sMax]=segmentTimeRange(tr,state.segmentIdx);
        if(state.t<sMin)state.t=sMin;
        if(state.t>sMax)state.t=sMax;
        renderCriteria(tr);
        renderPathView(tr);
        renderTimeSeries(tr);
        updatePlayhead();
      });
    }
  });
}

// ---------------------------------------------------------------------------
// Event log panel — shows discrete events (mode changes, weapon fires, etc.)
// in the current segment as a scrollable list. Auto-scrolls to the event
// closest to the current playhead time on each updatePlayhead().
// ---------------------------------------------------------------------------
function renderEventLog(tr){
  const panel=document.getElementById('event-log-panel');
  const host=document.getElementById('event-log');
  const countEl=document.getElementById('event-log-count');
  if(!panel||!host)return;
  if(!tr.events||!tr.events.length){
    panel.classList.add('hidden');
    host.innerHTML='';
    if(countEl)countEl.textContent='';
    return;
  }
  panel.classList.remove('hidden');
  // Filter events to the active segment's time window
  const [segMinT,segMaxT]=segmentTimeRange(tr,state.segmentIdx);
  const events=tr.events.filter(e=>e.t>=segMinT&&e.t<=segMaxT);
  if(countEl)countEl.textContent='('+events.length+' in segment)';
  if(!events.length){
    host.innerHTML='<div class="event-log-empty">No events in this segment.</div>';
    return;
  }
  // Render rows. Track which row index is closest to the playhead so we can
  // auto-scroll to it (and highlight it).
  let html='';
  let bestIdx=-1, bestDt=Infinity;
  for(let i=0;i<events.length;i++){
    const e=events[i];
    const sev=e.severity||'info';
    const dt=Math.abs(e.t-state.t);
    if(dt<bestDt){bestDt=dt;bestIdx=i;}
    html+='<div class="event-row sev-'+sev+(i===bestIdx?' current':'')+'" data-t="'+e.t.toFixed(3)+'">'+
      '<span class="ev-time">'+fmt(e.t,1)+'s</span>'+
      '<span class="ev-cat">'+esc(e.category||'info')+'</span>'+
      '<span class="ev-msg">'+esc(e.message||'')+'</span>'+
      '</div>';
  }
  host.innerHTML=html;
  // Auto-scroll to the current event row (closest to playhead)
  if(bestIdx>=0){
    const row=host.children[bestIdx];
    if(row){
      // Scroll into view only if not already visible (avoids fighting the
      // user's manual scroll).
      const cTop=host.scrollTop, cBot=cTop+host.clientHeight;
      const rTop=row.offsetTop, rBot=rTop+row.offsetHeight;
      if(rTop<cTop||rBot>cBot){
        // Center the current row in the visible area
        host.scrollTop=rTop-host.clientHeight/2+row.offsetHeight/2;
      }
    }
  }
  // Click an event row → jump the playhead to that event's time
  host.querySelectorAll('.event-row').forEach(row=>{
    row.addEventListener('click',()=>{
      state.t=+row.dataset.t;
      updatePlayhead();
    });
    row.style.cursor='pointer';
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
  renderEventLog(tr);
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
  // If the playhead is at the end of the segment, rewind to the start
  // so pressing Play always shows motion.
  const tr=TRACES[state.traceIdx];
  if(tr){
    const [segMinT,segMaxT]=segmentTimeRange(tr,state.segmentIdx);
    if(state.t>=segMaxT-0.01)state.t=segMinT;
  }
  if(state.playTimer)clearInterval(state.playTimer);
  state.playTimer=setInterval(()=>{
    const tr=TRACES[state.traceIdx]; if(!tr)return;
    state.t+=dt*state.speed;
    // Stop at the end of the active segment (not the full trace).
    const [segMinT,segMaxT]=segmentTimeRange(tr,state.segmentIdx);
    if(state.t>=segMaxT){state.t=segMaxT;stopPlay();}
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
  // Scrubber spans only the active segment's time window.
  const [segMinT,segMaxT]=segmentTimeRange(tr,state.segmentIdx);
  state.t=segMinT+(+e.target.value/1000)*(segMaxT-segMinT);
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
    // Yaw: drag right → camera orbits right (world appears to rotate left).
    // Pitch: drag UP (dy<0) → pitch increases → camera tilts UP from
    //   top-down toward horizontal (you see more of the horizon).
    //   drag DOWN (dy>0) → pitch decreases → camera tilts DOWN toward
    //   top-down, then past it to a slight belly view (negative pitch).
    //   This is the camera-centric Three.js OrbitControls convention.
    //   pitch=0 looks straight DOWN, pitch=π/2 looks horizontal.
    state.orbit.yaw+=dx*0.01;
    state.orbit.pitch-=dy*0.01;
    // Clamp pitch to [-0.35, 1.45]: -0.35 is a slight belly view (camera
    // just below the aircraft looking up), 1.45 is just below horizontal.
    // Going beyond ±π/2 breaks the perspective projection (zr approaches
    // -d, sending the scale factor f to infinity).
    state.orbit.pitch=Math.max(-0.35,Math.min(1.45,state.orbit.pitch));
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
    const [sMin]=segmentTimeRange(tr,state.segmentIdx);
    state.t=Math.max(sMin,state.t-1);updatePlayhead();
  }else if(e.code==='ArrowRight'){
    const tr=TRACES[state.traceIdx];
    const [,sMax]=segmentTimeRange(tr,state.segmentIdx);
    state.t=Math.min(sMax,state.t+1);updatePlayhead();
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
    out << kHtmlTail1 << kHtmlTail1b << kHtmlTail2 << kHtmlTail3 << kHtmlTail3b << kHtmlTail3c << kHtmlTail4;
}

} // namespace f4flight
