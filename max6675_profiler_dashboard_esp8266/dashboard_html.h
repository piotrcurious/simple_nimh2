#ifndef DASHBOARD_HTML_H
#define DASHBOARD_HTML_H

#ifndef MOCK_TEST
#include <Arduino.h>
#endif

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>MAX6675 Profiler Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <style>
    :root {
      --bg0: #02040a;
      --bg1: #07111d;
      --panel: rgba(8, 16, 28, 0.78);
      --text: #dbffff;
      --muted: #8acfd8;
      --cyan: #00f7ff;
      --mag: #ff4dff;
      --yel: #ffe86a;
      --red: #ff6c7a;
      --grn: #58ff98;
      --blu: #38bdf8;
    }

    html, body {
      margin: 0;
      padding: 0;
      background:
        radial-gradient(circle at 50% 0%, rgba(0, 255, 255, 0.08), transparent 28%),
        radial-gradient(circle at 80% 100%, rgba(255, 0, 255, 0.06), transparent 24%),
        linear-gradient(180deg, var(--bg1), var(--bg0));
      color: var(--text);
      font-family: Consolas, "Liberation Mono", Menlo, monospace;
      overflow-x: hidden;
      padding-bottom: 24px;
    }

    .wrap {
      width: min(100%, 1400px);
      margin: 0 auto;
      padding: 8px;
      box-sizing: border-box;
      display: flex;
      flex-direction: column;
      gap: 12px;
    }

    /* Profiler wrap styling with absolute lane labels */
    #profiler-wrap {
      width: 100%;
      height: 70px;
      background: #0b0f14;
      border-bottom: 1px solid rgba(0, 255, 255, 0.15);
      position: relative;
      overflow: hidden;
    }

    #gl {
      display: block;
      width: 100%;
      height: 100%;
    }

    #profiler-status {
      position: absolute;
      top: 2px;
      right: 8px;
      font-size: 9px;
      color: var(--muted);
      pointer-events: none;
      z-index: 10;
    }

    .lane-label {
      position: absolute;
      left: 6px;
      font-size: 8px;
      font-weight: bold;
      color: rgba(138, 207, 216, 0.5);
      text-transform: uppercase;
      pointer-events: none;
      z-index: 5;
    }
    #lane-label-0 { top: 12px; }
    #lane-label-1 { top: 40px; }

    /* Header & WebSocket Connection Status */
    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid rgba(0, 255, 255, 0.14);
      padding-bottom: 8px;
      margin-bottom: 4px;
    }

    .titleWrap {
      display: flex;
      flex-direction: column;
    }

    .mainTitle {
      font-size: 16px;
      font-weight: bold;
      color: var(--cyan);
      text-shadow: 0 0 8px rgba(0, 255, 255, 0.3);
      letter-spacing: 0.05em;
    }

    .subtitle {
      font-size: 9px;
      color: var(--muted);
      margin-top: 2px;
    }

    .wsStatusWrap {
      display: flex;
      align-items: center;
      gap: 8px;
      background: rgba(2, 6, 11, 0.6);
      border: 1px solid rgba(0, 255, 255, 0.15);
      padding: 4px 8px;
      border-radius: 6px;
    }

    .wsIndicator {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--red);
      box-shadow: 0 0 8px var(--red);
      transition: background 0.3s, box-shadow 0.3s;
    }

    .wsIndicator.connected {
      background: var(--grn);
      box-shadow: 0 0 10px var(--grn);
      animation: pulse 1.8s infinite alternate;
    }

    @keyframes pulse {
      0% { opacity: 0.6; box-shadow: 0 0 4px var(--grn); }
      100% { opacity: 1; box-shadow: 0 0 12px var(--grn); }
    }

    .wsLabel {
      font-size: 10px;
      font-weight: bold;
      text-transform: uppercase;
    }

    .card, .panel {
      background: var(--panel);
      border: 1px solid rgba(0, 255, 255, 0.14);
      border-radius: 9px;
      box-shadow:
        0 0 0 1px rgba(0, 255, 255, 0.03) inset,
        0 0 10px rgba(0, 255, 255, 0.07);
      padding: 12px;
      position: relative;
    }

    .panelTitle {
      font-size: 11px;
      font-weight: bold;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      color: #eaffff;
      margin-bottom: 8px;
    }

    .barsContainer {
      display: flex;
      justify-content: space-around;
      align-items: flex-end;
      height: 140px;
      border-bottom: 1px solid rgba(0, 255, 255, 0.15);
      padding-bottom: 4px;
    }

    .barWrap {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: flex-end;
      width: 22%;
      height: 100%;
    }

    .bar {
      width: 100%;
      min-height: 2px;
      border-radius: 4px 4px 0 0;
      transition: height 0.2s ease, background 0.2s ease;
      position: relative;
    }

    .barValue {
      font-size: 11px;
      font-weight: bold;
      margin-bottom: 4px;
    }

    .barLabel {
      font-size: 9px;
      color: var(--muted);
      margin-top: 6px;
      text-transform: uppercase;
    }

    .row {
      display: grid;
      grid-template-columns: 1fr;
      gap: 12px;
    }

    .selectorWrap {
      display: flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 12px;
    }

    select {
      background: #02060b;
      color: var(--text);
      border: 1px solid rgba(0, 255, 255, 0.3);
      border-radius: 4px;
      padding: 4px 8px;
      font-family: inherit;
      font-size: 11px;
      cursor: pointer;
    }

    canvas.graph {
      width: 100%;
      height: 250px;
      display: block;
      border-radius: 7px;
      background:
        linear-gradient(rgba(0, 255, 255, 0.02) 1px, transparent 1px),
        linear-gradient(90deg, rgba(0, 255, 255, 0.02) 1px, transparent 1px),
        #02060b;
      background-size: 20px 20px;
    }

    .legend {
      display: flex;
      gap: 12px;
      font-size: 9px;
      margin-top: 8px;
      flex-wrap: wrap;
    }

    .legendItem {
      display: flex;
      align-items: center;
      gap: 4px;
    }

    .dot {
      width: 6px;
      height: 6px;
      border-radius: 50%;
    }
  </style>
</head>
<body>
  <!-- System Profiler Panel with Realtime Overlays -->
  <div id="profiler-wrap">
    <div id="lane-label-0" class="lane-label">Core 0</div>
    <div id="lane-label-1" class="lane-label">Core 1</div>
    <div id="profiler-status">profiler idle</div>
    <canvas id="gl"></canvas>
  </div>

  <div class="wrap">
    <!-- Header with WS connection info -->
    <div class="header">
      <div class="titleWrap">
        <div class="mainTitle">MAX6675 Thermocouple Monitor</div>
        <div class="subtitle">Real-time scheduled nonblocking telemetry system</div>
      </div>
      <div class="wsStatusWrap">
        <div id="wsIndicator" class="wsIndicator"></div>
        <div id="wsLabel" class="wsLabel">Disconnected</div>
      </div>
    </div>

    <!-- Profiler Color Map Legend -->
    <div class="card" style="padding: 8px 12px;">
      <div class="legend" style="margin-top: 0;">
        <span style="font-size: 9px; font-weight: bold; text-transform: uppercase; color: var(--muted);">Profiler Legend:</span>
        <div class="legendItem"><div class="dot" style="background: rgb(94, 199, 255);"></div>Task 0: Main Loop Idle</div>
        <div class="legendItem"><div class="dot" style="background: rgb(255, 181, 94);"></div>Task 1: ReadMAX6675 (Core 0)</div>
        <div class="legendItem"><div class="dot" style="background: rgb(135, 255, 135);"></div>Task 2: WebServer / Telemetry (Core 1)</div>
      </div>
    </div>

    <div class="row">
      <!-- Auto-Scaled Relative Bars -->
      <div class="card">
        <div class="panelTitle">Relative Temperature (Min-Max Auto-Scaled)</div>
        <div class="barsContainer" id="relBars">
          <div class="barWrap">
            <div class="barValue" id="valRel0">--</div>
            <div class="bar" id="barRel0" style="background: var(--cyan); height: 0%;"></div>
            <div class="barLabel">T1</div>
          </div>
          <div class="barWrap">
            <div class="barValue" id="valRel1">--</div>
            <div class="bar" id="barRel1" style="background: var(--mag); height: 0%;"></div>
            <div class="barLabel">T2</div>
          </div>
          <div class="barWrap">
            <div class="barValue" id="valRel2">--</div>
            <div class="bar" id="barRel2" style="background: var(--yel); height: 0%;"></div>
            <div class="barLabel">T3</div>
          </div>
          <div class="barWrap">
            <div class="barValue" id="valRel3">--</div>
            <div class="bar" id="barRel3" style="background: var(--red); height: 0%;"></div>
            <div class="barLabel">T4</div>
          </div>
        </div>
        <div style="font-size: 9px; color: var(--muted); margin-top: 8px;" id="relScaleInfo">
          Scaling range: --
        </div>
      </div>

      <!-- Absolute Bars -->
      <div class="card">
        <div class="panelTitle">Absolute Temperature (0-1000°C Range)</div>
        <div class="barsContainer">
          <div class="barWrap">
            <div class="barValue" id="valAbs0">--</div>
            <div class="bar" id="barAbs0" style="background: var(--cyan); height: 0%;"></div>
            <div class="barLabel">T1</div>
          </div>
          <div class="barWrap">
            <div class="barValue" id="valAbs1">--</div>
            <div class="bar" id="barAbs1" style="background: var(--mag); height: 0%;"></div>
            <div class="barLabel">T2</div>
          </div>
          <div class="barWrap">
            <div class="barValue" id="valAbs2">--</div>
            <div class="bar" id="barAbs2" style="background: var(--yel); height: 0%;"></div>
            <div class="barLabel">T3</div>
          </div>
          <div class="barWrap">
            <div class="barValue" id="valAbs3">--</div>
            <div class="bar" id="barAbs3" style="background: var(--red); height: 0%;"></div>
            <div class="barLabel">T4</div>
          </div>
        </div>
        <div style="font-size: 9px; color: var(--muted); margin-top: 8px;">
          Fixed range: 0°C to 1000°C
        </div>
      </div>
    </div>

    <!-- Rolling Combined Graph Section -->
    <div class="card">
      <div class="panelTitle">Rolling Combined Graph (200-800°C Range)</div>
      <div class="selectorWrap">
        <span style="font-size: 11px;">Buffer Size:</span>
        <select id="bufferSizeSelect" onchange="updateBufferSize()">
          <option value="360" selected>360 Points</option>
          <option value="640">640 Points</option>
        </select>
        <span id="timeTickInfo" style="font-size: 11px; color: var(--muted);">Tick: 1s</span>
      </div>
      <canvas class="graph" id="combinedGraph"></canvas>
      <div class="legend">
        <div class="legendItem"><div class="dot" style="background: var(--cyan);"></div>T1</div>
        <div class="legendItem"><div class="dot" style="background: var(--mag);"></div>T2</div>
        <div class="legendItem"><div class="dot" style="background: var(--yel);"></div>T3</div>
        <div class="legendItem"><div class="dot" style="background: var(--red);"></div>T4</div>
      </div>
    </div>
  </div>

  <script>
    // --- State and History Buffer ---
    let maxBufferSize = 360;
    let t1_history = [];
    let t2_history = [];
    let t3_history = [];
    let t4_history = [];

    // --- Update WS Connection UI Status ---
    function updateWSStatus(connected, text) {
      const indicator = document.getElementById('wsIndicator');
      const label = document.getElementById('wsLabel');
      if (connected) {
        indicator.classList.add('connected');
        label.innerText = text || 'Connected';
        label.style.color = 'var(--grn)';
      } else {
        indicator.classList.remove('connected');
        label.innerText = text || 'Disconnected';
        label.style.color = 'var(--red)';
      }
    }

    // --- WebSocket Connection ---
    let ws = null;
    function connectWS() {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const url = `${protocol}//${window.location.host}/ws`;
      ws = new WebSocket(url);
      ws.binaryType = 'arraybuffer';

      ws.onopen = () => {
        console.log('WS Connected');
        updateWSStatus(true, 'Live');
      };

      ws.onmessage = (e) => {
        if (e.data instanceof ArrayBuffer) {
          const bytes = new Uint8Array(e.data);
          // Strict Signature Validation: Binary frames must carry "TP" LE (0x50, 0x54) magic header
          if (bytes.length >= 18 && bytes[0] === 0x50 && bytes[1] === 0x54) {
            decodeProfilerFrame(e.data);
          } else {
            console.warn('Unknown binary frame signature');
          }
        } else {
          try {
            const telemetry = JSON.parse(e.data);
            handleTelemetry(telemetry);
          } catch (err) {
            console.error('Error parsing JSON:', err);
          }
        }
      };

      ws.onclose = () => {
        console.log('WS Disconnected, retrying...');
        updateWSStatus(false, 'Retry...');
        setTimeout(connectWS, 2000);
      };

      ws.onerror = () => {
        updateWSStatus(false, 'Error');
      };
    }

    // --- Handle Telemetry Update ---
    function handleTelemetry(data) {
      const temps = [data.t1, data.t2, data.t3, data.t4];
      const errs = [data.e1, data.e2, data.e3, data.e4];

      // Update Absolute Bars
      for (let i = 0; i < 4; i++) {
        const valAbsEl = document.getElementById(`valAbs${i}`);
        const barAbsEl = document.getElementById(`barAbs${i}`);
        if (errs[i]) {
          valAbsEl.innerText = "ERR";
          barAbsEl.style.height = "0%";
        } else {
          valAbsEl.innerText = `${temps[i].toFixed(1)}°C`;
          const absPct = Math.min(100, Math.max(0, (temps[i] / 1000) * 100));
          barAbsEl.style.height = `${absPct}%`;
        }
      }

      // Update Relative (Auto-Scaled) Bars
      const validTemps = temps.filter((_, i) => !errs[i]);
      if (validTemps.length > 0) {
        const minTemp = Math.min(...validTemps);
        const maxTemp = Math.max(...validTemps);
        const tempRange = (maxTemp - minTemp) || 1.0;

        document.getElementById('relScaleInfo').innerText =
          `Scaling range: ${minTemp.toFixed(1)}°C to ${maxTemp.toFixed(1)}°C`;

        for (let i = 0; i < 4; i++) {
          const valRelEl = document.getElementById(`valRel${i}`);
          const barRelEl = document.getElementById(`barRel${i}`);
          if (errs[i]) {
            valRelEl.innerText = "ERR";
            barRelEl.style.height = "0%";
          } else {
            valRelEl.innerText = `${temps[i].toFixed(1)}°C`;
            const relPct = Math.min(100, Math.max(0, ((temps[i] - minTemp) / tempRange) * 100));
            barRelEl.style.height = `${relPct}%`;
          }
        }
      } else {
        document.getElementById('relScaleInfo').innerText = "Scaling range: --";
        for (let i = 0; i < 4; i++) {
          document.getElementById(`valRel${i}`).innerText = "--";
          document.getElementById(`barRel${i}`).style.height = "0%";
        }
      }

      // Append to graph history at 1s intervals
      t1_history.push(errs[0] ? null : data.t1);
      t2_history.push(errs[1] ? null : data.t2);
      t3_history.push(errs[2] ? null : data.t3);
      t4_history.push(errs[3] ? null : data.t4);

      while (t1_history.length > maxBufferSize) {
        t1_history.shift();
        t2_history.shift();
        t3_history.shift();
        t4_history.shift();
      }

      renderGraph();
    }

    // --- Buffer Size Control ---
    function updateBufferSize() {
      const sel = document.getElementById('bufferSizeSelect');
      maxBufferSize = parseInt(sel.value);
      while (t1_history.length > maxBufferSize) {
        t1_history.shift();
        t2_history.shift();
        t3_history.shift();
        t4_history.shift();
      }
      renderGraph();
    }

    // --- Render Combined Rolling Graph ---
    function renderGraph() {
      const canvas = document.getElementById('combinedGraph');
      const ctx = canvas.getContext('2d');
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();

      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      ctx.scale(dpr, dpr);

      const w = rect.width;
      const h = rect.height;
      const margin = { top: 20, right: 20, bottom: 24, left: 40 };
      const plotW = w - margin.left - margin.right;
      const plotH = h - margin.top - margin.bottom;

      ctx.clearRect(0, 0, w, h);

      // Draw Grid and Y Axis Labels (200C - 800C)
      ctx.strokeStyle = "rgba(0, 255, 255, 0.08)";
      ctx.lineWidth = 1;
      ctx.fillStyle = "rgba(138, 207, 216, 0.7)";
      ctx.font = "9px Consolas, monospace";

      for (let temp = 200; temp <= 800; temp += 100) {
        const y = margin.top + plotH - ((temp - 200) / 600) * plotH;
        ctx.beginPath();
        ctx.moveTo(margin.left, y);
        ctx.lineTo(w - margin.right, y);
        ctx.stroke();

        ctx.textAlign = "right";
        ctx.fillText(`${temp}°C`, margin.left - 6, y + 3);
      }

      // Draw Series Lines (Glowing style)
      const drawSeries = (history, color) => {
        if (history.length < 2) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.shadowColor = color;
        ctx.shadowBlur = 4;
        ctx.beginPath();

        let started = false;
        for (let i = 0; i < history.length; i++) {
          const v = history[i];
          if (v === null || v < 200 || v > 800) continue;

          const x = margin.left + (i / (maxBufferSize - 1)) * plotW;
          const y = margin.top + plotH - ((v - 200) / 600) * plotH;

          if (!started) {
            ctx.moveTo(x, y);
            started = true;
          } else {
            ctx.lineTo(x, y);
          }
        }
        ctx.stroke();
        ctx.shadowBlur = 0; // reset
      };

      drawSeries(t1_history, "var(--cyan)");
      drawSeries(t2_history, "var(--mag)");
      drawSeries(t3_history, "var(--yel)");
      drawSeries(t4_history, "var(--red)");
    }

    // --- WebGL Profiler Decoder & Render ---
    const profilerCanvas = document.getElementById('gl');
    const profilerStatus = document.getElementById('profiler-status');
    let p_gl, p_program, p_buffer, p_aPos, p_aColor, p_uRes;
    const FRAME_US_DEFAULT = 100000;
    const LANE_H = 10;
    const LANE_TOP = 8;
    const LANE_GAP = 14;

    function initProfilerGL() {
      p_gl = profilerCanvas.getContext('webgl', { antialias: false, alpha: false });
      if (!p_gl) return;

      const compile = (type, src) => {
        const sh = p_gl.createShader(type);
        p_gl.shaderSource(sh, src);
        p_gl.compileShader(sh);
        return sh;
      };

      const vs = compile(p_gl.VERTEX_SHADER, `
        attribute vec2 a_pos;
        attribute vec4 a_color;
        uniform vec2 u_res;
        varying vec4 v_color;
        void main() {
          vec2 zeroToOne = a_pos / u_res;
          vec2 clip = (zeroToOne * 2.0) - 1.0;
          gl_Position = vec4(clip * vec2(1.0, -1.0), 0.0, 1.0);
          v_color = a_color;
        }
      `);

      const fs = compile(p_gl.FRAGMENT_SHADER, `
        precision mediump float;
        varying vec4 v_color;
        void main() { gl_FragColor = v_color; }
      `);

      p_program = p_gl.createProgram();
      p_gl.attachShader(p_program, vs);
      p_gl.attachShader(p_program, fs);
      p_gl.linkProgram(p_program);
      p_gl.useProgram(p_program);

      p_buffer = p_gl.createBuffer();
      p_gl.bindBuffer(p_gl.ARRAY_BUFFER, p_buffer);
      p_aPos = p_gl.getAttribLocation(p_program, 'a_pos');
      p_aColor = p_gl.getAttribLocation(p_program, 'a_color');
      p_uRes = p_gl.getUniformLocation(p_program, 'u_res');

      p_gl.enableVertexAttribArray(p_aPos);
      p_gl.enableVertexAttribArray(p_aColor);
      p_gl.enable(p_gl.BLEND);
      p_gl.blendFunc(p_gl.SRC_ALPHA, p_gl.ONE_MINUS_SRC_ALPHA);
      p_gl.clearColor(0.05, 0.07, 0.10, 1.0);
    }

    function colorForTask(taskId) {
      const palette = [
        [0.37, 0.78, 1.00, 0.9], // Task 0: Blue
        [1.00, 0.71, 0.37, 0.9], // Task 1: Orange
        [0.53, 1.00, 0.53, 0.9], // Task 2: Green
        [1.00, 0.42, 0.62, 0.9]
      ];
      return palette[taskId % palette.length];
    }

    function pushRect(arr, x1, y1, x2, y2, r, g, b, a) {
      arr.push(x1,y1,r,g,b,a, x2,y1,r,g,b,a, x1,y2,r,g,b,a, x1,y2,r,g,b,a, x2,y1,r,g,b,a, x2,y2,r,g,b,a);
    }

    function drawProfilerFrame(frame) {
      if (!p_gl) return;
      const w = profilerCanvas.width;
      const h = profilerCanvas.height;
      p_gl.viewport(0, 0, w, h);
      p_gl.clear(p_gl.COLOR_BUFFER_BIT);
      p_gl.useProgram(p_program);
      p_gl.uniform2f(p_uRes, w, h);

      const verts = [];
      const period = frame.framePeriodUs || FRAME_US_DEFAULT;

      // Background rect
      pushRect(verts, 0, 0, w, h, 0.06, 0.08, 0.11, 1.0);

      // Grid every 10ms
      for (let ms = 10; ms < period / 1000; ms += 10) {
        const x = (ms * 1000 / period) * w;
        pushRect(verts, x, 0, x + 1, h, 1, 1, 1, 0.05);
      }

      for (let core = 0; core < 2; core++) {
        const laneTop = LANE_TOP + core * (LANE_H + LANE_GAP);
        const laneBottom = laneTop + LANE_H;
        for (const ev of frame.lanes[core]) {
          let x1 = (ev.startUs / period) * w;
          let x2 = ((ev.startUs + ev.durUs) / period) * w;
          const c = colorForTask(ev.taskId);
          pushRect(verts, x1, laneTop, x2, laneBottom, c[0], c[1], c[2], c[3]);
        }
      }

      p_gl.bindBuffer(p_gl.ARRAY_BUFFER, p_buffer);
      p_gl.bufferData(p_gl.ARRAY_BUFFER, new Float32Array(verts), p_gl.STREAM_DRAW);
      p_gl.vertexAttribPointer(p_aPos, 2, p_gl.FLOAT, false, 24, 0);
      p_gl.vertexAttribPointer(p_aColor, 4, p_gl.FLOAT, false, 24, 8);
      p_gl.drawArrays(p_gl.TRIANGLES, 0, verts.length / 6);
    }

    function decodeProfilerFrame(arrayBuffer) {
      const dv = new DataView(arrayBuffer);
      const framePeriodUs = dv.getUint16(12, true);
      const c0 = dv.getUint8(14);
      const c1 = dv.getUint8(15);
      let off = 18;
      const lanes = [[], []];
      for (let core = 0; core < 2; core++) {
        const count = core === 0 ? c0 : c1;
        for (let i = 0; i < count; i++) {
          lanes[core].push({
            taskId: dv.getUint8(off),
            flags: dv.getUint8(off + 1),
            startUs: dv.getUint16(off + 2, true),
            durUs: dv.getUint16(off + 4, true)
          });
          off += 6;
        }
      }
      const frame = { framePeriodUs, lanes };
      drawProfilerFrame(frame);
      profilerStatus.textContent = `frame ${dv.getUint32(4, true)} | C0:${c0} C1:${c1}`;
    }

    function fitProfiler() {
      const dpr = window.devicePixelRatio || 1;
      const rect = profilerCanvas.getBoundingClientRect();
      profilerCanvas.width = rect.width * dpr;
      profilerCanvas.height = rect.height * dpr;
    }

    window.addEventListener('resize', () => {
      fitProfiler();
      renderGraph();
    });

    // --- Init ---
    fitProfiler();
    initProfilerGL();
    connectWS();
    renderGraph();
  </script>
</body>
</html>
)rawliteral";

#endif // DASHBOARD_HTML_H
