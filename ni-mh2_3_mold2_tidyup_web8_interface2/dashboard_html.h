#ifndef DASHBOARD_HTML_H
#define DASHBOARD_HTML_H

#include <Arduino.h>

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Ni-MH Charger UI</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <style>
    :root{
      --bg0:#02040a;
      --bg1:#07111d;
      --panel:rgba(8,16,28,0.78);
      --text:#dbffff;
      --muted:#8acfd8;
      --cyan:#00f7ff;
      --mag:#ff4dff;
      --yel:#ffe86a;
      --red:#ff6c7a;
      --grn:#58ff98;
      --blu:#58a8ff;
      --org:#ffb05a;
    }

    html, body {
      margin: 0;
      padding: 0;
      background:
        radial-gradient(circle at 50% 0%, rgba(0,255,255,0.08), transparent 28%),
        radial-gradient(circle at 80% 100%, rgba(255,0,255,0.06), transparent 24%),
        linear-gradient(180deg, var(--bg1), var(--bg0));
      color: var(--text);
      font-family: Consolas, "Liberation Mono", Menlo, monospace;
      overflow-x: hidden;
    }

    body::before {
      content: "";
      position: fixed;
      inset: 0;
      pointer-events: none;
      background:
        linear-gradient(to bottom, rgba(255,255,255,0.03), transparent 10%, transparent 90%, rgba(255,255,255,0.025)),
        repeating-linear-gradient(
          to bottom,
          rgba(255,255,255,0.02) 0px,
          rgba(255,255,255,0.015) 1px,
          transparent 2px,
          transparent 4px
        );
      opacity: 0.22;
    }

    .wrap {
      width: min(100%, 100vw);
      margin: 0 auto;
      padding: 4px;
      box-sizing: border-box;
    }

    .card, .panel, .controls {
      background: var(--panel);
      border: 1px solid rgba(0,255,255,0.14);
      border-radius: 9px;
      box-shadow:
        0 0 0 1px rgba(0,255,255,0.03) inset,
        0 0 10px rgba(0,255,255,0.07);
      overflow: hidden;
      position: relative;
    }

    .card::after, .panel::after, .controls::after {
      content: "";
      position: absolute;
      inset: 0;
      pointer-events: none;
      border-radius: 9px;
      box-shadow: inset 0 0 12px rgba(0,255,255,0.035);
    }

    .topGrid {
      display: grid;
      grid-template-columns: minmax(210px, 0.8fr) minmax(0, 1.2fr);
      gap: 4px;
      margin-bottom: 4px;
    }

    .statusBox {
      padding: 6px 8px;
    }

    .statusRow {
      display: flex;
      flex-wrap: wrap;
      gap: 4px 8px;
      align-items: center;
    }

    .statusChip {
      font-size: 11px;
      color: #f4ffff;
      padding: 3px 0;
      white-space: nowrap;
      text-shadow: 0 0 6px rgba(0,255,255,0.14);
    }

    .statusChip.muted {
      color: var(--muted);
      font-size: 9px;
    }

    .controls {
      display: flex;
      flex-wrap: wrap;
      gap: 4px;
      padding: 4px;
      margin-top: 4px;
      margin-bottom: 4px;
    }

    button {
      appearance: none;
      border: 1px solid rgba(0,255,255,0.22);
      background: linear-gradient(180deg, rgba(12,24,40,0.94), rgba(3,8,14,0.98));
      color: #b8ffff;
      border-radius: 7px;
      padding: 7px 9px;
      font: inherit;
      font-size: 10px;
      letter-spacing: 0.04em;
      text-transform: uppercase;
      cursor: pointer;
      box-shadow:
        0 0 0 1px rgba(0,255,255,0.03) inset,
        0 0 7px rgba(0,255,255,0.06);
    }

    button:active {
      background: linear-gradient(180deg, rgba(3,8,14,0.98), rgba(12,24,40,0.94));
    }

    .gaugesBar {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 4px;
    }

    .gaugeCard {
      padding: 4px;
      min-height: 0;
    }

    .gaugeLabel {
      font-size: 8px;
      color: var(--muted);
      opacity: 0.9;
      margin: 0 0 2px 2px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
    }

    canvas.gauge {
      width: 100%;
      height: 72px;
      display: block;
      border-radius: 7px;
      background:
        radial-gradient(circle at 50% 45%, rgba(0,255,255,0.05), transparent 56%),
        linear-gradient(180deg, rgba(0,0,0,0.18), rgba(0,0,0,0.42)),
        #02060b;
    }

    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 4px;
      align-items: start;
      margin-top: 4px;
    }

    .panel {
      padding: 4px;
      min-height: 0;
    }

    .fullWidth {
      grid-column: 1 / -1;
    }

    .smallGraph canvas.graph {
      height: clamp(145px, 18vh, 190px);
    }

    .chargePanel canvas.graph {
      height: clamp(320px, 48vh, 620px);
    }

    .panelHead {
      display: flex;
      justify-content: space-between;
      gap: 5px;
      align-items: baseline;
      margin: 0 0 3px 0;
    }

    .panelTitle {
      font-size: 9px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      color: #eaffff;
    }

    .panelHint {
      font-size: 8px;
      color: var(--muted);
      opacity: 0.9;
      text-align: right;
    }

    canvas.graph {
      width: 100%;
      display: block;
      border-radius: 7px;
      background:
        linear-gradient(rgba(0,255,255,0.028) 1px, transparent 1px),
        linear-gradient(90deg, rgba(0,255,255,0.028) 1px, transparent 1px),
        linear-gradient(180deg, rgba(0,0,0,0.18), rgba(0,0,0,0.42)),
        #02060b;
      background-size: 18px 18px, 18px 18px, auto, auto;
    }

    .legend {
      margin-top: 3px;
      font-size: 8px;
      line-height: 1.3;
      color: #9ce9f4;
      white-space: pre-wrap;
      opacity: 0.92;
    }

    @media (max-width: 900px) {
      .topGrid { grid-template-columns: 1fr; }
      .gaugesBar { grid-template-columns: 1fr 1fr; }
      .grid { grid-template-columns: 1fr; }
    }

    @media (max-width: 640px) {
      .wrap { padding: 3px; }
      .gaugesBar { grid-template-columns: 1fr; }
      .smallGraph canvas.graph { height: 175px; }
      .chargePanel canvas.graph { height: 300px; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="topGrid">
      <div class="card statusBox">
        <div class="statusRow">
          <div class="statusChip" id="appState">IDLE</div>
          <div class="statusChip" id="vValue">0.000 V</div>
          <div class="statusChip" id="iValue">0.000 A</div>
          <div class="statusChip" id="mahValue">0.000 mAh</div>
          <div class="statusChip muted" id="extraState">CBOR live telemetry</div>
        </div>
      </div>

      <div class="card">
        <div class="gaugesBar">
          <div class="gaugeCard">
            <div class="gaugeLabel">Voltage</div>
            <canvas class="gauge" id="gaugeV"></canvas>
          </div>
          <div class="gaugeCard">
            <div class="gaugeLabel">Current</div>
            <canvas class="gauge" id="gaugeI"></canvas>
          </div>
          <div class="gaugeCard">
            <div class="gaugeLabel">Max dT</div>
            <canvas class="gauge" id="gaugeTD"></canvas>
          </div>
          <div class="gaugeCard">
            <div class="gaugeLabel">Duty</div>
            <canvas class="gauge" id="gaugeDuty"></canvas>
          </div>
        </div>
      </div>
    </div>

    <div class="controls">
      <button onclick="sendCommand('charge')">Start Charge</button>
      <button onclick="sendCommand('ir')">Measure IR</button>
      <button onclick="sendCommand('reset')">Reset Ah</button>
      <button onclick="sendCommand('stop')">Stop</button>
    </div>

    <div class="grid">
      <div class="panel smallGraph">
        <div class="panelHead">
          <div class="panelTitle">Main Graph</div>
          <div class="panelHint">V yellow · I magenta · T1 red · T2 green · dT blue</div>
        </div>
        <canvas class="graph" id="mainGraph"></canvas>
      </div>

      <div class="panel smallGraph">
        <div class="panelHead">
          <div class="panelTitle">Ambient / Mold</div>
          <div class="panelHint">T red · Dew green · H blue</div>
        </div>
        <canvas class="graph" id="ambientGraph"></canvas>
      </div>

      <div class="panel smallGraph">
        <div class="panelHead">
          <div class="panelTitle">IR Graph</div>
          <div class="panelHint">LU white · Pairs cyan</div>
        </div>
        <canvas class="graph" id="irGraph"></canvas>
      </div>

      <div class="panel chargePanel fullWidth">
        <div class="panelHead">
          <div class="panelTitle">Charge Log</div>
          <div class="panelHint">Largest area · auto-scaled traces · 1px glowing lines</div>
        </div>
        <canvas class="graph" id="chargeGraph"></canvas>
        <div class="legend" id="chargeLegend"></div>
      </div>
    </div>
  </div>

  <script>
    let maxDT = 1.5;
    let lastHistoryFetch = 0;
    let lastIRFetch = 0;
    let lastChargeFetch = 0;
    let lastLiveDT = NaN;

    const appStates = ['IDLE', 'BUILDING_MODEL', 'MEASURING_IR', 'CHARGING'];
    const modelPhases = ['Idle', 'Settle', 'Calibrate', 'DetectDeadRegion', 'SetDuty', 'WaitMeasurement', 'Finish'];

    // DOM Element Cache to prevent layout thrashing
    const dom = {
      appState: document.getElementById('appState'),
      vValue: document.getElementById('vValue'),
      iValue: document.getElementById('iValue'),
      mahValue: document.getElementById('mahValue'),
      extraState: document.getElementById('extraState'),
      gaugeV: document.getElementById('gaugeV'),
      gaugeI: document.getElementById('gaugeI'),
      gaugeTD: document.getElementById('gaugeTD'),
      gaugeDuty: document.getElementById('gaugeDuty'),
      chargeLegend: document.getElementById('chargeLegend')
    };

    // Reusing a single TextDecoder is much faster than creating one per string block
    const textDecoder = new TextDecoder();

    function sendCommand(cmd) {
      fetch('/command?cmd=' + encodeURIComponent(cmd), { cache: 'no-store' }).catch(console.error);
    }

    async function fetchPayload(url) {
      const res = await fetch(url, { cache: 'no-store' });
      const ct = (res.headers.get('content-type') || '').toLowerCase();
      if (ct.includes('application/cbor')) {
        return decodeCbor(new Uint8Array(await res.arrayBuffer()));
      }
      return await res.json();
    }

    function readUint(ai, view, state) {
      if (ai < 24) return ai;
      if (ai === 24) return view.getUint8(state.pos++);
      if (ai === 25) {
        const v = view.getUint16(state.pos, false); state.pos += 2; return v;
      }
      if (ai === 26) {
        const v = view.getUint32(state.pos, false); state.pos += 4; return v;
      }
      if (ai === 27) {
        const hi = view.getUint32(state.pos, false);
        const lo = view.getUint32(state.pos + 4, false);
        state.pos += 8;
        return hi * 4294967296 + lo;
      }
      throw new Error('Unsupported CBOR length');
    }

    function decodeItem(view, state) {
      const head = view.getUint8(state.pos++);
      const major = head >> 5;
      const ai = head & 0x1f;

      if (major === 0) return readUint(ai, view, state);
      if (major === 1) return -1 - readUint(ai, view, state);

      if (major === 2) {
        const len = readUint(ai, view, state);
        const bytes = new Uint8Array(view.buffer, view.byteOffset + state.pos, len);
        state.pos += len;
        return bytes.slice();
      }

      if (major === 3) {
        const len = readUint(ai, view, state);
        const bytes = new Uint8Array(view.buffer, view.byteOffset + state.pos, len);
        state.pos += len;
        return textDecoder.decode(bytes); // Use cached instance
      }

      if (major === 4) {
        const len = readUint(ai, view, state);
        const arr = [];
        for (let i = 0; i < len; i++) arr.push(decodeItem(view, state));
        return arr;
      }

      if (major === 5) {
        const len = readUint(ai, view, state);
        const obj = {};
        for (let i = 0; i < len; i++) {
          const k = decodeItem(view, state);
          const v = decodeItem(view, state);
          obj[k] = v;
        }
        return obj;
      }

      if (major === 6) {
        readUint(ai, view, state);
        return decodeItem(view, state);
      }

      if (major === 7) {
        if (ai === 20) return false;
        if (ai === 21) return true;
        if (ai === 22) return null;
        if (ai === 23) return undefined;
        if (ai === 25) {
          const half = view.getUint16(state.pos, false); state.pos += 2;
          const s = (half & 0x8000) ? -1 : 1;
          const e = (half >> 10) & 0x1f;
          const f = half & 0x3ff;
          if (e === 0) return s * Math.pow(2, -14) * (f / 1024);
          if (e === 31) return f ? NaN : s * Infinity;
          return s * Math.pow(2, e - 15) * (1 + f / 1024);
        }
        if (ai === 26) {
          const v = view.getFloat32(state.pos, false); state.pos += 4; return v;
        }
        if (ai === 27) {
          const v = view.getFloat64(state.pos, false); state.pos += 8; return v;
        }
        if (ai === 31) throw new Error('Indefinite CBOR not supported');
      }

      throw new Error('Unsupported CBOR major type: ' + major);
    }

    function decodeCbor(bytes) {
      const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      return decodeItem(view, { pos: 0 });
    }

    function clamp(v, lo, hi) {
      return Math.min(hi, Math.max(lo, v));
    }

    function fmt(v, digits = 2) {
      return Number.isFinite(v) ? v.toFixed(digits) : '—';
    }

    function lastFinite(arr) {
      if (!arr || !arr.length) return NaN;
      for (let i = arr.length - 1; i >= 0; --i) {
        const v = arr[i];
        if (Number.isFinite(v)) return v;
      }
      return NaN;
    }

    function autoScale(arr, padRatio = 0.12) {
      let min = Infinity, max = -Infinity;
      for (const v of arr) {
        if (v == null || !Number.isFinite(v)) continue;
        if (v < min) min = v;
        if (v > max) max = v;
      }
      if (min === Infinity) return [0, 1];
      const range = (max - min) || 1;
      return [min - range * padRatio, max + range * padRatio];
    }

    function fitCanvas(canvas) {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const w = Math.max(1, Math.round(rect.width * dpr));
      const h = Math.max(1, Math.round(rect.height * dpr));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }
      return { w, h, dpr };
    }

    function clearCanvas(ctx) {
      ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
    }

    function drawGlowText(ctx, text, x, y, color, align = 'left', size = 11, weight = 'bold') {
      ctx.save();
      ctx.font = `${weight} ${size}px Consolas, monospace`;
      ctx.textAlign = align;
      ctx.fillStyle = color;
      ctx.shadowColor = color;
      ctx.shadowBlur = 8;
      ctx.fillText(text, x, y);
      ctx.restore();
    }

    function drawAxes(ctx, margin, xMin, xMax, xLabel, yMin, yMax, yLabel) {
      const w = ctx.canvas.width;
      const h = ctx.canvas.height;
      const plotW = w - margin.left - margin.right;
      const plotH = h - margin.top - margin.bottom;

      ctx.save();
      ctx.lineWidth = 1;
      ctx.strokeStyle = 'rgba(120,255,255,0.18)';
      ctx.fillStyle = 'rgba(200,255,255,0.72)';
      ctx.font = '10px Consolas, monospace';

      ctx.beginPath();
      ctx.moveTo(margin.left, h - margin.bottom);
      ctx.lineTo(w - margin.right, h - margin.bottom);
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(margin.left, margin.top);
      ctx.lineTo(margin.left, h - margin.bottom);
      ctx.stroke();

      for (let i = 0; i <= 5; i++) {
        const x = margin.left + (i / 5) * plotW;
        const val = xMin + (i / 5) * (xMax - xMin);
        ctx.beginPath();
        ctx.moveTo(x, h - margin.bottom);
        ctx.lineTo(x, h - margin.bottom + 4);
        ctx.stroke();
        ctx.textAlign = 'center';
        ctx.fillText(val.toFixed((xMax - xMin) < 2 ? 2 : 0), x, h - margin.bottom + 13);
      }

      for (let i = 0; i <= 5; i++) {
        const y = h - margin.bottom - (i / 5) * plotH;
        const val = yMin + (i / 5) * (yMax - yMin);
        ctx.beginPath();
        ctx.moveTo(margin.left - 4, y);
        ctx.lineTo(margin.left, y);
        ctx.stroke();
        ctx.textAlign = 'right';
        ctx.fillText(val.toFixed((yMax - yMin) < 2 ? 2 : 0), margin.left - 6, y + 3);
      }

      ctx.textAlign = 'center';
      ctx.fillText(xLabel, margin.left + plotW / 2, h - 2);

      ctx.save();
      ctx.translate(11, margin.top + plotH / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText(yLabel, 0, 0);
      ctx.restore();
      ctx.restore();
    }

    function drawSeries(ctx, arr, color, min, max, margin, label = '', lineWidth = 1) {
      if (!arr || !arr.length) return;

      const w = ctx.canvas.width;
      const h = ctx.canvas.height;
      const plotW = w - margin.left - margin.right;
      const plotH = h - margin.top - margin.bottom;
      const range = (max - min) || 1;

      let first = true;
      let lastX = 0, lastY = 0, lastVal = null;

      ctx.save();
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';

      ctx.beginPath();
      arr.forEach((v, i) => {
        if (v == null || !Number.isFinite(v)) return;
        const x = margin.left + (i / Math.max(1, arr.length - 1)) * plotW;
        const y = (h - margin.bottom) - ((v - min) / range) * plotH;
        if (first) {
          ctx.moveTo(x, y);
          first = false;
        } else {
          ctx.lineTo(x, y);
        }
        lastX = x;
        lastY = y;
        lastVal = v;
      });

      ctx.shadowColor = color;
      ctx.shadowBlur = 10;
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.22;
      ctx.lineWidth = lineWidth + 2;
      ctx.stroke();

      ctx.shadowBlur = 0;
      ctx.globalAlpha = 1.0;
      ctx.lineWidth = lineWidth;
      ctx.stroke();

      if (label && Number.isFinite(lastY)) {
        ctx.fillStyle = color;
        ctx.font = 'bold 10px Consolas, monospace';
        ctx.textAlign = 'left';
        const yPos = clamp(lastY, margin.top + 10, h - margin.bottom - 2);
        ctx.fillText(`${label}:${lastVal.toFixed(2)}`, lastX + 5, yPos);
      }

      ctx.restore();
    }

    function drawXY(ctx, points, color, xMin, xMax, yMin, yMax, margin, label = '') {
      if (!points || !points.length) return;

      const w = ctx.canvas.width;
      const h = ctx.canvas.height;
      const plotW = w - margin.left - margin.right;
      const plotH = h - margin.top - margin.bottom;
      const xRange = (xMax - xMin) || 1;
      const yRange = (yMax - yMin) || 1;

      let lastX = 0, lastY = 0, lastVal = null;
      let first = true;

      ctx.save();
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.beginPath();

      points.forEach((p) => {
        if (!p || p.length < 2) return;
        const px = p[0], py = p[1];
        if (!Number.isFinite(px) || !Number.isFinite(py)) return;
        const x = margin.left + ((px - xMin) / xRange) * plotW;
        const y = (h - margin.bottom) - ((py - yMin) / yRange) * plotH;
        if (first) {
          ctx.moveTo(x, y);
          first = false;
        } else {
          ctx.lineTo(x, y);
        }
        lastX = x;
        lastY = y;
        lastVal = py;
      });

      ctx.shadowColor = color;
      ctx.shadowBlur = 10;
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.22;
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.shadowBlur = 0;
      ctx.globalAlpha = 1.0;
      ctx.lineWidth = 1;
      ctx.stroke();

      if (label && Number.isFinite(lastY)) {
        ctx.fillStyle = color;
        ctx.font = 'bold 10px Consolas, monospace';
        ctx.textAlign = 'left';
        const yPos = clamp(lastY, margin.top + 10, h - margin.bottom - 2);
        ctx.fillText(`${label}:${lastVal.toFixed(2)}`, lastX + 5, yPos);
      }

      ctx.restore();
    }

    function drawRegion(ctx, arr1, arr2, color, min, max, margin) {
      if (!arr1 || !arr1.length || !arr2 || !arr2.length) return;
      const w = ctx.canvas.width;
      const h = ctx.canvas.height;
      const plotW = w - margin.left - margin.right;
      const plotH = h - margin.top - margin.bottom;
      const range = (max - min) || 1;

      ctx.save();
      ctx.fillStyle = color;
      ctx.beginPath();

      let first = true;
      for (let i = 0; i < arr1.length; i++) {
        const v = arr1[i];
        if (v == null || !Number.isFinite(v)) continue;
        const x = margin.left + (i / Math.max(1, arr1.length - 1)) * plotW;
        const y = (h - margin.bottom) - ((v - min) / range) * plotH;
        if (first) {
          ctx.moveTo(x, y);
          first = false;
        } else {
          ctx.lineTo(x, y);
        }
      }

      for (let i = arr2.length - 1; i >= 0; i--) {
        const v = arr2[i];
        if (v == null || !Number.isFinite(v)) continue;
        const x = margin.left + (i / Math.max(1, arr2.length - 1)) * plotW;
        const y = (h - margin.bottom) - ((v - min) / range) * plotH;
        ctx.lineTo(x, y);
      }

      ctx.closePath();
      ctx.globalAlpha = 0.18;
      ctx.fill();
      ctx.restore();
    }

    function drawGauge(canvas, value, min, max, color, label, line1 = '', line2 = '') {
      const ctx = canvas.getContext('2d');
      const { w, h } = fitCanvas(canvas);
      ctx.clearRect(0, 0, w, h);

      const cx = w / 2;
      const cy = h * 0.66;
      const r = Math.min(w, h) * 0.30;
      const start = Math.PI;
      const end = 2 * Math.PI;

      ctx.save();
      ctx.lineCap = 'round';

      ctx.beginPath();
      ctx.strokeStyle = 'rgba(0,255,255,0.14)';
      ctx.lineWidth = 3;
      ctx.arc(cx, cy, r, start, end);
      ctx.stroke();

      const norm = clamp((value - min) / ((max - min) || 1), 0, 1);
      const a = start + norm * Math.PI;

      ctx.beginPath();
      ctx.shadowColor = color;
      ctx.shadowBlur = 10;
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.18;
      ctx.lineWidth = 4;
      ctx.arc(cx, cy, r, start, a);
      ctx.stroke();

      ctx.beginPath();
      ctx.shadowBlur = 0;
      ctx.globalAlpha = 1;
      ctx.lineWidth = 1;
      ctx.arc(cx, cy, r, start, a);
      ctx.stroke();

      drawGlowText(ctx, label, cx, h * 0.16, 'rgba(190,255,255,0.90)', 'center', 9, 'bold');
      drawGlowText(ctx, fmt(value, 2), cx, h * 0.44, color, 'center', 13, 'bold');
      if (line1) drawGlowText(ctx, line1, cx, h * 0.61, 'rgba(180,255,255,0.88)', 'center', 8, 'normal');
      if (line2) drawGlowText(ctx, line2, cx, h * 0.75, 'rgba(180,255,255,0.88)', 'center', 8, 'normal');

      ctx.restore();
    }

    function stateText(state) {
      const app = appStates[state.app] || 'UNKNOWN';
      if (state.app === 1) return `${app} (${modelPhases[state.phase] || '...'})`;
      return app;
    }

    function updateMetricText(state) {
      dom.appState.innerText = stateText(state);
      dom.vValue.innerText = `${fmt(state.v, 3)} V`;
      dom.iValue.innerText = `${fmt(state.i, 3)} A`;
      dom.mahValue.innerText = `${fmt(state.mah, 3)} mAh`;
      dom.extraState.innerText = `offset ${fmt(state.offset, 2)} mV · noise ${fmt(state.noise, 2)} mV`;
    }

    function drawAllGauges(state) {
      const dtLimit = Number.isFinite(state.max_dt) ? state.max_dt : 1.5;
      const liveDT = Number.isFinite(lastLiveDT) ? lastLiveDT : dtLimit;
      const gaugeDTMax = Math.max(3.0, dtLimit, liveDT);

      drawGauge(dom.gaugeV, state.v, 1.0, 2.0, '#ffe86a', 'Voltage', '', '');
      drawGauge(dom.gaugeI, state.i, 0.0, 0.5, '#ff4dff', 'Current', '', '');
      drawGauge(
        dom.gaugeTD,
        liveDT,
        0.0,
        gaugeDTMax,
        '#58a8ff',
        'Max dT',
        `max ${fmt(dtLimit, 2)} °C`,
        `dT ${fmt(liveDT, 2)} °C`
      );
      drawGauge(dom.gaugeDuty, state.duty || 0, 0, 255, '#58ff98', 'Duty', '', '');
    }

    function renderMain(data) {
      const canvas = document.getElementById('mainGraph');
      const ctx = canvas.getContext('2d');
      const margin = { top: 12, right: 34, bottom: 16, left: 30 };
      fitCanvas(canvas);
      clearCanvas(ctx);

      drawAxes(ctx, margin, 0, 320, 'Time', 0, 40, 'Value');
      drawSeries(ctx, data.v, '#ffe86a', 1.0, 2.0, margin, 'V', 1);
      drawSeries(ctx, data.i, '#ff4dff', 0.0, 0.5, margin, 'I', 1);
      drawSeries(ctx, data.t1, '#ff6c7a', 15, 40, margin, 'T1', 1);
      drawSeries(ctx, data.t2, '#58ff98', 15, 40, margin, 'T2', 1);
      drawSeries(ctx, data.td, '#58a8ff', -0.5, maxDT, margin, 'dT', 1);
    }

    function renderAmbient(data) {
      const canvas = document.getElementById('ambientGraph');
      const ctx = canvas.getContext('2d');
      const margin = { top: 12, right: 34, bottom: 16, left: 30 };
      fitCanvas(canvas);
      clearCanvas(ctx);

      drawAxes(ctx, margin, 0, 320, 'Time', 0, 100, 'T / H');
      drawSeries(ctx, data.t, '#ff6c7a', 10, 40, margin, 'T', 1);
      drawSeries(ctx, data.d, '#58ff98', 10, 40, margin, 'Dew', 1);
      drawSeries(ctx, data.h, '#58a8ff', 0, 100, margin, 'H', 1);

      const y = canvas.height - margin.bottom - (65 / 100) * (canvas.height - margin.top - margin.bottom);
      ctx.save();
      ctx.strokeStyle = 'rgba(255,176,90,0.72)';
      ctx.lineWidth = 1;
      ctx.setLineDash([5, 5]);
      ctx.shadowColor = 'rgba(255,176,90,0.55)';
      ctx.shadowBlur = 6;
      ctx.beginPath();
      ctx.moveTo(margin.left, y);
      ctx.lineTo(canvas.width - margin.right, y);
      ctx.stroke();
      ctx.restore();
    }

    function renderIR(data) {
      const canvas = document.getElementById('irGraph');
      const ctx = canvas.getContext('2d');
      const margin = { top: 12, right: 34, bottom: 16, left: 30 };
      fitCanvas(canvas);
      clearCanvas(ctx);

      const all = (data.lu || []).concat(data.pairs || []);
      let xMin = 0, xMax = 0.5, yMin = 0, yMax = 0.5;

      if (all.length) {
        xMin = Math.min(...all.map(p => p[0]));
        xMax = Math.max(...all.map(p => p[0]));
        yMin = Math.min(...all.map(p => p[1]));
        yMax = Math.max(...all.map(p => p[1]));
        const xr = (xMax - xMin) || 0.1;
        const yr = (yMax - yMin) || 0.1;
        xMin = Math.max(0, xMin - xr * 0.1);
        xMax = xMax + xr * 0.1;
        yMin = Math.max(0, yMin - yr * 0.1);
        yMax = yMax + yr * 0.1;
      }

      drawAxes(ctx, margin, xMin, xMax, 'Current (A)', yMin, yMax, 'Resistance (Ω)');
      drawXY(ctx, data.lu, '#f6ffff', xMin, xMax, yMin, yMax, margin, 'LU');
      drawXY(ctx, data.pairs, '#58fff3', xMin, xMax, yMin, yMax, margin, 'Pairs');
    }

    function renderCharge(data) {
      const canvas = document.getElementById('chargeGraph');
      const ctx = canvas.getContext('2d');
      const margin = { top: 12, right: 38, bottom: 16, left: 30 };
      fitCanvas(canvas);
      clearCanvas(ctx);

      if (!data || !data.length) {
        drawAxes(ctx, margin, 0, 100, 'Index', 0, 1, 'Value');
        dom.chargeLegend.textContent = '';
        return;
      }

      const arrI = data.map(d => d.i);
      const arrV = data.map(d => d.v);
      const arrD = data.map(d => d.d);
      const arrTD = data.map(d => d.td);
      const arrTH = data.map(d => d.th);
      const arrIRLU = data.map(d => d.irlu);
      const arrIRP = data.map(d => d.irp);

      const sI = autoScale(arrI);
      const sV = autoScale(arrV);
      const sD = autoScale(arrD);
      const sTD = autoScale(arrTD);
      const sTH = autoScale(arrTH);
      const sIRLU = autoScale(arrIRLU);
      const sIRP = autoScale(arrIRP);

      const yMin = Math.min(sI[0], sV[0], sD[0], sTD[0], sTH[0], sIRLU[0], sIRP[0]);
      const yMax = Math.max(sI[1], sV[1], sD[1], sTD[1], sTH[1], sIRLU[1], sIRP[1]);

      drawAxes(ctx, margin, 0, data.length - 1, 'Index', yMin, yMax, 'Auto-scaled');
      drawRegion(ctx, arrTD, arrTH, 'rgba(255,108,122,0.20)', Math.min(sTD[0], sTH[0]), Math.max(sTD[1], sTH[1]), margin);

      drawSeries(ctx, arrI, '#ff4dff', sI[0], sI[1], margin, 'I', 1);
      drawSeries(ctx, arrV, '#ffe86a', sV[0], sV[1], margin, 'V', 1);
      drawSeries(ctx, arrD, '#a9a9a9', sD[0], sD[1], margin, 'Duty', 1);
      drawSeries(ctx, arrTD, '#58a8ff', sTD[0], sTD[1], margin, 'dT', 1);
      drawSeries(ctx, arrTH, '#ff6c7a', sTH[0], sTH[1], margin, 'Th', 1);
      drawSeries(ctx, arrIRLU, '#ffb05a', sIRLU[0], sIRLU[1], margin, 'RiLU', 1);
      drawSeries(ctx, arrIRP, '#58fff3', sIRP[0], sIRP[1], margin, 'RiP', 1);

      const last = data[data.length - 1];
      dom.chargeLegend.textContent =
        `I[${fmt(sI[0],2)}, ${fmt(sI[1],2)}]  V[${fmt(sV[0],2)}, ${fmt(sV[1],2)}]  Duty[${fmt(sD[0],0)}, ${fmt(sD[1],0)}]  dT[${fmt(sTD[0],2)}, ${fmt(sTD[1],2)}]\n` +
        `Th[${fmt(sTH[0],2)}, ${fmt(sTH[1],2)}]  RiLU[${fmt(sIRLU[0],2)}, ${fmt(sIRLU[1],2)}]  RiP[${fmt(sIRP[0],2)}, ${fmt(sIRP[1],2)}]\n` +
        `Latest: V=${fmt(last.v,3)}  I=${fmt(last.i,3)}  dT=${fmt(last.td,2)}  Th=${fmt(last.th,2)}`;
    }

    async function updateData() {
      try {
        const data = await fetchPayload('/data?fmt=cbor');
        if (!data || !data.state) return;

        maxDT = Number.isFinite(data.state.max_dt) ? data.state.max_dt : 1.5;
        updateMetricText(data.state);

        if (data.ambient) renderAmbient(data.ambient);

        const now = Date.now();
        if (now - lastHistoryFetch > 1200) {
          lastHistoryFetch = now;
          const hist = await fetchPayload('/data?type=history&fmt=cbor');
          lastLiveDT = lastFinite(hist.td);
          renderMain(hist);
        }

        if (now - lastIRFetch > 5000) {
          lastIRFetch = now;
          const ir = await fetchPayload('/data?type=ir&fmt=cbor');
          renderIR(ir);
        }

        if (now - lastChargeFetch > 5000) {
          lastChargeFetch = now;
          const charge = await fetchPayload('/data?type=chargelog&fmt=cbor');
          renderCharge(charge);
        }

        drawAllGauges(data.state);

      } catch (e) {
        console.error(e);
      }
    }

    // Switched to setTimeout polling to prevent async fetch overlap/pileup on slow network connections
    async function loopData() {
        await updateData();
        setTimeout(loopData, 1000);
    }

    function resizeAll() {
      document.querySelectorAll('canvas').forEach(canvas => fitCanvas(canvas));
      updateData();
    }

    window.addEventListener('resize', resizeAll);
    window.addEventListener('orientationchange', resizeAll);
    resizeAll();
    loopData(); // start the loop
  </script>
</body>
</html>
)rawliteral";

#endif
