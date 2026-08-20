#ifndef CALIBRATE_HTML_H
#define CALIBRATE_HTML_H

#include <Arduino.h>

const char CALIBRATE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Current Calibration - Ni-MH Charger</title>
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
      width: min(100%, 800px);
      margin: 0 auto;
      padding: 12px;
      box-sizing: border-box;
    }

    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 12px;
      padding-bottom: 8px;
      border-bottom: 1px solid rgba(0,255,255,0.2);
    }

    h1 {
      font-size: 16px;
      margin: 0;
      color: var(--cyan);
      text-transform: uppercase;
      letter-spacing: 0.05em;
      text-shadow: 0 0 8px rgba(0,255,255,0.4);
    }

    .card, .panel {
      background: var(--panel);
      border: 1px solid rgba(0,255,255,0.14);
      border-radius: 4px;
      box-shadow:
        0 0 0 1px rgba(0,255,255,0.03) inset,
        0 0 10px rgba(0,255,255,0.07);
      padding: 12px;
      margin-bottom: 12px;
      position: relative;
    }

    .panelHead {
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      color: #eaffff;
      margin-bottom: 10px;
      padding-bottom: 4px;
      border-bottom: 1px solid rgba(0,255,255,0.1);
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 10px;
    }

    @media (max-width: 600px) {
      .grid { grid-template-columns: 1fr; }
    }

    .metricBox {
      background: rgba(0,255,255,0.03);
      border: 1px solid rgba(0,255,255,0.12);
      border-radius: 3px;
      padding: 8px 10px;
      text-align: center;
    }

    .metricVal {
      font-size: 18px;
      font-weight: bold;
      color: var(--cyan);
      text-shadow: 0 0 6px rgba(0,255,255,0.25);
    }

    .metricVal.highlight {
      font-size: 22px;
      color: var(--yel);
      text-shadow: 0 0 8px rgba(255,232,106,0.35);
    }

    .metricLbl {
      font-size: 10px;
      color: var(--muted);
      margin-top: 3px;
      text-transform: uppercase;
    }

    .ctrlRow {
      display: flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 8px;
      flex-wrap: wrap;
    }

    label {
      font-size: 11px;
      color: var(--muted);
      min-width: 140px;
    }

    input[type="number"], input[type="text"] {
      appearance: none;
      background: rgba(0,0,0,0.6);
      border: 1px solid rgba(0,255,255,0.3);
      color: var(--text);
      padding: 4px 8px;
      border-radius: 3px;
      font: inherit;
      font-size: 12px;
      width: 90px;
      box-sizing: border-box;
    }

    input[type="range"] {
      flex: 1;
      accent-color: var(--cyan);
    }

    button {
      appearance: none;
      border: 1px solid rgba(0,255,255,0.22);
      background: linear-gradient(180deg, rgba(12,24,40,0.94), rgba(3,8,14,0.98));
      color: #b8ffff;
      border-radius: 3px;
      padding: 5px 10px;
      font: inherit;
      font-size: 11px;
      letter-spacing: 0.03em;
      text-transform: uppercase;
      cursor: pointer;
      box-shadow:
        0 0 0 1px rgba(0,255,255,0.03) inset,
        0 0 3px rgba(0,255,255,0.06);
    }

    button:active {
      background: linear-gradient(180deg, rgba(3,8,14,0.98), rgba(12,24,40,0.94));
    }

    button.primary {
      border-color: rgba(88,255,152,0.5);
      background: linear-gradient(180deg, rgba(8,40,24,0.94), rgba(2,16,8,0.98));
      color: var(--grn);
      text-shadow: 0 0 6px rgba(88,255,152,0.3);
    }

    button.secondary {
      border-color: rgba(255,232,106,0.5);
      color: var(--yel);
    }

    .btnGroup {
      display: flex;
      gap: 4px;
      flex-wrap: wrap;
    }

    .statusMsg {
      font-size: 11px;
      margin-top: 8px;
      color: var(--grn);
      min-height: 16px;
    }

    a.navLink {
      color: var(--cyan);
      text-decoration: none;
      font-size: 11px;
      border: 1px solid rgba(0,255,255,0.3);
      padding: 4px 8px;
      border-radius: 3px;
      background: rgba(0,255,255,0.05);
    }

    a.navLink:hover {
      background: rgba(0,255,255,0.15);
    }
  </style>
</head>
<body>
  <div class="wrap">
    <header>
      <h1>Current Shunt Calibration</h1>
      <a href="/" class="navLink">&larr; Back to Dashboard</a>
    </header>

    <!-- Calibration Test Setup & Live Measurements -->
    <div class="panel">
      <div class="panelHead">
        <span>Calibration Measurements</span>
        <span id="connStatus" style="color:var(--muted); font-size:10px;">Connecting...</span>
      </div>

      <div class="ctrlRow" style="margin-bottom: 12px;">
        <label for="rCalib">Calibration Resistor (Ω):</label>
        <input type="number" id="rCalib" value="10.0" step="0.1" min="0.01" max="1000" oninput="updateRefCurrent()">
        <span style="font-size:10px; color:var(--muted);">(Placed in battery compartment)</span>
      </div>

      <div class="grid">
        <div class="metricBox">
          <div class="metricVal" id="valV">0.000 V</div>
          <div class="metricLbl">Battery/Resistor Voltage</div>
        </div>

        <div class="metricBox">
          <div class="metricVal highlight" id="valIRef">0.000 A</div>
          <div class="metricLbl">Reference Current (V / R_calib)</div>
        </div>

        <div class="metricBox">
          <div class="metricVal" id="valShuntMv">0.00 mV</div>
          <div class="metricLbl">Shunt Voltage Drop</div>
        </div>

        <div class="metricBox">
          <div class="metricVal" id="valIMeas">0.000 A</div>
          <div class="metricLbl">Measured Current</div>
        </div>
      </div>
    </div>

    <!-- Manual Duty Cycle Control -->
    <div class="panel">
      <div class="panelHead">Duty Cycle Control</div>
      <div class="ctrlRow">
        <label for="dutySlider">Manual Duty Cycle:</label>
        <input type="range" id="dutySlider" min="0" max="255" value="0" oninput="onDutySliderChange(this.value)">
        <input type="number" id="dutyNum" min="0" max="255" value="0" style="width:65px;" onchange="onDutyNumChange(this.value)">
      </div>
      <div class="ctrlRow">
        <label>Presets:</label>
        <div class="btnGroup">
          <button onclick="setDuty(0)">0 (OFF)</button>
          <button onclick="setDuty(32)">32</button>
          <button onclick="setDuty(64)">64</button>
          <button onclick="setDuty(128)">128 (50%)</button>
          <button onclick="setDuty(192)">192</button>
          <button onclick="setDuty(255)">255 (MAX)</button>
        </div>
      </div>
    </div>

    <!-- Shunt Resistor Manual Adjustment & Storage -->
    <div class="panel">
      <div class="panelHead">Shunt Resistor Calibration (R_shunt)</div>

      <div class="ctrlRow">
        <label for="rShuntInput">Shunt Resistance (Ω):</label>
        <input type="number" id="rShuntInput" value="13.500" step="0.001" min="0.01" max="1000" oninput="onShuntInputChange()">
        <div class="btnGroup">
          <button onclick="adjustShunt(-0.1)">-0.1</button>
          <button onclick="adjustShunt(-0.01)">-0.01</button>
          <button onclick="adjustShunt(0.01)">+0.01</button>
          <button onclick="adjustShunt(0.1)">+0.1</button>
        </div>
      </div>

      <div class="ctrlRow" style="margin-top: 12px; gap: 8px;">
        <button class="secondary" onclick="autoCalibrateShunt()">Auto-Calculate Shunt Resistance</button>
        <button class="primary" onclick="saveShuntToFlash()">Save to Flash Memory</button>
      </div>

      <div class="statusMsg" id="statusMsg"></div>
    </div>
  </div>

  <script>
    const textDecoder = new TextDecoder();
    let ws = null;
    let currentV = 0.0;
    let currentShuntMv = 0.0;
    let currentShuntR = 13.5;
    let currentIMeas = 0.0;

    // Load saved calibration resistor from localStorage if available
    const savedRCalib = localStorage.getItem('calib_resistor_ohm');
    if (savedRCalib && parseFloat(savedRCalib) > 0) {
      document.getElementById('rCalib').value = savedRCalib;
    }

    function connectWS() {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const url = `${protocol}//${window.location.host}/ws`;
      ws = new WebSocket(url);
      ws.binaryType = 'arraybuffer';

      ws.onopen = () => {
        document.getElementById('connStatus').innerText = 'WebSocket Connected';
        document.getElementById('connStatus').style.color = 'var(--grn)';
      };

      ws.onmessage = (e) => {
        if (!(e.data instanceof ArrayBuffer)) return;
        const bytes = new Uint8Array(e.data);
        const data = decodeCbor(bytes);
        if (data) handleStateData(data);
      };

      ws.onclose = () => {
        document.getElementById('connStatus').innerText = 'Disconnected, retrying...';
        document.getElementById('connStatus').style.color = 'var(--red)';
        setTimeout(connectWS, 2000);
      };

      ws.onerror = (err) => console.error('WS Error:', err);
    }

    function sendCommand(cmd) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(cmd);
      } else {
        fetch('/command?cmd=' + encodeURIComponent(cmd), { cache: 'no-store' }).catch(console.error);
      }
    }

    function handleStateData(data) {
      if (!data) return;
      if (data.v !== undefined) {
        currentV = data.v;
        document.getElementById('valV').innerText = currentV.toFixed(3) + ' V';
      }
      if (data.shunt_mv !== undefined) {
        currentShuntMv = data.shunt_mv;
        document.getElementById('valShuntMv').innerText = currentShuntMv.toFixed(2) + ' mV';
      }
      if (data.i !== undefined) {
        currentIMeas = data.i;
        document.getElementById('valIMeas').innerText = currentIMeas.toFixed(3) + ' A (' + (currentIMeas * 1000).toFixed(1) + ' mA)';
      }
      if (data.duty !== undefined) {
        document.getElementById('dutySlider').value = data.duty;
        document.getElementById('dutyNum').value = data.duty;
      }
      if (data.shunt_r !== undefined && !document.activeElement.classList.contains('userEditing')) {
        currentShuntR = data.shunt_r;
        document.getElementById('rShuntInput').value = currentShuntR.toFixed(3);
      }
      updateRefCurrent();
    }

    function updateRefCurrent() {
      const rCalibInput = parseFloat(document.getElementById('rCalib').value);
      localStorage.setItem('calib_resistor_ohm', rCalibInput);

      if (isNaN(rCalibInput) || rCalibInput <= 0) {
        document.getElementById('valIRef').innerText = '—';
        return;
      }

      const iRef = currentV / rCalibInput;
      document.getElementById('valIRef').innerText = iRef.toFixed(3) + ' A (' + (iRef * 1000).toFixed(1) + ' mA)';
    }

    function onDutySliderChange(val) {
      document.getElementById('dutyNum').value = val;
      setDuty(val);
    }

    function onDutyNumChange(val) {
      val = Math.max(0, Math.min(255, parseInt(val) || 0));
      document.getElementById('dutySlider').value = val;
      setDuty(val);
    }

    function setDuty(val) {
      document.getElementById('dutySlider').value = val;
      document.getElementById('dutyNum').value = val;
      sendCommand('SET_DUTY:' + val);
    }

    function onShuntInputChange() {
      const input = document.getElementById('rShuntInput');
      input.classList.add('userEditing');
      const val = parseFloat(input.value);
      if (!isNaN(val) && val > 0.01) {
        sendCommand('SET_SHUNT_R:' + val.toFixed(4));
      }
    }

    function adjustShunt(delta) {
      const input = document.getElementById('rShuntInput');
      input.classList.add('userEditing');
      let currentVal = parseFloat(input.value) || 13.5;
      currentVal = Math.max(0.01, currentVal + delta);
      input.value = currentVal.toFixed(3);
      sendCommand('SET_SHUNT_R:' + currentVal.toFixed(4));
    }

    function autoCalibrateShunt() {
      const rCalib = parseFloat(document.getElementById('rCalib').value);
      if (isNaN(rCalib) || rCalib <= 0) {
        showStatus('Invalid calibration resistor value!', true);
        return;
      }

      const iRef = currentV / rCalib;
      if (iRef <= 0.001 || Math.abs(currentShuntMv) < 0.1) {
        showStatus('Current or shunt voltage too low to calibrate. Increase Duty Cycle!', true);
        return;
      }

      // R_shunt = V_shunt_mv / (1000 * I_ref)
      const calculatedR = currentShuntMv / (1000.0 * iRef);
      if (isNaN(calculatedR) || calculatedR <= 0.01 || calculatedR > 1000) {
        showStatus('Calculated shunt resistance out of bounds!', true);
        return;
      }

      const input = document.getElementById('rShuntInput');
      input.value = calculatedR.toFixed(3);
      sendCommand('SET_SHUNT_R:' + calculatedR.toFixed(4));
      showStatus('Auto-calculated Shunt Resistance: ' + calculatedR.toFixed(3) + ' Ω', false);
    }

    function saveShuntToFlash() {
      const val = parseFloat(document.getElementById('rShuntInput').value);
      if (isNaN(val) || val <= 0.01 || val > 1000) {
        showStatus('Invalid shunt resistance value!', true);
        return;
      }

      sendCommand('SAVE_SHUNT_R:' + val.toFixed(4));
      showStatus('Shunt Resistance (' + val.toFixed(3) + ' Ω) saved to flash memory!', false);
      setTimeout(() => {
        document.getElementById('rShuntInput').classList.remove('userEditing');
      }, 2000);
    }

    function showStatus(msg, isError) {
      const el = document.getElementById('statusMsg');
      el.innerText = msg;
      el.style.color = isError ? 'var(--red)' : 'var(--grn)';
    }

    // Standard CBOR Decoder
    function readUint(ai, view, state) {
      if (ai < 24) return ai;
      if (ai === 24) return view.getUint8(state.pos++);
      if (ai === 25) { const v = view.getUint16(state.pos, false); state.pos += 2; return v; }
      if (ai === 26) { const v = view.getUint32(state.pos, false); state.pos += 4; return v; }
      if (ai === 27) {
        const hi = view.getUint32(state.pos, false);
        const lo = view.getUint32(state.pos + 4, false);
        state.pos += 8; return hi * 4294967296 + lo;
      }
      throw new Error('Unsupported CBOR len');
    }

    function decodeItem(view, state) {
      const head = view.getUint8(state.pos++);
      const major = head >> 5;
      const ai = head & 0x1f;
      if (major === 0) return readUint(ai, view, state);
      if (major === 1) return -1 - readUint(ai, view, state);
      if (major === 3) {
        const len = readUint(ai, view, state);
        const bytes = new Uint8Array(view.buffer, view.byteOffset + state.pos, len);
        state.pos += len; return textDecoder.decode(bytes);
      }
      if (major === 4) {
        const len = readUint(ai, view, state);
        const arr = []; for (let i = 0; i < len; i++) arr.push(decodeItem(view, state));
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
      if (major === 7) {
        if (ai === 20) return false;
        if (ai === 21) return true;
        if (ai === 22) return null;
        if (ai === 26) { const v = view.getFloat32(state.pos, false); state.pos += 4; return v; }
        if (ai === 27) { const v = view.getFloat64(state.pos, false); state.pos += 8; return v; }
      }
      return null;
    }

    function decodeCbor(bytes) {
      try {
        return decodeItem(new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength), { pos: 0 });
      } catch(e) { return null; }
    }

    function loopState() {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send('REQ_STATE');
      }
      setTimeout(loopState, 1000);
    }

    connectWS();
    loopState();
  </script>
</body>
</html>
)rawliteral";

#endif
