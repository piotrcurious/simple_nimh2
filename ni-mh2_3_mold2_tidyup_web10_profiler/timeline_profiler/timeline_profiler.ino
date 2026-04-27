#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_timer.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

static constexpr uint32_t FRAME_PERIOD_US = 50000;   // 20 Hz
static constexpr uint8_t   CORE_COUNT     = 2;
static constexpr uint8_t   MAX_EVENTS_PER_CORE = 8;

// -----------------------------
// Binary protocol
// Header (18 bytes, little-endian):
//   u16 magic      = 0x5450 ("TP")
//   u8  version    = 1
//   u8  cores      = 2
//   u32 frameSeq
//   u32 frameStartUs
//   u16 framePeriodUs
//   u8  countCore0
//   u8  countCore1
//   u8  flags      (bit0 = frame timeout / overrun)
//   u8  reserved
//
// Event (6 bytes):
//   u8  taskId
//   u8  flags
//   u16 startUs
//   u16 durUs
// -----------------------------

struct __attribute__((packed)) EventRec {
  uint8_t  taskId;
  uint8_t  flags;
  uint16_t startUs;
  uint16_t durUs;
};

struct CoreBuf {
  volatile uint8_t count;
  EventRec events[MAX_EVENTS_PER_CORE];
};

static CoreBuf g_coreBuf[CORE_COUNT];

static volatile uint32_t g_frameSeq     = 0;
static volatile uint32_t g_frameStartUs  = 0;
static volatile uint8_t  g_frameFlags    = 0;

static TaskHandle_t g_masterTaskHandle   = nullptr;
static TaskHandle_t g_workerTaskHandle[CORE_COUNT] = { nullptr, nullptr };

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// -----------------------------
// Embedded browser app
// -----------------------------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 Realtime Core Timeline</title>
  <style>
    html, body {
      margin: 0;
      height: 100%;
      background: #0b0f14;
      color: #d9e6f2;
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      overflow: hidden;
    }
    #topbar {
      box-sizing: border-box;
      height: 72px;
      padding: 10px 14px;
      border-bottom: 1px solid rgba(255,255,255,0.08);
      background: linear-gradient(to bottom, #111722, #0b0f14);
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px;
      align-items: center;
    }
    #title {
      font-size: 18px;
      font-weight: 700;
      letter-spacing: 0.3px;
    }
    #status {
      font-size: 12px;
      opacity: 0.9;
      margin-top: 4px;
      line-height: 1.35;
    }
    .legend {
      display: flex;
      gap: 14px;
      align-items: center;
      font-size: 12px;
      justify-self: end;
      white-space: nowrap;
    }
    .dot {
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      margin-right: 6px;
      vertical-align: middle;
    }
    .c0 { background: #5cc8ff; }
    .c1 { background: #ffb45c; }
    #axis {
      display: flex;
      justify-content: space-between;
      padding: 4px 14px 0 14px;
      font-size: 11px;
      opacity: 0.65;
    }
    #gl {
      display: block;
      width: 100vw;
      height: calc(100vh - 92px);
    }
  </style>
</head>
<body>
  <div id="topbar">
    <div>
      <div id="title">ESP32 realtime task timeline</div>
      <div id="status">connecting…</div>
    </div>
    <div class="legend">
      <span><span class="dot c0"></span>Core 0</span>
      <span><span class="dot c1"></span>Core 1</span>
    </div>
  </div>
  <div id="axis"><span>0 ms</span><span>50 ms</span></div>
  <canvas id="gl"></canvas>

<script>
(() => {
  const canvas = document.getElementById('gl');
  const statusEl = document.getElementById('status');

  const MAGIC = 0x5450;          // "TP"
  const HEADER_BYTES = 18;
  const EVENT_BYTES = 6;
  const FRAME_US_DEFAULT = 50000;

  const ROW_H = 30;
  const LANE_H = 9;
  const LANE_TOP = 4;
  const LANE_GAP = 5;
  const MAX_HISTORY = 90;

  const frames = [];
  let gl, program, buffer, aPos, aColor, uRes;
  let dpr = 1;

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const wantW = Math.max(1, Math.floor(rect.width * window.devicePixelRatio));
    const wantH = Math.max(1, Math.floor(rect.height * window.devicePixelRatio));
    if (canvas.width !== wantW || canvas.height !== wantH) {
      canvas.width = wantW;
      canvas.height = wantH;
    }
    dpr = window.devicePixelRatio || 1;
  }

  function compileShader(type, src) {
    const sh = gl.createShader(type);
    gl.shaderSource(sh, src);
    gl.compileShader(sh);
    if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
      throw new Error(gl.getShaderInfoLog(sh));
    }
    return sh;
  }

  function initGL() {
    gl = canvas.getContext('webgl', { antialias: false, alpha: false });
    if (!gl) {
      statusEl.textContent = 'WebGL not available';
      return;
    }

    const vs = compileShader(gl.VERTEX_SHADER, `
      attribute vec2 a_pos;
      attribute vec4 a_color;
      uniform vec2 u_res;
      varying vec4 v_color;
      void main() {
        vec2 zeroToOne = a_pos / u_res;
        vec2 zeroToTwo = zeroToOne * 2.0;
        vec2 clip = zeroToTwo - 1.0;
        gl_Position = vec4(clip * vec2(1.0, -1.0), 0.0, 1.0);
        v_color = a_color;
      }
    `);

    const fs = compileShader(gl.FRAGMENT_SHADER, `
      precision mediump float;
      varying vec4 v_color;
      void main() {
        gl_FragColor = v_color;
      }
    `);

    program = gl.createProgram();
    gl.attachShader(program, vs);
    gl.attachShader(program, fs);
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      throw new Error(gl.getProgramInfoLog(program));
    }

    gl.useProgram(program);
    buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);

    aPos = gl.getAttribLocation(program, 'a_pos');
    aColor = gl.getAttribLocation(program, 'a_color');
    uRes = gl.getUniformLocation(program, 'u_res');

    gl.enableVertexAttribArray(aPos);
    gl.enableVertexAttribArray(aColor);
    gl.vertexAttribPointer(aPos, 2, gl.FLOAT, false, 24, 0);
    gl.vertexAttribPointer(aColor, 4, gl.FLOAT, false, 24, 8);

    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.clearColor(0.05, 0.07, 0.10, 1.0);
  }

  function colorForTask(taskId) {
    const palette = [
      [0.37, 0.78, 1.00, 0.96],
      [1.00, 0.71, 0.37, 0.96],
      [0.53, 1.00, 0.53, 0.96],
      [1.00, 0.42, 0.62, 0.96],
      [0.68, 0.56, 1.00, 0.96],
      [0.98, 0.93, 0.35, 0.96],
      [0.35, 0.90, 0.88, 0.96],
      [1.00, 0.55, 0.32, 0.96],
    ];
    return palette[(taskId - 1) % palette.length];
  }

  function pushRect(arr, x1, y1, x2, y2, r, g, b, a) {
    arr.push(
      x1, y1, r, g, b, a,
      x2, y1, r, g, b, a,
      x1, y2, r, g, b, a,

      x1, y2, r, g, b, a,
      x2, y1, r, g, b, a,
      x2, y2, r, g, b, a
    );
  }

  function draw() {
    if (!gl) return;
    resizeCanvas();

    const w = canvas.width;
    const h = canvas.height;

    gl.viewport(0, 0, w, h);
    gl.clear(gl.COLOR_BUFFER_BIT);
    gl.useProgram(program);
    gl.uniform2f(uRes, w, h);

    const verts = [];
    const visibleRows = Math.max(1, Math.min(frames.length, Math.floor(h / ROW_H)));

    for (let row = 0; row < visibleRows; row++) {
      const f = frames[row];
      const y0 = row * ROW_H;
      const y1 = y0 + ROW_H;

      const period = f.framePeriodUs || FRAME_US_DEFAULT;

      // Row background subtle bands for the two cores.
      pushRect(verts, 0, y0, w, y0 + ROW_H, 0.06, 0.08, 0.11, 0.18);

      // Frame start and end markers.
      pushRect(verts, 0, y0, 2, y1, 1.0, 1.0, 1.0, 0.18);
      pushRect(verts, w - 2, y0, w, y1, 1.0, 1.0, 1.0, 0.10);

      // Grid every 10 ms, stronger every 20 ms.
      for (let ms = 10; ms < period / 1000; ms += 10) {
        const x = (ms * 1000 / period) * w;
        const alpha = (ms % 20 === 0) ? 0.13 : 0.08;
        pushRect(verts, x, y0, x + 1, y1, 0.75, 0.82, 0.92, alpha);
      }

      const lanes = f.lanes;
      for (let core = 0; core < 2; core++) {
        const laneTop = y0 + LANE_TOP + core * (LANE_H + LANE_GAP);
        const laneBottom = laneTop + LANE_H;

        // lane separator
        pushRect(verts, 0, laneTop - 1, w, laneTop, 0.9, 0.9, 0.9, 0.05);

        for (const ev of lanes[core]) {
          let x1 = (ev.startUs / period) * w;
          let x2 = ((ev.startUs + ev.durUs) / period) * w;
          x1 = Math.max(0, Math.min(w, x1));
          x2 = Math.max(x1 + 1, Math.min(w, x2));

          const c = colorForTask(ev.taskId);
          const alpha = (ev.flags & 1) ? 1.0 : c[3];
          pushRect(verts, x1, laneTop, x2, laneBottom, c[0], c[1], c[2], alpha);
        }
      }
    }

    const data = new Float32Array(verts);
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);

    gl.vertexAttribPointer(aPos, 2, gl.FLOAT, false, 24, 0);
    gl.vertexAttribPointer(aColor, 4, gl.FLOAT, false, 24, 8);

    if (verts.length > 0) {
      gl.drawArrays(gl.TRIANGLES, 0, verts.length / 6);
    }
  }

  function decodeFrame(arrayBuffer) {
    const dv = new DataView(arrayBuffer);
    if (dv.byteLength < HEADER_BYTES) return;

    const magic = dv.getUint16(0, true);
    if (magic !== MAGIC) return;

    const version = dv.getUint8(2);
    const cores = dv.getUint8(3);
    const frameSeq = dv.getUint32(4, true);
    const frameStartUs = dv.getUint32(8, true);
    const framePeriodUs = dv.getUint16(12, true);
    const c0 = dv.getUint8(14);
    const c1 = dv.getUint8(15);
    const flags = dv.getUint8(16);

    if (version !== 1 || cores !== 2) return;

    let off = HEADER_BYTES;
    const lanes = [[], []];

    for (let core = 0; core < 2; core++) {
      const count = core === 0 ? c0 : c1;
      for (let i = 0; i < count; i++) {
        if (off + EVENT_BYTES > dv.byteLength) break;
        const taskId = dv.getUint8(off + 0);
        const evFlags = dv.getUint8(off + 1);
        const startUs = dv.getUint16(off + 2, true);
        const durUs = dv.getUint16(off + 4, true);
        lanes[core].push({ taskId, flags: evFlags, startUs, durUs });
        off += EVENT_BYTES;
      }
    }

    frames.unshift({ frameSeq, frameStartUs, framePeriodUs, flags, lanes });
    if (frames.length > MAX_HISTORY) frames.pop();

    statusEl.textContent =
      `connected | frame ${frameSeq} | start ${frameStartUs} us | ` +
      `core0 ${c0} events | core1 ${c1} events` +
      ((flags & 1) ? ' | frame timeout' : '');

    draw();
  }

  function connect() {
    const proto = (location.protocol === 'https:') ? 'wss://' : 'ws://';
    const sock = new WebSocket(proto + location.host + '/ws');
    sock.binaryType = 'arraybuffer';

    sock.onopen = () => {
      statusEl.textContent = 'connected';
    };
    sock.onclose = () => {
      statusEl.textContent = 'disconnected, reconnecting…';
      setTimeout(connect, 1000);
    };
    sock.onerror = () => {
      statusEl.textContent = 'socket error';
    };
    sock.onmessage = (e) => {
      if (e.data instanceof ArrayBuffer) decodeFrame(e.data);
    };
  }

  window.addEventListener('resize', draw);

  initGL();
  connect();
  draw();
})();
</script>
</body>
</html>
)rawliteral";

// -----------------------------
// Helpers
// -----------------------------
static inline void putU8(uint8_t *buf, size_t &off, uint8_t v) {
  buf[off++] = v;
}

static inline void putU16LE(uint8_t *buf, size_t &off, uint16_t v) {
  buf[off++] = (uint8_t)(v & 0xFF);
  buf[off++] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void putU32LE(uint8_t *buf, size_t &off, uint32_t v) {
  buf[off++] = (uint8_t)(v & 0xFF);
  buf[off++] = (uint8_t)((v >> 8) & 0xFF);
  buf[off++] = (uint8_t)((v >> 16) & 0xFF);
  buf[off++] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void busyWaitUs(uint32_t us) {
  // Small demo work; keeps the timing visible.
  delayMicroseconds(us);
}

static inline void recordEvent(uint8_t core, uint8_t taskId, uint16_t startUs, uint16_t durUs, uint8_t flags = 0) {
  if (core >= CORE_COUNT) return;
  portENTER_CRITICAL(&g_mux);
  uint8_t idx = g_coreBuf[core].count;
  if (idx < MAX_EVENTS_PER_CORE) {
    g_coreBuf[core].events[idx].taskId = taskId;
    g_coreBuf[core].events[idx].flags  = flags;
    g_coreBuf[core].events[idx].startUs = startUs;
    g_coreBuf[core].events[idx].durUs   = durUs;
    g_coreBuf[core].count = idx + 1;
  }
  portEXIT_CRITICAL(&g_mux);
}

static inline void runTimedTask(uint8_t core, uint8_t taskId, uint32_t workUs, uint32_t gapUs = 0) {
  uint32_t t0 = (uint32_t)(esp_timer_get_time() - g_frameStartUs);
  busyWaitUs(workUs);
  uint32_t t1 = (uint32_t)(esp_timer_get_time() - g_frameStartUs);
  uint32_t dur = (t1 > t0) ? (t1 - t0) : 0;
  if (dur > 0xFFFF) dur = 0xFFFF;
  if (t0 > 0xFFFF) t0 = 0xFFFF;
  recordEvent(core, taskId, (uint16_t)t0, (uint16_t)dur, 0);

  if (gapUs > 0) busyWaitUs(gapUs);
}

static void sendFramePacket(bool timeoutFlag) {
  uint8_t packet[18 + (CORE_COUNT * MAX_EVENTS_PER_CORE * 6)];
  size_t p = 0;

  uint8_t c0, c1;
  portENTER_CRITICAL(&g_mux);
  c0 = g_coreBuf[0].count;
  c1 = g_coreBuf[1].count;
  portEXIT_CRITICAL(&g_mux);

  if (c0 > MAX_EVENTS_PER_CORE) c0 = MAX_EVENTS_PER_CORE;
  if (c1 > MAX_EVENTS_PER_CORE) c1 = MAX_EVENTS_PER_CORE;

  putU16LE(packet, p, 0x5450); // "TP"
  putU8(packet, p, 1);         // version
  putU8(packet, p, 2);         // cores
  putU32LE(packet, p, g_frameSeq);
  putU32LE(packet, p, g_frameStartUs);
  putU16LE(packet, p, (uint16_t)FRAME_PERIOD_US);
  putU8(packet, p, c0);
  putU8(packet, p, c1);

  uint8_t flags = 0;
  if (timeoutFlag) flags |= 0x01;
  if (c0 >= MAX_EVENTS_PER_CORE || c1 >= MAX_EVENTS_PER_CORE) flags |= 0x02;
  putU8(packet, p, flags);
  putU8(packet, p, 0);

  // Core 0 events
  for (uint8_t i = 0; i < c0; i++) {
    EventRec e;
    portENTER_CRITICAL(&g_mux);
    e = g_coreBuf[0].events[i];
    portEXIT_CRITICAL(&g_mux);

    putU8(packet, p, e.taskId);
    putU8(packet, p, e.flags);
    putU16LE(packet, p, e.startUs);
    putU16LE(packet, p, e.durUs);
  }

  // Core 1 events
  for (uint8_t i = 0; i < c1; i++) {
    EventRec e;
    portENTER_CRITICAL(&g_mux);
    e = g_coreBuf[1].events[i];
    portEXIT_CRITICAL(&g_mux);

    putU8(packet, p, e.taskId);
    putU8(packet, p, e.flags);
    putU16LE(packet, p, e.startUs);
    putU16LE(packet, p, e.durUs);
  }

  ws.binaryAll(packet, p);
}

// -----------------------------
// Worker tasks
// -----------------------------
static void workerTask(void *param) {
  const uint8_t core = (uint8_t)(uintptr_t)param;

  for (;;) {
    // Wait for the master frame tick.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Optional: clear any stale state from a previous frame.
    // The master resets counts before sending the next tick.

    if (core == 0) {
      // Core 0 pattern
      runTimedTask(0, 1, 1200, 250);
      runTimedTask(0, 2, 2200, 300);
      runTimedTask(0, 3,  900, 100);
    } else {
      // Core 1 pattern
      runTimedTask(1, 4,  700, 300);
      runTimedTask(1, 5, 2500, 150);
      runTimedTask(1, 6, 1300,  50);
    }

    // Tell master this core is finished for the current frame.
    if (g_masterTaskHandle) {
      xTaskNotifyGive(g_masterTaskHandle);
    }
  }
}

// -----------------------------
// Master frame task
// -----------------------------
static void masterTask(void *param) {
  (void)param;

  // Wait until workers are ready.
  while (g_workerTaskHandle[0] == nullptr || g_workerTaskHandle[1] == nullptr) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  const TickType_t frameTicks = pdMS_TO_TICKS(FRAME_PERIOD_US / 1000);

  for (;;) {
    const uint32_t frameStart = (uint32_t)esp_timer_get_time();

    // Reset per-frame buffers.
    portENTER_CRITICAL(&g_mux);
    g_coreBuf[0].count = 0;
    g_coreBuf[1].count = 0;
    portEXIT_CRITICAL(&g_mux);

    g_frameStartUs = frameStart;
    g_frameFlags = 0;
    g_frameSeq++;

    // Kick both worker cores.
    xTaskNotifyGive(g_workerTaskHandle[0]);
    xTaskNotifyGive(g_workerTaskHandle[1]);

    // Wait for both workers to finish, but do not block forever.
    uint8_t done = 0;
    const TickType_t startTick = xTaskGetTickCount();
    while (done < 2) {
      TickType_t elapsed = xTaskGetTickCount() - startTick;
      if (elapsed >= frameTicks) {
        g_frameFlags |= 0x01;
        break;
      }
      TickType_t remaining = frameTicks - elapsed;
      uint32_t got = ulTaskNotifyTake(pdTRUE, remaining);
      if (got == 0) {
        g_frameFlags |= 0x01;
        break;
      }
      done += (uint8_t)got;
    }

    const bool timeout = (done < 2);
    sendFramePacket(timeout);

    // Hold a 20 Hz cadence.
    const uint32_t elapsedUs = (uint32_t)(esp_timer_get_time() - frameStart);
    if (elapsedUs < FRAME_PERIOD_US) {
      delayMicroseconds(FRAME_PERIOD_US - elapsedUs);
    }
  }
}

// -----------------------------
// Web server
// -----------------------------
static void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-Timeline", "12345678");
  IPAddress ip = WiFi.softAPIP();

  Serial.print("AP IP: ");
  Serial.println(ip);

  ws.onEvent([](AsyncWebSocket *server,
                AsyncWebSocketClient *client,
                AwsEventType type,
                void *arg,
                uint8_t *data,
                size_t len) {
    (void)server; (void)client; (void)arg; (void)data; (void)len;
    if (type == WS_EVT_CONNECT) {
      Serial.println("WebSocket client connected");
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.println("WebSocket client disconnected");
    }
  });

  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html; charset=utf-8", INDEX_HTML);
    request->send(response);
  });

  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ok");
  });

  server.begin();
  ws.setNoDelay(true);
}

// -----------------------------
// Arduino setup / loop
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("ESP32 realtime task timeline starting...");

  setupWeb();

  // Create workers first.
  xTaskCreatePinnedToCore(
    workerTask,
    "worker0",
    4096,
    (void*)0,
    2,
    &g_workerTaskHandle[0],
    0
  );

  xTaskCreatePinnedToCore(
    workerTask,
    "worker1",
    4096,
    (void*)1,
    2,
    &g_workerTaskHandle[1],
    1
  );

  // Master on core 0, slightly higher priority.
  xTaskCreatePinnedToCore(
    masterTask,
    "master",
    4096,
    nullptr,
    3,
    &g_masterTaskHandle,
    0
  );

  Serial.println("Open http://192.168.4.1/ after connecting to ESP32-Timeline");
}

void loop() {
  // Async server + dedicated tasks do all the work.
    ws.cleanupClients();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
