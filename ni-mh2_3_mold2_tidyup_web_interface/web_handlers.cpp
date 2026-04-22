#include "definitions.h"
#include "home_screen.h"
#include <WebServer.h>

extern WebServer server;

// Forward declarations
void applyDuty(uint32_t duty);

String getJsonState() {
    String json = "{";
    json += "\"app\":" + String(currentAppState) + ",";
    json += "\"display\":" + String(currentDisplayState) + ",";
    json += "\"duty\":" + String(dutyCycle) + ",";
    json += "\"v\":" + String(voltage_mv / 1000.0, 3) + ",";
    json += "\"i\":" + String(current_ma / 1000.0, 3) + ",";
    json += "\"mah\":" + String(mAh_charged, 3);
    json += "}";
    return json;
}

String getJsonHistory() {
    String json = "{";
    auto addArray = [&](const String& name, float* arr, int len) {
        json += "\"" + name + "\":[";
        for (int i = 0; i < len; i++) {
            if (std::isnan(arr[i])) json += "null";
            else json += String(arr[i], 2);
            if (i < len - 1) json += ",";
        }
        json += "]";
    };
    addArray("t1", temp1_values, PLOT_WIDTH); json += ",";
    addArray("t2", temp2_values, PLOT_WIDTH); json += ",";
    addArray("td", diff_values, PLOT_WIDTH); json += ",";
    addArray("v", voltage_values, PLOT_WIDTH); json += ",";
    addArray("i", current_values, PLOT_WIDTH);
    json += "}";
    return json;
}

String getJsonAmbient() {
    String json = "{";
    auto addArray = [&](const String& name, float* arr, int len) {
        json += "\"" + name + "\":[";
        for (int i = 0; i < len; i++) {
            if (std::isnan(arr[i])) json += "null";
            else json += String(arr[i], 2);
            if (i < len - 1) json += ",";
        }
        json += "]";
    };
    addArray("t", homeScreen.temp_history, PLOT_WIDTH); json += ",";
    addArray("h", homeScreen.humidity_history, PLOT_WIDTH); json += ",";
    addArray("d", homeScreen.dew_point_history, PLOT_WIDTH);
    json += "}";
    return json;
}

String getJsonChargeLog() {
    String json = "[";
    for (size_t i = 0; i < chargeLog.size(); i++) {
        json += "{";
        json += "\"t\":" + String(chargeLog[i].timestamp) + ",";
        json += "\"i\":" + String(chargeLog[i].current, 3) + ",";
        json += "\"v\":" + String(chargeLog[i].voltage, 3) + ",";
        json += "\"at\":" + String(chargeLog[i].ambientTemperature, 2) + ",";
        json += "\"bt\":" + String(chargeLog[i].batteryTemperature, 2) + ",";
        json += "\"d\":" + String(chargeLog[i].dutyCycle) + ",";
        json += "\"irlu\":" + String(chargeLog[i].internalResistanceLoadedUnloaded, 3) + ",";
        json += "\"irp\":" + String(chargeLog[i].internalResistancePairs, 3) + ",";

        float td = chargeLog[i].batteryTemperature - chargeLog[i].ambientTemperature;
        json += "\"td\":" + String(td, 2) + ",";

        // Calculate estimated threshold like in graphing.cpp
        size_t prevIdx = (i > 0) ? i - 1 : 0;
        float estimatedDiff = estimateTempDiff(
                chargeLog[i].voltage,
                chargeLog[i].voltage,
                chargeLog[i].current,
                regressedInternalResistancePairsIntercept,
                chargeLog[i].ambientTemperature,
                chargeLog[i].timestamp,
                chargeLog[prevIdx].timestamp,
                chargeLog[i].batteryTemperature,
                DEFAULT_CELL_MASS_KG,
                DEFAULT_SPECIFIC_HEAT,
                DEFAULT_SURFACE_AREA_M2,
                DEFAULT_CONVECTIVE_H,
                DEFAULT_EMISSIVITY
            );
        float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;
        json += "\"th\":" + String(thresholdValue, 2);

        json += "}";
        if (i < chargeLog.size() - 1) json += ",";
    }
    json += "]";
    return json;
}

String getJsonIR() {
    String json = "{";
    auto addIRData = [&](const String& name, float data[][2], int count) {
        json += "\"" + name + "\":[";
        for (int i = 0; i < count; i++) {
            json += "[" + String(data[i][0], 3) + "," + String(data[i][1], 3) + "]";
            if (i < count - 1) json += ",";
        }
        json += "]";
    };
    addIRData("lu", internalResistanceData, resistanceDataCount); json += ",";
    addIRData("pairs", internalResistanceDataPairs, resistanceDataCountPairs);
    json += "}";
    return json;
}

void handleData() {
    String type = server.arg("type");
    if (type == "state") server.send(200, "application/json", getJsonState());
    else if (type == "history") server.send(200, "application/json", getJsonHistory());
    else if (type == "ambient") server.send(200, "application/json", getJsonAmbient());
    else if (type == "chargelog") server.send(200, "application/json", getJsonChargeLog());
    else if (type == "ir") server.send(200, "application/json", getJsonIR());
    else {
        String json = "{";
        json += "\"state\":" + getJsonState() + ",";
        json += "\"ambient\":" + getJsonAmbient();
        json += "}";
        server.send(200, "application/json", json);
    }
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Ni-MH Charger UI</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: sans-serif; background: #222; color: #eee; margin: 0; padding: 10px; }
        .container { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .panel { background: #333; padding: 10px; border-radius: 5px; border: 1px solid #444; position: relative; }
        canvas { width: 100%; height: 250px; background: #000; display: block; margin-top: 5px; }
        .controls { grid-column: span 2; display: flex; gap: 10px; flex-wrap: wrap; margin-bottom:10px; }
        button { padding: 10px 20px; font-size: 16px; cursor: pointer; background: #555; color: #fff; border: none; border-radius: 3px; }
        button:hover { background: #666; }
        .status { margin-bottom: 10px; font-size: 1.2em; display: flex; justify-content: space-between; flex-wrap: wrap; }
        @media (max-width: 800px) { .container { grid-template-columns: 1fr; } .controls { grid-column: span 1; } }
    </style>
</head>
<body>
    <div class="status">
        <div id="appState">IDLE</div>
        <div id="metrics">V: 0.000V | I: 0.000A | 0.000 mAh</div>
    </div>

    <div class="controls panel">
        <button onclick="sendCommand('charge')">START CHARGE</button>
        <button onclick="sendCommand('ir')">MEASURE IR</button>
        <button onclick="sendCommand('reset')">RESET Ah</button>
        <button onclick="sendCommand('stop')">STOP</button>
    </div>

    <div class="container">
        <div class="panel">
            Main Graph (V:Yellow, I:Magenta, T1:Red, T2:Green, dT:Blue)
            <canvas id="mainGraph"></canvas>
        </div>
        <div class="panel">
            Ambient / Mold (T:Red, Dew:Green, H:Blue)
            <canvas id="ambientGraph"></canvas>
        </div>
        <div class="panel">
            IR Graph (LU:White, Pairs:Cyan)
            <canvas id="irGraph"></canvas>
        </div>
        <div class="panel">
            Charge Log (V:Y, I:M, %:Grey, dT:B, /dT:R, RiMH:Or, Ri:Cy)
            <canvas id="chargeGraph"></canvas>
        </div>
    </div>

    <script>
        function sendCommand(cmd) {
            fetch('/command?cmd=' + cmd);
        }

        async function updateData() {
            try {
                const res = await fetch('/data');
                const data = await res.json();

                const appStates = ['IDLE', 'BUILDING_MODEL', 'MEASURING_IR', 'CHARGING'];
                document.getElementById('appState').innerText = appStates[data.state.app] || 'UNKNOWN';
                document.getElementById('metrics').innerText = `V: ${data.state.v.toFixed(3)}V | I: ${data.state.i.toFixed(3)}A | ${data.state.mah.toFixed(3)} mAh | Duty: ${data.state.duty}`;

                renderAmbient(data.ambient);

                if (Date.now() % 5000 < 1100) {
                   fetchLargeData();
                }

            } catch (e) { console.error(e); }
        }

        async function fetchLargeData() {
             try {
                const hres = await fetch('/data?type=history');
                renderMain(await hres.json());

                const irres = await fetch('/data?type=ir');
                renderIR(await irres.json());

                const cres = await fetch('/data?type=chargelog');
                renderCharge(await cres.json());
             } catch (e) { console.error(e); }
        }

        function renderMain(data) {
            const canvas = document.getElementById('mainGraph');
            const ctx = canvas.getContext('2d');
            const margin = { top: 20, right: 40, bottom: 30, left: 50 };
            ctx.clearRect(0,0,canvas.width, canvas.height);

            drawAxes(ctx, margin, 0, 320, "Time", 0, 40, "Value");
            drawSeries(ctx, data.v, 'yellow', 1.0, 2.0, margin);
            drawSeries(ctx, data.i, 'magenta', 0.0, 0.4, margin);
            drawSeries(ctx, data.t1, 'red', 15, 40, margin);
            drawSeries(ctx, data.t2, 'green', 15, 40, margin);
            drawSeries(ctx, data.td, 'blue', -0.5, 1.5, margin);
        }

        function renderAmbient(data) {
            const canvas = document.getElementById('ambientGraph');
            const ctx = canvas.getContext('2d');
            const margin = { top: 20, right: 40, bottom: 30, left: 50 };
            ctx.clearRect(0,0,canvas.width, canvas.height);

            drawAxes(ctx, margin, 0, 320, "Time", 0, 100, "T/H");
            drawSeries(ctx, data.t, 'red', 10, 40, margin);
            drawSeries(ctx, data.d, 'green', 10, 40, margin);
            drawSeries(ctx, data.h, 'blue', 0, 100, margin);

            // Draw Mold Threshold line
            const y = (canvas.height - margin.bottom) - (65 / 100) * (canvas.height - margin.top - margin.bottom);
            ctx.strokeStyle = 'orange';
            ctx.lineWidth = 1;
            ctx.setLineDash([5, 5]);
            ctx.beginPath(); ctx.moveTo(margin.left, y); ctx.lineTo(canvas.width - margin.right, y); ctx.stroke();
            ctx.setLineDash([]);
        }

        function renderIR(data) {
             const canvas = document.getElementById('irGraph');
             const ctx = canvas.getContext('2d');
             const margin = { top: 20, right: 40, bottom: 30, left: 50 };
             ctx.clearRect(0,0,canvas.width, canvas.height);

             drawAxes(ctx, margin, 0, 0.5, "Current (A)", 0, 1.0, "Resistance (Ω)");
             drawXY(ctx, data.lu, 'white', 0, 0.5, 0, 1.0, margin);
             drawXY(ctx, data.pairs, 'cyan', 0, 0.5, 0, 1.0, margin);
        }

        function renderCharge(data) {
             const canvas = document.getElementById('chargeGraph');
             const ctx = canvas.getContext('2d');
             const margin = { top: 20, right: 40, bottom: 30, left: 50 };
             ctx.clearRect(0,0,canvas.width, canvas.height);

             if(!data || !data.length) {
                drawAxes(ctx, margin, 0, 100, "Time", 0, 255, "Value");
                return;
             }

             drawAxes(ctx, margin, 0, data.length, "Index", 0, 255, "Scaled Val");
             drawSeries(ctx, data.map(d => d.i), 'magenta', 0.0, 0.4, margin);
             drawSeries(ctx, data.map(d => d.v), 'yellow', 1.0, 2.0, margin);
             drawSeries(ctx, data.map(d => d.d), 'grey', 0, 255, margin);
             drawSeries(ctx, data.map(d => d.td), 'blue', -0.5, 1.5, margin);
             drawSeries(ctx, data.map(d => d.th), 'red', -0.5, 1.5, margin);
             drawSeries(ctx, data.map(d => d.irlu), 'orange', 0.0, 1.5, margin);
             drawSeries(ctx, data.map(d => d.irp), 'cyan', 0.0, 1.5, margin);
        }

        function drawAxes(ctx, margin, xMin, xMax, xLabel, yMin, yMax, yLabel) {
            const w = ctx.canvas.width;
            const h = ctx.canvas.height;
            const plotW = w - margin.left - margin.right;
            const plotH = h - margin.top - margin.bottom;

            ctx.strokeStyle = '#666';
            ctx.fillStyle = '#aaa';
            ctx.lineWidth = 1;
            ctx.font = '10px sans-serif';

            // X Axis
            ctx.beginPath();
            ctx.moveTo(margin.left, h - margin.bottom);
            ctx.lineTo(w - margin.right, h - margin.bottom);
            ctx.stroke();

            // Y Axis
            ctx.beginPath();
            ctx.moveTo(margin.left, margin.top);
            ctx.lineTo(margin.left, h - margin.bottom);
            ctx.stroke();

            // Ticks X
            for (let i = 0; i <= 5; i++) {
                const x = margin.left + (i / 5) * plotW;
                const val = xMin + (i / 5) * (xMax - xMin);
                ctx.beginPath();
                ctx.moveTo(x, h - margin.bottom);
                ctx.lineTo(x, h - margin.bottom + 5);
                ctx.stroke();
                ctx.textAlign = 'center';
                ctx.fillText(val.toFixed(xMax - xMin < 2 ? 2 : 0), x, h - margin.bottom + 15);
            }

            // Ticks Y
            for (let i = 0; i <= 5; i++) {
                const y = h - margin.bottom - (i / 5) * plotH;
                const val = yMin + (i / 5) * (yMax - yMin);
                ctx.beginPath();
                ctx.moveTo(margin.left - 5, y);
                ctx.lineTo(margin.left, y);
                ctx.stroke();
                ctx.textAlign = 'right';
                ctx.fillText(val.toFixed(yMax - yMin < 2 ? 2 : 0), margin.left - 8, y + 3);
            }

            // Labels
            ctx.textAlign = 'center';
            ctx.fillText(xLabel, margin.left + plotW / 2, h - 5);

            ctx.save();
            ctx.translate(12, margin.top + plotH / 2);
            ctx.rotate(-Math.PI / 2);
            ctx.fillText(yLabel, 0, 0);
            ctx.restore();
        }

        function drawSeries(ctx, arr, color, min, max, margin) {
            if(!arr || !arr.length) return;
            const plotW = ctx.canvas.width - margin.left - margin.right;
            const plotH = ctx.canvas.height - margin.top - margin.bottom;

            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            ctx.beginPath();
            let first = true;
            arr.forEach((v, i) => {
                if (v === null || isNaN(v)) return;
                const x = margin.left + (i / (arr.length-1)) * plotW;
                const y = (ctx.canvas.height - margin.bottom) - ((v - min) / (max - min)) * plotH;
                if (first) { ctx.moveTo(x, y); first = false; } else ctx.lineTo(x, y);
            });
            ctx.stroke();
        }

        function drawXY(ctx, points, color, xMin, xMax, yMin, yMax, margin) {
             if(!points || !points.length) return;
             const plotW = ctx.canvas.width - margin.left - margin.right;
             const plotH = ctx.canvas.height - margin.top - margin.bottom;

             ctx.strokeStyle = color;
             ctx.lineWidth = 2;
             ctx.beginPath();
             points.forEach((p, i) => {
                 const x = margin.left + ((p[0] - xMin) / (xMax - xMin)) * plotW;
                 const y = (ctx.canvas.height - margin.bottom) - ((p[1] - yMin) / (yMax - yMin)) * plotH;
                 if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
             });
             ctx.stroke();
        }

        setInterval(updateData, 1000);
        updateData();
    </script>
</body>
</html>
)rawliteral";

void handleRoot() {
    server.send(200, "text/html", INDEX_HTML);
}

void handleCommand() {
    String cmd = server.arg("cmd");
    if (cmd == "charge") {
        resetAh = true;
        currentAppState = APP_STATE_BUILDING_MODEL;
    } else if (cmd == "ir") {
        currentAppState = APP_STATE_MEASURING_IR;
        extern IRState currentIRState;
        currentIRState = IR_STATE_START;
    } else if (cmd == "reset") {
        resetAh = true;
    } else if (cmd == "stop") {
        currentAppState = APP_STATE_IDLE;
        applyDuty(0);
    }
    server.send(200, "text/plain", "OK");
}
