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
        json += "\"irp\":" + String(chargeLog[i].internalResistancePairs, 3);
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
        .panel { background: #333; padding: 10px; border-radius: 5px; border: 1px solid #444; }
        canvas { width: 100%; height: 200px; background: #000; display: block; margin-top: 5px; }
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
            Main Graph (V:Yellow, I:Magenta, T1:Red, T2:Green)
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
            Charge Log (V, I)
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
            ctx.clearRect(0,0,canvas.width, canvas.height);

            drawSeries(ctx, data.v, 'yellow', 1.0, 2.0);
            drawSeries(ctx, data.i, 'magenta', 0.0, 0.4);
            drawSeries(ctx, data.t1, 'red', 15, 40);
            drawSeries(ctx, data.t2, 'green', 15, 40);
        }

        function renderAmbient(data) {
            const canvas = document.getElementById('ambientGraph');
            const ctx = canvas.getContext('2d');
            ctx.clearRect(0,0,canvas.width, canvas.height);

            drawSeries(ctx, data.t, 'red', 10, 40);
            drawSeries(ctx, data.d, 'green', 10, 40);
            drawSeries(ctx, data.h, 'blue', 0, 100);

            // Draw Mold Threshold line
            const y = canvas.height - (65 / 100) * canvas.height;
            ctx.strokeStyle = 'orange';
            ctx.lineWidth = 1;
            ctx.setLineDash([5, 5]);
            ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(canvas.width, y); ctx.stroke();
            ctx.setLineDash([]);
        }

        function renderIR(data) {
             const canvas = document.getElementById('irGraph');
             const ctx = canvas.getContext('2d');
             ctx.clearRect(0,0,canvas.width, canvas.height);

             drawXY(ctx, data.lu, 'white', 0, 0.5, 0, 1.0);
             drawXY(ctx, data.pairs, 'cyan', 0, 0.5, 0, 1.0);
        }

        function renderCharge(data) {
             const canvas = document.getElementById('chargeGraph');
             const ctx = canvas.getContext('2d');
             ctx.clearRect(0,0,canvas.width, canvas.height);

             if(!data || !data.length) return;

             drawSeries(ctx, data.map(d => d.v), 'yellow', 1.0, 2.0);
             drawSeries(ctx, data.map(d => d.i), 'magenta', 0.0, 0.4);
        }

        function drawSeries(ctx, arr, color, min, max) {
            if(!arr || !arr.length) return;
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            ctx.beginPath();
            let first = true;
            arr.forEach((v, i) => {
                if (v === null) return;
                const x = (i / (arr.length-1)) * ctx.canvas.width;
                const y = ctx.canvas.height - ((v - min) / (max - min)) * ctx.canvas.height;
                if (first) { ctx.moveTo(x, y); first = false; } else ctx.lineTo(x, y);
            });
            ctx.stroke();
        }

        function drawXY(ctx, points, color, xMin, xMax, yMin, yMax) {
             if(!points || !points.length) return;

             ctx.strokeStyle = color;
             ctx.lineWidth = 2;
             ctx.beginPath();
             points.forEach((p, i) => {
                 const x = ((p[0] - xMin) / (xMax - xMin)) * ctx.canvas.width;
                 const y = ctx.canvas.height - ((p[1] - yMin) / (yMax - yMin)) * ctx.canvas.height;
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
