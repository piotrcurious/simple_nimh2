#ifndef DASHBOARD_HTML_H
#define DASHBOARD_HTML_H

#include <Arduino.h>

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
        let maxDT = 1.5;

        function sendCommand(cmd) {
            fetch('/command?cmd=' + cmd);
        }

        async function updateData() {
            try {
                const res = await fetch('/data');
                const data = await res.json();

                maxDT = data.state.max_dt || 1.5;
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
            const margin = { top: 20, right: 60, bottom: 30, left: 50 };
            ctx.clearRect(0,0,canvas.width, canvas.height);

            drawAxes(ctx, margin, 0, 320, "Time", 0, 40, "Value");
            drawSeries(ctx, data.v, 'yellow', 1.0, 2.0, margin, "V");
            drawSeries(ctx, data.i, 'magenta', 0.0, 0.4, margin, "I");
            drawSeries(ctx, data.t1, 'red', 15, 40, margin, "T1");
            drawSeries(ctx, data.t2, 'green', 15, 40, margin, "T2");
            drawSeries(ctx, data.td, 'blue', -0.5, maxDT, margin, "dT");
        }

        function renderAmbient(data) {
            const canvas = document.getElementById('ambientGraph');
            const ctx = canvas.getContext('2d');
            const margin = { top: 20, right: 60, bottom: 30, left: 50 };
            ctx.clearRect(0,0,canvas.width, canvas.height);

            drawAxes(ctx, margin, 0, 320, "Time", 0, 100, "T/H");
            drawSeries(ctx, data.t, 'red', 10, 40, margin, "T");
            drawSeries(ctx, data.d, 'green', 10, 40, margin, "Dew");
            drawSeries(ctx, data.h, 'blue', 0, 100, margin, "H");

            // Draw Mold Threshold line
            const y = (canvas.height - margin.bottom) - (65 / 100) * (canvas.height - margin.top - margin.bottom);
            ctx.strokeStyle = 'rgba(255, 165, 0, 0.6)';
            ctx.lineWidth = 1;
            ctx.setLineDash([5, 5]);
            ctx.beginPath(); ctx.moveTo(margin.left, y); ctx.lineTo(canvas.width - margin.right, y); ctx.stroke();
            ctx.setLineDash([]);
        }

        function renderIR(data) {
             const canvas = document.getElementById('irGraph');
             const ctx = canvas.getContext('2d');
             const margin = { top: 20, right: 60, bottom: 30, left: 50 };
             ctx.clearRect(0,0,canvas.width, canvas.height);

             const allPoints = (data.lu || []).concat(data.pairs || []);
             let xMin = 0, xMax = 0.5, yMin = 0, yMax = 0.5;

             if (allPoints.length > 0) {
                 xMin = Math.min(...allPoints.map(p => p[0]));
                 xMax = Math.max(...allPoints.map(p => p[0]));
                 yMin = Math.min(...allPoints.map(p => p[1]));
                 yMax = Math.max(...allPoints.map(p => p[1]));

                 // Add 10% padding
                 const xRange = xMax - xMin || 0.1;
                 const yRange = yMax - yMin || 0.1;
                 xMin = Math.max(0, xMin - xRange * 0.1);
                 xMax = xMax + xRange * 0.1;
                 yMin = Math.max(0, yMin - yRange * 0.1);
                 yMax = yMax + yRange * 0.1;
             }

             drawAxes(ctx, margin, xMin, xMax, "Current (A)", yMin, yMax, "Resistance (Ω)");
             drawXY(ctx, data.lu, 'white', xMin, xMax, yMin, yMax, margin, "LU");
             drawXY(ctx, data.pairs, 'cyan', xMin, xMax, yMin, yMax, margin, "Pairs");
        }

        function renderCharge(data) {
             const canvas = document.getElementById('chargeGraph');
             const ctx = canvas.getContext('2d');
             const margin = { top: 20, right: 80, bottom: 30, left: 50 };
             ctx.clearRect(0,0,canvas.width, canvas.height);

             if(!data || !data.length) {
                drawAxes(ctx, margin, 0, 100, "Time", 0, 255, "Value");
                return;
             }

             drawAxes(ctx, margin, 0, data.length, "Index", 0, 255, "Scaled Val");

             const tdArr = data.map(d => d.td);
             const thArr = data.map(d => d.th);
             // Use a larger scale for thermal data to ensure visibility
             const tMin = -1.0, tMax = 10.0;
             drawRegion(ctx, tdArr, thArr, 'rgba(255, 0, 0, 0.2)', tMin, tMax, margin);

             drawSeries(ctx, data.map(d => d.i), 'magenta', 0.0, 0.5, margin, "I");
             drawSeries(ctx, data.map(d => d.v), 'yellow', 1.0, 2.0, margin, "V");
             drawSeries(ctx, data.map(d => d.d), 'grey', 0, 255, margin, "Duty");
             drawSeries(ctx, data.map(d => d.td), 'blue', tMin, tMax, margin, "dT");
             drawSeries(ctx, data.map(d => d.th), 'red', tMin, tMax, margin, "Th");
             drawSeries(ctx, data.map(d => d.irlu), 'orange', 0.0, 1.0, margin, "RiLU");
             drawSeries(ctx, data.map(d => d.irp), 'cyan', 0.0, 1.0, margin, "RiP");
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

        function drawSeries(ctx, arr, color, min, max, margin, label = "") {
            if(!arr || !arr.length) return;
            const plotW = ctx.canvas.width - margin.left - margin.right;
            const plotH = ctx.canvas.height - margin.top - margin.bottom;

            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            ctx.beginPath();
            let lastX, lastY, lastVal;
            let first = true;
            arr.forEach((v, i) => {
                if (v === null || isNaN(v)) return;
                const x = margin.left + (i / (arr.length-1)) * plotW;
                const y = (ctx.canvas.height - margin.bottom) - ((v - min) / (max - min)) * plotH;
                if (first) { ctx.moveTo(x, y); first = false; } else ctx.lineTo(x, y);
                lastX = x; lastY = y; lastVal = v;
            });
            ctx.stroke();

            if (label && !isNaN(lastY)) {
                ctx.fillStyle = color;
                ctx.textAlign = 'left';
                ctx.font = 'bold 10px sans-serif';
                let yPos = Math.min(Math.max(lastY, margin.top + 10), ctx.canvas.height - margin.bottom - 2);
                ctx.fillText(`${label}:${lastVal.toFixed(2)}`, lastX + 5, yPos);
            }
        }

        function drawRegion(ctx, arr1, arr2, color, min, max, margin) {
            if(!arr1 || !arr1.length || !arr2 || !arr2.length) return;
            const plotW = ctx.canvas.width - margin.left - margin.right;
            const plotH = ctx.canvas.height - margin.top - margin.bottom;

            ctx.fillStyle = color;
            ctx.beginPath();
            let first = true;
            for(let i=0; i<arr1.length; i++) {
                if (arr1[i] === null || isNaN(arr1[i])) continue;
                const x = margin.left + (i / (arr1.length-1)) * plotW;
                const y = (ctx.canvas.height - margin.bottom) - ((arr1[i] - min) / (max - min)) * plotH;
                if (first) { ctx.moveTo(x, y); first = false; } else ctx.lineTo(x, y);
            }
            for(let i=arr2.length-1; i>=0; i--) {
                if (arr2[i] === null || isNaN(arr2[i])) continue;
                const x = margin.left + (i / (arr2.length-1)) * plotW;
                const y = (ctx.canvas.height - margin.bottom) - ((arr2[i] - min) / (max - min)) * plotH;
                ctx.lineTo(x, y);
            }
            ctx.closePath();
            ctx.fill();
        }

        function drawXY(ctx, points, color, xMin, xMax, yMin, yMax, margin, label = "") {
             if(!points || !points.length) return;
             const plotW = ctx.canvas.width - margin.left - margin.right;
             const plotH = ctx.canvas.height - margin.top - margin.bottom;

             ctx.strokeStyle = color;
             ctx.lineWidth = 2;
             ctx.beginPath();
             let lastX, lastY, lastVal;
             points.forEach((p, i) => {
                 const x = margin.left + ((p[0] - xMin) / (xMax - xMin)) * plotW;
                 const y = (ctx.canvas.height - margin.bottom) - ((p[1] - yMin) / (yMax - yMin)) * plotH;
                 if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
                 lastX = x; lastY = y; lastVal = p[1];
             });
             ctx.stroke();

             if (label && !isNaN(lastY)) {
                ctx.fillStyle = color;
                ctx.textAlign = 'left';
                ctx.font = 'bold 10px sans-serif';
                let yPos = Math.min(Math.max(lastY, margin.top + 10), ctx.canvas.height - margin.bottom - 2);
                ctx.fillText(`${label}:${lastVal.toFixed(2)}`, lastX + 5, yPos);
            }
        }

        function resizeAll() {
            const canvases = document.querySelectorAll('canvas');
            canvases.forEach(canvas => {
                const rect = canvas.getBoundingClientRect();
                if (canvas.width !== rect.width || canvas.height !== rect.height) {
                    canvas.width = rect.width;
                    canvas.height = rect.height;
                }
            });
        }

        window.addEventListener('resize', resizeAll);
        resizeAll();
        setInterval(updateData, 1000);
        updateData();
    </script>
</body>
</html>
)rawliteral";

#endif
