#ifndef HTML_HOME_H
#define HTML_HOME_H

#include <Arduino.h>
#include "chart_js.h"

const char HTML_HOME[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<title>Energy2Shelly ESP</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: Arial, sans-serif; text-align: center; padding: 20px; background-color: #f4f4f4; color: #333; }
  h1 { color: #0056b3; margin-bottom: 10px; }
  p { font-size: 1.1em; margin-top: 5px; }
  .nav { margin: 30px 0; }
  .nav a { display: inline-block; padding: 12px 24px; margin: 8px; background-color: #007bff; color: white; text-decoration: none; border-radius: 5px; transition: background-color 0.3s; }
  .nav a:hover { background-color: #0056b3; }
  .nav a.reset { background-color: #d9534f; }
  .nav a.reset:hover { background-color: #c9302c; }
  .data-container { max-width: 1300px; margin: 0 auto; background: white; border-radius: 10px; padding: 20px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
  .data-section { margin: 20px 0; }
  .data-section h2 { color: #0056b3; border-bottom: 2px solid #007bff; padding-bottom: 10px; margin-bottom: 15px; font-size: 1.3em; }
  .phase-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; margin-bottom: 20px; }
  .phase-card { background: #f8f9fa; padding: 15px; border-radius: 8px; border-left: 4px solid #007bff; }
  .phase-card.phase-a { border-left-color: #dc3545; }
  .phase-card.phase-b { border-left-color: #ffc107; }
  .phase-card.phase-c { border-left-color: #28a745; }
  .phase-card.total { background: #e7f3ff; }
  .phase-card h3 { margin: 0 0 10px 0; font-size: 1.1em; }
  .data-row { display: flex; justify-content: space-between; padding: 5px 0; border-bottom: 1px solid #dee2e6; }
  .data-row:last-child { border-bottom: none; }
  .data-label { font-weight: 600; color: #555; }
  .data-value { color: #007bff; font-weight: bold; }
  .timestamp { text-align: center; color: #6c757d; font-size: 0.9em; margin-top: 15px; font-style: italic; }
  .loading { color: #6c757d; }
  .error { color: #dc3545; padding: 10px; background: #f8d7da; border-radius: 5px; }
</style>
</head>
<body>
  <h1>Energy2Shelly ESP</h1>
  <p>This device emulates a Shelly Pro 3EM to integrate various energy meters.</p>
  <div class="nav">
    <a href="/status">View Status</a>
    <a href="/console">Console</a>
    <a href="/plot">Plot Data</a>
    <a href="/update" class="reset">Update SW</a>
    <a href="/config" class="reset">Configure Parameters</a>
  </div>

  <div class="data-container">
    <div class="data-section">
      <h2>Current Power Data</h2>
      <div id="power-data" class="loading">Loading power data...</div>
    </div>

    <div class="data-section">
      <h2>Energy Data</h2>
      <div id="energy-data" class="loading">Loading energy data...</div>
    </div>

    <div class="timestamp" id="timestamp"></div>
    <div class="timestamp" id="version">Build: 
)====="
VERSION_BUILD
R"=====(</div>
  </div>

<script>
function formatValue(value, unit = '', decimals = 1, scale = 1) {
  if (value === null || value === undefined || isNaN(value)) return 'N/A';

  const num = Number(value) * scale;
  const formatted = num.toFixed(decimals);

  return unit ? `${formatted} ${unit}` : formatted;
}

function updatePowerData() {
  fetch('/rpc/EM.GetStatus')
    .then(response => response.json())
    .then(data => {
      const phases = [
        { name: 'Phase A', prefix: 'a', class: 'phase-a' },
        { name: 'Phase B', prefix: 'b', class: 'phase-b' },
        { name: 'Phase C', prefix: 'c', class: 'phase-c' }
      ];

      let html = '<div class="phase-grid">';
      phases.forEach(phase => {
        html += `<div class="phase-card ${phase.class}">
          <h3>${phase.name}</h3>
          <div class="data-row"><span class="data-label">Voltage:</span><span class="data-value">${formatValue(data[phase.prefix + '_voltage'], 'V')}</span></div>
          <div class="data-row"><span class="data-label">Current:</span><span class="data-value">${formatValue(data[phase.prefix + '_current'], 'A')}</span></div>
          <div class="data-row"><span class="data-label">Power:</span><span class="data-value">${formatValue(data[phase.prefix + '_act_power'], 'W')}</span></div>
          <div class="data-row"><span class="data-label">Apparent:</span><span class="data-value">${formatValue(data[phase.prefix + '_aprt_power'], 'VA')}</span></div>
          <div class="data-row"><span class="data-label">Power Factor:</span><span class="data-value">${formatValue(data[phase.prefix + '_pf'], '', 3)}</span></div>
          <div class="data-row"><span class="data-label">Frequency:</span><span class="data-value">${formatValue(data[phase.prefix + '_freq'], 'Hz')}</span></div>
        </div>`;
      });

      html += '<div class="phase-card total"><h3 style="margin-top:0;">Totals</h3>';
      html += `<div class="data-row"><span class="data-label">Total Current:</span><span class="data-value">${formatValue(data.total_current, 'A')}</span></div>`;
      html += `<div class="data-row"><span class="data-label">Total Power:</span><span class="data-value">${formatValue(data.total_act_power, 'W')}</span></div>`;
      html += `<div class="data-row"><span class="data-label">Total Apparent:</span><span class="data-value">${formatValue(data.total_aprt_power, 'VA')}</span></div>`;
      html += '</div>';
      html += '</div>';

      document.getElementById('power-data').innerHTML = html;
    })
    .catch(error => {
      document.getElementById('power-data').innerHTML = '<div class="error">Error loading power data: ' + error.message + '</div>';
    });
}

function updateEnergyData() {
  fetch('/rpc/EMData.GetStatus')
    .then(response => response.json())
    .then(data => {
      const phases = [
        { name: 'Phase A', prefix: 'a', class: 'phase-a' },
        { name: 'Phase B', prefix: 'b', class: 'phase-b' },
        { name: 'Phase C', prefix: 'c', class: 'phase-c' }
      ];

      let html = '<div class="phase-grid">';
      phases.forEach(phase => {
        html += `<div class="phase-card ${phase.class}">
          <h3>${phase.name}</h3>
          <div class="data-row"><span class="data-label">Consumption:</span><span class="data-value">${formatValue(data[phase.prefix + '_total_act_energy'], 'kWh', 2, 0.001)}</span></div>
          <div class="data-row"><span class="data-label">Grid Feed-in:</span><span class="data-value">${formatValue(data[phase.prefix + '_total_act_ret_energy'], 'kWh', 2, 0.001)}</span></div>
        </div>`;
      });

      html += '<div class="phase-card total"><h3 style="margin-top:0;">Totals</h3>';
      html += `<div class="data-row"><span class="data-label">Total Consumption:</span><span class="data-value">${formatValue(data.total_act, 'kWh', 2, 0.001)}</span></div>`;
      html += `<div class="data-row"><span class="data-label">Total Grid Feed-in:</span><span class="data-value">${formatValue(data.total_act_ret, 'kWh', 2, 0.001)}</span></div>`;
      html += '</div>';
      html += '</div>';

      document.getElementById('energy-data').innerHTML = html;
    })
    .catch(error => {
      document.getElementById('energy-data').innerHTML = '<div class="error">Error loading energy data: ' + error.message + '</div>';
    });
}

function updateTimestamp() {
  const now = new Date();
  document.getElementById('timestamp').textContent = 'Last updated: ' + now.toLocaleString();
}

function refreshData() {
  updatePowerData();
  updateEnergyData();
  updateTimestamp();
}

// Initial load
window.addEventListener('load', () => {
  setTimeout(() => {
    refreshData();
    
    
    setInterval(refreshData, 5000);
  }, 400); // 400ms delay to allow the page to render before fetching data
});
</script>
</body>
</html>
)=====";



// Single-file HTML & JavaScript source for the browser console
const char* htmlPage_console PROGMEM  = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Energy2Shelly_ESP HTTP Console</title>
    <style>
        body { background-color: #1e1e1e; color: #00ff00; font-family: monospace; padding: 20px; margin: 0; }
        h2 { color: #ffffff; font-family: sans-serif; margin-bottom: 10px; }
        #console { width: calc(100% - 20px); height: 500px; border: 1px solid #444; overflow-y: scroll; padding: 10px; background: #000; box-shadow: inset 0 0 10px #000; }
        .status { color: #888; font-style: italic; }
        .timestamp { color: #00aaaa; margin-right: 8px; }
        .controls-container { margin-top: 15px; display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; gap: 15px; width: calc(100% - 0px); }
        .btn-container { display: flex; gap: 10px; }
        .btn { background-color: #000; color: #00ff00; border: 1px solid #00ff00; padding: 8px 15px; font-family: monospace; cursor: pointer; transition: all 0.2s; }
        .btn:hover { background-color: #00ff00; color: #000; }
        .scroll-lock-label { color: #ffffff; font-family: sans-serif; font-size: 14px; display: flex; align-items: center; cursor: pointer; }
        .scroll-lock-label input { margin-right: 8px; cursor: pointer; }
    </style>
</head>
<body>
    <h2>Energy2Shelly_ESP Live Web Console</h2>
    <div id="console"><span class="status">[Connecting to console stream...]</span><br></div>
    <div class="controls-container">
        <div class="btn-container">
            <button class="btn" onclick="clearConsole()">Clear Console</button>
            <button class="btn" onclick="saveConsole()">Save Log</button>
        </div>
        <label class="scroll-lock-label">
            <input type="checkbox" id="autoscroll" checked> Enable Autoscroll
        </label>
    </div>
    <script>
        var wsPort = 8080; 
        var ws = new WebSocket('ws://' + window.location.hostname +  '/consolews');
        var consoleDiv = document.getElementById('console');
        var autoscrollCheck = document.getElementById('autoscroll');
        var messageBuffer = "";
        function getTimestamp() {
            var now = new Date();
            return '[' + now.toTimeString().split(' ')[0] + '] ';
        }
        ws.onopen = function() {
            consoleDiv.innerHTML += '<span class="timestamp">' + getTimestamp() + '</span><span style="color:#00ff00;">[Connected to WebSocket on Port ' + wsPort + ']</span><br>';
        };
        ws.onmessage = function(event) {
            messageBuffer += event.data;
            if (messageBuffer.indexOf('\n') !== -1) {
                var lines = messageBuffer.split('\n');
                messageBuffer = lines.pop();
                var timeTag = '<span class="timestamp">' + getTimestamp() + '</span>';
                lines.forEach(function(line) {
                    if (line.length > 0 || line === "") {
                        consoleDiv.innerHTML += timeTag + line + '<br>';
                    }
                });
                if (autoscrollCheck.checked) {
                    consoleDiv.scrollTop = consoleDiv.scrollHeight;
                }
            }
        };
        ws.onclose = function() {
            consoleDiv.innerHTML += '<span class="timestamp">' + getTimestamp() + '</span><span style="color:#ff0000; font-weight:bold;">[Disconnected from Server]</span><br>';
        };
        function clearConsole() {
            consoleDiv.innerHTML = '<span class="timestamp">' + getTimestamp() + '</span><span class="status">[Console cleared]</span><br>';
            messageBuffer = ""; 
        }
        function saveConsole() {
            var text = consoleDiv.innerText;
            var blob = new Blob([text], { type: 'text/plain' });
            var anchor = document.createElement('a');
            var now = new Date();
            var dateStr = now.toISOString().slice(0,10);
            var timeStr = now.toTimeString().split(' ')[0].replace(/:/g, '-');
            anchor.download = 'console_log_' + dateStr + '_' + timeStr + '.txt';
            anchor.href = window.URL.createObjectURL(blob);
            anchor.target = '_blank';
            anchor.style.display = 'none';
            document.body.appendChild(anchor);
            anchor.click();
            document.body.removeChild(anchor);
        }
    </script>
</body>
</html>
)rawliteral";

const char PASSWORD_INPUT_HTML[] PROGMEM = "<input type='password' name='reset_password' placeholder='Enter reset password' required><br/>";


const char RESET_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><title>Reset Confirmation</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>body{font-family:Arial,sans-serif;text-align:center;padding:20px;}
.btn{padding:10px 20px;margin:10px;cursor:pointer;text-decoration:none;display:inline-block;border-radius:5px;font-size:16px;}
.btn-yes{background-color:#d9534f;color:white;border:none;}
.btn-no{background-color:#5bc0de;color:white;border:none;}</style></head><body>
<h2>Reset Configuration?</h2>
<p>Are you sure you want to reset the WiFi configuration? This will clear current WiFi settings and restart the device in AP mode.</p>
<form method='POST' style='display:inline;' accept-charset='UTF-8'>
)rawhtml";

const char RESET_HTML_END[] PROGMEM = R"rawhtml(
<button type='submit' class='btn btn-yes'>Yes, Reset</button>
</form>
<a href='/' class='btn btn-no'>Cancel</a>
</body></html>
)rawhtml";


const char UPDATE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><title>Update Confirmation</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>body{font-family:Arial,sans-serif;text-align:center;padding:20px;}
.btn{padding:10px 20px;margin:10px;cursor:pointer;text-decoration:none;display:inline-block;border-radius:5px;font-size:16px;}
.btn-yes{background-color:#d9534f;color:white;border:none;}
.btn-no{background-color:#5bc0de;color:white;border:none;}</style></head><body>
<h2>Update Configuration?</h2>
<p>Are you sure you want to update the WiFi configuration?</p>
<form method='POST' style='display:inline;' accept-charset='UTF-8'>
)rawhtml";

const char UPDATE_HTML_END[] PROGMEM = R"rawhtml(
<button type='submit' class='btn btn-yes'>Yes, Update</button>
</form>
<a href='/' class='btn btn-no'>Cancel</a>
</body></html>
)rawhtml";


  
const char webplotter_html[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Energy2Shelly_ESP EM Live Power Dashboard</title>
    <!-- Chart.js via CDN -->
    <script src="/chart.min.js"></script>
    <style>
        .footer {
            margin-top: 30px;
            padding: 15px 0;
            text-align: center;
            font-size: 0.8rem;
            color: #9ca3af;
            border-top: 1px solid #e5e7eb;
        }
        .footer a {
            color: #6b7280;
            text-decoration: underline;
        }
        .footer a:hover {
            color: #374151;
        }
        body { font-family: Arial, sans-serif; text-align: center; padding: 20px; background-color: #f4f4f4; color: #333; }
        h1 { color: #0056b3; margin-bottom: 10px; }
        p { font-size: 1.1em; margin-top: 5px; }
        .nav { margin: 30px 0; }
        .nav a { display: inline-block; padding: 12px 24px; margin: 8px; background-color: #007bff; color: white; text-decoration: none; border-radius: 5px; transition: background-color 0.3s; }
        .nav a:hover { background-color: #0056b3; }
        .nav a.reset { background-color: #d9534f; }
        .nav a.reset:hover { background-color: #c9302c; }
        .data-container { max-width: 1300px; margin: 0 auto; background: white; border-radius: 10px; padding: 20px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .data-section { margin: 20px 0; }
        .data-section h2 { color: #0056b3; border-bottom: 2px solid #007bff; padding-bottom: 10px; margin-bottom: 15px; font-size: 1.3em; }
        .phase-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; margin-bottom: 20px; }
        .phase-card { background: #f8f9fa; padding: 15px; border-radius: 8px; border-left: 4px solid #007bff; }
        .phase-card.phase-a { border-left-color: #dc3545; }
        .phase-card.phase-b { border-left-color: #ffc107; }
        .phase-card.phase-c { border-left-color: #28a745; }
        .phase-card.total { background: #e7f3ff; }
        .phase-card h3 { margin: 0 0 10px 0; font-size: 1.1em; }
        .data-row { display: flex; justify-content: space-between; padding: 5px 0; border-bottom: 1px solid #dee2e6; }
        .data-row:last-child { border-bottom: none; }
        .data-label { font-weight: 600; color: #555; }
        .data-value { color: #007bff; font-weight: bold; }
        .timestamp { text-align: center; color: #6c757d; font-size: 0.9em; margin-top: 15px; font-style: italic; }
        .loading { color: #6c757d; }
        .error { color: #dc3545; padding: 10px; background: #f8d7da; border-radius: 5px; }
         .back-btn {
        display: inline-flex;
        align-items: center;
        padding: 10px 16px;
        background-color: #ffffff;
        color: #333333;
        border: 1px solid #cbd5e1;
        border-radius: 6px;
        font-size: 14px;
        font-weight: 600;
        cursor: pointer;
        margin-bottom: 20px;
         }
        .back-btn:hover {
            background-color: #e2e8f0;
        }     
    </style>
</head>
<body>
    <button onclick="goBack()" class="back-btn">
        ← Back
    </button>

    <script>
    function goBack() {
        if (document.referrer) {
            window.history.back();
        } else {
            window.location.href = 'index.html'; 
        }
    }
    </script>
    <div class="container">
    <h2>⚡ Power Consumption Trend</h2>

    <div class="chart-container">
        <canvas id="powerChart"></canvas>
    </div>

    <div id="power-data"></div>
    <footer class="footer">
        <p>Energy2Shelly_ESP EM Live Power Dashboard</p>
        <p>This software uses <a href="https://chartjs.org" target="_blank" rel="noopener">Chart.js</a> (Released under the MIT License, Copyright © 2025 Chart.js Contributors).</p>
    </footer>
</div>

<script>
    // Helperfunction for formating
    function formatValue(value, unit, decimals = 2) {
        if (value === undefined || value === null) return '-- ' + unit;
        return Number(value).toFixed(decimals) + ' ' + unit;
    }

    // init CHART.JS
    const ctx = document.getElementById('powerChart').getContext('2d');
    const maxDataPoints = 120; // Number of points to display on the chart at once

    const powerChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [], // filled dynamically with timestamps
            datasets: [
                { label: 'Phase A', data: [], borderColor: '#dc354599', backgroundColor: 'transparent', borderWidth: 2, tension: 0.2, pointRadius: 2 },
                { label: 'Phase B', data: [], borderColor: '#ffc10799', backgroundColor: 'transparent', borderWidth: 2, tension: 0.2, pointRadius: 3 },
                { label: 'Phase C', data: [], borderColor: '#28a74599', backgroundColor: 'transparent', borderWidth: 2, tension: 0.2, pointRadius: 5 },
                { label: 'Sum Power', data: [], borderColor: '#007bff99', backgroundColor: '#007bff10', borderWidth: 3, tension: 0.2, fill: true, pointRadius: 2 }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false, // allow CSS control of height
            scales: {
                y: {
                    beginAtZero: true,
                    title: { display: true, text: 'Power (Watt)' }
                },
                x: {
                    title: { display: true, text: 'Time' }
                }
            },
            plugins: {
                legend: { position: 'top' }
            }
        }
    });

    // Exchange Standard-Canvas-Control with fixed height
    ctx.canvas.parentNode.style.height = '350px';

    function updatePowerData() {
        fetch('/rpc/EM.GetStatus')
            .then(response => response.json())
            .then(data => {
                
                // Inject live data in graph
                const now_time = new Date();
                const timeString = now_time.toLocaleTimeString(undefined, { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });


                powerChart.data.labels.push(timeString);
                powerChart.data.datasets[0].data.push(Number(data.a_act_power || 0));
                powerChart.data.datasets[1].data.push(Number(data.b_act_power || 0));
                powerChart.data.datasets[2].data.push(Number(data.c_act_power || 0));
                powerChart.data.datasets[3].data.push(Number(data.total_act_power || 0));

                // Shift effect: Remove oldest values on the left if limit is exceeded
                if (powerChart.data.labels.length > maxDataPoints) {
                    powerChart.data.labels.shift();
                    powerChart.data.datasets.forEach(dataset => dataset.data.shift());
                }

                // display chart
                powerChart.update('none'); // 'none' deactivate internal standard animation for better performance at fast updates


                const phases = [
                    { name: 'Phase A', prefix: 'a', class: 'phase-a' },
                    { name: 'Phase B', prefix: 'b', class: 'phase-b' },
                    { name: 'Phase C', prefix: 'c', class: 'phase-c' }
                ];

                let html = '<div class="phase-grid">';
                phases.forEach(phase => {
                    html += `<div class="phase-card ${phase.class}">
                        <h3>${phase.name}</h3>
                        <div class="data-row"><span class="data-label">Voltage:</span><span class="data-value">${formatValue(data[phase.prefix + '_voltage'], 'V')}</span></div>
                        <div class="data-row"><span class="data-label">Current:</span><span class="data-value">${formatValue(data[phase.prefix + '_current'], 'A')}</span></div>
                        <div class="data-row"><span class="data-label">Power:</span><span class="data-value">${formatValue(data[phase.prefix + '_act_power'], 'W')}</span></div>
                        <div class="data-row"><span class="data-label">Apparent:</span><span class="data-value">${formatValue(data[phase.prefix + '_aprt_power'], 'VA')}</span></div>
                        <div class="data-row"><span class="data-label">Power Factor:</span><span class="data-value">${formatValue(data[phase.prefix + '_pf'], '', 3)}</span></div>
                        <div class="data-row"><span class="data-label">Frequency:</span><span class="data-value">${formatValue(data[phase.prefix + '_freq'], 'Hz')}</span></div>
                    </div>`;
                });

                html += '<div class="phase-card total"><h3>Totals</h3>';
                html += `<div class="data-row"><span class="data-label">Total Current:</span><span class="data-value">${formatValue(data.total_current, 'A')}</span></div>`;
                html += `<div class="data-row"><span class="data-label">Total Power:</span><span class="data-value">${formatValue(data.total_act_power, 'W')}</span></div>`;
                html += `<div class="data-row"><span class="data-label">Total Apparent:</span><span class="data-value">${formatValue(data.total_aprt_power, 'VA')}</span></div>`;
                html += '</div>';
                html += '</div>';

                document.getElementById('power-data').innerHTML = html;
            })
            .catch(err => console.error("Error fetching Shelly data:", err));
    }

    // --- TIMING / INTERVALL ---
    // force update by first entry
    updatePowerData();
    // and every 500 ms after
    setInterval(updatePowerData, 500);
</script>

</body>
</html>
)rawliteral";

const char UPDATE_SUCCESS_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><title>Rebooting...</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<meta charset='UTF-8'>
<style>
  body{font-family:Arial,sans-serif;text-align:center;padding:40px 20px;}
  .countdown{font-size:24px;font-weight:bold;color:#d9534f;margin:20px 0;}
  .loader {border: 5px solid #f3f3f3; border-top: 5px solid #d9534f; border-radius: 50%; width: 40px; height: 40px; animation: spin 1s linear infinite; margin: 20px auto;}
  @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
</style>
<script>
  let seconds = 20;
  function startCountdown() {
    const timer = setInterval(() => {
      seconds--;
      document.getElementById('count').textContent = seconds;
      if (seconds <= 0) {
        clearInterval(timer);
        window.location.href = '/'; // Forwarding to Root Page
      }
    }, 1000);
  }
  window.onload = startCountdown;
</script>
</head>
<body>
  <h2>System reboots...</h2>
  <p>The device will be reachable again in WiFiManager-Mode within your local network shortly.</p>
  <div class='loader'></div>
  <div class='countdown'>Redirecting in <span id='count'>20</span> seconds...</div>
</body></html>
)rawhtml";

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
  

#define X(name, val)      "\"" #name "\": " TOSTRING(val) ",\n"
#define X_LAST(name, val) "\"" #name "\": " TOSTRING(val) "\n"



const char CONFIG_PAGE_HTML[] PROGMEM = R"=====(  
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Energy2Shelly Config Editor</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .card { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 500px; margin: auto; }
        .form-group { margin-bottom: 15px; }
        label { display: block; font-weight: bold; margin-bottom: 5px; color: #333; }
        input { width: 100%; padding: 8px; box-sizing: border-box; border: 1px solid #ccc; border-radius: 4px; }
        button { background: #007bff; color: white; border: none; padding: 10px 15px; border-radius: 4px; cursor: pointer; width: 100%; font-size: 16px; }
        button:hover { background: #0056b3; }
        #status { margin-top: 15px; font-weight: bold; text-align: center; }
    </style>
</head>
<body>
    <div class="card">
        <h2>Energy2Shelly Config Editor</h2>
        
        <!-- Input fields will be dynamically inserted here by JavaScript -->
        <div id="configForm">Loading configuration...</div>
        
        <button id="saveBtn" style="margin-top: 15px;">Save Changes</button>
        <p id="status"></p>
    </div>

    <script>
        const formContainer = document.getElementById('configForm');
        const saveBtn = document.getElementById('saveBtn');
        const statusText = document.getElementById('status');
        let currentConfig = {};
        const CONFIG_DICTIONARY = {
            basic: {
                _label: "Basic Settings",
                _description: "General settings for the device.",
                reset_password: {
                    label: "Reset Password",
                    max_length : 33,
                    description: "For resetting the WiFi configuration and putting the device in AP / config mode"
                },
                input_type: {
                    label: "Input Type",
                    description: "Defines the type of input for the device.",
                    options: ["NONE", "MQTT", "HTTP", "SMA", "SUNSPEC", "RCT", "SHRDZM", "TIBBERPULSE" ] 
                },
                ntp_server: {
                    label: "NTP Server",
                    max_length : 40,
                    description: "The NTP server used for time synchronization. e.g. de.pool.ntp.org"
                },
                timezone: {
                    label: "Timezone",
                    max_length : 64,
                    description: "The timezone setting for the device. e.g. CET-1CEST,M3.5.0/2,M10.5.0/3"
                },
                phase_number: {
                    label: "Phase Number",
                    description: "The number of phases for the device (e.g., 1 or 3).",
                    options: [1, 3]
                },
                power_offset: {
                    label: "Power Offset (Watt)",
                    max_length : 10,
                    description: "The power offset value for calibration."
                }
            },
            led: {
                _label: "LED Settings",
                _description: "LED settings for the device.",
                led_gpio: {
                    label: "LED Pin",
                    max_length : 6,
                    description: "Please specify the LED pin number"
                },
                led_gpio_i: {
                    label: "Inverse LED",
                    description: "Invert LED on/off logic",
                    options: ["false", "true"] 
                }
            },
           sunspec: {
                _label: "SUNSPEC Settings",
                _description: "Settings related to SUNSPEC Modbus communication.",
                modbus_dev: {
                    label: "Modbus Device",
                    max_length : 6,
                    description: "The device number for Modbus communication (e.g., 71)."
                },
                modbus_ip: {
                    label: "Modbus Server IP",
                    max_length : 60,
                    description: "The IP address of the Modbus server."
                },
                modbus_port: {
                    label: "Modbus Port",
                    max_length : 6,
                    description: "The port number for Modbus communication. e.g. 502"
                },
                query_period: {
                    label: "Query Period (ms)",
                    description: "The period in milliseconds for querying the Modbus server."
                }
            },

            shelly: {
                _label: "Shelly Settings",
                _description: "Settings related to Shelly communication.",
                shelly_mac: {
                    label: "Shelly MAC Address",
                    max_length : 13,
                    description: "The MAC address of the Shelly device."
                },
                shelly_name: {
                    label: "Shelly Device Name",
                    max_length : 26,
                    description: "The name of the Shelly device. e.g. shellypro3em-"
                },
                shelly_gen: {
                    label: "Shelly Generation",
                    description: "The generation of the Shelly device (e.g., 1, 2, 3).",
                    options: [1, 2, 3]
                },
                shelly_fw_id: {
                    label: "Shelly Firmware ID",
                    max_length : 32,
                    description: "The firmware ID of the Shelly device. e.g. 20241011-114455/1.4.4-g6d2a586"
                },
                shelly_port: {
                    label: "Shelly Port",
                    description: "The port number for communication with the Shelly device. e.g. 2220"
                }
            },

           rct: {
                _label: "RCT Settings",
                _description: "Settings related to RCT inverter communication.",

                rct_host: {
                    label: "RCT hostname/IP",
                    max_length : 41,
                    description: "IP address or hostname of the RCT Power device (with integrated grid sensor)."
                },
                rct_port: {
                    label: "RCT port",
                    max_length : 6,
                    description: "Defaults to 8899."
                },
                query_period: {
                    label: "Query Period (ms)",
                    description: "The period in milliseconds for querying the RCT inverter."
                }
            },

           sma: {
                _label: "SMA Settings",
                _description: "Settings related to SMA Speedwire communication.",
                sma_id: {
                    label: "SMA serial number",
                    max_length : 17,
                    description: "Optional serial number if you have more than one SMA EM/HM in your network"
                }
            },

           tibber: {
                _label: "TibberPulse Settings",
                _description: "Settings related to TibberPulse communication.",
                tibber_host: {
                    label: "Tibber Hostnane/IP[:port]",
                    max_length : 41,
                    description: "The address of the  Tibber device (e.g. 192.168.0.1:8080)."
                },
                tibber_user: {
                    label: "Tibber User",
                    max_length : 11,
                    description: "Defaults to admin."
                },
                tibber_password: {
                    label: "Tibber Password",
                    max_length : 10,
                    description: "As Printed on bridge device: xxxx-xxxx."
                },
                tibber_nodeid: {
                    label: "Tibber Node ID",
                    max_length : 2,
                    description: "Defaults to 1, check the bridge web interface for the right nodeID."
                },
                query_period: {
                    label: "Query Period (ms)",
                    description: "The period in milliseconds for querying the Tibber device."
                }
            },


            mqtt: {
                _label: "MQTT Settings",
                _description: "Settings related to MQTT communication.",
                mqtt_server: {
                    label: "MQTT Hostname/IP[:port]",
                    max_length : 60,
                    description: "The address of the MQTT broker (e.g. 192.168.0.1:1883)."
                },
                mqtt_port: {
                    label: "MQTT Port",
                    max_length : 6,
                    description: "The port number for MQTT communication."
                },
                mqtt_user: {
                    label: "MQTT User",
                    max_length : 40,
                    description: "Defaults to admin."
                },
                mqtt_password: {
                    label: "MQTT Password",
                    max_length : 40,
                    description: "The password for the MQTT broker."
                },
                mqtt_topic: {
                    label: "MQTT Topic",
                    max_length : 90,
                    description: "The topic to which the device will read published data."
                }
                
            },   
            http: {
                _label: "HTTP Settings",
                _description: "Settings related to HTTP communication.",
                server_ip: {
                    label: "HTTP Hostname/IP[:port]",
                    max_length : 60,
                    description: "The address of the HTTP server (e.g. 192.168.0.1:8080)."
                },
                query_period: {
                    label: "Query Period (ms)",
                    description: "The period in milliseconds for querying the HTTP server."
                }              
            },
            json: {
                _label: "JSON Settings",
                _description: "Settings related to JSON communication. (HTTP/MQTT)",
                power_path: {
                    label: "Power Path",
                    description: "The path to retrieve power data from the JSON server.",
                    max_length : 60
                },
                pwr_export_path: {
                    label: "Power Export Path",
                    max_length : 60,
                    description: "The path to retrieve power export data from the JSON server."
                },
                power_l1_path: {
                    label: "Power L1 Path",
                    max_length : 60,
                    description: "The path to retrieve power data for phase L1 from the JSON server."
                },
                power_l2_path: {
                    label: "Power L2 Path",
                    max_length : 60,
                    description: "The path to retrieve power data for phase L2 from the JSON server."
                },
                power_l3_path: {
                    label: "Power L3 Path",
                    max_length : 60,
                    description: "The path to retrieve power data for phase L3 from the JSON server."
                },
                energy_in_path: {
                    label: "Energy In Path",
                    max_length : 60,
                    description: "The path to retrieve energy input data from the JSON server."
                },
                energy_out_path: {  
                    label: "Energy Out Path",
                    max_length : 60,
                    description: "The path to retrieve energy output data from the JSON server."
                }
            }
        };

   async function loadConfig() {
    try {
        const res = await fetch('/json_preferences');
        if (res.status === 401) {
            setTimeout(loadConfig, 500); 
            return; 
        }
        
        currentConfig = await res.json();
       
        formContainer.innerHTML = ''; 
        generateFormFromDictionary(CONFIG_DICTIONARY, currentConfig, formContainer);
        
        statusText.innerText = "Configuration successfully loaded.";
        statusText.style.color = "green";
    } catch (err) {
        console.error(err);
        statusText.innerText = "Error loading configuration.";
        statusText.style.color = "red";
    }
}
    const INPUT_TYPE_ENUM = {
)=====" INPUT_TYPE_LIST  R"=====(
   
};
function generateFormFromDictionary(dictionary, currentConfig, container, currentConfigNode = currentConfig, prefix = '') {
    Object.keys(dictionary).forEach(key => {
        const entry = dictionary[key];
       
        const currentPath = prefix ? `${prefix}_${key}` : key;

        if (entry && typeof entry === 'object' && !entry.label && !entry.options) {
            const fieldset = document.createElement('fieldset');
            fieldset.className = 'form-group-sub';
            fieldset.style.marginBottom = '25px';
            fieldset.style.padding = '15px';
            fieldset.style.border = '1px solid #ccc';
            fieldset.style.borderRadius = '5px';

            const legend = document.createElement('legend');
            legend.innerText = entry._label || key;
            legend.style.fontWeight = 'bold';
            legend.style.padding = '0 10px';
            fieldset.appendChild(legend);

            if (entry._description) {
                const desc = document.createElement('small');
                desc.innerText = entry._description;
                desc.style.display = 'block';
                desc.style.color = '#666';
                desc.style.marginBottom = '10px';
                fieldset.appendChild(desc);
            }

            const nextConfigNode = (currentConfigNode && currentConfigNode[key]) ? currentConfigNode[key] : {};

            generateFormFromDictionary(entry, currentConfig, fieldset, nextConfigNode, currentPath);
            container.appendChild(fieldset);
            return;
        }

        if (key.startsWith('_')) return;

        const group = document.createElement('div');
        group.className = 'form-group';
        group.style.marginBottom = '20px';

        const inputWrapper = document.createElement('div');
        inputWrapper.style.display = 'flex';
        inputWrapper.style.alignItems = 'center';

        const displayLabel = entry.label || key;
        const displayDesc = entry.description || '';

        const label = document.createElement('label');
        label.innerText = displayLabel;
        label.style.fontWeight = 'bold';
        label.style.display = 'block';

        let descElement = null;
        if (displayDesc) {
            descElement = document.createElement('small');
            descElement.innerText = displayDesc;
            descElement.style.display = 'block';
            descElement.style.color = '#666';
            descElement.style.marginBottom = '4px';
        }

        group.appendChild(label);
        if (descElement) group.appendChild(descElement);
        group.appendChild(inputWrapper);

        let input;
        const configValue = (currentConfigNode && currentConfigNode[key] !== undefined) ? currentConfigNode[key] : '';
        
        if (entry.options) {
            input = document.createElement('select');
            input.id = 'input_' + currentPath; 

            entry.options.forEach(optValue => {
                const option = document.createElement('option');
                option.text = optValue; 
                
                const mappedValue = INPUT_TYPE_ENUM[optValue] !== undefined ? INPUT_TYPE_ENUM[optValue] : optValue;
                option.value = mappedValue;

                if (configValue === mappedValue || String(configValue) === String(mappedValue) || configValue === optValue) {
                    option.selected = true;
                }
                input.appendChild(option);
            });

            inputWrapper.appendChild(input);
        } 
        else {
            input = document.createElement('input');
            input.id = 'input_' + currentPath; 
            if (entry.max_length) {
                input.maxLength = entry.max_length; 
            }
                    

            input.value = configValue;
            inputWrapper.appendChild(input);

       

            if (key.toLowerCase().includes('password')) {
                input.type = 'password';

                const toggleBtn = document.createElement('button');
                toggleBtn.type = 'button';
                toggleBtn.innerText = '👁️';
                toggleBtn.style.marginLeft = '8px';
                toggleBtn.style.cursor = 'pointer';
                toggleBtn.style.background = 'none';
                toggleBtn.style.border = 'none';

                toggleBtn.addEventListener('click', () => {
                    if (input.type === 'password') {
                        input.type = 'text';
                        toggleBtn.innerText = '🔒';
                    } else {
                        input.type = 'password';
                        toggleBtn.innerText = '👁️';
                    }
                });

                inputWrapper.appendChild(toggleBtn);
            } else {
                input.type = 'text';
            }
        }

        container.appendChild(group);
    });
}
async function saveConfig() {
    const castValue = (val) => isNaN(val) || val.trim() === "" ? val : Number(val);

    function buildPayload(configObj, dictionaryObj, prefix = '') {
        const result = {};

        Object.keys(configObj).forEach(key => {
            if (key.startsWith('_')) return;

            const currentPath = prefix ? `${prefix}_${key}` : key;
            
            const inputField = document.getElementById('input_' + currentPath);
          
            if (inputField) {
                result[key] = castValue(inputField.value);
            } else if (typeof configObj[key] === 'object' && configObj[key] !== null) {
                result[key] = buildPayload(configObj[key], dictionaryObj[key] || {}, currentPath);
            } else {
                result[key] = configObj[key];
            }
        });

        return result;
    }

    const payload = buildPayload(currentConfig, CONFIG_DICTIONARY);

    try {
        statusText.innerText = "Saving to device...";
        statusText.style.color = "orange";

        const res = await fetch('/json_preferences', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload) 
        });

        if (res.ok) {
            statusText.innerText = "Successfully saved to flash storage!";
            statusText.style.color = "green";
        } else {
            statusText.innerText = "Device rejected the save request.";
            statusText.style.color = "red";
        }
    } catch (err) {
        console.error("Netzwerkfehler:", err);
        statusText.innerText = "Network error while saving.";
        statusText.style.color = "red";
    }
}
            saveBtn.addEventListener('click', saveConfig);
        window.addEventListener('DOMContentLoaded', loadConfig);
        loadConfig(); // Initial load
    </script>
</body>
</html>
)=====";
#undef X
#undef X_LAST
#endif // HTML_HOME_H