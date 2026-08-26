#ifndef HTML_HOME_H
#define HTML_HOME_H

#include <Arduino.h>

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
    <a href="/reset" class="reset">Reset Device</a>
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




#endif // HTML_HOME_H