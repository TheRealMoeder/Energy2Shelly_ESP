#include <Arduino.h>

// Configuration & setup
#include "config/Configuration.h"

// Data structures & processing
#include "data/DataStructures.h"
#include "data/DataProcessing.h"

// Protocol parsers
#include "parsers/Parsers.h" 

// RPC handlers
#include "rpc/RpcHandlers.h"
#include "rpc/RpcComm.h"

void setup(void) {
  DEBUG_SERIAL.begin(115200);
  WifiManagerSetup();

  // Initialize time via NTP
#ifdef ESP32
  configTime(0, 0, ntp_server);
  setenv("TZ", timezone, 1);
  tzset();
#else
  // ESP8266
  configTime(timezone, ntp_server);
#endif
  while (!getLocalTime(&timeinfo))
  {
    DEBUG_SERIAL.println("Waiting for NTP time...");
    delay(500);
  }
  DEBUG_SERIAL.print("Current time: ");
  char time_buffer[20];
  strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  DEBUG_SERIAL.println(time_buffer);

  if (String(led_gpio).toInt() > 0) {
    led = String(led_gpio).toInt();
  }

  if (led > 0) {
    pinMode(led, OUTPUT);
    if (led_i) {
      digitalWrite(led, LOW);
    } else {
      digitalWrite(led, HIGH);
    }
  }

  // Set up web server and endpoints

  server.on("/", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = R"=====(
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
  .data-container { max-width: 1000px; margin: 0 auto; background: white; border-radius: 10px; padding: 20px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
  .data-section { margin: 20px 0; }
  .data-section h2 { color: #0056b3; border-bottom: 2px solid #007bff; padding-bottom: 10px; margin-bottom: 15px; font-size: 1.3em; }
  .phase-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; margin-bottom: 20px; }
  .phase-card { background: #f8f9fa; padding: 15px; border-radius: 8px; border-left: 4px solid #007bff; }
  .phase-card.phase-a { border-left-color: #dc3545; }
  .phase-card.phase-b { border-left-color: #ffc107; }
  .phase-card.phase-c { border-left-color: #28a745; }
  .phase-card h3 { margin: 0 0 10px 0; font-size: 1.1em; }
  .data-row { display: flex; justify-content: space-between; padding: 5px 0; border-bottom: 1px solid #dee2e6; }
  .data-row:last-child { border-bottom: none; }
  .data-label { font-weight: 600; color: #555; }
  .data-value { color: #007bff; font-weight: bold; }
  .totals { background: #e7f3ff; padding: 15px; border-radius: 8px; margin-top: 15px; }
  .totals .data-row { border-bottom: 1px solid #b3d9ff; }
  .timestamp { text-align: center; color: #6c757d; font-size: 0.9em; margin-top: 15px; font-style: italic; }
  .loading { color: #6c757d; font-style: italic; }
  .error { color: #dc3545; padding: 10px; background: #f8d7da; border-radius: 5px; }
</style>
</head>
<body>
  <h1>Energy2Shelly ESP</h1>
  <p>This device emulates a Shelly Pro 3EM to integrate various energy meters.</p>
  <div class="nav">
    <a href="/status">View Status</a>
    <a href="/config">Change Configuration</a>
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
  </div>

<script>
function formatValue(value, unit, decimals = 2) {
  if (value === null || value === undefined) return 'N/A';
  return parseFloat(value).toFixed(decimals) + ' ' + unit;
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
      html += '</div>';

      html += '<div class="totals"><h3 style="margin-top:0;">Totals</h3>';
      html += `<div class="data-row"><span class="data-label">Total Current:</span><span class="data-value">${formatValue(data.total_current, 'A')}</span></div>`;
      html += `<div class="data-row"><span class="data-label">Total Power:</span><span class="data-value">${formatValue(data.total_act_power, 'W')}</span></div>`;
      html += `<div class="data-row"><span class="data-label">Total Apparent:</span><span class="data-value">${formatValue(data.total_aprt_power, 'VA')}</span></div>`;
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
          <div class="data-row"><span class="data-label">Consumption:</span><span class="data-value">${formatValue(data[phase.prefix + '_total_act_energy'], 'Wh')}</span></div>
          <div class="data-row"><span class="data-label">Grid Feed-in:</span><span class="data-value">${formatValue(data[phase.prefix + '_total_act_ret_energy'], 'Wh')}</span></div>
        </div>`;
      });
      html += '</div>';

      html += '<div class="totals"><h3 style="margin-top:0;">Totals</h3>';
      html += `<div class="data-row"><span class="data-label">Total Consumption:</span><span class="data-value">${formatValue(data.total_act, 'Wh')}</span></div>`;
      html += `<div class="data-row"><span class="data-label">Total Grid Feed-in:</span><span class="data-value">${formatValue(data.total_act_ret, 'Wh')}</span></div>`;
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
refreshData();

// Auto-refresh every 5 seconds
setInterval(refreshData, 5000);
</script>
</body>
</html>
)=====";
    request->send(200, "text/html", html);
  });

  server.on("/shelly", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/status", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/reset", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head><title>Reset Confirmation</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial,sans-serif;text-align:center;padding:20px;}";
    html += ".btn{padding:10px 20px;margin:10px;cursor:pointer;text-decoration:none;display:inline-block;border-radius:5px;font-size:16px;}";
    html += ".btn-yes{background-color:#d9534f;color:white;border:none;}";
    html += ".btn-no{background-color:#5bc0de;color:white;border:none;}</style></head><body>";
    html += "<h2>Reset Configuration?</h2>";
    html += "<p>Are you sure you want to reset the WiFi configuration? This will clear current WiFi settings and restart the device in AP mode.</p>";
    html += "<form method='POST' style='display:inline;' accept-charset='UTF-8'>";
    if (reset_password != nullptr && strlen(reset_password) > 0) {
      html += "<input type='password' name='reset_password' placeholder='Enter reset password' required><br/>";
    }
    html += "<button type='submit' class='btn btn-yes'>Yes, Reset</button>";
    html += "</form>";
    html += "<a href='/' class='btn btn-no'>Cancel</a>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/reset", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {
    if (reset_password != nullptr && strlen(reset_password) > 0) {
       if (request->hasParam("reset_password", true)) {
        if (String(reset_password) == request->getParam("reset_password", true)->value()) {
          shouldResetConfig = true;
          request->send(200, "text/plain", "Resetting WiFi configuration, please log back into the hotspot to reconfigure...\r\n");
        } else {
          request->send(403, "text/plain", "Unauthorized: Invalid reset password.\r\n");
        }
      } else {
        request->send(400, "text/plain", "Reset password missing.\r\n");
      }
    } else {
      shouldResetConfig = true;
      request->send(200, "text/plain", "Resetting WiFi configuration, please log back into the hotspot to reconfigure...\r\n");
    }
  });

  // Shelly RPC endpoints called via HTTP GET method
  server.on("/rpc/EM.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/EM.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/EMData.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    EMDataGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/Shelly.GetComponents", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetComponents();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Shelly.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Shelly.GetDeviceInfo", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Shelly.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/Sys.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sysGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Sys.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sysGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/WiFi.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    wifiGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  // Shelly RPC endpoint called via HTTP POST method with JSON-RPC body
  server.on("/rpc", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String rpcRequestBuffer;
      if (index == 0) {
        // New request, clear buffer
        rpcRequestBuffer = "";
      }
      // Append incoming data chunk to buffer
      rpcRequestBuffer += String((char *)data).substring(0, len);
      if (index + len >= total) {
        // All data received, process RPC request
        parseHttpRPC(rpcRequestBuffer, request);
      }
    }
  );

  webSocket.onEvent(webSocketEvent);
  server.addHandler(&webSocket);
  server.begin();

  // Set up RPC over UDP for Marstek users
  UdpRPC.begin(String(shelly_port).toInt()); 

  // Set up MQTT
  if (dataMQTT) {
    mqtt_client.setBufferSize(2048);
    if (isValidIPAddress(mqtt_server)) {
      mqtt_client.setServer(mqtt_server, String(mqtt_port).toInt());
    } else {
      mqtt_client.setServer(mqtt_server, String(mqtt_port).toInt());
    }
    mqtt_client.setCallback(mqtt_callback);
  }

  // Set Up Multicast for SMA Energy Meter
  if (dataSMA) {
    Udp.begin(multicastPort);
#ifdef ESP8266
    Udp.beginMulticast(WiFi.localIP(), multicastIP, multicastPort);
#else
    Udp.beginMulticast(multicastIP, multicastPort);
#endif
  }

  // Set Up UDP for SHRDZM smart meter interface
  if (dataSHRDZM) {
    Udp.begin(multicastPort);
  }

  // Set Up Modbus TCP for SUNSPEC register query
  if (dataSUNSPEC) {
    modbus1.client();
    modbus_ip.fromString(mqtt_server);
    if (!modbus1.isConnected(modbus_ip)) {  // reuse mqtt server adresss for modbus adress
      Serial.println("Trying to connect SUNSPEC powermeter data");
      modbus1.connect(modbus_ip, String(mqtt_port).toInt());
    }
  }

  // Set Up HTTP query
  if (dataHTTP) {
    period = atol(query_period);
    http.useHTTP10(true);
  }

  // Set up mDNS responder
  setupMdns();

  startMillis = millis();
}

void loop() {
  currentMillis = millis();
#ifndef ESP32
  MDNS.update();
#endif
  parseUdpRPC();
  if (shouldResetConfig) {
#ifdef ESP32
    WiFi.disconnect(true, true);
#else
    WiFi.disconnect(true);
#endif
    delay(1000);
    ESP.restart();
  }
  if (dataMQTT) {
    if (!mqtt_client.connected()) {
      mqtt_reconnect();
    }
    mqtt_client.loop();
  }
  if (dataSMA) {
    parseSMA();
  }
  if (dataSHRDZM) {
    parseSHRDZM();
  }
  if (dataSUNSPEC) {
    if (currentMillis - startMillis >= period) {
      parseSUNSPEC();
      startMillis = currentMillis;
    }
  }
  if (dataHTTP) {
    if (currentMillis - startMillis >= period) {
      queryHTTP();
      startMillis = currentMillis;
    }
  }
  if (dataTIBBERPULSE) {
    if (currentMillis - startMillis >= period) {
      parseTibberPulse();
      startMillis = currentMillis;
    }
  }
  handleblinkled();
}