// Energy2Shelly_ESP v0.5.2
// Main orchestration - delegates to modular components

#include <Arduino.h>

// Configuration & setup
#include "config/Configuration.h"
#include "config/WebConfig.h"

// Data structures & processing
#include "data/DataStructures.h"
#include "data/DataProcessor.h"

// Protocol parsers
#include "parsers/MqttParser.h" // Will implement as header stubs
#include "parsers/SmaParser.h"
#include "parsers/ShrDzmParser.h"
#include "parsers/SunspecParser.h"
#include "parsers/HttpParser.h"

// RPC handlers
#include "rpc/RpcHandlers.h"
#include "rpc/RpcComm.h"

// Hardware
#include "hardware/MdnsSetup.h"

// Forward declarations for parser functions
extern void parseSMA();
extern void parseSHRDZM();
extern void parseSUNSPEC();
extern void queryHTTP();
extern void parseUdpRPC();
extern void mqtt_callback(char *topic, byte *payload, unsigned int length);
extern void mqtt_reconnect();

// ============================================================================
// SETUP - Initialize all subsystems
// ============================================================================

void setup(void) {
  // Serial initialization for debugging
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("\n\n=== SETUP START ===");
  DEBUG_SERIAL.println("About to call WifiManagerSetup()");

  // WiFi & Configuration initialization
  WifiManagerSetup();
  DEBUG_SERIAL.println("WifiManagerSetup() completed");

  // LED setup
  led = ledGpioInt;
  if (led > 0) {
    pinMode(led, OUTPUT);
    if (led_i) {
      digitalWrite(led, LOW);
    } else {
      digitalWrite(led, HIGH);
    }
  }

  // ========================================================================
  // HTTP SERVER ROUTES
  // ========================================================================

  // Home page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
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

  // Configuration page
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/save", HTTP_POST, handleSave);

  // Status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  // Reset WiFi configuration (GET - show confirmation)
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html =
        "<!DOCTYPE html><html><head><title>Reset Confirmation</title>";
    html +=
        "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial,sans-serif;text-align:center;"
            "padding:20px;}";
    html += ".btn{padding:10px "
            "20px;margin:10px;cursor:pointer;text-decoration:none;display:"
            "inline-block;border-radius:5px;font-size:16px;}";
    html += ".btn-yes{background-color:#d9534f;color:white;border:none;}";
    html += ".btn-no{background-color:#5bc0de;color:white;border:none;}</"
            "style></head><body>";
    html += "<h2>Reset Configuration?</h2>";
    html += "<p>Are you sure you want to reset the WiFi configuration? This "
            "will clear all settings and restart the device.</p>";
    html += "<form method='POST' action='/reset' style='display:inline;'>";
    html += "<button type='submit' class='btn btn-yes'>Yes, Reset</button>";
    html += "</form>";
    html += "<a href='/' class='btn btn-no'>Cancel</a>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  // Reset WiFi configuration (POST - perform reset)
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    shouldResetConfig = true;
    request->send(200, "text/plain",
                  "Resetting WiFi configuration, please log back into the "
                  "hotspot to reconfigure...\r\n");
  });

  // RPC endpoints (REST-style)
  server.on("/rpc/EM.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/EMData.GetStatus", HTTP_GET,
            [](AsyncWebServerRequest *request) {
              EMDataGetStatus();
              request->send(200, "application/json", serJsonResponse);
            });

  server.on("/rpc/EM.GetConfig", HTTP_GET,
            [](AsyncWebServerRequest *request) {
              EMGetConfig();
              request->send(200, "application/json", serJsonResponse);
            });

  server.on("/rpc/Shelly.GetDeviceInfo", HTTP_GET,
            [](AsyncWebServerRequest *request) {
              GetDeviceInfo();
              request->send(200, "application/json", serJsonResponse);
            });

  // Generic RPC POST endpoint
  server.on("/rpc", HTTP_POST, [](AsyncWebServerRequest *request) {
    GetDeviceInfo();
    rpcWrapper();
    request->send(200, "application/json", serJsonResponse);
  });

  // WebSocket handler for RPC
  webSocket.onEvent(webSocketEvent);
  server.addHandler(&webSocket);
  server.begin();

  // ========================================================================
  // COMMUNICATIONS SETUP
  // ========================================================================

  // UDP RPC for Marstek compatibility
  UdpRPC.begin(shellyPortInt);

  // MQTT setup (if enabled)
  if (dataMQTT) {
    if (mqtt_server[0] != '\0') {
      mqtt_client.setBufferSize(2048);
      IPAddress mqttIP;
      if (mqttIP.fromString(mqtt_server)) {
        // It's an IP address
        mqtt_client.setServer(mqttIP, mqttPortInt);
      } else {
        // It's a hostname
        mqtt_client.setServer(mqtt_server, mqttPortInt);
      }
      mqtt_client.setCallback(mqtt_callback);
      mqtt_configured = true;
    } else {
      DEBUG_SERIAL.println("MQTT server not configured - skipping MQTT setup");
      dataMQTT = false;
      mqtt_configured = false;
    }
  }

  // SMA multicast setup
  if (dataSMA) {
    Udp.begin(multicastPort);
#ifdef ESP8266
    Udp.beginMulticast(WiFi.localIP(), multicastIP, multicastPort);
#else
    Udp.beginMulticast(multicastIP, multicastPort);
#endif
  }

  // SHRDZM UDP setup
  if (dataSHRDZM) {
    Udp.begin(multicastPort);
  }

  // SUNSPEC Modbus TCP setup
  if (dataSUNSPEC) {
    if (mqtt_server[0] == '\0') {
      DEBUG_SERIAL.println(
          "SUNSPEC server not configured - skipping SUNSPEC setup");
      dataSUNSPEC = false;
    } else {
      modbus1.client();
      modbus_ip.fromString(mqtt_server);
      if (!modbus1.isConnected(modbus_ip)) {
        modbus1.connect(modbus_ip, mqttPortInt);
        DEBUG_SERIAL.println("Trying to connect SUNSPEC powermeter data");
      }
    }
  }

  // HTTP query setup
  if (dataHTTP) {
    period = queryPeriodMs;
    startMillis = millis();
    http.useHTTP10(true);
  }

  // mDNS setup
  setupMdns();

  DEBUG_SERIAL.println("=== SETUP COMPLETE ===");
}

// ============================================================================
// LOOP - Main event loop
// ============================================================================

void loop() {
  // Handle reboot request
  if (shouldReboot) {
    delay(1000);
    ESP.restart();
  }

  // ESP8266 mDNS update
#ifndef ESP32
  MDNS.update();
#endif

  // UDP RPC polling (Marstek)
  parseUdpRPC();

  // Handle reset request
  if (shouldResetConfig) {
#ifdef ESP32
    WiFi.disconnect(true, true);
#else
    WiFi.disconnect(true);
#endif
    delay(1000);
    ESP.restart();
  }

  // MQTT handling
  if (mqtt_configured) {
    if (!mqtt_client.connected()) {
      mqtt_reconnect();
    }
    mqtt_client.loop();
  }

  // SMA multicast polling
  if (dataSMA) {
    parseSMA();
  }

  // SHRDZM UDP polling
  if (dataSHRDZM) {
    parseSHRDZM();
  }

  // SUNSPEC Modbus polling (periodic)
  if (dataSUNSPEC) {
    currentMillis = millis();
    if (currentMillis - startMillis_sunspec >= period) {
      parseSUNSPEC();
      startMillis_sunspec = currentMillis;
    }
  }

  // HTTP query polling (periodic)
  if (dataHTTP) {
    currentMillis = millis();
    if (currentMillis - startMillis >= period) {
      queryHTTP();
      startMillis = currentMillis;
    }
  }

  // LED blinking
  handleblinkled();
}
