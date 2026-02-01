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
  body { font-family: Arial, sans-serif; text-align: center; padding: 40px; background-color: #f4f4f4; color: #333; }
  h1 { color: #0056b3; }
  p { font-size: 1.1em; }
  .nav { margin-top: 30px; }
  .nav a { display: inline-block; padding: 12px 24px; margin: 8px; background-color: #007bff; color: white; text-decoration: none; border-radius: 5px; transition: background-color 0.3s; }
  .nav a:hover { background-color: #0056b3; }
  .nav a.reset { background-color: #d9534f; }
  .nav a.reset:hover { background-color: #c9302c; }
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
