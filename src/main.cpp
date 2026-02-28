
                
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

// Web content
#include "web/html_home.h"
#include "web/html_reset.h"
#include "web/svg_favicon.h"

// Security
#include "security/CsrfProtection.h"

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
  DEBUG_SERIAL.println(F("\n\n=== SETUP START ==="));
  DEBUG_SERIAL.println(F("About to call WifiManagerSetup()"));

  // WiFi & Configuration initialization
  WifiManagerSetup();
  DEBUG_SERIAL.println(F("WifiManagerSetup() completed"));

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
    request->send(200, F("text/html"), FPSTR(HTML_HOME));
  });

  // Configuration page - more specific routes first
  server.on("/config/export", HTTP_GET, handleExportConfig);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  DEBUG_SERIAL.println(F("Config routes registered"));

  // Test endpoint to verify server is working
  server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request) {
    DEBUG_SERIAL.println(F("Test endpoint hit"));
    request->send(200, F("text/plain"), F("Server is working!"));
  });

  // Status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, F("application/json"), serJsonResponse);
  });

  // Reset WiFi configuration (GET - show confirmation)
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, F("text/html"), FPSTR(HTML_RESET));
  });

  // Reset WiFi configuration (POST - perform reset)
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    // CSRF protection: verify request came from this device
    if (!validateCsrfToken(request)) {
      DEBUG_SERIAL.println(F("CSRF attempt blocked on /reset endpoint"));
      request->send(403, F("text/plain"), F("Forbidden: Invalid request origin"));
      return;
    }

    shouldResetConfig = true;
    request->send(200, F("text/plain"),
                  F("Resetting WiFi configuration, please log back into the "
                    "hotspot to reconfigure...\r\n"));
  });

  // RPC endpoints (REST-style)
  server.on("/rpc/EM.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, F("application/json"), serJsonResponse);
  });

  server.on("/rpc/EMData.GetStatus", HTTP_GET,
            [](AsyncWebServerRequest *request) {
              EMDataGetStatus();
              request->send(200, F("application/json"), serJsonResponse);
            });

  server.on("/rpc/EM.GetConfig", HTTP_GET,
            [](AsyncWebServerRequest *request) {
              EMGetConfig();
              request->send(200, F("application/json"), serJsonResponse);
            });

  server.on("/rpc/Shelly.GetDeviceInfo", HTTP_GET,
            [](AsyncWebServerRequest *request) {
              GetDeviceInfo();
              request->send(200, F("application/json"), serJsonResponse);
            });

  // Generic RPC POST endpoint
  server.on("/rpc", HTTP_POST, [](AsyncWebServerRequest *request) {
    GetDeviceInfo();
    rpcWrapper();
    request->send(200, F("application/json"), serJsonResponse);
  });

  // Favicon endpoint
  server.on("/favicon.svg", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, F("image/svg+xml"), FPSTR(SVG_FAVICON));
  });

  // WebSocket handler for RPC
  webSocket.onEvent(webSocketEvent);
  server.addHandler(&webSocket);
  server.begin();

  DEBUG_SERIAL.println(F("Web server started successfully"));
  DEBUG_SERIAL.print(F("Access at: http://"));
  DEBUG_SERIAL.println(WiFi.localIP());

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
      DEBUG_SERIAL.println(F("MQTT server not configured - skipping MQTT setup"));
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
      DEBUG_SERIAL.println(F("SUNSPEC server not configured - skipping SUNSPEC setup"));
      dataSUNSPEC = false;
    } else {
      modbus1.client();
      modbus_ip.fromString(mqtt_server);
      if (!modbus1.isConnected(modbus_ip)) {
        modbus1.connect(modbus_ip, mqttPortInt);
        DEBUG_SERIAL.println(F("Trying to connect SUNSPEC powermeter data"));
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

  DEBUG_SERIAL.println(F("=== SETUP COMPLETE ==="));
}

// ============================================================================
// LOOP - Main event loop
// ============================================================================

void loop() {
  // FIX: Call millis() once per loop iteration and reuse the value everywhere.
  // Avoids redundant calls and ensures consistent timestamps within one cycle.
  currentMillis = millis();

  // FIX: Non-blocking reboot - replaces delay(1000) which starved networking.
  // Sets a future timestamp on first trigger, restarts only when time has elapsed.
  if (shouldReboot) {
    static unsigned long rebootAt = 0;
    if (rebootAt == 0) rebootAt = currentMillis + 1000;
    if (currentMillis >= rebootAt) ESP.restart();
  }

  // ESP8266 mDNS update
#ifndef ESP32
  MDNS.update();
#endif

  // UDP RPC polling (Marstek)
  parseUdpRPC();

  // FIX: Non-blocking reset - same pattern as reboot above.
  if (shouldResetConfig) {
    static unsigned long resetAt = 0;
    if (resetAt == 0) resetAt = currentMillis + 1000;
    if (currentMillis >= resetAt) {
#ifdef ESP32
      WiFi.disconnect(true, true);
#else
      WiFi.disconnect(true);
#endif
      ESP.restart();
    }
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
  // FIX: Reuses currentMillis captured at top of loop instead of calling millis() again.
  if (dataSUNSPEC) {
    if (currentMillis - startMillis_sunspec >= period) {
      parseSUNSPEC();
      startMillis_sunspec = currentMillis;
    }
  }

  // HTTP query polling (periodic)
  // FIX: Same - reuses currentMillis, no redundant millis() call.
  if (dataHTTP) {
    if (currentMillis - startMillis >= period) {
      queryHTTP();
      startMillis = currentMillis;
    }
  }

  // LED blinking
  handleblinkled();

  // FIX: Periodic WebSocket cleanup to free memory from stale/disconnected clients.
  // Critical for long-running ESP devices to prevent heap fragmentation.
  webSocket.cleanupClients();

  // FIX: yield() on ESP8266 feeds the watchdog timer and allows the WiFi stack
  // to process background tasks. Prevents random resets under heavy load.
  // Not needed on ESP32 which uses FreeRTOS and manages this automatically.
#ifndef ESP32
  yield();
#endif
}
