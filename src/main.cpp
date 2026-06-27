#include <Arduino.h>

#ifdef ESP32
#include "esp_task_wdt.h"
#endif

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

// Web content
#include "web/html_home.h"
#include <Preferences.h>

void handleSettingsGet(AsyncWebServerRequest *request) {
  String html = "<!DOCTYPE html><html><head><title>Energy2Shelly Settings</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial,sans-serif;max-width:600px;margin:0 auto;padding:20px;}";
  html += "label{display:block;margin-top:15px;font-weight:bold;}";
  html += "input,select{width:100%;padding:8px;margin-top:5px;box-sizing:border-box;}";
  html += "button{margin-top:20px;padding:10px 20px;background:#007bff;color:white;border:none;border-radius:5px;cursor:pointer;}";
  html += "button:hover{background:#0056b3;}</style></head><body>";
  html += "<h1>Energy2Shelly Settings</h1>";
  html += "<form method='POST'>";
  html += "<label>Shelly UDP Port (1010 or 2220)</label>";
  html += "<input name='shelly_port' value='" + String(shelly_port) + "'>";
  html += "<label>Phase Number (1 or 3)</label>";
  html += "<input name='phase_number' value='" + String(phase_number) + "'>";
  html += "<label>Power Offset (W)</label>";
  html += "<input name='power_offset' value='" + String(power_offset) + "'>";
  html += "<label>Query Period (ms)</label>";
  html += "<input name='query_period' value='" + String(query_period) + "'>";
  html += "<label>Reset Password</label>";
  html += "<input name='reset_password' type='password' value='" + String(reset_password) + "'>";
  html += "<button type='submit'>Save</button>";
  html += "</form>";
  html += "<p><a href='/'>Back to Dashboard</a></p>";
  html += "</body></html>";
  request->send(200, "text/html", html);
}

void handleSettingsPost(AsyncWebServerRequest *request) {
  Preferences prefs;
  prefs.begin("settings", false);
  if (request->hasParam("shelly_port", true)) {
    strncpy(shelly_port, request->getParam("shelly_port", true)->value().c_str(), 5);
    prefs.putString("shelly_port", shelly_port);
  }
  if (request->hasParam("phase_number", true)) {
    strncpy(phase_number, request->getParam("phase_number", true)->value().c_str(), 1);
    prefs.putString("phase_number", phase_number);
  }
  if (request->hasParam("power_offset", true)) {
    strncpy(power_offset, request->getParam("power_offset", true)->value().c_str(), 9);
    prefs.putString("power_offset", power_offset);
  }
  if (request->hasParam("query_period", true)) {
    strncpy(query_period, request->getParam("query_period", true)->value().c_str(), 9);
    prefs.putString("query_period", query_period);
  }
  if (request->hasParam("reset_password", true)) {
    strncpy(reset_password, request->getParam("reset_password", true)->value().c_str(), 32);
    prefs.putString("reset_password", reset_password);
  }
  prefs.end();
  String html = "<!DOCTYPE html><html><head><title>Settings Saved</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='2;url=/settings'>";
  html += "<style>body{font-family:Arial,sans-serif;text-align:center;padding:50px;}</style></head><body>";
  html += "<h1>Settings saved!</h1><p>Restarting...</p></body></html>";
  request->send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

void setup(void) {
  DEBUG_SERIAL.begin(115200);

  // Initialize watchdog timer (30s timeout) - before WifiManagerSetup
#ifdef ESP32
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);
#endif

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
  uint32_t ntp_timeout = millis() + 10000;
  bool ntp_success = false;
  while (millis() < ntp_timeout) {
    if (getLocalTime(&timeinfo)) {
      ntp_success = true;
      break;
    }
    DEBUG_SERIAL.println(F("Waiting for NTP time..."));
    delay(500);
#ifdef ESP32
    esp_task_wdt_reset();
#endif
  }
  if (!ntp_success) {
    DEBUG_SERIAL.println(F("NTP timeout - setting default time"));
    time_t default_time = 1704067200; // 2024-01-01 00:00:00 UTC
    struct timeval tv = { .tv_sec = default_time, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    getLocalTime(&timeinfo);
  }
  DEBUG_SERIAL.print(F("Current time: "));
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
    request->send(200, "text/html", FPSTR(HTML_HOME));
  });

  server.on("/shelly", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/status", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/settings", AsyncWebRequestMethod::HTTP_GET, handleSettingsGet);
  server.on("/settings", AsyncWebRequestMethod::HTTP_POST, handleSettingsPost);

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
      Serial.println(F("Trying to connect SUNSPEC powermeter data"));
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
    // WiFiManager may leave persistent=false; force it so disconnect(true)
    // actually erases the SDK-saved credentials in flash.
    WiFi.persistent(true);
    WiFi.disconnect(true);
    ESP.eraseConfig();
#endif
    delay(1000);
    ESP.restart();
  }
  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_SERIAL.println(F("WiFi disconnected, reconnecting..."));
    WiFi.reconnect();
#ifdef ESP32
    esp_task_wdt_reset();
#endif
    delay(1000);
    return;
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
#ifdef ESP32
  esp_task_wdt_reset();
#endif
}