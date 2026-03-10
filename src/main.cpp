// Energy2Shelly_ESP v0.6.0
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

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "This is the Energy2Shelly for ESP converter!\r\nDevice and Energy status is available under /status\r\nTo reset configuration, goto /reset\r\n");
  });

  server.on("/shelly", HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head><title>Reset Confirmation</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial,sans-serif;text-align:center;padding:20px;}";
    html += ".btn{padding:10px 20px;margin:10px;cursor:pointer;text-decoration:none;display:inline-block;border-radius:5px;font-size:16px;}";
    html += ".btn-yes{background-color:#d9534f;color:white;border:none;}";
    html += ".btn-no{background-color:#5bc0de;color:white;border:none;}</style></head><body>";
    html += "<h2>Reset Configuration?</h2>";
    html += "<p>Are you sure you want to reset the WiFi configuration? This will clear all settings and restart the device.</p>";
    html += "<form method='POST' action='/reset' style='display:inline;'>";
    html += "<button type='submit' class='btn btn-yes'>Yes, Reset</button>";
    html += "</form>";
    html += "<a href='/' class='btn btn-no'>Cancel</a>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    shouldResetConfig = true;
    request->send(200, "text/plain", "Resetting WiFi configuration, please log back into the hotspot to reconfigure...\r\n");
  });

  // Shelly RPC endpoints called via HTTP GET method
  server.on("/rpc/EM.GetConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/EM.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/EMData.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMDataGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/Shelly.GetComponents", HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetComponents();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Shelly.GetConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Shelly.GetDeviceInfo", HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Shelly.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/Sys.GetConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    sysGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Sys.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    sysGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/WiFi.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    wifiGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  // Shelly RPC endpoint called via HTTP POST method with JSON-RPC body
  server.on("/rpc", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr,
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
  handleblinkled();
}
