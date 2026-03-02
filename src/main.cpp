// Energy2Shelly_ESP v0.6.0
#include <Arduino.h>

// Configuration & setup
#include "config/Configuration.h"

// Data structures & processing
#include "data/DataStructures.h"
#include "data/DataProcessing.h"

// Protocol parsers
#include "parsers/Parsers.h" 

void rpcWrapper() {
  JsonDocument jsonResponse;
  JsonDocument doc;
  deserializeJson(doc, serJsonResponse);
  jsonResponse["id"] = rpcId;
  jsonResponse["src"] = shelly_name;
  if (strcmp(rpcUser, "EMPTY") != 0) {
    jsonResponse["dst"] = rpcUser;
  }
  jsonResponse["result"] = doc;
  serializeJson(jsonResponse, serJsonResponse);
}

void blinkled(int duration) {
  if (led > 0) {
    if (led_i) {
      digitalWrite(led, HIGH);
    } else {
      digitalWrite(led, LOW);
    }
    ledOffTime = millis() + duration;
  }
}

void handleblinkled() {
  if (led > 0) {
    if (ledOffTime > 0 && millis() > ledOffTime) {
      if (led_i) {
        digitalWrite(led, LOW);
      } else {
        digitalWrite(led, HIGH);
      }
      ledOffTime = 0;
    }
  }
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly#shellygetdeviceinfo-example
void shellyGetDeviceInfo() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = shelly_name;
  jsonResponse["mac"] = shelly_mac;
  jsonResponse["slot"] = 1;
  jsonResponse["model"] = "SPEM-003CEBEU";
  jsonResponse["gen"] = atoi(shelly_gen);
  jsonResponse["fw_id"] = shelly_fw_id;
  jsonResponse["ver"] = "1.4.4";
  jsonResponse["app"] = "Pro3EM";
  jsonResponse["auth_en"] = false;
  jsonResponse["auth_domain"] = nullptr;
  jsonResponse["profile"] = "triphase";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("shellyGetDeviceInfo: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Sys#sysgetconfig-example
void sysGetConfig() {
  JsonDocument jsonResponse;
  jsonResponse["device"]["name"] = shelly_name;
  jsonResponse["device"]["mac"] = shelly_mac;
  jsonResponse["device"]["fw_id"] = shelly_fw_id;
  jsonResponse["device"]["eco_mode"] = false;
  jsonResponse["device"]["profile"] = "triphase";
  jsonResponse["device"]["discoverable"] = false;
  jsonResponse["location"]["tz"] = "Europe/Berlin";
  jsonResponse["location"]["lat"] = 54.306;
  jsonResponse["location"]["lon"] = 9.663;
  jsonResponse["debug"]["mqtt"]["enable"] = false;
  jsonResponse["debug"]["websocket"]["enable"] = false;
  jsonResponse["debug"]["udp"]["addr"] = nullptr;
  jsonResponse["ui_data"].to<JsonObject>();
  jsonResponse["rpc_udp"]["dst_addr"] = WiFi.localIP().toString();
  jsonResponse["rpc_udp"]["listen_port"] = shelly_port;
  jsonResponse["sntp"]["server"] = nullptr;
  jsonResponse["cfg_rev"] = 10;
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("sysGetConfig: ");
  DEBUG_SERIAL.println(serJsonResponse);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Sys#sysgetstatus-example
void sysGetStatus() {
  JsonDocument jsonResponse;

  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char time_buffer[6];
  strftime(time_buffer, sizeof(time_buffer), "%H:%M", &timeinfo);

  uint32_t ram_total;

#ifdef ESP32
  ram_total = ESP.getHeapSize();
#else
  ram_total = 0; // what makes sense here?
#endif

  jsonResponse["mac"] = shelly_mac;
  jsonResponse["restart_required"] = false;
  jsonResponse["time"] = time_buffer;
  jsonResponse["unixtime"] = now;
  jsonResponse["last_sync_ts"] = nullptr;
  jsonResponse["uptime"] = millis() / 1000;
  jsonResponse["ram_size"] = ram_total;
  jsonResponse["ram_free"] = ESP.getFreeHeap();
  jsonResponse["fs_size"] = ESP.getFlashChipSize();
  jsonResponse["fs_free"] = ESP.getFreeSketchSpace();
  jsonResponse["cfg_rev"] = 10;
  jsonResponse["kvs_rev"] = 2725;
  jsonResponse["schedule_rev"] = 0;
  jsonResponse["webhook_rev"] = 0;
  jsonResponse["btrelay_rev"] = 0;
  jsonResponse["avail_updates"].to<JsonObject>();
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("sysGetStatus: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EM#emgetstatus-example
void EMGetStatus() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = 0;
  jsonResponse["a_current"] = serialized(String(PhasePower[0].current, 2));
  jsonResponse["a_voltage"] = serialized(String(PhasePower[0].voltage, 2));
  jsonResponse["a_act_power"] = serialized(String(PhasePower[0].power, 2));
  jsonResponse["a_aprt_power"] = serialized(String(PhasePower[0].apparentPower, 2));
  jsonResponse["a_pf"] = serialized(String(PhasePower[0].powerFactor, 2));
  jsonResponse["a_freq"] = serialized(String(PhasePower[0].frequency, 2));
  jsonResponse["b_current"] = serialized(String(PhasePower[1].current, 2));
  jsonResponse["b_voltage"] = serialized(String(PhasePower[1].voltage, 2));
  jsonResponse["b_act_power"] = serialized(String(PhasePower[1].power, 2));
  jsonResponse["b_aprt_power"] = serialized(String(PhasePower[1].apparentPower, 2));
  jsonResponse["b_pf"] = serialized(String(PhasePower[1].powerFactor, 2));
  jsonResponse["b_freq"] = serialized(String(PhasePower[1].frequency, 2));
  jsonResponse["c_current"] = serialized(String(PhasePower[2].current, 2));
  jsonResponse["c_voltage"] = serialized(String(PhasePower[2].voltage, 2));
  jsonResponse["c_act_power"] = serialized(String(PhasePower[2].power, 2));
  jsonResponse["c_aprt_power"] = serialized(String(PhasePower[2].apparentPower, 2));
  jsonResponse["c_pf"] = serialized(String(PhasePower[2].powerFactor, 2));
  jsonResponse["c_freq"] = serialized(String(PhasePower[2].frequency, 2));
  jsonResponse["total_current"] = serialized(String((PhasePower[0].power + PhasePower[1].power + PhasePower[2].power) / defaultVoltage, 2));
  jsonResponse["total_act_power"] = serialized(String(PhasePower[0].power + PhasePower[1].power + PhasePower[2].power, 2));
  jsonResponse["total_aprt_power"] = serialized(String(PhasePower[0].apparentPower + PhasePower[1].apparentPower + PhasePower[2].apparentPower, 2));
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("EMGetStatus: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EMData#emdatagetstatus-example
void EMDataGetStatus() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = 0;
  jsonResponse["a_total_act_energy"] = serialized(String(PhaseEnergy[0].consumption, 2));
  jsonResponse["a_total_act_ret_energy"] = serialized(String(PhaseEnergy[0].gridfeedin, 2));
  jsonResponse["b_total_act_energy"] = serialized(String(PhaseEnergy[1].consumption, 2));
  jsonResponse["b_total_act_ret_energy"] = serialized(String(PhaseEnergy[1].gridfeedin, 2));
  jsonResponse["c_total_act_energy"] = serialized(String(PhaseEnergy[2].consumption, 2));
  jsonResponse["c_total_act_ret_energy"] = serialized(String(PhaseEnergy[2].gridfeedin, 2));
  jsonResponse["total_act"] = serialized(String(PhaseEnergy[0].consumption + PhaseEnergy[1].consumption + PhaseEnergy[2].consumption, 2));
  jsonResponse["total_act_ret"] = serialized(String(PhaseEnergy[0].gridfeedin + PhaseEnergy[1].gridfeedin + PhaseEnergy[2].gridfeedin, 2));
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("EMDataGetStatus: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EM#emgetconfig-example
void EMGetConfig() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = 0;
  jsonResponse["name"] = nullptr;
  jsonResponse["blink_mode_selector"] = "active_energy";
  jsonResponse["phase_selector"] = "a";
  jsonResponse["monitor_phase_sequence"] = true;
  jsonResponse["reverse"].to<JsonObject>();
  jsonResponse["ct_type"] = "120A";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("EMGetConfig: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly#shellygetconfig-example
void shellyGetConfig() {
  JsonDocument jsonResponse, tempDoc;
  jsonResponse["ble"]["enable"] = false;
  jsonResponse["cloud"]["enable"] = false;
  jsonResponse["cloud"]["server"] = nullptr;
  EMGetConfig();
  jsonResponse["em:0"] = serialized(serJsonResponse);
  sysGetConfig();
  jsonResponse["sys"] = serialized(serJsonResponse);
  jsonResponse["wifi"]["sta"]["ssid"] = WiFi.SSID();
  jsonResponse["wifi"]["sta"]["is_open"] = false;
  jsonResponse["wifi"]["sta"]["enable"] = true;
  jsonResponse["wifi"]["sta"]["ipv4mode"] = "dhcp";
  jsonResponse["wifi"]["sta"]["ip"] = WiFi.localIP().toString();
  jsonResponse["wifi"]["sta"]["netmask"] = WiFi.subnetMask().toString();
  jsonResponse["wifi"]["sta"]["gw"] = WiFi.gatewayIP().toString();
  jsonResponse["wifi"]["sta"]["nameserver"] = WiFi.dnsIP().toString();
  jsonResponse["wifi"]["ws"]["enable"] = false;
  jsonResponse["wifi"]["ws"]["server"] = nullptr;
  jsonResponse["wifi"]["ws"]["ssl_ca"] = "ca.pem";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("shellyGetConfig: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly#shellygetcomponents-example
void shellyGetComponents() {
  JsonDocument jsonResponse, comp1, comp2, tempDoc;
  JsonArray components = jsonResponse["components"].to<JsonArray>();
  comp1["key"] = "em:0";
  EMGetStatus();
  comp1["status"] = serialized(serJsonResponse);
  EMGetConfig();
  comp1["config"] = serialized(serJsonResponse);
  components.add(comp1);
  comp2["key"] = "emdata:0";
  EMDataGetStatus();
  comp2["status"] = serialized(serJsonResponse);
  comp2["config"].to<JsonObject>(); // no config for emdata
  components.add(comp2);
  jsonResponse["cfg_rev"] = 1;
  jsonResponse["offset"] = 0;
  jsonResponse["total"] = 2;
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("shellyGetComponents: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly#shellygetstatus-example
void shellyGetStatus() {
  JsonDocument jsonResponse;
  double temperature;
#ifdef ESP32
  temperature = temperatureRead();
#else
  temperature = 26.55;
#endif

  jsonResponse["ble"].to<JsonObject>();

  jsonResponse["cloud"]["connected"] = false;
  jsonResponse["mqtt"]["connected"] = false;

  EMGetStatus();
  jsonResponse["em:0"] = serialized(serJsonResponse);
  EMDataGetStatus();
  jsonResponse["emdata:0"] = serialized(serJsonResponse);

  // temperature is not really in the examples, but makes sense to include it
  JsonObject temp = jsonResponse["tmp"].to<JsonObject>();
  temp["tC"] = serialized(String(temperature, 2));
  temp["tF"] = serialized(String((temperature * 9.0 / 5.0) + 32.0, 2));

  sysGetStatus();
  jsonResponse["sys"] = serialized(serJsonResponse);

  jsonResponse["wifi"]["sta_ip"] = WiFi.localIP().toString();
  jsonResponse["wifi"]["status"] = (WiFi.status() == WL_CONNECTED);
  jsonResponse["wifi"]["ssid"] = WiFi.SSID();
  jsonResponse["wifi"]["rssi"] = WiFi.RSSI();

  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("shellyGetStatus: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

void scriptGetCode() {
  JsonDocument jsonResponse;
  jsonResponse["data"] = "";
  jsonResponse["left"] = "0";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("scriptGetCode: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

void scriptList() {
  JsonDocument jsonResponse;
  jsonResponse["scripts"].to<JsonArray>();
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("scriptList: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Wifi#wifigetstatus-example
void wifiGetStatus() {
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  JsonDocument jsonResponse;
  jsonResponse["sta_ip"] = WiFi.localIP() ? WiFi.localIP().toString() : "null";
  switch (WiFi.status()) {
    case WL_CONNECTED:
      jsonResponse["status"] = "connected";
      break;
    case WL_DISCONNECTED:
      jsonResponse["status"] = "disconnected";
      break;
    default:
      jsonResponse["status"] = WiFi.localIP() ? "got ip" : "connecting";
      break;
  }
  jsonResponse["ssid"] = wifiConnected ? WiFi.SSID() : "null";
  jsonResponse["bssid"] = wifiConnected ? WiFi.BSSIDstr() : "null";
  jsonResponse["rssi"] = WiFi.RSSI();
  jsonResponse["ap_client_count"] = 0; // not really relevant, as we are not in AP mode, but included for completeness
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("wifiGetStatus: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  JsonDocument json;
  switch (type) {
    case WS_EVT_DISCONNECT:
      DEBUG_SERIAL.printf("[%u] Websocket: disconnected!\n", client->id());
      break;
    case WS_EVT_CONNECT:
      DEBUG_SERIAL.printf("[%u] Websocket: connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DATA:
      {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
          data[len] = 0;
          deserializeJson(json, data);
          rpcId = json["id"];
          if (json["method"] == "Shelly.GetDeviceInfo") {
            strcpy(rpcUser, "EMPTY");
            shellyGetDeviceInfo();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Shelly.GetComponents") {
            strcpy(rpcUser, "EMPTY");
            shellyGetComponents();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Shelly.GetConfig") {
            strcpy(rpcUser, "EMPTY");
            shellyGetConfig();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Shelly.GetStatus") {
            strcpy(rpcUser, "EMPTY");
            shellyGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "EM.GetStatus") {
            strcpy(rpcUser, json["src"]);
            EMGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "EMData.GetStatus") {
            strcpy(rpcUser, json["src"]);
            EMDataGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "EM.GetConfig") {
            EMGetConfig();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Script.GetCode") {
            scriptGetCode();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Script.List") {
            scriptList();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "WiFi.GetStatus") {
            wifiGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else {
            DEBUG_SERIAL.printf("Websocket: unknown request: %s\n", data);
          }
        }
        break;
      }
    case WS_EVT_PING:
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void parseUdpRPC() {
  uint8_t buffer[1024];
  int packetSize = UdpRPC.parsePacket();
  if (packetSize) {
    JsonDocument json;
    int rSize = UdpRPC.read(buffer, 1024);
    buffer[rSize] = 0;
    DEBUG_SERIAL.print("Received UDP packet on port 1010: ");
    DEBUG_SERIAL.println((char *)buffer);
    deserializeJson(json, buffer);
    if (json["method"].is<JsonVariant>()) {
      rpcId = json["id"];
      strcpy(rpcUser, "EMPTY");
      UdpRPC.beginPacket(UdpRPC.remoteIP(), UdpRPC.remotePort());
      if (json["method"] == "Shelly.GetDeviceInfo") {
        shellyGetDeviceInfo();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "Shelly.GetComponents") {
        shellyGetComponents();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "Shelly.GetConfig") {
        shellyGetConfig();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "Shelly.GetStatus") {
        shellyGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "EM.GetStatus") {
        EMGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "EMData.GetStatus") {
        EMDataGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "EM.GetConfig") {
        EMGetConfig();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else {
        DEBUG_SERIAL.printf("RPC over UDP: unknown request: %s\n", buffer);
      }
      UdpRPC.endPacket();
    }
  }
}

void parseHttpRPC(String requestBody, AsyncWebServerRequest *request) {
  if (request && requestBody) {
    JsonDocument json;
    DEBUG_SERIAL.print("Received HTTP RPC request: ");
    DEBUG_SERIAL.println(requestBody);
    deserializeJson(json, requestBody);
    if (json["method"].is<JsonVariant>()) {
      rpcId = json["id"];
      // strcpy(rpcUser, "EMPTY");
      if (json["method"] == "Shelly.GetDeviceInfo") {
        shellyGetDeviceInfo();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "Shelly.GetComponents") {
        shellyGetComponents();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "Shelly.GetConfig") {
        shellyGetConfig();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "Shelly.GetStatus") {
        shellyGetStatus();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "EM.GetStatus") {
        EMGetStatus();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "EMData.GetStatus") {
        EMDataGetStatus();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "EM.GetConfig") {
        EMGetConfig();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else {
        DEBUG_SERIAL.printf("RPC over HTTP: unknown request: %s\n", requestBody);
      }
    }
  }
}

void setup(void) {
  DEBUG_SERIAL.begin(115200);
  WifiManagerSetup();

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
    startMillis = millis();
    http.useHTTP10(true);
  }

  // Set up mDNS responder
  setupMdns();
}

void loop() {
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
     currentMillis = millis();
    if (currentMillis - startMillis_sunspec >= period) {
       parseSUNSPEC();
      startMillis_sunspec = currentMillis;
    }
   
  }
  if (dataHTTP) {
    currentMillis = millis();
    if (currentMillis - startMillis >= period) {
      queryHTTP();
      startMillis = currentMillis;
    }
  }
  handleblinkled();
}
