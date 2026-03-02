// Energy2Shelly_ESP v0.6.0
#include <Arduino.h>

// Configuration & setup
#include "config/Configuration.h"

// Data structures & processing
#include "data/DataStructures.h"

bool isValidIPAddress(const char* ipString) {
  IPAddress ip;
  return ip.fromString(ipString);
}

double round2(double value) {
  int ivalue = (int)(value * 100.0 + (value > 0.0 ? 0.5 : -0.5));

  // fix Marstek bug: make sure to have decimal numbers
  if(forcePwrDecimals && (ivalue % 100 == 0)) ivalue++;
  
  return ivalue / 100.0;
}

JsonVariant resolveJsonPath(JsonVariant variant, const char *path) {
  for (size_t n = 0; path[n]; n++) {
    // Not a full array support, but works for Shelly 3EM emeters array!
    if (path[n] == '[') {
      variant = variant[JsonString(path, n)][atoi(&path[n+1])];
      path += n + 4;
      n = 0;
    }
    if (path[n] == '.') {
      variant = variant[JsonString(path, n)];
      path += n + 1;
      n = 0;
    }
  }
  return variant[path];
}

void setPowerData(double totalPower) {
  for (int i = 0; i <= 2; i++) {
    PhasePower[i].power = round2(totalPower * 0.3333);
    PhasePower[i].voltage = round2(defaultVoltage);
    PhasePower[i].current = round2(PhasePower[i].power / PhasePower[i].voltage);
    PhasePower[i].apparentPower = round2(PhasePower[i].power);
    PhasePower[i].powerFactor = round2(defaultPowerFactor);
    PhasePower[i].frequency = defaultFrequency;
  }
  DEBUG_SERIAL.print("Current total power: ");
  DEBUG_SERIAL.println(totalPower);
}

void setPowerData(double phase1Power, double phase2Power, double phase3Power) {
  PhasePower[0].power = round2(phase1Power);
  PhasePower[1].power = round2(phase2Power);
  PhasePower[2].power = round2(phase3Power);
  for (int i = 0; i <= 2; i++) {
    PhasePower[i].voltage = round2(defaultVoltage);
    PhasePower[i].current = round2(PhasePower[i].power / PhasePower[i].voltage);
    PhasePower[i].apparentPower = round2(PhasePower[i].power);
    PhasePower[i].powerFactor = round2(defaultPowerFactor);
    PhasePower[i].frequency = defaultFrequency;
  }
  DEBUG_SERIAL.print("Current power L1: ");
  DEBUG_SERIAL.print(phase1Power);
  DEBUG_SERIAL.print(" - L2: ");
  DEBUG_SERIAL.print(phase2Power);
  DEBUG_SERIAL.print(" - L3: ");
  DEBUG_SERIAL.println(phase3Power);
}

void setEnergyData(double totalEnergyGridSupply, double totalEnergyGridFeedIn) {
  for (int i = 0; i <= 2; i++) {
    PhaseEnergy[i].consumption = round2(totalEnergyGridSupply * 0.3333);
    PhaseEnergy[i].gridfeedin = round2(totalEnergyGridFeedIn * 0.3333);
  }
  DEBUG_SERIAL.print("Total consumption: ");
  DEBUG_SERIAL.print(totalEnergyGridSupply);
  DEBUG_SERIAL.print(" - Total Grid Feed-In: ");
  DEBUG_SERIAL.println(totalEnergyGridFeedIn);
}



void setJsonPathPower(JsonDocument json) {
  // If the incoming JSON already uses Shelly 3EM field names, parse directly
  if (json["a_current"].is<JsonVariant>() || json["a_act_power"].is<JsonVariant>()) {
    DEBUG_SERIAL.println("Parsing direct Shelly 3EM payload");
    PhasePower[0].current = round2((double)json["a_current"].as<double>());
    PhasePower[0].voltage = round2((double)json["a_voltage"].as<double>());
    PhasePower[0].power = round2((double)json["a_act_power"].as<double>());
    PhasePower[0].apparentPower = round2((double)json["a_aprt_power"].as<double>());
    PhasePower[0].powerFactor = round2((double)json["a_pf"].as<double>());
    PhasePower[0].frequency = json["a_freq"].as<int>();

    PhasePower[1].current = round2((double)json["b_current"].as<double>());
    PhasePower[1].voltage = round2((double)json["b_voltage"].as<double>());
    PhasePower[1].power = round2((double)json["b_act_power"].as<double>());
    PhasePower[1].apparentPower = round2((double)json["b_aprt_power"].as<double>());
    PhasePower[1].powerFactor = round2((double)json["b_pf"].as<double>());
    PhasePower[1].frequency = json["b_freq"].as<int>();

    PhasePower[2].current = round2((double)json["c_current"].as<double>());
    PhasePower[2].voltage = round2((double)json["c_voltage"].as<double>());
    PhasePower[2].power = round2((double)json["c_act_power"].as<double>());
    PhasePower[2].apparentPower = round2((double)json["c_aprt_power"].as<double>());
    PhasePower[2].powerFactor = round2((double)json["c_pf"].as<double>());
    PhasePower[2].frequency = json["c_freq"].as<int>();

    // Optionally use total fields if present
    if (json["total_act_power"].is<JsonVariant>()) {
      double total = json["total_act_power"].as<double>();
      // distribute if individual phases missing or for logging
      DEBUG_SERIAL.print("Total power from payload: ");
      DEBUG_SERIAL.println(total);
    }
    return;
  }
  if (strcmp(power_path, "TRIPHASE") == 0) {
    DEBUG_SERIAL.println("resolving triphase");
    double power1 = resolveJsonPath(json, power_l1_path);
    double power2 = resolveJsonPath(json, power_l2_path);
    double power3 = resolveJsonPath(json, power_l3_path);
    DEBUG_SERIAL.println(power1);
    setPowerData(power1, power2, power3);
  } else {
    // Check if BOTH paths (Import = power_path, Export = pwr_export_path) are defined
    if ((strcmp(power_path, "") != 0) && (strcmp(pwr_export_path, "") != 0)) {
      DEBUG_SERIAL.println("Resolving net power (import - export)");
      double importPower = resolveJsonPath(json, power_path).as<double>();
      double exportPower = resolveJsonPath(json, pwr_export_path).as<double>();
      double netPower = importPower - exportPower;
      setPowerData(netPower);
    }
    // (FALLBACK): Only the normal power_path (import path) is defined (old logic)
    else if (strcmp(power_path, "") != 0) {
      DEBUG_SERIAL.println("Resolving monophase (single path only)");
      double power = resolveJsonPath(json, power_path).as<double>();
      setPowerData(power);
    }
  }
  if ((strcmp(energy_in_path, "") != 0) && (strcmp(energy_out_path, "") != 0)) {
    double energyIn = resolveJsonPath(json, energy_in_path);
    double energyOut = resolveJsonPath(json, energy_out_path);
    setEnergyData(energyIn, energyOut);
  }
}

// Helper: parse a raw Shelly JSON string and forward to setJsonPathPower
void parseShellyString(const char *jsonStr) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) {
    DEBUG_SERIAL.print("deserializeJson failed: ");
    DEBUG_SERIAL.println(err.c_str());
    return;
  }
  setJsonPathPower(doc);
}

void parseShellyString(const String &s) {
  parseShellyString(s.c_str());
}

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

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  JsonDocument json;
  deserializeJson(json, payload, length);
  setJsonPathPower(json);
}

void mqtt_reconnect() {
  DEBUG_SERIAL.print("Attempting MQTT connection...");
  if (mqtt_client.connect(shelly_name, String(mqtt_user).c_str(), String(mqtt_passwd).c_str())) {
    DEBUG_SERIAL.println("connected");
    mqtt_client.subscribe(mqtt_topic);
  } else {
    DEBUG_SERIAL.print("failed, rc=");
    DEBUG_SERIAL.print(mqtt_client.state());
    DEBUG_SERIAL.println(" try again in 5 seconds");
    delay(5000);
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

void parseSMA() {
  uint8_t buffer[1024];
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int rSize = Udp.read(buffer, 1024);
    if (buffer[0] != 'S' || buffer[1] != 'M' || buffer[2] != 'A') {
      DEBUG_SERIAL.println("Not an SMA packet?");
      return;
    }
    uint16_t grouplen;
    uint16_t grouptag;
    uint8_t *offset = buffer + 4;
    do {
      grouplen = (offset[0] << 8) + offset[1];
      grouptag = (offset[2] << 8) + offset[3];
      offset += 4;
      if (grouplen == 0xffff) return;
      if (grouptag == 0x02A0 && grouplen == 4) {
        offset += 4;
      } else if (grouptag == 0x0010) {
        uint8_t *endOfGroup = offset + grouplen;
        // uint16_t protocolID = (offset[0] << 8) + offset[1];
        offset += 2;
        // uint16_t susyID = (offset[0] << 8) + offset[1];
        offset += 2;
        uint32_t serial = (offset[0] << 24) + (offset[1] << 16) + (offset[2] << 8) + offset[3];
        DEBUG_SERIAL.print("Received SMA multicast from ");
        DEBUG_SERIAL.println(serial);
        if ((strcmp(sma_id, "") != 0) && (String(sma_id).toInt() != serial)) {
          DEBUG_SERIAL.println("SMA serial not matching - ignoring packet");
          break;
        }
        offset += 4;
        // uint32_t timestamp = (offset[0] << 24) + (offset[1] << 16) + (offset[2] << 8) + offset[3];
        offset += 4;
        while (offset < endOfGroup) {
          uint8_t channel = offset[0];
          uint8_t index = offset[1];
          uint8_t type = offset[2];
          // uint8_t tarif = offset[3];
          offset += 4;
          if (type == 8) {
            uint64_t data = ((uint64_t)offset[0] << 56) + ((uint64_t)offset[1] << 48) + ((uint64_t)offset[2] << 40) + ((uint64_t)offset[3] << 32) + ((uint64_t)offset[4] << 24) + ((uint64_t)offset[5] << 16) + ((uint64_t)offset[6] << 8) + offset[7];
            offset += 8;
            switch (index) {
              case 21:
                PhaseEnergy[0].consumption = data / 3600000;
                break;
              case 22:
                PhaseEnergy[0].gridfeedin = data / 3600000;
                break;
              case 41:
                PhaseEnergy[1].consumption = data / 3600000;
                break;
              case 42:
                PhaseEnergy[1].gridfeedin = data / 3600000;
                break;
              case 61:
                PhaseEnergy[2].consumption = data / 3600000;
                break;
              case 62:
                PhaseEnergy[2].gridfeedin = data / 3600000;
                break;
            }
          } else if (type == 4) {
            uint32_t data = (offset[0] << 24) + (offset[1] << 16) + (offset[2] << 8) + offset[3];
            offset += 4;
            switch (index) {
              case 1:
                // 1.4.0 Total grid power in dW - unused
                break;
              case 2:
                // 2.4.0 Total feed-in power in dW - unused
                break;
              case 21:
                PhasePower[0].power = round2(data * 0.1);
                PhasePower[0].frequency = defaultFrequency;
                break;
              case 22:
                PhasePower[0].power -= round2(data * 0.1);
                break;
              case 29:
                PhasePower[0].apparentPower = round2(data * 0.1);
                break;
              case 30:
                PhasePower[0].apparentPower -= round2(data * 0.1);
                break;
              case 31:
                PhasePower[0].current = round2(data * 0.001);
                break;
              case 32:
                PhasePower[0].voltage = round2(data * 0.001);
                break;
              case 33:
                PhasePower[0].powerFactor = round2(data * 0.001);
                break;
              case 41:
                PhasePower[1].power = round2(data * 0.1);
                PhasePower[1].frequency = defaultFrequency;
                break;
              case 42:
                PhasePower[1].power -= round2(data * 0.1);
                break;
              case 49:
                PhasePower[1].apparentPower = round2(data * 0.1);
                break;
              case 50:
                PhasePower[1].apparentPower -= round2(data * 0.1);
                break;
              case 51:
                PhasePower[1].current = round2(data * 0.001);
                break;
              case 52:
                PhasePower[1].voltage = round2(data * 0.001);
                break;
              case 53:
                PhasePower[1].powerFactor = round2(data * 0.001);
                break;
              case 61:
                PhasePower[2].power = round2(data * 0.1);
                PhasePower[2].frequency = defaultFrequency;
                break;
              case 62:
                PhasePower[2].power -= round2(data * 0.1);
                break;
              case 69:
                PhasePower[2].apparentPower = round2(data * 0.1);
                break;
              case 70:
                PhasePower[2].apparentPower -= round2(data * 0.1);
                break;
              case 71:
                PhasePower[2].current = round2(data * 0.001);
                break;
              case 72:
                PhasePower[2].voltage = round2(data * 0.001);
                break;
              case 73:
                PhasePower[2].powerFactor = round2(data * 0.001);
                break;
              default:
                break;
            }
          } else if (channel == 144) {
            // optional handling of version number
            offset += 4;
          } else {
            offset += type;
            DEBUG_SERIAL.println("Unknown measurement");
          }
        }
      } else if (grouptag == 0) {
        // end marker
        offset += grouplen;
      } else {
        DEBUG_SERIAL.print("unhandled group ");
        DEBUG_SERIAL.print(grouptag);
        DEBUG_SERIAL.print(" with len=");
        DEBUG_SERIAL.println(grouplen);
        offset += grouplen;
      }
    } while (grouplen > 0 && offset + 4 < buffer + rSize);
  }
}

void parseSHRDZM() {
  JsonDocument json;
  uint8_t buffer[1024];
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int rSize = Udp.read(buffer, 1024);
    buffer[rSize] = 0;
    deserializeJson(json, buffer);
    if (json["data"]["16.7.0"].is<JsonVariant>()) {
      double power = json["data"]["16.7.0"];
      setPowerData(power);
    }
    if (json["data"]["1.8.0"].is<JsonVariant>() && json["data"]["2.8.0"].is<JsonVariant>()) {
      double energyIn = 0.001 * json["data"]["1.8.0"].as<double>();
      double energyOut = 0.001 * json["data"]["2.8.0"].as<double>();
      setEnergyData(energyIn, energyOut);
    }
  }
}

double SUNSPEC_scale(int n)
{
  double val=1.0;
  switch (n) {
    case -3: val=0.001; break;
    case -2: val=0.01; break;
    case -1: val=0.1; break;
    case 0: val=1.0; break;
    case 1: val=10.0; break;
    case 2: val=100.0; break;
    default:
    val=1.0;
  }
  return val;
}

void parseSUNSPEC() {
  #define SUNSPEC_BASE 40072
  #define SUNSPEC_VOLTAGE 40077
  #define SUNSPEC_VOLTAGE_SCALE 40084
  #define SUNSPEC_REAL_POWER 40088
  #define SUNSPEC_REAL_POWER_SCALE 40091
  #define SUNSPEC_APPARANT_POWER 40093
  #define SUNSPEC_APPARANT_POWER_SCALE 40096
  #define SUNSPEC_CURRENT 40072
  #define SUNSPEC_CURRENT_SCALE 40075
  #define SUNSPEC_POWER_FACTOR 40103
  #define SUNSPEC_POWER_FACTOR_SCALE 40106
  #define SUNSPEC_FREQUENCY 40085
  #define SUNSPEC_FREQUENCY_SCALE 40086
  
  modbus_ip.fromString(mqtt_server);
  if (!modbus1.isConnected(modbus_ip)) {
    modbus1.connect(modbus_ip, String(mqtt_port).toInt());
  } else {
    uint16_t transaction = modbus1.readHreg(modbus_ip, SUNSPEC_BASE, (uint16_t*) &modbus_result[0], 64, nullptr, String(modbus_dev).toInt());
    delay(10);
    modbus1.task();
    int t = 0;
    while (modbus1.isTransaction(transaction)) {
      modbus1.task();
      delay(10);
      t++;
      if (t > 50) {
        DEBUG_SERIAL.println("Timeout SUNSPEC");
        //prolong=10;
        modbus1.disconnect(modbus_ip);
        break;
      }
    }
    int32_t power = 0;
    if (t<=50) {
      double scale_V=SUNSPEC_scale(modbus_result[SUNSPEC_VOLTAGE_SCALE-SUNSPEC_BASE]);
      double scale_real_power=SUNSPEC_scale(modbus_result[SUNSPEC_REAL_POWER_SCALE-SUNSPEC_BASE]);
      double scale_apparant_power=SUNSPEC_scale(modbus_result[SUNSPEC_APPARANT_POWER_SCALE-SUNSPEC_BASE]);
      double scale_current=SUNSPEC_scale(modbus_result[SUNSPEC_CURRENT_SCALE-SUNSPEC_BASE]);
      double scale_powerfactor=SUNSPEC_scale(modbus_result[SUNSPEC_POWER_FACTOR_SCALE-SUNSPEC_BASE]);
      double scale_frequency=SUNSPEC_scale(modbus_result[SUNSPEC_FREQUENCY_SCALE-SUNSPEC_BASE]);

      for (int n=0;n<3;n++) {
        PhasePower[n].power=modbus_result[SUNSPEC_REAL_POWER-SUNSPEC_BASE+n]*scale_real_power;
        PhasePower[n].apparentPower=modbus_result[SUNSPEC_APPARANT_POWER-SUNSPEC_BASE+n]*scale_apparant_power;
        PhasePower[n].current= modbus_result[SUNSPEC_CURRENT-SUNSPEC_BASE+n]*scale_current;
        PhasePower[n].powerFactor=modbus_result[SUNSPEC_POWER_FACTOR-SUNSPEC_BASE+n]*scale_powerfactor;
        PhasePower[n].voltage=modbus_result[SUNSPEC_VOLTAGE-SUNSPEC_BASE+n]*scale_V;
        PhasePower[n].frequency=modbus_result[SUNSPEC_FREQUENCY-SUNSPEC_BASE]*scale_frequency;
        power+= PhasePower[n].power;
      }

      #define SUNSPEC_REAL_ENERGY_EXPORTED 40109
      #define SUNSPEC_REAL_IMPORTED_EXPORTED 40117
      #define SUNSPEC_REAL_ENERGY_SCALE 40123
      double scale_real_energy=SUNSPEC_scale(modbus_result[SUNSPEC_REAL_ENERGY_SCALE-SUNSPEC_BASE]);
        for (int n=0;n<3;n++) {
          uint32_t p=0;
          uint8_t *p_u8=(uint8_t *)&modbus_result[SUNSPEC_REAL_IMPORTED_EXPORTED-SUNSPEC_BASE+2*n];
          p|=((uint32_t)p_u8[2])<<0;
        p|=((uint32_t)p_u8[3])<<8;
        p|=((uint32_t)p_u8[0])<<16;
        p|=((uint32_t)p_u8[1])<<24;
          PhaseEnergy[n].consumption=p/1000.0*scale_real_energy;
          p=0;
          p_u8=(uint8_t *)&modbus_result[SUNSPEC_REAL_ENERGY_EXPORTED-SUNSPEC_BASE+2*n];
          p|=((uint32_t)p_u8[2])<<0;
        p|=((uint32_t)p_u8[3])<<8;
        p|=((uint32_t)p_u8[0])<<16;
        p|=((uint32_t)p_u8[1])<<24;
          PhaseEnergy[n].gridfeedin = -p/1000.0*scale_real_energy;
        }
    }
    DEBUG_SERIAL.printf("SUNSPEC power: %d,%d\n\r", t, power);
  }
}

void queryHTTP() {
  JsonDocument json;
  DEBUG_SERIAL.println("Querying HTTP source");
  http.begin(wifi_client, mqtt_server);
  http.GET();
  deserializeJson(json, http.getStream());
  if (strcmp(power_path, "") == 0) {
    DEBUG_SERIAL.println("HTTP query: no JSONPath for power data provided");
  } else {
    setJsonPathPower(json);
  }
  http.end();
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
