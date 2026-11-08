#include "RpcHandlers.h"
#include "../config/Configuration.h"
#include "../data/DataStructures.h"
#include "../data/DataProcessing.h"

static void setFixed2(JsonObject obj, const char* key, double value) {
  char formatted[32];
  int length = snprintf(formatted, sizeof(formatted), "%.2f", value);

  if (length > 0 && length < static_cast<int>(sizeof(formatted))) {
    obj[key] = serialized(static_cast<const char*>(formatted));
  }
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

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly#shellygetdeviceinfo-example
void shellyGetDeviceInfo() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = shelly_name.c_str();
  jsonResponse["mac"] = shelly_mac.as<const char*>();
  jsonResponse["slot"] = 1;
  jsonResponse["model"] = "SPEM-003CEBEU";
  jsonResponse["gen"] = shelly_gen.as<int>();
  jsonResponse["fw_id"] = shelly_fw_id.as<const char*>();
  jsonResponse["ver"] = "1.4.4";
  jsonResponse["app"] = "Pro3EM";
  jsonResponse["auth_en"] = false;
  jsonResponse["auth_domain"] = nullptr;
  jsonResponse["profile"] = "triphase";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print(F("shellyGetDeviceInfo: "));
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Sys#sysgetconfig-example
void sysGetConfig() {
  JsonDocument jsonResponse;
  jsonResponse["device"]["name"] = shelly_name.c_str();
  jsonResponse["device"]["mac"] = shelly_mac.as<const char*>();
  jsonResponse["device"]["fw_id"] = shelly_fw_id.as<const char*>();
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
  String portStr = String(config_doc["shelly"]["shelly_port"].as<int>());
  jsonResponse["rpc_udp"]["listen_port"] = portStr.c_str();
  jsonResponse["sntp"]["server"] = nullptr;
  jsonResponse["cfg_rev"] = 10;
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print(F("sysGetConfig: "));
  DEBUG_SERIAL.println(serJsonResponse);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Sys#sysgetstatus-example
static void fillSysGetStatus(JsonObject jsonResponse) {
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

  jsonResponse["mac"] = shelly_mac.as<const char*>();
  jsonResponse["restart_required"] = false;
  jsonResponse["time"] = time_buffer;
  jsonResponse["unixtime"] = now;
  jsonResponse["last_sync_ts"] = now;
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
  jsonResponse["available_updates"]["beta"]["version"] = "1.7.5-beta1";
}

void sysGetStatus() {
  JsonDocument jsonResponse;
  fillSysGetStatus(jsonResponse.to<JsonObject>());
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print(F("sysGetStatus: "));
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EM#emgetstatus-example
void fillEMStatus(JsonObject jsonResponse) {
  jsonResponse["id"] = 0;
  setFixed2(jsonResponse, "a_current", PhasePower[0].current);
  setFixed2(jsonResponse, "a_voltage", PhasePower[0].voltage);
  setFixed2(jsonResponse, "a_act_power", PhasePower[0].power);
  setFixed2(jsonResponse, "a_aprt_power", PhasePower[0].apparentPower);
  setFixed2(jsonResponse, "a_pf", PhasePower[0].powerFactor);
  setFixed2(jsonResponse, "a_freq", PhasePower[0].frequency);
  setFixed2(jsonResponse, "b_current", PhasePower[1].current);
  setFixed2(jsonResponse, "b_voltage", PhasePower[1].voltage);
  setFixed2(jsonResponse, "b_act_power", PhasePower[1].power);
  setFixed2(jsonResponse, "b_aprt_power", PhasePower[1].apparentPower);
  setFixed2(jsonResponse, "b_pf", PhasePower[1].powerFactor);
  setFixed2(jsonResponse, "b_freq", PhasePower[1].frequency);
  setFixed2(jsonResponse, "c_current", PhasePower[2].current);
  setFixed2(jsonResponse, "c_voltage", PhasePower[2].voltage);
  setFixed2(jsonResponse, "c_act_power", PhasePower[2].power);
  setFixed2(jsonResponse, "c_aprt_power", PhasePower[2].apparentPower);
  setFixed2(jsonResponse, "c_pf", PhasePower[2].powerFactor);
  setFixed2(jsonResponse, "c_freq", PhasePower[2].frequency);
  jsonResponse["n_current"] = nullptr;
  setFixed2(jsonResponse, "total_current", (PhasePower[0].power + PhasePower[1].power + PhasePower[2].power) / defaultVoltage);
  setFixed2(jsonResponse, "total_act_power", PhasePower[0].power + PhasePower[1].power + PhasePower[2].power);
  setFixed2(jsonResponse, "total_aprt_power", PhasePower[0].apparentPower + PhasePower[1].apparentPower + PhasePower[2].apparentPower);
  jsonResponse["user_calibrated_phase"] = JsonArray();
}

void EMGetStatus() {
  JsonDocument jsonResponse;
  fillEMStatus(jsonResponse.to<JsonObject>());
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print(F("EMGetStatus: "));
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EMData#emdatagetstatus-example
void fillEMDataStatus(JsonObject jsonResponse) {
  jsonResponse["id"] = 0;
  setFixed2(jsonResponse, "a_total_act_energy", PhaseEnergy[0].consumption);
  setFixed2(jsonResponse, "a_total_act_ret_energy", PhaseEnergy[0].gridfeedin);
  setFixed2(jsonResponse, "b_total_act_energy", PhaseEnergy[1].consumption);
  setFixed2(jsonResponse, "b_total_act_ret_energy", PhaseEnergy[1].gridfeedin);
  setFixed2(jsonResponse, "c_total_act_energy", PhaseEnergy[2].consumption);
  setFixed2(jsonResponse, "c_total_act_ret_energy", PhaseEnergy[2].gridfeedin);
  setFixed2(jsonResponse, "total_act", PhaseEnergy[0].consumption + PhaseEnergy[1].consumption + PhaseEnergy[2].consumption);
  setFixed2(jsonResponse, "total_act_ret", PhaseEnergy[0].gridfeedin + PhaseEnergy[1].gridfeedin + PhaseEnergy[2].gridfeedin);
}

void EMDataGetStatus() {
  JsonDocument jsonResponse;
  fillEMDataStatus(jsonResponse.to<JsonObject>());
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print(F("EMDataGetStatus: "));
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
  DEBUG_SERIAL.print(F("EMGetConfig: "));
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
  DEBUG_SERIAL.print(F("shellyGetConfig: "));
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
  DEBUG_SERIAL.print(F("shellyGetComponents: "));
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
  jsonResponse["bthome"].to<JsonObject>();
  jsonResponse["cloud"]["connected"] = false;
  fillEMStatus(jsonResponse["em:0"].to<JsonObject>());
  fillEMDataStatus(jsonResponse["emdata:0"].to<JsonObject>());
  JsonObject eth = jsonResponse["eth"].to<JsonObject>();
  eth["ip"] = nullptr;
  eth["ip6"] = nullptr;
  jsonResponse["modbus"].to<JsonObject>();
  jsonResponse["mqtt"]["connected"] = false;
  fillSysGetStatus(jsonResponse["sys"].to<JsonObject>());
  JsonObject temp = jsonResponse["temperature:0"].to<JsonObject>();
  temp["id"] = 0;
  temp["tC"] = serialized(String(temperature, 2));
  temp["tF"] = serialized(String((temperature * 9.0 / 5.0) + 32.0, 2));
  jsonResponse["wifi"]["sta_ip"] = WiFi.localIP().toString();
  jsonResponse["wifi"]["status"] = (WiFi.status() == WL_CONNECTED) ? "got ip" : "connecting";
  jsonResponse["wifi"]["ssid"] = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : "null";
  jsonResponse["wifi"]["bssid"] = (WiFi.status() == WL_CONNECTED) ? WiFi.BSSIDstr() : "null";
  jsonResponse["wifi"]["rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  jsonResponse["wifi"]["sta_ip6"].to<JsonArray>();
  jsonResponse["ws"]["connected"] = false;
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print(F("shellyGetStatus: "));
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly#shellylistmethods-example
void shellyListMethods() {
  JsonDocument jsonResponse;
  JsonArray methods = jsonResponse["methods"].to<JsonArray>();
  methods.add("EM.GetStatus");
  methods.add("EM.GetConfig");
  methods.add("EMData.GetStatus");
  methods.add("Shelly.GetComponents");
  methods.add("Shelly.GetConfig");
  methods.add("Shelly.GetDeviceInfo");
  methods.add("Shelly.GetStatus");
  methods.add("Sys.GetConfig");
  methods.add("Sys.GetStatus");
  methods.add("WiFi.GetStatus");
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print(F("shellyListMethods: "));
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// aligned with Shelly API docs
// https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Wifi#wifigetstatus-example
void wifiGetStatus() {
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  JsonDocument jsonResponse;
  jsonResponse["sta_ip"] = wifiConnected ? WiFi.localIP().toString() : "null";
  jsonResponse["status"] = wifiConnected ? "got ip" : "connecting";
  jsonResponse["ssid"] = wifiConnected ? WiFi.SSID() : "null";
  jsonResponse["bssid"] = wifiConnected ? WiFi.BSSIDstr() : "null";
  jsonResponse["rssi"] = wifiConnected ? WiFi.RSSI() : 0;
  jsonResponse["ap_client_count"] = 0; // not really relevant, as we are not in AP mode, but included for completeness
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print(F("wifiGetStatus: "));
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}
