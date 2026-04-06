#include "RpcHandlers.h"
#include "../config/Configuration.h"
#include "../data/DataStructures.h"
#include "../data/DataProcessing.h"

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
  jsonResponse["n_current"] = nullptr;
  jsonResponse["total_current"] = serialized(String((PhasePower[0].power + PhasePower[1].power + PhasePower[2].power) / defaultVoltage, 2));
  jsonResponse["total_act_power"] = serialized(String(PhasePower[0].power + PhasePower[1].power + PhasePower[2].power, 2));
  jsonResponse["total_aprt_power"] = serialized(String(PhasePower[0].apparentPower + PhasePower[1].apparentPower + PhasePower[2].apparentPower, 2));
  jsonResponse["user_calibrated_phase"] = JsonArray();
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
  jsonResponse["bthome"].to<JsonObject>();
  jsonResponse["cloud"]["connected"] = false;
  EMGetStatus();
  jsonResponse["em:0"] = serialized(serJsonResponse);
  EMDataGetStatus();
  jsonResponse["emdata:0"] = serialized(serJsonResponse);
  JsonObject eth = jsonResponse["eth"].to<JsonObject>();
  eth["ip"] = nullptr;
  eth["ip6"] = nullptr;
  jsonResponse["modbus"].to<JsonObject>();
  jsonResponse["mqtt"]["connected"] = false;
  sysGetStatus();
  jsonResponse["sys"] = serialized(serJsonResponse);
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
  DEBUG_SERIAL.print("shellyGetStatus: ");
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
  DEBUG_SERIAL.print("wifiGetStatus: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}
