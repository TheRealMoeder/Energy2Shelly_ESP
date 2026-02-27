// Energy2Shelly_ESP v0.5.3
#include <Arduino.h>
#include <Preferences.h>
#ifndef ESP32
  #define WEBSERVER_H "fix WifiManager conflict"
#endif
#ifdef ESP32
  #include <HTTPClient.h>
  #include <AsyncTCP.h>
  #include <ESPmDNS.h>
  #include <WiFi.h>
#else
  #include <ESP8266HTTPClient.h>
  #include <ESPAsyncTCP.h>
  #include <ESP8266mDNS.h>
#endif
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>
#include <ModbusIP_ESP8266.h>

#define DEBUG true // set to false for no DEBUG output
#define DEBUG_SERIAL if(DEBUG)Serial

unsigned long startMillis = 0;
unsigned long currentMillis;

// for getting time
time_t now;
tm timeinfo;

// define your default values here, if there are different values in config.json, they are overwritten.
char input_type[40];
char reset_password[33] = "admin"; // default reset password
char ntp_server[40] = "de.pool.ntp.org";
char timezone[64] = "CET-1CEST,M3.5.0/2,M10.5.0/3"; // Central European Time
char phase_number[2] = "3"; // number of phases: 1 or 3
char query_period[10] = "1000";
// MQTT related
char mqtt_server[160];
char mqtt_port[6] = "1883";
char mqtt_topic[90] = "tele/meter/SENSOR";
char mqtt_user[40] = "";
char mqtt_passwd[40] = "";
// HTTP related
char http_url[160];
// JSON PATHs for power and energy values in the source data
char power_path[150] = "";
char pwr_export_path[150] = "";
char power_l1_path[150] = "";
char power_l2_path[150] = "";
char power_l3_path[150] = "";
char energy_in_path[150] = "";
char energy_out_path[150] = "";
// Shelly API related
char shelly_gen[2] = "2";
char shelly_fw_id[32] = "20250924-062729/1.7.1-gd336f31";
char shelly_mac[13];
char shelly_name[26] = "ShellyPro3EM-";
char shelly_udp_port[6] = "2220"; // old: 1010; new (FW>=226): 2220
// Modbus related
char modbus_server_ip[16];
char modbus_port[6] = "502";
char modbus_dev[10] = "71"; // default for KSEM
// SMA related
char sma_id[17] = "";

IPAddress modbus_ip;
ModbusIP modbus1;
int16_t modbus_result[256];

const uint8_t defaultVoltage = 230;
const uint8_t defaultFrequency = 50;
const uint8_t defaultPowerFactor = 1;

// LED blink default values
unsigned long ledOffTime = 0;
uint8_t led = 0;
bool led_i = false;
const uint8_t ledblinkduration = 50;
char led_gpio[3] = "";
char led_gpio_i[6];

unsigned long period = 1000;
int rpcId = 1;
char rpcUser[20] = "user_1";

// SMA Multicast IP and Port
unsigned int multicastPort = 9522;  // local port to listen on
IPAddress multicastIP(239, 12, 255, 254);

// flags for saving/resetting WifiManager data
bool shouldSaveConfig = false;
bool shouldResetConfig = false;

Preferences preferences;

// flags for data sources
bool dataMQTT = false;
bool dataSMA = false;
bool dataSHRDZM = false;
bool dataHTTP = false;
bool dataSUNSPEC = false;

struct PowerData {
  double current;
  double voltage;
  double power;
  double apparentPower;
  double powerFactor;
  double frequency;
};

struct EnergyData {
  double gridfeedin;
  double consumption;
};

PowerData PhasePower[3];
EnergyData PhaseEnergy[3];
String serJsonResponse;

#ifndef ESP32
  MDNSResponder::hMDNSService hMDNSService = 0; // handle of the http service in the MDNS responder
  MDNSResponder::hMDNSService hMDNSService2 = 0; // handle of the shelly service in the MDNS responder
#endif

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
static AsyncWebServer server(80);
static AsyncWebSocket webSocket("/rpc");
WiFiUDP Udp;
HTTPClient http;
WiFiUDP UdpRPC;
#ifdef ESP32
#define UDPPRINT print
#else
#define UDPPRINT write
#endif

// use for values in JsonDocument to force 2 decimals in double/float
double round2(double value) {
  return (int)(value * 100 + (value > 0.0 ? 0.5 : -0.5)) / 100.0;
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
  switch (phase_number[0]) {
    case '1': // monophase
      for (int i = 0; i <= 2; i++) {
        PhasePower[i].power = (i == 0) ? round2(totalPower) : 0.0;
        PhasePower[i].voltage = defaultVoltage;
        PhasePower[i].current = (i == 0) ? round2(PhasePower[i].power / PhasePower[i].voltage) : 0.0;
        PhasePower[i].apparentPower = (i == 0) ? round2(PhasePower[i].power) : 0.0;
        PhasePower[i].powerFactor = defaultPowerFactor;
        PhasePower[i].frequency = defaultFrequency;
      }
      break;
    case '3': // triphase
  for (int i = 0; i <= 2; i++) {
    PhasePower[i].power = round2(totalPower * 0.3333);
    PhasePower[i].voltage = defaultVoltage;
    PhasePower[i].current = round2(PhasePower[i].power / PhasePower[i].voltage);
    PhasePower[i].apparentPower = round2(PhasePower[i].power);
    PhasePower[i].powerFactor = defaultPowerFactor;
    PhasePower[i].frequency = defaultFrequency;
      }
      break;
    default:
      break;
  }
  DEBUG_SERIAL.print("Current total power: ");
  DEBUG_SERIAL.println(totalPower);
}

void setPowerData(double phase1Power, double phase2Power, double phase3Power) {
  PhasePower[0].power = round2(phase1Power);
  PhasePower[1].power = round2(phase2Power);
  PhasePower[2].power = round2(phase3Power);
  for (int i = 0; i <= 2; i++) {
    PhasePower[i].voltage = defaultVoltage;
    PhasePower[i].current = round2(PhasePower[i].power / PhasePower[i].voltage);
    PhasePower[i].apparentPower = round2(PhasePower[i].power);
    PhasePower[i].powerFactor = defaultPowerFactor;
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
  switch (phase_number[0]) {
    case '1': // monophase
      for (int i = 0; i <= 2; i++) {
        PhaseEnergy[i].consumption = (i == 0) ? round2(totalEnergyGridSupply) : 0.0;
        PhaseEnergy[i].gridfeedin = (i == 0) ? round2(totalEnergyGridFeedIn) : 0.0;
      }
      break;
    case '3': // triphase
  for (int i = 0; i <= 2; i++) {
    PhaseEnergy[i].consumption = round2(totalEnergyGridSupply * 0.3333);
    PhaseEnergy[i].gridfeedin = round2(totalEnergyGridFeedIn * 0.3333);
      }
      break;
    default:
      break;
  }
  DEBUG_SERIAL.print("Total Consumption (Grid Feed-From)): ");
  DEBUG_SERIAL.print(totalEnergyGridSupply);
  DEBUG_SERIAL.print(" - Total Production (Grid Feed-In)): ");
  DEBUG_SERIAL.println(totalEnergyGridFeedIn);
}

//callback notifying us of the need to save WifiManager config
void saveConfigCallback() {
  DEBUG_SERIAL.println("Should save config");
  shouldSaveConfig = true;
}

void setJsonPathPower(JsonDocument json) {
  if (strcmp(power_path, "TRIPHASE") == 0) {
    DEBUG_SERIAL.println("resolving triphase");
    double power1 = resolveJsonPath(json, power_l1_path);
    double power2 = resolveJsonPath(json, power_l2_path);
    double power3 = resolveJsonPath(json, power_l3_path);
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
  jsonResponse["rpc_udp"]["listen_port"] = shelly_udp_port;
  jsonResponse["sntp"]["server"] = ntp_server;
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

  // these were in the uni-meter output, but not in the Shelly examples
  // will keep them commented out for possible future use
  //
  // jsonResponse["sys"]["mac"] = mac.c_str();
  // jsonResponse["sys"]["restart_required"] = false;
  // jsonResponse["sys"]["time"] = time_buffer;
  // jsonResponse["sys"]["unixtime"] = now;
  // jsonResponse["sys"]["last_sync_ts"] = nullptr;
  // jsonResponse["sys"]["uptime"] = millis() / 1000;
  // jsonResponse["sys"]["ram_size"] = ram_total;
  // jsonResponse["sys"]["ram_free"] = ESP.getFreeHeap();
  // jsonResponse["sys"]["fs_size"] = ESP.getFlashChipSize();
  // jsonResponse["sys"]["fs_free"] = ESP.getFreeSketchSpace();
  // jsonResponse["sys"]["cfg_rev"] = 10;
  // jsonResponse["sys"]["kvs_rev"] = 2725;
  //
  // jsonResponse["serial"] = 1;
  // jsonResponse["has_update"] = false;
  //
  // jsonResponse["temperature"] = serialized(String(temperature, 2));
  // jsonResponse["overtemperature"] = false;
  // temp["is_valid"] = true;
  // JsonObject modbus = jsonResponse["modbus"].to<JsonObject>();
  // modbus["enabled"] = false;
  // jsonResponse["total_power"] = serialized(String(PhasePower[0].power + PhasePower[1].power + PhasePower[2].power, 2));
  // jsonResponse["fs_mounted"] = true;
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

void WifiManagerSetup() {
  // Set Shelly ID to ESP's MAC address by default
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(shelly_mac, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  preferences.begin("e2s_config", false);
  strcpy(reset_password, preferences.getString("reset_password", reset_password).c_str());
  strcpy(ntp_server, preferences.getString("ntp_server", ntp_server).c_str());
  strcpy(timezone, preferences.getString("timezone", timezone).c_str());
  strcpy(query_period, preferences.getString("query_period", query_period).c_str());
  strcpy(led_gpio, preferences.getString("led_gpio", led_gpio).c_str());
  strcpy(led_gpio_i, preferences.getString("led_gpio_i", led_gpio_i).c_str());
  strcpy(shelly_mac, preferences.getString("shelly_mac", shelly_mac).c_str());
  strcpy(shelly_udp_port, preferences.getString("shelly_udp_port", shelly_udp_port).c_str());
  strcpy(phase_number, preferences.getString("phase_number", phase_number).c_str());

  strcpy(input_type, preferences.getString("input_type", input_type).c_str());
  // MQTT settings
  strcpy(mqtt_server, preferences.getString("mqtt_server", mqtt_server).c_str());
  strcpy(mqtt_port, preferences.getString("mqtt_port", mqtt_port).c_str());
  strcpy(mqtt_topic, preferences.getString("mqtt_topic", mqtt_topic).c_str());
  strcpy(mqtt_user, preferences.getString("mqtt_user", mqtt_user).c_str());
  strcpy(mqtt_passwd, preferences.getString("mqtt_passwd", mqtt_passwd).c_str());
  // SMA settings
  strcpy(sma_id, preferences.getString("sma_id", sma_id).c_str());
  // SUNSPEC settings
  strcpy(modbus_server_ip, preferences.getString("modbus_server", modbus_server_ip).c_str());
  strcpy(modbus_port, preferences.getString("modbus_port", modbus_port).c_str());
  strcpy(modbus_dev, preferences.getString("modbus_dev", modbus_dev).c_str());
  // HTTP settings
  strcpy(http_url, preferences.getString("http_url", http_url).c_str());
  strcpy(power_path, preferences.getString("power_path", power_path).c_str());
  strcpy(pwr_export_path, preferences.getString("pwr_export_path", pwr_export_path).c_str());
  strcpy(power_l1_path, preferences.getString("power_l1_path", power_l1_path).c_str());
  strcpy(power_l2_path, preferences.getString("power_l2_path", power_l2_path).c_str());
  strcpy(power_l3_path, preferences.getString("power_l3_path", power_l3_path).c_str());
  strcpy(energy_in_path, preferences.getString("energy_in_path", energy_in_path).c_str());
  strcpy(energy_out_path, preferences.getString("energy_out_path", energy_out_path).c_str());

  const char *dd_select_str = R"(
  <br/>
  <hr>
  <br/>
  <label for='datasource'>Datasource</label>
  <select name="datasource" id="datasource">
    <option value=""></option>
    <option value="MQTT">MQTT topic</option>
    <option value="SMA">SMA EM/HM UDP multicast</option>
    <option value="SHRDZM">SHRDZM UDP</option>
    <option value="HTTP">generic HTTP input</option>
    <option value="SUNSPEC">SUNSPEC via Modbus TCP</option>
  </select>
  <script>
  window.addEventListener('DOMContentLoaded', () => {
    document.getElementById('input_type').hidden = true;
    const myvalue = "%s";
    if (myvalue.length > 0 && document.getElementById(myvalue)) {
      console.log("Setting datasource to " + myvalue);
      document.getElementById('datasource').value = myvalue;
      document.getElementById(myvalue).style.display = "block";
    }
    document.querySelector("[for='input_type']").hidden = true;
    document.getElementById('datasource').addEventListener('change', function() {
      document.getElementById('input_type').value = this.value;
      document.getElementById('JSONPATH').style.display = (this.value === "MQTT" || this.value === "HTTP") ? "block" : "none";
      for (const option of this.options) {
        var element = document.getElementById(option.value);
        if (element) {
          element.style.display = (this.value === option.value) ? "block" : "none";
        }
      }
    });
  });
  </script>
  )";
  char buffer_datasource[1500];
  sprintf(buffer_datasource, dd_select_str, input_type);

  const char *show_pwd_str = "<input type=\"checkbox\" onclick=\"t('%s')\">&nbsp;<label>Show password</label><br/>";

  WiFiManagerParameter param_section_general("<h3>General settings</h3><script>function t(s) { var x = document.getElementById(s); x.type === \"password\" ? x.type = \"text\" : x.type = \"password\"; }</script>");
  WiFiManagerParameter param_reset_password("reset_password", "Reset password <span title=\"Required to trigger config mode(Wifi AP mode)\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", reset_password, 20, "type='password'");
  char buf_rst_pwd_show_pwd[150];
  sprintf(buf_rst_pwd_show_pwd, show_pwd_str, "reset_password");
  WiFiManagerParameter param_reset_password_show_password(buf_rst_pwd_show_pwd);
  WiFiManagerParameter param_ntp_server("ntp_server", "NTP server <span title=\"for time synchronization\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", ntp_server, 40);
  WiFiManagerParameter param_timezone("timezone", "Timezone <span title=\"e.g. UTC0, UTC+1, UTC-3, UTC+1CET-1CEST,M3.5.0/02:00:00,M10.5.0/03:00:00\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", timezone, 64);
  WiFiManagerParameter param_query_period("query_period", "Query period <span title=\"for generic HTTP and SUNSPEC, in milliseconds\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", query_period, 10);
  WiFiManagerParameter param_led_gpio("led_gpio", "GPIO of internal LED <span title=\"GPIO of internal LED\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", led_gpio, 3);
  WiFiManagerParameter param_led_gpio_i("led_gpio_i", "GPIO is inverted <span title=\"true or false\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", led_gpio_i, 6);
  WiFiManagerParameter param_shelly_mac("shelly_mac", "Shelly ID (12 char hexadecimal) <span title=\"defaults to MAC address of ESP\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", shelly_mac, 13);
  WiFiManagerParameter param_shelly_udp_port("shelly_udp_port", "Shelly UDP port <span title=\"1010 for old Marstek FW, 2220 for new Marstek FW v226+/v108+\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", shelly_udp_port, 6);
  WiFiManagerParameter param_phase_number("phase_number", "Number of phases <span title=\"Number of phases (e.g. 1 or 3)\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", phase_number, 1);

  WiFiManagerParameter param_datasource("input_type", "Will be hidden", input_type, 40);
  WiFiManagerParameter param_dd_datasource(buffer_datasource);

  // MQTT section
  WiFiManagerParameter param_section_mqtt("<div id=\"MQTT\" style=\"display:none\"><h4>MQTT Topic options</h4>");
  WiFiManagerParameter param_mqtt_server("mqtt_server", "Host (IP / FQDN)", mqtt_server, 160);
  WiFiManagerParameter param_mqtt_port("mqtt_port", "Port", mqtt_port, 6);
  WiFiManagerParameter param_mqtt_topic("mqtt_topic", "Topic", mqtt_topic, 90);
  WiFiManagerParameter param_mqtt_user("mqtt_user", "User (optional)", mqtt_user, 40);
  WiFiManagerParameter param_mqtt_passwd("mqtt_passwd", "Password (optional)", mqtt_passwd, 40, "type='password'");
  char buf_mqtt_pwd_show_pwd[150];
  sprintf(buf_mqtt_pwd_show_pwd, show_pwd_str, "mqtt_passwd");
  WiFiManagerParameter param_mqtt_passwd_show_password(buf_mqtt_pwd_show_pwd);
  // SMA section
  WiFiManagerParameter param_section_sma("<div id=\"SMA\" style=\"display:none\"><h4>SMA options</h4>");
  WiFiManagerParameter param_sma_id("sma_id", "SMA serial number <span title=\"optional serial number (if you have more than one SMA EM/HM in your network)\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", sma_id, 16);
  // HTTP section
  WiFiManagerParameter param_section_http("<div id=\"HTTP\" style=\"display:none\"><h4>generic HTTP options</h4>");
  WiFiManagerParameter param_http_url("http_url", "HTTP URL <span title=\"e.g. /status\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", http_url, 160);
  // SUNSPEC section
  WiFiManagerParameter param_section_modbus("<div id=\"SUNSPEC\" style=\"display:none\"><h4>SUNSPEC Modbus options</h4>");
  WiFiManagerParameter param_modbus_server("modbus_server", "Host IP", modbus_server_ip, 16);
  WiFiManagerParameter param_modbus_port("modbus_port", "Port", modbus_port, 6);
  WiFiManagerParameter param_modbus_dev("modbus_dev", "Modbus device ID <span title=\"71 for Kostal SEM\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", modbus_dev, 60);
  // JSON paths for MQTT and HTTP
  WiFiManagerParameter param_section_jsonpath("<div id=\"JSONPATH\" style=\"display:none\"><h5>JSON paths for MQTT and HTTP input</h5>");
  WiFiManagerParameter param_power_path("power_path", "Total power <span title=\"e.g. ENERGY.Power or TRIPHASE for tri-phase data\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", power_path, 150);
  WiFiManagerParameter param_pwr_export_path("pwr_export_path", "Export power <span title=\"Optional, for net calc (e.g. 'i-e')\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", pwr_export_path, 150);
  WiFiManagerParameter param_power_l1_path("power_l1_path", "Phase 1 power <span title=\"optional\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", power_l1_path, 150);
  WiFiManagerParameter param_power_l2_path("power_l2_path", "Phase 2 power <span title=\"optional\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", power_l2_path, 150);
  WiFiManagerParameter param_power_l3_path("power_l3_path", "Phase 3 power <span title=\"optional\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", power_l3_path, 150);
  WiFiManagerParameter param_energy_in_path("energy_in_path", "Energy consumed from grid <span title=\"e.g. ENERGY.Grid\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", energy_in_path, 150);
  WiFiManagerParameter param_energy_out_path("energy_out_path", "Energy feeded into grid <span title=\"e.g. ENERGY.FeedIn\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", energy_out_path, 150);

  WiFiManagerParameter param_sectionx_end("</div>");

  WiFiManager wifiManager;
  if (!DEBUG) {
    wifiManager.setDebugOutput(false);
  }
  wifiManager.setTitle("Energy2Shelly for ESP");
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&param_section_general);
  wifiManager.addParameter(&param_reset_password);
  wifiManager.addParameter(&param_reset_password_show_password);
  wifiManager.addParameter(&param_ntp_server);
  wifiManager.addParameter(&param_timezone);
  wifiManager.addParameter(&param_query_period);
  wifiManager.addParameter(&param_led_gpio);
  wifiManager.addParameter(&param_led_gpio_i);
  wifiManager.addParameter(&param_shelly_mac);
  wifiManager.addParameter(&param_shelly_udp_port);
  wifiManager.addParameter(&param_phase_number);
  wifiManager.addParameter(&param_datasource);
  wifiManager.addParameter(&param_dd_datasource);
  // MQTT section
  wifiManager.addParameter(&param_section_mqtt);
  wifiManager.addParameter(&param_mqtt_server);
  wifiManager.addParameter(&param_mqtt_port);
  wifiManager.addParameter(&param_mqtt_topic);
  wifiManager.addParameter(&param_mqtt_user);
  wifiManager.addParameter(&param_mqtt_passwd);
  wifiManager.addParameter(&param_mqtt_passwd_show_password);
  wifiManager.addParameter(&param_sectionx_end);
  // SMA section
  wifiManager.addParameter(&param_section_sma);
  wifiManager.addParameter(&param_sma_id);
  wifiManager.addParameter(&param_sectionx_end);
  // HTTP section
  wifiManager.addParameter(&param_section_http);
  wifiManager.addParameter(&param_http_url);
  wifiManager.addParameter(&param_sectionx_end);
  // SUNSPEC section
  wifiManager.addParameter(&param_section_modbus);
  wifiManager.addParameter(&param_modbus_server);
  wifiManager.addParameter(&param_modbus_port);
  wifiManager.addParameter(&param_modbus_dev);
  wifiManager.addParameter(&param_sectionx_end);
  // JSON path section for MQTT and HTTP
  wifiManager.addParameter(&param_section_jsonpath);
  wifiManager.addParameter(&param_power_path);
  wifiManager.addParameter(&param_pwr_export_path);
  wifiManager.addParameter(&param_power_l1_path);
  wifiManager.addParameter(&param_power_l2_path);
  wifiManager.addParameter(&param_power_l3_path);
  wifiManager.addParameter(&param_energy_in_path);
  wifiManager.addParameter(&param_energy_out_path);
  wifiManager.addParameter(&param_sectionx_end);

  if (!wifiManager.autoConnect("Energy2Shelly")) {
    DEBUG_SERIAL.println("failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
    delay(5000);
  }
  DEBUG_SERIAL.println("connected");

  //read updated parameters
  // general options
  strcpy(reset_password, param_reset_password.getValue());
  strcpy(ntp_server, param_ntp_server.getValue());
  strcpy(timezone, param_timezone.getValue());
  strcpy(query_period, param_query_period.getValue());
  strcpy(led_gpio, param_led_gpio.getValue());
  strcpy(led_gpio_i, param_led_gpio_i.getValue());
  strcpy(shelly_mac, param_shelly_mac.getValue());
  strcpy(shelly_udp_port, param_shelly_udp_port.getValue());
  strcpy(phase_number, param_phase_number.getValue());
  strcpy(input_type, param_datasource.getValue());
  // MQTT
  strcpy(mqtt_server, param_mqtt_server.getValue());
  strcpy(mqtt_port, param_mqtt_port.getValue());
  strcpy(mqtt_topic, param_mqtt_topic.getValue());
  strcpy(mqtt_user, param_mqtt_user.getValue());
  strcpy(mqtt_passwd, param_mqtt_passwd.getValue());
  // SMA
  strcpy(sma_id, param_sma_id.getValue());
  // HTTP
  strcpy(http_url, param_http_url.getValue());
  // SUNSPEC
  strcpy(modbus_server_ip, param_modbus_server.getValue());
  strcpy(modbus_port, param_modbus_port.getValue());
  strcpy(modbus_dev, param_modbus_dev.getValue());
  // JSON paths for MQTT and HTTP
  strcpy(power_path, param_power_path.getValue());
  strcpy(pwr_export_path, param_pwr_export_path.getValue());
  strcpy(power_l1_path, param_power_l1_path.getValue());
  strcpy(power_l2_path, param_power_l2_path.getValue());
  strcpy(power_l3_path, param_power_l3_path.getValue());
  strcpy(energy_in_path, param_energy_in_path.getValue());
  strcpy(energy_out_path, param_energy_out_path.getValue());

  DEBUG_SERIAL.println("The values in the preferences are: ");
  DEBUG_SERIAL.println("  reset_password: ********");
  DEBUG_SERIAL.println("  ntp_server: " + String(ntp_server));
  DEBUG_SERIAL.println("  timezone: " + String(timezone));
  DEBUG_SERIAL.println("  query_period: " + String(query_period));
  DEBUG_SERIAL.println("  led_gpio: " + String(led_gpio));
  DEBUG_SERIAL.println("  led_gpio_i: " + String(led_gpio_i));
  DEBUG_SERIAL.println("  shelly_mac: " + String(shelly_mac));
  DEBUG_SERIAL.println("  shelly_udp_port: " + String(shelly_udp_port));
  DEBUG_SERIAL.println("  phase_number: " + String(phase_number));
  DEBUG_SERIAL.println("  input_type: " + String(input_type));
  DEBUG_SERIAL.println("  MQTT options:");
  DEBUG_SERIAL.println("    - mqtt_server: " + String(mqtt_server));
  DEBUG_SERIAL.println("    - mqtt_port: " + String(mqtt_port));
  DEBUG_SERIAL.println("    - mqtt_topic: " + String(mqtt_topic));
  DEBUG_SERIAL.println("    - mqtt_user: " + String(mqtt_user));
  DEBUG_SERIAL.println("    - mqtt_passwd: ********");
  DEBUG_SERIAL.println("  SMA options:");
  DEBUG_SERIAL.println("    - sma_id: " + String(sma_id));
  DEBUG_SERIAL.println("  HTTP options:");
  DEBUG_SERIAL.println("    - http_url: " + String(http_url));
  DEBUG_SERIAL.println("  SUNSPEC options:");
  DEBUG_SERIAL.println("    - modbus_server" + String(modbus_server_ip));
  DEBUG_SERIAL.println("    - modbus_port: " + String(modbus_port));
  DEBUG_SERIAL.println("    - modbus_dev: " + String(modbus_dev));
  DEBUG_SERIAL.println("  JSON paths for MQTT and HTTP:");
  DEBUG_SERIAL.println("    - power_path: " + String(power_path));
  DEBUG_SERIAL.println("    - pwr_export_path: " + String(pwr_export_path));
  DEBUG_SERIAL.println("    - power_l1_path: " + String(power_l1_path));
  DEBUG_SERIAL.println("    - power_l2_path: " + String(power_l2_path));
  DEBUG_SERIAL.println("    - power_l3_path: " + String(power_l3_path));
  DEBUG_SERIAL.println("    - energy_in_path: " + String(energy_in_path));
  DEBUG_SERIAL.println("    - energy_out_path: " + String(energy_out_path));
  DEBUG_SERIAL.println("------------------------------");

  if (strcmp(input_type, "MQTT") == 0) {
    dataMQTT = true;
    DEBUG_SERIAL.println("Enabling MQTT data input");
  } else if (strcmp(input_type, "SMA") == 0) {
    dataSMA = true;
    DEBUG_SERIAL.println("Enabling SMA Multicast data input");
  } else if (strcmp(input_type, "SHRDZM") == 0) {
    dataSHRDZM = true;
    DEBUG_SERIAL.println("Enabling SHRDZM UDP data input");
  } else if (strcmp(input_type, "HTTP") == 0) {
    dataHTTP = true;
    DEBUG_SERIAL.println("Enabling generic HTTP data input");
  } else if (strcmp(input_type, "SUNSPEC") == 0) {
    dataSUNSPEC = true;
    DEBUG_SERIAL.println("Enabling SUNSPEC data input");
  }

  if (strcmp(led_gpio_i, "true") == 0) {
    led_i = true;
  } else {
    led_i = false;
  }

  if (shouldSaveConfig) {
    DEBUG_SERIAL.println("saving config");
    preferences.putString("reset_password", reset_password);
    preferences.putString("ntp_server", ntp_server);
    preferences.putString("timezone", timezone);
    preferences.putString("query_period", query_period);
    preferences.putString("led_gpio", led_gpio);
    preferences.putString("led_gpio_i", led_gpio_i);
    preferences.putString("shelly_mac", shelly_mac);
    preferences.putString("shelly_udp_port", shelly_udp_port);
    preferences.putString("phase_number", phase_number);
    preferences.putString("input_type", input_type);
    preferences.putString("sma_id", sma_id);
    preferences.putString("mqtt_server", mqtt_server);
    preferences.putString("mqtt_port", mqtt_port);
    preferences.putString("mqtt_topic", mqtt_topic);
    preferences.putString("mqtt_user", mqtt_user);
    preferences.putString("mqtt_passwd", mqtt_passwd);
    preferences.putString("modbus_server", modbus_server_ip);
    preferences.putString("modbus_port", modbus_port);
    preferences.putString("modbus_dev", modbus_dev);
    preferences.putString("http_url", http_url);
    preferences.putString("power_path", power_path);
    preferences.putString("pwr_export_path", pwr_export_path);
    preferences.putString("power_l1_path", power_l1_path);
    preferences.putString("power_l2_path", power_l2_path);
    preferences.putString("power_l3_path", power_l3_path);
    preferences.putString("energy_in_path", energy_in_path);
    preferences.putString("energy_out_path", energy_out_path);
    wifiManager.reboot();
  }
  DEBUG_SERIAL.print("local ip: ");
  DEBUG_SERIAL.println(WiFi.localIP());
}

void setup(void) {
  DEBUG_SERIAL.begin(115200);
  WifiManagerSetup();

  // Initialize time via NTP
#ifdef ESP32
  configTime(0, 0, ntp_server);
  setenv("TZ", timezone, 1);
  tzset();
#else
  //ESP8266
  configTime(timezone, ntp_server);
#endif
  while (!getLocalTime(&timeinfo)) {
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
    request->send(200, "text/html", "<html><body><form method='post' accept-charset='UTF-8'><pre>Enter \"Reset password\" to put device in configuration mode:<br/><br/><input type='password' name='password'> <input type='submit' value='Reset device'></pre></form></body></html>");
  });
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    String password = "";
    if (request->hasParam("password", true)) {
      AsyncWebServerResponse *response;
      const AsyncWebParameter *p = request->getParam("password", true);
      password = p->value();
      String storedPassword = preferences.getString("reset_password");
      if (password == storedPassword) {
        shouldResetConfig = true;
        response = request->beginResponse(200, "text/plain", "Resetting WiFi configuration, please log back into the hotspot to reconfigure...\r\n");
      } else {
        response = request->beginResponse(401, "text/plain", "Unauthorized: Invalid reset password.\r\n");
      }
      request->send(response);
    }
  });

  // Shelly RPC endpoints

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

  webSocket.onEvent(webSocketEvent);
  server.addHandler(&webSocket);
  server.begin();

  // Set up RPC over UDP for Marstek users
  UdpRPC.begin(String(shelly_udp_port).toInt()); 

  // Set up MQTT
  if (dataMQTT) {
    mqtt_client.setBufferSize(2048);
    mqtt_client.setServer(mqtt_server, String(mqtt_port).toInt());
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
    period = atol(query_period);
    modbus1.client();
    modbus_ip.fromString(modbus_server_ip);
    if (!modbus1.isConnected(modbus_ip)) {
      Serial.println("Trying to connect SUNSPEC powermeter data");
      modbus1.connect(modbus_ip, String(modbus_port).toInt());
    }
  }

  // Set Up HTTP query
  if (dataHTTP) {
    period = atol(query_period);
    http.useHTTP10(true);
  }

  // Set up mDNS responder
  strcat(shelly_name, shelly_mac);
  if (!MDNS.begin(shelly_name)) {
    DEBUG_SERIAL.println("Error setting up MDNS responder!");
  }

#ifdef ESP32
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("shelly", "tcp", 80);
  mdns_txt_item_t serviceTxtData[4] = {
      {"id", shelly_name},
      {"fw_id", shelly_fw_id},
      {"gen", shelly_gen},
      {"arch", "esp8266"}
  };
  mdns_service_instance_name_set("_http", "_tcp", shelly_name);
  mdns_service_txt_set("_http", "_tcp", serviceTxtData, 4);
  mdns_service_instance_name_set("_shelly", "_tcp", shelly_name);
  mdns_service_txt_set("_shelly", "_tcp", serviceTxtData, 4);
#else
  hMDNSService = MDNS.addService(0, "http", "tcp", 80);
  hMDNSService2 = MDNS.addService(0, "shelly", "tcp", 80);
  if (hMDNSService) {
    MDNS.setServiceName(hMDNSService, shelly_name);
    MDNS.addServiceTxt(hMDNSService, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService, "gen", shelly_gen);
    MDNS.addServiceTxt(hMDNSService, "fw_id", shelly_fw_id);
    MDNS.addServiceTxt(hMDNSService, "id", shelly_name);
  }
  if (hMDNSService2) {
    MDNS.setServiceName(hMDNSService2, shelly_name);
    MDNS.addServiceTxt(hMDNSService2, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService2, "gen", shelly_gen);
    MDNS.addServiceTxt(hMDNSService2, "fw_id", shelly_fw_id);
    MDNS.addServiceTxt(hMDNSService2, "id", shelly_name);
  }
#endif
  DEBUG_SERIAL.println("mDNS responder started");
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

