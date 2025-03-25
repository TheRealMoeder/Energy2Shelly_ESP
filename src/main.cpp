// Energy2Shelly_ESP v0.5.0
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

#define DEBUG true // set to false for no DEBUG output
#define DEBUG_SERIAL if(DEBUG)Serial

unsigned long startMillis;
unsigned long currentMillis;

// standard values ​​for the WiFi manager
// !! some values ​​are overwritten by the WifiManager !!
char wifi_AP[14] = "Energy2Shelly";
String wifi_ssid;

char input_type[40];
char mqtt_server[80];
char mqtt_port[6] = "1883";
// topic examples
// user define:     tele/meter/SENSOR           ->  {"ENERGY":{"Power": 9.99,"Consumption":77,"Production":33}}
// Tasmota device:  tele/tasmota_B60F3F/SENSOR  ->  {"Time":"2025-03-22T11:23:22","Main":{"power":35,"counter_pos":11241.750,"counter_neg":356.376}}
char mqtt_topic[60] = "tele/meter/SENSOR";
char mqtt_user[40] = "";
char mqtt_passwd[40] = "";
// Prerequisite, no change to the default values power_path energy_in_path energy_out_path
char power_path[60] = "ENERGY.Power";
char power_l1_path[60] = "";
char power_l2_path[60] = "";
char power_l3_path[60] = "";
char energy_in_path[60] ="ENERGY.Consumption";
char energy_out_path[60] ="ENERGY.Production";

// Shelly emulated device configuration
char shelly_app[7] = "Pro3EM";
char shelly_gen[2] = "2";
uint8_t shelly_slot = 1;
char shelly_fw_id[32] = "20241011-114455/1.4.4-g6d2a586";
char shelly_mac[13];
char shelly_model[15] = "SPEM-003CEBEU";
char shelly_name[26] = "shellypro3em-";
char shelly_ver[7] = "1.4.4";
char shelly_verBeta[12] = "1.4.9-beta6";
char query_period[10] = "1000";       // milliseconds
String power_variant = "triphase";    // standard, program adjusts variables depending on usage 

uint8_t VALvoltage = 230;
uint8_t VALfrequency = 50;
uint8_t VALpowerFactor = 1;

unsigned long period = 1000;
unsigned long uptime = 0;         // uptime uC
unsigned long secTick = 0;        // time since last second

int rpcId = 1;
char rpcUser[20] = "user_1";

// SMA Multicast IP and Port
unsigned int multicastPort = 9522;  // local port to listen on
IPAddress multicastIP(239, 12, 255, 254);

//flag for saving/resetting WifiManager data
bool shouldSaveConfig = false;
bool shouldResetConfig = false;

Preferences preferences;

//flags for data sources
bool dataMQTT = false;
bool dataMQTTconnect = false;
bool dataSMA = false;
bool dataSHRDZM = false;
bool dataHTTP = false;

struct PowerData
{
  double current;
  double voltage;
  double power;
  double apparentPower;
  double powerFactor;
  double frequency;
};

struct EnergyData
{
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

double round2(double value) {
  return (int)(value * 100 + 0.5) / 100.0;
}

JsonVariant resolveJsonPath(JsonVariant variant, const char* path) {
  for (size_t n = 0; path[n]; n++) {
    if (path[n] == '.') {
      variant = variant[JsonString(path, n)];
      path += n + 1;
      n = 0;
    }
  }
  return variant[path];
}

void setPowerData(double totalPower) {
  PhasePower[0].power = round2(totalPower * 0.3333);
  PhasePower[1].power = round2(totalPower * 0.3333);
  PhasePower[2].power = round2(totalPower * 0.3333);
  for(int i=0;i<=2;i++) {
    PhasePower[i].voltage = VALvoltage;
    PhasePower[i].current = round2(PhasePower[i].power / PhasePower[i].voltage);
    PhasePower[i].apparentPower = PhasePower[i].power;
    PhasePower[i].powerFactor = VALpowerFactor;
    PhasePower[i].frequency = VALfrequency;
  }
  DEBUG_SERIAL.print("Current total power: ");
  DEBUG_SERIAL.println(totalPower);
}

void setPowerData(double phase1Power, double phase2Power, double phase3Power) {
  PhasePower[0].power = round2(phase1Power);
  PhasePower[1].power = round2(phase2Power);
  PhasePower[2].power = round2(phase3Power);
  for(int i=0;i<=2;i++) {
    PhasePower[i].voltage = VALvoltage;
    PhasePower[i].current = round2(PhasePower[i].power / PhasePower[i].voltage);
    PhasePower[i].apparentPower = PhasePower[i].power;
    PhasePower[i].powerFactor = VALpowerFactor;
    PhasePower[i].frequency = VALfrequency;
  }
  DEBUG_SERIAL.print("Current power L1: ");
  DEBUG_SERIAL.print(phase1Power);
  DEBUG_SERIAL.print(" - L2: ");
  DEBUG_SERIAL.print(phase2Power);
  DEBUG_SERIAL.print(" - L3: ");
  DEBUG_SERIAL.println(phase3Power);
}

void setEnergyData(double totalEnergyGridSupply, double totalEnergyGridFeedIn) {    
  for(int i=0;i<=2;i++) {
    PhaseEnergy[i].consumption = round2(totalEnergyGridSupply * 0.3333);
    PhaseEnergy[i].gridfeedin = round2(totalEnergyGridFeedIn * 0.3333);
  }
  DEBUG_SERIAL.print("Total consumption: ");
  DEBUG_SERIAL.print(totalEnergyGridSupply);
  DEBUG_SERIAL.print(" - Total Grid Feed-In: ");
  DEBUG_SERIAL.println(totalEnergyGridFeedIn);
}

//callback notifying us of the need to save WifiManager config
void saveConfigCallback () {
  DEBUG_SERIAL.println("Should save config");
  shouldSaveConfig = true;
}

void setJsonPathPower(JsonDocument json) {
  if (strcmp(power_path, "TRIPHASE") == 0) {
    DEBUG_SERIAL.println("resolving triphase");
    power_variant = "triphase";
    double power1 = resolveJsonPath(json, power_l1_path);
    double power2 = resolveJsonPath(json, power_l2_path);
    double power3 = resolveJsonPath(json, power_l3_path);
    setPowerData(power1, power2, power3);
  } else {
    DEBUG_SERIAL.println("resolving monophase");
    power_variant = "monophase";
    double power = resolveJsonPath(json, power_path);
    setPowerData(power);
  }
  if ((strcmp(energy_in_path, "") != 0) && (strcmp(energy_out_path, "") != 0)) {
    double energyIn = resolveJsonPath(json, energy_in_path);
    double energyOut = resolveJsonPath(json, energy_out_path);
    setEnergyData(energyIn, energyOut);
  }
}

void rpcWSWrapper() {
  JsonDocument jsonResponse;
  JsonDocument doc;
  deserializeJson(doc,serJsonResponse);
  jsonResponse["id"] = rpcId;
  jsonResponse["src"] = shelly_name;
  if (strcmp(rpcUser,"EMPTY") != 0) {
    jsonResponse["dst"] = rpcUser;
  }
  jsonResponse["result"] = doc;
  serializeJson(jsonResponse,serJsonResponse);
}

void GetDeviceInfo() {
  JsonDocument jsonResponse;
  jsonResponse["name"] = shelly_name;
  jsonResponse["id"] = shelly_name;
  jsonResponse["mac"] = shelly_mac;
  jsonResponse["slot"] = shelly_slot;
  jsonResponse["model"] = shelly_model;
  jsonResponse["gen"] = shelly_gen;
  jsonResponse["fw_id"] = shelly_fw_id;
  jsonResponse["ver"] = shelly_ver;
  jsonResponse["app"] = shelly_app;
  jsonResponse["auth_en"] = false;
  jsonResponse["auth_domain"] = nullptr;
  jsonResponse["profile"] = power_variant;
  serializeJson(jsonResponse,serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
}

void EM1GetStatus_mono(){
  JsonDocument jsonResponse;
  // Reconstruction structure for FHEM -> no WARNINGS 
  jsonResponse["id"] = 0;
  jsonResponse["current"] = PhasePower[0].power + PhasePower[1].power + PhasePower[2].power;    // due to rounding, there is a difference to totalPower | example 12.4 -> 12.39
  jsonResponse["act_power"] = PhaseEnergy[0].consumption + PhaseEnergy[1].consumption + PhaseEnergy[2].consumption;   // due to rounding, there is a difference | example 98.00 -> 97.98
  jsonResponse["aprt_power"] = PhaseEnergy[0].gridfeedin + PhaseEnergy[1].gridfeedin + PhaseEnergy[2].gridfeedin;   // due to rounding, there is a difference |  8.00 -> 8.01
  jsonResponse["voltage"] = VALvoltage;
  jsonResponse["freq"] = VALfrequency;
  jsonResponse["pf"] = VALpowerFactor;
  serializeJson(jsonResponse,serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
}

void EM1DataGetStatus_mono(){
  JsonDocument jsonResponse;
  // Reconstruction structure for FHEM -> no WARNINGS 
  jsonResponse["id"] = 0;
  jsonResponse["total_act_energy"] = PhaseEnergy[0].consumption + PhaseEnergy[1].consumption + PhaseEnergy[2].consumption;    // Wirkenergie_Bezug
  jsonResponse["total_act_ret_energy"] = PhaseEnergy[0].gridfeedin + PhaseEnergy[1].gridfeedin + PhaseEnergy[2].gridfeedin;   // Wirkenergie_Einspeisung
  serializeJson(jsonResponse,serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
}

void EMGetStatus(){
  JsonDocument jsonResponse;
  String prepar = "{\"user_calibrated_phase\":[],\"errors\":[\"phase_sequence\"]}";     // Preparing JSON with empty array
  deserializeJson(jsonResponse, prepar);
  jsonResponse["id"] = 0;
  jsonResponse["a_current"] = PhasePower[0].current;
  jsonResponse["a_voltage"] = PhasePower[0].voltage;
  jsonResponse["a_act_power"] = PhasePower[0].power;
  jsonResponse["a_aprt_power"] = PhasePower[0].apparentPower;
  jsonResponse["a_pf"] = PhasePower[0].powerFactor;
  jsonResponse["a_freq"] = PhasePower[0].frequency;
  jsonResponse["b_current"] = PhasePower[1].current;
  jsonResponse["b_voltage"] = PhasePower[1].voltage;
  jsonResponse["b_act_power"] = PhasePower[1].power;
  jsonResponse["b_aprt_power"] = PhasePower[1].apparentPower;
  jsonResponse["b_pf"] = PhasePower[1].powerFactor;
  jsonResponse["b_freq"] = PhasePower[1].frequency;
  jsonResponse["c_current"] = PhasePower[2].current;
  jsonResponse["c_voltage"] = PhasePower[2].voltage;
  jsonResponse["c_act_power"] = PhasePower[2].power;
  jsonResponse["c_aprt_power"] = PhasePower[2].apparentPower;
  jsonResponse["c_pf"] = PhasePower[2].powerFactor;
  jsonResponse["c_freq"] = PhasePower[2].frequency;
  jsonResponse["n_current"] = 0.0;
  jsonResponse["total_current"] = round2((PhasePower[0].power + PhasePower[1].power + PhasePower[2].power) / VALvoltage);
  jsonResponse["total_act_power"] = PhasePower[0].power + PhasePower[1].power + PhasePower[2].power;
  jsonResponse["total_aprt_power"] = PhasePower[0].apparentPower + PhasePower[1].apparentPower + PhasePower[2].apparentPower;
  serializeJson(jsonResponse,serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
}

void EMDataGetStatus() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = 0;
  jsonResponse["a_total_act_energy"] = PhaseEnergy[0].consumption;
  jsonResponse["a_total_act_ret_energy"] = PhaseEnergy[0].gridfeedin;
  jsonResponse["b_total_act_energy"] = PhaseEnergy[1].consumption;
  jsonResponse["b_total_act_ret_energy"] = PhaseEnergy[1].gridfeedin;
  jsonResponse["c_total_act_energy"] = PhaseEnergy[2].consumption;
  jsonResponse["c_total_act_ret_energy"] = PhaseEnergy[2].gridfeedin;
  jsonResponse["total_act"] = PhaseEnergy[0].consumption + PhaseEnergy[1].consumption + PhaseEnergy[2].consumption;
  jsonResponse["total_act_ret"] = PhaseEnergy[0].gridfeedin + PhaseEnergy[1].gridfeedin + PhaseEnergy[2].gridfeedin;
  serializeJson(jsonResponse,serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
}

void EMGetConfig() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = 0;
  jsonResponse["name"] = nullptr;
  jsonResponse["blink_mode_selector"] = "active_energy";
  jsonResponse["phase_selector"] = "a";
  jsonResponse["monitor_phase_sequence"] = true;
  jsonResponse["ct_type"] = "120A";
  serializeJson(jsonResponse,serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
}

void ShellyGetConfig() {
  JsonDocument jsonResponse;
  String prepar = "{\"sys\":{\"ui_data\":{}}}";   // Preparing JSON with empty array
  deserializeJson(jsonResponse, prepar);

  jsonResponse["ble"]["enable"] = false;
  jsonResponse["ble"]["rpc"]["enable"] = false;
  jsonResponse["ble"]["observer"]["enable"] = false;
  jsonResponse["cloud"]["enable"] = false;
  jsonResponse["cloud"]["server"] = "iot.shelly.cloud:6012/jrpc";
  jsonResponse["mqtt"]["enable"] = dataMQTT;
  jsonResponse["mqtt"]["server"] = mqtt_server;
  jsonResponse["mqtt"]["client_id"] = shelly_name;
  jsonResponse["mqtt"]["user"] = nullptr;
  jsonResponse["mqtt"]["ssl_ca"] = nullptr;
  jsonResponse["mqtt"]["topic_prefix"] = mqtt_topic;
  jsonResponse["mqtt"]["rpc_ntf"] = true;
  jsonResponse["mqtt"]["status_ntf"] = true;
  jsonResponse["mqtt"]["use_client_cert"] = false;
  jsonResponse["mqtt"]["enable_rpc"] = true;
  jsonResponse["mqtt"]["enable_control"] = true;
  jsonResponse["sys"]["device"]["name"] = nullptr;
  jsonResponse["sys"]["device"]["mac"] = shelly_mac;
  jsonResponse["sys"]["device"]["fw_id"] = shelly_fw_id;
  jsonResponse["sys"]["device"]["discoverable"] = true;
  jsonResponse["sys"]["device"]["eco_mode"] = false;
  jsonResponse["sys"]["location"]["tz"] = "Europe/Berlin";
  jsonResponse["sys"]["location"]["lat"] = 51.342;
  jsonResponse["sys"]["location"]["lon"] = 12.3819;
  jsonResponse["sys"]["debug"]["level"] = 2;
  jsonResponse["sys"]["debug"]["file_level"] = nullptr;
  jsonResponse["sys"]["debug"]["mqtt"]["enable"] = false;
  jsonResponse["sys"]["debug"]["websocket"]["enable"] = false;
  jsonResponse["sys"]["debug"]["udp"]["addr"] = nullptr;
  jsonResponse["sys"]["debug"]["rpc_udp"]["dst_addr"] = nullptr;
  jsonResponse["sys"]["debug"]["rpc_udp"]["listen_port"] = nullptr;
  jsonResponse["sys"]["debug"]["sntp"]["server"] = "time.google.com";
  jsonResponse["sys"]["debug"]["cfg_rev"] = 13;
  jsonResponse["wifi"]["ap"]["ssid"] = wifi_AP;
  jsonResponse["wifi"]["ap"]["is_open"] = true;
  jsonResponse["wifi"]["ap"]["enable"] = false;
  jsonResponse["wifi"]["ap"]["range_extender"]["enable"] = false;
  jsonResponse["wifi"]["sta"]["ssid"] = wifi_ssid;
  jsonResponse["wifi"]["sta"]["is_open"] = false;
  jsonResponse["wifi"]["sta"]["enable"] = true;
  jsonResponse["wifi"]["sta"]["ipv4mode"] = "dhcp";
  jsonResponse["wifi"]["sta"]["ip"] = nullptr;
  jsonResponse["wifi"]["sta"]["netmask"] = nullptr;
  jsonResponse["wifi"]["sta"]["gw"] = nullptr;
  jsonResponse["wifi"]["sta"]["nameserver"] = nullptr;
  jsonResponse["wifi"]["sta1"]["ssid"] = nullptr;
  jsonResponse["wifi"]["sta1"]["is_open"] = true;
  jsonResponse["wifi"]["sta1"]["enable"] = false;
  jsonResponse["wifi"]["sta1"]["ipv4mode"] = "dhcp";
  jsonResponse["wifi"]["sta1"]["ip"] = nullptr;
  jsonResponse["wifi"]["sta1"]["netmask"] = nullptr;
  jsonResponse["wifi"]["sta1"]["gw"] = nullptr;
  jsonResponse["wifi"]["sta1"]["nameserver"] = nullptr;
  jsonResponse["wifi"]["roam"]["rssi_thr"] = WiFi.RSSI();
  jsonResponse["wifi"]["roam"]["interval"] = 60;
  jsonResponse["ws"]["enable"] = false;
  jsonResponse["ws"]["server"] = nullptr;
  jsonResponse["ws"]["ssl_ca"] = "ca.pem";
  serializeJson(jsonResponse,serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
}

void ShellyGetStatus(){
  JsonDocument jsonResponse;
  // https://forum.arduino.cc/t/shelly-pro-3em-esp8266-json/1152558
  String prepar = "{\"ble\":{},\"em:0\":{\"user_calibrated_phase\":[]},\"modbus\":{}}";     // Preparing JSON with empty array
  deserializeJson(jsonResponse, prepar);
  jsonResponse["cloud"]["connected"] = false;
  jsonResponse["em:0"]["id"] = 0;
  jsonResponse["em:0"]["a_current"] = 0;
  jsonResponse["em:0"]["a_voltage"] = 0;
  jsonResponse["em:0"]["a_act_power"] = 0;
  jsonResponse["em:0"]["a_aprt_power"] = 0;
  jsonResponse["em:0"]["a_pf"] = 0;
  jsonResponse["em:0"]["b_current"] = 0;
  jsonResponse["em:0"]["b_voltage"] = 0;
  jsonResponse["em:0"]["b_act_power"] = 0;
  jsonResponse["em:0"]["b_aprt_power"] = 0;
  jsonResponse["em:0"]["b_pf"] = 0;
  jsonResponse["em:0"]["c_current"] = 0;
  jsonResponse["em:0"]["c_voltage"] = 0;
  jsonResponse["em:0"]["c_act_power"] = 0;
  jsonResponse["em:0"]["c_aprt_power"] = 0;
  jsonResponse["em:0"]["c_pf"] = 0;
  jsonResponse["em:0"]["n_current"] = 0;
  jsonResponse["em:0"]["total_current"] = 0;
  jsonResponse["em:0"]["total_act_power"] = 0;
  jsonResponse["em:0"]["total_aprt_power"] = 0;
  jsonResponse["emdata:0"]["id"] = 0;
  jsonResponse["emdata:0"]["a_total_act_energy"] = 0;
  jsonResponse["emdata:0"]["a_total_act_ret_energy"] = 0;
  jsonResponse["emdata:0"]["b_total_act_energy"] = 0;
  jsonResponse["emdata:0"]["b_total_act_ret_energy"] = 0;
  jsonResponse["emdata:0"]["c_total_act_energy"] = 0;
  jsonResponse["emdata:0"]["c_total_act_ret_energy"] = 0;
  jsonResponse["emdata:0"]["total_act"] = 0;
  jsonResponse["emdata:0"]["total_act_ret"] = 0;
  jsonResponse["eth"]["ip"] = nullptr;
  jsonResponse["mqtt"]["connected"] = dataMQTTconnect;
  jsonResponse["sys"]["mac"] = shelly_mac;
  jsonResponse["sys"]["restart_required"] = false;
  jsonResponse["sys"]["time"] = "12:03";
  jsonResponse["sys"]["unixtime"] = 1690538581;
  jsonResponse["sys"]["uptime"] = uptime;
  jsonResponse["sys"]["ram_size"] = 246644;
  jsonResponse["sys"]["ram_free"] = 121648;
  jsonResponse["sys"]["fs_size"] = 524288;
  jsonResponse["sys"]["fs_free"] = 172032;
  jsonResponse["sys"]["cfg_rev"] = 17;
  jsonResponse["sys"]["kvs_rev"] = 0;
  jsonResponse["sys"]["webhook_rev"] = 0;
  jsonResponse["sys"]["available_updates"]["beta"]["version"] = shelly_verBeta;
  jsonResponse["temperature:0"]["id"] = 0;
  jsonResponse["temperature:0"]["tC"] = 44.9;
  jsonResponse["temperature:0"]["tF"] = 112.7;
  #ifdef ESP32
  jsonResponse["wifi"]["sta_ip"] = WiFi.localIP();    // supports only ESP32
  #else
  jsonResponse["wifi"]["sta_ip"] = "192.168.172.21";  // ToDo for ESP8266 Variant
  #endif
  jsonResponse["wifi"]["status"] = "got ip";
  jsonResponse["wifi"]["ssid"] = wifi_ssid;
  jsonResponse["wifi"]["rssi"] = WiFi.RSSI();
  jsonResponse["ws"]["connected"] = false;
  serializeJson(jsonResponse,serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
}

void webSocketEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
  JsonDocument json;
  switch(type) {
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
              GetDeviceInfo();
              rpcWSWrapper();
              webSocket.textAll(serJsonResponse);
            } else if(json["method"] == "EM.GetStatus") {
              strcpy(rpcUser,json["src"]);
              EMGetStatus();
              rpcWSWrapper();
              webSocket.textAll(serJsonResponse);
            } else if(json["method"] == "EMData.GetStatus") {
              strcpy(rpcUser,json["src"]);
              EMDataGetStatus();
              rpcWSWrapper();
              webSocket.textAll(serJsonResponse);
            } else if(json["method"] == "EM.GetConfig") {
              EMGetConfig();
              rpcWSWrapper();
              webSocket.textAll(serJsonResponse);
            }
            else {
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

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  JsonDocument json;
  deserializeJson(json, payload, length);
  setJsonPathPower(json);
}

void mqtt_reconnect() {
  DEBUG_SERIAL.print("Attempting MQTT connection...");
  if (mqtt_client.connect(shelly_name, String(mqtt_user).c_str(), String(mqtt_passwd).c_str())) {
    DEBUG_SERIAL.println("connected");
    dataMQTTconnect = true;
    mqtt_client.subscribe(mqtt_topic);
  } else {
    DEBUG_SERIAL.print("failed, rc=");
    DEBUG_SERIAL.print(mqtt_client.state());
    DEBUG_SERIAL.println(" try again in 5 seconds");
    dataMQTTconnect = false;
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
        GetDeviceInfo();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if(json["method"] == "EM.GetStatus") {
        EMGetStatus();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if(json["method"] == "EMData.GetStatus") {
        EMDataGetStatus();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if(json["method"] == "EM.GetConfig") {
        EMGetConfig();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      }
      else {
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
      uint8_t* offset = buffer + 4;
      do {
          grouplen = (offset[0] << 8) + offset[1];
          grouptag = (offset[2] << 8) + offset[3];
          offset += 4;
          if (grouplen == 0xffff) return;
          if (grouptag == 0x02A0 && grouplen == 4) {
              offset += 4;
          } else if (grouptag == 0x0010) {
              uint8_t* endOfGroup = offset + grouplen;
              #ifdef ESP32
              uint16_t protocolID = (offset[0] << 8) + offset[1];   // current status | warning: unused variable 'protocolID' with ESP8266 compile
              #endif
              offset += 2;
              #ifdef ESP32
              uint16_t susyID = (offset[0] << 8) + offset[1];   // current status | warning: unused variable 'susyID' with ESP8266 compile
              #endif
              offset += 2;
              #ifdef ESP32    // current status | warning: unused variable 'serial' with ESP8266 compile
              uint32_t serial = (offset[0] << 24) + (offset[1] << 16) + (offset[2] << 8) + offset[3];
              #endif
              offset += 4;
              #ifdef ESP32    // current status | warning: unused variable 'timestamp' with ESP8266 compile
              uint32_t timestamp = (offset[0] << 24) + (offset[1] << 16) + (offset[2] << 8) + offset[3];
              #endif
              offset += 4;
              while (offset < endOfGroup) {
                  uint8_t channel = offset[0];
                  uint8_t index = offset[1];
                  uint8_t type = offset[2];
                  #ifdef ESP32    // current status | warning: unused variable ' with ESP8266 compile
                  uint8_t tarif = offset[3];
                  #endif
                  offset += 4;
                  if (type == 8) {
                    uint64_t data = ((uint64_t)offset[0] << 56) +
                                  ((uint64_t)offset[1] << 48) +
                                  ((uint64_t)offset[2] << 40) +
                                  ((uint64_t)offset[3] << 32) +
                                  ((uint64_t)offset[4] << 24) +
                                  ((uint64_t)offset[5] << 16) +
                                  ((uint64_t)offset[6] << 8) +
                                  offset[7];
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
                    uint32_t data = (offset[0] << 24) +
                    (offset[1] << 16) +
                    (offset[2] << 8) +
                    offset[3];
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
                      PhasePower[0].frequency = VALfrequency;
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
                      PhasePower[1].frequency = VALfrequency;
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
                      PhasePower[2].frequency = VALfrequency;
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
      setEnergyData(energyIn,energyOut);
    }
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
  sprintf (shelly_mac, "%02x%02x%02x%02x%02x%02x", mac [0], mac [1], mac [2], mac [3], mac [4], mac [5]);

  preferences.begin("e2s_config", false);
  strcpy(input_type, preferences.getString("input_type", input_type).c_str());
  strcpy(mqtt_server, preferences.getString("mqtt_server", mqtt_server).c_str());
  strcpy(query_period, preferences.getString("query_period", query_period).c_str());
  strcpy(shelly_mac, preferences.getString("shelly_mac", shelly_mac).c_str());
  strcpy(mqtt_port, preferences.getString("mqtt_port", mqtt_port).c_str());
  strcpy(mqtt_topic, preferences.getString("mqtt_topic", mqtt_topic).c_str());
  strcpy(mqtt_user, preferences.getString("mqtt_user", mqtt_user).c_str());
  strcpy(mqtt_passwd, preferences.getString("mqtt_passwd", mqtt_passwd).c_str());
  strcpy(power_path, preferences.getString("power_path", power_path).c_str());
  strcpy(power_l1_path, preferences.getString("power_l1_path", power_l1_path).c_str());
  strcpy(power_l2_path, preferences.getString("power_l2_path", power_l2_path).c_str());
  strcpy(power_l3_path, preferences.getString("power_l3_path", power_l3_path).c_str());
  strcpy(energy_in_path, preferences.getString("energy_in_path", energy_in_path).c_str());
  strcpy(energy_out_path, preferences.getString("energy_out_path", energy_out_path).c_str());
  
  WiFiManagerParameter custom_section1("<h3>General settings</h3>");
  WiFiManagerParameter custom_input_type("type", "<b>Data source</b><br>\"MQTT\" for MQTT, \"HTTP\" for generic HTTP, \"SMA\" for SMA EM/HM multicast or \"SHRDZM\" for SHRDZM UDP data", input_type, 40);
  WiFiManagerParameter custom_mqtt_server("server", "<b>Server</b><br>MQTT Server IP or query url for generic HTTP", mqtt_server, 80);
  WiFiManagerParameter custom_query_period("query_period", "<b>Query period</b><br>for generic HTTP, in milliseconds", query_period, 10);
  WiFiManagerParameter custom_shelly_mac("mac", "<b>Shelly ID</b><br>12 char hexadecimal, defaults to MAC address of ESP", shelly_mac, 13);
  WiFiManagerParameter custom_section2("<hr><h3>MQTT options</h3>");
  WiFiManagerParameter custom_mqtt_port("port", "<b>MQTT Port</b>", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_topic("topic", "<b>MQTT Topic</b>", mqtt_topic, 60);
  WiFiManagerParameter custom_mqtt_user("user", "<b>MQTT user</b><br>optional", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_passwd("passwd", "<b>MQTT password</b><br>optional", mqtt_passwd, 40);
  WiFiManagerParameter custom_section3("<hr><h3>JSON paths for MQTT and generic HTTP</h3>");
  WiFiManagerParameter custom_power_path("power_path", "<b>Total power JSON path</b><br>e.g. \"ENERGY.Power\" or \"TRIPHASE\" for tri-phase data", power_path, 60);
  WiFiManagerParameter custom_power_l1_path("power_l1_path", "<b>Phase 1 power JSON path</b><br>optional", power_l1_path, 60);
  WiFiManagerParameter custom_power_l2_path("power_l2_path", "<b>Phase 2 power JSON path</b><br>Phase 2 power JSON path<br>optional", power_l2_path, 60);
  WiFiManagerParameter custom_power_l3_path("power_l3_path", "<b>Phase 3 power JSON path</b><br>Phase 3 power JSON path<br>optional", power_l3_path, 60);
  WiFiManagerParameter custom_energy_in_path("energy_in_path", "<b>Energy from grid JSON path</b><br>e.g. \"ENERGY.Grid\"", energy_in_path, 60);
  WiFiManagerParameter custom_energy_out_path("energy_out_path", "<b>Energy to grid JSON path</b><br>e.g. \"ENERGY.FeedIn\"", energy_out_path, 60);

  WiFiManager wifiManager;
  if(!DEBUG) {
    wifiManager.setDebugOutput(false);
  }
  wifiManager.setTitle("Energy2Shelly for ESP");
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_section1);
  wifiManager.addParameter(&custom_input_type);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_query_period);
  wifiManager.addParameter(&custom_shelly_mac);
  wifiManager.addParameter(&custom_section2);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_topic);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_passwd);
  wifiManager.addParameter(&custom_section3);
  wifiManager.addParameter(&custom_power_path);
  wifiManager.addParameter(&custom_power_l1_path);
  wifiManager.addParameter(&custom_power_l2_path);
  wifiManager.addParameter(&custom_power_l3_path);
  wifiManager.addParameter(&custom_energy_in_path);
  wifiManager.addParameter(&custom_energy_out_path);


  if (!wifiManager.autoConnect(wifi_AP)) {
    DEBUG_SERIAL.println("failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
    delay(5000);
  }
  DEBUG_SERIAL.println("connected");
  wifi_ssid = wifiManager.getWiFiSSID();

  //read updated parameters
  strcpy(input_type, custom_input_type.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(query_period, custom_query_period.getValue());
  strcpy(shelly_mac, custom_shelly_mac.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_passwd, custom_mqtt_passwd.getValue());
  strcpy(power_path, custom_power_path.getValue());
  strcpy(power_l1_path, custom_power_l1_path.getValue());
  strcpy(power_l2_path, custom_power_l2_path.getValue());
  strcpy(power_l3_path, custom_power_l3_path.getValue());
  strcpy(energy_in_path, custom_energy_in_path.getValue());
  strcpy(energy_out_path, custom_energy_out_path.getValue());

  DEBUG_SERIAL.println("The values in the preferences are: ");
  DEBUG_SERIAL.println("\tinput_type : " + String(input_type));
  DEBUG_SERIAL.println("\tmqtt_server : " + String(mqtt_server));
  DEBUG_SERIAL.println("\tquery_period : " + String(query_period));
  DEBUG_SERIAL.println("\tshelly_mac : " + String(shelly_mac));
  DEBUG_SERIAL.println("\tmqtt_port : " + String(mqtt_port));
  DEBUG_SERIAL.println("\tmqtt_topic : " + String(mqtt_topic));
  DEBUG_SERIAL.println("\tmqtt_user : " + String(mqtt_user));
  DEBUG_SERIAL.println("\tmqtt_passwd : " + String(mqtt_passwd));
  DEBUG_SERIAL.println("\tpower_path : " + String(power_path));
  DEBUG_SERIAL.println("\tpower_l1_path : " + String(power_l1_path));
  DEBUG_SERIAL.println("\tpower_l2_path : " + String(power_l2_path));
  DEBUG_SERIAL.println("\tpower_l3_path : " + String(power_l3_path));
  DEBUG_SERIAL.println("\tenergy_in_path : " + String(energy_in_path));
  DEBUG_SERIAL.println("\tenergy_out_path : " + String(energy_out_path));


  if(strcmp(input_type, "SMA") == 0) {
    dataSMA = true;
    DEBUG_SERIAL.println("Enabling SMA Multicast data input");
  } else if (strcmp(input_type, "SHRDZM") == 0) {
    dataSHRDZM = true;
    DEBUG_SERIAL.println("Enabling SHRDZM UDP data input");
  } else if (strcmp(input_type, "HTTP") == 0) {
    dataHTTP = true;
    DEBUG_SERIAL.println("Enabling generic HTTP data input");
  } else {
    dataMQTT = true;
    DEBUG_SERIAL.println("Enabling MQTT data input");
  }

  if (shouldSaveConfig) {
    DEBUG_SERIAL.println("saving config");
    preferences.putString("input_type", input_type);
    preferences.putString("mqtt_server", mqtt_server);
    preferences.putString("query_period", query_period);
    preferences.putString("shelly_mac", shelly_mac);
    preferences.putString("mqtt_port", mqtt_port);
    preferences.putString("mqtt_topic", mqtt_topic);
    preferences.putString("mqtt_user", mqtt_user);
    preferences.putString("mqtt_passwd", mqtt_passwd);
    preferences.putString("power_path", power_path);
    preferences.putString("power_l1_path", power_l1_path);
    preferences.putString("power_l2_path", power_l2_path);
    preferences.putString("power_l3_path", power_l3_path);
    preferences.putString("energy_in_path", energy_in_path);
    preferences.putString("energy_out_path", energy_out_path);
    wifiManager.reboot();
  }
  DEBUG_SERIAL.println("local ip");
  DEBUG_SERIAL.println(WiFi.localIP());
}

void setup(void) {
  DEBUG_SERIAL.begin(115200);
  WifiManagerSetup();


  // HTTP Sites //
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "This is the Energy2Shelly for ESP converter!\r\nDevice and Energy status is available under /status\r\nTo reset configuration, goto /reset\r\n");
  });

  // https://shelly-api-docs.shelly.cloud/gen2/0.14/Devices/ShellyPro3EM/
  // https://shelly-api-docs.shelly.cloud/gen2/Devices/Gen2/ShellyPro3EM/
  // https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly/#http-endpoint-shelly
  // https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly/#shellygetdeviceinfo
  // http://${SHELLY}/shelly == this resource is equivalent to invoking Shelly.GetDeviceInfo
  server.on("/shelly", HTTP_GET, [](AsyncWebServerRequest *request) {
    GetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/Shelly.GetConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    ShellyGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/Shelly.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    ShellyGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
    shouldResetConfig = true;
    request->send(200, "text/plain", "Resetting WiFi configuration, please log back into the hotspot to reconfigure...\r\n");
  });

  // https://shelly-api-docs.shelly.cloud/gen2/Devices/Gen2/ShellyPro3EM/
  // https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EM/#emgetstatus-example
  // https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EMData/#emdatagetstatus-example

  // !!! FHEM comp.
  server.on("/rpc/BLE.CloudRelay.List", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", "{\"rev\":0,\"addrs\":[]}");
  });

  // !!! Leistungswerte | ./rpc/EM.GetStatus nur bei triphase
  // !!! FHEM comp.
  server.on("/rpc/EM.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  // !!! Leistungswerte | ./rpc/EM1.GetStatus nur bei monophase
  // !!! FHEM comp.
  server.on("/rpc/EM1.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EM1GetStatus_mono();
    request->send(200, "application/json", serJsonResponse);
  });

  // !!! Zählerstände | ./rpc/EMData.GetStatus nur bei triphase
  // !!! FHEM comp.
  server.on("/rpc/EMData.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMDataGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  // !!! Zählerstände | ./rpc/EM1Data.GetStatus nur bei monophase
  // !!! FHEM comp.
  server.on("/rpc/EM1Data.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EM1DataGetStatus_mono();
    request->send(200, "application/json", serJsonResponse);
  });

  // !!! FHEM comp. ???
  server.on("/rpc/EM.GetConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });

  // !!! FHEM comp.
  server.on("/rpc/Shelly.GetDeviceInfo", HTTP_GET, [](AsyncWebServerRequest *request) {
    GetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });

  // !!! FHEM comp.
  server.on("/rpc/Script.List", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", "{\"scripts\":[]}");
  });

  // !!! FHEM comp.
  server.on("/rpc/Webhook.List", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", "{\"hooks\":[], \"rev\":0}");
  });

  server.on("/rpc", HTTP_POST, [](AsyncWebServerRequest *request) {
    GetDeviceInfo();
    rpcWSWrapper();
    request->send(200, "application/json", serJsonResponse);
  });

  webSocket.onEvent(webSocketEvent);
  server.addHandler(&webSocket);
  server.begin();

  // Set up RPC over UDP for Marstek users
  UdpRPC.begin(1010);

  // Set up MQTT
  if(dataMQTT) {
    mqtt_client.setServer(mqtt_server, String(mqtt_port).toInt());
    mqtt_client.setCallback(mqtt_callback);
  }

  // Set Up Multicast for SMA Energy Meter
  if(dataSMA) {
    Udp.begin(multicastPort);
    #ifdef ESP8266
      Udp.beginMulticast(WiFi.localIP(), multicastIP, multicastPort);
    #else
      Udp.beginMulticast(multicastIP, multicastPort);
    #endif
  }

  // Set Up UDP for SHRDZM smart meter interface
  if(dataSHRDZM) {
    Udp.begin(multicastPort);
  }

  // Set Up HTTP query
  if(dataHTTP) {
    period = atol(query_period);
    startMillis = millis();
    http.useHTTP10(true);
  }

  // Set up mDNS responder
  strcat(shelly_name,shelly_mac);
  if (!MDNS.begin(shelly_name)) {
    DEBUG_SERIAL.println("Error setting up MDNS responder!");
  }

  #ifdef ESP32
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("shelly", "tcp", 80);
    mdns_txt_item_t serviceTxtData[4] = {
      {"fw_id",shelly_fw_id},
      {"arch","esp8266"},
      {"id",shelly_name},
      {"gen",shelly_gen}
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
      MDNS.addServiceTxt(hMDNSService, "fw_id", shelly_fw_id);
      MDNS.addServiceTxt(hMDNSService, "arch", "esp8266");
      MDNS.addServiceTxt(hMDNSService, "id", shelly_name);
      MDNS.addServiceTxt(hMDNSService, "gen", shelly_gen);
    }
    if (hMDNSService2) {
      MDNS.setServiceName(hMDNSService2, shelly_name);
      MDNS.addServiceTxt(hMDNSService2, "fw_id", shelly_fw_id);
      MDNS.addServiceTxt(hMDNSService2, "arch", "esp8266");
      MDNS.addServiceTxt(hMDNSService2, "id", shelly_name);
      MDNS.addServiceTxt(hMDNSService2, "gen", shelly_gen);
    }
  #endif
  DEBUG_SERIAL.println("mDNS responder started");
}

void loop() {
  if ((millis() - secTick) >= 1000UL) {   // jede Sekunde
    secTick += 1000UL;
    uptime++;
  }
  #ifndef ESP32
    MDNS.update();
  #endif
  parseUdpRPC();
  if(shouldResetConfig) {
    #ifdef ESP32
      WiFi.disconnect(true, true);
    #else
      WiFi.disconnect(true);
    #endif
    delay(1000);
    ESP.restart();  
  }
  if(dataMQTT) {
    if (!mqtt_client.connected()) {
      mqtt_reconnect();
    }
    mqtt_client.loop();
  }
  if(dataSMA) {
    parseSMA();
  }
  if(dataSHRDZM) {
    parseSHRDZM();
  
  }
  if(dataHTTP) {
    currentMillis = millis();
    if (currentMillis - startMillis >= period) {
      queryHTTP();
      startMillis = currentMillis;
    }
  }
}