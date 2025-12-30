// Energy2Shelly_ESP v0.5.2
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
unsigned long startMillis_sunspec = 0;
unsigned long currentMillis;

// Configuration struct - replaces 20+ char arrays
struct Config {
  String inputType;
  String mqttServer;
  String mqttPort = "1883";
  String mqttTopic = "tele/meter/SENSOR";
  String mqttUser;
  String mqttPasswd;
  String powerPath;
  String pwrExportPath;
  String powerL1Path;
  String powerL2Path;
  String powerL3Path;
  String energyInPath;
  String energyOutPath;
  String shellyGen = "2";
  String shellyFwId = "20241011-114455/1.4.4-g6d2a586";
  String shellyMac;
  String shellyName = "shellypro3em-";
  String queryPeriod = "1000";
  String modbusDevice = "71";
  String shellyPort = "2220";
  bool forcePwrDecimals = true;
  String smaId;
  
  // Cached numeric conversions
  int mqttPortInt = 1883;
  int shellyPortInt = 2220;
  int ledGpioInt = 0;
  int queryPeriodMs = 1000;
  int modbusDeviceId = 71;
  bool ledInverted = false;
} config;

// Global buffers - moved from stack to avoid fragmentation
static uint8_t networkBuffer[1024];
static JsonDocument globalJsonDoc;

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

unsigned long period = 1430;
int rpcId = 1;
String rpcUser = "user_1";

// SMA Multicast IP and Port
unsigned int multicastPort = 9522;  // local port to listen on
IPAddress multicastIP(239, 12, 255, 254);

// flags for saving/resetting WifiManager data
bool shouldSaveConfig = false;
bool shouldResetConfig = false;
bool shouldReboot = false;

Preferences preferences;

// flags for data sources
bool dataMQTT = false;
bool dataSMA = false;
bool dataSHRDZM = false;
bool dataHTTP = false;
bool dataSUNSPEC = false;
bool mqtt_configured = false;  // Flag to indicate MQTT is fully configured and safe to use

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

bool isValidIPAddress(const char* ipString) {
  IPAddress ip;
  return ip.fromString(ipString);
}

double round2(double value) {
  int ivalue = (int)(value * 100.0 + (value > 0.0 ? 0.5 : -0.5));

  // fix Marstek bug: make sure to have decimal numbers
  if(config.forcePwrDecimals && (ivalue % 100 == 0)) ivalue++;
  
  return ivalue / 100.0;
}

// Helper function to set individual phase data
void setPhaseData(int phaseIndex, double current, double voltage, double power, 
                  double apparentPower, double powerFactor, int frequency) {
  PhasePower[phaseIndex].current = round2(current);
  PhasePower[phaseIndex].voltage = round2(voltage);
  PhasePower[phaseIndex].power = round2(power);
  PhasePower[phaseIndex].apparentPower = round2(apparentPower);
  PhasePower[phaseIndex].powerFactor = round2(powerFactor);
  PhasePower[phaseIndex].frequency = frequency;
}

// Helper function to distribute single power value across three phases
void setPhaseDataSingle(int phaseIndex, double power) {
  PhasePower[phaseIndex].power = round2(power);
  PhasePower[phaseIndex].voltage = round2(defaultVoltage);
  PhasePower[phaseIndex].current = round2(PhasePower[phaseIndex].power / PhasePower[phaseIndex].voltage);
  PhasePower[phaseIndex].apparentPower = round2(PhasePower[phaseIndex].power);
  PhasePower[phaseIndex].powerFactor = round2(defaultPowerFactor);
  PhasePower[phaseIndex].frequency = defaultFrequency;
}

// Helper function to set individual phase energy data
void setPhaseEnergyData(int phaseIndex, double consumption, double gridfeedin) {
  PhaseEnergy[phaseIndex].consumption = round2(consumption);
  PhaseEnergy[phaseIndex].gridfeedin = round2(gridfeedin);
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
  double powerPerPhase = totalPower * 0.3333;
  for (int i = 0; i <= 2; i++) {
    setPhaseDataSingle(i, powerPerPhase);
  }
  DEBUG_SERIAL.print("Current total power: ");
  DEBUG_SERIAL.println(totalPower);
}

void setPowerData(double phase1Power, double phase2Power, double phase3Power) {
  for (int i = 0; i <= 2; i++) {
    double power = (i == 0) ? phase1Power : (i == 1) ? phase2Power : phase3Power;
    setPhaseDataSingle(i, power);
  }
  DEBUG_SERIAL.print("Current power L1: ");
  DEBUG_SERIAL.print(phase1Power);
  DEBUG_SERIAL.print(" - L2: ");
  DEBUG_SERIAL.print(phase2Power);
  DEBUG_SERIAL.print(" - L3: ");
  DEBUG_SERIAL.println(phase3Power);
}


void setEnergyData(double totalEnergyGridSupply, double totalEnergyGridFeedIn) {
  double consumptionPerPhase = totalEnergyGridSupply * 0.3333;
  double gridFeedinPerPhase = totalEnergyGridFeedIn * 0.3333;
  for (int i = 0; i <= 2; i++) {
    setPhaseEnergyData(i, consumptionPerPhase, gridFeedinPerPhase);
  }
  DEBUG_SERIAL.print("Total consumption: ");
  DEBUG_SERIAL.print(totalEnergyGridSupply);
  DEBUG_SERIAL.print(" - Total Grid Feed-In: ");
  DEBUG_SERIAL.println(totalEnergyGridFeedIn);
}

//callback notifying us of the need to save WifiManager config
void saveConfigCallback() {
  DEBUG_SERIAL.println("Should save config");
  shouldSaveConfig = true;
}

void setJsonPathPower(JsonDocument json) {
  // If the incoming JSON already uses Shelly 3EM field names, parse directly
  if (json["a_current"].is<JsonVariant>() || json["a_act_power"].is<JsonVariant>()) {
    DEBUG_SERIAL.println("Parsing direct Shelly 3EM payload");
    setPhaseData(0, json["a_current"].as<double>(), json["a_voltage"].as<double>(), 
                 json["a_act_power"].as<double>(), json["a_aprt_power"].as<double>(),
                 json["a_pf"].as<double>(), json["a_freq"].as<int>());
    setPhaseData(1, json["b_current"].as<double>(), json["b_voltage"].as<double>(),
                 json["b_act_power"].as<double>(), json["b_aprt_power"].as<double>(),
                 json["b_pf"].as<double>(), json["b_freq"].as<int>());
    setPhaseData(2, json["c_current"].as<double>(), json["c_voltage"].as<double>(),
                 json["c_act_power"].as<double>(), json["c_aprt_power"].as<double>(),
                 json["c_pf"].as<double>(), json["c_freq"].as<int>());

    // Optionally use total fields if present
    if (json["total_act_power"].is<JsonVariant>()) {
      double total = json["total_act_power"].as<double>();
      // distribute if individual phases missing or for logging
      DEBUG_SERIAL.print("Total power from payload: ");
      DEBUG_SERIAL.println(total);
    }
    return;
  }
  if (config.powerPath == "TRIPHASE") {
    DEBUG_SERIAL.println("resolving triphase");
    double power1 = resolveJsonPath(json, config.powerL1Path.c_str());
    double power2 = resolveJsonPath(json, config.powerL2Path.c_str());
    double power3 = resolveJsonPath(json, config.powerL3Path.c_str());
    DEBUG_SERIAL.println(power1);
    setPowerData(power1, power2, power3);
  } else {
    // Check if BOTH paths (Import = powerPath, Export = pwrExportPath) are defined
    if ((config.powerPath != "") && (config.pwrExportPath != "")) {
      DEBUG_SERIAL.println("Resolving net power (import - export)");
      double importPower = resolveJsonPath(json, config.powerPath.c_str()).as<double>();
      double exportPower = resolveJsonPath(json, config.pwrExportPath.c_str()).as<double>();
      double netPower = importPower - exportPower;
      setPowerData(netPower);
    }
    // (FALLBACK): Only the normal powerPath (import path) is defined (old logic)
    else if (config.powerPath != "") {
      DEBUG_SERIAL.println("Resolving monophase (single path only)");
      double power = resolveJsonPath(json, config.powerPath.c_str()).as<double>();
      setPowerData(power);
    }
  }
  if ((config.energyInPath != "") && (config.energyOutPath != "")) {
    double energyIn = resolveJsonPath(json, config.energyInPath.c_str());
    double energyOut = resolveJsonPath(json, config.energyOutPath.c_str());
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
  jsonResponse["src"] = config.shellyName;
  if (rpcUser != "EMPTY") {
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

void GetDeviceInfo() {
  JsonDocument jsonResponse;
  jsonResponse["name"] = config.shellyName;
  jsonResponse["id"] = config.shellyName;
  jsonResponse["mac"] = config.shellyMac;
  jsonResponse["slot"] = 1;
  jsonResponse["model"] = "SPEM-003CEBEU";
  jsonResponse["gen"] = config.shellyGen;
  jsonResponse["fw_id"] = config.shellyFwId;
  jsonResponse["ver"] = "1.4.4";
  jsonResponse["app"] = "Pro3EM";
  jsonResponse["auth_en"] = false;
  jsonResponse["profile"] = "triphase";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

void EMGetStatus() {
  JsonDocument jsonResponse;
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
  jsonResponse["total_current"] = round2((PhasePower[0].power + PhasePower[1].power + PhasePower[2].power) / ((float)defaultVoltage));
  jsonResponse["total_act_power"] = PhasePower[0].power + PhasePower[1].power + PhasePower[2].power;
  jsonResponse["total_aprt_power"] = PhasePower[0].apparentPower + PhasePower[1].apparentPower + PhasePower[2].apparentPower;
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
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
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

void EMGetConfig() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = 0;
  jsonResponse["name"] = nullptr;
  jsonResponse["blink_mode_selector"] = "active_energy";
  jsonResponse["phase_selector"] = "a";
  jsonResponse["monitor_phase_sequence"] = true;
  jsonResponse["ct_type"] = "120A";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
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
          deserializeJson(globalJsonDoc, data);
          rpcId = globalJsonDoc["id"];
          if (globalJsonDoc["method"] == "Shelly.GetDeviceInfo") {
            rpcUser = "EMPTY";
            GetDeviceInfo();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (globalJsonDoc["method"] == "EM.GetStatus") {
            rpcUser = globalJsonDoc["src"].as<String>();
            EMGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (globalJsonDoc["method"] == "EMData.GetStatus") {
            rpcUser = globalJsonDoc["src"].as<String>();
            EMDataGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (globalJsonDoc["method"] == "EM.GetConfig") {
            EMGetConfig();
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
  DEBUG_SERIAL.println("DEBUG: mqtt_reconnect() called, server=" + config.mqttServer + ", mqtt_configured=" + String(mqtt_configured));
  // Prevent reconnection if MQTT not properly configured
  if (!mqtt_configured || config.mqttServer == "" || config.mqttServer.length() == 0) {
    DEBUG_SERIAL.println("DEBUG: mqtt_reconnect() aborting - MQTT not configured or empty server");
    return;
  }
  
  DEBUG_SERIAL.print("Attempting MQTT connection...");
  if (mqtt_client.connect(config.shellyName.c_str(), config.mqttUser.c_str(), config.mqttPasswd.c_str())) {
    DEBUG_SERIAL.println("connected");
    mqtt_client.subscribe(config.mqttTopic.c_str());
  } else {
    DEBUG_SERIAL.print("failed, rc=");
    DEBUG_SERIAL.print(mqtt_client.state());
    DEBUG_SERIAL.println(" try again in 5 seconds");
    delay(5000);
  }
}

void parseUdpRPC() {
  int packetSize = UdpRPC.parsePacket();
  if (packetSize) {
    int rSize = UdpRPC.read(networkBuffer, 1024);
    networkBuffer[rSize] = 0;
    DEBUG_SERIAL.print("Received UDP packet on port 1010: ");
    DEBUG_SERIAL.println((char *)networkBuffer);
    deserializeJson(globalJsonDoc, networkBuffer);
    if (globalJsonDoc["method"].is<JsonVariant>()) {
      rpcId = globalJsonDoc["id"];
      rpcUser = "EMPTY";
      UdpRPC.beginPacket(UdpRPC.remoteIP(), UdpRPC.remotePort());
      if (globalJsonDoc["method"] == "Shelly.GetDeviceInfo") {
        GetDeviceInfo();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (globalJsonDoc["method"] == "EM.GetStatus") {
        EMGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (globalJsonDoc["method"] == "EMData.GetStatus") {
        EMDataGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (globalJsonDoc["method"] == "EM.GetConfig") {
        EMGetConfig();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else {
        DEBUG_SERIAL.printf("RPC over UDP: unknown request: %s\n", networkBuffer);
      }
      UdpRPC.endPacket();
    }
    globalJsonDoc.clear();
  }
}

void parseSMA() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int rSize = Udp.read(networkBuffer, 1024);
    if (networkBuffer[0] != 'S' || networkBuffer[1] != 'M' || networkBuffer[2] != 'A') {
      DEBUG_SERIAL.println("Not an SMA packet?");
      return;
    }
    uint16_t grouplen;
    uint16_t grouptag;
    uint8_t *offset = networkBuffer + 4;
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
        if ((config.smaId != "") && (config.smaId.toInt() != serial)) {
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
    } while (grouplen > 0 && offset + 4 < networkBuffer + rSize);
  }
}

void parseSHRDZM() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int rSize = Udp.read(networkBuffer, 1024);
    networkBuffer[rSize] = 0;
    deserializeJson(globalJsonDoc, networkBuffer);
    if (globalJsonDoc["data"]["16.7.0"].is<JsonVariant>()) {
      double power = globalJsonDoc["data"]["16.7.0"];
      setPowerData(power);
    }
    if (globalJsonDoc["data"]["1.8.0"].is<JsonVariant>() && globalJsonDoc["data"]["2.8.0"].is<JsonVariant>()) {
      double energyIn = 0.001 * globalJsonDoc["data"]["1.8.0"].as<double>();
      double energyOut = 0.001 * globalJsonDoc["data"]["2.8.0"].as<double>();
      setEnergyData(energyIn, energyOut);
    }
    globalJsonDoc.clear();
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
  DEBUG_SERIAL.println("DEBUG: parseSUNSPEC() called, server=" + config.mqttServer);
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
  
  if (config.mqttServer == "" || config.mqttServer.length() == 0) {
    return;
  }
  modbus_ip.fromString(config.mqttServer.c_str());
  if (!modbus1.isConnected(modbus_ip)) {
    modbus1.connect(modbus_ip, config.mqttPortInt);
  } else {
    uint16_t transaction = modbus1.readHreg(modbus_ip, SUNSPEC_BASE, (uint16_t*) &modbus_result[0], 64, nullptr, config.modbusDeviceId);
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
  DEBUG_SERIAL.println("DEBUG: queryHTTP() called, server=" + config.mqttServer);
  String serverAddr = config.mqttServer;
  serverAddr.trim();
  if (serverAddr.length() == 0 || serverAddr == "http://" || serverAddr == "https://") {
    DEBUG_SERIAL.println("HTTP server not configured or invalid - skipping HTTP query");
    return;
  }
  DEBUG_SERIAL.println("Querying HTTP source");
  http.begin(wifi_client, serverAddr.c_str());
  http.GET();
  deserializeJson(globalJsonDoc, http.getStream());
  if (config.powerPath == "") {
    DEBUG_SERIAL.println("HTTP query: no JSONPath for power data provided");
  } else {
    setJsonPathPower(globalJsonDoc);
  }
  http.end();
  globalJsonDoc.clear();
}


const char CONFIG_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<title>E2S Configuration</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 20px; }
h2 { text-align: center; color: #333; }
form { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 600px; margin: 0 auto; }
label { display: block; margin-bottom: 5px; font-weight: bold; color: #555; }
input[type=text], input[type=password], select { width: 100%; padding: 8px; margin-bottom: 15px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
.btn { padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; display: block; width: 100%; }
.btn-save { background-color: #4CAF50; color: white; }
.note { font-size: 0.9em; color: #777; text-align: center; margin-top: 20px; }
fieldset { border: 1px solid #ddd; border-radius: 4px; padding: 15px; margin-bottom: 20px; }
legend { font-weight: bold; color: #333; padding: 0 5px; }
</style>
</head>
<body>
<h2>Energy2Shelly Configuration</h2>
<form method='POST' action='/save'>
  <fieldset>
    <legend>General Settings</legend>
    <label for="inputType">Data Source Type</label>
    <select id="inputType" name="inputType">
      <option value="MQTT" %s_MQTT%>MQTT</option>
      <option value="HTTP" %s_HTTP%>Generic HTTP</option>
      <option value="SMA" %s_SMA%>SMA Multicast</option>
      <option value="SHRDZM" %s_SHRDZM%>SHRDZM UDP</option>
      <option value="SUNSPEC" %s_SUNSPEC%>Modbus TCP (SUNSPEC)</option>
    </select>
    <label for="mqttServer">Server / URL</label>
    <input type='text' id='mqttServer' name='mqttServer' value='%v_mqttServer%'>
    <label for="queryPeriod">Query Period (ms)</label>
    <input type='text' id='queryPeriod' name='queryPeriod' value='%v_queryPeriod%'>
    <label for="ledGpio">LED GPIO</label>
    <input type='text' id='ledGpio' name='ledGpio' value='%v_ledGpio%'>
    <label for="ledInverted">Invert LED GPIO</label>
    <select id="ledInverted" name="ledInverted">
      <option value="false" %s_ledInverted_false%>No</option>
      <option value="true" %s_ledInverted_true%>Yes</option>
    </select>
    <label for="shellyMac">Shelly ID (MAC)</label>
    <input type='text' id='shellyMac' name='shellyMac' value='%v_shellyMac%'>
    <label for="shellyPort">Shelly UDP Port</label>
    <input type='text' id='shellyPort' name='shellyPort' value='%v_shellyPort%'>
    <label for="forcePwrDecimals">Force Power Decimals</label>
    <select id="forcePwrDecimals" name="forcePwrDecimals">
      <option value="true" %s_forcePwrDecimals_true%>Yes</option>
      <option value="false" %s_forcePwrDecimals_false%>No</option>
    </select>
    <label for="smaId">SMA Serial Number</label>
    <input type='text' id='smaId' name='smaId' value='%v_smaId%'>
  </fieldset>
  
  <fieldset>
    <legend>MQTT Options</legend>
    <label for="mqttPort">MQTT Port</label>
    <input type='text' id='mqttPort' name='mqttPort' value='%v_mqttPort%'>
    <label for="mqttTopic">MQTT Topic</label>
    <input type='text' id='mqttTopic' name='mqttTopic' value='%v_mqttTopic%'>
    <label for="mqttUser">MQTT User</label>
    <input type='text' id='mqttUser' name='mqttUser' value='%v_mqttUser%'>
    <label for="mqttPasswd">MQTT Password</label>
    <input type='password' id='mqttPasswd' name='mqttPasswd' value='%v_mqttPasswd%'>
  </fieldset>
  
  <fieldset>
    <legend>Modbus TCP Options</legend>
    <label for="modbusDevice">Modbus Device ID</label>
    <input type='text' id='modbusDevice' name='modbusDevice' value='%v_modbusDevice%'>
  </fieldset>

  <fieldset>
    <legend>JSON Paths</legend>
    <label for="powerPath">Total Power Path</label>
    <input type='text' id='powerPath' name='powerPath' value='%v_powerPath%'>
    <label for="pwrExportPath">Export Power Path</label>
    <input type='text' id='pwrExportPath' name='pwrExportPath' value='%v_pwrExportPath%'>
    <label for="powerL1Path">Phase 1 Power Path</label>
    <input type='text' id='powerL1Path' name='powerL1Path' value='%v_powerL1Path%'>
    <label for="powerL2Path">Phase 2 Power Path</label>
    <input type='text' id='powerL2Path' name='powerL2Path' value='%v_powerL2Path%'>
    <label for="powerL3Path">Phase 3 Power Path</label>
    <input type='text' id='powerL3Path' name='powerL3Path' value='%v_powerL3Path%'>
    <label for="energyInPath">Energy In Path</label>
    <input type='text' id='energyInPath' name='energyInPath' value='%v_energyInPath%'>
    <label for="energyOutPath">Energy Out Path</label>
    <input type='text' id='energyOutPath' name='energyOutPath' value='%v_energyOutPath%'>
  </fieldset>

  <button type='submit' class='btn btn-save'>Save Configuration</button>
</form>
<p class="note">Device will restart after saving.</p>
</body>
</html>
)=====";

String processor(const String& var) {
  // General
  if (var == "v_mqttServer") return config.mqttServer;
  if (var == "v_queryPeriod") return config.queryPeriod;
  if (var == "v_ledGpio") return String(config.ledGpioInt);
  if (var == "v_shellyMac") return config.shellyMac;
  if (var == "v_shellyPort") return config.shellyPort;
  if (var == "v_smaId") return config.smaId;

  // Booleans for selects
  if (var == "s_ledInverted_true") return config.ledInverted ? "selected" : "";
  if (var == "s_ledInverted_false") return !config.ledInverted ? "selected" : "";
  if (var == "s_forcePwrDecimals_true") return config.forcePwrDecimals ? "selected" : "";
  if (var == "s_forcePwrDecimals_false") return !config.forcePwrDecimals ? "selected" : "";

  // Data Source Type select
  if (var == "s_MQTT") return (config.inputType == "MQTT") ? "selected" : "";
  if (var == "s_HTTP") return (config.inputType == "HTTP") ? "selected" : "";
  if (var == "s_SMA") return (config.inputType == "SMA") ? "selected" : "";
  if (var == "s_SHRDZM") return (config.inputType == "SHRDZM") ? "selected" : "";
  if (var == "s_SUNSPEC") return (config.inputType == "SUNSPEC") ? "selected" : "";

  // MQTT
  if (var == "v_mqttPort") return config.mqttPort;
  if (var == "v_mqttTopic") return config.mqttTopic;
  if (var == "v_mqttUser") return config.mqttUser;
  if (var == "v_mqttPasswd") return config.mqttPasswd;

  // Modbus
  if (var == "v_modbusDevice") return config.modbusDevice;
  
  // JSON Paths
  if (var == "v_powerPath") return config.powerPath;
  if (var == "v_pwrExportPath") return config.pwrExportPath;
  if (var == "v_powerL1Path") return config.powerL1Path;
  if (var == "v_powerL2Path") return config.powerL2Path;
  if (var == "v_powerL3Path") return config.powerL3Path;
  if (var == "v_energyInPath") return config.energyInPath;
  if (var == "v_energyOutPath") return config.energyOutPath;
  
  return String();
}

void handleConfig(AsyncWebServerRequest *request) {
  request->send(200, "text/html", CONFIG_page, processor);
}

void handleSave(AsyncWebServerRequest *request) {
  // Helper lambda to get a parameter value
  auto getParam = [&](const char* name) {
    if (request->hasParam(name, true)) {
      return request->getParam(name, true)->value();
    }
    return String();
  };

  // Update config struct from form parameters
  config.inputType = getParam("inputType");
  config.mqttServer = getParam("mqttServer");
  config.queryPeriod = getParam("queryPeriod");
  String ledGpioStr = getParam("ledGpio");
  config.ledGpioInt = ledGpioStr.toInt();
  String ledInvertedStr = getParam("ledInverted");
  config.ledInverted = (ledInvertedStr == "true");
  config.shellyMac = getParam("shellyMac");
  config.shellyPort = getParam("shellyPort");
  String forcePwrDecimalsStr = getParam("forcePwrDecimals");
  config.forcePwrDecimals = (forcePwrDecimalsStr == "true");
  config.smaId = getParam("smaId");
  config.mqttPort = getParam("mqttPort");
  config.mqttTopic = getParam("mqttTopic");
  config.mqttUser = getParam("mqttUser");
  config.mqttPasswd = getParam("mqttPasswd");
  config.modbusDevice = getParam("modbusDevice");
  config.powerPath = getParam("powerPath");
  config.pwrExportPath = getParam("pwrExportPath");
  config.powerL1Path = getParam("powerL1Path");
  config.powerL2Path = getParam("powerL2Path");
  config.powerL3Path = getParam("powerL3Path");
  config.energyInPath = getParam("energyInPath");
  config.energyOutPath = getParam("energyOutPath");

  // Save all settings to Preferences
  preferences.begin("e2s_config", false);
  preferences.putString("input_type", config.inputType);
  preferences.putString("mqtt_server", config.mqttServer);
  preferences.putString("query_period", config.queryPeriod);
  preferences.putString("led_gpio", ledGpioStr);
  preferences.putString("led_gpio_i", ledInvertedStr);
  preferences.putString("shelly_mac", config.shellyMac);
  preferences.putString("shelly_port", config.shellyPort);
  preferences.putBool("force_pwr_decimals", config.forcePwrDecimals);
  preferences.putString("sma_id", config.smaId);
  preferences.putString("mqtt_port", config.mqttPort);
  preferences.putString("mqtt_topic", config.mqttTopic);
  preferences.putString("mqtt_user", config.mqttUser);
  preferences.putString("mqtt_passwd", config.mqttPasswd);
  preferences.putString("modbus_dev", config.modbusDevice);
  preferences.putString("power_path", config.powerPath);
  preferences.putString("pwr_export_path", config.pwrExportPath);
  preferences.putString("power_l1_path", config.powerL1Path);
  preferences.putString("power_l2_path", config.powerL2Path);
  preferences.putString("power_l3_path", config.powerL3Path);
  preferences.putString("energy_in_path", config.energyInPath);
  preferences.putString("energy_out_path", config.energyOutPath);
  preferences.end();

  String response = "<html><head><title>Settings Saved</title><meta http-equiv='refresh' content='5;url=/'></head><body>";
  response += "<h1>Settings Saved</h1>";
  response += "<p>The device will now restart to apply the changes.</p>";
  response += "<p>You will be redirected to the home page in 5 seconds. If not, please <a href='/'>click here</a>.</p>";
  response += "</body></html>";
  request->send(200, "text/html", response);
  
  shouldReboot = true;
}

void WifiManagerSetup() {
  DEBUG_SERIAL.println("DEBUG: WifiManagerSetup() START");
  
  // Set Shelly ID to ESP's MAC address by default
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macBuffer[13];
  sprintf(macBuffer, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  config.shellyMac = macBuffer;

  DEBUG_SERIAL.println("DEBUG: About to call preferences.begin()");
  preferences.begin("e2s_config", false);
  DEBUG_SERIAL.println("DEBUG: preferences.begin() done, now loading values");
  
  // Load all preferences first - avoid any network operations during loading
  config.inputType = preferences.getString("input_type", config.inputType);
  DEBUG_SERIAL.println("DEBUG: loaded inputType=" + config.inputType);
  config.mqttServer = preferences.getString("mqtt_server", config.mqttServer);
  DEBUG_SERIAL.println("DEBUG: loaded mqttServer=" + config.mqttServer);
  config.queryPeriod = preferences.getString("query_period", config.queryPeriod);
  String ledGpio = preferences.getString("led_gpio", "");
  config.ledGpioInt = ledGpio.toInt();
  String ledInvert = preferences.getString("led_gpio_i", "false");
  config.ledInverted = (ledInvert == "true");
  config.shellyMac = preferences.getString("shelly_mac", config.shellyMac);
  config.mqttPort = preferences.getString("mqtt_port", config.mqttPort);
  config.mqttPortInt = config.mqttPort.toInt();
  config.mqttTopic = preferences.getString("mqtt_topic", config.mqttTopic);
  config.mqttUser = preferences.getString("mqtt_user", config.mqttUser);
  config.mqttPasswd = preferences.getString("mqtt_passwd", config.mqttPasswd);
  config.modbusDevice = preferences.getString("modbus_dev", config.modbusDevice);
  config.modbusDeviceId = config.modbusDevice.toInt();
  config.powerPath = preferences.getString("power_path", config.powerPath);
  config.pwrExportPath = preferences.getString("pwr_export_path", config.pwrExportPath);
  config.powerL1Path = preferences.getString("power_l1_path", config.powerL1Path);
  config.powerL2Path = preferences.getString("power_l2_path", config.powerL2Path);
  config.powerL3Path = preferences.getString("power_l3_path", config.powerL3Path);
  config.energyInPath = preferences.getString("energy_in_path", config.energyInPath);
  config.energyOutPath = preferences.getString("energy_out_path", config.energyOutPath);
  config.shellyPort = preferences.getString("shelly_port", config.shellyPort);
  config.shellyPortInt = config.shellyPort.toInt();
  config.forcePwrDecimals = preferences.getBool("force_pwr_decimals", true);
  config.smaId = preferences.getString("sma_id", config.smaId);
  
  WiFiManagerParameter custom_section1("<h3>General settings</h3>");
  WiFiManagerParameter custom_input_type("type", "<b>Data source</b><br><code>MQTT</code> for MQTT<br><code>HTTP</code> for generic HTTP<br><code>SMA</code> for SMA EM/HM multicast<br><code>SHRDZM</code> for SHRDZM UDP data<br><code>SUNSPEC</code> for Modbus TCP SUNSPEC data", config.inputType.c_str(), 40);
  WiFiManagerParameter custom_mqtt_server("server", "<b>Server</b><br>MQTT Server IP, query url for generic HTTP or Modbus TCP server IP for SUNSPEC", config.mqttServer.c_str(), 80);
  WiFiManagerParameter custom_mqtt_port("port", "<b>Port</b><br> for MQTT or Modbus TCP (SUNSPEC)", config.mqttPort.c_str(), 6);
  WiFiManagerParameter custom_query_period("query_period", "<b>Query period</b><br>for generic HTTP and SUNSPEC, in milliseconds", config.queryPeriod.c_str(), 10);
  WiFiManagerParameter custom_led_gpio("led_gpio", "<b>GPIO</b><br>of internal LED", ledGpio.c_str(), 3);
  WiFiManagerParameter custom_led_gpio_i("led_gpio_i", "<b>GPIO is inverted</b><br><code>true</code> or <code>false</code>", ledInvert.c_str(), 6);
  WiFiManagerParameter custom_shelly_mac("mac", "<b>Shelly ID</b><br>12 char hexadecimal, defaults to MAC address of ESP", config.shellyMac.c_str(), 13);
  WiFiManagerParameter custom_shelly_port("shelly_port", "<b>Shelly UDP port</b><br><code>1010</code> for old Marstek FW, <code>2220</code> for new Marstek FW v226+/v108+", config.shellyPort.c_str(), 6);
  WiFiManagerParameter custom_force_pwr_decimals("force_pwr_decimals", "<b>Force decimals numbers for Power values</b><br><code>true</code> to fix Marstek bug", (config.forcePwrDecimals ? "true" : "false"), 6);
  WiFiManagerParameter custom_sma_id("sma_id", "<b>SMA serial number</b><br>optional serial number if you have more than one SMA EM/HM in your network", config.smaId.c_str(), 16);
  WiFiManagerParameter custom_section2("<hr><h3>MQTT options</h3>");
  WiFiManagerParameter custom_mqtt_topic("topic", "<b>MQTT Topic</b>", config.mqttTopic.c_str(), 90);
  WiFiManagerParameter custom_mqtt_user("user", "<b>MQTT user</b><br>optional", config.mqttUser.c_str(), 40);
  WiFiManagerParameter custom_mqtt_passwd("passwd", "<b>MQTT password</b><br>optional", config.mqttPasswd.c_str(), 40);
  WiFiManagerParameter custom_section3("<hr><h3>Modbus TCP options</h3>");
  WiFiManagerParameter custom_modbus_dev("modbus_dev", "<b>Modbus device ID</b><br><code>71</code> for Kostal SEM", config.modbusDevice.c_str(), 60);
  WiFiManagerParameter custom_section4("<hr><h3>JSON paths for MQTT and generic HTTP</h3>");
  WiFiManagerParameter custom_power_path("power_path", "<b>Total power JSON path</b><br>e.g. <code>ENERGY.Power</code> or <code>TRIPHASE</code> for tri-phase data", config.powerPath.c_str(), 60);
  WiFiManagerParameter custom_pwr_export_path("pwr_export_path", "<b>Export power JSON path</b><br>Optional, for net calc (e.g. \"i-e\"", config.pwrExportPath.c_str(), 60);
  WiFiManagerParameter custom_power_l1_path("power_l1_path", "<b>Phase 1 power JSON path</b><br>optional", config.powerL1Path.c_str(), 60);
  WiFiManagerParameter custom_power_l2_path("power_l2_path", "<b>Phase 2 power JSON path</b><br>Phase 2 power JSON path<br>optional", config.powerL2Path.c_str(), 60);
  WiFiManagerParameter custom_power_l3_path("power_l3_path", "<b>Phase 3 power JSON path</b><br>Phase 3 power JSON path<br>optional", config.powerL3Path.c_str(), 60);
  WiFiManagerParameter custom_energy_in_path("energy_in_path", "<b>Energy from grid JSON path</b><br>e.g. <code>ENERGY.Grid</code>", config.energyInPath.c_str(), 60);
  WiFiManagerParameter custom_energy_out_path("energy_out_path", "<b>Energy to grid JSON path</b><br>e.g. <code>ENERGY.FeedIn</code>", config.energyOutPath.c_str(), 60);

  WiFiManager wifiManager;
  if (!DEBUG) {
    wifiManager.setDebugOutput(false);
  }
  wifiManager.setTitle("Energy2Shelly for ESP");
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_section1);
  wifiManager.addParameter(&custom_input_type);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_query_period);
  wifiManager.addParameter(&custom_led_gpio);
  wifiManager.addParameter(&custom_led_gpio_i);
  wifiManager.addParameter(&custom_shelly_mac);
  wifiManager.addParameter(&custom_shelly_port);
  wifiManager.addParameter(&custom_force_pwr_decimals);
  wifiManager.addParameter(&custom_sma_id);
  wifiManager.addParameter(&custom_section2);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_topic);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_passwd);
  wifiManager.addParameter(&custom_section3);
  wifiManager.addParameter(&custom_modbus_dev);
  wifiManager.addParameter(&custom_section4);
  wifiManager.addParameter(&custom_power_path);
  wifiManager.addParameter(&custom_pwr_export_path);
  wifiManager.addParameter(&custom_power_l1_path);
  wifiManager.addParameter(&custom_power_l2_path);
  wifiManager.addParameter(&custom_power_l3_path);
  wifiManager.addParameter(&custom_energy_in_path);
  wifiManager.addParameter(&custom_energy_out_path);
  

  DEBUG_SERIAL.println("DEBUG: About to call wifiManager.autoConnect()");
  // Only attempt WiFiManager if configuration is invalid or incomplete
  bool needsConfiguration = (config.inputType == "" || config.mqttServer == "");
  if (needsConfiguration) {
    DEBUG_SERIAL.println("DEBUG: Configuration incomplete, running WiFiManager.autoConnect()");
    if (!wifiManager.autoConnect("Energy2Shelly")) {
      DEBUG_SERIAL.println("failed to connect and hit timeout");
      delay(3000);
      ESP.restart();
      delay(5000);
    }
    DEBUG_SERIAL.println("DEBUG: wifiManager.autoConnect() completed");
  } else {
    DEBUG_SERIAL.println("DEBUG: Configuration complete, attempting normal WiFi connection");
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    int timeout = 20;
    while (WiFi.status() != WL_CONNECTED && timeout--) {
      delay(500);
      DEBUG_SERIAL.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
      DEBUG_SERIAL.println("\nFailed to connect, entering WiFiManager");
      if (!wifiManager.autoConnect("Energy2Shelly")) {
        DEBUG_SERIAL.println("failed to connect and hit timeout");
        delay(3000);
        ESP.restart();
      }
    }
  }
  DEBUG_SERIAL.println("connected");

  //read updated parameters
  config.inputType = custom_input_type.getValue();
  config.mqttServer = custom_mqtt_server.getValue();
  config.mqttServer.trim();
  config.mqttPort = custom_mqtt_port.getValue();
  config.mqttPortInt = config.mqttPort.toInt();
  config.queryPeriod = custom_query_period.getValue();
  config.queryPeriodMs = config.queryPeriod.toInt();
  config.ledGpioInt = String(custom_led_gpio.getValue()).toInt();
  String ledInvStr = custom_led_gpio_i.getValue();
  config.ledInverted = (ledInvStr == "true");
  config.shellyMac = custom_shelly_mac.getValue();
  config.mqttTopic = custom_mqtt_topic.getValue();
  config.mqttUser = custom_mqtt_user.getValue();
  config.mqttPasswd = custom_mqtt_passwd.getValue();
  config.modbusDevice = custom_modbus_dev.getValue();
  config.modbusDeviceId = config.modbusDevice.toInt();
  config.powerPath = custom_power_path.getValue();
  config.pwrExportPath = custom_pwr_export_path.getValue();
  config.powerL1Path = custom_power_l1_path.getValue();
  config.powerL2Path = custom_power_l2_path.getValue();
  config.powerL3Path = custom_power_l3_path.getValue();
  config.energyInPath = custom_energy_in_path.getValue();
  config.energyOutPath = custom_energy_out_path.getValue();
  config.shellyPort = custom_shelly_port.getValue();
  config.shellyPortInt = config.shellyPort.toInt();
  String forcePwrStr(custom_force_pwr_decimals.getValue());
  config.forcePwrDecimals = (forcePwrStr == "true");
  config.smaId = custom_sma_id.getValue();

  DEBUG_SERIAL.println("The values in the preferences are: ");
  DEBUG_SERIAL.println("\tinput_type : " + config.inputType);
  DEBUG_SERIAL.println("\tmqtt_server : " + config.mqttServer);
  DEBUG_SERIAL.println("\tmqtt_port : " + config.mqttPort);
  DEBUG_SERIAL.println("\tquery_period : " + config.queryPeriod);
  DEBUG_SERIAL.println("\tled_gpio : " + String(config.ledGpioInt));
  DEBUG_SERIAL.println("\tled_gpio_i : " + String(config.ledInverted));
  DEBUG_SERIAL.println("\tshelly_mac : " + config.shellyMac);
  DEBUG_SERIAL.println("\tmqtt_topic : " + config.mqttTopic);
  DEBUG_SERIAL.println("\tmqtt_user : " + config.mqttUser);
  DEBUG_SERIAL.println("\tmqtt_passwd : " + config.mqttPasswd);
  DEBUG_SERIAL.println("\tmodbus_dev : " + config.modbusDevice);
  DEBUG_SERIAL.println("\tpower_path : " + config.powerPath);
  DEBUG_SERIAL.println("\tpwr_export_path : " + config.pwrExportPath);
  DEBUG_SERIAL.println("\tpower_l1_path : " + config.powerL1Path);
  DEBUG_SERIAL.println("\tpower_l2_path : " + config.powerL2Path);
  DEBUG_SERIAL.println("\tpower_l3_path : " + config.powerL3Path);
  DEBUG_SERIAL.println("\tenergy_in_path : " + config.energyInPath);
  DEBUG_SERIAL.println("\tenergy_out_path : " + config.energyOutPath);
  DEBUG_SERIAL.println("\tshelly_port : " + config.shellyPort);
  DEBUG_SERIAL.println("\tforce_pwr_decimals : " + String(config.forcePwrDecimals));
  DEBUG_SERIAL.println("\tsma_id : " + config.smaId);

  if (config.inputType == "SMA") {
    dataSMA = true;
    DEBUG_SERIAL.println("Enabling SMA Multicast data input");
  } else if (config.inputType == "SHRDZM") {
    dataSHRDZM = true;
    DEBUG_SERIAL.println("Enabling SHRDZM UDP data input");
  } else if (config.inputType == "HTTP") {
    if (config.mqttServer != "" && config.mqttServer.length() > 0) {
      dataHTTP = true;
      DEBUG_SERIAL.println("Enabling generic HTTP data input");
    } else {
      DEBUG_SERIAL.println("HTTP server not configured - disabling HTTP data input");
    }
  } else if (config.inputType == "SUNSPEC") {
    if (config.mqttServer != "" && config.mqttServer.length() > 0) {
      dataSUNSPEC = true;
      DEBUG_SERIAL.println("Enabling SUNSPEC data input");
    } else {
      DEBUG_SERIAL.println("SUNSPEC server not configured - disabling SUNSPEC data input");
    }
  }
  else {
    if (config.mqttServer != "" && config.mqttServer.length() > 0) {
      dataMQTT = true;
      DEBUG_SERIAL.println("Enabling MQTT data input");
    } else {
      DEBUG_SERIAL.println("MQTT server not configured - disabling MQTT data input");
    }
  }

  led_i = config.ledInverted;

  if (shouldSaveConfig) {
    DEBUG_SERIAL.println("saving config");
    preferences.putString("input_type", config.inputType);
    preferences.putString("mqtt_server", config.mqttServer);
    preferences.putString("mqtt_port", config.mqttPort);
    preferences.putString("query_period", config.queryPeriod);
    preferences.putString("led_gpio", String(config.ledGpioInt));
    preferences.putString("led_gpio_i", config.ledInverted ? "true" : "false");
    preferences.putString("shelly_mac", config.shellyMac);
    preferences.putString("mqtt_topic", config.mqttTopic);
    preferences.putString("mqtt_user", config.mqttUser);
    preferences.putString("mqtt_passwd", config.mqttPasswd);
    preferences.putString("modbus_dev", config.modbusDevice);
    preferences.putString("power_path", config.powerPath);
    preferences.putString("pwr_export_path", config.pwrExportPath);
    preferences.putString("power_l1_path", config.powerL1Path);
    preferences.putString("power_l2_path", config.powerL2Path);
    preferences.putString("power_l3_path", config.powerL3Path);
    preferences.putString("energy_in_path", config.energyInPath);
    preferences.putString("energy_out_path", config.energyOutPath);
    preferences.putString("shelly_port", config.shellyPort);
    preferences.putBool("force_pwr_decimals", config.forcePwrDecimals);
    preferences.putString("sma_id", config.smaId);
    wifiManager.reboot();
  }
  DEBUG_SERIAL.println("local ip");
  DEBUG_SERIAL.println(WiFi.localIP());
}

void setup(void) {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("\n\n=== SETUP START ===");
  DEBUG_SERIAL.println("About to call WifiManagerSetup()");
  WifiManagerSetup();
  DEBUG_SERIAL.println("WifiManagerSetup() completed");

  led = config.ledGpioInt;

  if (led > 0) {
    pinMode(led, OUTPUT);
    if (led_i) {
      digitalWrite(led, LOW);
    } else {
      digitalWrite(led, HIGH);
    }
  }

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

  server.on("/config", HTTP_GET, handleConfig);
  server.on("/save", HTTP_POST, handleSave);

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
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

  server.on("/rpc/EM.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/EMData.GetStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMDataGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/EM.GetConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/Shelly.GetDeviceInfo", HTTP_GET, [](AsyncWebServerRequest *request) {
    GetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc", HTTP_POST, [](AsyncWebServerRequest *request) {
    GetDeviceInfo();
    rpcWrapper();
    request->send(200, "application/json", serJsonResponse);
  });

  webSocket.onEvent(webSocketEvent);
  server.addHandler(&webSocket);
  server.begin();

  // Set up RPC over UDP for Marstek users
  UdpRPC.begin(config.shellyPortInt); 

  // Set up MQTT
  if (dataMQTT) {
    String server = config.mqttServer;
    server.trim();
    // Only setup MQTT if server is configured
    if (server.length() > 0) {
      mqtt_client.setBufferSize(2048);
      // Parse IP address to avoid DNS lookups
      IPAddress mqttIP;
      if (mqttIP.fromString(server)) {
        // It's an IP address, use it directly
        mqtt_client.setServer(mqttIP, config.mqttPortInt);
      } else {
        // It's a hostname, pass as string (will do DNS lookup)
        mqtt_client.setServer(server.c_str(), config.mqttPortInt);
      }
      mqtt_client.setCallback(mqtt_callback);
      mqtt_configured = true;  // Mark MQTT as properly configured
    } else {
      DEBUG_SERIAL.println("MQTT server not configured - skipping MQTT setup");
      dataMQTT = false;
      mqtt_configured = false;
    }
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
    if (config.mqttServer == "" || config.mqttServer.length() == 0) {
      DEBUG_SERIAL.println("SUNSPEC server not configured - skipping SUNSPEC setup");
      dataSUNSPEC = false;
    } else {
      modbus1.client();
      modbus_ip.fromString(config.mqttServer.c_str());
      if (!modbus1.isConnected(modbus_ip)) {  // reuse mqtt server address for modbus address
        modbus1.connect(modbus_ip, config.mqttPortInt);
        Serial.println("Trying to connect SUNSPEC powermeter data");
      }
    }
  }

  // Set Up HTTP query
  if (dataHTTP) {
    period = config.queryPeriodMs;
    startMillis = millis();
    http.useHTTP10(true);
  }

  // Set up mDNS responder
  config.shellyName += config.shellyMac;
  if (!MDNS.begin(config.shellyName.c_str())) {
    DEBUG_SERIAL.println("Error setting up MDNS responder!");
  }

#ifdef ESP32
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("shelly", "tcp", 80);
  mdns_txt_item_t serviceTxtData[4] = {
    { "fw_id", config.shellyFwId.c_str() },
    { "arch", "esp8266" },
    { "id", config.shellyName.c_str() },
    { "gen", config.shellyGen.c_str() }
  };
  mdns_service_instance_name_set("_http", "_tcp", config.shellyName.c_str());
  mdns_service_txt_set("_http", "_tcp", serviceTxtData, 4);
  mdns_service_instance_name_set("_shelly", "_tcp", config.shellyName.c_str());
  mdns_service_txt_set("_shelly", "_tcp", serviceTxtData, 4);
#else
  hMDNSService = MDNS.addService(0, "http", "tcp", 80);
  hMDNSService2 = MDNS.addService(0, "shelly", "tcp", 80);
  if (hMDNSService) {
    MDNS.setServiceName(hMDNSService, config.shellyName.c_str());
    MDNS.addServiceTxt(hMDNSService, "fw_id", config.shellyFwId.c_str());
    MDNS.addServiceTxt(hMDNSService, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService, "id", config.shellyName.c_str());
    MDNS.addServiceTxt(hMDNSService, "gen", config.shellyGen.c_str());
  }
  if (hMDNSService2) {
    MDNS.setServiceName(hMDNSService2, config.shellyName.c_str());
    MDNS.addServiceTxt(hMDNSService2, "fw_id", config.shellyFwId.c_str());
    MDNS.addServiceTxt(hMDNSService2, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService2, "id", config.shellyName.c_str());
    MDNS.addServiceTxt(hMDNSService2, "gen", config.shellyGen.c_str());
  }
#endif
  DEBUG_SERIAL.println("mDNS responder started");
}

void loop() {
  if (shouldReboot) {
    delay(1000); // Give a moment for the response to be sent
    ESP.restart();
  }
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
  if (mqtt_configured) {  // Only use mqtt_client if it was properly configured
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
