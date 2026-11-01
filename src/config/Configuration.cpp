#include "Configuration.h"
#include "../utils/utils.h"
#include <LittleFS.h>
#define FS_INSTANCE LittleFS
const char* configPath = "/config.json";
unsigned long startMillis = 0;
unsigned long currentMillis;
// for time synchronization
time_t now;
tm timeinfo;
JsonDocument config_doc;

// ============================================================================
// CONFIGURATION VARIABLES (stored in Preferences)
// ============================================================================

// Data source and server settings

String shelly_name;

uint16_t sunspec_port_int; // default port
uint8_t modbusdev_int;     // default device id for KSEM

char tibber_rpc[2][26] = {"/data.json?node_id=","/node_data.json?node_id="}; // fixed rpc path

unsigned long ledOffTime = 0;
uint8_t led = 0;
bool led_i = false;
const uint8_t ledblinkduration = 50;

// SMA Multicast IP and Port
unsigned int multicastPort = 9522; // local port to listen on
IPAddress multicastIP(239, 12, 255, 254);

// MODBUS settings
IPAddress modbus_ip;
ModbusIP modbus1;
int16_t modbus_result[256];

// Default electrical values and offset
const uint8_t defaultVoltage = 230;
const uint8_t defaultFrequency = 50;
const uint8_t defaultPowerFactor = 1;
double offsetPerPhase = 0;

// RPC settings
int rpcId = 1;
char rpcUser[20] = "user_1";

// flags for saving/resetting WifiManager data
bool shouldSaveConfig = false;
bool shouldResetConfig = false;
bool shouldupdate = false;

// flags for data sources
bool dataMQTT = false;
bool dataSMA = false;
bool dataSHRDZM = false;
bool dataHTTP = false;
bool dataSUNSPEC = false;
bool dataTIBBERPULSE = false;
bool dataRCT = false;
bool dataSMLSERIAL = false;

Preferences preferences;

// ============================================================================
// NETWORK OBJECTS
// ============================================================================

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
AsyncWebServer server(80);
AsyncWebSocket webSocket("/rpc");
AsyncWebSocket wsConsole("/consolews");
WiFiUDP Udp;
HTTPClient http;
WiFiUDP UdpRPC;
uint16_t wifi_reconnect_attempts = 0;

// ============================================================================
// MDNS RESPONDER HANDLES (ESP8266 only)
// ============================================================================

#ifndef ESP32
MDNSResponder::hMDNSService hMDNSService = 0;  // handle of the http service in the MDNS responder
MDNSResponder::hMDNSService hMDNSService2 = 0; // handle of the shelly service in the MDNS responder
#endif

// Blink LED handlers
void blinkled(int duration)
{
  if (led > 0)
  {
    if (led_i)
    {
      digitalWrite(led, HIGH);
    }
    else
    {
      digitalWrite(led, LOW);
    }
    ledOffTime = millis() + duration;
  }
}

void handleblinkled()
{
  if (led > 0)
  {
    if (ledOffTime > 0 && millis() > ledOffTime)
    {
      if (led_i)
      {
        digitalWrite(led, LOW);
      }
      else
      {
        digitalWrite(led, HIGH);
      }
      ledOffTime = 0;
    }
  }
}


void interprete_parameters()
{
  // Interpret the loaded parameters and set flags accordingly
  input_type_enum current_input_type = static_cast<input_type_enum>(config_doc["basic"]["input_type"].as<uint32_t>());

  switch (current_input_type) {
    case input_type_enum::SMA:
        dataSMA = true;
        DEBUG_SERIAL.println(F("Enabling SMA Multicast data input"));
        break;
    case input_type_enum::SHRDZM:
        dataSHRDZM = true;
        DEBUG_SERIAL.println(F("Enabling SHRDZM UDP data input"));
        break;
    case input_type_enum::HTTP:
        dataHTTP = true;
        DEBUG_SERIAL.println(F("Enabling generic HTTP data input"));
        break;
    case input_type_enum::SUNSPEC:
        dataSUNSPEC = true;
        modbusdev_int = config_doc["sunspec"]["modbus_dev"].as<uint8_t>();
        sunspec_port_int = config_doc["sunspec"]["modbus_port"].as<uint16_t>();
        modbus_ip.fromString(config_doc["sunspec"]["modbus_ip"].as<const char*>()); // Todo, replace by DNS resolver for hostnames, currently only IP addresses are supported
        DEBUG_SERIAL.println(F("Enabling SUNSPEC data input"));
        break;
   case input_type_enum::TIBBERPULSE:
        dataTIBBERPULSE = true;
        DEBUG_SERIAL.println(F("Enabling TIBBERPULSE data input"));
        break;
   case input_type_enum::SMLSERIAL:
        dataSMLSERIAL = true;
        DEBUG_SERIAL.println(F("Enabling SML serial (UART) data input"));
        break;
   case input_type_enum::RCT:
        dataRCT = true;
        DEBUG_SERIAL.println(F("Enabling RCT Power data input"));
        break;
   case input_type_enum::MQTT:
        dataMQTT = true;
        DEBUG_SERIAL.println(F("Enabling MQTT data input"));
        break;
   case input_type_enum::NONE:
   default:
        DEBUG_SERIAL.println(F("None"));
  }
 
  if (led_gpio_i!=NULL && strlen(led_gpio_i)>=4 && strcmp(led_gpio_i, "true") == 0)
  {
    led_i = true;
  }
  else
  {
    led_i = false;
  }

  String temp=config_doc["console"]["web_console"]|"";
  if (strlen(temp.c_str())>=4 && strncmp(temp.c_str(), "true",4) == 0) {
    enableConsoleOutputWeb=true;
  }
  else {
     enableConsoleOutputWeb=false;
  }
 
  config_doc["console"]["serial_console_time"]  =    config_doc["console"]["serial_console_time"].as<int>() ? config_doc["console"]["serial_console_time"].as<int>() : (int)1;
  if (config_doc["console"]["serial_console_time"].as<int>()>0) {
     enableConsoleOutputSerial=true;
  }
  else {
    enableConsoleOutputSerial=false;
  }


  offsetPerPhase = power_offset / 3.0; // distribute offset equally across phases

  String name_str = config_doc["shelly"]["shelly_name"] | "";
  String mac_str  = config_doc["shelly"]["shelly_mac"] | "";
  shelly_name = name_str + mac_str;

 if (shelly_name.length() > 128) {
    shelly_name = shelly_name.substring(0, 128);
 }



}

void erase_preferences()
{
  preferences.begin("e2s_config", false);
  preferences.clear();
  preferences.end();
}




String json_prefs_string;
JsonDocument update_config_doc;


void mergeJson(JsonVariant dst, JsonVariantConst src) {
  if (src.is<JsonObjectConst>()) {
    for (JsonPairConst kv : src.as<JsonObjectConst>()) {
      if (dst[kv.key()].is<JsonObject>()) {
        mergeJson(dst[kv.key()], kv.value());
      } else {
        dst[kv.key()] = kv.value();
      }
    }
  } else {
    dst.set(src);
  }
}


bool save_parameters() {
  File configFile = FS_INSTANCE.open(configPath, "w");
  if (!configFile) {
    DEBUG_SERIAL.println(F("Error: Could not open file"));
    return false;
  }

  if (serializeJson(config_doc, configFile) == 0) {
    DEBUG_SERIAL.println(F("Error: not possible to serialize"));
    configFile.close();
    return false;
  }

  configFile.close();
  return true;
}

const char DEFAULT_CONFIG_JSON[] PROGMEM = R"=====(
{
  "basic": {
    "ntp_server": "de.pool.ntp.org",
    "timezone": "CET-1CEST,M3.5.0/2,M10.5.0/3",
    "phase_number": 3,
    "input_type": 0
  },
  "shelly": {
    "shelly_port": 2220,
    "shelly_gen": 2,
    "shelly_fw_id": "20241011-114455/1.4.4-g6d2a586",
    "shelly_name": "shellypro3em-"
  },
  "sunspec": { "query_period": 5000 },
  "tibber": { "query_period": 5000 },
  "rct": { "query_period": 5000 },
  "http": { "query_period": 5000 },
  "smlserial": {
    "rx_pin": 3,
    "baud": 9600,
    "invert": "false"
  },
  "console": {
    "web_console": "true",
    "serial_console_time": 1
  }
}
)=====";

void load_parameters() {
  config_doc.clear();
  DeserializationError defaultError = deserializeJson(config_doc, (const __FlashStringHelper*)DEFAULT_CONFIG_JSON);
  if (defaultError) {
    DEBUG_SERIAL.println(F("Error during default config initialization!"));
  }

  if (FS_INSTANCE.exists(configPath)) {
    File configFile = FS_INSTANCE.open(configPath, "r");
    if (configFile) {
      JsonDocument fs_doc;
      DeserializationError fsError = deserializeJson(fs_doc, configFile);
      configFile.close();

      if (fsError == DeserializationError::Ok) {
        mergeJson(config_doc.to<JsonVariant>(), fs_doc.as<JsonVariantConst>());
      }
    }
  } 

 
}





#if DEBUG
WebDebugLogger DebugConsole;
bool enableConsoleOutputSerial = true;
bool enableConsoleOutputWeb = true;

#else
NullDebug EmptyConsole;
bool enableConsoleOutputSerial = false;
bool enableConsoleOutputWeb = false;

#endif

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    client->text("--- Connected to WebSocket ---");
  }
}

void saveConfigCallback() {
    DEBUG_SERIAL.println(F("New WiFi settings, reboot..."));
    delay(1000);
    ESP.restart();
    delay(5000);
}

void WifiManagerSetup(bool stationmode)
{
  WiFi.setAutoReconnect(true);
 
  #if defined(ESP32)
    if (!LittleFS.begin(true)) {
      DEBUG_SERIAL.println(F("LittleFS Mounting Error!"));
    }
  #elif defined(ESP8266)
    if (!LittleFS.begin()) {
      DEBUG_SERIAL.println(F("LittleFS Mounting Error. Try formating..."));
      LittleFS.format();
      LittleFS.begin();
    }
  #endif
  load_parameters(); // Load parameters from Preferences
 

  static WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);
  wifiManager.setShowStaticFields(true);

  wifiManager.setTitle(String("Energy2Shelly for ESP ") + VERSION_BUILD);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setConfigPortalTimeout(180);
  std::vector<const char *> usermenu = {"wifi","info","exit","sep","update"};
  wifiManager.setMenu(usermenu);
  if (stationmode)
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000)
    {
      delay(500);
      Serial.print(".");
    }
    update_reset_reason(Energy2Shelly_ResetReason::OTA_UPDATE);
    if (WiFi.status() == WL_CONNECTED)
    {
      shouldSaveConfig=true;      
      wifiManager.startWebPortal();
      startAttemptTime = millis();
      while (wifiManager.getWebPortalActive())
      {
        wifiManager.process();
        delay(10);
      }
     
    }
  }
  else
  {
    if (!wifiManager.autoConnect("Energy2Shelly"))
    {
      DEBUG_SERIAL.println(F("failed to connect and hit timeout"));
     
      delay(3000);
      ESP.restart();
      delay(5000);
    }
  }
  
  DEBUG_SERIAL.println(F("connected"));
 
  // Set Shelly ID to ESP's MAC address by default
  String currentMac = config_doc["shelly"]["shelly_mac"].as<String>();
  if (currentMac == "null" || currentMac.length() == 0) {
    
    String macStr = WiFi.macAddress(); 
    macStr.replace(":", "");        
    
    config_doc["shelly"]["shelly_mac"] = macStr;  
  }
  
  interprete_parameters();

  DEBUG_SERIAL.println(F("local ip"));
  DEBUG_SERIAL.println(WiFi.localIP());
}

void setupMdns()
{
  // Set up mDNS responder
  if (!MDNS.begin(shelly_name.c_str()))
  {
    DEBUG_SERIAL.println(F("Error setting up MDNS responder!"));
  }
  String gen_str = shelly_gen.as<String>();

#ifdef ESP32
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("shelly", "tcp", 80);
  mdns_txt_item_t serviceTxtData[4] = {
      {"arch", "esp8266"},
      {"gen", gen_str.c_str()},
      {"fw_id", shelly_fw_id.as<const char*>()},
      {"id", shelly_name.c_str()}};
  mdns_service_instance_name_set("_http", "_tcp", shelly_name.c_str());
  mdns_service_txt_set("_http", "_tcp", serviceTxtData, 4);
  mdns_service_instance_name_set("_shelly", "_tcp", shelly_name.c_str());
  mdns_service_txt_set("_shelly", "_tcp", serviceTxtData, 4);
#else
  hMDNSService = MDNS.addService(0, "http", "tcp", 80);
  hMDNSService2 = MDNS.addService(0, "shelly", "tcp", 80);
  if (hMDNSService)
  {
    MDNS.setServiceName(hMDNSService, shelly_name.c_str());
    MDNS.addServiceTxt(hMDNSService, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService, "gen", gen_str.c_str());
    MDNS.addServiceTxt(hMDNSService, "fw_id", shelly_fw_id.as<const char*>());
    MDNS.addServiceTxt(hMDNSService, "id", shelly_name.c_str());
  }
  if (hMDNSService2)
  {
    MDNS.setServiceName(hMDNSService2, shelly_name.c_str());
    MDNS.addServiceTxt(hMDNSService2, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService2, "gen", gen_str.c_str());
    MDNS.addServiceTxt(hMDNSService2, "fw_id", shelly_fw_id.as<const char*>());
    MDNS.addServiceTxt(hMDNSService2, "id", shelly_name.c_str());
  }
#endif
  DEBUG_SERIAL.println(F("mDNS responder started"));
}
