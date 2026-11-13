#include "Configuration.h"
#include "../utils/utils.h"
#include "../parsers/SunspecParser.h"
const char* configPath = "/config.json";
unsigned long startMillis = 0;
unsigned long currentMillis;
// for time synchronization
time_t now;
tm timeinfo;
JsonDocument config_doc;

// ============================================================================
// CONFIGURATION VARIABLES (stored in LittleFS)
// ============================================================================

// Data source and server settings

String shelly_name;

uint16_t sunspec_port_int; // default port
uint8_t modbusdev_int;     // default device id for KSEM

unsigned long ledOffTime = 0;
uint8_t led = 0;
bool led_i = false;
const uint8_t ledblinkduration = 50;

// SMA Multicast IP and Port
unsigned int multicastPort = 9522; // local port to listen on
IPAddress multicastIP(239, 12, 255, 254);

// MODBUS settings
IPAddress modbus_ip;
ModbusIP* modbus1 = nullptr;
int16_t* modbus_result = nullptr;

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
bool shouldReboot = false;

// flags for data sources
input_type_enum current_input_type  = input_type_enum::NONE;

// ============================================================================
// NETWORK OBJECTS
// ============================================================================

WiFiClient wifi_client;
PubSubClient* mqtt_client = nullptr;
AsyncWebServer server(80);
AsyncWebSocket webSocket("/rpc");
AsyncWebSocket wsConsole("/consolews");
WiFiUDP* Udp = nullptr;
HTTPClient* http = nullptr;
WiFiUDP UdpRPC;
uint16_t wifi_reconnect_attempts = 0;
DNSServer dnsServer;
const byte DNS_PORT = 53;
unsigned long WiFi_lastScanTime = 0;
bool WiFi_scanInProgress = false;
// ============================================================================
// MDNS RESPONDER HANDLES (ESP8266 only)
// ============================================================================

#ifndef ESP32
MDNSResponder::hMDNSService hMDNSService = 0;  // handle of the http service in the MDNS responder
MDNSResponder::hMDNSService hMDNSService2 = 0; // handle of the shelly service in the MDNS responder
#endif



#ifdef ESP32
#define LITTLEFS_BEGIN() LittleFS.begin(false)
#else
#define LITTLEFS_BEGIN() LittleFS.begin()
#endif


static void destroyClientObjects()
{
  if (mqtt_client != nullptr) {
    mqtt_client->disconnect();
    delete mqtt_client;
    mqtt_client = nullptr;
  }

  if (http != nullptr) {
    http->end();
    delete http;
    http = nullptr;
  }

  if (Udp != nullptr) {
    Udp->stop();
    delete Udp;
    Udp = nullptr;
  }

  if (modbus1 != nullptr) {
    delete modbus1;
    modbus1 = nullptr;
  }
  if (modbus_result!=nullptr) {
    delete modbus_result;
    modbus_result = nullptr;
  }
}

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

void remove_filtered_parameters_ram(JsonDocument filter) {
  for (JsonPair p : filter.as<JsonObject>()) {
    config_doc.remove(p.key());
  }
  config_doc.shrinkToFit();
  DEBUG_SERIAL.println(F("Sections successfully removed from config_doc in RAM."));
}

void WiFi_processScanResult(int n) {
  if (!config_doc["wifi"]["networks"].isNull()) {
    config_doc["wifi"].remove("networks"); 
  }
  
  JsonArray networks = config_doc["wifi"]["networks"].to<JsonArray>();
  
  for (int i = 0; i < n; ++i) {
    JsonObject net = networks.add<JsonObject>();
    
    net["ssid"] = WiFi.SSID(i);
    net["rssi"] = WiFi.RSSI(i);
    net["channel"] = WiFi.channel(i);
    
#if defined(ESP32)
    net["encryption"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured";
#elif defined(ESP8266)
    net["encryption"] = (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? "Open" : "Secured";
#endif
  }
}

void wifi_scan()
{
  if (WiFi_scanInProgress) {
    int16_t scanResult = WiFi.scanComplete();

    if (scanResult >= 0) {
        WiFi_processScanResult(scanResult);
        WiFi.scanDelete();
        WiFi_scanInProgress = false;
        WiFi_lastScanTime = millis();
    } 
    else if (scanResult == WIFI_SCAN_FAILED) {
      DEBUG_SERIAL.println(F("ESP8266 WiFi-Scan Error."));
      WiFi.scanDelete();
      WiFi_scanInProgress = false;
      WiFi_lastScanTime = millis();
    }
  }
  if (!WiFi_scanInProgress && (millis() - WiFi_lastScanTime >= WIFI_SCAN_INTERVAL)) {
      DEBUG_SERIAL.println(F("\nStart asynchron ESP8266 WiFi-Scan..."));
#ifdef ESP8266
      WiFi.scanNetworks(true); // async=true
#else      

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      WiFi.scanNetworks(true, false, false); 
#endif
#endif
      WiFi_scanInProgress = true;
  }

}

void interprete_parameters()
{
  JsonDocument filter;
  config_doc.clear();

  filter["shelly"]=true; filter["basic"]=true; filter["console"]=true; filter["led"]=true;

  // Interpret the loaded parameters and set flags accordingly
  load_filtered_parameters(filter);
  serializeJsonPretty(config_doc,Serial);
  filter.clear();
  current_input_type = static_cast<input_type_enum>(config_doc["basic"]["input_type"].as<uint32_t>());
  destroyClientObjects(); // Destroy unused client objects; Todo: Input change without reboot will lead to crash!
  
  switch (current_input_type) {
    case input_type_enum::SMA:
        filter["sma"]=true;
        load_filtered_parameters(filter);
        DEBUG_SERIAL.println(F("Enabling SMA Multicast data input"));
        break;
    case input_type_enum::SHRDZM:
        DEBUG_SERIAL.println(F("Enabling SHRDZM UDP data input"));
        break;
    case input_type_enum::HTTP:
        filter["http"]=true;
        filter["json"]=true;
        load_filtered_parameters(filter);
        DEBUG_SERIAL.println(F("Enabling generic HTTP data input"));
        break;
    case input_type_enum::SUNSPEC:
        filter["sunspec"]=true;
        filter["sunspec_registers"]=true;
        load_filtered_parameters(filter);
        modbus_result = new int16_t[MAX_MODBUS_REGISTERS];
        memset(modbus_result, 0, MAX_MODBUS_REGISTERS * sizeof(int16_t));
        convert_json_to_sunspec_registers();
        filter.clear();
        filter["sunspec_registers"]=true;
        remove_filtered_parameters_ram(filter);
        DEBUG_SERIAL.println(F("Enabling SUNSPEC data input"));
        break;
   case input_type_enum::TIBBERPULSE:
        filter["tibber"]=true;
        load_filtered_parameters(filter);
        DEBUG_SERIAL.println(F("Enabling TIBBERPULSE data input"));
        break;
   case input_type_enum::SMLSERIAL:
        DEBUG_SERIAL.println(F("Enabling SML serial (UART) data input"));
        break;
   case input_type_enum::RCT:
        filter["rct"]=true;
        load_filtered_parameters(filter);
        DEBUG_SERIAL.println(F("Enabling RCT Power data input"));
        break;
   case input_type_enum::MQTT:
        filter["mqtt"]=true;
        filter["json"]=true;
        load_filtered_parameters(filter);
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

  config_doc["console"]["serial_console_time"] = config_doc["console"]["serial_console_time"].as<int>() ? config_doc["console"]["serial_console_time"].as<int>() : (int)1;
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

config_doc.shrinkToFit();

}

void mergeJson(JsonVariant dst, JsonVariantConst src) {
  if (src.is<JsonObjectConst>()) {
    if (!dst.is<JsonObject>()) {
      dst.to<JsonObject>();
    } 
    JsonObject dstObj = dst.as<JsonObject>();
    
    for (JsonPairConst kv : src.as<JsonObjectConst>()) {
      if (kv.value().is<JsonObjectConst>()) {
        if (!dstObj[kv.key()].is<JsonObject>()) {
          dstObj[kv.key()].to<JsonObject>();
        }
        mergeJson(dstObj[kv.key()], kv.value());
      } else {
        dstObj[kv.key()] = kv.value();
      }
    }
  } else {
    dst.set(src);
  }
}

bool save_parameters() {
   if (!config_doc["wifi"]["networks"].isNull()) {
    config_doc["wifi"].remove("networks"); 
  }
  File configFile = LittleFS.open(configPath, "w");
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
  "wifi": {
    "ssid": "",
    "password": ""
  },
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
    "serial_console_time": 10
  }
}
)=====";

void init_parameters() {
  config_doc.clear();

  if (LittleFS.exists(configPath)) {
    File configFile = LittleFS.open(configPath, "r");
    if (configFile) { // exists?, super!
      configFile.close();
      return; 
      }
  }
 
  DEBUG_SERIAL.println(F("Config file does not exist, creating default..."));
 
  // else write default file
  File configFile = LittleFS.open(configPath, "w");
  if (configFile) {
    configFile.print(FPSTR(DEFAULT_CONFIG_JSON)); 
    configFile.close();
    DEBUG_SERIAL.println(F("Default config text successfully written to LittleFS."));
  }
}



void load_filtered_parameters(JsonVariantConst filter) {
  if (!LittleFS.exists(configPath)) {
    DEBUG_SERIAL.println(F("Config file does not exist for filtered loading."));
    return;
  }

  File configFile = LittleFS.open(configPath, "r");
  if (!configFile) {
    DEBUG_SERIAL.println(F("Failed to open config file for filtered loading."));
    return;
  }

  JsonDocument temp;
  DeserializationError error;
  if (filter.isNull()|| filter.size()==0) {
    error = deserializeJson(config_doc, configFile);  
  }
  else {
    error = deserializeJson(temp, configFile, DeserializationOption::Filter(filter));  
    mergeJson(config_doc,temp);
  }
  configFile.close();
  

  if (error) {
    DEBUG_SERIAL.print(F("Error during filtered deserialization: "));
    DEBUG_SERIAL.println(error.f_str());
  } else {
    DEBUG_SERIAL.println(F("Filtered parameters successfully loaded."));
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


void WifiManagerSetup(bool SW_update)
{
  JsonDocument filter;
  WiFi.setAutoReconnect(true);
 
   if (!LITTLEFS_BEGIN()) {
      DEBUG_SERIAL.println(F("LittleFS Mounting Error. Try formating..."));
      LittleFS.end();
      LittleFS.format();
      LITTLEFS_BEGIN();
   }
  init_parameters();
  filter["wifi"]=true; filter["shelly"]=true;
  load_filtered_parameters(filter); 
  if (config_doc["wifi"]["ssid"] == "") // check for valid WiFi credentials, if not present, start config portal
  {
      DEBUG_SERIAL.println(F("No WiFi credentials found, starting config portal..."));
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP("Energy2Shelly", "");

#if defined(ESP32)
      WiFi.disconnect(false); 
      WiFi.setSleep(false);       
#endif
  }
  else { //try to connect to WiFi with stored credentials
      WiFi.mode(WIFI_STA);
      WiFi.begin(config_doc["wifi"]["ssid"].as<const char*>(), config_doc["wifi"]["password"].as<const char*>());
      unsigned long startAttemptTime = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000)
      {
        delay(500);
        DEBUG_SERIAL.print(".");
      }
      if (WiFi.status() != WL_CONNECTED) // if connection failed, start config portal
      {
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP("Energy2Shelly", "");
        DEBUG_SERIAL.println(F("Failed to connect to WiFi, starting config portal..."));
#if defined(ESP32)
        WiFi.disconnect(false);
        WiFi.setSleep(false);       
#endif

      }
  }
  
   
  // Set Shelly ID to ESP's MAC address by default
  String currentMac = config_doc["shelly"]["shelly_mac"].as<String>();
 
  if (currentMac == "null" || currentMac.length() == 0) {// at least first time boot setup
    filter.clear();
    load_filtered_parameters(filter);//load all 
    String macStr = WiFi.macAddress(); 
    macStr.replace(":", "");            
    config_doc["shelly"]["shelly_mac"] = macStr;
    save_parameters();
  }

  interprete_parameters();

  DEBUG_SERIAL.println(F("local ip"));
  DEBUG_SERIAL.println(WiFi.localIP());
}


String read_config_as_string() {
  if (!LittleFS.exists(configPath)) {
    DEBUG_SERIAL.println(F("Config file does not exist."));
    return "";
  }

  File configFile = LittleFS.open(configPath, "r");
  if (!configFile) {
    DEBUG_SERIAL.println(F("Failed to open config file for reading."));
    return "";
  }

  String jsonString = "";
  
  jsonString.reserve(configFile.size());

  while (configFile.available()) {
    jsonString += (char)configFile.read();
  }

  configFile.close();
  return jsonString;
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
