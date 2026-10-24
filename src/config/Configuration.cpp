#include "Configuration.h"
#include "../utils/utils.h"
unsigned long startMillis = 0;
unsigned long onamillis=0;
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
bool enableOna=false;

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


  if (config_doc["ona"]["publish_period"].isNull())  config_doc["ona"]["publish_period"]=0;
  enableOna=config_doc["ona"]["publish_period"].as<int>()>0;

  if (config_doc["ona"]["latitude"].isNull())  config_doc["ona"]["latitude"]=0.0;
  if (config_doc["ona"]["longitude"].isNull()) config_doc["ona"]["longitude"]=0.0;

}

void erase_preferences()
{
  preferences.begin("e2s_config", false);
  preferences.clear();
  preferences.end();
}




String json_prefs_string;
JsonDocument update_config_doc;


void mergeJson(JsonObject target, JsonObjectConst source) {
    for (JsonPairConst p : source) {
        JsonVariantConst value = p.value();
        
        if (value.is<JsonObjectConst>()) {
            JsonObject targetChild = target[p.key()].to<JsonObject>();
            mergeJson(targetChild, value.as<JsonObjectConst>());
        } else {
            target[p.key()] = value;
        }
    }
}



void save_parameters() {
    preferences.begin("e2s_config", false);
    serializeJson(config_doc, json_prefs_string);
    preferences.putString("config_doc", json_prefs_string);
    preferences.end();

}

void load_parameters() {
  preferences.begin("e2s_config", false);
  config_doc.clear();
  if (preferences.isKey("config_doc")) {
    json_prefs_string = preferences.getString("config_doc", "");
    DEBUG_SERIAL.println(json_prefs_string);
    deserializeJson(config_doc, json_prefs_string);
  } else {
    // Load individual parameters if the combined config_doc is not present
    config_doc["basic"]["reset_password"] = preferences.getString("reset_password", "");
    config_doc["basic"]["input_type"] = preferences.getString("input_type", "");
    config_doc["basic"]["ntp_server"] = preferences.getString("ntp_server", "");
    config_doc["basic"]["timezone"] = preferences.getString("timezone", "");
    config_doc["basic"]["phase_number"] = preferences.getString("phase_number", "");
    config_doc["basic"]["power_offset"] = preferences.getString("power_offset", "");

    config_doc["mqtt"]["mqtt_server"] = preferences.getString("mqtt_server", "");
    config_doc["mqtt"]["mqtt_port"] = preferences.getString("mqtt_port", "");
    config_doc["mqtt"]["mqtt_topic"] = preferences.getString("mqtt_topic", "");
    config_doc["mqtt"]["mqtt_user"] = preferences.getString("mqtt_user", "");
    config_doc["mqtt"]["mqtt_password"] = preferences.getString("mqtt_passwd", "");

    config_doc["sunspec"]["modbus_dev"] = preferences.getString("modbus_dev", "");
    config_doc["sunspec"]["modbus_ip"] = preferences.getString("mqtt_server", "");
    config_doc["sunspec"]["modbus_port"] = preferences.getString("mqtt_port", "");
    config_doc["sunspec"]["query_period"] = preferences.getString("query_period", "");
   

    config_doc["led"]["led_gpio"] = preferences.getString("led_gpio", "");
    config_doc["led"]["led_gpio_i"] = preferences.getString("led_gpio_i", "");

    config_doc["shelly"]["shelly_port"] = preferences.getString("shelly_port", "2220");
   
    config_doc["json"]["power_path"] = preferences.getString("power_path", "");
    config_doc["json"]["pwr_export_path"] = preferences.getString("pwr_export_path", "");
    config_doc["json"]["power_l1_path"] = preferences.getString("power_l1_path", "");
    config_doc["json"]["power_l2_path"] = preferences.getString("power_l2_path", "");
    config_doc["json"]["power_l3_path"] = preferences.getString("power_l3_path", "");
    config_doc["json"]["energy_in_path"] = preferences.getString("energy_in_path", "");
    config_doc["json"]["energy_out_path"] = preferences.getString("energy_out_path", "");
    
    
    config_doc["http"]["server_ip"] = preferences.getString("mqtt_server", ""); 
    config_doc["http"]["query_period"] = preferences.getString("query_period", "");

    config_doc["sma"]["sma_id"] = preferences.getString("sma_id", "");
    
    
    config_doc["tibber"]["tibber_host"] = preferences.getString("tibber_host", "");
    config_doc["tibber"]["tibber_nodeid"] = preferences.getString("tibber_nodeid", "");
    config_doc["tibber"]["tibber_user"] = preferences.getString("tibber_user", "");
    config_doc["tibber"]["tibber_password"] = preferences.getString("tibber_password", "");
    config_doc["tibber"]["query_period"] = preferences.getString("query_period", "");

    config_doc["rct"]["rct_host"] = preferences.getString("rct_host", "");
    config_doc["rct"]["rct_port"] = preferences.getString("rct_port", "");
    config_doc["rct"]["query_period"] = preferences.getString("query_period", "");
  }
    preferences.end();


   /* make sure default values are available */
   config_doc["shelly"]["shelly_port"] = config_doc["shelly"]["shelly_port"].as<int>() ? config_doc["shelly"]["shelly_port"].as<int>() : 2220;
   config_doc["shelly"]["shelly_gen"] = config_doc["shelly"]["shelly_gen"].as<int>() ? config_doc["shelly"]["shelly_gen"].as<int>() : 2;
   config_doc["shelly"]["shelly_fw_id"] = config_doc["shelly"]["shelly_fw_id"] | "20241011-114455/1.4.4-g6d2a586";
   config_doc["shelly"]["shelly_name"] = config_doc["shelly"]["shelly_name"] | "shellypro3em-";

   config_doc["basic"]["phase_number"] = config_doc["basic"]["phase_number"].as<int>() ? config_doc["basic"]["phase_number"].as<int>(): 3;
   config_doc["basic"]["ntp_server"] = config_doc["basic"]["ntp_server"]| "de.pool.ntp.org";
   config_doc["basic"]["input_type"] = config_doc["basic"]["input_type"].as<int>() ? config_doc["basic"]["input_type"].as<int>() : static_cast<int>(input_type_enum::NONE);
   if (config_doc["basic"]["ntp_server"] == "") config_doc["basic"]["ntp_server"] = "de.pool.ntp.org";
   if (config_doc["basic"]["timezone"] == "") config_doc["basic"]["timezone"] = "CET-1CEST,M3.5.0/2,M10.5.0/3";
   
   config_doc["sunspec"]["query_period"] = config_doc["sunspec"]["query_period"].as<int>() ? config_doc["sunspec"]["query_period"].as<int>() : 5000;
   config_doc["tibber"]["query_period"] = config_doc["tibber"]["query_period"].as<int>() ? config_doc["tibber"]["query_period"].as<int>() : 5000;
   config_doc["rct"]["query_period"]    = config_doc["rct"]["query_period"].as<int>() ? config_doc["rct"]["query_period"].as<int>() : 5000;
   config_doc["http"]["query_period"]   = config_doc["http"]["query_period"].as<int>() ? config_doc["http"]["query_period"].as<int>() : 5000;
   
 
   config_doc["console"]["web_console"]  =    config_doc["console"]["web_console"] | "true";
   config_doc["console"]["serial_console_time"] = config_doc["console"]["serial_console_time"]|"";
   config_doc["console"]["serial_console_time"]  = config_doc["console"]["serial_console_time"].as<int>() ? config_doc["console"]["serial_console_time"].as<int>() : (int)1;

 

 
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
