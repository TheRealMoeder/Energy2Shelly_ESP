#include "Configuration.h"
#include "../utils/utils.h"
#include "load_save_parameters.h"
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


uint16_t sunspec_port_int; // default port
uint8_t modbusdev_int;     // default device id for KSEM

char tibber_rpc[2][26] = {"/data.json?node_id=","/node_data.json?node_id="}; // fixed rpc path


void setup_json_config_doc()
{
config_doc["reset_password"] = "admin-changeMe";
config_doc["input_type"] = "";
config_doc["ntp_server"] = "de.pool.ntp.org";
config_doc["timezone"] =  "CET-1CEST,M3.5.0/2,M10.5.0/3";
config_doc["phase_number"] = "3";
config_doc["power_offset"] = "0";
config_doc["mqtt_server"] = "";
config_doc["mqtt_port"] =  "1883";
config_doc["mqtt_topic"] = "tele/meter/SENSOR";
config_doc["mqtt_user"] = "";
config_doc["mqtt_passwd"] = "";
config_doc["power_path"] = "";
config_doc["pwr_export_path"] = "";
config_doc["power_l1_path"] = "";
config_doc["power_l2_path"] = "";
config_doc["power_l3_path"] = "";
config_doc["energy_in_path"] = "";
config_doc["energy_out_path"] = "";
config_doc["shelly_gen"] = "2";
config_doc["shelly_fw_id"] = "20241011-114455/1.4.4-g6d2a586";
config_doc["shelly_mac"] = "";
config_doc["shelly_name"] = "shellypro3em-";
config_doc["shelly_port"] = "2220";
config_doc["query_period"] = "1000";
config_doc["modbus_dev"] = "71";
config_doc["sma_id"] = "";
config_doc["tibber_host"] = "x.x.x.x[:xxxx]";
config_doc["tibber_user"] = "admin";
config_doc["tibber_password"] = "xxxx-xxxx";
config_doc["tibber_nodeid"] = "1";
config_doc["rct_host"] = "192.168.0.1";
config_doc["rct_port"] = "8899";
}

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

// RPC and query settings
unsigned long period = 1000;
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
  if (strcmp(input_type, "SMA") == 0)
  {
    dataSMA = true;
    DEBUG_SERIAL.println(F("Enabling SMA Multicast data input"));
  }
  else if (strcmp(input_type, "SHRDZM") == 0)
  {
    dataSHRDZM = true;
    DEBUG_SERIAL.println(F("Enabling SHRDZM UDP data input"));
  }
  else if (strcmp(input_type, "HTTP") == 0)
  {
    dataHTTP = true;
    DEBUG_SERIAL.println(F("Enabling generic HTTP data input"));
  }
  else if (strcmp(input_type, "SUNSPEC") == 0)
  {
    dataSUNSPEC = true;
    DEBUG_SERIAL.println(F("Enabling SUNSPEC data input"));
  }
  else if (strcmp(input_type, "TIBBERPULSE") == 0)
  {
    dataTIBBERPULSE = true;
    DEBUG_SERIAL.println(F("Enabling TIBBERPULSE data input"));
  }
  else if (strcmp(input_type, "RCT") == 0)
  {
    dataRCT = true;
    DEBUG_SERIAL.println(F("Enabling RCT Power data input"));
  }
  else
  {
    dataMQTT = true;
    DEBUG_SERIAL.println(F("Enabling MQTT data input"));
  }

  if (strcmp(led_gpio_i, "true") == 0)
  {
    led_i = true;
  }
  else
  {
    led_i = false;
  }

  // Convert string parameters to integers where necessary
  modbusdev_int = config_doc["sunspec"]["modbus_dev"].as<uint8_t>();
  sunspec_port_int = config_doc["sunspec"]["modbus_port"].as<uint16_t>();

  period = config_doc["http"]["query_period"].as<unsigned long>();
  modbus_ip.fromString(config_doc["sunspec"]["modbus_ip"].as<const char*>()); // Todo, replace by DNS resolver for hostnames, currently only IP addresses are supported

}


// callback notifying us of the need to save WifiManager config
void saveConfigCallback()
{
  DEBUG_SERIAL.println(F("Should save config"));
  shouldSaveConfig = true;
}

#if DEBUG
WebDebugLogger DebugConsole;
bool enableConsoleOutput = true;
#else
NullDebug EmptyConsole;
bool enableConsoleOutput = false;
#endif

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    client->text("--- Connected to WebSocket ---");
  }
}

void WifiManagerSetup(bool stationmode)
{
  WiFi.setAutoReconnect(true);
  // Set Shelly ID to ESP's MAC address by default
  uint8_t mac[6];
  WiFi.macAddress(mac);
   
  load_parameters(); // Load parameters from Preferences
  config_doc["shelly"]["shelly_mac"] = String(mac[0], HEX) + String(mac[1], HEX) + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
 
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

  offsetPerPhase = power_offset_double / 3.0; // distribute offset equally across phases

 
  interprete_parameters();
 
  if (shouldSaveConfig)
  {
    DEBUG_SERIAL.println(F("saving config"));
    //save_parameters();   
    wifiManager.reboot();
  }
  DEBUG_SERIAL.println(F("local ip"));
  DEBUG_SERIAL.println(WiFi.localIP());
}

void setupMdns()
{
  // Set up mDNS responder
  String name_str = config_doc["shelly"]["shelly_name"] | "";
  String mac_str  = config_doc["shelly"]["shelly_mac"] | "";
  String cat_name = name_str + mac_str;

 if (cat_name.length() > 128) {
    cat_name = cat_name.substring(0, 128);
 }
  if (!MDNS.begin(cat_name.c_str( )))
  {
    DEBUG_SERIAL.println(F("Error setting up MDNS responder!"));
  }

#ifdef ESP32
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("shelly", "tcp", 80);
  mdns_txt_item_t serviceTxtData[4] = {
      {"arch", "esp8266"},
      {"gen", shelly_gen},
      {"fw_id", shelly_fw_id},
      {"id", shelly_name}};
  mdns_service_instance_name_set("_http", "_tcp", cat_name.c_str());
  mdns_service_txt_set("_http", "_tcp", serviceTxtData, 4);
  mdns_service_instance_name_set("_shelly", "_tcp", cat_name.c_str());
  mdns_service_txt_set("_shelly", "_tcp", serviceTxtData, 4);
#else
  hMDNSService = MDNS.addService(0, "http", "tcp", 80);
  hMDNSService2 = MDNS.addService(0, "shelly", "tcp", 80);
  if (hMDNSService)
  {
    MDNS.setServiceName(hMDNSService, cat_name.c_str());
    MDNS.addServiceTxt(hMDNSService, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService, "gen", shelly_gen);
    MDNS.addServiceTxt(hMDNSService, "fw_id", shelly_fw_id);
    MDNS.addServiceTxt(hMDNSService, "id", shelly_name);
  }
  if (hMDNSService2)
  {
    MDNS.setServiceName(hMDNSService2, cat_name.c_str());
    MDNS.addServiceTxt(hMDNSService2, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService2, "gen", shelly_gen);
    MDNS.addServiceTxt(hMDNSService2, "fw_id", shelly_fw_id);
    MDNS.addServiceTxt(hMDNSService2, "id", shelly_name);
  }
#endif
  DEBUG_SERIAL.println(F("mDNS responder started"));
}
