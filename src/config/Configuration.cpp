#include "Configuration.h"

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS
// ============================================================================

// Debug & Timing
unsigned long startMillis = 0;
unsigned long startMillis_sunspec = 0;
unsigned long currentMillis;

// Configuration Variables - Data source and server settings
char input_type[40];
char mqtt_server[80];
char mqtt_port[6] = "1883";
char mqtt_topic[90] = "tele/meter/SENSOR";
char mqtt_user[40] = "";
char mqtt_passwd[40] = "";

// JSON paths
char power_path[60] = "";
char pwr_export_path[60] = "";
char power_l1_path[60] = "";
char power_l2_path[60] = "";
char power_l3_path[60] = "";
char energy_in_path[60] = "";
char energy_out_path[60] = "";

// Device settings
char shelly_gen[2] = "2";
char shelly_fw_id[32] = "20241011-114455/1.4.4-g6d2a586";
char shelly_mac[13];
char shelly_name[26] = "shellypro3em-";
char shelly_port[6] = "2220"; // old: 1010; new (FW>=226): 2220

// Query and protocol settings
char query_period[10] = "1000";
char modbus_dev[10] = "71"; // default for KSEM
char force_pwr_decimals[6] = "true"; // to fix Marstek bug
bool forcePwrDecimals = true; // to fix Marstek bug
char sma_id[17] = "";

// LED configuration
char led_gpio[3] = "";
char led_gpio_i[6];

// Cached numeric conversions
int mqttPortInt = 1883;
int shellyPortInt = 2220;
int ledGpioInt = 0;
int queryPeriodMs = 1000;
int modbusDeviceId = 71;
bool ledInverted = false;

// Global buffers
uint8_t networkBuffer[1024];
JsonDocument globalJsonDoc;

// Hardware-specific
IPAddress modbus_ip;
ModbusIP modbus1;
int16_t modbus_result[256];

const uint8_t defaultVoltage = 230;
const uint8_t defaultFrequency = 50;
const uint8_t defaultPowerFactor = 1;

// LED blink
unsigned long ledOffTime = 0;
uint8_t led = 0;
bool led_i = false;
const uint8_t ledblinkduration = 50;

// RPC & period
unsigned long period = 1000;
int rpcId = 1;
char rpcUser[20] = "user_1";

// SMA Multicast
unsigned int multicastPort = 9522; // local port to listen on
IPAddress multicastIP(239, 12, 255, 254);

// WiFiManager & Preferences flags
bool shouldSaveConfig = false;
bool shouldResetConfig = false;
bool shouldReboot = false;

Preferences preferences;

// Data source flags
bool dataMQTT = false;
bool dataSMA = false;
bool dataSHRDZM = false;
bool dataHTTP = false;
bool dataSUNSPEC = false;
bool mqtt_configured = false; // Flag to indicate MQTT is fully configured and safe to use

// Network objects
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
AsyncWebServer server(80);
AsyncWebSocket webSocket("/rpc");
WiFiUDP Udp;
HTTPClient http;
WiFiUDP UdpRPC;

// mDNS handles (ESP8266 only)
#ifndef ESP32
MDNSResponder::hMDNSService hMDNSService = 0; // handle of the http service in the MDNS responder
MDNSResponder::hMDNSService hMDNSService2 = 0; // handle of the shelly service in the MDNS responder
#endif

// ============================================================================
// CALLBACK FUNCTIONS
// ============================================================================

// callback notifying us of the need to save WifiManager config
void saveConfigCallback() {
  DEBUG_SERIAL.println("Should save config");
  shouldSaveConfig = true;
}

// ============================================================================
// CONFIGURATION MANAGEMENT
// ============================================================================

void loadConfiguration() {
  preferences.begin("e2s_config", false);
  preferences.getString("input_type", input_type, sizeof(input_type));
  preferences.getString("mqtt_server", mqtt_server, sizeof(mqtt_server));
  preferences.getString("query_period", query_period, sizeof(query_period));
  queryPeriodMs = atoi(query_period);

  preferences.getString("led_gpio", led_gpio, sizeof(led_gpio));
  ledGpioInt = atoi(led_gpio);
  preferences.getString("led_gpio_i", led_gpio_i, sizeof(led_gpio_i));
  ledInverted = (strcmp(led_gpio_i, "true") == 0);

  preferences.getString("shelly_mac", shelly_mac, sizeof(shelly_mac));
  preferences.getString("mqtt_port", mqtt_port, sizeof(mqtt_port));
  mqttPortInt = atoi(mqtt_port);
  preferences.getString("mqtt_topic", mqtt_topic, sizeof(mqtt_topic));
  preferences.getString("mqtt_user", mqtt_user, sizeof(mqtt_user));
  preferences.getString("mqtt_passwd", mqtt_passwd, sizeof(mqtt_passwd));
  preferences.getString("modbus_dev", modbus_dev, sizeof(modbus_dev));
  modbusDeviceId = atoi(modbus_dev);
  preferences.getString("power_path", power_path, sizeof(power_path));
  preferences.getString("pwr_export_path", pwr_export_path, sizeof(pwr_export_path));
  preferences.getString("power_l1_path", power_l1_path, sizeof(power_l1_path));
  preferences.getString("power_l2_path", power_l2_path, sizeof(power_l2_path));
  preferences.getString("power_l3_path", power_l3_path, sizeof(power_l3_path));
  preferences.getString("energy_in_path", energy_in_path, sizeof(energy_in_path));
  preferences.getString("energy_out_path", energy_out_path, sizeof(energy_out_path));
  preferences.getString("shelly_port", shelly_port, sizeof(shelly_port));
  shellyPortInt = atoi(shelly_port);
  forcePwrDecimals = preferences.getBool("force_pwr_decimals", true);
  preferences.getString("sma_id", sma_id, sizeof(sma_id));
  preferences.end();
}

void saveConfiguration() {
  preferences.begin("e2s_config", false);
  preferences.putString("input_type", input_type);
  preferences.putString("mqtt_server", mqtt_server);
  preferences.putString("query_period", query_period);
  preferences.putString("led_gpio", led_gpio);
  preferences.putString("led_gpio_i", led_gpio_i);
  preferences.putString("shelly_mac", shelly_mac);
  preferences.putString("shelly_port", shelly_port);
  preferences.putBool("force_pwr_decimals", forcePwrDecimals);
  preferences.putString("sma_id", sma_id);
  preferences.putString("mqtt_port", mqtt_port);
  preferences.putString("mqtt_topic", mqtt_topic);
  preferences.putString("mqtt_user", mqtt_user);
  preferences.putString("mqtt_passwd", mqtt_passwd);
  preferences.putString("modbus_dev", modbus_dev);
  preferences.putString("power_path", power_path);
  preferences.putString("pwr_export_path", pwr_export_path);
  preferences.putString("power_l1_path", power_l1_path);
  preferences.putString("power_l2_path", power_l2_path);
  preferences.putString("power_l3_path", power_l3_path);
  preferences.putString("energy_in_path", energy_in_path);
  preferences.putString("energy_out_path", energy_out_path);
  preferences.end();
}

// ============================================================================
// WIFIMANAGER SETUP (Large function extracted from original main.cpp)
// ============================================================================

void WifiManagerSetup() {
  DEBUG_SERIAL.println("DEBUG: WifiManagerSetup() START");

  // Set Shelly ID to ESP's MAC address by default
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(shelly_mac, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2],
          mac[3], mac[4], mac[5]);

  DEBUG_SERIAL.println("DEBUG: About to call preferences.begin()");
  loadConfiguration();
  DEBUG_SERIAL.println("DEBUG: preferences.begin() done, now loading values");

  // Load all preferences first - avoid any network operations during loading
  DEBUG_SERIAL.print("DEBUG: loaded input_type=");
  DEBUG_SERIAL.println(input_type);
  DEBUG_SERIAL.print("DEBUG: loaded mqtt_server=");
  DEBUG_SERIAL.println(mqtt_server);

  // WiFi-only initial setup - all other configuration is done via web interface
  WiFiManagerParameter custom_info(
      "<p style='margin-top:20px;'><b>WiFi Configuration Only</b></p>"
      "<p>After connecting to WiFi, visit the device's IP address in your browser to configure all other settings at <b>/config</b></p>");

  /*
  // All configuration parameters removed from initial WiFi setup
  // These are now configured via the web interface at /config
  WiFiManagerParameter custom_section1("<h3>General settings</h3>");
  WiFiManagerParameter custom_input_type(...);
  WiFiManagerParameter custom_mqtt_server(...);
  ... etc ...
  */

  WiFiManager wifiManager;
  if (!DEBUG) {
    wifiManager.setDebugOutput(false);
  }
  wifiManager.setTitle("Energy2Shelly for ESP");
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Add info message to WiFi setup portal
  wifiManager.addParameter(&custom_info);

  // All other parameters removed - configure via web interface at /config

  DEBUG_SERIAL.println("DEBUG: About to call wifiManager.autoConnect()");
  // Only attempt WiFiManager if configuration is invalid or incomplete
  bool needsConfiguration = (input_type[0] == '\0' || mqtt_server[0] == '\0');
  if (needsConfiguration) {
    DEBUG_SERIAL.println(
        "DEBUG: Configuration incomplete, running WiFiManager.autoConnect()");
    if (!wifiManager.autoConnect("Energy2Shelly")) {
      DEBUG_SERIAL.println("failed to connect and hit timeout");
      delay(3000);
      ESP.restart();
      delay(5000);
    }
    DEBUG_SERIAL.println("DEBUG: wifiManager.autoConnect() completed");
  } else {
    DEBUG_SERIAL.println(
        "DEBUG: Configuration complete, attempting normal WiFi connection");
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

  // Configuration is loaded from Preferences only (not from WiFiManager)
  // All configuration is done via web interface at /config after initial WiFi setup

  DEBUG_SERIAL.println("The values in the preferences are: ");
  DEBUG_SERIAL.printf("\tinput_type : %s\n", input_type);
  DEBUG_SERIAL.printf("\tmqtt_server : %s\n", mqtt_server);
  DEBUG_SERIAL.printf("\tmqtt_port : %s\n", mqtt_port);
  DEBUG_SERIAL.printf("\tquery_period : %s\n", query_period);
  DEBUG_SERIAL.printf("\tled_gpio : %s\n", led_gpio);
  DEBUG_SERIAL.printf("\tled_gpio_i : %s\n", led_gpio_i);
  DEBUG_SERIAL.printf("\tshelly_mac : %s\n", shelly_mac);
  DEBUG_SERIAL.printf("\tmqtt_topic : %s\n", mqtt_topic);
  DEBUG_SERIAL.printf("\tmqtt_user : %s\n", mqtt_user);
  DEBUG_SERIAL.printf("\tmqtt_passwd : %s\n", mqtt_passwd);
  DEBUG_SERIAL.printf("\tmodbus_dev : %s\n", modbus_dev);
  DEBUG_SERIAL.printf("\tpower_path : %s\n", power_path);
  DEBUG_SERIAL.printf("\tpwr_export_path : %s\n", pwr_export_path);
  DEBUG_SERIAL.printf("\tpower_l1_path : %s\n", power_l1_path);
  DEBUG_SERIAL.printf("\tpower_l2_path : %s\n", power_l2_path);
  DEBUG_SERIAL.printf("\tpower_l3_path : %s\n", power_l3_path);
  DEBUG_SERIAL.printf("\tenergy_in_path : %s\n", energy_in_path);
  DEBUG_SERIAL.printf("\tenergy_out_path : %s\n", energy_out_path);
  DEBUG_SERIAL.printf("\tshelly_port : %s\n", shelly_port);
  DEBUG_SERIAL.printf("\tforce_pwr_decimals : %s\n", force_pwr_decimals);
  DEBUG_SERIAL.printf("\tsma_id : %s\n", sma_id);

  if (strcmp(input_type, "SMA") == 0) {
    dataSMA = true;
    DEBUG_SERIAL.println("Enabling SMA Multicast data input");
  } else if (strcmp(input_type, "SHRDZM") == 0) {
    dataSHRDZM = true;
    DEBUG_SERIAL.println("Enabling SHRDZM UDP data input");
  } else if (strcmp(input_type, "HTTP") == 0) {
    if (mqtt_server[0] != '\0') {
      dataHTTP = true;
      DEBUG_SERIAL.println("Enabling generic HTTP data input");
    } else {
      DEBUG_SERIAL.println(
          "HTTP server not configured - disabling HTTP data input");
    }
  } else if (strcmp(input_type, "SUNSPEC") == 0) {
    if (mqtt_server[0] != '\0') {
      dataSUNSPEC = true;
      DEBUG_SERIAL.println("Enabling SUNSPEC data input");
    } else {
      DEBUG_SERIAL.println(
          "SUNSPEC server not configured - disabling SUNSPEC data input");
    }
  } else {
    if (mqtt_server[0] != '\0') {
      dataMQTT = true;
      DEBUG_SERIAL.println("Enabling MQTT data input");
    } else {
      DEBUG_SERIAL.println(
          "MQTT server not configured - disabling MQTT data input");
    }
  }

  led_i = ledInverted;

  if (shouldSaveConfig) {
    DEBUG_SERIAL.println("saving config");
    saveConfiguration();
    wifiManager.reboot();
  }
  DEBUG_SERIAL.println("local ip");
  DEBUG_SERIAL.println(WiFi.localIP());
}
