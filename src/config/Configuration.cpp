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

  WiFiManagerParameter custom_section1("<h3>General settings</h3>");
  WiFiManagerParameter custom_input_type(
      "type",
      "<b>Data source</b><br><code>MQTT</code> for MQTT<br><code>HTTP</code> "
      "for generic HTTP<br><code>SMA</code> for SMA EM/HM "
      "multicast<br><code>SHRDZM</code> for SHRDZM UDP "
      "data<br><code>SUNSPEC</code> for Modbus TCP SUNSPEC data",
      input_type, 40);
  WiFiManagerParameter custom_mqtt_server(
      "server",
      "<b>Server</b><br>MQTT Server IP, query url for generic HTTP or Modbus "
      "TCP server IP for SUNSPEC",
      mqtt_server, 80);
  WiFiManagerParameter custom_mqtt_port(
      "port", "<b>Port</b><br> for MQTT or Modbus TCP (SUNSPEC)", mqtt_port, 6);
  WiFiManagerParameter custom_query_period(
      "query_period",
      "<b>Query period</b><br>for generic HTTP and SUNSPEC, in milliseconds",
      query_period, 10);
  WiFiManagerParameter custom_led_gpio(
      "led_gpio", "<b>GPIO</b><br>of internal LED", led_gpio, 3);
  WiFiManagerParameter custom_led_gpio_i(
      "led_gpio_i",
      "<b>GPIO is inverted</b><br><code>true</code> or <code>false</code>",
      led_gpio_i, 6);
  WiFiManagerParameter custom_shelly_mac(
      "mac",
      "<b>Shelly ID</b><br>12 char hexadecimal, defaults to MAC address of ESP",
      shelly_mac, 13);
  WiFiManagerParameter custom_shelly_port(
      "shelly_port",
      "<b>Shelly UDP port</b><br><code>1010</code> for old Marstek FW, "
      "<code>2220</code> for new Marstek FW v226+/v108+",
      shelly_port, 6);
  WiFiManagerParameter custom_force_pwr_decimals(
      "force_pwr_decimals",
      "<b>Force decimals numbers for Power values</b><br><code>true</code> to "
      "fix Marstek bug",
      force_pwr_decimals, 6);
  WiFiManagerParameter custom_sma_id(
      "sma_id",
      "<b>SMA serial number</b><br>optional serial number if you have more "
      "than one SMA EM/HM in your network",
      sma_id, 16);
  WiFiManagerParameter custom_section2("<hr><h3>MQTT options</h3>");
  WiFiManagerParameter custom_mqtt_topic("topic", "<b>MQTT Topic</b>",
                                         mqtt_topic, 90);
  WiFiManagerParameter custom_mqtt_user("user", "<b>MQTT user</b><br>optional",
                                        mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_passwd(
      "passwd", "<b>MQTT password</b><br>optional", mqtt_passwd, 40);
  WiFiManagerParameter custom_section3("<hr><h3>Modbus TCP options</h3>");
  WiFiManagerParameter custom_modbus_dev(
      "modbus_dev", "<b>Modbus device ID</b><br><code>71</code> for Kostal SEM",
      modbus_dev, 60);
  WiFiManagerParameter custom_section4(
      "<hr><h3>JSON paths for MQTT and generic HTTP</h3>");
  WiFiManagerParameter custom_power_path(
      "power_path",
      "<b>Total power JSON path</b><br>e.g. <code>ENERGY.Power</code> or "
      "<code>TRIPHASE</code> for tri-phase data",
      power_path, 60);
  WiFiManagerParameter custom_pwr_export_path(
      "pwr_export_path",
      "<b>Export power JSON path</b><br>Optional, for net calc (e.g. \"i-e\"",
      pwr_export_path, 60);
  WiFiManagerParameter custom_power_l1_path(
      "power_l1_path", "<b>Phase 1 power JSON path</b><br>optional",
      power_l1_path, 60);
  WiFiManagerParameter custom_power_l2_path(
      "power_l2_path",
      "<b>Phase 2 power JSON path</b><br>Phase 2 power JSON path<br>optional",
      power_l2_path, 60);
  WiFiManagerParameter custom_power_l3_path(
      "power_l3_path",
      "<b>Phase 3 power JSON path</b><br>Phase 3 power JSON path<br>optional",
      power_l3_path, 60);
  WiFiManagerParameter custom_energy_in_path(
      "energy_in_path",
      "<b>Energy from grid JSON path</b><br>e.g. <code>ENERGY.Grid</code>",
      energy_in_path, 60);
  WiFiManagerParameter custom_energy_out_path(
      "energy_out_path",
      "<b>Energy to grid JSON path</b><br>e.g. <code>ENERGY.FeedIn</code>",
      energy_out_path, 60);

  WiFiManager wifiManager;
  if (!DEBUG) {
    wifiManager.setDebugOutput(false);
  }
  wifiManager.setTitle("Energy2Shelly for ESP");
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // add all your parameters here
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

  // read updated parameters
  strncpy(input_type, custom_input_type.getValue(), sizeof(input_type) - 1);
  input_type[sizeof(input_type) - 1] = '\0';
  strncpy(mqtt_server, custom_mqtt_server.getValue(), sizeof(mqtt_server) - 1);
  mqtt_server[sizeof(mqtt_server) - 1] = '\0';

  // Manual trim for mqtt_server
  char *trimmedMqtt = mqtt_server;
  while (isspace(*trimmedMqtt))
    trimmedMqtt++;
  if (trimmedMqtt != mqtt_server) {
    memmove(mqtt_server, trimmedMqtt, strlen(trimmedMqtt) + 1);
  }
  char *endMqtt = mqtt_server + strlen(mqtt_server) - 1;
  while (endMqtt >= mqtt_server && isspace(*endMqtt))
    *endMqtt-- = '\0';

  strncpy(mqtt_port, custom_mqtt_port.getValue(), sizeof(mqtt_port) - 1);
  mqtt_port[sizeof(mqtt_port) - 1] = '\0';
  mqttPortInt = atoi(mqtt_port);
  strncpy(query_period, custom_query_period.getValue(),
          sizeof(query_period) - 1);
  query_period[sizeof(query_period) - 1] = '\0';
  queryPeriodMs = atoi(query_period);
  strncpy(led_gpio, custom_led_gpio.getValue(), sizeof(led_gpio) - 1);
  led_gpio[sizeof(led_gpio) - 1] = '\0';
  ledGpioInt = atoi(led_gpio);
  strncpy(led_gpio_i, custom_led_gpio_i.getValue(), sizeof(led_gpio_i) - 1);
  led_gpio_i[sizeof(led_gpio_i) - 1] = '\0';
  ledInverted = (strcmp(led_gpio_i, "true") == 0);
  strncpy(shelly_mac, custom_shelly_mac.getValue(), sizeof(shelly_mac) - 1);
  shelly_mac[sizeof(shelly_mac) - 1] = '\0';
  strncpy(mqtt_topic, custom_mqtt_topic.getValue(), sizeof(mqtt_topic) - 1);
  mqtt_topic[sizeof(mqtt_topic) - 1] = '\0';
  strncpy(mqtt_user, custom_mqtt_user.getValue(), sizeof(mqtt_user) - 1);
  mqtt_user[sizeof(mqtt_user) - 1] = '\0';
  strncpy(mqtt_passwd, custom_mqtt_passwd.getValue(), sizeof(mqtt_passwd) - 1);
  mqtt_passwd[sizeof(mqtt_passwd) - 1] = '\0';
  strncpy(modbus_dev, custom_modbus_dev.getValue(), sizeof(modbus_dev) - 1);
  modbus_dev[sizeof(modbus_dev) - 1] = '\0';
  modbusDeviceId = atoi(modbus_dev);
  strncpy(power_path, custom_power_path.getValue(), sizeof(power_path) - 1);
  power_path[sizeof(power_path) - 1] = '\0';
  strncpy(pwr_export_path, custom_pwr_export_path.getValue(),
          sizeof(pwr_export_path) - 1);
  pwr_export_path[sizeof(pwr_export_path) - 1] = '\0';
  strncpy(power_l1_path, custom_power_l1_path.getValue(),
          sizeof(power_l1_path) - 1);
  power_l1_path[sizeof(power_l1_path) - 1] = '\0';
  strncpy(power_l2_path, custom_power_l2_path.getValue(),
          sizeof(power_l2_path) - 1);
  power_l2_path[sizeof(power_l2_path) - 1] = '\0';
  strncpy(power_l3_path, custom_power_l3_path.getValue(),
          sizeof(power_l3_path) - 1);
  power_l3_path[sizeof(power_l3_path) - 1] = '\0';
  strncpy(energy_in_path, custom_energy_in_path.getValue(),
          sizeof(energy_in_path) - 1);
  energy_in_path[sizeof(energy_in_path) - 1] = '\0';
  strncpy(energy_out_path, custom_energy_out_path.getValue(),
          sizeof(energy_out_path) - 1);
  energy_out_path[sizeof(energy_out_path) - 1] = '\0';
  strncpy(shelly_port, custom_shelly_port.getValue(), sizeof(shelly_port) - 1);
  shelly_port[sizeof(shelly_port) - 1] = '\0';
  shellyPortInt = atoi(shelly_port);
  strncpy(force_pwr_decimals, custom_force_pwr_decimals.getValue(),
          sizeof(force_pwr_decimals) - 1);
  force_pwr_decimals[sizeof(force_pwr_decimals) - 1] = '\0';
  forcePwrDecimals = (strcmp(force_pwr_decimals, "true") == 0);
  strncpy(sma_id, custom_sma_id.getValue(), sizeof(sma_id) - 1);
  sma_id[sizeof(sma_id) - 1] = '\0';

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
