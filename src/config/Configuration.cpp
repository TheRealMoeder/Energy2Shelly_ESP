#include "Configuration.h"

unsigned long startMillis = 0;
unsigned long currentMillis;
// for time synchronization
time_t now;
tm timeinfo;

// ============================================================================
// CONFIGURATION VARIABLES (stored in Preferences)
// ============================================================================

// Data source and server settings
char input_type[40];
char ntp_server[40] = "de.pool.ntp.org";
char timezone[64] = "CET-1CEST,M3.5.0/2,M10.5.0/3"; // Central European Time
char mqtt_server[160];
char mqtt_port[6] = "1883";
char mqtt_topic[90] = "tele/meter/SENSOR";
char mqtt_user[40] = "";
char mqtt_passwd[40] = "";

// JSON path settings
char power_path[60] = "";
char pwr_export_path[60] = "";
char power_l1_path[60] = "";
char power_l2_path[60] = "";
char power_l3_path[60] = "";
char energy_in_path[60] = "";
char energy_out_path[60] = "";

// Shelly device settings
char shelly_gen[2] = "2";
char shelly_fw_id[32] = "20241011-114455/1.4.4-g6d2a586";
char shelly_mac[13];
char shelly_name[26] = "shellypro3em-";
char shelly_port[6] = "2220"; // old: 1010; new (FW>=226): 2220; Venus A and E 3.0 use 1010 again

// Query and protocol settings
char query_period[10] = "1000";
char modbus_dev[10] = "71"; // default for KSEM
char force_pwr_decimals[6] = "true"; // to fix Marstek bug
bool forcePwrDecimals = true; // to fix Marstek bug
char sma_id[17] = "";

// LED settings
char led_gpio[3] = "";
char led_gpio_i[6];
unsigned long ledOffTime = 0;
uint8_t led = 0;
bool led_i = false;
const uint8_t ledblinkduration = 50;

// SMA Multicast IP and Port
unsigned int multicastPort = 9522;  // local port to listen on
IPAddress multicastIP(239, 12, 255, 254);

// MODBUS settings
IPAddress modbus_ip;
ModbusIP modbus1;
int16_t modbus_result[256];

// Default electrical values
const uint8_t defaultVoltage = 230;
const uint8_t defaultFrequency = 50;
const uint8_t defaultPowerFactor = 1;

// RPC and query settings
unsigned long period = 1000;
int rpcId = 1;
char rpcUser[20] = "user_1";

// flags for saving/resetting WifiManager data
bool shouldSaveConfig = false;
bool shouldResetConfig = false;

// flags for data sources
bool dataMQTT = false;
bool dataSMA = false;
bool dataSHRDZM = false;
bool dataHTTP = false;
bool dataSUNSPEC = false;

Preferences preferences;

// ============================================================================
// NETWORK OBJECTS
// ============================================================================

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
AsyncWebServer server(80);
AsyncWebSocket webSocket("/rpc");
WiFiUDP Udp;
HTTPClient http;
WiFiUDP UdpRPC;

// ============================================================================
// MDNS RESPONDER HANDLES (ESP8266 only)
// ============================================================================

#ifndef ESP32
  MDNSResponder::hMDNSService hMDNSService = 0; // handle of the http service in the MDNS responder
  MDNSResponder::hMDNSService hMDNSService2 = 0; // handle of the shelly service in the MDNS responder
#endif

// Blink LED handlers
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

//callback notifying us of the need to save WifiManager config
void saveConfigCallback() {
  DEBUG_SERIAL.println("Should save config");
  shouldSaveConfig = true;
}

void WifiManagerSetup() {
  // Set Shelly ID to ESP's MAC address by default
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(shelly_mac, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  preferences.begin("e2s_config", false);
  strcpy(input_type, preferences.getString("input_type", input_type).c_str());
  strcpy(mqtt_server, preferences.getString("mqtt_server", mqtt_server).c_str());
  strcpy(ntp_server, preferences.getString("ntp_server", ntp_server).c_str());
  strcpy(timezone, preferences.getString("timezone", timezone).c_str());
  strcpy(query_period, preferences.getString("query_period", query_period).c_str());
  strcpy(led_gpio, preferences.getString("led_gpio", led_gpio).c_str());
  strcpy(led_gpio_i, preferences.getString("led_gpio_i", led_gpio_i).c_str());
  strcpy(shelly_mac, preferences.getString("shelly_mac", shelly_mac).c_str());
  strcpy(mqtt_port, preferences.getString("mqtt_port", mqtt_port).c_str());
  strcpy(mqtt_topic, preferences.getString("mqtt_topic", mqtt_topic).c_str());
  strcpy(mqtt_user, preferences.getString("mqtt_user", mqtt_user).c_str());
  strcpy(mqtt_passwd, preferences.getString("mqtt_passwd", mqtt_passwd).c_str());
  strcpy(modbus_dev, preferences.getString("modbus_dev", modbus_dev).c_str());
  strcpy(power_path, preferences.getString("power_path", power_path).c_str());
  strcpy(pwr_export_path, preferences.getString("pwr_export_path", pwr_export_path).c_str());
  strcpy(power_l1_path, preferences.getString("power_l1_path", power_l1_path).c_str());
  strcpy(power_l2_path, preferences.getString("power_l2_path", power_l2_path).c_str());
  strcpy(power_l3_path, preferences.getString("power_l3_path", power_l3_path).c_str());
  strcpy(energy_in_path, preferences.getString("energy_in_path", energy_in_path).c_str());
  strcpy(energy_out_path, preferences.getString("energy_out_path", energy_out_path).c_str());
  strcpy(shelly_port, preferences.getString("shelly_port", shelly_port).c_str());
  strcpy(force_pwr_decimals, preferences.getString("force_pwr_decimals", force_pwr_decimals).c_str());
  strcpy(sma_id, preferences.getString("sma_id", sma_id).c_str());
  
  WiFiManagerParameter custom_section1("<h3>General settings</h3>");
  WiFiManagerParameter custom_input_type("type", "<b>Data source</b><br><code>MQTT</code> for MQTT<br><code>HTTP</code> for generic HTTP<br><code>SMA</code> for SMA EM/HM multicast<br><code>SHRDZM</code> for SHRDZM UDP data<br><code>SUNSPEC</code> for Modbus TCP SUNSPEC data", input_type, 40);
  WiFiManagerParameter custom_mqtt_server("server", "<b>Server</b><br>MQTT Server IP, query url for generic HTTP or Modbus TCP server IP for SUNSPEC", mqtt_server, 160);
  WiFiManagerParameter custom_mqtt_port("port", "<b>Port</b><br> for MQTT or Modbus TCP (SUNSPEC)", mqtt_port, 6);
  WiFiManagerParameter param_ntp_server("ntp_server", "NTP server <span title=\"for time synchronization\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", ntp_server, 40);
  WiFiManagerParameter param_timezone("timezone", "Timezone <span title=\"e.g. UTC0, UTC+1, UTC-3, UTC+1CET-1CEST,M3.5.0/02:00:00,M10.5.0/03:00:00\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", timezone, 64);
  WiFiManagerParameter custom_query_period("query_period", "<b>Query period</b><br>for generic HTTP and SUNSPEC, in milliseconds", query_period, 10);
  WiFiManagerParameter custom_led_gpio("led_gpio", "<b>GPIO</b><br>of internal LED", led_gpio, 3);
  WiFiManagerParameter custom_led_gpio_i("led_gpio_i", "<b>GPIO is inverted</b><br><code>true</code> or <code>false</code>", led_gpio_i, 6);
  WiFiManagerParameter custom_shelly_mac("mac", "<b>Shelly ID</b><br>12 char hexadecimal, defaults to MAC address of ESP", shelly_mac, 13);
  WiFiManagerParameter custom_shelly_port("shelly_port", "<b>Shelly UDP port</b><br><code>1010</code> or <code>2220</code> depending on Marstek Venus model and firmware version", shelly_port, 6);
  WiFiManagerParameter custom_force_pwr_decimals("force_pwr_decimals", "<b>Force decimals numbers for Power values</b><br><code>true</code> to fix Marstek bug", force_pwr_decimals, 6);
  WiFiManagerParameter custom_sma_id("sma_id", "<b>SMA serial number</b><br>optional serial number if you have more than one SMA EM/HM in your network", sma_id, 16);
  WiFiManagerParameter custom_section2("<hr><h3>MQTT options</h3>");
  WiFiManagerParameter custom_mqtt_topic("topic", "<b>MQTT Topic</b>", mqtt_topic, 90);
  WiFiManagerParameter custom_mqtt_user("user", "<b>MQTT user</b><br>optional", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_passwd("passwd", "<b>MQTT password</b><br>optional", mqtt_passwd, 40);
  WiFiManagerParameter custom_section3("<hr><h3>Modbus TCP options</h3>");
  WiFiManagerParameter custom_modbus_dev("modbus_dev", "<b>Modbus device ID</b><br><code>71</code> for Kostal SEM", modbus_dev, 60);
  WiFiManagerParameter custom_section4("<hr><h3>JSON paths for MQTT and generic HTTP</h3>");
  WiFiManagerParameter custom_power_path("power_path", "<b>Total power JSON path</b><br>e.g. <code>ENERGY.Power</code> or <code>TRIPHASE</code> for tri-phase data", power_path, 60);
  WiFiManagerParameter custom_pwr_export_path("pwr_export_path", "<b>Export power JSON path</b><br>Optional, for net calc (e.g. \"i-e\"", pwr_export_path, 60);
  WiFiManagerParameter custom_power_l1_path("power_l1_path", "<b>Phase 1 power JSON path</b><br>optional", power_l1_path, 60);
  WiFiManagerParameter custom_power_l2_path("power_l2_path", "<b>Phase 2 power JSON path</b><br>Phase 2 power JSON path<br>optional", power_l2_path, 60);
  WiFiManagerParameter custom_power_l3_path("power_l3_path", "<b>Phase 3 power JSON path</b><br>Phase 3 power JSON path<br>optional", power_l3_path, 60);
  WiFiManagerParameter custom_energy_in_path("energy_in_path", "<b>Energy from grid JSON path</b><br>e.g. <code>ENERGY.Grid</code>", energy_in_path, 60);
  WiFiManagerParameter custom_energy_out_path("energy_out_path", "<b>Energy to grid JSON path</b><br>e.g. <code>ENERGY.FeedIn</code>", energy_out_path, 60);

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
  wifiManager.addParameter(&param_ntp_server);
  wifiManager.addParameter(&param_timezone);
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
  

  if (!wifiManager.autoConnect("Energy2Shelly")) {
    DEBUG_SERIAL.println("failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
    delay(5000);
  }
  DEBUG_SERIAL.println("connected");

  //read updated parameters
  strcpy(input_type, custom_input_type.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(ntp_server, param_ntp_server.getValue());
  strcpy(timezone, param_timezone.getValue());
  strcpy(query_period, custom_query_period.getValue());
  strcpy(led_gpio, custom_led_gpio.getValue());
  strcpy(led_gpio_i, custom_led_gpio_i.getValue());
  strcpy(shelly_mac, custom_shelly_mac.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_passwd, custom_mqtt_passwd.getValue());
  strcpy(modbus_dev, custom_modbus_dev.getValue());
  strcpy(power_path, custom_power_path.getValue());
  strcpy(pwr_export_path, custom_pwr_export_path.getValue());
  strcpy(power_l1_path, custom_power_l1_path.getValue());
  strcpy(power_l2_path, custom_power_l2_path.getValue());
  strcpy(power_l3_path, custom_power_l3_path.getValue());
  strcpy(energy_in_path, custom_energy_in_path.getValue());
  strcpy(energy_out_path, custom_energy_out_path.getValue());
  strcpy(shelly_port, custom_shelly_port.getValue());
  strcpy(force_pwr_decimals, custom_force_pwr_decimals.getValue());
  strcpy(sma_id, custom_sma_id.getValue());

  DEBUG_SERIAL.println("The values in the preferences are: ");
  DEBUG_SERIAL.println("\tinput_type : " + String(input_type));
  DEBUG_SERIAL.println("\tmqtt_server : " + String(mqtt_server));
  DEBUG_SERIAL.println("\tmqtt_port : " + String(mqtt_port));
  DEBUG_SERIAL.println("\tntp_server: " + String(ntp_server));
  DEBUG_SERIAL.println("\ttimezone: " + String(timezone));
  DEBUG_SERIAL.println("\tquery_period : " + String(query_period));
  DEBUG_SERIAL.println("\tled_gpio : " + String(led_gpio));
  DEBUG_SERIAL.println("\tled_gpio_i : " + String(led_gpio_i));
  DEBUG_SERIAL.println("\tshelly_mac : " + String(shelly_mac));
  DEBUG_SERIAL.println("\tmqtt_topic : " + String(mqtt_topic));
  DEBUG_SERIAL.println("\tmqtt_user : " + String(mqtt_user));
  DEBUG_SERIAL.println("\tmqtt_passwd : " + String(mqtt_passwd));
  DEBUG_SERIAL.println("\tmodbus_dev : " + String(modbus_dev));
  DEBUG_SERIAL.println("\tpower_path : " + String(power_path));
  DEBUG_SERIAL.println("\tpwr_export_path : " + String(pwr_export_path));
  DEBUG_SERIAL.println("\tpower_l1_path : " + String(power_l1_path));
  DEBUG_SERIAL.println("\tpower_l2_path : " + String(power_l2_path));
  DEBUG_SERIAL.println("\tpower_l3_path : " + String(power_l3_path));
  DEBUG_SERIAL.println("\tenergy_in_path : " + String(energy_in_path));
  DEBUG_SERIAL.println("\tenergy_out_path : " + String(energy_out_path));
  DEBUG_SERIAL.println("\tshelly_port : " + String(shelly_port));
  DEBUG_SERIAL.println("\tforce_pwr_decimals : " + String(force_pwr_decimals));
  DEBUG_SERIAL.println("\tsma_id : " + String(sma_id));

  if (strcmp(input_type, "SMA") == 0) {
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
  else {
    dataMQTT = true;
    DEBUG_SERIAL.println("Enabling MQTT data input");
  }

  if (strcmp(led_gpio_i, "true") == 0) {
    led_i = true;
  } else {
    led_i = false;
  }

  if (strcmp(force_pwr_decimals, "true") == 0) {
    forcePwrDecimals = true;
  } else {
    forcePwrDecimals = false;
  }

  if (shouldSaveConfig) {
    DEBUG_SERIAL.println("saving config");
    preferences.putString("input_type", input_type);
    preferences.putString("mqtt_server", mqtt_server);
    preferences.putString("mqtt_port", mqtt_port);
    preferences.putString("ntp_server", ntp_server);
    preferences.putString("timezone", timezone);
    preferences.putString("query_period", query_period);
    preferences.putString("led_gpio", led_gpio);
    preferences.putString("led_gpio_i", led_gpio_i);
    preferences.putString("shelly_mac", shelly_mac);
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
    preferences.putString("shelly_port", shelly_port);
    preferences.putString("force_pwr_decimals", force_pwr_decimals);
    preferences.putString("sma_id", sma_id);
    wifiManager.reboot();
  }
  DEBUG_SERIAL.println("local ip");
  DEBUG_SERIAL.println(WiFi.localIP());
}

void setupMdns() {
  // Set up mDNS responder
  strncat(shelly_name, shelly_mac, sizeof(shelly_name) - strlen(shelly_name) - 1);

  if (!MDNS.begin(shelly_name)) {
    DEBUG_SERIAL.println("Error setting up MDNS responder!");
  }

#ifdef ESP32
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("shelly", "tcp", 80);
  mdns_txt_item_t serviceTxtData[4] = {
    { "id", shelly_name },
    { "fw_id", shelly_fw_id },
    { "gen", shelly_gen },
    { "arch", "esp8266" }
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
}
