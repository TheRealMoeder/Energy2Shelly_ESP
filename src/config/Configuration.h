#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <Preferences.h>

// Fix WifiManager/WebServer conflicts
#ifndef ESP32
  #define WEBSERVER_H "fix WifiManager conflict"
#endif

// Platform-specific includes
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

// Web & JSON libraries
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>
#include <ModbusIP_ESP8266.h>

// ============================================================================
// DEBUG & TIMING
// ============================================================================

#define DEBUG true // set to false for no DEBUG output
#define DEBUG_SERIAL if(DEBUG)Serial

extern unsigned long startMillis;
extern unsigned long startMillis_sunspec;
extern unsigned long currentMillis;

// ============================================================================
// CONFIGURATION VARIABLES (stored in Preferences)
// ============================================================================

// Data source and server settings
extern char input_type[40];
extern char mqtt_server[160];
extern char mqtt_port[6];
extern char mqtt_topic[90];
extern char mqtt_user[40];
extern char mqtt_passwd[40];

// JSON path settings
extern char power_path[60];
extern char pwr_export_path[60];
extern char power_l1_path[60];
extern char power_l2_path[60];
extern char power_l3_path[60];
extern char energy_in_path[60];
extern char energy_out_path[60];

// Shelly device settings
extern char shelly_gen[2];
extern char shelly_fw_id[32];
extern char shelly_mac[13];
extern char shelly_name[26];
extern char shelly_port[6];

// Query and protocol settings
extern char query_period[10];
extern char modbus_dev[10];
extern char force_pwr_decimals[6];
extern bool forcePwrDecimals;
extern char sma_id[17];

// LED settings
extern char led_gpio[3];
extern char led_gpio_i[6];
extern unsigned long ledOffTime;
extern uint8_t led;
extern bool led_i;
extern const uint8_t ledblinkduration;

// SMA Multicast IP and Port
extern unsigned int multicastPort;  // local port to listen on
extern IPAddress multicastIP;

// MODBUS settings
extern IPAddress modbus_ip;
extern ModbusIP modbus1;
extern int16_t modbus_result[256];

// Default electrical values
extern const uint8_t defaultVoltage;
extern const uint8_t defaultFrequency;
extern const uint8_t defaultPowerFactor;

// RPC and query settings
extern unsigned long period;
extern int rpcId;
extern char rpcUser[20];

// flags for saving/resetting WifiManager data
extern bool shouldSaveConfig;
extern bool shouldResetConfig;

// flags for data sources
extern bool dataMQTT;
extern bool dataSMA;
extern bool dataSHRDZM;
extern bool dataHTTP;
extern bool dataSUNSPEC;

extern Preferences preferences;


// ============================================================================
// NETWORK OBJECTS
// ============================================================================

extern WiFiClient wifi_client;
extern PubSubClient mqtt_client;
extern AsyncWebServer server;
extern AsyncWebSocket webSocket;
extern WiFiUDP Udp;
extern HTTPClient http;
extern WiFiUDP UdpRPC;

// Platform-specific UDP print macro
#ifdef ESP32
#define UDPPRINT print
#else
#define UDPPRINT write
#endif

// ============================================================================
// MDNS RESPONDER HANDLES (ESP8266 only)
// ============================================================================

#ifndef ESP32
  extern MDNSResponder::hMDNSService hMDNSService; // handle of the http service in the MDNS responder
  extern MDNSResponder::hMDNSService hMDNSService2; // handle of the shelly service in the MDNS responder
#endif

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void saveConfigCallback();
void WifiManagerSetup();

#endif // CONFIGURATION_H