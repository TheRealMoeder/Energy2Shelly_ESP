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
#define WM_MDNS  
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



extern bool enableConsoleOutput; // Global flag to control console output visibility
extern AsyncWebSocket wsConsole;
// Custom Logger class inheriting from Print to natively support all data types
class WebDebugLogger : public Print {
private:
    // Safe static size inside the global BSS RAM segment (0 bytes on Stack)
    #ifdef ESP32
    static const size_t QUEUE_SIZE = 4096; // allow more queue size for ESP32
    #else
    static const size_t QUEUE_SIZE = 1024;
    #endif
    char queue[QUEUE_SIZE];
    volatile size_t head = 0;
    volatile size_t tail = 0;
    volatile size_t droppedBytes = 0;

    // Calculates remaining capacity in the ring buffer
    size_t getFreeSpace() {
        if (head >= tail) {
            return QUEUE_SIZE - (head - tail) - 1;
        }
        return tail - head - 1;
    }

public:
    void begin(unsigned long baud) {
        Serial.begin(baud);
    }

    // Instantly flushes the buffer pointers
    void clearQueue() {
        tail = head; 
        droppedBytes = 0;
    }

    // Intercepts character data from print/println calls
    virtual size_t write(uint8_t character) override {
        // Local hardware USB serial transmission remains direct and synchronous
        Serial.write(character);

        // Global runtime visibility check
        if (!enableConsoleOutput) return 1;

        // If no browser clients are connected, discard data to save CPU cycles
        if (wsConsole.count() == 0) {
            if (head != tail) clearQueue();
            return 1;
        }

        // Add character to queue if space permits
        if (getFreeSpace() > 0) {
            
            // Check if we need to inject a safe dropped-bytes warning message
            if (droppedBytes > 0 && getFreeSpace() > 45) {
                char overflowMsg[45];
                int len = snprintf(overflowMsg, sizeof(overflowMsg), "\n[Logger Overflow: %u bytes dropped]\n", droppedBytes);
                
                for (int i = 0; i < len; i++) {
                    queue[head] = overflowMsg[i];
                    head = (head + 1) % QUEUE_SIZE;
                }
                droppedBytes = 0; 
            }

            queue[head] = (char)character;
            head = (head + 1) % QUEUE_SIZE;
        } else {
            // Memory safe guard: track drop count instead of corrupting data positions
            droppedBytes=droppedBytes+1;
        }
        return 1;
    }

    // Asynchronously processes and flushes data to WebSockets via the main loop
    void handleQueue() {
        if (wsConsole.count() == 0) {
            if (head != tail) clearQueue();
            return;
        }

        if (head == tail) return;
        if (!wsConsole.availableForWriteAll()) return; 

        size_t currentTail = tail;
        size_t currentHead = head;

        // 1. Calculate how many bytes are available sequentially until the array boundary
        size_t maxLinearBytes = (currentHead >= currentTail) ? (currentHead - currentTail) : (QUEUE_SIZE - currentTail);
        
        size_t sendLength = 0;
        bool foundNewline = false;

        // 2. Scan the linear chunk for a newline character
        for (size_t i = 0; i < maxLinearBytes; i++) {
            sendLength++;
            if (queue[currentTail + i] == '\n') { 
                foundNewline = true;
                break; 
            }
        }

        // 3. FIXED STRATEGY:
        // If NO newline is found up to the array boundary, we must check if a newline 
        // exists further ahead (wrapped around index 0).
        if (!foundNewline) {
            // Calculate total unread bytes in the entire buffer (including wrap-around)
            size_t totalBytes = (currentHead >= currentTail) ? (currentHead - currentTail) : (QUEUE_SIZE - currentTail + currentHead);
            
            bool newlineExistsAhead = false;
            for (size_t i = 0; i < totalBytes; i++) {
                if (queue[(currentTail + i) % QUEUE_SIZE] == '\n') {
                    newlineExistsAhead = true;
                    break;
                }
            }

            // If a newline exists somewhere ahead, we are allowed to send the current 
            // linear chunk up to the array boundary right now! The browser will receive 
            // the first half, and the loop will stream the remaining half from index 0 
            // on the very next iteration—keeping the output clean and perfectly intact.
            if (newlineExistsAhead) {
                sendLength = maxLinearBytes;
            } else {
                // If there is no newline anywhere in the entire buffer, the line is still 
                // actively being written by the system. We must wait.
                return; 
            }
        }

        // 4. Safely dispatch the verified chunk directly from memory
        if (sendLength > 0) {
            wsConsole.textAll(&(queue[currentTail]), sendLength);
            tail = (currentTail + sendLength) % QUEUE_SIZE; // Naturally wraps to 0 if sendLength == maxLinearBytes
        }
    }
};
#if DEBUG
    #define DEBUG_SERIAL DebugConsole
    extern WebDebugLogger DebugConsole;
#else
    // If DEBUG is false, dummy class removes all strings from memory entirely
    class NullDebug : public Print { 
      public: virtual size_t write(uint8_t c) override { return 1; }
      void begin(unsigned long baud)  {}
      void handleQueue() {}
    };
    extern NullDebug EmptyConsole;
    #define DEBUG_SERIAL EmptyConsole
#endif

//#define DEBUG_SERIAL if(DEBUG)Serial

extern unsigned long startMillis;
extern unsigned long currentMillis;
// for time synchronization
extern time_t now;
extern tm timeinfo;
extern  JsonDocument config_doc;

// ============================================================================
// CONFIGURATION VARIABLES (stored in Preferences)
// ============================================================================

// Data source and server settings
#define reset_password config_doc["reset_password"].as<const char*>()
#define input_type config_doc["input_type"].as<const char*>()
#define ntp_server config_doc["ntp_server"].as<const char*>()
#define timezone config_doc["timezone"].as<const char*>() 
#define phase_number config_doc["phase_number"].as<const char*>()
#define power_offset config_doc["power_offset"].as <const char*>()
#define mqtt_server config_doc["mqtt_server"].as<const char*>()
#define mqtt_port config_doc["mqtt_port"].as<const char*>()
#define mqtt_topic config_doc["mqtt_topic"].as<const char*>()
#define mqtt_user config_doc["mqtt_user"].as<const char*>()
#define mqtt_passwd config_doc["mqtt_password"].as<const char*>()

// JSON path settings
#define power_path config_doc["power_path"].as<const char*>()
#define pwr_export_path config_doc["pwr_export_path"].as<const char*>()
#define power_l1_path config_doc["power_l1_path"].as<const char*>()
#define power_l2_path config_doc["power_l2_path"].as<const char*>()
#define power_l3_path config_doc["power_l3_path"].as<const char*>()
#define energy_in_path config_doc["energy_in_path"].as<const char*>()
#define energy_out_path config_doc["energy_out_path"].as<const char*>()

// Shelly device settings
#define shelly_gen config_doc["shelly_gen"].as<const char*>()
#define shelly_fw_id config_doc["shelly_fw_id"].as<const char*>()
#define shelly_mac config_doc["shelly_mac"].as<const char*>()
#define shelly_name config_doc["shelly_name"].as<const char*>()
#define shelly_port config_doc["shelly_port"].as<const char*>()

// Query and protocol settings
#define query_period config_doc["query_period"].as<const char*>()
#define modbus_dev config_doc["modbus_dev"].as<const char*>()
#define sma_id config_doc["sma_id"].as<const char*>()
extern uint16_t sunspec_port_int; // default port
extern uint8_t modbusdev_int; 

// RCT Power settings
#define rct_host config_doc["rct_host"].as<const char*>()
#define rct_port config_doc["rct_port"].as<const char*>()

#define tibber_host config_doc["tibber_host"].as<const char*>()
#define tibber_user config_doc["tibber_user"].as<const char*>()
#define tibber_password config_doc["tibber_password"].as<const char*>()
#define tibber_nodeid config_doc["tibber_nodeid"].as<const char*>()
extern char tibber_rpc[2][26];
// LED settings
#define led_gpio config_doc["led_gpio"].as<const char*>()
#define led_gpio_i config_doc["led_gpio_i"].as<const char*>()

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

// Default electrical values and offset
extern const uint8_t defaultVoltage;
extern const uint8_t defaultFrequency;
extern const uint8_t defaultPowerFactor;
extern double offsetPerPhase;

// RPC and query settings
extern unsigned long period;
extern int rpcId;
extern char rpcUser[20];

// flags for saving/resetting WifiManager data
extern bool shouldSaveConfig;
extern bool shouldResetConfig;
extern bool shouldupdate;

// flags for data sources
extern bool dataMQTT;
extern bool dataSMA;
extern bool dataSHRDZM;
extern bool dataHTTP;
extern bool dataSUNSPEC;
extern bool dataTIBBERPULSE;
extern bool dataRCT;

extern Preferences preferences;


// ============================================================================
// NETWORK OBJECTS
// ============================================================================

extern WiFiClient wifi_client;
extern PubSubClient mqtt_client;
extern AsyncWebServer server;
extern AsyncWebSocket webSocket;
extern uint16_t wifi_reconnect_attempts;

extern WiFiUDP Udp;
extern HTTPClient http;
extern WiFiUDP UdpRPC;
extern void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

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

void blinkled(int duration);
void handleblinkled();
void saveConfigCallback();
void WifiManagerSetup(bool stationmode);
void setupMdns();
void interprete_parameters();

#endif // CONFIGURATION_H
