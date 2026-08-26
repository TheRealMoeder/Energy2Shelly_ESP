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



extern bool enableConsoleOutput; // Global flag to control console output visibility
extern AsyncWebSocket wsConsole;
// Custom Logger class inheriting from Print to natively support all data types
class WebDebugLogger : public Print {
private:
    // Safe static size inside the global BSS RAM segment (0 bytes on Stack)
    static const size_t QUEUE_SIZE = 1024; 
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
            droppedBytes++;
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

// ============================================================================
// CONFIGURATION VARIABLES (stored in Preferences)
// ============================================================================

// Data source and server settings
extern char reset_password[33];
extern char input_type[40];
extern char ntp_server[40];
extern char timezone[64];
extern char phase_number[2];
extern char power_offset[10];
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
extern char sma_id[17];
extern uint16_t sunspec_port_int; // default port
extern uint8_t modbusdev_int; // default device id for KSEM


extern char tibber_host[41];
extern char tibber_user[11];
extern char tibber_password[10];
extern char tibber_rpc[21];
extern char tibber_nodeid[2];

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

// flags for data sources
extern bool dataMQTT;
extern bool dataSMA;
extern bool dataSHRDZM;
extern bool dataHTTP;
extern bool dataSUNSPEC;
extern bool dataTIBBERPULSE;

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
void WifiManagerSetup();
void setupMdns();

#endif // CONFIGURATION_H
