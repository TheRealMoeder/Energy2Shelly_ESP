#include <Arduino.h>

#ifdef ESP32
#include "esp_task_wdt.h"
#endif

// Configuration & setup
#include "config/Configuration.h"
// Data structures & processing
#include "data/DataStructures.h"
#include "data/DataProcessing.h"

// Protocol parsers
#include "parsers/Parsers.h"

// RPC handlers
#include "rpc/RpcHandlers.h"
#include "rpc/RpcComm.h"

// Web content
#include "web/html_home.h"

// Utils function for monitoring and reset
#include "utils/utils.h"

#ifdef ESP32
#include "dst/ortsnetzauslastung.h"
#endif

#define WiFicheckInterval 60000 // Check every 60 seconds

unsigned long updateTimestamp = 0; 
bool parametersUpdated = false; 

bool isNotAuthorized(AsyncWebServerRequest *request) {
    String lp=(config_doc["basic"]["reset_password"]|"");
    if (lp==NULL) return false;
    if (lp.length()==0) return false;
    if (!request->authenticate("", lp.c_str())) {
        request->requestAuthentication(); 
        return true;
    }
    return false;
}


void setup(void)
{
  DEBUG_SERIAL.begin(115200);
  WifiManagerSetup(clear_rtc_power_on() == Energy2Shelly_ResetReason::RECONFIGURE);
 
  // Initialize watchdog timer (30s timeout)
#ifdef ESP32
  // check SDK version.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  // ---new version ---
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 30000,                             // 30 seconds timeout
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // watch all idle tasks on all cpus
      .trigger_panic = true                            // true = restart if needed
  };
  // reconfigure WD
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);
#else
  // --- old SDK version
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);
#endif
#endif

  // Initialize time via NTP
#ifdef ESP32
  configTime(0, 0, ntp_server);
  setenv("TZ", timezone, 1);
  tzset();
#else
  // ESP8266
  configTime(timezone, ntp_server);
#endif
  uint32_t ntp_timeout = millis() + 10000;
  bool ntp_success = false;
  while (millis() < ntp_timeout)
  {
    if (getLocalTime(&timeinfo))
    {
      ntp_success = true;
      break;
    }
    DEBUG_SERIAL.println(F("Waiting for NTP time..."));
    delay(500);
  }
  if (!ntp_success)
  {
    DEBUG_SERIAL.println(F("NTP timeout - setting default time"));
    time_t default_time = 1704067200; // 2024-01-01 00:00:00 UTC
    struct timeval tv = {.tv_sec = default_time, .tv_usec = 0};
    settimeofday(&tv, NULL);
    getLocalTime(&timeinfo);
  }
  DEBUG_SERIAL.print(F("Current time: "));
  char time_buffer[20];
  strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  DEBUG_SERIAL.println(time_buffer);

  if (String(led_gpio).toInt() > 0)
  {
    led = String(led_gpio).toInt();
  }

  if (led > 0)
  {
    pinMode(led, OUTPUT);
    if (led_i)
    {
      digitalWrite(led, LOW);
    }
    else
    {
      digitalWrite(led, HIGH);
    }
  }

  // Set up web server and endpoints
  server.on("/", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    AsyncWebServerResponse *response = request->beginResponse("text/html", strlen_P(HTML_HOME), [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        // copy fragment bytes from PROGMEM to buffer, use less resources by copying only the requested fragment
        memcpy_P(buffer, HTML_HOME + index, maxLen); return maxLen;
    });
    request->send(response); });

  server.on("/plot", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    AsyncWebServerResponse *response = request->beginResponse("text/html", strlen_P(webplotter_html), [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        // copy fragment bytes from PROGMEM to buffer, use less resources by copying only the requested fragment
        memcpy_P(buffer, webplotter_html + index, maxLen); return maxLen;
    });
    request->send(response); });
 server.on("/chart.min.js", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse("application/javascript", chart_js_len, [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        size_t remaining = chart_js_len - index;
        size_t toCopy = (remaining < maxLen) ? remaining : maxLen;
        
        memcpy_P(buffer, chart_js + index, toCopy); 
        return toCopy;
    });
    
    response->addHeader("Content-Encoding", "gzip");
    request->send(response); 
});
  server.on("/favicon.ico", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
              request->send(204); // 204 "No Content"
            });
  //Reloads of wifi manager page, redirect to main page 
   server.on("/exit", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->redirect("/");
   });

   server.on("/close", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->redirect("/");
   });

  server.on("/console", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
      AsyncWebServerResponse *response = request->beginResponse("text/html", strlen_P(htmlPage_console), [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        // copy fragment bytes from PROGMEM to buffer, use less resources by copying only the requested fragment
        memcpy_P(buffer, htmlPage_console + index, maxLen); return maxLen;
    });
    request->send(response); });

  server.on("/config", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
      AsyncWebServerResponse *response = request->beginResponse("text/html", strlen_P(CONFIG_PAGE_HTML), [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        // copy fragment bytes from PROGMEM to buffer, use less resources by copying only the requested fragment
        memcpy_P(buffer, CONFIG_PAGE_HTML + index, maxLen); return maxLen;
    });
    request->send(response); });

    
  server.on("/shelly", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    shellyGetDeviceInfo();
    request->send(200, "application/json", serJsonResponse); });

  server.on("/status", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    shellyGetStatus();
    request->send(200, "application/json", serJsonResponse); });

  server.on("/reset", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    bool hasPassword = (reset_password != nullptr && strlen(reset_password) > 0);  
    
    size_t maxNeededLength = (sizeof(RESET_HTML) - 1) + (sizeof(RESET_HTML_END) - 1);
    
    if (hasPassword) {
        maxNeededLength += (sizeof(PASSWORD_INPUT_HTML) - 1);
    }   
    String html;
    html.reserve(maxNeededLength); 
    
    html += FPSTR(RESET_HTML);
    if (hasPassword) {
        html += FPSTR(PASSWORD_INPUT_HTML);
    }
    html += FPSTR(RESET_HTML_END);
    
    request->send(200, "text/html", html); });

  server.on("/reset", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request)
            {
    if (reset_password != nullptr && strlen(reset_password) > 0) {
        if (request->hasParam("reset_password", true)) {            
            const char* submitted_pw = request->getParam("reset_password", true)->value().c_str();
            
            if (strcmp(reset_password, submitted_pw) == 0) {
                shouldResetConfig = true;
                request->send(200, "text/plain", PSTR("Resetting WiFi configuration, please log back into the hotspot to reconfigure...\r\n"));
            } else {
                request->send(403, "text/plain", PSTR("Unauthorized: Invalid reset password.\r\n"));
            }
        } else {
            request->send(400, "text/plain", PSTR("Reset password missing.\r\n"));
        }
    } else {        
        shouldResetConfig = true;
        request->send(200, "text/plain", PSTR("Resetting WiFi configuration...\r\n"));
    } });
  
    server.on("/update", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    bool hasPassword = (reset_password != nullptr && strlen(reset_password) > 0);
    
    size_t maxNeededLength = (sizeof(UPDATE_HTML) - 1) + (sizeof(UPDATE_HTML_END) - 1);
    
    if (hasPassword) {
        maxNeededLength += (sizeof(PASSWORD_INPUT_HTML) - 1);
    }
    
    String html;
    html.reserve(maxNeededLength); 
    
    html += FPSTR(UPDATE_HTML);
    if (hasPassword) {
        html += FPSTR(PASSWORD_INPUT_HTML);
    }
    html += FPSTR(UPDATE_HTML_END);
    
    request->send(200, "text/html", html); });

  server.on("/update", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request)
            {
    if (reset_password != nullptr && strlen(reset_password) > 0) {
        if (request->hasParam("reset_password", true)) {
            const char* submitted_pw = request->getParam("reset_password", true)->value().c_str();
            
            if (strcmp(reset_password, submitted_pw) == 0) {
                  shouldupdate = true;
                  updateTimestamp=millis();
                  request->send(200, "text/html", FPSTR(UPDATE_SUCCESS_HTML));
            } else {
                request->send(403, "text/plain", PSTR("Unauthorized: Invalid reset password.\r\n"));
            }
        } else {
            request->send(400, "text/plain", PSTR("Reset password missing.\r\n"));
        }
    } else {
        shouldupdate = true;
        updateTimestamp=millis();
        request->send(200, "text/html", FPSTR(UPDATE_SUCCESS_HTML));
    } });

  server.on("/json_preferences", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
   
    if (isNotAuthorized(request)) {
            return request->requestAuthentication(); 
        }
    serializeJson(config_doc, json_prefs_string);   
    request->send(200, "application/json", json_prefs_string); 
  });

server.on("/json_preferences", AsyncWebRequestMethod::HTTP_POST, 
    [](AsyncWebServerRequest *request) {
        if (isNotAuthorized(request)) {
            request->send(401, "text/plain", "Unauthorized");
            json_prefs_string = "";
            return;
        }
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", "{\"status\":\"OK\"}");
        response->addHeader("Connection", "close");
        request->send(response);
        parametersUpdated = true;
    },
    NULL, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        if (index == 0) {
            json_prefs_string = "";
            json_prefs_string.reserve(total); 
        }
        
        
        json_prefs_string += String((const char*)data).substring(0, len);
    }
);



// Shelly RPC endpoints called via HTTP GET method
  server.on("/rpc/EM.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    EMGetConfig();
    request->send(200, "application/json", serJsonResponse); });
  server.on("/rpc/EM.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    EMGetStatus();
    request->send(200, "application/json", serJsonResponse); });

  server.on("/rpc/EMData.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    EMDataGetStatus();
    request->send(200, "application/json", serJsonResponse); });

  server.on("/rpc/Shelly.GetComponents", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    shellyGetComponents();
    request->send(200, "application/json", serJsonResponse); });
  server.on("/rpc/Shelly.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    shellyGetConfig();
    request->send(200, "application/json", serJsonResponse); });
  server.on("/rpc/Shelly.GetDeviceInfo", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    shellyGetDeviceInfo();
    request->send(200, "application/json", serJsonResponse); });
  server.on("/rpc/Shelly.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    shellyGetStatus();
    request->send(200, "application/json", serJsonResponse); });

  server.on("/rpc/Sys.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    sysGetConfig();
    request->send(200, "application/json", serJsonResponse); });
  server.on("/rpc/Sys.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    sysGetStatus();
    request->send(200, "application/json", serJsonResponse); });

  server.on("/rpc/WiFi.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
    wifiGetStatus();
    request->send(200, "application/json", serJsonResponse); });

  // Shelly RPC endpoint called via HTTP POST method with JSON-RPC body
  server.on("/rpc", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            {
      String rpcRequestBuffer;
      if (index == 0) {
        // New request, clear buffer
        rpcRequestBuffer = "";
      }
      // Append incoming data chunk to buffer
      rpcRequestBuffer += String((char *)data).substring(0, len);
      if (index + len >= total) {
        // All data received, process RPC request
        parseHttpRPC(rpcRequestBuffer, request);
      } });

  webSocket.onEvent(webSocketEvent);
  wsConsole.onEvent(onWsEvent);
  server.addHandler(&webSocket);
  server.addHandler(&wsConsole);
  server.begin();

  // Set up RPC over UDP for Marstek users
  UdpRPC.begin(shelly_port);

  // Set up MQTT
  if (dataMQTT)
  {
    mqtt_client.setBufferSize(2048);
    if (isValidIPAddress(mqtt_server))
    {
      mqtt_client.setServer(mqtt_server, String(mqtt_port).toInt());
    }
    else
    {
      mqtt_client.setServer(mqtt_server, String(mqtt_port).toInt());
    }
    mqtt_client.setCallback(mqtt_callback);
  }

  // Set Up Multicast for SMA Energy Meter
  if (dataSMA)
  {
    Udp.begin(multicastPort);
#ifdef ESP8266
    Udp.beginMulticast(WiFi.localIP(), multicastIP, multicastPort);
#else
    Udp.beginMulticast(multicastIP, multicastPort);
#endif
  }

  // Set Up UDP for SHRDZM smart meter interface
  if (dataSHRDZM)
  {
    Udp.begin(multicastPort);
  }

  // Set Up Modbus TCP for SUNSPEC register query
  if (dataSUNSPEC)
  {
    modbus1.client();
    DEBUG_SERIAL.println(F("Trying to connect SUNSPEC powermeter data"));
    modbus1.connect(modbus_ip, sunspec_port_int);
  }

  // Set Up HTTP query
  if (dataHTTP)
  {
    http.useHTTP10(true);
  }

#ifdef ONA_ACTIVE
  ona_Setup();
#endif

  // Set up mDNS responder
  setupMdns();

  startMillis = millis();
  onamillis=startMillis;  
}

/* main worker loop, in case of WiFi connection*/
void worker_loop(int currentMillis)
{
#ifndef ESP32
  MDNS.update();
#endif
  parseUdpRPC();
  if (shouldupdate && (currentMillis - updateTimestamp >= 2000))
  {
    all_esp_reset(Energy2Shelly_ResetReason::RECONFIGURE);
    shouldupdate=false;
  }
  if (shouldResetConfig)
  {
#ifdef ESP32
    WiFi.disconnect(true, true);
#else
    // WiFiManager may leave persistent=false; force it so disconnect(true)
    // actually erases the SDK-saved credentials in flash.
    WiFi.persistent(true);
    WiFi.disconnect(true);
    ESP.eraseConfig();
#endif
    delay(1000);
    ESP.restart();
  }
  if (dataMQTT)
  {
    if (!mqtt_client.connected())
    {
      mqtt_reconnect();
    }
    mqtt_client.loop();
  }
  if (dataSMA)
  {
    parseSMA();
  }
  if (dataSHRDZM)
  {
    parseSHRDZM();
  }
  if (dataSUNSPEC)
  {
    modbus1.task(); // allow modbus TCP client to process incoming data and maintain connection, call callbacks
    if (currentMillis - startMillis >= query_period_modbus_u32)
    {
      parseSUNSPEC();
      startMillis = currentMillis;
    }
  }
  if (dataHTTP)
  {
    if (currentMillis - startMillis >= query_period_http_u32)
    {
      queryHTTP();
      startMillis = currentMillis;
    }
  }
  if (dataTIBBERPULSE)
  {
    if (currentMillis - startMillis >= query_period_tibber_u32)
    {
      parseTibberPulse();
      startMillis = currentMillis;
    }
  }
  if (dataRCT)
  {
    if (currentMillis - startMillis >= query_period_rct_u32)
    {
      parseRCT();
      startMillis = currentMillis;
    }
  }
#ifdef ONA_ACTIVE
  if (enableOna) {
    
    if (currentMillis - onamillis >= config_doc["ona"]["publish_period"].as<unsigned long>()*1000ul)
    {
      ona_sendMeasurement();
      onamillis = currentMillis;
    }
  }
#endif
  DEBUG_SERIAL.handleQueue();
}

void loop()
{
  currentMillis = millis();
  static unsigned long WiFilastConnectionCheck = 0; // remember the last time WiFi connection was true

  if (WiFi.status() == WL_CONNECTED)
  {
    WiFilastConnectionCheck = currentMillis; // Reset the last connection check time if connected
    worker_loop(currentMillis);
  }
  else
  {
    // not connected, attempt to reconnect
    static unsigned long reconnectStart = 0;
    if (currentMillis - reconnectStart > 10000) // Attempt to reconnect every 10 seconds, to fast reconnection can cause issues with some routers and APs
    {
      reconnectStart = currentMillis;
      wifi_reconnect_attempts++;
      WiFi.reconnect(); // forces a disconnect and reconnect, hence the 10s delay to avoid rapid reconnection attempts
      DEBUG_SERIAL.println(F("WiFi disconnected, attempting to reconnect..."));
    }
  }

  // check Watchdogs and print status info
  static unsigned long WDTimer = 0;
  if (currentMillis - WDTimer >= 30000)
  {
    stackWD();
    status_print();
    WDTimer = currentMillis;
  }

  if (enableConsoleOutputSerial && (extendedMillis()>(config_doc["console"]["serial_console_time"].as<uint64_t>()*60000ull))) {
      enableConsoleOutputSerial=false;
      DEBUG_SERIAL.println(F("Disabled serial plottin"));
  }

  // check if WiFi connection has been lost for more than the allowed interval
  // saveguard against nasty router/AP behavior
  if (millis() - WiFilastConnectionCheck > WiFicheckInterval)
  {
    DEBUG_SERIAL.println(F("Lost WiFi connection or SSID changed!"));
    all_esp_reset(Energy2Shelly_ResetReason::WIFI_DISCONNECT);
  }
  if (parametersUpdated) {
    DEBUG_SERIAL.println(F("New keys"));
    update_config_doc.clear(); 
    DeserializationError error = deserializeJson(update_config_doc, json_prefs_string);
    DEBUG_SERIAL.handleQueue();
    DEBUG_SERIAL.println(json_prefs_string);
    json_prefs_string = String();
    DEBUG_SERIAL.handleQueue();
    if (!error) {
      mergeJson(config_doc.as<JsonObject>(), update_config_doc.as<JsonObjectConst>());  
      interprete_parameters();
      save_parameters();
    }
    update_config_doc.clear();        
    parametersUpdated = false;
  }

  handleblinkled();
  DEBUG_SERIAL.handleQueue();
#ifdef ESP32
  esp_task_wdt_reset();
#endif
}