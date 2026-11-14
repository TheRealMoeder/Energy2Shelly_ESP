#include <Arduino.h>
#include <memory>

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
#include "web/html.h"

#if defined(ESP32)
#include <Update.h>
#else
#include <Updater.h>
#endif

// Utils function for monitoring and reset
#include "utils/utils.h"

#include "dst/ortsnetzauslastung.h"

#define WIFI_CHECK_INTERVAL 60000 // Check every 60 seconds

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

// Create a gzip-compressed HTML response
static AsyncWebServerResponse *beginGzipHtmlResponse(AsyncWebServerRequest *request, const char *data, size_t compressedLength) {
  AsyncWebServerResponse *response = request->beginResponse("text/html", compressedLength, [data, compressedLength](uint8_t *buffer, size_t maxLen, size_t index) -> size_t
  {
      if (index >= compressedLength) {
          return 0;
      }

      const size_t remaining = compressedLength - index;
      const size_t toCopy = (maxLen < remaining) ? maxLen : remaining;
      memcpy_P(buffer, data + index, toCopy);
      return toCopy;
  });

  response->addHeader("Content-Encoding", "gzip");
  return response;
}

template <size_t N, typename Func>
void sendJsonRpcResult(AsyncWebServerRequest *request, Func generator) {
  if (N <= 512) {
    char buf[N];
    buf[0] = '\0';
    generator(buf, N);
    request->send(200, "application/json", buf);
    return;
  }

  auto buffer = std::make_shared<std::array<char, N>>();
  auto raw = buffer->data();
  raw[0] = '\0';
  generator(raw, N);

  size_t len = strnlen(raw, N);
  AsyncWebServerResponse *response = request->beginResponse(
      "application/json",
      len,
      [buffer, len](uint8_t *out, size_t maxLen, size_t index) -> size_t {
        if (index >= len) return 0;

        size_t n = (maxLen < (len - index)) ? maxLen : (len - index);
        memcpy(out, buffer->data() + index, n);
        return n;
      });

  request->send(response);
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
 
  if ((WiFi.getMode() == WIFI_STA)) { // only ask for NTP if we are save in STA mode
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
  }

  if (led_gpio > 0)
  {
    led = led_gpio;
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
  if ((WiFi.getMode() == WIFI_AP_STA)||(WiFi.getMode()==WIFI_AP))  {
#if defined(ESP32) && defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR < 3
    WiFi.scanNetworks(false, false, false, 1000); // just on blocking scan 1s for each channel
#endif

    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    server.on("/hotspot-detect.html", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
      AsyncWebServerResponse *response = beginGzipHtmlResponse(request, home_html, home_html_len);
      response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
      response->addHeader("Pragma", "no-cache");
      response->addHeader("Expires", "-1");
      request->send(response);
    });

    // Android / Google Hotspot
    server.on("/generate_204", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
      request->redirect(String("http://") + WiFi.softAPIP().toString() + "/");
    });

    // Windows / Microsoft 
    server.on("/connecttest.txt", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
      request->redirect(String("http://") + WiFi.softAPIP().toString() + "/");
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
      DEBUG_SERIAL.printf("Not Found -> Redirect to IP: %s\n", request->url().c_str());
      request->redirect(String("http://") + WiFi.softAPIP().toString() + "/");
    });
  }

  // Set up web server endpoints using the thread-safe local buffer paradigm
  server.on("/", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(beginGzipHtmlResponse(request, home_html, home_html_len));
  });

  server.on("/plot", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(beginGzipHtmlResponse(request, webplotter_html, webplotter_html_len));
  });

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

  server.on("/console", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(beginGzipHtmlResponse(request, webconsole_html, webconsole_html_len));
  });

  server.on("/config", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {  
    size_t totalLen = strlen_P(CONFIG_PAGE_HTML); 
    AsyncWebServerResponse *response = request->beginResponse("text/html", totalLen, [totalLen](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      // copy fragment bytes from PROGMEM to buffer, use less resources by copying only the requested fragment
      size_t remaining = totalLen - index;
        if (remaining == 0) return 0; 

        size_t toCopy = (maxLen > remaining) ? remaining : maxLen;
        memcpy_P(buffer, CONFIG_PAGE_HTML + index, toCopy); 
        
        return toCopy; 
    });
    request->send(response); 
  });

  server.on("/shelly", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    const size_t BUFFER_SIZE = 2048;
    String responseBuffer;
    responseBuffer.reserve(BUFFER_SIZE);
    char* ptr = const_cast<char*>(responseBuffer.c_str());
    shellyGetDeviceInfo(ptr, BUFFER_SIZE);
    responseBuffer = String(ptr);
    
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", responseBuffer);
    response->addHeader("Connection", "close");
    request->send(response);
  });

  server.on("/reset", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
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
    request->send(200, "text/html", html); 
  });

  server.on("/reset", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {
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
    } 
  });
  
  server.on("/update", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request){
       if (isNotAuthorized(request)) { return request->requestAuthentication(); }
       request->send(beginGzipHtmlResponse(request, update_index_html, update_index_html_len));
  });

  server.on("/update", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request){
    if (isNotAuthorized(request)) { return request->requestAuthentication(); }
      bool success = !Update.hasError();
    if (success) {
      AsyncWebServerResponse *response = beginGzipHtmlResponse(request, update_success_html, update_success_html_len);
      response->addHeader("Connection", "close");
      request->send(response);
      updateTimestamp=millis();
      shouldReboot=true;
    } else {
      AsyncWebServerResponse *response = beginGzipHtmlResponse(request, update_error_html, update_error_html_len);
      response->addHeader("Connection", "close");
      request->send(response);
    }
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if (isNotAuthorized(request)) {
      return request->requestAuthentication();
    }
    if (!index) {
      DEBUG_SERIAL.print(F("Update Started: "));
      DEBUG_SERIAL.println(filename.c_str());
#ifdef ESP32
      size_t updateSize = UPDATE_SIZE_UNKNOWN;
#elif defined(ESP8266)
      size_t updateSize = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
      Update.runAsync(true);
#endif
      if (!Update.begin(updateSize, U_FLASH)) {
        DEBUG_SERIAL.println(F("Error flash"));
      }
    }
#ifdef ESP8266
    ESP.wdtFeed();
#endif
    if (Update.write(data, len) != len) {
      DEBUG_SERIAL.println(F("Error flash"));
    }
    if (final) {
      if(Update.end(true)){
        DEBUG_SERIAL.print(F("Update done: "));
        DEBUG_SERIAL.print(index + len);
        DEBUG_SERIAL.println(F(" bytes written"));
      }
    }
  });

  server.on("/json_config", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    if (isNotAuthorized(request)) {
      return request->requestAuthentication();
     }
    if ((WiFi.getMode() == WIFI_AP_STA)||(WiFi.getMode()==WIFI_AP)) {
      String temp;
      serializeJson(config_doc, temp);
      request->send(200, "application/json", temp);
    } else {
      if (LittleFS.exists(configPath)) {
        request->send(LittleFS, configPath, "application/json");
      } else {
        request->send(404, "application/json", "{\"error\":\"File not found\"}");
      }
    }
  });

  server.on("/reboot", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {
    if (isNotAuthorized(request)) {
      return request->requestAuthentication();
    }
    request->send(200, "text/plain", FPSTR(UPDATE_SUCCESS_HTML));
    updateTimestamp=millis();
    shouldReboot=true;
  });
  server.on("/json_config", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {
    if (isNotAuthorized(request)) {
      request->send(401, "text/plain", "Unauthorized");
      return;
    }
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", "{\"status\":\"OK\"}");
    response->addHeader("Connection", "close");
    request->send(response);

    parametersUpdated = true;
    },
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (isNotAuthorized(request)) return;

      File file = LittleFS.open(configPath, index == 0 ? "w" : "a");
      if (file) {
        file.write(data, len);
        file.close();
      }
    }
  );
  
  // Shelly RPC routes mapping directly to local buffer string parsers
  server.on("/rpc/EM.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<192>(request, EMGetConfig);
  });
  server.on("/rpc/EM.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<768>(request, EMGetStatus);
  });
  server.on("/rpc/EMData.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<384>(request, EMDataGetStatus);
  });
  server.on("/rpc/Shelly.GetComponents", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<1280>(request, shellyComponents);
  });
  server.on("/rpc/Shelly.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<1024>(request, shellyGetConfig);
  });
  server.on("/rpc/Shelly.GetDeviceInfo", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<256>(request, shellyGetDeviceInfo);
  });
  server.on("/rpc/Shelly.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<1792>(request, shellyGetStatus);
  });
  server.on("/rpc/Sys.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<512>(request, sysGetConfig);
  });
  server.on("/rpc/Sys.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<512>(request, sysGetStatus);
  });
  server.on("/rpc/WiFi.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonRpcResult<256>(request, wifiGetStatus);
  });

  // Thread-safe JSON-RPC POST payload assembler
  server.on("/rpc", AsyncWebRequestMethod::HTTP_POST, 
    [](AsyncWebServerRequest *request) {
        if (request->_tempObject != nullptr) {
            String* pAnswer = static_cast<String*>(request->_tempObject);
            AsyncWebServerResponse *response = request->beginResponse(200, "application/json", *pAnswer);
            request->send(response);
            delete pAnswer;
            request->_tempObject = nullptr;
        } else {
            request->send(500, "text/plain", "Internal Server Error: Missing payload.");
        }
    }, nullptr, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        if (total > 2048) { request->send(413, "text/plain", "Payload Too Large"); return; }
        String* pBuffer = nullptr;
        if (index == 0) {
            pBuffer = new String();
            pBuffer->reserve(total + 1);
            request->_tempObject = pBuffer;
        } else {
            pBuffer = static_cast<String*>(request->_tempObject);
        }
        if (pBuffer != nullptr) {
          pBuffer->concat((char*)data, len);
        }
        if (index + len >= total && pBuffer != nullptr) {
          String responseBody;
          parseHttpRPC(*pBuffer, responseBody);
          *pBuffer = responseBody;
        }
    }
  );
  webSocket.onEvent(webSocketEvent);
  wsConsole.onEvent(onWsEvent);
  server.addHandler(&webSocket);
  server.addHandler(&wsConsole);
  server.begin();

  // Set up RPC over UDP for Marstek users
  UdpRPC.begin(shelly_port);

  // Set up MQTT
  switch (current_input_type) { 
    case input_type_enum::MQTT:
    {
      mqtt_client = new PubSubClient(wifi_client);
      IPAddress ip;
      mqtt_client->setBufferSize(2048);
      if (ip.fromString(mqtt_server))
      {
        mqtt_client->setServer(ip, mqtt_port);
      }
      else
      {
        mqtt_client->setServer(mqtt_server, mqtt_port);
      }
      mqtt_client->setCallback(mqtt_callback);
      break;
    }
    case input_type_enum::SMA:
      Udp = new WiFiUDP();
#ifdef ESP8266
      Udp->beginMulticast(WiFi.localIP(), multicastIP, multicastPort);
#else
      Udp->beginMulticast(multicastIP, multicastPort);
#endif
     break;
    
     case input_type_enum::SHRDZM:
      // Set Up UDP for SHRDZM smart meter interface
      Udp = new WiFiUDP();
      Udp->begin(multicastPort);
      break;

    case input_type_enum::SUNSPEC:
      modbus1 = new ModbusIP();
      modbus1->client();
      DEBUG_SERIAL.println(F("Trying to connect SUNSPEC powermeter data"));
      modbus1->connect(modbus_ip, sunspec_port_int);
      break;

      case input_type_enum::SMLSERIAL:
      setupSMLSerial();
      break;

      case input_type_enum::HTTP:
      http = new HTTPClient();
      http->useHTTP10(true);
      break;

      case input_type_enum::NONE:
      default:
      DEBUG_SERIAL.println(F("No source"));
  }
  
#if defined(ONA_ACTIVE) && defined(ESP32)
  ONA_Setup();
#endif
  // Set up mDNS responder
  setupMdns();

  startMillis = millis();
#ifdef ONA_ACTIVE
  ONA_Millis=startMillis;  
#endif
}

/* main worker loop, in case of WiFi connection*/
void worker_loop(int currentMillis)
{
#ifndef ESP32
  MDNS.update();
#endif
  parseUdpRPC();
  notifyStatusWebSocket();
  if (shouldupdate && (currentMillis - updateTimestamp >= 2000))
  {
    all_esp_reset(Energy2Shelly_ResetReason::RECONFIGURE);
    shouldupdate=false;
  }
  if (shouldResetConfig)
  {
     wifi_reset();
  }

  switch (current_input_type) {
    case input_type_enum::MQTT:
      if (!mqtt_client->connected())
      {
        mqtt_reconnect();
      }
      mqtt_client->loop();
      break;
    case input_type_enum::SMA:
      parseSMA();
      break;
    case input_type_enum::SHRDZM:
      parseSHRDZM();
      break;
    case input_type_enum::SUNSPEC:
      modbus1->task(); // allow modbus TCP client to process incoming data and maintain connection, call callbacks
      if (currentMillis - startMillis >= query_period_modbus_u32)
      {
        parseSUNSPEC();
        startMillis = currentMillis;
      }
      break;
    case input_type_enum::HTTP:
      if (currentMillis - startMillis >= query_period_http_u32)
      {
        queryHTTP();
        startMillis = currentMillis;
      }
      break;
    case input_type_enum::TIBBERPULSE:
      if (currentMillis - startMillis >= query_period_tibber_u32)
      {
        parseTibberPulse();
        startMillis = currentMillis;
      }
      break;
    case input_type_enum::SMLSERIAL:
      parseSMLSerial(); // non-blocking, consumes the bytes received since the last call
      break;
    case input_type_enum::RCT:
      if (currentMillis - startMillis >= query_period_rct_u32)
      {
        parseRCT();
        startMillis = currentMillis;
      }
      break;
    case input_type_enum::NONE:
      default:
      delay(10);
  }
#ifdef ONA_ACTIVE
  if (enableONA) {
    ONA_loop_state_machine();
    if (currentMillis - ONA_Millis >= config_doc["ona"]["publish_period"].as<unsigned long>()*1000ul)
    {
      ONA_sendMeasurement();
      ONA_Millis = currentMillis;
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
    if ((currentMillis - reconnectStart > 10000)&&(WiFi.getMode() == WIFI_STA)) // Attempt to reconnect every 10 seconds, to fast reconnection can cause issues with some routers and APs
    {
      reconnectStart = currentMillis;
      wifi_reconnect_attempts++;
      WiFi.reconnect(); // forces a disconnect and reconnect, hence the 10s delay to avoid rapid reconnection attempts
      DEBUG_SERIAL.println(F("WiFi disconnected, attempting to reconnect..."));
    }
  }

  if (shouldReboot && (currentMillis - updateTimestamp >= 2000))
  {
    shouldReboot = false;
    all_esp_reset(Energy2Shelly_ResetReason::REBOOT_REQUEST);
  }

  // Clean up disconnected/zombie WebSocket clients every 1 second
  static unsigned long wsCleanupTimer = 0;
  if (currentMillis - wsCleanupTimer >= 1000)
  {
    webSocket.cleanupClients();
    wsCleanupTimer = currentMillis;
  }

  // check Watchdogs and print status info
  static unsigned long WDTimer = 0;
  if (currentMillis - WDTimer >= 30000)
  {
    stackWD();
    status_print();
    WDTimer = currentMillis;
  }

  // Check if serial console debug messages should be disabled after timeout
  if (enableConsoleOutputSerial && (extendedMillis() > static_cast<uint64_t>(config_doc["console"]["serial_console_time"].as<int>() * 60000ll))) {
      DEBUG_SERIAL.println(F("Disabled serial debug messages"));
      enableConsoleOutputSerial = false;
  }

  if ((WiFi.getMode() == WIFI_AP_STA) || (WiFi.getMode() == WIFI_AP)) {
      dnsServer.processNextRequest();
      wifi_scan();
      if ((extendedMillis() > 15ull * 60ull * 1000ull) && (WiFi.softAPgetStationNum() == 0)) { // Reset device after 15 minutes in AP mode, to avoid being stuck in AP mode forever
          DEBUG_SERIAL.println(F("AP mode timeout, resetting device"));
          all_esp_reset(Energy2Shelly_ResetReason::AP_TIMEOUT);
      }
  } else {
    // check if WiFi connection has been lost for more than the allowed interval
    // saveguard against nasty router/AP behavior
    if (millis() - WiFilastConnectionCheck > WIFI_CHECK_INTERVAL)
    {
      DEBUG_SERIAL.println(F("Lost WiFi connection or SSID changed!"));
      all_esp_reset(Energy2Shelly_ResetReason::WIFI_DISCONNECT);
    }
  }

  if (parametersUpdated) {
    DEBUG_SERIAL.println(F("New keys"));
    input_type_enum previous_input_type = current_input_type;

    load_parameters();
    serializeJsonPretty(config_doc, DEBUG_SERIAL);
    DEBUG_SERIAL.println();
    interprete_parameters();
    parametersUpdated = false;

    if (previous_input_type != current_input_type) {
      DEBUG_SERIAL.println(F("Input type changed, rebooting..."));
      current_input_type = input_type_enum::NONE; // avoid nullptr access before reboot
      updateTimestamp = millis();
      shouldReboot = true;
    }
  }

  // set reset counter to 0, after one minute
  checkreset_counter();

  handleblinkled();
  DEBUG_SERIAL.handleQueue();
#ifdef ESP32
  esp_task_wdt_reset();
#endif
}
