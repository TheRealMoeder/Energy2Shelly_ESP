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

#define WiFicheckInterval 60000 // Check every 60 seconds

void setup(void)
{
  DEBUG_SERIAL.begin(115200);
  clear_rtc_power_on();
  WifiManagerSetup();

  // Initialize watchdog timer (30s timeout)
#ifdef ESP32
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);
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

  server.on("/favicon.ico", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
              request->send(204); // 204 "No Content"
            });

  server.on("/console", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
            {
      AsyncWebServerResponse *response = request->beginResponse("text/html", strlen_P(htmlPage_console), [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        // copy fragment bytes from PROGMEM to buffer, use less resources by copying only the requested fragment
        memcpy_P(buffer, htmlPage_console + index, maxLen); return maxLen;
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
    String html = "<!DOCTYPE html><html><head><title>Reset Confirmation</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial,sans-serif;text-align:center;padding:20px;}";
    html += ".btn{padding:10px 20px;margin:10px;cursor:pointer;text-decoration:none;display:inline-block;border-radius:5px;font-size:16px;}";
    html += ".btn-yes{background-color:#d9534f;color:white;border:none;}";
    html += ".btn-no{background-color:#5bc0de;color:white;border:none;}</style></head><body>";
    html += "<h2>Reset Configuration?</h2>";
    html += "<p>Are you sure you want to reset the WiFi configuration? This will clear current WiFi settings and restart the device in AP mode.</p>";
    html += "<form method='POST' style='display:inline;' accept-charset='UTF-8'>";
    if (reset_password != nullptr && strlen(reset_password) > 0) {
      html += "<input type='password' name='reset_password' placeholder='Enter reset password' required><br/>";
    }
    html += "<button type='submit' class='btn btn-yes'>Yes, Reset</button>";
    html += "</form>";
    html += "<a href='/' class='btn btn-no'>Cancel</a>";
    html += "</body></html>";
    request->send(200, "text/html", html); });

  server.on("/reset", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request)
            {
    if (reset_password != nullptr && strlen(reset_password) > 0) {
       if (request->hasParam("reset_password", true)) {
        if (String(reset_password) == request->getParam("reset_password", true)->value()) {
          shouldResetConfig = true;
          request->send(200, "text/plain", "Resetting WiFi configuration, please log back into the hotspot to reconfigure...\r\n");
        } else {
          request->send(403, "text/plain", "Unauthorized: Invalid reset password.\r\n");
        }
      } else {
        request->send(400, "text/plain", "Reset password missing.\r\n");
      }
    } else {
      shouldResetConfig = true;
      request->send(200, "text/plain", "Resetting WiFi configuration, please log back into the hotspot to reconfigure...\r\n");
    } });

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
  UdpRPC.begin(String(shelly_port).toInt());

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
    period = atol(query_period);
    modbus1.client();
    modbus_ip.fromString(mqtt_server); // Todo, replace by DNS resolver for hostnames, currently only IP addresses are supported
    sunspec_port_int = atol(mqtt_port);
    modbusdev_int = atol(modbus_dev);
    DEBUG_SERIAL.println(F("Trying to connect SUNSPEC powermeter data"));
    modbus1.connect(modbus_ip, sunspec_port_int);   
  }

  // Set Up HTTP query
  if (dataHTTP)
  {
    period = atol(query_period);
    http.useHTTP10(true);
  }

  // Set up mDNS responder
  setupMdns();

  startMillis = millis();
}

/* main worker loop, in case of WiFi connection*/
void worker_loop(int currentMillis)
{
#ifndef ESP32
  MDNS.update();
#endif
  parseUdpRPC();
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
    if (currentMillis - startMillis >= period)
    {
      parseSUNSPEC();
      startMillis = currentMillis;
    }
  }
  if (dataHTTP)
  {
    if (currentMillis - startMillis >= period)
    {
      queryHTTP();
      startMillis = currentMillis;
    }
  }
  if (dataTIBBERPULSE)
  {
    if (currentMillis - startMillis >= period)
    {
      parseTibberPulse();
      startMillis = currentMillis;
    }
  }
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

  // check if WiFi connection has been lost for more than the allowed interval
  // saveguard against nasty router/AP behavior
  if (millis() - WiFilastConnectionCheck > WiFicheckInterval)
  {
    DEBUG_SERIAL.println(F("Lost WiFi connection or SSID changed!"));
    all_esp_reset(Energ2Shelly_ResetReason::WIFI_DISCONNECT);
  }

  handleblinkled();
  DEBUG_SERIAL.handleQueue();
#ifdef ESP32
  esp_task_wdt_reset();
#endif
}