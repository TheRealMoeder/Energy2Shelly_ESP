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

unsigned long updateTimestamp = 0;

static String htmlEscape(const char *s)
{
  if (!s) return String();
  String out;
  out.reserve(strlen(s) * 2 + 8);
  for (size_t i = 0; s[i] != '\0'; ++i) {
    char c = s[i];
    switch (c) {
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += c; break;
    }
  }
  return out;
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

  // Reloads of wifi manager page, redirect to main page
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

  // Config page
  server.on("/config", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request)
  {
    bool hasPassword = (reset_password != nullptr && strlen(reset_password) > 0);

    String html;
    html.reserve(4096);

    html += FPSTR(CONFIG_HTML);

/* Backup for config Parameters from WifiManager
static WiFiManagerParameter custom_section1("<h3>General settings</h3><script>function t(s) { var x = document.getElementById(s); x.type === \"password\" ? x.type = \"text\" : x.type = \"password\"; }</script>");
  static WiFiManagerParameter param_reset_password("reset_password", "Reset Password <span title=\"For resetting the WiFi configuration and putting the device in AP / config mode\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", reset_password, 32, "type='password'");
  static WiFiManagerParameter param_reset_password_show_password(buf_rst_pwd_show_pwd);
  static WiFiManagerParameter custom_input_type("type", "<hr><b>Data source</b><br><code>MQTT</code> for MQTT<br><code>HTTP</code> for generic HTTP<br><code>SMA</code> for SMA EM/HM multicast<br><code>SHRDZM</code> for SHRDZM UDP data<br><code>SUNSPEC</code> for Modbus TCP SUNSPEC data<br><code>TIBBERPULSE</code> for TibberPulse SML data<br><code>RCT</code> for RCT Power grid data", input_type, 40);
  static WiFiManagerParameter custom_mqtt_server("server", "<b>Server</b><br>MQTT Server IP, query url for generic HTTP or Modbus TCP server IP for SUNSPEC", mqtt_server, 160);
  static WiFiManagerParameter custom_mqtt_port("port", "<b>Port</b><br> for MQTT or Modbus TCP (SUNSPEC)", mqtt_port, 6);
  static WiFiManagerParameter param_ntp_server("ntp_server", "NTP server <span title=\"for time synchronization\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", ntp_server, 40);
  static WiFiManagerParameter param_timezone("timezone", "Timezone <span title=\"e.g. UTC0, UTC+1, UTC-3, UTC+1CET-1CEST,M3.5.0/02:00:00,M10.5.0/03:00:00\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", timezone, 64);
  static WiFiManagerParameter param_phase_number("phase_number", "Number of phases <span title=\"Number of phases (e.g. 1 or 3)\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", phase_number, 1);
  static WiFiManagerParameter custom_power_offset("power_offset", "Power offset <span title=\"optional offset value in Watts added to the emulated output\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", power_offset, 10);
  static WiFiManagerParameter custom_query_period("query_period", "<b>Query period</b><br>for generic HTTP and SUNSPEC, in milliseconds", query_period, 10);
  static WiFiManagerParameter custom_led_gpio("led_gpio", "<b>GPIO</b><br>of internal LED", led_gpio, 3);
  static WiFiManagerParameter custom_led_gpio_i("led_gpio_i", "<b>GPIO is inverted</b><br><code>true</code> or <code>false</code>", led_gpio_i, 6);
  static WiFiManagerParameter custom_shelly_mac("mac", "<b>Shelly ID</b><br>12 char hexadecimal, defaults to MAC address of ESP", shelly_mac, 13);
  static WiFiManagerParameter custom_shelly_port("shelly_port", "<b>Shelly UDP port</b><br><code>1010</code> or <code>2220</code> depending on Marstek Venus model and firmware version", shelly_port, 6);
  static WiFiManagerParameter custom_sma_id("sma_id", "<b>SMA serial number</b><br>optional serial number if you have more than one SMA EM/HM in your network", sma_id, 16);
  static WiFiManagerParameter custom_section2("<hr><h3>MQTT options</h3>");
  static WiFiManagerParameter custom_mqtt_topic("topic", "<b>MQTT Topic</b>", mqtt_topic, 90);
  static WiFiManagerParameter custom_mqtt_user("user", "<b>MQTT user</b> (optional)", mqtt_user, 40);
  static WiFiManagerParameter custom_mqtt_passwd("passwd", "<b>MQTT password</b> (optional)", mqtt_passwd, 40, "type='password'");
  static WiFiManagerParameter param_mqtt_passwd_show_password(buf_mqtt_pwd_show_pwd);
  static WiFiManagerParameter custom_section3("<hr><h3>Modbus TCP options</h3>");
  static WiFiManagerParameter custom_modbus_dev("modbus_dev", "<b>Modbus device ID</b><br><code>71</code> for Kostal SEM", modbus_dev, 60);
  static WiFiManagerParameter custom_section4("<hr><h3>JSON paths for MQTT and generic HTTP</h3>");
  static WiFiManagerParameter custom_power_path("power_path", "<b>Total power JSON path</b><br>e.g. <code>ENERGY.Power</code> or <code>TRIPHASE</code> for tri-phase data", power_path, 60);
  static WiFiManagerParameter custom_pwr_export_path("pwr_export_path", "<b>Export power JSON path</b><br>Optional, for net calc (e.g. \"i-e\"", pwr_export_path, 60);
  static WiFiManagerParameter custom_power_l1_path("power_l1_path", "<b>Phase 1 power JSON path</b><br>optional", power_l1_path, 60);
  static WiFiManagerParameter custom_power_l2_path("power_l2_path", "<b>Phase 2 power JSON path</b><br>optional", power_l2_path, 60);
  static WiFiManagerParameter custom_power_l3_path("power_l3_path", "<b>Phase 3 power JSON path</b><br>optional", power_l3_path, 60);
  static WiFiManagerParameter custom_energy_in_path("energy_in_path", "<b>Energy from grid JSON path</b><br>e.g. <code>ENERGY.Grid</code>", energy_in_path, 60);
  static WiFiManagerParameter custom_energy_out_path("energy_out_path", "<b>Energy to grid JSON path</b><br>e.g. <code>ENERGY.FeedIn</code>", energy_out_path, 60);
  // TibberPulse section
  static WiFiManagerParameter param_section_tibberpulse("<hr><h3>TibberPulse options</h3>");
  static WiFiManagerParameter param_tibber_host_port("tibber_host_port", "Hostname/IP[:port] <span title=\"e.g.: 192.168.0.1:8080\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", tibber_host, 40);
  static WiFiManagerParameter param_tibber_node_id("tibber_node_id", "Node ID <span title=\"defaults to: 1, check the bridge web interface for the right nodeId\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", tibber_nodeid, 1);
  static WiFiManagerParameter param_tibber_user("tibber_user", "User <span title=\"defaults to: admin\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", tibber_user, 10);
  static WiFiManagerParameter param_tibber_password("tibber_password", "Password <span title=\"as printed on bridge device: xxxx-xxxx\" style=\"cursor: help;\" aria-label=\"Help\" tabindex=\"0\">(?)</span>", tibber_password, 10, "type='password'");
  static WiFiManagerParameter param_tibber_password_show_password(buf_tibber_pwd_show_pwd);

  // RCT Power section
  static WiFiManagerParameter custom_section_rct("<hr><h3>RCT Power options</h3>");
  static WiFiManagerParameter custom_rct_host("rct_host", "<b>RCT host</b><br>IP address or hostname of the RCT Power device (with integrated grid sensor)", rct_host, 41);
  static WiFiManagerParameter custom_rct_port("rct_port", "<b>RCT port</b><br><code>8899</code> default", rct_port, 6);
*/

    html += "<label>MQTT Server</label>\n";
    html += "<input type='text' name='mqtt_server' value='";
    html += htmlEscape(mqtt_server);
    html += "'>\n";

    html += "<label>MQTT Port</label>\n";
    html += "<input type='text' name='mqtt_port' value='";
    html += htmlEscape(mqtt_port);
    html += "'>\n";

    html += "<label>MQTT User</label>\n";
    html += "<input type='text' name='mqtt_user' value='";
    html += htmlEscape(mqtt_user);
    html += "'>\n";

    html += "<label>MQTT Password</label>\n";
    html += "<input id='mqtt_passwd' type='password' name='mqtt_passwd' value=''>\n";
    html += "<label style='font-weight:normal; display:flex; align-items:center; gap:8px; margin-top:8px;'>\n";
    html += "  <input type='checkbox' onclick=\"togglePassword('mqtt_passwd')\"> Show password\n";
    html += "</label>\n";

    html += "<label>NTP Server</label>\n";
    html += "<input type='text' name='ntp_server' value='";
    html += htmlEscape(ntp_server);
    html += "'>\n";

    html += "<label>Timezone</label>\n";
    html += "<input type='text' name='timezone' value='";
    html += htmlEscape(timezone);
    html += "'>\n";

    html += "<label>Query Period (seconds)</label>\n";
    html += "<input type='text' name='query_period' value='";
    html += htmlEscape(query_period);
    html += "'>\n";

    html += "<label>LED GPIO</label>\n";
    html += "<input type='text' name='led_gpio' value='";
    html += htmlEscape(led_gpio);
    html += "'>\n";

    html += "<label>Shelly Port</label>\n";
    html += "<input type='text' name='shelly_port' value='";
    html += htmlEscape(shelly_port);
    html += "'>\n";

    html += "<label>Input Type</label>\n";
    html += "<select name='input_type'>\n";
    const char *types[] = {"MQTT", "SMA", "HTTP", "TIBBERPULSE", "RCT"};
    for (size_t t = 0; t < sizeof(types) / sizeof(types[0]); ++t) {
      html += "<option value='";
      html += types[t];
      html += "'";
      if (strcmp(input_type, types[t]) == 0) html += " selected";
      html += ">";
      html += types[t];
      html += "</option>\n";
    }
    html += "</select>\n";

    if (hasPassword) {
      html += "<label>Reset Password</label>\n";
      html += "<input id='reset_password' type='password' name='reset_password' placeholder='Enter reset password'>\n";
      html += "<label style='font-weight:normal; display:flex; align-items:center; gap:8px; margin-top:8px;'>\n";
      html += "  <input type='checkbox' onclick=\"togglePassword('reset_password')\"> Show password\n";
      html += "</label>\n";
    }

    html += "<script>\n";
    html += "function togglePassword(id) {\n";
    html += "  const el = document.getElementById(id);\n";
    html += "  if (!el) return;\n";
    html += "  el.type = (el.type === 'password') ? 'text' : 'password';\n";
    html += "}\n";
    html += "</script>\n";

    html += FPSTR(CONFIG_HTML_END);

    request->send(200, "text/html", html);
  });

  server.on("/config", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request)
  {
    if (reset_password != nullptr && strlen(reset_password) > 0) {
      if (request->hasParam("reset_password", true)) {
        const char* submitted_pw = request->getParam("reset_password", true)->value().c_str();
        if (strcmp(reset_password, submitted_pw) != 0) {
          request->send(403, "text/plain", PSTR("Unauthorized: Invalid reset password.\r\n"));
          return;
        }
      } else {
        request->send(400, "text/plain", PSTR("Reset password missing.\r\n"));
        return;
      }
    }

    bool changed = false;

    struct Field {
      const char* name;
      char* dest;
      size_t maxlen;
    };

    Field fields[] = {
      {"mqtt_server", mqtt_server, sizeof(mqtt_server)},
      {"mqtt_port", mqtt_port, sizeof(mqtt_port)},
      {"mqtt_user", mqtt_user, sizeof(mqtt_user)},
      {"mqtt_passwd", mqtt_passwd, sizeof(mqtt_passwd)},
      {"ntp_server", ntp_server, sizeof(ntp_server)},
      {"timezone", timezone, sizeof(timezone)},
      {"query_period", query_period, sizeof(query_period)},
      {"led_gpio", led_gpio, sizeof(led_gpio)},
      {"shelly_port", shelly_port, sizeof(shelly_port)},
      {"input_type", input_type, sizeof(input_type)},
      {"reset_password", reset_password, sizeof(reset_password)}
    };
    const size_t fieldCount = sizeof(fields) / sizeof(fields[0]);

    for (size_t i = 0; i < request->params(); ++i) {
      const AsyncWebParameter* p = request->getParam(i);
      if (!p || !p->isPost()) continue;

      const String &pname = p->name();
      const String &pvalue = p->value();

      for (size_t f = 0; f < fieldCount; ++f) {
        if (pname.equals(fields[f].name)) {
          size_t copyLen = (pvalue.length() < (fields[f].maxlen - 1))
            ? pvalue.length()
            : (fields[f].maxlen - 1);

          pvalue.toCharArray(fields[f].dest, copyLen + 1);
          fields[f].dest[copyLen] = '\0';
          changed = true;
          break;
        }
      }
    }

    if (changed) {
      writeConfig();
      delay(500);
      ESP.restart();
    }

    request->send(200, "text/html", FPSTR(CONFIG_SUCCESS_HTML));
  });

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

// main worker loop, in case of WiFi connection
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
  if (dataRCT)
  {
    if (currentMillis - startMillis >= period)
    {
      parseRCT();
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
    all_esp_reset(Energy2Shelly_ResetReason::WIFI_DISCONNECT);
  }

  handleblinkled();
  DEBUG_SERIAL.handleQueue();
#ifdef ESP32
  esp_task_wdt_reset();
#endif
}