#include "WebConfig.h"
#include "Configuration.h"
#include <ESPAsyncWebServer.h>

// ============================================================================
// HTML CONFIGURATION PAGE
// ============================================================================

const char CONFIG_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<title>E2S Configuration</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 20px; }
h2 { text-align: center; color: #333; }
form { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 600px; margin: 0 auto; }
label { display: block; margin-bottom: 5px; font-weight: bold; color: #555; }
input[type=text], input[type=password], select { width: 100%; padding: 8px; margin-bottom: 15px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
.btn { padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; display: block; width: 100%; }
.btn-save { background-color: #4CAF50; color: white; }
.note { font-size: 0.9em; color: #777; text-align: center; margin-top: 20px; }
fieldset { border: 1px solid #ddd; border-radius: 4px; padding: 15px; margin-bottom: 20px; }
legend { font-weight: bold; color: #333; padding: 0 5px; }
</style>
</head>
<body>
<h2>Energy2Shelly Configuration</h2>
<form method='POST' action='/save'>
  <fieldset>
    <legend>General Settings</legend>
    <label for="inputType">Data Source Type</label>
    <select id="inputType" name="inputType">
      <option value="MQTT" %s_MQTT%>MQTT</option>
      <option value="HTTP" %s_HTTP%>Generic HTTP</option>
      <option value="SMA" %s_SMA%>SMA Multicast</option>
      <option value="SHRDZM" %s_SHRDZM%>SHRDZM UDP</option>
      <option value="SUNSPEC" %s_SUNSPEC%>Modbus TCP (SUNSPEC)</option>
    </select>
    <label for="mqttServer">Server / URL</label>
    <input type='text' id='mqttServer' name='mqttServer' value='%v_mqttServer%'>
    <label for="queryPeriod">Query Period (ms)</label>
    <input type='text' id='queryPeriod' name='queryPeriod' value='%v_qPeriod%'>
    <label for="ledGpio">LED GPIO</label>
    <input type='text' id='ledGpio' name='ledGpio' value='%v_ledGpio%'>
    <label for="ledInverted">Invert LED GPIO</label>
    <select id="ledInverted" name="ledInverted">
      <option value="false" %s_ledInv_f%>No</option>
      <option value="true" %s_ledInv_t%>Yes</option>
    </select>
    <label for="shellyMac">Shelly ID (MAC)</label>
    <input type='text' id='shellyMac' name='shellyMac' value='%v_shellyMac%'>
    <label for="shellyPort">Shelly UDP Port</label>
    <input type='text' id='shellyPort' name='shellyPort' value='%v_shellyPort%'>
    <label for="forcePwrDecimals">Force Power Decimals</label>
    <select id="forcePwrDecimals" name="forcePwrDecimals">
      <option value="true" %s_fPwrDec_t%>Yes</option>
      <option value="false" %s_fPwrDec_f%>No</option>
    </select>
    <label for="smaId">SMA Serial Number</label>
    <input type='text' id='smaId' name='smaId' value='%v_smaId%'>
  </fieldset>

  <fieldset>
    <legend>MQTT Options</legend>
    <label for="mqttPort">MQTT Port</label>
    <input type='text' id='mqttPort' name='mqttPort' value='%v_mqttPort%'>
    <label for="mqttTopic">MQTT Topic</label>
    <input type='text' id='mqttTopic' name='mqttTopic' value='%v_mqttTopic%'>
    <label for="mqttUser">MQTT User</label>
    <input type='text' id='mqttUser' name='mqttUser' value='%v_mqttUser%'>
    <label for="mqttPasswd">MQTT Password</label>
    <input type='password' id='mqttPasswd' name='mqttPasswd' value='%v_mqttPasswd%'>
  </fieldset>

  <fieldset>
    <legend>Modbus TCP Options</legend>
    <label for="modbusDevice">Modbus Device ID</label>
    <input type='text' id='modbusDevice' name='modbusDevice' value='%v_mbDevice%'>
  </fieldset>

  <fieldset>
    <legend>JSON Paths</legend>
    <label for="powerPath">Total Power Path</label>
    <input type='text' id='powerPath' name='powerPath' value='%v_powerPath%'>
    <label for="pwrExportPath">Export Power Path</label>
    <input type='text' id='pwrExportPath' name='pwrExportPath' value='%v_pwrExpPath%'>
    <label for="powerL1Path">Phase 1 Power Path</label>
    <input type='text' id='powerL1Path' name='powerL1Path' value='%v_powerL1Path%'>
    <label for="powerL2Path">Phase 2 Power Path</label>
    <input type='text' id='powerL2Path' name='powerL2Path' value='%v_powerL2Path%'>
    <label for="powerL3Path">Phase 3 Power Path</label>
    <input type='text' id='powerL3Path' name='powerL3Path' value='%v_powerL3Path%'>
    <label for="energyInPath">Energy In Path</label>
    <input type='text' id='energyInPath' name='energyInPath' value='%v_energyInPath%'>
    <label for="energyOutPath">Energy Out Path</label>
    <input type='text' id='energyOutPath' name='energyOutPath' value='%v_energyOutPath%'>
  </fieldset>

  <button type='submit' class='btn btn-save'>Save Configuration</button>
</form>
<p class="note">Device will restart after saving.</p>
</body>
</html>
)=====";

// ============================================================================
// TEMPLATE PROCESSOR - Substitutes variables in HTML template
// ============================================================================

String processor(const String &var) {
  // General
  if (var == "v_mqttServer")
    return mqtt_server;
  if (var == "v_qPeriod")
    return query_period;
  if (var == "v_ledGpio")
    return String(ledGpioInt);
  if (var == "v_shellyMac")
    return shelly_mac;
  if (var == "v_shellyPort")
    return shelly_port;
  if (var == "v_smaId")
    return sma_id;

  // Booleans for selects
  if (var == "s_ledInv_t")
    return ledInverted ? "selected" : "";
  if (var == "s_ledInv_f")
    return !ledInverted ? "selected" : "";
  if (var == "s_fPwrDec_t")
    return forcePwrDecimals ? "selected" : "";
  if (var == "s_fPwrDec_f")
    return !forcePwrDecimals ? "selected" : "";

  // Data Source Type select
  if (var == "s_MQTT")
    return (strcmp(input_type, "MQTT") == 0) ? "selected" : "";
  if (var == "s_HTTP")
    return (strcmp(input_type, "HTTP") == 0) ? "selected" : "";
  if (var == "s_SMA")
    return (strcmp(input_type, "SMA") == 0) ? "selected" : "";
  if (var == "s_SHRDZM")
    return (strcmp(input_type, "SHRDZM") == 0) ? "selected" : "";
  if (var == "s_SUNSPEC")
    return (strcmp(input_type, "SUNSPEC") == 0) ? "selected" : "";

  // MQTT
  if (var == "v_mqttPort")
    return mqtt_port;
  if (var == "v_mqttTopic")
    return mqtt_topic;
  if (var == "v_mqttUser")
    return mqtt_user;
  if (var == "v_mqttPasswd")
    return mqtt_passwd;

  // Modbus
  if (var == "v_mbDevice")
    return modbus_dev;

  // JSON Paths
  if (var == "v_powerPath")
    return power_path;
  if (var == "v_pwrExpPath")
    return pwr_export_path;
  if (var == "v_powerL1Path")
    return power_l1_path;
  if (var == "v_powerL2Path")
    return power_l2_path;
  if (var == "v_powerL3Path")
    return power_l3_path;
  if (var == "v_energyInPath")
    return energy_in_path;
  if (var == "v_energyOutPath")
    return energy_out_path;

  return String();
}

// ============================================================================
// WEB REQUEST HANDLERS
// ============================================================================

void handleConfig(AsyncWebServerRequest *request) {
  // Send config page with template processing
  // Note: CONFIG_page is in PROGMEM to save RAM
  request->send_P(200, "text/html", CONFIG_page, processor);
}

void handleSave(AsyncWebServerRequest *request) {
  // Helper lambda to get a parameter value
  auto getParam = [&](const char *name) {
    if (request->hasParam(name, true)) {
      return request->getParam(name, true)->value();
    }
    return String();
  };

  // Update global variables from form parameters
  getParam("inputType").toCharArray(input_type, sizeof(input_type));
  getParam("mqttServer").toCharArray(mqtt_server, sizeof(mqtt_server));
  getParam("queryPeriod").toCharArray(query_period, sizeof(query_period));
  String ledGpioStr = getParam("ledGpio");
  ledGpioInt = ledGpioStr.toInt();
  String ledInvertedStr = getParam("ledInverted");
  ledInverted = (ledInvertedStr == "true");
  getParam("shellyMac").toCharArray(shelly_mac, sizeof(shelly_mac));
  getParam("shellyPort").toCharArray(shelly_port, sizeof(shelly_port));
  String forcePwrDecimalsStr = getParam("forcePwrDecimals");
  forcePwrDecimals = (forcePwrDecimalsStr == "true");
  getParam("smaId").toCharArray(sma_id, sizeof(sma_id));
  getParam("mqttPort").toCharArray(mqtt_port, sizeof(mqtt_port));
  getParam("mqttTopic").toCharArray(mqtt_topic, sizeof(mqtt_topic));
  getParam("mqttUser").toCharArray(mqtt_user, sizeof(mqtt_user));
  getParam("mqttPasswd").toCharArray(mqtt_passwd, sizeof(mqtt_passwd));
  getParam("modbusDevice").toCharArray(modbus_dev, sizeof(modbus_dev));
  getParam("powerPath").toCharArray(power_path, sizeof(power_path));
  getParam("pwrExportPath")
      .toCharArray(pwr_export_path, sizeof(pwr_export_path));
  getParam("powerL1Path").toCharArray(power_l1_path, sizeof(power_l1_path));
  getParam("powerL2Path").toCharArray(power_l2_path, sizeof(power_l2_path));
  getParam("powerL3Path").toCharArray(power_l3_path, sizeof(power_l3_path));
  getParam("energyInPath").toCharArray(energy_in_path, sizeof(energy_in_path));
  getParam("energyOutPath")
      .toCharArray(energy_out_path, sizeof(energy_out_path));

  // Save all settings to Preferences
  saveConfiguration();

  String response = "<html><head><title>Settings Saved</title><meta "
                    "http-equiv='refresh' content='5;url=/'></head><body>";
  response += "<h1>Settings Saved</h1>";
  response += "<p>The device will now restart to apply the changes.</p>";
  response += "<p>You will be redirected to the home page in 5 seconds. If "
              "not, please <a href='/'>click here</a>.</p>";
  response += "</body></html>";
  request->send(200, "text/html", response);

  shouldReboot = true;
}
