#include "WebConfig.h"
#include "Configuration.h"
#include "../security/CsrfProtection.h"
#include "../web/html_config_template.h"
#include "../web/html_save_success.h"
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
<link rel="icon" href="/favicon.svg" type="image/svg+xml">
<style>
body { font-family: Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 20px; }
h2 { text-align: center; color: #333; }
form { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 600px; margin: 0 auto; }
label { display: block; margin-bottom: 5px; font-weight: bold; color: #555; }
input[type=text], input[type=password], select { width: 100%; padding: 8px; margin-bottom: 15px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
.path-row { display: flex; gap: 10px; align-items: center; margin-bottom: 15px; }
.path-row input[type=text] { flex: 1; margin-bottom: 0; }
.path-row label.checkbox-label { font-weight: normal; display: flex; align-items: center; gap: 5px; margin-bottom: 0; white-space: nowrap; }
.path-row input[type=checkbox] { width: auto; margin: 0; }
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
    <div class="path-row">
      <input type='text' id='powerPath' name='powerPath' value='%v_powerPath%'>
      <label class="checkbox-label"><input type='checkbox' name='negatePowerPath' %c_negatePowerPath%> Negate</label>
    </div>
    <label for="pwrExportPath">Export Power Path</label>
    <div class="path-row">
      <input type='text' id='pwrExportPath' name='pwrExportPath' value='%v_pwrExpPath%'>
      <label class="checkbox-label"><input type='checkbox' name='negatePwrExportPath' %c_negatePwrExportPath%> Negate</label>
    </div>
    <label for="powerL1Path">Phase 1 Power Path</label>
    <div class="path-row">
      <input type='text' id='powerL1Path' name='powerL1Path' value='%v_powerL1Path%'>
      <label class="checkbox-label"><input type='checkbox' name='negatePowerL1Path' %c_negatePowerL1Path%> Negate</label>
    </div>
    <label for="powerL2Path">Phase 2 Power Path</label>
    <div class="path-row">
      <input type='text' id='powerL2Path' name='powerL2Path' value='%v_powerL2Path%'>
      <label class="checkbox-label"><input type='checkbox' name='negatePowerL2Path' %c_negatePowerL2Path%> Negate</label>
    </div>
    <label for="powerL3Path">Phase 3 Power Path</label>
    <div class="path-row">
      <input type='text' id='powerL3Path' name='powerL3Path' value='%v_powerL3Path%'>
      <label class="checkbox-label"><input type='checkbox' name='negatePowerL3Path' %c_negatePowerL3Path%> Negate</label>
    </div>
    <label for="energyInPath">Energy In Path</label>
    <div class="path-row">
      <input type='text' id='energyInPath' name='energyInPath' value='%v_energyInPath%'>
      <label class="checkbox-label"><input type='checkbox' name='negateEnergyInPath' %c_negateEnergyInPath%> Negate</label>
    </div>
    <label for="energyOutPath">Energy Out Path</label>
    <div class="path-row">
      <input type='text' id='energyOutPath' name='energyOutPath' value='%v_energyOutPath%'>
      <label class="checkbox-label"><input type='checkbox' name='negateEnergyOutPath' %c_negateEnergyOutPath%> Negate</label>
    </div>
  </fieldset>

  <button type='submit' class='btn btn-save'>Save Configuration</button>
</form>
<p class="note">Device will restart after saving.</p>

<div style="max-width: 600px; margin: 20px auto; text-align: center;">
  <button onclick="exportConfig()" class='btn' style="background-color: #007bff; color: white; margin-bottom: 10px;">Export Configuration</button>
  <input type="file" id="importFile" accept=".json" style="display:none;" onchange="importConfig(event)">
  <button onclick="document.getElementById('importFile').click()" class='btn' style="background-color: #6c757d; color: white;">Import Configuration</button>
</div>

<script>
function exportConfig() {
  fetch('/config/export')
    .then(response => response.json())
    .then(data => {
      const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'energy2shelly_config.json';
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    })
    .catch(error => alert('Error exporting configuration: ' + error));
}

function importConfig(event) {
  const file = event.target.files[0];
  if (!file) return;

  const reader = new FileReader();
  reader.onload = function(e) {
    try {
      const config = JSON.parse(e.target.result);

      // Populate form fields
      if (config.inputType) document.getElementById('inputType').value = config.inputType;
      if (config.mqttServer) document.getElementById('mqttServer').value = config.mqttServer;
      if (config.mqttPort) document.getElementById('mqttPort').value = config.mqttPort;
      if (config.mqttTopic) document.getElementById('mqttTopic').value = config.mqttTopic;
      if (config.mqttUser) document.getElementById('mqttUser').value = config.mqttUser;
      if (config.mqttPasswd) document.getElementById('mqttPasswd').value = config.mqttPasswd;
      if (config.queryPeriod) document.getElementById('queryPeriod').value = config.queryPeriod;
      if (config.ledGpio !== undefined) document.getElementById('ledGpio').value = config.ledGpio;
      if (config.ledInverted !== undefined) document.getElementById('ledInverted').value = config.ledInverted ? 'true' : 'false';
      if (config.shellyMac) document.getElementById('shellyMac').value = config.shellyMac;
      if (config.shellyPort) document.getElementById('shellyPort').value = config.shellyPort;
      if (config.forcePwrDecimals !== undefined) document.getElementById('forcePwrDecimals').value = config.forcePwrDecimals ? 'true' : 'false';
      if (config.smaId) document.getElementById('smaId').value = config.smaId;
      if (config.modbusDevice) document.getElementById('modbusDevice').value = config.modbusDevice;
      if (config.powerPath) document.getElementById('powerPath').value = config.powerPath;
      if (config.pwrExportPath) document.getElementById('pwrExportPath').value = config.pwrExportPath;
      if (config.powerL1Path) document.getElementById('powerL1Path').value = config.powerL1Path;
      if (config.powerL2Path) document.getElementById('powerL2Path').value = config.powerL2Path;
      if (config.powerL3Path) document.getElementById('powerL3Path').value = config.powerL3Path;
      if (config.energyInPath) document.getElementById('energyInPath').value = config.energyInPath;
      if (config.energyOutPath) document.getElementById('energyOutPath').value = config.energyOutPath;

      // Set negation checkboxes
      if (config.negatePowerPath !== undefined) document.querySelector('input[name="negatePowerPath"]').checked = config.negatePowerPath;
      if (config.negatePwrExportPath !== undefined) document.querySelector('input[name="negatePwrExportPath"]').checked = config.negatePwrExportPath;
      if (config.negatePowerL1Path !== undefined) document.querySelector('input[name="negatePowerL1Path"]').checked = config.negatePowerL1Path;
      if (config.negatePowerL2Path !== undefined) document.querySelector('input[name="negatePowerL2Path"]').checked = config.negatePowerL2Path;
      if (config.negatePowerL3Path !== undefined) document.querySelector('input[name="negatePowerL3Path"]').checked = config.negatePowerL3Path;
      if (config.negateEnergyInPath !== undefined) document.querySelector('input[name="negateEnergyInPath"]').checked = config.negateEnergyInPath;
      if (config.negateEnergyOutPath !== undefined) document.querySelector('input[name="negateEnergyOutPath"]').checked = config.negateEnergyOutPath;

      alert('Configuration imported successfully! Please review and click Save to apply.');
    } catch (error) {
      alert('Error parsing configuration file: ' + error.message);
    }
  };
  reader.readAsText(file);
  event.target.value = ''; // Reset file input
}
</script>
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

  // Path negation checkboxes
  if (var == "c_negatePowerPath")
    return negate_power_path ? "checked" : "";
  if (var == "c_negatePwrExportPath")
    return negate_pwr_export_path ? "checked" : "";
  if (var == "c_negatePowerL1Path")
    return negate_power_l1_path ? "checked" : "";
  if (var == "c_negatePowerL2Path")
    return negate_power_l2_path ? "checked" : "";
  if (var == "c_negatePowerL3Path")
    return negate_power_l3_path ? "checked" : "";
  if (var == "c_negateEnergyInPath")
    return negate_energy_in_path ? "checked" : "";
  if (var == "c_negateEnergyOutPath")
    return negate_energy_out_path ? "checked" : "";

  return String();
}

// ============================================================================
// WEB REQUEST HANDLERS
// ============================================================================

void handleConfig(AsyncWebServerRequest *request) {
  DEBUG_SERIAL.println("Config page requested");
  DEBUG_SERIAL.print("Free heap before: ");
  DEBUG_SERIAL.println(ESP.getFreeHeap());

  // Debug: print current configuration values
  DEBUG_SERIAL.println("Current config values:");
  DEBUG_SERIAL.print("  input_type: "); DEBUG_SERIAL.println(input_type);
  DEBUG_SERIAL.print("  mqtt_server: "); DEBUG_SERIAL.println(mqtt_server);
  DEBUG_SERIAL.print("  power_path: "); DEBUG_SERIAL.println(power_path);
  DEBUG_SERIAL.print("  power_l1_path: "); DEBUG_SERIAL.println(power_l1_path);
  DEBUG_SERIAL.print("  energy_in_path: "); DEBUG_SERIAL.println(energy_in_path);

  // Build HTML with dynamic values
  String html = "";
  html.reserve(2048); // Pre-allocate to avoid fragmentation

  // Add header from template
  html = FPSTR(CONFIG_HEADER);

  // Data source selector
  html += F("<label>Data Source</label><select name='inputType'>");
  html += F("<option value='MQTT'"); if (strcmp(input_type, "MQTT") == 0) html += F(" selected"); html += F(">MQTT</option>");
  html += F("<option value='HTTP'"); if (strcmp(input_type, "HTTP") == 0) html += F(" selected"); html += F(">HTTP</option>");
  html += F("<option value='SMA'"); if (strcmp(input_type, "SMA") == 0) html += F(" selected"); html += F(">SMA</option>");
  html += F("<option value='SHRDZM'"); if (strcmp(input_type, "SHRDZM") == 0) html += F(" selected"); html += F(">SHRDZM</option>");
  html += F("<option value='SUNSPEC'"); if (strcmp(input_type, "SUNSPEC") == 0) html += F(" selected"); html += F(">SUNSPEC</option>");
  html += F("</select>");

  html += F("<label>Server/URL</label><input name='mqttServer' value='"); html += mqtt_server; html += F("'>");
  html += F("<label>Query Period (ms)</label><input name='queryPeriod' value='"); html += query_period; html += F("'>");
  html += F("<label>Shelly MAC</label><input name='shellyMac' value='"); html += shelly_mac; html += F("'>");
  html += F("<label>Shelly Port</label><input name='shellyPort' value='"); html += shelly_port; html += F("'>");
  html += F("</fieldset>");

  html += F("<fieldset><legend>MQTT</legend>");
  html += F("<label>Port</label><input name='mqttPort' value='"); html += mqtt_port; html += F("'>");
  html += F("<label>Topic</label><input name='mqttTopic' value='"); html += mqtt_topic; html += F("'>");
  html += F("<label>User</label><input name='mqttUser' value='"); html += mqtt_user; html += F("'>");
  html += F("<label>Password</label><input type='password' name='mqttPasswd' value='"); html += mqtt_passwd; html += F("'>");
  html += F("</fieldset>");

  html += F("<fieldset><legend>Modbus TCP</legend>");
  html += F("<label>Modbus Device ID</label><input name='modbusDevice' value='"); html += modbus_dev; html += F("'>");
  html += F("</fieldset>");

  html += F("<fieldset><legend>JSON Paths</legend>");

  html += F("<label>Total Power Path</label><div class='path-row'><input name='powerPath' value='");
  html += power_path;
  html += F("'><label class='checkbox-label'><input type='checkbox' name='negatePowerPath'");
  if (negate_power_path) html += F(" checked");
  html += F("> Negate</label></div>");

  html += F("<label>Export Power Path</label><div class='path-row'><input name='pwrExportPath' value='");
  html += pwr_export_path;
  html += F("'><label class='checkbox-label'><input type='checkbox' name='negatePwrExportPath'");
  if (negate_pwr_export_path) html += F(" checked");
  html += F("> Negate</label></div>");

  html += F("<label>Phase 1 Power Path</label><div class='path-row'><input name='powerL1Path' value='");
  html += power_l1_path;
  html += F("'><label class='checkbox-label'><input type='checkbox' name='negatePowerL1Path'");
  if (negate_power_l1_path) html += F(" checked");
  html += F("> Negate</label></div>");

  html += F("<label>Phase 2 Power Path</label><div class='path-row'><input name='powerL2Path' value='");
  html += power_l2_path;
  html += F("'><label class='checkbox-label'><input type='checkbox' name='negatePowerL2Path'");
  if (negate_power_l2_path) html += F(" checked");
  html += F("> Negate</label></div>");

  html += F("<label>Phase 3 Power Path</label><div class='path-row'><input name='powerL3Path' value='");
  html += power_l3_path;
  html += F("'><label class='checkbox-label'><input type='checkbox' name='negatePowerL3Path'");
  if (negate_power_l3_path) html += F(" checked");
  html += F("> Negate</label></div>");

  html += F("<label>Energy In Path</label><div class='path-row'><input name='energyInPath' value='");
  html += energy_in_path;
  html += F("'><label class='checkbox-label'><input type='checkbox' name='negateEnergyInPath'");
  if (negate_energy_in_path) html += F(" checked");
  html += F("> Negate</label></div>");

  html += F("<label>Energy Out Path</label><div class='path-row'><input name='energyOutPath' value='");
  html += energy_out_path;
  html += F("'><label class='checkbox-label'><input type='checkbox' name='negateEnergyOutPath'");
  if (negate_energy_out_path) html += F(" checked");
  html += F("> Negate</label></div>");

  html += F("</fieldset>");

  // Add footer with export/import functionality from template
  html += FPSTR(CONFIG_FOOTER);

  request->send(200, "text/html", html);

  DEBUG_SERIAL.println("Config page sent");
  DEBUG_SERIAL.print("Free heap after: ");
  DEBUG_SERIAL.println(ESP.getFreeHeap());
}

void handleSave(AsyncWebServerRequest *request) {
  // CSRF protection: verify request came from this device
  if (!validateCsrfToken(request)) {
    DEBUG_SERIAL.println("CSRF attempt blocked on /save endpoint");
    request->send(403, "text/plain", "Forbidden: Invalid request origin");
    return;
  }

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

  // Get negation checkboxes (unchecked checkboxes don't send values, so default to false)
  negate_power_path = request->hasParam("negatePowerPath", true);
  negate_pwr_export_path = request->hasParam("negatePwrExportPath", true);
  negate_power_l1_path = request->hasParam("negatePowerL1Path", true);
  negate_power_l2_path = request->hasParam("negatePowerL2Path", true);
  negate_power_l3_path = request->hasParam("negatePowerL3Path", true);
  negate_energy_in_path = request->hasParam("negateEnergyInPath", true);
  negate_energy_out_path = request->hasParam("negateEnergyOutPath", true);

  // Save all settings to Preferences
  saveConfiguration();

  DEBUG_SERIAL.println("Configuration saved, device will reboot");

  // Send success page from template
  request->send(200, "text/html", FPSTR(HTML_SAVE_SUCCESS));

  shouldReboot = true;
}

void handleExportConfig(AsyncWebServerRequest *request) {
  JsonDocument jsonConfig;

  // General Settings
  jsonConfig["inputType"] = input_type;
  jsonConfig["mqttServer"] = mqtt_server;
  jsonConfig["queryPeriod"] = query_period;
  jsonConfig["ledGpio"] = ledGpioInt;
  jsonConfig["ledInverted"] = ledInverted;
  jsonConfig["shellyMac"] = shelly_mac;
  jsonConfig["shellyPort"] = shelly_port;
  jsonConfig["forcePwrDecimals"] = forcePwrDecimals;
  jsonConfig["smaId"] = sma_id;

  // MQTT Options
  jsonConfig["mqttPort"] = mqtt_port;
  jsonConfig["mqttTopic"] = mqtt_topic;
  jsonConfig["mqttUser"] = mqtt_user;
  jsonConfig["mqttPasswd"] = mqtt_passwd;

  // Modbus Options
  jsonConfig["modbusDevice"] = modbus_dev;

  // JSON Paths
  jsonConfig["powerPath"] = power_path;
  jsonConfig["pwrExportPath"] = pwr_export_path;
  jsonConfig["powerL1Path"] = power_l1_path;
  jsonConfig["powerL2Path"] = power_l2_path;
  jsonConfig["powerL3Path"] = power_l3_path;
  jsonConfig["energyInPath"] = energy_in_path;
  jsonConfig["energyOutPath"] = energy_out_path;

  // Path negation flags
  jsonConfig["negatePowerPath"] = negate_power_path;
  jsonConfig["negatePwrExportPath"] = negate_pwr_export_path;
  jsonConfig["negatePowerL1Path"] = negate_power_l1_path;
  jsonConfig["negatePowerL2Path"] = negate_power_l2_path;
  jsonConfig["negatePowerL3Path"] = negate_power_l3_path;
  jsonConfig["negateEnergyInPath"] = negate_energy_in_path;
  jsonConfig["negateEnergyOutPath"] = negate_energy_out_path;

  String jsonString;
  serializeJson(jsonConfig, jsonString);

  request->send(200, "application/json", jsonString);
}
