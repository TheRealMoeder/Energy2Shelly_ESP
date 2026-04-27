#ifndef HTML_CONFIG_H
#define HTML_CONFIG_H

#include <Arduino.h>

const char HTML_CONFIG[] PROGMEM = R"=====(
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
      <option value="TIBBERPULSE" %s_TIBBERPULSE%>TibberPulse SML data</option>
    </select>
    <label for="mqttServer">Server / URL</label>
    <input type='text' id='mqttServer' name='mqttServer' value='%v_mqttServer%'>
    <label for="queryPeriod">Query Period (ms)</label>
    <input type='text' id='queryPeriod' name='queryPeriod' value='%v_qPeriod%'>
    <label for="ntpServer">NTP Server</label>
    <input type='text' id='ntpServer' name='ntpServer' value='%v_ntpServer%'>
    <label for="timezone">Timezone</label>
    <input type='text' id='timezone' name='timezone' value='%v_timezone%'>
    <label for="phaseNumber">Number of phases</label>
    <input type='text' id='phaseNumber' name='phaseNumber' value='%v_phasenumber%'>
    <label for="powerOffset">Power offset</label>
    <input type='text' id='powerOffset' name='powerOffset' value='%v_powerOffset%'>
    <label for="ledGpio">LED GPIO</label>
    <input type='text' id='ledGpio' name='ledGpio' value='%v_ledGpio%'>
    <label for="ledGpioInv">LED GPIO Inverted</label>
    <input type='text' id='ledGpioInv' name='ledGpioInv' value='%v_ledGpioInv%'>
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
    <input type='text' id='modbusDevice' name='modbusDevice' value='%v_modbusDevice%'>
  </fieldset>

  <fieldset>
    <legend>JSON Paths</legend>
    <label for="powerPath">Total Power Path</label>
    <input type='text' id='powerPath' name='powerPath' value='%v_powerPath%'>
    <label for="pwrExportPath">Export Power Path</label>
    <input type='text' id='pwrExportPath' name='pwrExportPath' value='%v_pwrExportPath%'>
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

  <fieldset>
    <legend>TibberPulse Options</legend>
    <label for="tibberHost">Tibber Host</label>
    <input type='text' id='tibberHost' name='tibberHost' value='%v_tibberHost%'>
    <label for="tibberNodeId">Tibber Node ID</label>
    <input type='text' id='tibberNodeId' name='tibberNodeId' value='%v_tibberNodeId%'>
    <label for="tibberUser">Tibber User</label>
    <input type='text' id='tibberUser' name='tibberUser' value='%v_tibberUser%'>
    <label for="tibberPassword">Tibber Password</label>
    <input type='text' id='tibberPassword' name='tibberPassword' value='%v_tibberPassword%'>
  </fieldset>

  <button type='submit' class='btn btn-save'>Save Configuration</button>
</form>
<p class="note">Device will restart after saving.</p>
</body>
</html>
)=====";

const char HTML_SAVE[] PROGMEM = R"=====(
<html><head><title>Settings Saved</title><meta http-equiv='refresh' content='5;url=/'></head><body>
<h1>Settings Saved</h1>
<p>The device will now restart to apply the changes.</p>
<p>You will be redirected to the home page in 5 seconds. If not, please <a href='/'>click here</a>.</p>
</body></html>
)=====";

#endif // HTML_CONFIG_H