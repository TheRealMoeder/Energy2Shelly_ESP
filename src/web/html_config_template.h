#ifndef HTML_CONFIG_TEMPLATE_H
#define HTML_CONFIG_TEMPLATE_H

#include <Arduino.h>

// Config page header and styles
const char CONFIG_HEADER[] PROGMEM = R"=====(
<!DOCTYPE html><html><head><title>E2S Configuration</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<link rel='icon' href='/favicon.svg' type='image/svg+xml'>
<style>
body{font-family:Arial;background:#f4f4f4;padding:20px}
form{background:#fff;padding:20px;border-radius:8px;max-width:600px;margin:0 auto}
label{display:block;margin:10px 0 5px;font-weight:bold}
input,select{width:100%;padding:8px;margin-bottom:10px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box}
.btn{padding:10px;border:none;border-radius:5px;width:100%;background:#4CAF50;color:#fff;font-size:16px;cursor:pointer}
fieldset{border:1px solid #ddd;padding:15px;margin-bottom:15px;border-radius:4px}
</style></head><body><h2 style='text-align:center'>Configuration</h2>
<form method='POST' action='/save'><fieldset><legend>General</legend>
)=====";

// Config page footer with export/import functionality
const char CONFIG_FOOTER[] PROGMEM = R"=====(
<button class='btn'>Save & Restart</button></form>
<div style='max-width:600px;margin:20px auto;text-align:center'>
<button onclick='exportConfig()' style='padding:10px 20px;margin:5px;background:#007bff;color:#fff;border:none;border-radius:5px;cursor:pointer'>Export Config</button>
<input type='file' id='importFile' accept='.json' style='display:none' onchange='importConfig(event)'>
<button onclick='document.getElementById("importFile").click()' style='padding:10px 20px;margin:5px;background:#6c757d;color:#fff;border:none;border-radius:5px;cursor:pointer'>Import Config</button>
</div>
<p style='text-align:center;margin-top:20px'><a href='/'>Back to Home</a></p>
<script>
function exportConfig(){
fetch('/config/export').then(r=>r.json()).then(data=>{
const blob=new Blob([JSON.stringify(data,null,2)],{type:'application/json'});
const url=URL.createObjectURL(blob);
const a=document.createElement('a');
a.href=url;a.download='energy2shelly_config.json';
document.body.appendChild(a);a.click();
document.body.removeChild(a);URL.revokeObjectURL(url);
}).catch(e=>alert('Export error: '+e));}
function importConfig(e){
const file=e.target.files[0];if(!file)return;
const reader=new FileReader();
reader.onload=function(ev){try{
const cfg=JSON.parse(ev.target.result);
if(cfg.inputType)document.querySelector('[name=inputType]').value=cfg.inputType;
if(cfg.mqttServer)document.querySelector('[name=mqttServer]').value=cfg.mqttServer;
if(cfg.queryPeriod)document.querySelector('[name=queryPeriod]').value=cfg.queryPeriod;
if(cfg.shellyMac)document.querySelector('[name=shellyMac]').value=cfg.shellyMac;
if(cfg.shellyPort)document.querySelector('[name=shellyPort]').value=cfg.shellyPort;
if(cfg.mqttPort)document.querySelector('[name=mqttPort]').value=cfg.mqttPort;
if(cfg.mqttTopic)document.querySelector('[name=mqttTopic]').value=cfg.mqttTopic;
if(cfg.mqttUser)document.querySelector('[name=mqttUser]').value=cfg.mqttUser;
if(cfg.mqttPasswd)document.querySelector('[name=mqttPasswd]').value=cfg.mqttPasswd;
alert('Config imported! Review and Save.');
}catch(err){alert('Import error: '+err.message);}};
reader.readAsText(file);e.target.value='';
}
</script>
</body></html>
)=====";

#endif // HTML_CONFIG_TEMPLATE_H
