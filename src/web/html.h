#ifndef HTML_H
#define HTML_H

#include <Arduino.h>
#include "chart_js.h"

// Header files generated in generate_web_assets.py
#include "home_html.h"
#include "webconsole_html.h"
#include "webplotter_html.h"
#include "update_index_html.h"
#include "update_error_html.h"
#include "update_success_html.h"

const char PASSWORD_INPUT_HTML[] PROGMEM = "<input type='password' name='reset_password' placeholder='Enter reset password' required><br/>";

const char RESET_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><title>Reset Confirmation</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>body{font-family:Arial,sans-serif;text-align:center;padding:20px;}
.btn{padding:10px 20px;margin:10px;cursor:pointer;text-decoration:none;display:inline-block;border-radius:5px;font-size:16px;}
.btn-yes{background-color:#d9534f;color:white;border:none;}
.btn-no{background-color:#5bc0de;color:white;border:none;}</style></head><body>
<h2>Reset Configuration?</h2>
<p>Are you sure you want to reset the WiFi configuration? This will clear current WiFi settings and restart the device in AP mode.</p>
<form method='POST' style='display:inline;' accept-charset='UTF-8'>
)rawhtml";

const char RESET_HTML_END[] PROGMEM = R"rawhtml(
<button type='submit' class='btn btn-yes'>Yes, Reset</button>
</form>
<a href='/' class='btn btn-no'>Cancel</a>
</body></html>
)rawhtml";


const char UPDATE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><title>Update Confirmation</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>body{font-family:Arial,sans-serif;text-align:center;padding:20px;}
.btn{padding:10px 20px;margin:10px;cursor:pointer;text-decoration:none;display:inline-block;border-radius:5px;font-size:16px;}
.btn-yes{background-color:#d9534f;color:white;border:none;}
.btn-no{background-color:#5bc0de;color:white;border:none;}</style></head><body>
<h2>Update Configuration?</h2>
<p>Are you sure you want to update the WiFi configuration?</p>
<form method='POST' style='display:inline;' accept-charset='UTF-8'>
)rawhtml";

const char UPDATE_HTML_END[] PROGMEM = R"rawhtml(
<button type='submit' class='btn btn-yes'>Yes, Update</button>
</form>
<a href='/' class='btn btn-no'>Cancel</a>
</body></html>
)rawhtml";

const char UPDATE_SUCCESS_HTML[] PROGMEM = R"rawhtml(

)rawhtml";

// Beautiful upload interface in English
const char UPDATE_INDEX_HTML[] PROGMEM = R"rawhtml(

)rawhtml";

const char UPDATE_ERROR_HTML[] PROGMEM = R"rawhtml(

)rawhtml";

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
  

#define X(name, val)      "\"" #name "\": " TOSTRING(val) ",\n"
#define X_LAST(name, val) "\"" #name "\": " TOSTRING(val) "\n"



const char CONFIG_PAGE_HTML[] PROGMEM = R"=====(  
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Energy2Shelly Config Editor</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .card { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 500px; margin: auto; }
        .form-group { margin-bottom: 15px; }
        label { display: block; font-weight: bold; margin-bottom: 5px; color: #333; }
        input { width: 100%; padding: 8px; box-sizing: border-box; border: 1px solid #ccc; border-radius: 4px; }
        button { background: #007bff; color: white; border: none; padding: 10px 15px; border-radius: 4px; cursor: pointer; width: 100%; font-size: 16px; }
        button:hover { background: #0056b3; }
        #status { margin-top: 15px; font-weight: bold; text-align: center; }
    </style>
</head>
<body>
    <div class="card">
        <h2>Energy2Shelly Config Editor</h2>
        <button onclick="goBack()" style="margin-bottom: 15px;">
        ← Back
        </button>
        <!-- Input fields will be dynamically inserted here by JavaScript -->
        <div id="configForm">Loading configuration...</div>
        
        <button id="saveBtn" style="margin-top: 15px;">Save Changes</button>
        <button id="rebootBtn" style="margin-top: 15px; background-color: #dc3545; color: white;">Restart Device</button>
        <p id="status"></p>
    </div>

    

    <script>
    function goBack() {
        if (document.referrer) {
            window.history.back();
        } else {
            window.location.href = '/'; 
        }
    }
    
document.getElementById('rebootBtn').addEventListener('click', async () => {

    const statusText = document.getElementById('status');
    statusText.innerText = "Sending reboot command...";
    statusText.style.color = "orange";

    const redirectToHome = () => {
        setTimeout(() => {
            window.location.href = '/'; 
        }, 20000); 
    };
    try {
        const res = await fetch('/reboot', { method: 'POST' }); 
        
        if (res.ok) {
            statusText.innerText = "Device is restarting... Redirecting to homepage in 20 seconds.";
            statusText.style.color = "green";
            
            document.getElementById('saveBtn').disabled = true;
            document.getElementById('rebootBtn').disabled = true;

            redirectToHome();
        } else {
            statusText.innerText = `Reboot failed (Status: ${res.status}).`;
            statusText.style.color = "red";
        }
    } catch (err) {
        statusText.innerText = "Device disconnected. Restarting... Redirecting to homepage in 20 seconds.";
        statusText.style.color = "green";
        
        document.getElementById('saveBtn').disabled = true;
        document.getElementById('rebootBtn').disabled = true;
        
        redirectToHome();
    }
});



        const formContainer = document.getElementById('configForm');
        const saveBtn = document.getElementById('saveBtn');
        const statusText = document.getElementById('status');
        let currentConfig = {};
        const CONFIG_DICTIONARY = {
            wifi: {
                _label: "WiFi Settings",
                _description: "Settings for connecting to WiFi networks.",
                ssid: {
                    label: "SSID",
                    max_length : 32,
                    description: "The name of the WiFi network."
                },
                password: {
                    label: "Password",
                    max_length : 64,
                    description: "The password for the WiFi network."
                }
            },
            basic: {
                _label: "Basic Settings",
                _description: "General settings for the device.",
                reset_password: {
                    label: "Reset Password",
                    max_length : 33,
                    description: "For resetting the WiFi configuration and putting the device in AP / config mode"
                },
                input_type: {
                    label: "Input Type",
                    description: "Defines the type of input for the device.",
                    options: ["NONE", "MQTT", "HTTP", "SMA", "SUNSPEC", "RCT", "SHRDZM", "TIBBERPULSE", "SMLSERIAL" ] 
                },
                ntp_server: {
                    label: "NTP Server",
                    max_length : 40,
                    description: "The NTP server used for time synchronization. e.g. de.pool.ntp.org"
                },
                timezone: {
                    label: "Timezone",
                    max_length : 64,
                    description: "The timezone setting for the device. e.g. CET-1CEST,M3.5.0/2,M10.5.0/3"
                },
                phase_number: {
                    label: "Phase Number",
                    description: "The number of phases for the device (e.g., 1 or 3).",
                    options: [1, 3]
                },
                power_offset: {
                    label: "Power Offset (Watt)",
                    max_length : 10,
                    description: "The power offset value for calibration."
                }
            },

            sunspec: {
                _label: "SUNSPEC Settings",
                _description: "Settings related to SUNSPEC Modbus communication.",
                _show_if: ["SUNSPEC"],
                modbus_dev: {
                    label: "Modbus Device",
                    max_length : 6,
                    description: "The device number for Modbus communication (e.g., 71)."
                },
                modbus_ip: {
                    label: "Modbus Server IP",
                    max_length : 60,
                    description: "The IP address of the Modbus server."
                },
                modbus_port: {
                    label: "Modbus Port",
                    max_length : 6,
                    description: "The port number for Modbus communication. e.g. 502"
                },
                query_period: {
                    label: "Query Period (ms)",
                    description: "The period in milliseconds for querying the Modbus server."
                }
            },

            rct: {
                _label: "RCT Settings",
                _description: "Settings related to RCT inverter communication.",
                _show_if: ["RCT"],
                rct_host: {
                    label: "RCT hostname/IP",
                    max_length : 41,
                    description: "IP address or hostname of the RCT Power device (with integrated grid sensor)."
                },
                rct_port: {
                    label: "RCT port",
                    max_length : 6,
                    description: "Defaults to 8899."
                },
                query_period: {
                    label: "Query Period (ms)",
                    max_length : 26,
                    description: "The period in milliseconds for querying the RCT inverter."
                }
            },

            sma: {
                _label: "SMA Settings",
                _description: "Settings related to SMA Speedwire communication.",
                _show_if: ["SMA"],
                sma_id: {
                    label: "SMA serial number",
                    max_length : 17,
                    description: "Optional serial number if you have more than one SMA EM/HM in your network"
                }
            },

            tibber: {
                _label: "TibberPulse Settings",
                _description: "Settings related to TibberPulse communication.",
                _show_if: ["TIBBERPULSE"],
                tibber_host: {
                    label: "Tibber Hostnane/IP[:port]",
                    max_length : 41,
                    description: "The address of the  Tibber device (e.g. 192.168.0.1:8080)."
                },
                tibber_user: {
                    label: "Tibber User",
                    max_length : 11,
                    description: "Defaults to admin."
                },
                tibber_password: {
                    label: "Tibber Password",
                    max_length : 10,
                    description: "As Printed on bridge device: xxxx-xxxx."
                },
                tibber_nodeid: {
                    label: "Tibber Node ID",
                    max_length : 2,
                    description: "Defaults to 1, check the bridge web interface for the right nodeID."
                },
                query_period: {
                    label: "Query Period (ms)",
                    max_length : 26,
                    description: "The period in milliseconds for querying the Tibber device."
                }
            },

            smlserial: {
                _label: "SML Serial Settings",
                _description: "SML data from an IR reading head connected directly to a UART pin of the ESP. Requires a reboot after saving.",
                _show_if: ["SMLSERIAL"],
                rx_pin: {
                    label: "RX GPIO",
                    max_length : 3,
                    description: "GPIO connected to the TX line of the reading head (e.g. 20 for ESP32-C3 SuperMini, 16 for ESP32). Boards with a USB-UART chip on GPIO20 need another pin, e.g. 4 or 10. Flash and USB pins are rejected."
                },
                baud: {
                    label: "Baud rate",
                    description: "Most SML meters send with 9600 baud, 8N1.",
                    options: [9600, 2400, 4800, 19200, 38400, 57600, 115200]
                },
                invert: {
                    label: "Invert RX signal",
                    description: "Only needed for reading heads without inverting output stage (e.g. bare phototransistor).",
                    options: ["false", "true"]
                }
            },

            mqtt: {
                _label: "MQTT Settings",
                _description: "Settings related to MQTT communication.",
                _show_if: ["MQTT"],
                mqtt_server: {
                    label: "MQTT Hostname/IP[:port]",
                    max_length : 60,
                    description: "The address of the MQTT broker (e.g. 192.168.0.1:1883)."
                },
                mqtt_port: {
                    label: "MQTT Port",
                    max_length : 6,
                    description: "The port number for MQTT communication."
                },
                mqtt_user: {
                    label: "MQTT User",
                    max_length : 40,
                    description: "Defaults to admin."
                },
                mqtt_password: {
                    label: "MQTT Password",
                    max_length : 40,
                    description: "The password for the MQTT broker."
                },
                mqtt_topic: {
                    label: "MQTT Topic",
                    max_length : 90,
                    description: "The topic to which the device will read published data."
                }
                
            },   
            http: {
                _label: "HTTP Settings",
                _description: "Settings related to HTTP communication.",
                _show_if: ["HTTP"],
                server_ip: {
                    label: "HTTP Hostname/IP[:port]",
                    max_length : 60,
                    description: "The address of the HTTP server (e.g. 192.168.0.1:8080)."
                },
                query_period: {
                    label: "Query Period (ms)",
                    max_length : 26,
                    description: "The period in milliseconds for querying the HTTP server."
                }              
            },
   
            json: {
                _label: "JSON Settings",
                _description: "Settings related to JSON communication. (HTTP/MQTT)",
                _show_if: ["HTTP", "MQTT"],
                power_path: {
                    label: "Power Path",
                    description: "The path to retrieve power data from the JSON server.",
                    max_length : 60
                },
                pwr_export_path: {
                    label: "Power Export Path",
                    max_length : 60,
                    description: "The path to retrieve power export data from the JSON server."
                },
                power_l1_path: {
                    label: "Power L1 Path",
                    max_length : 60,
                    description: "The path to retrieve power data for phase L1 from the JSON server."
                },
                power_l2_path: {
                    label: "Power L2 Path",
                    max_length : 60,
                    description: "The path to retrieve power data for phase L2 from the JSON server."
                },
                power_l3_path: {
                    label: "Power L3 Path",
                    max_length : 60,
                    description: "The path to retrieve power data for phase L3 from the JSON server."
                },
                energy_in_path: {
                    label: "Energy In Path",
                    max_length : 60,
                    description: "The path to retrieve energy input data from the JSON server."
                },
                energy_out_path: {  
                    label: "Energy Out Path",
                    max_length : 60,
                    description: "The path to retrieve energy output data from the JSON server."
                }
            },

            led: {
                _label: "LED Settings",
                _description: "LED settings for the device.",
                led_gpio: {
                    label: "LED Pin",
                    max_length : 6,
                    description: "Please specify the LED pin number"
                },
                led_gpio_i: {
                    label: "Inverse LED",
                    description: "Invert LED on/off logic",
                    options: ["false", "true"] 
                }
            },

            shelly: {
                _label: "Shelly Settings",
                _description: "Settings related to Shelly communication.",
                shelly_mac: {
                    label: "Shelly MAC Address",
                    max_length : 13,
                    description: "The MAC address of the Shelly device."
                },
                shelly_name: {
                    label: "Shelly Device Name",
                    max_length : 26,
                    description: "The name of the Shelly device. e.g. shellypro3em-"
                },
                shelly_gen: {
                    label: "Shelly Generation",
                    description: "The generation of the Shelly device (e.g., 1, 2, 3).",
                    options: [1, 2, 3]
                },
                shelly_fw_id: {
                    label: "Shelly Firmware ID",
                    max_length : 32,
                    description: "The firmware ID of the Shelly device. e.g. 20241011-114455/1.4.4-g6d2a586"
                },
                shelly_port: {
                    label: "Shelly Port",
                    max_length : 6,
                    description: "The port number for communication with the Shelly device. e.g. 2220"
                }
            },
            console: {
                _label: "Console Settings",
                _description: "Manage data output of Serial and Console printing",
                web_console: {
                    label: "Enable web console",
                    max_length : 6,
                    description: "With set to false, console web view is disabled",
                    options : ["false", "true"]
                },
                serial_console_time: {
                    label: "Live time for serial console",
                    max_length : 30,
                    description: "Live time of the serial output in minutes after reset.",
                }
            }     
        };

   async function loadConfig() {
    try {
        const res = await fetch('/json_config');
        if (res.status === 401) {
            setTimeout(loadConfig, 500); 
            return; 
        }
        
        currentConfig = await res.json();
       
        if (currentConfig.wifi && currentConfig.wifi.networks) {
            const uniqueNetworksMap = new Map();
    
            currentConfig.wifi.networks.forEach(net => {
                if (net.ssid && net.ssid.trim() !== "") {
                    const existing = uniqueNetworksMap.get(net.ssid);
                    if (!existing || net.rssi > existing.rssi) {
                        uniqueNetworksMap.set(net.ssid, net);
                    }
                }
            });

            CONFIG_DICTIONARY.wifi.ssid.options = Array.from(uniqueNetworksMap.values());
            delete currentConfig.wifi.networks;   
        }


        formContainer.innerHTML = ''; 
        generateFormFromDictionary(CONFIG_DICTIONARY, currentConfig, formContainer);
          const inputTypeField = document.getElementById('input_basic_input_type');

        if (inputTypeField) {
                inputTypeField.addEventListener('change', (e) => {
                applyDynamicVisibility(e.target.value);
            });

            applyDynamicVisibility(inputTypeField.value);
        } 
        statusText.innerText = "Configuration successfully loaded.";
        statusText.style.color = "green";
    } catch (err) {
        statusText.innerText = "Error loading configuration.";
        statusText.style.color = "red";
    }
}
    const INPUT_TYPE_ENUM = {
)=====" INPUT_TYPE_LIST  R"=====(
   
};

function applyDynamicVisibility(currentInputType) {
    const conditionalSections = document.querySelectorAll('fieldset[data-show-if]'); 
    let activeTypeName = currentInputType;
    
    if (typeof INPUT_TYPE_ENUM !== 'undefined') {
        const foundKey = Object.keys(INPUT_TYPE_ENUM).find(key => String(INPUT_TYPE_ENUM[key]) === String(currentInputType));
        if (foundKey) activeTypeName = foundKey;
    }

    conditionalSections.forEach(fieldset => {
        let allowedTypes = [];
        try {
            allowedTypes = JSON.parse(fieldset.dataset.showIf);
        } catch (e) {
            return; 
        }
        
        const isMatch = allowedTypes.some(type => 
            String(type).toUpperCase() === String(currentInputType).toUpperCase() ||
            String(type).toUpperCase() === String(activeTypeName).toUpperCase()
        );

        
        if (isMatch) {
            fieldset.style.display = 'block'; 
        } else {
            fieldset.style.display = 'none';
        }
    });
}
function generateFormFromDictionary(dictionary, currentConfig, container, currentConfigNode = currentConfig, prefix = '') {
    Object.keys(dictionary).forEach(key => {
        const entry = dictionary[key];
       
        const currentPath = prefix ? `${prefix}_${key}` : key;

        if (entry && typeof entry === 'object' && !entry.label && !entry.options  && !key.startsWith('_')) {
            const fieldset = document.createElement('fieldset');
            fieldset.className = 'form-group-sub';
            fieldset.style.marginBottom = '25px';
            fieldset.style.padding = '15px';
            fieldset.style.border = '1px solid #ccc';
            fieldset.style.borderRadius = '5px';
            
           

            const legend = document.createElement('legend');
            legend.innerText = entry._label || key;
            legend.style.fontWeight = 'bold';
            legend.style.padding = '0 10px';
            fieldset.appendChild(legend);

            if (entry._description) {
                const desc = document.createElement('small');
                desc.innerText = entry._description;
                desc.style.display = 'block';
                desc.style.color = '#666';
                desc.style.marginBottom = '10px';
                fieldset.appendChild(desc);
            }
            if (entry._show_if) {
                fieldset.dataset.showIf = JSON.stringify(entry._show_if);
            }

            const nextConfigNode = (currentConfigNode && currentConfigNode[key]) ? currentConfigNode[key] : {};

            generateFormFromDictionary(entry, currentConfig, fieldset, nextConfigNode, currentPath);
            container.appendChild(fieldset);
            return;
        }

        if (key.startsWith('_')) return;

        const group = document.createElement('div');
        group.className = 'form-group';
        group.style.marginBottom = '20px';

        const inputWrapper = document.createElement('div');
        inputWrapper.style.display = 'flex';
        inputWrapper.style.alignItems = 'center';

        const displayLabel = entry.label || key;
        const displayDesc = entry.description || '';

        const label = document.createElement('label');
        label.innerText = displayLabel;
        label.style.fontWeight = 'bold';
        label.style.display = 'block';

        let descElement = null;
        if (displayDesc) {
            descElement = document.createElement('small');
            descElement.innerText = displayDesc;
            descElement.style.display = 'block';
            descElement.style.color = '#666';
            descElement.style.marginBottom = '4px';
        }

        group.appendChild(label);
        if (descElement) group.appendChild(descElement);
        group.appendChild(inputWrapper);

        let input;
        const configValue = (currentConfigNode && currentConfigNode[key] !== undefined) ? currentConfigNode[key] : '';
        
if (entry.options) {
    input = document.createElement('select');
    input.id = 'input_' + currentPath; 

    entry.options.forEach(opt => {
        const option = document.createElement('option');
        
        const isNetworkObj = typeof opt === 'object' && opt !== null;
        const ssidValue = isNetworkObj ? opt.ssid : opt;
        const rssiValue = isNetworkObj ? opt.rssi : null;

        let signalText = "";
        if (rssiValue !== null) {
            const percentage = Math.min(Math.max(2 * (rssiValue + 100), 0), 100);            
            signalText = ` (${percentage}%)`;
        }

        option.text = ssidValue + signalText; 
        
        const mappedValue = INPUT_TYPE_ENUM[ssidValue] !== undefined ? INPUT_TYPE_ENUM[ssidValue] : ssidValue;
        option.value = mappedValue;

        if (configValue === mappedValue || String(configValue) === String(mappedValue) || configValue === ssidValue) {
            option.selected = true;
        }
        input.appendChild(option);
    });

    inputWrapper.appendChild(input);
       }  
        else {
            input = document.createElement('input');
            input.id = 'input_' + currentPath; 
            if (entry.max_length) {
                input.maxLength = entry.max_length; 
            }
                    

            input.value = configValue;
            inputWrapper.appendChild(input);

       

            if (key.toLowerCase().includes('password')) {
                input.type = 'password';

                const toggleBtn = document.createElement('button');
                toggleBtn.type = 'button';
                toggleBtn.innerText = '👁️';
                toggleBtn.style.marginLeft = '8px';
                toggleBtn.style.cursor = 'pointer';
                toggleBtn.style.background = 'none';
                toggleBtn.style.border = 'none';

                toggleBtn.addEventListener('click', () => {
                    if (input.type === 'password') {
                        input.type = 'text';
                        toggleBtn.innerText = '🔒';
                    } else {
                        input.type = 'password';
                        toggleBtn.innerText = '👁️';
                    }
                });

                inputWrapper.appendChild(toggleBtn);
            } else {
                input.type = 'text';
            }
        }

        container.appendChild(group);
    });
}
async function saveConfig() {
    const castValue = (val) => isNaN(val) || val.trim() === "" ? val : Number(val);

   function buildPayload(configObj, dictionaryObj, prefix = '') {
        const result = {};

        Object.keys(dictionaryObj).forEach(key => {
            if (key.startsWith('_')) return; // Metadaten überspringen

            const currentPath = prefix ? `${prefix}_${key}` : key;
            const inputField = document.getElementById('input_' + currentPath);

            if (inputField) {
                const lowerKey = key.toLowerCase();
                if (lowerKey.includes('password') || lowerKey.includes('ssid')) {
                    result[key] = String(inputField.value); // Niemals in Number konvertieren
                } else {
                    result[key] = castValue(inputField.value); // Normale Konvertierung für den Rest
                }
            } else if (typeof dictionaryObj[key] === 'object' && dictionaryObj[key] !== null && !dictionaryObj[key].hasOwnProperty('label')) {
                const subConfig = (configObj && configObj[key]) ? configObj[key] : {};
                result[key] = buildPayload(subConfig, dictionaryObj[key], currentPath);
            } else if (configObj && configObj.hasOwnProperty(key)) {
                result[key] = configObj[key];
            }
        });

        if (configObj) {
            Object.keys(configObj).forEach(key => {
                if (!result.hasOwnProperty(key)) {
                    result[key] = configObj[key];
                }
            });
        }

        return result;
    }
    const payload = buildPayload(currentConfig, CONFIG_DICTIONARY);

    try {
        statusText.innerText = "Saving to device...";
        statusText.style.color = "orange";

        const res = await fetch('/json_config', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload) 
        });

        if (res.ok) {
            statusText.innerText = "Successfully saved to flash storage! Please restart the device to make sure all changes are in effect.";
            statusText.style.color = "green";
        } else {
            statusText.innerText = "Device rejected the save request.";
            statusText.style.color = "red";
        }
    } catch (err) {
        statusText.innerText = "Network error while saving.";
        statusText.style.color = "red";
    }
}
            saveBtn.addEventListener('click', saveConfig);
        window.addEventListener('DOMContentLoaded', loadConfig);
        loadConfig(); // Initial load
    </script>
</body>
</html>
)=====";
#undef X
#undef X_LAST
#endif // HTML_HOME_H