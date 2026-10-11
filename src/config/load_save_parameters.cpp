#include <Preferences.h>
#include "Configuration.h"
#include "load_save_parameters.h"
#include <ArduinoJson.h> 


String json_prefs_string;
JsonDocument update_config_doc;


void mergeJson(JsonObject target, JsonObjectConst source) {
    for (JsonPairConst p : source) {
        JsonVariantConst value = p.value();
        
        if (value.is<JsonObjectConst>()) {
            JsonObject targetChild = target[p.key()].to<JsonObject>();
            mergeJson(targetChild, value.as<JsonObjectConst>());
        } else {
            target[p.key()] = value;
        }
    }
}



void save_parameters() {
    preferences.begin("e2s_config", false);
    preferences.putString("reset_password", config_doc["reset_password"].as<String>());
    preferences.putString("input_type", config_doc["input_type"].as<String>());
    preferences.putString("mqtt_server", config_doc["mqtt_server"].as<String>());
    preferences.putString("mqtt_port", config_doc["mqtt_port"].as<String>());
    preferences.putString("ntp_server", config_doc["ntp_server"].as<String>());
    preferences.putString("timezone", config_doc["timezone"].as<String>());
    preferences.putString("phase_number", config_doc["phase_number"].as<String>());
    preferences.putString("power_offset", config_doc["power_offset"].as<String>());
    preferences.putString("query_period", config_doc["query_period"].as<String>());
    preferences.putString("led_gpio", config_doc["led_gpio"].as<String>());
    preferences.putString("led_gpio_i", config_doc["led_gpio_i"].as<String>());
    preferences.putString("shelly_mac", config_doc["shelly_mac"].as<String>());
    preferences.putString("mqtt_topic", config_doc["mqtt_topic"].as<String>());
    preferences.putString("mqtt_user", config_doc["mqtt_user"].as<String>());
    preferences.putString("mqtt_passwd", config_doc["mqtt_passwd"].as<String>());
    preferences.putString("modbus_dev", config_doc["modbus_dev"].as<String>());
    preferences.putString("power_path", config_doc["power_path"].as<String>());
    preferences.putString("pwr_export_path", config_doc["pwr_export_path"].as<String>());
    preferences.putString("power_l1_path", config_doc["power_l1_path"].as<String>());
    preferences.putString("power_l2_path", config_doc["power_l2_path"].as<String>());
    preferences.putString("power_l3_path", config_doc["power_l3_path"].as<String>());
    preferences.putString("energy_in_path", config_doc["energy_in_path"].as<String>());
    preferences.putString("energy_out_path", config_doc["energy_out_path"].as<String>());
    preferences.putString("shelly_port", config_doc["shelly_port"].as<String>());
    preferences.putString("sma_id", config_doc["sma_id"].as<String>());
    preferences.putString("tibber_host", config_doc["tibber_host"].as<String>());
    preferences.putString("tibber_nodeid", config_doc["tibber_nodeid"].as<String>());
    preferences.putString("tibber_user", config_doc["tibber_user"].as<String>());
    preferences.putString("tibber_password", config_doc["tibber_password"].as<String>());
    preferences.putString("rct_host", config_doc["rct_host"].as<String>());
    preferences.putString("rct_port", config_doc["rct_port"].as<String>());
    preferences.end();

}

void load_parameters() {
  preferences.begin("e2s_config", true);
  config_doc["reset_password"] = preferences.getString("reset_password", reset_password);
  config_doc["input_type"] = preferences.getString("input_type", input_type);
  config_doc["mqtt_server"] = preferences.getString("mqtt_server", mqtt_server);
  config_doc["mqtt_port"] = preferences.getString("mqtt_port", mqtt_port);
  config_doc["ntp_server"] = preferences.getString("ntp_server", ntp_server);
  config_doc["timezone"] = preferences.getString("timezone", timezone);
  config_doc["phase_number"] = preferences.getString("phase_number", phase_number);
  config_doc["power_offset"] = preferences.getString("power_offset", power_offset);
  config_doc["query_period"] = preferences.getString("query_period", query_period);
  config_doc["led_gpio"] = preferences.getString("led_gpio", led_gpio);
  config_doc["led_gpio_i"] = preferences.getString("led_gpio_i", led_gpio_i);
  config_doc["shelly_mac"] = preferences.getString("shelly_mac", shelly_mac);
  config_doc["mqtt_topic"] = preferences.getString("mqtt_topic", mqtt_topic);
  config_doc["mqtt_user"] = preferences.getString("mqtt_user", mqtt_user);
  config_doc["mqtt_passwd"] = preferences.getString("mqtt_passwd", mqtt_passwd);
  config_doc["modbus_dev"] = preferences.getString("modbus_dev", modbus_dev);
  config_doc["power_path"] = preferences.getString("power_path", power_path);
  config_doc["pwr_export_path"] = preferences.getString("pwr_export_path", pwr_export_path);
  config_doc["power_l1_path"] = preferences.getString("power_l1_path", power_l1_path);
  config_doc["power_l2_path"] = preferences.getString("power_l2_path", power_l2_path);
  config_doc["power_l3_path"] = preferences.getString("power_l3_path", power_l3_path);
  config_doc["energy_in_path"] = preferences.getString("energy_in_path", energy_in_path);
  config_doc["energy_out_path"] = preferences.getString("energy_out_path", energy_out_path);
  config_doc["shelly_port"] = preferences.getString("shelly_port", shelly_port);
  config_doc["sma_id"] = preferences.getString("sma_id", sma_id);
  config_doc["tibber_host"] = preferences.getString("tibber_host", tibber_host);
  config_doc["tibber_nodeid"] = preferences.getString("tibber_nodeid", tibber_nodeid);
  config_doc["tibber_user"] = preferences.getString("tibber_user", tibber_user);
  config_doc["tibber_password"] = preferences.getString("tibber_password", tibber_password);
  config_doc["rct_host"] = preferences.getString("rct_host", rct_host);
  config_doc["rct_port"] = preferences.getString("rct_port", rct_port);
  preferences.end();
}


