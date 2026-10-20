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
#if 0
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
    preferences.putString("mqtt_passwd", config_doc["mqtt_password"].as<String>());
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
    #else
    //preferences.clear(); // clear all old settings
    serializeJson(config_doc, json_prefs_string);
    preferences.putString("config_doc", json_prefs_string);
    #endif
    preferences.end();

}

void load_parameters() {
  preferences.begin("e2s_config", false);
  config_doc.clear();
  if (preferences.isKey("config_doc")) {
    json_prefs_string = preferences.getString("config_doc", "");
    deserializeJson(config_doc, json_prefs_string);
  } else {
    // Load individual parameters if the combined config_doc is not present
    config_doc.clear();
    config_doc["basic"]["reset_password"] = preferences.getString("reset_password", "");
    config_doc["basic"]["input_type"] = preferences.getString("input_type", "");
    config_doc["basic"]["ntp_server"] = preferences.getString("ntp_server", "");
    config_doc["basic"]["timezone"] = preferences.getString("timezone", "");
    config_doc["basic"]["phase_number"] = preferences.getString("phase_number", "");
    config_doc["basic"]["power_offset"] = preferences.getString("power_offset", "");


    config_doc["mqtt"]["mqtt_server"] = preferences.getString("mqtt_server", "");
    config_doc["mqtt"]["mqtt_port"] = preferences.getString("mqtt_port", "");
    config_doc["mqtt"]["mqtt_topic"] = preferences.getString("mqtt_topic", "");
    config_doc["mqtt"]["mqtt_user"] = preferences.getString("mqtt_user", "");
    config_doc["mqtt"]["mqtt_password"] = preferences.getString("mqtt_passwd", "");

    config_doc["sunspec"]["modbus_dev"] = preferences.getString("modbus_dev", "");
    config_doc["sunspec"]["modbus_ip"] = preferences.getString("mqtt_server", "");
    config_doc["sunspec"]["modbus_port"] = preferences.getString("mqtt_port", "");
    config_doc["sunspec"]["query_period"] = preferences.getString("query_period", "");
   

    config_doc["led"]["led_gpio"] = preferences.getString("led_gpio", "");
    config_doc["led"]["led_gpio_i"] = preferences.getString("led_gpio_i", "");

    config_doc["shelly"]["shelly_port"] = preferences.getString("shelly_port", "2220");
   
    config_doc["json"]["power_path"] = preferences.getString("power_path", "");
    config_doc["json"]["pwr_export_path"] = preferences.getString("pwr_export_path", "");
    config_doc["json"]["power_l1_path"] = preferences.getString("power_l1_path", "");
    config_doc["json"]["power_l2_path"] = preferences.getString("power_l2_path", "");
    config_doc["json"]["power_l3_path"] = preferences.getString("power_l3_path", "");
    config_doc["json"]["energy_in_path"] = preferences.getString("energy_in_path", "");
    config_doc["json"]["energy_out_path"] = preferences.getString("energy_out_path", "");
    
    
    config_doc["http"]["server_ip"] = preferences.getString("mqtt_server", ""); 
    config_doc["http"]["query_period"] = preferences.getString("query_period", "");

    config_doc["sma"]["sma_id"] = preferences.getString("sma_id", "");
    
    
    config_doc["tibber"]["tibber_host"] = preferences.getString("tibber_host", "");
    config_doc["tibber"]["tibber_nodeid"] = preferences.getString("tibber_nodeid", "");
    config_doc["tibber"]["tibber_user"] = preferences.getString("tibber_user", "");
    config_doc["tibber"]["tibber_password"] = preferences.getString("tibber_password", "");
    config_doc["tibber"]["query_period"] = preferences.getString("query_period", "");

    config_doc["rct"]["rct_host"] = preferences.getString("rct_host", "");
    config_doc["rct"]["rct_port"] = preferences.getString("rct_port", "");
    config_doc["rct"]["query_period"] = preferences.getString("query_period", "");
  }
    preferences.end();


   /* make sure default values are available */
   config_doc["shelly"]["shelly_port"] = config_doc["shelly"]["shelly_port"].as<int>() ? config_doc["shelly"]["shelly_port"].as<int>() :  2220;
   config_doc["shelly"]["shelly_gen"] = config_doc["shelly"]["shelly_gen"].as<int>() ? config_doc["shelly"]["shelly_gen"].as<int>() : 2;
   config_doc["shelly"]["shelly_fw_id"] = config_doc["shelly"]["shelly_fw_id"] | "20241011-114455/1.4.4-g6d2a586";
   config_doc["shelly"]["shelly_name"] = config_doc["shelly"]["shelly_name"] | "shellypro3em-";

   config_doc["basic"]["phase_number"] = config_doc["basic"]["phase_number"].as<int>() ? config_doc["basic"]["phase_number"].as<int>(): 3;
   config_doc["basic"]["ntp_server"] = config_doc["basic"]["ntp_server"]| "de.pool.ntp.org";
   config_doc["basic"]["timezone"] = config_doc["basic"]["timezone"] | "CET-1CEST,M3.5.0/2,M10.5.0/3";
   config_doc["basic"]["input_type"] = config_doc["basic"]["input_type"] | "NONE";
   if (config_doc["basic"]["ntp_server"] == "") config_doc["basic"]["ntp_server"] = "de.pool.ntp.org";
   if (config_doc["basic"]["timezone"] == "") config_doc["basic"]["timezone"] = "CET-1CEST,M3.5.0/2,M10.5.0/3";
   if (config_doc["basic"]["input_type"]=="") config_doc["basic"]["input_type"] = "NONE";

   config_doc["sunspec"]["query_period"] = config_doc["sunspec"]["query_period"].as<int>() ? config_doc["sunspec"]["query_period"].as<int>() : 5000;
   config_doc["tibber"]["query_period"] = config_doc["tibber"]["query_period"].as<int>() ? config_doc["tibber"]["query_period"].as<int>() : 5000;
   config_doc["rct"]["query_period"]    = config_doc["rct"]["query_period"].as<int>() ? config_doc["rct"]["query_period"].as<int>() : 5000;
   config_doc["http"]["query_period"]   = config_doc["http"]["query_period"].as<int>() ? config_doc["http"]["query_period"].as<int>() : 5000;

 
}


