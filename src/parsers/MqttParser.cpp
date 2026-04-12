#include "../config/Configuration.h"
#include "../data/DataProcessing.h"

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  JsonDocument json;
  
  // Try to parse as JSON first
  DeserializationError error = deserializeJson(json, payload, length);
  
  if (error) {
    // If JSON parsing fails, try to parse as a raw number
    String payloadStr = String((char*)payload);
    payloadStr.trim();
    
    // Check if payload is a valid number
    if (payloadStr.length() > 0 && (isdigit(payloadStr[0]) || payloadStr[0] == '-' || payloadStr[0] == '.')) {
      float rawValue = payloadStr.toFloat();
      setPowerData(rawValue);
    } else {
      DEBUG_SERIAL.print("Error parsing MQTT payload");
    }
  } else {
    // Successfully parsed as JSON
    setJsonPathPower(json);
  }
}

void mqtt_reconnect() {
  DEBUG_SERIAL.print("Attempting MQTT connection...");
  if (mqtt_client.connect(shelly_name, String(mqtt_user).c_str(), String(mqtt_passwd).c_str())) {
    DEBUG_SERIAL.println("connected");
    mqtt_client.subscribe(mqtt_topic);
  } else {
    DEBUG_SERIAL.print("failed, rc=");
    DEBUG_SERIAL.print(mqtt_client.state());
    DEBUG_SERIAL.println(" try again in 5 seconds");
    delay(5000);
  }
}