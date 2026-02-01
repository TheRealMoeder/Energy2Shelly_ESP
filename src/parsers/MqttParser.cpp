#include "../config/Configuration.h"
#include "../data/DataProcessor.h"

// MQTT message callback - deserializes payload and processes JSON
void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  JsonDocument json;
  deserializeJson(json, payload, length);
  setJsonPathPower(json);
}

// Attempt to connect/reconnect to MQTT broker
void mqtt_reconnect() {
  DEBUG_SERIAL.printf(
      "DEBUG: mqtt_reconnect() called, server=%s, mqtt_configured=%d\n",
      mqtt_server, mqtt_configured);

  // Prevent reconnection if MQTT not properly configured
  if (!mqtt_configured || mqtt_server[0] == '\0') {
    DEBUG_SERIAL.println("DEBUG: mqtt_reconnect() aborting - MQTT not "
                         "configured or empty server");
    return;
  }

  DEBUG_SERIAL.print("Attempting MQTT connection...");
  if (mqtt_client.connect(shelly_name, mqtt_user, mqtt_passwd)) {
    DEBUG_SERIAL.println("connected");
    mqtt_client.subscribe(mqtt_topic);
  } else {
    DEBUG_SERIAL.print("failed, rc=");
    DEBUG_SERIAL.print(mqtt_client.state());
    DEBUG_SERIAL.println(" try again in 5 seconds");
    delay(5000);
  }
}
