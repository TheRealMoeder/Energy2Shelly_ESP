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
  DEBUG_SERIAL.printf("DEBUG: mqtt_reconnect() called, server=%s, mqtt_configured=%d\n",
      mqtt_server, mqtt_configured);

  // Prevent reconnection if MQTT not properly configured
  if (!mqtt_configured || mqtt_server[0] == '\0') {
    DEBUG_SERIAL.println(F("DEBUG: mqtt_reconnect() aborting - MQTT not configured or empty server"));
    return;
  }

  // FIX: Non-blocking reconnect throttle - avoids delay(5000) blocking the CPU.
  // Only attempt reconnect once every 5 seconds.
  static unsigned long lastReconnectAttempt = 0;
  unsigned long now = millis();
  if (now - lastReconnectAttempt < 5000) {
    return;
  }
  lastReconnectAttempt = now;

  DEBUG_SERIAL.print(F("Attempting MQTT connection..."));
  if (mqtt_client.connect(shelly_name, mqtt_user, mqtt_passwd)) {
    DEBUG_SERIAL.println(F("connected"));
    mqtt_client.subscribe(mqtt_topic);
  } else {
    DEBUG_SERIAL.print(F("failed, rc="));
    DEBUG_SERIAL.print(mqtt_client.state());
    DEBUG_SERIAL.println(F(" try again in 5 seconds"));
  }
}
