#include "../config/Configuration.h"
#include "../data/DataProcessing.h"

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  if (payload == nullptr || length == 0) {
    return;
  }

  JsonDocument json;
  
  // Try to parse as JSON first
  DeserializationError error = deserializeJson(json, (const char*)payload, length);
  
  if (error) {
    // Copy up to 31 bytes to a local stack buffer instead of writing past payload[length]
    char numBuf[32];
    size_t copyLen = (length < sizeof(numBuf) - 1) ? length : (sizeof(numBuf) - 1);
    memcpy(numBuf, payload, copyLen);
    numBuf[copyLen] = '\0';

    char* rawStr = numBuf;
    while (*rawStr == ' ' || *rawStr == '\t' || *rawStr == '\n' || *rawStr == '\r') {
      rawStr++;
    }

    // Check if payload starts with a valid number character
    if (isdigit((unsigned char)rawStr[0]) || rawStr[0] == '-' || rawStr[0] == '.') {
      float rawValue = atof(rawStr);
      setPowerData(rawValue);
    } else {
      DEBUG_SERIAL.println(F("Error parsing MQTT payload"));
    }
  } else {
    // Successfully parsed as JSON
    setJsonPathPower(json);
  }
}


void mqtt_reconnect() {
  static unsigned long lastReconnectAttempt = 0;
  static bool hasAttempted = false;
  const unsigned long now = millis();
  if (hasAttempted && now - lastReconnectAttempt < 5000) {
    return;
  }
  hasAttempted = true;
  lastReconnectAttempt = now;

  DEBUG_SERIAL.print(F("Attempting MQTT connection..."));

  const char* user = mqtt_user;
  const char* pass = mqtt_passwd;
  if (user == nullptr || user[0] == '\0') {
    user = nullptr;
    pass = nullptr;
  }

  if (mqtt_client->connect(shelly_name.c_str(), user, pass)) {
    DEBUG_SERIAL.println(F("connected"));
    const char* topic = mqtt_topic;
    if (topic != nullptr && topic[0] != '\0') {
      mqtt_client->subscribe(topic);
    }
  } else {
    DEBUG_SERIAL.print(F("failed, rc="));
    DEBUG_SERIAL.print(mqtt_client->state());
    DEBUG_SERIAL.println(F(" try again in 5 seconds"));
  }
}