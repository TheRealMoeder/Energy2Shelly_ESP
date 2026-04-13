#include "../config/Configuration.h"
#include "../data/DataProcessing.h"

// Query a generic HTTP endpoint and parse JSON response
void queryHTTP() {
  JsonDocument json;
  DEBUG_SERIAL.println(F("Querying HTTP source"));
  http.begin(wifi_client, mqtt_server);
  http.GET();
  deserializeJson(json, http.getStream());
  if (strcmp(power_path, "") == 0) {
    DEBUG_SERIAL.println(F("HTTP query: no JSONPath for power data provided"));
  } else {
    setJsonPathPower(json);
  }
  http.end();
}

