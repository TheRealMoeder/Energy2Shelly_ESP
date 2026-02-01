#include "../config/Configuration.h"
#include "../data/DataProcessor.h"

// Query a generic HTTP endpoint and parse JSON response
void queryHTTP() {
  // DEBUG_SERIAL.printf("DEBUG: queryHTTP() called, server=%s\n", mqtt_server);
  String serverAddr = String(mqtt_server);
  serverAddr.trim();
  if (serverAddr.length() == 0 || serverAddr == "http://" ||
      serverAddr == "https://") {
    DEBUG_SERIAL.println(
        "HTTP server not configured or invalid - skipping HTTP query");
    return;
  }
  if (!serverAddr.startsWith("http://") && !serverAddr.startsWith("https://")) {
    serverAddr = "http://" + serverAddr;
  }

  http.begin(wifi_client, serverAddr.c_str());
  http.GET();
  deserializeJson(globalJsonDoc, http.getStream());
  if (power_path[0] == '\0') {
    DEBUG_SERIAL.println("HTTP query: no JSONPath for power data provided");
  } else {
    setJsonPathPower(globalJsonDoc);
  }
  http.end();
  globalJsonDoc.clear();
}
