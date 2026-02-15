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

  // Query power data endpoint
  http.begin(wifi_client, serverAddr.c_str());
  int httpCode = http.GET();
  DEBUG_SERIAL.print("HTTP GET status code: ");
  DEBUG_SERIAL.println(httpCode);

  deserializeJson(globalJsonDoc, http.getStream());

  // Debug: print the received JSON
  String debugJson;
  serializeJson(globalJsonDoc, debugJson);
  DEBUG_SERIAL.print("Received JSON from ");
  DEBUG_SERIAL.print(serverAddr);
  DEBUG_SERIAL.print(": ");
  DEBUG_SERIAL.println(debugJson);

  if (power_path[0] == '\0') {
    DEBUG_SERIAL.println("HTTP query: no JSONPath for power data provided");
  } else {
    DEBUG_SERIAL.print("Calling setJsonPathPower() with power_path: ");
    DEBUG_SERIAL.println(power_path);
    setJsonPathPower(globalJsonDoc);
  }
  http.end();
  globalJsonDoc.clear();

  // If this is a Shelly device, also query energy data from EMData.GetStatus
  if (serverAddr.indexOf("/EM.GetStatus") > 0) {
    String energyUrl = serverAddr;
    energyUrl.replace("/EM.GetStatus", "/EMData.GetStatus");

    DEBUG_SERIAL.print("Querying energy data from: ");
    DEBUG_SERIAL.println(energyUrl);

    http.begin(wifi_client, energyUrl.c_str());
    int httpCode = http.GET();
    DEBUG_SERIAL.print("Energy endpoint HTTP status: ");
    DEBUG_SERIAL.println(httpCode);

    if (httpCode == 200) {
      deserializeJson(globalJsonDoc, http.getStream());

      // Debug: print the received energy JSON
      String debugEnergyJson;
      serializeJson(globalJsonDoc, debugEnergyJson);
      DEBUG_SERIAL.print("Received energy JSON: ");
      DEBUG_SERIAL.println(debugEnergyJson);

      // Extract energy data if present in Shelly format
      if (globalJsonDoc["a_total_act_energy"].is<JsonVariant>()) {
        double energyA = globalJsonDoc["a_total_act_energy"].as<double>();
        double energyB = globalJsonDoc["b_total_act_energy"].as<double>();
        double energyC = globalJsonDoc["c_total_act_energy"].as<double>();
        double retEnergyA = globalJsonDoc["a_total_act_ret_energy"].as<double>();
        double retEnergyB = globalJsonDoc["b_total_act_ret_energy"].as<double>();
        double retEnergyC = globalJsonDoc["c_total_act_ret_energy"].as<double>();

        setPhaseEnergyData(0, energyA, retEnergyA);
        setPhaseEnergyData(1, energyB, retEnergyB);
        setPhaseEnergyData(2, energyC, retEnergyC);

        DEBUG_SERIAL.print("Energy Phase A: ");
        DEBUG_SERIAL.print(energyA);
        DEBUG_SERIAL.print(" Wh, Return: ");
        DEBUG_SERIAL.println(retEnergyA);
      }
    } else {
      DEBUG_SERIAL.print("Energy query failed with code: ");
      DEBUG_SERIAL.println(httpCode);
    }
    http.end();
    globalJsonDoc.clear();
  }
}
