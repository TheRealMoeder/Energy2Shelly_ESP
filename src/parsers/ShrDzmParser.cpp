#include "../config/Configuration.h"
#include "../data/DataProcessor.h"

// Parse SHRDZM smart meter UDP packets (JSON format)
void parseSHRDZM() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int rSize = Udp.read(networkBuffer, 1024);
    networkBuffer[rSize] = 0;
    deserializeJson(globalJsonDoc, networkBuffer);
    if (globalJsonDoc["data"]["16.7.0"].is<JsonVariant>()) {
      double power = globalJsonDoc["data"]["16.7.0"];
      setPowerData(power);
    }
    if (globalJsonDoc["data"]["1.8.0"].is<JsonVariant>() &&
        globalJsonDoc["data"]["2.8.0"].is<JsonVariant>()) {
      double energyIn = 0.001 * globalJsonDoc["data"]["1.8.0"].as<double>();
      double energyOut = 0.001 * globalJsonDoc["data"]["2.8.0"].as<double>();
      setEnergyData(energyIn, energyOut);
    }
    globalJsonDoc.clear();
  }
}
