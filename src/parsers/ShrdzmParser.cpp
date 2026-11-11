#include "../config/Configuration.h"
#include "../data/DataProcessing.h"

void parseSHRDZM() {
  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  char buffer[1024];
  int rSize = Udp.read(buffer, sizeof(buffer) - 1);
  if (rSize <= 0) {
    return;
  }
  buffer[rSize] = '\0';

  JsonDocument json;
  DeserializationError error = deserializeJson(json, buffer);
  if (error) {
    DEBUG_SERIAL.print(F("SHRDZM deserializeJson() error: "));
    DEBUG_SERIAL.println(error.f_str());
    return;
  }

  JsonVariant data = json["data"];
  if (data.isNull()) {
    return;
  }

  JsonVariant pwr = data["16.7.0"];
  if (!pwr.isNull()) {
    setPowerData(pwr.as<double>());
  }

  JsonVariant eIn = data["1.8.0"];
  JsonVariant eOut = data["2.8.0"];
  if (!eIn.isNull() && !eOut.isNull()) {
    double energyIn = 0.001 * eIn.as<double>();
    double energyOut = 0.001 * eOut.as<double>();
    setEnergyData(energyIn, energyOut);
  }
}
