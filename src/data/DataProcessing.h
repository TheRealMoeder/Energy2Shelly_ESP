#ifndef DATA_PROCESSING_H
#define DATA_PROCESSING_H

#include <Arduino.h>
#include <ArduinoJson.h>

// Utility functions
double round2(double value);
bool isValidIPAddress(const char *ipString);
JsonVariant resolveJsonPath(JsonVariant variant, const char *path);

// Power setters - distribute to all three phases
void setPowerData(double totalPower);
void setPowerData(double phase1Power, double phase2Power, double phase3Power);

// Grid setter - overwrite the defaulted voltage/current/apparent power/frequency with metered values.
// Call after setPowerData(), pass NAN for values the source did not provide.
void setGridData(const double voltage[3], const double current[3], const double apparentPower[3], double frequency);

// Energy setter - distribute to all three phases
void setEnergyData(double totalEnergyGridSupply, double totalEnergyGridFeedIn);

// JSON path processing
void setJsonPathPower(JsonDocument json);
void parseShellyString(const char *jsonStr);
void parseShellyString(const String &s);

#endif // DATA_PROCESSING_H
