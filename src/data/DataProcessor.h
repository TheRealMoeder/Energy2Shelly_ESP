#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>

// Utility functions
double round2(double value);
bool isValidIPAddress(const char *ipString);

// Phase data setters - individual parameter setting
void setPhaseData(int phaseIndex, double current, double voltage, double power,
                  double apparentPower, double powerFactor, int frequency);

// Phase data setters - distribute single power value across three phases
void setPhaseDataSingle(int phaseIndex, double power);

// Energy data setter
void setPhaseEnergyData(int phaseIndex, double consumption, double gridfeedin);

// High-level power setters - distribute to all three phases
void setPowerData(double totalPower);
void setPowerData(double phase1Power, double phase2Power, double phase3Power);

// High-level energy setter - distribute to all three phases
void setEnergyData(double totalEnergyGridSupply, double totalEnergyGridFeedIn);

// JSON path processing
void setJsonPathPower(JsonDocument json);
void parseShellyString(const char *jsonStr);
void parseShellyString(const String &s);

// JSON utilities (declared in JsonUtils.cpp)
JsonVariant resolveJsonPath(JsonVariant variant, const char *path);

#endif // DATA_PROCESSOR_H
