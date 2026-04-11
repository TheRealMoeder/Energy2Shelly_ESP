#include "DataStructures.h"
#include "DataProcessing.h"
#include "../config/Configuration.h"

// Global data structures
PowerData PhasePower[3];
EnergyData PhaseEnergy[3];
String serJsonResponse;

double round2(double value) {
  int ivalue = (int) round(value * 100.0);

  // fix Marstek bug: make sure to have decimal numbers
  if(forcePwrDecimals && (ivalue % 100 == 0)) ivalue++;

  return ivalue / 100.0;
}

bool isValidIPAddress(const char* ipString) {
  IPAddress ip;
  return ip.fromString(ipString);
}

JsonVariant resolveJsonPath(JsonVariant variant, const char *path) {
  for (size_t n = 0; path[n]; n++) {
    // Not a full array support, but works for Shelly 3EM emeters array!
    if (path[n] == '[') {
      variant = variant[JsonString(path, n)][atoi(&path[n+1])];
      path += n + 4;
      n = 0;
    }
    if (path[n] == '.') {
      variant = variant[JsonString(path, n)];
      path += n + 1;
      n = 0;
    }
  }
  return variant[path];
}

void setPowerData(double totalPower) {
  switch(phase_number[0]) {
    case '1': // monophase
      PhasePower[0].power = round2(totalPower);
      PhasePower[1].power = 0.0;
      PhasePower[2].power = 0.0;
      break;
    case '3': // triphase
    default:
      PhasePower[0].power = round2(totalPower * 0.3333);
      PhasePower[1].power = round2(totalPower * 0.3333);
      PhasePower[2].power = round2(totalPower * 0.3333);
      break;
  }
  for (int i = 0; i <= 2; i++) {
    PhasePower[i].voltage = round2(defaultVoltage);
    PhasePower[i].current = round2(PhasePower[i].power / PhasePower[i].voltage);
    PhasePower[i].apparentPower = round2(PhasePower[i].power);
    PhasePower[i].powerFactor = round2(defaultPowerFactor);
    PhasePower[i].frequency = defaultFrequency;
  }
  DEBUG_SERIAL.print("Current total power: ");
  DEBUG_SERIAL.println(totalPower);
}

void setPowerData(double phase1Power, double phase2Power, double phase3Power) {
  switch(phase_number[0]) {
    case '1': // monophase
      PhasePower[0].power = round2(phase1Power) + round2(phase2Power) + round2(phase3Power);
      PhasePower[1].power = 0.0;
      PhasePower[2].power = 0.0;
      break;
    case '3': // triphase
    default:
      PhasePower[0].power = round2(phase1Power);
      PhasePower[1].power = round2(phase2Power);
      PhasePower[2].power = round2(phase3Power);
      break;
  }
  for (int i = 0; i <= 2; i++) {
    PhasePower[i].voltage = round2(defaultVoltage);
    PhasePower[i].current = round2(PhasePower[i].power / PhasePower[i].voltage);
    PhasePower[i].apparentPower = round2(PhasePower[i].power);
    PhasePower[i].powerFactor = round2(defaultPowerFactor);
    PhasePower[i].frequency = defaultFrequency;
  }
  DEBUG_SERIAL.print("Current power L1: ");
  DEBUG_SERIAL.print(phase1Power);
  DEBUG_SERIAL.print(" - L2: ");
  DEBUG_SERIAL.print(phase2Power);
  DEBUG_SERIAL.print(" - L3: ");
  DEBUG_SERIAL.println(phase3Power);
}

void setEnergyData(double totalEnergyGridSupply, double totalEnergyGridFeedIn) {
  switch (phase_number[0]) {
  case '1': // monophase
    for (int i = 0; i <= 2; i++) {
      PhaseEnergy[i].consumption = (i == 0) ? round2(totalEnergyGridSupply) : 0.0;
      PhaseEnergy[i].gridfeedin = (i == 0) ? round2(totalEnergyGridFeedIn) : 0.0;
    }
    break;
  case '3': // triphase
  default:
    for (int i = 0; i <= 2; i++){
      PhaseEnergy[i].consumption = round2(totalEnergyGridSupply * 0.3333);
      PhaseEnergy[i].gridfeedin = round2(totalEnergyGridFeedIn * 0.3333);
    }
    break;
  }
  DEBUG_SERIAL.print("Total Consumption (Grid Supply): ");
  DEBUG_SERIAL.print(totalEnergyGridSupply);
  DEBUG_SERIAL.print(" - Total Production (Grid Feed-In): ");
  DEBUG_SERIAL.println(totalEnergyGridFeedIn);
}

void setJsonPathPower(JsonDocument json) {
  // If the incoming JSON already uses Shelly 3EM field names, parse directly
  if (json["a_current"].is<JsonVariant>() || json["a_act_power"].is<JsonVariant>()) {
    DEBUG_SERIAL.println("Parsing direct Shelly 3EM payload");
    PhasePower[0].current = round2((double)json["a_current"].as<double>());
    PhasePower[0].voltage = round2((double)json["a_voltage"].as<double>());
    PhasePower[0].power = round2((double)json["a_act_power"].as<double>());
    PhasePower[0].apparentPower = round2((double)json["a_aprt_power"].as<double>());
    PhasePower[0].powerFactor = round2((double)json["a_pf"].as<double>());
    PhasePower[0].frequency = json["a_freq"].as<int>();

    PhasePower[1].current = round2((double)json["b_current"].as<double>());
    PhasePower[1].voltage = round2((double)json["b_voltage"].as<double>());
    PhasePower[1].power = round2((double)json["b_act_power"].as<double>());
    PhasePower[1].apparentPower = round2((double)json["b_aprt_power"].as<double>());
    PhasePower[1].powerFactor = round2((double)json["b_pf"].as<double>());
    PhasePower[1].frequency = json["b_freq"].as<int>();

    PhasePower[2].current = round2((double)json["c_current"].as<double>());
    PhasePower[2].voltage = round2((double)json["c_voltage"].as<double>());
    PhasePower[2].power = round2((double)json["c_act_power"].as<double>());
    PhasePower[2].apparentPower = round2((double)json["c_aprt_power"].as<double>());
    PhasePower[2].powerFactor = round2((double)json["c_pf"].as<double>());
    PhasePower[2].frequency = json["c_freq"].as<int>();

    // Optionally use total fields if present
    if (json["total_act_power"].is<JsonVariant>()) {
      double total = json["total_act_power"].as<double>();
      // distribute if individual phases missing or for logging
      DEBUG_SERIAL.print("Total power from payload: ");
      DEBUG_SERIAL.println(total);
    }
    return;
  }
  if (strcmp(power_path, "TRIPHASE") == 0) {
    DEBUG_SERIAL.println("resolving triphase");
    double power1 = resolveJsonPath(json, power_l1_path);
    double power2 = resolveJsonPath(json, power_l2_path);
    double power3 = resolveJsonPath(json, power_l3_path);
    DEBUG_SERIAL.println(power1);
    setPowerData(power1, power2, power3);
  } else {
    // Check if BOTH paths (Import = power_path, Export = pwr_export_path) are defined
    if ((strcmp(power_path, "") != 0) && (strcmp(pwr_export_path, "") != 0)) {
      DEBUG_SERIAL.println("Resolving net power (import - export)");
      double importPower = resolveJsonPath(json, power_path).as<double>();
      double exportPower = resolveJsonPath(json, pwr_export_path).as<double>();
      double netPower = importPower - exportPower;
      setPowerData(netPower);
    }
    // (FALLBACK): Only the normal power_path (import path) is defined (old logic)
    else if (strcmp(power_path, "") != 0) {
      DEBUG_SERIAL.println("Resolving monophase (single path only)");
      double power = resolveJsonPath(json, power_path).as<double>();
      setPowerData(power);
    }
  }
  if ((strcmp(energy_in_path, "") != 0) && (strcmp(energy_out_path, "") != 0)) {
    double energyIn = resolveJsonPath(json, energy_in_path);
    double energyOut = resolveJsonPath(json, energy_out_path);
    setEnergyData(energyIn, energyOut);
  }
}

// Helper: parse a raw Shelly JSON string and forward to setJsonPathPower
void parseShellyString(const char *jsonStr) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) {
    DEBUG_SERIAL.print("deserializeJson failed: ");
    DEBUG_SERIAL.println(err.c_str());
    return;
  }
  setJsonPathPower(doc);
}

void parseShellyString(const String &s) {
  parseShellyString(s.c_str());
}