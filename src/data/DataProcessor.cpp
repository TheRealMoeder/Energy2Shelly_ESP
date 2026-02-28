#include "DataProcessor.h"
#include "DataStructures.h"
#include "../config/Configuration.h"

// Global data structures
PowerData PhasePower[3];
EnergyData PhaseEnergy[3];
String serJsonResponse;

// Validate IP address
bool isValidIPAddress(const char *ipString) {
  IPAddress ip;
  return ip.fromString(ipString);
}

// Round to 2 decimal places with Marstek bug fix
double round2(double value) {
  int ivalue = (int)(value * 100.0 + (value > 0.0 ? 0.5 : -0.5));

  // fix Marstek bug: make sure to have decimal numbers
  if (forcePwrDecimals && (ivalue % 100 == 0))
    ivalue++;

  return ivalue / 100.0;
}

// Helper function to set individual phase data
void setPhaseData(int phaseIndex, double current, double voltage, double power,
                  double apparentPower, double powerFactor, int frequency) {
  DEBUG_SERIAL.print("Setting Phase ");
  DEBUG_SERIAL.print(phaseIndex);
  DEBUG_SERIAL.print(" - Raw values: I=");
  DEBUG_SERIAL.print(current);
  DEBUG_SERIAL.print(" V=");
  DEBUG_SERIAL.print(voltage);
  DEBUG_SERIAL.print(" P=");
  DEBUG_SERIAL.println(power);

  PhasePower[phaseIndex].current = round2(current);
  PhasePower[phaseIndex].voltage = round2(voltage);
  PhasePower[phaseIndex].power = round2(power);
  PhasePower[phaseIndex].apparentPower = round2(apparentPower);
  PhasePower[phaseIndex].powerFactor = round2(powerFactor);
  PhasePower[phaseIndex].frequency = frequency;

  DEBUG_SERIAL.print("Phase ");
  DEBUG_SERIAL.print(phaseIndex);
  DEBUG_SERIAL.print(" - After round2: I=");
  DEBUG_SERIAL.print(PhasePower[phaseIndex].current);
  DEBUG_SERIAL.print(" V=");
  DEBUG_SERIAL.print(PhasePower[phaseIndex].voltage);
  DEBUG_SERIAL.print(" P=");
  DEBUG_SERIAL.println(PhasePower[phaseIndex].power);
}

// Helper function to distribute single power value across three phases
void setPhaseDataSingle(int phaseIndex, double power) {
  PhasePower[phaseIndex].power = round2(power);
  PhasePower[phaseIndex].voltage = round2(defaultVoltage);
  PhasePower[phaseIndex].current =
      round2(PhasePower[phaseIndex].power / PhasePower[phaseIndex].voltage);
  PhasePower[phaseIndex].apparentPower = round2(PhasePower[phaseIndex].power);
  PhasePower[phaseIndex].powerFactor = round2(defaultPowerFactor);
  PhasePower[phaseIndex].frequency = defaultFrequency;
}

// Helper function to set individual phase energy data
void setPhaseEnergyData(int phaseIndex, double consumption, double gridfeedin) {
  PhaseEnergy[phaseIndex].consumption = round2(consumption);
  PhaseEnergy[phaseIndex].gridfeedin = round2(gridfeedin);
}

// Set power data from a single power value - distribute to all phases
void setPowerData(double totalPower) {
  double powerPerPhase = totalPower * 0.3333;
  for (int i = 0; i <= 2; i++) {
    setPhaseDataSingle(i, powerPerPhase);
  }
  DEBUG_SERIAL.print("Current total power: ");
  DEBUG_SERIAL.println(totalPower);
}

// Set power data from three individual phase values
void setPowerData(double phase1Power, double phase2Power, double phase3Power) {
  for (int i = 0; i <= 2; i++) {
    double power = (i == 0)   ? phase1Power
                   : (i == 1) ? phase2Power
                              : phase3Power;
    setPhaseDataSingle(i, power);
  }
  DEBUG_SERIAL.print("Current power L1: ");
  DEBUG_SERIAL.print(phase1Power);
  DEBUG_SERIAL.print(" - L2: ");
  DEBUG_SERIAL.print(phase2Power);
  DEBUG_SERIAL.print(" - L3: ");
  DEBUG_SERIAL.println(phase3Power);
}

// Set energy data - distribute to all phases
void setEnergyData(double totalEnergyGridSupply, double totalEnergyGridFeedIn) {
  double consumptionPerPhase = totalEnergyGridSupply * 0.3333;
  double gridFeedinPerPhase = totalEnergyGridFeedIn * 0.3333;
  for (int i = 0; i <= 2; i++) {
    setPhaseEnergyData(i, consumptionPerPhase, gridFeedinPerPhase);
  }
  DEBUG_SERIAL.print("Total consumption: ");
  DEBUG_SERIAL.print(totalEnergyGridSupply);
  DEBUG_SERIAL.print(" - Total Grid Feed-In: ");
  DEBUG_SERIAL.println(totalEnergyGridFeedIn);
}

// Parse JSON with multiple power modes: direct Shelly format, TRIPHASE, net power, or single power
void setJsonPathPower(JsonDocument json) {
  // If the incoming JSON already uses Shelly 3EM field names, parse directly
  if (json["a_current"].is<JsonVariant>() ||
      json["a_act_power"].is<JsonVariant>()) {
    DEBUG_SERIAL.println("Auto-detected Shelly 3EM format - parsing direct fields");
    DEBUG_SERIAL.print("Phase A - Current: ");
    DEBUG_SERIAL.print(json["a_current"].as<double>());
    DEBUG_SERIAL.print(", Voltage: ");
    DEBUG_SERIAL.print(json["a_voltage"].as<double>());
    DEBUG_SERIAL.print(", Power: ");
    DEBUG_SERIAL.println(json["a_act_power"].as<double>());

    setPhaseData(
        0, json["a_current"].as<double>(), json["a_voltage"].as<double>(),
        json["a_act_power"].as<double>(), json["a_aprt_power"].as<double>(),
        json["a_pf"].as<double>(), json["a_freq"].as<int>());
    setPhaseData(
        1, json["b_current"].as<double>(), json["b_voltage"].as<double>(),
        json["b_act_power"].as<double>(), json["b_aprt_power"].as<double>(),
        json["b_pf"].as<double>(), json["b_freq"].as<int>());
    setPhaseData(
        2, json["c_current"].as<double>(), json["c_voltage"].as<double>(),
        json["c_act_power"].as<double>(), json["c_aprt_power"].as<double>(),
        json["c_pf"].as<double>(), json["c_freq"].as<int>());

    DEBUG_SERIAL.println("All three phases parsed successfully");

    // Optionally use total fields if present
    //if (json["total_act_power"].is<JsonVariant>()) {
    //  double total = json["total_act_power"].as<double>();
      // distribute if individual phases missing or for logging
      // DEBUG_SERIAL.print("Total power from payload: ");
      // DEBUG_SERIAL.println(total);
    //}
    return;
  }

  DEBUG_SERIAL.println("Shelly format NOT detected, checking other parsing modes");
  DEBUG_SERIAL.print("power_path value: '");
  DEBUG_SERIAL.print(power_path);
  DEBUG_SERIAL.println("'");

  if (strcmp(power_path, "TRIPHASE") == 0) {
    DEBUG_SERIAL.println("Using TRIPHASE mode");
    double power1 = resolveJsonPath(json, power_l1_path).as<double>();
    double power2 = resolveJsonPath(json, power_l2_path).as<double>();
    double power3 = resolveJsonPath(json, power_l3_path).as<double>();

    // Apply negation if configured
    if (negate_power_l1_path) power1 = -power1;
    if (negate_power_l2_path) power2 = -power2;
    if (negate_power_l3_path) power3 = -power3;

    DEBUG_SERIAL.print("TRIPHASE powers: L1=");
    DEBUG_SERIAL.print(power1);
    DEBUG_SERIAL.print(", L2=");
    DEBUG_SERIAL.print(power2);
    DEBUG_SERIAL.print(", L3=");
    DEBUG_SERIAL.println(power3);
    setPowerData(power1, power2, power3);
  } else {
    // Check if BOTH paths (Import = powerPath, Export = pwrExportPath) are
    // defined
    if ((power_path[0] != '\0') && (pwr_export_path[0] != '\0')) {
      DEBUG_SERIAL.println("Resolving net power (import - export)");
      double importPower = resolveJsonPath(json, power_path).as<double>();
      double exportPower = resolveJsonPath(json, pwr_export_path).as<double>();

      // Apply negation if configured
      if (negate_power_path) importPower = -importPower;
      if (negate_pwr_export_path) exportPower = -exportPower;

      double netPower = importPower - exportPower;
      DEBUG_SERIAL.print("Import: ");
      DEBUG_SERIAL.print(importPower);
      DEBUG_SERIAL.print(", Export: ");
      DEBUG_SERIAL.print(exportPower);
      DEBUG_SERIAL.print(", Net: ");
      DEBUG_SERIAL.println(netPower);
      setPowerData(netPower);
    }
    // (FALLBACK): Only the normal powerPath (import path) is defined (old
    // logic)
    else if (power_path[0] != '\0') {
      DEBUG_SERIAL.println("Resolving monophase (single path only)");
      double power = resolveJsonPath(json, power_path).as<double>();

      // Apply negation if configured
      if (negate_power_path) power = -power;

      DEBUG_SERIAL.print("Resolved power: ");
      DEBUG_SERIAL.println(power);
      setPowerData(power);
    } else {
      DEBUG_SERIAL.println("WARNING: No valid power parsing mode selected!");
    }
  }
  if ((energy_in_path[0] != '\0') && (energy_out_path[0] != '\0')) {
    DEBUG_SERIAL.print("Parsing energy data - energyInPath: ");
    DEBUG_SERIAL.print(energy_in_path);
    DEBUG_SERIAL.print(", energyOutPath: ");
    DEBUG_SERIAL.println(energy_out_path);

    double energyIn = resolveJsonPath(json, energy_in_path).as<double>();
    double energyOut = resolveJsonPath(json, energy_out_path).as<double>();

    // Apply negation if configured
    if (negate_energy_in_path) energyIn = -energyIn;
    if (negate_energy_out_path) energyOut = -energyOut;

    DEBUG_SERIAL.print("Resolved values - energyIn: ");
    DEBUG_SERIAL.print(energyIn);
    DEBUG_SERIAL.print(", energyOut: ");
    DEBUG_SERIAL.println(energyOut);

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

void parseShellyString(const String &s) { parseShellyString(s.c_str()); }
