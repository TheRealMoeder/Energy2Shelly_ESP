#include "RpcHandlers.h"
#include "../config/Configuration.h"
#include "../data/DataStructures.h"
#include "../data/DataProcessor.h"

// ============================================================================
// LED CONTROL
// ============================================================================

void blinkled(int duration) {
  if (led > 0) {
    if (led_i) {
      digitalWrite(led, HIGH);
    } else {
      digitalWrite(led, LOW);
    }
    ledOffTime = millis() + duration;
  }
}

void handleblinkled() {
  if (led > 0) {
    if (ledOffTime > 0 && millis() > ledOffTime) {
      if (led_i) {
        digitalWrite(led, LOW);
      } else {
        digitalWrite(led, HIGH);
      }
      ledOffTime = 0;
    }
  }
}

// ============================================================================
// RPC RESPONSE GENERATORS
// ============================================================================

// Wrap a response in RPC envelope with id, src, dst
void rpcWrapper() {
  JsonDocument jsonResponse;
  JsonDocument doc;
  deserializeJson(doc, serJsonResponse);
  jsonResponse["id"] = rpcId;
  jsonResponse["src"] = shelly_name;
  if (strcmp(rpcUser, "EMPTY") != 0) {
    jsonResponse["dst"] = rpcUser;
  }
  jsonResponse["result"] = doc;
  serializeJson(jsonResponse, serJsonResponse);
}

// Return device information (Shelly Pro 3EM emulation)
void GetDeviceInfo() {
  JsonDocument jsonResponse;
  jsonResponse["name"] = shelly_name;
  jsonResponse["id"] = shelly_name;
  jsonResponse["mac"] = shelly_mac;
  jsonResponse["slot"] = 1;
  jsonResponse["model"] = "SPEM-003CEBEU";
  jsonResponse["gen"] = shelly_gen;
  jsonResponse["fw_id"] = shelly_fw_id;
  jsonResponse["ver"] = "1.4.4";
  jsonResponse["app"] = "Pro3EM";
  jsonResponse["auth_en"] = false;
  jsonResponse["profile"] = "triphase";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// Return current power data for all phases
void EMGetStatus() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = 0;
  jsonResponse["a_current"] = PhasePower[0].current;
  jsonResponse["a_voltage"] = PhasePower[0].voltage;
  jsonResponse["a_act_power"] = PhasePower[0].power;
  jsonResponse["a_aprt_power"] = PhasePower[0].apparentPower;
  jsonResponse["a_pf"] = PhasePower[0].powerFactor;
  jsonResponse["a_freq"] = PhasePower[0].frequency;
  jsonResponse["b_current"] = PhasePower[1].current;
  jsonResponse["b_voltage"] = PhasePower[1].voltage;
  jsonResponse["b_act_power"] = PhasePower[1].power;
  jsonResponse["b_aprt_power"] = PhasePower[1].apparentPower;
  jsonResponse["b_pf"] = PhasePower[1].powerFactor;
  jsonResponse["b_freq"] = PhasePower[1].frequency;
  jsonResponse["c_current"] = PhasePower[2].current;
  jsonResponse["c_voltage"] = PhasePower[2].voltage;
  jsonResponse["c_act_power"] = PhasePower[2].power;
  jsonResponse["c_aprt_power"] = PhasePower[2].apparentPower;
  jsonResponse["c_pf"] = PhasePower[2].powerFactor;
  jsonResponse["c_freq"] = PhasePower[2].frequency;
  jsonResponse["total_current"] =
      round2((PhasePower[0].power + PhasePower[1].power + PhasePower[2].power) /
             ((float)defaultVoltage));
  jsonResponse["total_act_power"] =
      PhasePower[0].power + PhasePower[1].power + PhasePower[2].power;
  jsonResponse["total_aprt_power"] = PhasePower[0].apparentPower +
                                     PhasePower[1].apparentPower +
                                     PhasePower[2].apparentPower;
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// Return energy consumption and feed-in data
void EMDataGetStatus() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = 0;
  jsonResponse["a_total_act_energy"] = PhaseEnergy[0].consumption;
  jsonResponse["a_total_act_ret_energy"] = PhaseEnergy[0].gridfeedin;
  jsonResponse["b_total_act_energy"] = PhaseEnergy[1].consumption;
  jsonResponse["b_total_act_ret_energy"] = PhaseEnergy[1].gridfeedin;
  jsonResponse["c_total_act_energy"] = PhaseEnergy[2].consumption;
  jsonResponse["c_total_act_ret_energy"] = PhaseEnergy[2].gridfeedin;
  jsonResponse["total_act"] = PhaseEnergy[0].consumption +
                              PhaseEnergy[1].consumption +
                              PhaseEnergy[2].consumption;
  jsonResponse["total_act_ret"] = PhaseEnergy[0].gridfeedin +
                                  PhaseEnergy[1].gridfeedin +
                                  PhaseEnergy[2].gridfeedin;
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

// Return meter configuration
void EMGetConfig() {
  JsonDocument jsonResponse;
  jsonResponse["id"] = 0;
  jsonResponse["name"] = nullptr;
  jsonResponse["blink_mode_selector"] = "active_energy";
  jsonResponse["phase_selector"] = "a";
  jsonResponse["monitor_phase_sequence"] = true;
  jsonResponse["ct_type"] = "120A";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}
