#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <Arduino.h>

// Core data structures for power and energy measurements
struct PowerData {
  double current;
  double voltage;
  double power;
  double apparentPower;
  double powerFactor;
  double frequency;
};

struct EnergyData {
  double gridfeedin;
  double consumption;
};

// Global power data for three phases
extern PowerData PhasePower[3];

// Global energy data for three phases
extern EnergyData PhaseEnergy[3];

// Global JSON response buffer
extern String serJsonResponse;

#endif // DATA_STRUCTURES_H