#ifndef SML_COMMON_H
#define SML_COMMON_H

#include <sml.h>

// Shared SML handling for all SML based inputs (TibberPulse via HTTP, SMLSERIAL via UART).
// Feed the stream byte by byte. The supported OBIS values are collected and published via
// setEnergyData()/setPowerData() as soon as a telegram with valid checksum is complete,
// which is signaled by the returned state SML_FINAL.
sml_states_t smlParseByte(unsigned char val);

// true if the last complete telegram contained a power value (16.7.0 or all phases)
bool smlLastTelegramHadPower();

#endif // SML_COMMON_H
