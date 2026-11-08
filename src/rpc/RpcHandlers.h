#ifndef RPC_HANDLERS_H
#define RPC_HANDLERS_H

#include <Arduino.h>
#include <ArduinoJson.h>

// RPC response wrapper and generators
void rpcWrapper();
void shellyGetDeviceInfo();
void sysGetConfig();
void sysGetStatus();
void fillEMStatus(JsonObject jsonResponse);
void fillEMDataStatus(JsonObject jsonResponse);
void EMGetStatus();
void EMDataGetStatus();
void EMGetConfig();
void shellyGetConfig();
void shellyGetComponents();
void shellyGetStatus();
void shellyListMethods();
void wifiGetStatus();

#endif // RPC_HANDLERS_H