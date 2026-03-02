#ifndef RPC_HANDLERS_H
#define RPC_HANDLERS_H

#include <Arduino.h>

// RPC response wrapper and generators
void rpcWrapper();
void shellyGetDeviceInfo();
void sysGetConfig();
void sysGetStatus();
void EMGetStatus();
void EMDataGetStatus();
void EMGetConfig();
void shellyGetConfig();
void shellyGetComponents();
void shellyGetStatus();
void wifiGetStatus();

#endif // RPC_HANDLERS_H