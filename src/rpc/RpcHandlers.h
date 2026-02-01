#ifndef RPC_HANDLERS_H
#define RPC_HANDLERS_H

#include <Arduino.h>

// RPC Response wrapper and generators
void rpcWrapper();
void GetDeviceInfo();
void EMGetStatus();
void EMDataGetStatus();
void EMGetConfig();

// LED control (used by RPC handlers and main loop)
void blinkled(int duration);
void handleblinkled();

#endif // RPC_HANDLERS_H
