#ifndef RPC_HANDLERS_H
#define RPC_HANDLERS_H

#include <Arduino.h>

size_t finalize_buffer(int written, size_t maxLen);
size_t shellyGetDeviceInfo(char* buffer, size_t maxLen);
size_t sysGetConfig(char* buffer, size_t maxLen);
size_t sysGetStatus(char* buffer, size_t maxLen);
size_t EMGetStatus(char* buffer, size_t maxLen);
size_t EMDataGetStatus(char* buffer, size_t maxLen);
size_t EMGetConfig(char* buffer, size_t maxLen);
size_t shellyGetConfig(char* buffer, size_t maxLen);
size_t shellyComponents(char* buffer, size_t maxLen);
size_t shellyGetStatus(char* buffer, size_t maxLen);
size_t wifiGetStatus(char* buffer, size_t maxLen);

#endif // RPC_HANDLERS_H