#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

enum class Energ2Shelly_ResetReason
{
  POWER_ON = 0,
  LOW_MEMORY = 1,
  MANUAL_RESET = 2,
  WIFI_DISCONNECT = 3,
  OTHER = 4
};

void all_esp_reset(Energ2Shelly_ResetReason reason);
void stackWD(void);
uint64_t extendedMillis();
void status_print(void);
void clear_rtc_power_on(void);

// to be tuned for ESP32 and ESP8266
#if defined(ESP32)
#define MIN_HEAP_SIZE 12000
#elif defined(ESP8266)
#define MIN_HEAP_SIZE 5000
#endif


#endif // UTILS_H