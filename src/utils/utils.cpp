#include "utils.h"
#include "config/Configuration.h"
#ifdef ESP32
extern "C" {
 #include "nvs_flash.h"
  #include "nvs.h"
}
#endif


// 1. Variable definitions and platform separation
#if defined(ESP32)
RTC_NOINIT_ATTR Energy2Shelly_ResetReason rtc_reset_reason;
RTC_NOINIT_ATTR uint32_t rtcMagicNumber;
uint32_t rtc_reset_counter;
#elif defined(ESP8266)
Energy2Shelly_ResetReason rtc_reset_reason; // Manually synced.
uint32_t rtc_reset_counter;
// first 0..31 slots maybe used by OTA
#define RTC_reset_counter_slot 125
#define RTC_magic_key_slot 126
#define RTC_reset_reason_slot 127

#endif
Energy2Shelly_ResetReason ram_copy_rtc_reset_reason;

void wifi_reset()
{
#ifdef ESP32
    WiFi.disconnect(true, true);
#else
    // WiFiManager may leave persistent=false; force it so disconnect(true)
    // actually erases the SDK-saved credentials in flash.
    WiFi.persistent(true);
    WiFi.disconnect(true);
    ESP.eraseConfig();
#endif
    update_reset_reason(Energy2Shelly_ResetReason::WIFI_RECONFIG);
    delay(1000);
    ESP.restart();
    delay(1000);
 
}

void reset_esp_to_default_and_reboot()
{
  // disconnect from WiFi and erase all credentials
  #ifdef ESP32
    WiFi.disconnect(true, true);
  #else
    WiFi.disconnect(true);      
  #endif

  LittleFS.format(); // format allways, to ensure a clean state

  #ifdef ESP32
  // Erase NVS to remove all stored WiFi credentials and other settings
    if (nvs_flash_erase() == ESP_OK) {
        nvs_flash_init(); 
    }
  #endif
  
  update_reset_reason(Energy2Shelly_ResetReason::FACTORY_RESET);
  
  delay(1000);
  
  ESP.restart();

  delay(1000);
}


#ifdef ESP32

uint32_t read_reset_counter_from_nvs()
{
    uint32_t rtc_reset_counter = 0; // Default value
    nvs_handle_t my_handle;
    if (nvs_open("e2s_config", NVS_READONLY, &my_handle) == ESP_OK) {
        uint32_t temp_counter = 0;
        if (nvs_get_u32(my_handle, "reset_counter", &temp_counter) == ESP_OK) {
            rtc_reset_counter = temp_counter;
        } else {
            rtc_reset_counter = 0; // Default value if not present
        }
        nvs_close(my_handle);
    } else {
        rtc_reset_counter = 0; // Default value if NVS open fails
    }
    return rtc_reset_counter;
}

void write_reset_counter_to_nvs(uint32_t rtc_reset_counter)
{
    nvs_handle_t my_handle;
    if (nvs_open("e2s_config", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u32(my_handle, "reset_counter", rtc_reset_counter);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}


Energy2Shelly_ResetReason clear_rtc_power_on(void)
{
  esp_reset_reason_t reason = esp_reset_reason();

  // Check if hardware reports either a power-on or a hardware pin reset
  if (reason == ESP_RST_POWERON || reason == ESP_RST_EXT)
  {
    // If the magic number is missing, it was a genuine cold boot (power applied)
    if (rtcMagicNumber != 0xDEADBEEF)
    {
      rtcMagicNumber = 0xDEADBEEF; // Prepare for the next boot cycle
      rtc_reset_reason = Energy2Shelly_ResetReason::POWER_ON;
    }
    else
    {
      // If the magic number WAS present, but hardware reports POWERON/EXT, it was the pin!
      rtc_reset_reason = Energy2Shelly_ResetReason::MANUAL_RESET;
    }
  }
  // For all other reset reasons (e.g., Software Restart, Deep Sleep),
  // the old value of rtc_reset_reason is naturally preserved in the RTC RAM.

  if ((rtc_reset_reason==Energy2Shelly_ResetReason::MANUAL_RESET)||(rtc_reset_reason==Energy2Shelly_ResetReason::POWER_ON)) {
      rtc_reset_counter = read_reset_counter_from_nvs(); // return 0 if not present

      rtc_reset_counter++;

      write_reset_counter_to_nvs(rtc_reset_counter);
    
       if (rtc_reset_counter>=RTC_RESET_COUNTER_THRESHOLD) {
          rtc_reset_counter=0;
          // no need to set rtc_reset_counter in nvs, the next function will erase everything
          reset_esp_to_default_and_reboot();
       } 
  }
  else {
    rtc_reset_counter = read_reset_counter_from_nvs();
  }
  DEBUG_SERIAL.print(F("Reset counter:"));
  DEBUG_SERIAL.println(rtc_reset_counter); 
  
  
  ram_copy_rtc_reset_reason=rtc_reset_reason;
  update_reset_reason(Energy2Shelly_ResetReason::OTHER); // set to unknown;
  return ram_copy_rtc_reset_reason;

}


#else
Energy2Shelly_ResetReason clear_rtc_power_on(void)
{
  rst_info *resetInfo = ESP.getResetInfoPtr();

  // Check if hardware reports a power-on or external reset
  if (resetInfo->reason == REASON_DEFAULT_RST || resetInfo->reason == REASON_EXT_SYS_RST)
  {
    uint32_t esp8266MagicNumber = 0;
    ESP.rtcUserMemoryRead(RTC_magic_key_slot, &esp8266MagicNumber, sizeof(esp8266MagicNumber));

    if (esp8266MagicNumber != 0xDEADBEEF)
    {
      // Magic number missing -> True cold boot (power applied)
      esp8266MagicNumber = 0xDEADBEEF;
      rtc_reset_counter = 0;
      ESP.rtcUserMemoryWrite(RTC_magic_key_slot, &esp8266MagicNumber, sizeof(esp8266MagicNumber));
      ESP.rtcUserMemoryWrite(RTC_reset_counter_slot, &rtc_reset_counter, sizeof(rtc_reset_counter));
      rtc_reset_reason = Energy2Shelly_ResetReason::POWER_ON;
    }
    else
    {
      // Magic number exists -> True manual press on the RST pin
      rtc_reset_reason = Energy2Shelly_ResetReason::MANUAL_RESET;
    }

     ESP.rtcUserMemoryWrite(RTC_reset_reason_slot, (uint32_t *)&rtc_reset_reason, sizeof(rtc_reset_reason));
  }
  else
  {
    // Not a power-on/pin reset -> Restore previous state from slot (e.g., after software reset)
    ESP.rtcUserMemoryRead(RTC_reset_reason_slot, (uint32_t *)&rtc_reset_reason, sizeof(rtc_reset_reason));
  }

  if ((rtc_reset_reason==Energy2Shelly_ResetReason::MANUAL_RESET)||(rtc_reset_reason==Energy2Shelly_ResetReason::POWER_ON)) {
      ESP.rtcUserMemoryRead(RTC_reset_counter_slot, &rtc_reset_counter, sizeof(rtc_reset_counter));
      rtc_reset_counter++;

      ESP.rtcUserMemoryWrite(RTC_reset_counter_slot, &rtc_reset_counter, sizeof(rtc_reset_counter));
    
      if (rtc_reset_counter>=RTC_RESET_COUNTER_THRESHOLD) {
          rtc_reset_counter=0;
          ESP.rtcUserMemoryWrite(RTC_reset_counter_slot, &rtc_reset_counter, sizeof(rtc_reset_counter));
          reset_esp_to_default_and_reboot();
       } 
  }
  else {
    ESP.rtcUserMemoryRead(RTC_reset_counter_slot, &rtc_reset_counter, sizeof(rtc_reset_counter));
  }
  DEBUG_SERIAL.print(F("Reset counter:"));
  DEBUG_SERIAL.println(rtc_reset_counter); 
  
  
  ram_copy_rtc_reset_reason=rtc_reset_reason;
  update_reset_reason(Energy2Shelly_ResetReason::OTHER); // set to unknown;
  return ram_copy_rtc_reset_reason;

}



#endif


void update_reset_reason(Energy2Shelly_ResetReason reason)
{
  rtc_reset_reason = reason;
#if defined(ESP8266)
  ESP.rtcUserMemoryWrite(RTC_reset_reason_slot, (uint32_t *)&rtc_reset_reason, sizeof(rtc_reset_reason));
#endif
}

void set_rtc_reset_counter(uint32_t value)
{
  rtc_reset_counter = value;
#if defined(ESP8266)
  ESP.rtcUserMemoryWrite(RTC_reset_counter_slot, (uint32_t *)&rtc_reset_counter, sizeof(rtc_reset_counter));
#else
  write_reset_counter_to_nvs(rtc_reset_counter);
#endif
}


void all_esp_reset(Energy2Shelly_ResetReason reason)
{
  update_reset_reason(reason);

  delay(1000);
  ESP.restart();
  delay(1000);
}

void stackWD(void)
{
  uint32_t freeHeap = ESP.getFreeHeap();
#if defined(ESP32)
  uint32_t maxBlock = ESP.getMaxAllocHeap();
#elif defined(ESP8266)
  uint32_t maxBlock = ESP.getMaxFreeBlockSize();
#endif
  DEBUG_SERIAL.print(F("Free heap: "));
  DEBUG_SERIAL.print(freeHeap);
  DEBUG_SERIAL.print(F(" bytes, Max free block: "));
  DEBUG_SERIAL.print(maxBlock);
  DEBUG_SERIAL.println(F(" bytes"));
  DEBUG_SERIAL.print(F("Reset counts: "));
  DEBUG_SERIAL.println(rtc_reset_counter);
  
  if (maxBlock < MIN_HEAP_SIZE)
  {
    DEBUG_SERIAL.println(F("Low memory detected, restarting..."));
    all_esp_reset(Energy2Shelly_ResetReason::LOW_MEMORY);
  }
}


uint64_t extendedMillis()
{
  static unsigned long lastMillis = 0;
  static uint64_t overflows = 0;

  unsigned long currentMillis = millis();

  // detect overflow of millis() and increment the overflow counter
  if (currentMillis < lastMillis)
  {
    overflows++;
  }
  lastMillis = currentMillis;

  // combine overflows and currentMillis to get the extended time in milliseconds
  return (overflows << 32) + currentMillis;
}


void checkreset_counter()
{
  if ((extendedMillis()>RTC_RESET_COUNTER_CLEAR_INTERVAL_MS) && (rtc_reset_counter>0)) {
      DEBUG_SERIAL.println(F("Set reset counter to 0"));
      set_rtc_reset_counter(0);
  }

}


void status_print(void)
{
  DEBUG_SERIAL.print(F("Wifi, RSSI: "));
  DEBUG_SERIAL.print(WiFi.RSSI());
  DEBUG_SERIAL.print(F(" dBm, Access Point MAC: "));
  uint8_t *bssid = WiFi.BSSID();
  if (bssid)
  {
    DEBUG_SERIAL.printf("%02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
  }
  DEBUG_SERIAL.println();
  uint64_t longMillis = extendedMillis();

  uint64_t ontime_minutes = longMillis / 60000ll;
  uint64_t ontime_hours = ontime_minutes / 60ll;

  // separate days, hours, and minutes for better readability
  uint32_t days = (uint32_t)(ontime_hours / 24ll);
  uint32_t hours = (uint32_t)(ontime_hours % 24ll);
  uint32_t minutes = (uint32_t)(ontime_minutes % 60ll);

  DEBUG_SERIAL.print(F("Ontime: "));
  DEBUG_SERIAL.print(days);
  DEBUG_SERIAL.print(F(" days, "));
  DEBUG_SERIAL.printf("%02d:%02d", hours, minutes);
  DEBUG_SERIAL.println(F(" h"));
 
  DEBUG_SERIAL.print(F("Reset reason: "));
  switch (ram_copy_rtc_reset_reason)
  {
  case Energy2Shelly_ResetReason::POWER_ON:
    DEBUG_SERIAL.print(F("Power-on reset"));
    break;
  case Energy2Shelly_ResetReason::LOW_MEMORY:
    DEBUG_SERIAL.print(F("Low memory reset"));
    break;
  case Energy2Shelly_ResetReason::WIFI_DISCONNECT:
    DEBUG_SERIAL.print(F("WiFi disconnect reset"));
    break;
  case Energy2Shelly_ResetReason::MANUAL_RESET:
    DEBUG_SERIAL.print(F("Manual reset"));
    break;
  case Energy2Shelly_ResetReason::OTHER:
    DEBUG_SERIAL.print(F("Other reset"));
    break;
    case Energy2Shelly_ResetReason::OTA_UPDATE:
    DEBUG_SERIAL.print(F("OTA reset"));
    break;
  default:
    DEBUG_SERIAL.print(F("Unknown reset reason"));
    break;
  }
  DEBUG_SERIAL.print(F(", reconnect attempts: "));
  DEBUG_SERIAL.println(wifi_reconnect_attempts);
}


String getISO8601Time() {
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "1970-01-01T00:00:00Z"; // Fallback, 
    }
    time(&now);
    gmtime_r(&now, &timeinfo); // In UTC 

    char buf[25];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(buf);
}
