#include "RpcHandlers.h"
#include "../config/Configuration.h"
#include "../data/DataStructures.h"
#include "../data/DataProcessing.h"

// Safe processing helper for bounded snprintf operations
size_t finalize_buffer(int written, size_t maxLen) {
  if (written < 0) return 0;
  if ((size_t)written >= maxLen) return maxLen - 1;
  return (size_t)written;
}

// aligned with Shelly API docs
size_t shellyGetDeviceInfo(char* buffer, size_t maxLen) {
  int written = snprintf(buffer, maxLen,
      "{\"id\":\"%s\",\"mac\":\"%s\",\"slot\":1,\"model\":\"SPEM-003CEBEU\",\"gen\":%d,\"fw_id\":\"%s\",\"ver\":\"1.4.4\",\"app\":\"Pro3EM\",\"auth_en\":false,\"auth_domain\":null,\"profile\":\"triphase\"}",
      shelly_name.c_str(), shelly_mac.as<const char*>(), shelly_gen.as<int>(), shelly_fw_id.as<const char*>()
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t sysGetConfig(char* buffer, size_t maxLen) {
  IPAddress ip = WiFi.localIP();
  int written = snprintf(buffer, maxLen,
      "{\"device\":{\"name\":\"%s\",\"mac\":\"%s\",\"fw_id\":\"%s\",\"eco_mode\":false,\"profile\":\"triphase\",\"discoverable\":false},"
      "\"location\":{\"tz\":\"Europe/Berlin\",\"lat\":54.306,\"lon\":9.663},\"debug\":{\"mqtt\":{\"enable\":false},\"websocket\":{\"enable\":false},\"udp\":{\"addr\":null}},"
      "\"ui_data\":{},\"rpc_udp\":{\"dst_addr\":\"%u.%u.%u.%u\",\"listen_port\":%d},\"sntp\":{\"server\":null},\"cfg_rev\":10}",
      shelly_name.c_str(), shelly_mac.as<const char*>(), shelly_fw_id.as<const char*>(),
      (unsigned int)ip[0], (unsigned int)ip[1], (unsigned int)ip[2], (unsigned int)ip[3], config_doc["shelly"]["shelly_port"].as<int>()
  );
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t sysGetStatus(char* buffer, size_t maxLen) {
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char time_buffer[6];
  strftime(time_buffer, sizeof(time_buffer), "%H:%M", &timeinfo);

  uint32_t ram_total = 0;

#ifdef ESP32
  ram_total = ESP.getHeapSize();
#else
  ram_total = 0; // what makes sense here?
#endif

  int written = snprintf(buffer, maxLen,
      "{\"mac\":\"%s\",\"restart_required\":false,\"time\":\"%s\",\"unixtime\":%lld,\"last_sync_ts\":%lld,\"uptime\":%lu,"
      "\"ram_size\":%lu,\"ram_free\":%lu,\"fs_size\":%lu,\"fs_free\":%lu,\"cfg_rev\":10,\"kvs_rev\":2725,\"schedule_rev\":0,"
      "\"webhook_rev\":0,\"btrelay_rev\":0,\"available_updates\":{\"beta\":{\"version\":\"1.7.5-beta1\"}}}",
      shelly_mac.as<const char*>(), time_buffer, (long long)now, (long long)now, millis() / 1000,
      (unsigned long)ram_total, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFlashChipSize(), (unsigned long)ESP.getFreeSketchSpace()
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t EMGetStatus(char* buffer, size_t maxLen) {
  float total_power = PhasePower[0].power + PhasePower[1].power + PhasePower[2].power;
  float total_apparent = PhasePower[0].apparentPower + PhasePower[1].apparentPower + PhasePower[2].apparentPower;

  int written = snprintf(buffer, maxLen,
      "{\"id\":0,\"a_current\":%.2f,\"a_voltage\":%.2f,\"a_act_power\":%.2f,\"a_aprt_power\":%.2f,\"a_pf\":%.2f,\"a_freq\":%.2f,"
      "\"b_current\":%.2f,\"b_voltage\":%.2f,\"b_act_power\":%.2f,\"b_aprt_power\":%.2f,\"b_pf\":%.2f,\"b_freq\":%.2f,"
      "\"c_current\":%.2f,\"c_voltage\":%.2f,\"c_act_power\":%.2f,\"c_aprt_power\":%.2f,\"c_pf\":%.2f,\"c_freq\":%.2f,"
      "\"n_current\":null,\"total_current\":%.2f,\"total_act_power\":%.2f,\"total_aprt_power\":%.2f,\"user_calibrated_phase\":[]}",
      PhasePower[0].current, PhasePower[0].voltage, PhasePower[0].power, PhasePower[0].apparentPower, PhasePower[0].powerFactor, PhasePower[0].frequency,
      PhasePower[1].current, PhasePower[1].voltage, PhasePower[1].power, PhasePower[1].apparentPower, PhasePower[1].powerFactor, PhasePower[1].frequency,
      PhasePower[2].current, PhasePower[2].voltage, PhasePower[2].power, PhasePower[2].apparentPower, PhasePower[2].powerFactor, PhasePower[2].frequency,
      total_power / defaultVoltage, total_power, total_apparent
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t EMDataGetStatus(char* buffer, size_t maxLen) {
  int written = snprintf(buffer, maxLen,
      "{\"id\":0,\"a_total_act_energy\":%.2f,\"a_total_act_ret_energy\":%.2f,"
      "\"b_total_act_energy\":%.2f,\"b_total_act_ret_energy\":%.2f,"
      "\"c_total_act_energy\":%.2f,\"c_total_act_ret_energy\":%.2f,"
      "\"total_act\":%.2f,\"total_act_ret\":%.2f}",
      PhaseEnergy[0].consumption, PhaseEnergy[0].gridfeedin,
      PhaseEnergy[1].consumption, PhaseEnergy[1].gridfeedin,
      PhaseEnergy[2].consumption, PhaseEnergy[2].gridfeedin,
      PhaseEnergy[0].consumption + PhaseEnergy[1].consumption + PhaseEnergy[2].consumption,
      PhaseEnergy[0].gridfeedin + PhaseEnergy[1].gridfeedin + PhaseEnergy[2].gridfeedin
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t EMGetConfig(char* buffer, size_t maxLen) {
  int written = snprintf(buffer, maxLen,
      "{\"id\":0,\"name\":null,\"blink_mode_selector\":\"active_energy\",\"phase_selector\":\"a\",\"monitor_phase_sequence\":true,\"reverse\":{},\"ct_type\":\"120A\"}"
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t shellyGetConfig(char* buffer, size_t maxLen) {
  char emConfig[256] = "";
  char sysConfig[512] = "";
  EMGetConfig(emConfig, sizeof(emConfig));
  sysGetConfig(sysConfig, sizeof(sysConfig));

  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress gw = WiFi.gatewayIP();
  IPAddress dns = WiFi.dnsIP();

  int written = snprintf(buffer, maxLen,
      "{\"ble\":{\"enable\":false},\"cloud\":{\"enable\":false,\"server\":null},\"em:0\":%s,\"sys\":%s,"
      "\"wifi\":{\"sta\":{\"ssid\":\"%s\",\"is_open\":false,\"enable\":true,\"ipv4mode\":\"dhcp\","
      "\"ip\":\"%u.%u.%u.%u\",\"netmask\":\"%u.%u.%u.%u\",\"gw\":\"%u.%u.%u.%u\",\"nameserver\":\"%u.%u.%u.%u\"},"
      "\"ws\":{\"enable\":false,\"server\":null,\"ssl_ca\":\"ca.pem\"}}}",
      emConfig, sysConfig, WiFi.SSID().c_str(),
      (unsigned int)ip[0], (unsigned int)ip[1], (unsigned int)ip[2], (unsigned int)ip[3],
      (unsigned int)mask[0], (unsigned int)mask[1], (unsigned int)mask[2], (unsigned int)mask[3],
      (unsigned int)gw[0], (unsigned int)gw[1], (unsigned int)gw[2], (unsigned int)gw[3],
      (unsigned int)dns[0], (unsigned int)dns[1], (unsigned int)dns[2], (unsigned int)dns[3]
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t shellyComponents(char* buffer, size_t maxLen) {
  char emStatus[768] = "";
  char emConfig[256] = "";
  char emDataStatus[512] = "";

  EMGetStatus(emStatus, sizeof(emStatus));
  EMGetConfig(emConfig, sizeof(emConfig));
  EMDataGetStatus(emDataStatus, sizeof(emDataStatus));

  int written = snprintf(buffer, maxLen,
      "{\"components\":[{\"key\":\"em:0\",\"status\":%s,\"config\":%s},{\"key\":\"emdata:0\",\"status\":%s,\"config\":{}}],\"cfg_rev\":1,\"offset\":0,\"total\":2}",
      emStatus, emConfig, emDataStatus
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t shellyGetStatus(char* buffer, size_t maxLen) {
  double temperature = 26.55;
#ifdef ESP32
  temperature = temperatureRead();
#endif

  char emStatus[768] = "";
  char emDataStatus[512] = "";
  char sysStatus[512] = "";

  EMGetStatus(emStatus, sizeof(emStatus));
  EMDataGetStatus(emDataStatus, sizeof(emDataStatus));
  sysGetStatus(sysStatus, sizeof(sysStatus));

  bool connected = (WiFi.status() == WL_CONNECTED);
  char wifiBlock[256];

  if (connected) {
    IPAddress ip = WiFi.localIP();
    const uint8_t* bssid = WiFi.BSSID();
    uint8_t zeroBssid[6] = {0, 0, 0, 0, 0, 0};
    if (bssid == nullptr) {
      bssid = zeroBssid;
    }

    snprintf(wifiBlock, sizeof(wifiBlock),
        "{\"sta_ip\":\"%u.%u.%u.%u\",\"status\":\"got ip\",\"ssid\":\"%s\","
        "\"bssid\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"rssi\":%d,\"sta_ip6\":[]}",
        (unsigned int)ip[0], (unsigned int)ip[1], (unsigned int)ip[2], (unsigned int)ip[3],
        WiFi.SSID().c_str(),
        (unsigned int)bssid[0], (unsigned int)bssid[1], (unsigned int)bssid[2], (unsigned int)bssid[3], (unsigned int)bssid[4], (unsigned int)bssid[5],
        WiFi.RSSI()
    );
  } else {
    snprintf(wifiBlock, sizeof(wifiBlock),
        "{\"sta_ip\":null,\"status\":\"connecting\",\"ssid\":null,\"bssid\":null,\"rssi\":0,\"sta_ip6\":[]}"
    );
  }

  int written = snprintf(buffer, maxLen,
      "{\"ble\":{},\"bthome\":{},\"cloud\":{\"connected\":false},\"em:0\":%s,\"emdata:0\":%s,\"eth\":{\"ip\":null,\"ip6\":null},"
      "\"modbus\":{},\"mqtt\":{\"connected\":false},\"sys\":%s,\"temperature:0\":{\"id\":0,\"tC\":%.2f,\"tF\":%.2f},"
      "\"wifi\":%s,\"ws\":{\"connected\":false}}",
      emStatus, emDataStatus, sysStatus, temperature, (temperature * 9.0 / 5.0) + 32.0, wifiBlock
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t wifiGetStatus(char* buffer, size_t maxLen) {
  int written = 0;
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    const uint8_t* bssid = WiFi.BSSID();
    uint8_t zeroBssid[6] = {0, 0, 0, 0, 0, 0};
    if (bssid == nullptr) {
      bssid = zeroBssid;
    }

    written = snprintf(buffer, maxLen,
        "{\"sta_ip\":\"%u.%u.%u.%u\",\"status\":\"got ip\",\"ssid\":\"%s\","
        "\"bssid\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"rssi\":%d,\"ap_client_count\":0}",
        (unsigned int)ip[0], (unsigned int)ip[1], (unsigned int)ip[2], (unsigned int)ip[3],
        WiFi.SSID().c_str(),
        (unsigned int)bssid[0], (unsigned int)bssid[1], (unsigned int)bssid[2], (unsigned int)bssid[3], (unsigned int)bssid[4], (unsigned int)bssid[5],
        WiFi.RSSI()
    );
  } else {
    written = snprintf(buffer, maxLen,
        "{\"sta_ip\":null,\"status\":\"connecting\",\"ssid\":null,\"bssid\":null,\"rssi\":0,\"ap_client_count\":0}"
    );
  }
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}
