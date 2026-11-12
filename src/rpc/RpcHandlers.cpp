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
  int written = snprintf_P(buffer, maxLen,
      PSTR("{\"id\":\"%s\",\"mac\":\"%s\",\"slot\":1,\"model\":\"SPEM-003CEBEU\",\"gen\":%d,\"fw_id\":\"%s\",\"ver\":\"1.4.4\",\"app\":\"Pro3EM\",\"auth_en\":false,\"auth_domain\":null,\"profile\":\"triphase\"}"),
      shelly_name.c_str(), shelly_mac.as<const char*>(), shelly_gen.as<int>(), shelly_fw_id.as<const char*>()
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t sysGetConfig(char* buffer, size_t maxLen) {
  IPAddress ip = WiFi.localIP();
  int written = snprintf_P(buffer, maxLen,
      PSTR("{\"device\":{\"name\":\"%s\",\"mac\":\"%s\",\"fw_id\":\"%s\",\"eco_mode\":false,\"profile\":\"triphase\",\"discoverable\":false},"
      "\"location\":{\"tz\":\"Europe/Berlin\",\"lat\":54.306,\"lon\":9.663},\"debug\":{\"mqtt\":{\"enable\":false},\"websocket\":{\"enable\":false},\"udp\":{\"addr\":null}},"
      "\"ui_data\":{},\"rpc_udp\":{\"dst_addr\":\"%u.%u.%u.%u\",\"listen_port\":%d},\"sntp\":{\"server\":null},\"cfg_rev\":10}"),
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

  int written = snprintf_P(buffer, maxLen,
      PSTR("{\"mac\":\"%s\",\"restart_required\":false,\"time\":\"%s\",\"unixtime\":%lld,\"last_sync_ts\":%lld,\"uptime\":%lu,"
      "\"ram_size\":%lu,\"ram_free\":%lu,\"fs_size\":%lu,\"fs_free\":%lu,\"cfg_rev\":10,\"kvs_rev\":2725,\"schedule_rev\":0,"
      "\"webhook_rev\":0,\"btrelay_rev\":0,\"available_updates\":{\"beta\":{\"version\":\"1.7.5-beta1\"}}}"),
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

  int written = snprintf_P(buffer, maxLen,
      PSTR("{\"id\":0,\"a_current\":%.2f,\"a_voltage\":%.2f,\"a_act_power\":%.2f,\"a_aprt_power\":%.2f,\"a_pf\":%.2f,\"a_freq\":%.2f,"
      "\"b_current\":%.2f,\"b_voltage\":%.2f,\"b_act_power\":%.2f,\"b_aprt_power\":%.2f,\"b_pf\":%.2f,\"b_freq\":%.2f,"
      "\"c_current\":%.2f,\"c_voltage\":%.2f,\"c_act_power\":%.2f,\"c_aprt_power\":%.2f,\"c_pf\":%.2f,\"c_freq\":%.2f,"
      "\"n_current\":null,\"total_current\":%.2f,\"total_act_power\":%.2f,\"total_aprt_power\":%.2f,\"user_calibrated_phase\":[]}"),
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
  int written = snprintf_P(buffer, maxLen,
      PSTR("{\"id\":0,\"a_total_act_energy\":%.2f,\"a_total_act_ret_energy\":%.2f,"
      "\"b_total_act_energy\":%.2f,\"b_total_act_ret_energy\":%.2f,"
      "\"c_total_act_energy\":%.2f,\"c_total_act_ret_energy\":%.2f,"
      "\"total_act\":%.2f,\"total_act_ret\":%.2f}"),
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
  int written = snprintf_P(buffer, maxLen,
      PSTR("{\"id\":0,\"name\":null,\"blink_mode_selector\":\"active_energy\",\"phase_selector\":\"a\",\"monitor_phase_sequence\":true,\"reverse\":{},\"ct_type\":\"120A\"}")
  );
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}

// aligned with Shelly API docs
size_t shellyGetConfig(char* buffer, size_t maxLen) {
  if (buffer == nullptr || maxLen == 0) {
    return 0;
  }

  size_t pos = 0;
  int written = snprintf_P(buffer + pos, maxLen - pos,
      PSTR("{\"ble\":{\"enable\":false},\"cloud\":{\"enable\":false,\"server\":null},\"em:0\":"));
  if (written < 0 || (size_t)written >= maxLen - pos) {
    buffer[maxLen - 1] = '\0';
    return maxLen - 1;
  }
  pos += (size_t)written;

  // em:0-Konfiguration direkt in den Ausgabepuffer schreiben
  size_t remaining = maxLen - pos;
  if (remaining <= 1) {
    buffer[maxLen - 1] = '\0';
    return maxLen - 1;
  }

  size_t appended = EMGetConfig(buffer + pos, remaining);
  if (appended >= remaining - 1) {
    buffer[maxLen - 1] = '\0';
    return maxLen - 1;
  }
  pos += appended;

  written = snprintf_P(buffer + pos, maxLen - pos, PSTR(",\"sys\":"));
  if (written < 0 || (size_t)written >= maxLen - pos) {
    buffer[maxLen - 1] = '\0';
    return maxLen - 1;
  }
  pos += (size_t)written;

  // sys-Konfiguration direkt in den Ausgabepuffer schreiben
  remaining = maxLen - pos;
  if (remaining <= 1) {
    buffer[maxLen - 1] = '\0';
    return maxLen - 1;
  }

  appended = sysGetConfig(buffer + pos, remaining);
  if (appended >= remaining - 1) {
    buffer[maxLen - 1] = '\0';
    return maxLen - 1;
  }
  pos += appended;

  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress gw = WiFi.gatewayIP();
  IPAddress dns = WiFi.dnsIP();
  const char* ssid = config_doc["wifi"]["ssid"] | "";

  written = snprintf_P(buffer + pos, maxLen - pos,
      PSTR(",\"wifi\":{\"sta\":{\"ssid\":\"%s\",\"is_open\":false,\"enable\":true,\"ipv4mode\":\"dhcp\","
           "\"ip\":\"%u.%u.%u.%u\",\"netmask\":\"%u.%u.%u.%u\","
           "\"gw\":\"%u.%u.%u.%u\",\"nameserver\":\"%u.%u.%u.%u\"},"
           "\"ws\":{\"enable\":false,\"server\":null,\"ssl_ca\":\"ca.pem\"}}}"),
      ssid,
      (unsigned int)ip[0], (unsigned int)ip[1], (unsigned int)ip[2], (unsigned int)ip[3],
      (unsigned int)mask[0], (unsigned int)mask[1], (unsigned int)mask[2], (unsigned int)mask[3],
      (unsigned int)gw[0], (unsigned int)gw[1], (unsigned int)gw[2], (unsigned int)gw[3],
      (unsigned int)dns[0], (unsigned int)dns[1], (unsigned int)dns[2], (unsigned int)dns[3]);

  if (written < 0 || (size_t)written >= maxLen - pos) {
    buffer[maxLen - 1] = '\0';
    return maxLen - 1;
  }

  pos += (size_t)written;
  blinkled(ledblinkduration);
  return pos;
}

// aligned with Shelly API docs
size_t shellyComponents(char* buffer, size_t maxLen) {
  if (buffer == nullptr || maxLen == 0) {
    return 0;
  }

  size_t pos = 0;

  // 1) em:0 + status
  if (pos < maxLen - 1) {
    pos += finalize_buffer(
      snprintf_P(buffer + pos, maxLen - pos,
        PSTR("{\"components\":[{\"key\":\"em:0\",\"status\":")),
      maxLen - pos
    );
  }
  if (pos < maxLen - 1) {
    pos += EMGetStatus(buffer + pos, maxLen - pos);
  }

  // 2) config
  if (pos < maxLen - 1) {
    pos += finalize_buffer(
      snprintf_P(buffer + pos, maxLen - pos,
        PSTR(",\"config\":")),
      maxLen - pos
    );
  }
  if (pos < maxLen - 1) {
    pos += EMGetConfig(buffer + pos, maxLen - pos);
  }

  // 3) emdata:0 + status
  if (pos < maxLen - 1) {
    pos += finalize_buffer(
      snprintf_P(buffer + pos, maxLen - pos,
        PSTR("},{\"key\":\"emdata:0\",\"status\":")),
      maxLen - pos
    );
  }
  if (pos < maxLen - 1) {
    pos += EMDataGetStatus(buffer + pos, maxLen - pos);
  }

  // 4) config + closing JSON
  if (pos < maxLen - 1) {
    pos += finalize_buffer(
      snprintf_P(buffer + pos, maxLen - pos,
        PSTR(",\"config\":{}}],\"cfg_rev\":1,\"offset\":0,\"total\":2}")),
      maxLen - pos
    );
  }

  if (pos < maxLen) {
    buffer[pos] = '\0';
  } else {
    buffer[maxLen - 1] = '\0';
  }

  blinkled(ledblinkduration);
  return pos;
}

// aligned with Shelly API docs
size_t shellyGetStatus(char* buffer, size_t maxLen) {
  if (buffer == nullptr || maxLen == 0) {
    return 0;
  }

  double temperature = 26.55;
#ifdef ESP32
  temperature = temperatureRead();
#endif

  size_t pos = 0;

  if (pos < maxLen - 1) {
    pos += finalize_buffer(
      snprintf_P(buffer + pos, maxLen - pos,
        PSTR("{\"ble\":{},\"bthome\":{},\"cloud\":{\"connected\":false},\"em:0\":")),
      maxLen - pos
    );
  }
  if (pos < maxLen - 1) {
    pos += EMGetStatus(buffer + pos, maxLen - pos);
  }

  if (pos < maxLen - 1) {
    pos += finalize_buffer(
      snprintf_P(buffer + pos, maxLen - pos,
        PSTR(",\"emdata:0\":")),
      maxLen - pos
    );
  }
  if (pos < maxLen - 1) {
    pos += EMDataGetStatus(buffer + pos, maxLen - pos);
  }

  if (pos < maxLen - 1) {
    pos += finalize_buffer(
      snprintf_P(buffer + pos, maxLen - pos,
        PSTR(",\"eth\":{\"ip\":null,\"ip6\":null},\"modbus\":{},\"mqtt\":{\"connected\":false},\"sys\":")),
      maxLen - pos
    );
  }
  if (pos < maxLen - 1) {
    pos += sysGetStatus(buffer + pos, maxLen - pos);
  }

  if (pos < maxLen - 1) {
    pos += finalize_buffer(
      snprintf_P(buffer + pos, maxLen - pos,
        PSTR(",\"temperature:0\":{\"id\":0,\"tC\":%.2f,\"tF\":%.2f},\"wifi\":"),
        temperature,
        (temperature * 9.0 / 5.0) + 32.0),
      maxLen - pos
    );
  }

  if (pos < maxLen - 1) {
    if (WiFi.status() == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      const uint8_t* bssid = WiFi.BSSID();
      uint8_t zeroBssid[6] = {0};

      if (bssid == nullptr) {
        bssid = zeroBssid;
      }

      const char* ssid = config_doc["wifi"]["ssid"] | "";

      pos += finalize_buffer(
        snprintf_P(buffer + pos, maxLen - pos,
          PSTR("{\"sta_ip\":\"%u.%u.%u.%u\",\"status\":\"got ip\",\"ssid\":\"%s\","
               "\"bssid\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"rssi\":%d,\"sta_ip6\":[]}"),
          (unsigned int)ip[0], (unsigned int)ip[1],
          (unsigned int)ip[2], (unsigned int)ip[3],
          ssid,
          (unsigned int)bssid[0], (unsigned int)bssid[1],
          (unsigned int)bssid[2], (unsigned int)bssid[3],
          (unsigned int)bssid[4], (unsigned int)bssid[5],
          WiFi.RSSI()),
        maxLen - pos
      );
    } else {
      pos += finalize_buffer(
        snprintf_P(buffer + pos, maxLen - pos,
          PSTR("{\"sta_ip\":null,\"status\":\"connecting\",\"ssid\":null,\"bssid\":null,\"rssi\":0,\"sta_ip6\":[]}")),
        maxLen - pos
      );
    }
  }

  if (pos < maxLen) {
    buffer[pos] = '\0';
  } else {
    buffer[maxLen - 1] = '\0';
  }

  blinkled(ledblinkduration);
  return pos;
}

// aligned with Shelly API docs
size_t wifiGetStatus(char* buffer, size_t maxLen) {
  int written = 0;
  if (WiFi.status() == WL_CONNECTED) {
    const char* ssid = config_doc["wifi"]["ssid"] | "";
    IPAddress ip = WiFi.localIP();
    const uint8_t* bssid = WiFi.BSSID();
    uint8_t zeroBssid[6] = {0, 0, 0, 0, 0, 0};
    if (bssid == nullptr) {
      bssid = zeroBssid;
    }

    written = snprintf_P(buffer, maxLen,
        PSTR("{\"sta_ip\":\"%u.%u.%u.%u\",\"status\":\"got ip\",\"ssid\":\"%s\","
        "\"bssid\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"rssi\":%d,\"ap_client_count\":0}"),
        (unsigned int)ip[0], (unsigned int)ip[1], (unsigned int)ip[2], (unsigned int)ip[3],
        ssid,
        (unsigned int)bssid[0], (unsigned int)bssid[1], (unsigned int)bssid[2], (unsigned int)bssid[3], (unsigned int)bssid[4], (unsigned int)bssid[5],
        WiFi.RSSI()
    );
  } else {
    written = snprintf_P(buffer, maxLen,
        PSTR("{\"sta_ip\":null,\"status\":\"connecting\",\"ssid\":null,\"bssid\":null,\"rssi\":0,\"ap_client_count\":0}")
    );
  }
  blinkled(ledblinkduration);
  return finalize_buffer(written, maxLen);
}
