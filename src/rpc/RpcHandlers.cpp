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

// Stack-only helper for safe, zero-heap JSON composition
struct JsonWriter {
  char* buf;
  size_t maxLen;
  size_t pos;

  JsonWriter(char* buffer, size_t len) : buf(buffer), maxLen(len), pos(0) {
    if (buf != nullptr && maxLen > 0) {
      buf[0] = '\0';
    }
  }

  bool hasSpace() const {
    return buf != nullptr && maxLen > 0 && pos < maxLen - 1;
  }

  // Overload 1: Append formatted PROGMEM string
  template <typename... Args>
  void write(PGM_P fmt, Args... args) {
    if (!hasSpace()) return;
    int written = snprintf_P(buf + pos, maxLen - pos, fmt, args...);
    pos += finalize_buffer(written, maxLen - pos);
  }

  // Overload 2: Embed output from another RPC handler function
  void write(size_t (*handler)(char*, size_t)) {
    if (!hasSpace()) return;
    pos += handler(buf + pos, maxLen - pos);
  }
};

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
  JsonWriter json(buffer, maxLen);

  json.write(PSTR("{\"ble\":{\"enable\":false},\"cloud\":{\"enable\":false,\"server\":null},\"em:0\":"));
  json.write(EMGetConfig);
  json.write(PSTR(",\"sys\":"));
  json.write(sysGetConfig);

  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress gw = WiFi.gatewayIP();
  IPAddress dns = WiFi.dnsIP();
  const char* ssid = config_doc["wifi"]["ssid"] | "";

  json.write(
      PSTR(",\"wifi\":{\"sta\":{\"ssid\":\"%s\",\"is_open\":false,\"enable\":true,\"ipv4mode\":\"dhcp\","
           "\"ip\":\"%u.%u.%u.%u\",\"netmask\":\"%u.%u.%u.%u\","
           "\"gw\":\"%u.%u.%u.%u\",\"nameserver\":\"%u.%u.%u.%u\"},"
           "\"ws\":{\"enable\":false,\"server\":null,\"ssl_ca\":\"ca.pem\"}}}"),
      ssid,
      (unsigned int)ip[0], (unsigned int)ip[1], (unsigned int)ip[2], (unsigned int)ip[3],
      (unsigned int)mask[0], (unsigned int)mask[1], (unsigned int)mask[2], (unsigned int)mask[3],
      (unsigned int)gw[0], (unsigned int)gw[1], (unsigned int)gw[2], (unsigned int)gw[3],
      (unsigned int)dns[0], (unsigned int)dns[1], (unsigned int)dns[2], (unsigned int)dns[3]
  );

  blinkled(ledblinkduration);
  return json.pos;
}

// aligned with Shelly API docs
size_t shellyComponents(char* buffer, size_t maxLen) {
  JsonWriter json(buffer, maxLen);

  json.write(PSTR("{\"components\":[{\"key\":\"em:0\",\"status\":"));
  json.write(EMGetStatus);
  json.write(PSTR(",\"config\":"));
  json.write(EMGetConfig);
  json.write(PSTR("},{\"key\":\"emdata:0\",\"status\":"));
  json.write(EMDataGetStatus);
  json.write(PSTR(",\"config\":{}}],\"cfg_rev\":1,\"offset\":0,\"total\":2}"));

  blinkled(ledblinkduration);
  return json.pos;
}

// aligned with Shelly API docs
size_t shellyGetStatus(char* buffer, size_t maxLen) {
  JsonWriter json(buffer, maxLen);

  double temperature = 26.55;
#ifdef ESP32
  temperature = temperatureRead();
#endif

  json.write(PSTR("{\"ble\":{},\"bthome\":{},\"cloud\":{\"connected\":false},\"em:0\":"));
  json.write(EMGetStatus);
  json.write(PSTR(",\"emdata:0\":"));
  json.write(EMDataGetStatus);
  json.write(PSTR(",\"eth\":{\"ip\":null,\"ip6\":null},\"modbus\":{},\"mqtt\":{\"connected\":false},\"sys\":"));
  json.write(sysGetStatus);
  json.write(
      PSTR(",\"temperature:0\":{\"id\":0,\"tC\":%.2f,\"tF\":%.2f},\"wifi\":"),
      temperature,
      (temperature * 9.0 / 5.0) + 32.0
  );

  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    const uint8_t* bssid = WiFi.BSSID();
    uint8_t zeroBssid[6] = {0};

    if (bssid == nullptr) {
      bssid = zeroBssid;
    }

    const char* ssid = config_doc["wifi"]["ssid"] | "";

    json.write(
        PSTR("{\"sta_ip\":\"%u.%u.%u.%u\",\"status\":\"got ip\",\"ssid\":\"%s\","
             "\"bssid\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"rssi\":%d,\"sta_ip6\":[]}"),
        (unsigned int)ip[0], (unsigned int)ip[1], (unsigned int)ip[2], (unsigned int)ip[3],
        ssid,
        (unsigned int)bssid[0], (unsigned int)bssid[1], (unsigned int)bssid[2],
        (unsigned int)bssid[3], (unsigned int)bssid[4], (unsigned int)bssid[5],
        WiFi.RSSI()
    );
  } else {
    json.write(PSTR("{\"sta_ip\":null,\"status\":\"connecting\",\"ssid\":null,\"bssid\":null,\"rssi\":0,\"sta_ip6\":[]}"));
  }

  blinkled(ledblinkduration);
  return json.pos;
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
