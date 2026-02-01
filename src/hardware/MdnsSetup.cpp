#include "../config/Configuration.h"

// Set up mDNS responder with Shelly service registration
void setupMdns() {
  strncat(shelly_name, shelly_mac,
          sizeof(shelly_name) - strlen(shelly_name) - 1);

  if (!MDNS.begin(shelly_name)) {
    DEBUG_SERIAL.println("Error setting up MDNS responder!");
  }

#ifdef ESP32
  // ESP32 mDNS setup
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("shelly", "tcp", 80);
  mdns_txt_item_t serviceTxtData[4] = {{"fw_id", shelly_fw_id},
                                       {"arch", "esp8266"},
                                       {"id", shelly_name},
                                       {"gen", shelly_gen}};
  mdns_service_instance_name_set("_http", "_tcp", shelly_name);
  mdns_service_txt_set("_http", "_tcp", serviceTxtData, 4);
  mdns_service_instance_name_set("_shelly", "_tcp", shelly_name);
  mdns_service_txt_set("_shelly", "_tcp", serviceTxtData, 4);
#else
  // ESP8266 mDNS setup
  hMDNSService = MDNS.addService(0, "http", "tcp", 80);
  hMDNSService2 = MDNS.addService(0, "shelly", "tcp", 80);
  if (hMDNSService) {
    MDNS.setServiceName(hMDNSService, shelly_name);
    MDNS.addServiceTxt(hMDNSService, "fw_id", shelly_fw_id);
    MDNS.addServiceTxt(hMDNSService, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService, "id", shelly_name);
    MDNS.addServiceTxt(hMDNSService, "gen", shelly_gen);
  }
  if (hMDNSService2) {
    MDNS.setServiceName(hMDNSService2, shelly_name);
    MDNS.addServiceTxt(hMDNSService2, "fw_id", shelly_fw_id);
    MDNS.addServiceTxt(hMDNSService2, "arch", "esp8266");
    MDNS.addServiceTxt(hMDNSService2, "id", shelly_name);
    MDNS.addServiceTxt(hMDNSService2, "gen", shelly_gen);
  }
#endif
  DEBUG_SERIAL.println("mDNS responder started");
}
