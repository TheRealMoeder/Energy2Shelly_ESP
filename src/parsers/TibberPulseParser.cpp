#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include <sml.h>

// functions for TibberPulse
double tibber_consumption = 0, tibber_production = 0, tibber_power = 0;

typedef struct {
  const unsigned char OBIS[6];
  void (*Handler)();
} OBISHandler;

// supports currently only:
// - consumption (OBIS 1-0:1.8.0)
// - production  (OBIS 1-0:2.8.0)
// - power       (OBIS 1-0:16.7.0)
// this could be extended later if needed for other OBIS codes (at least those used in TibberPulse SML)
void Consumption() { smlOBISWh(tibber_consumption); }
void Production() { smlOBISWh(tibber_production); }
void Power() { smlOBISW(tibber_power); }

OBISHandler OBISHandlers[] = {
  {{0x01, 0x00, 0x01, 0x08, 0x00, 0xff}, &Consumption}, /* 1-0: 1. 8.0*255 (Consumption Total) */
  {{0x01, 0x00, 0x02, 0x08, 0x00, 0xff}, &Production},  /* 1-0: 2. 8.0*255 (Production Total) */
  {{0x01, 0x00, 0x10, 0x07, 0x00, 0xff}, &Power},       /* 1-0:16. 7.0*255 (power) */
  {{0, 0}}
};

// SML message length tested with following power meters:
enum {
  // EMH eHZB
  SML_PM_EMH_EHZB = 248,
  // eBZ DD3
  SML_PM_EBZ_DD3 = 396,
  SMLPAYLOADMAXSIZE = 500
};
byte smlpayload[SMLPAYLOADMAXSIZE]{0};

bool parseTibberPulse() {
  bool ret = true;
  int getlength = 0;
  DEBUG_SERIAL.print("Querying TibberPulse raw SML: ");
  String url = "http://";
  url += String(tibber_host_port);
  url += String(tibber_rpc);
  url += String(tibber_nodeid);
  DEBUG_SERIAL.printf("URL:%s, user:%s\r\n", url.c_str(), tibber_user);
  http.begin(wifi_client, url);
  http.setAuthorization(tibber_user, tibber_password);
  int httpResponseCode = http.GET();
  if (httpResponseCode > 0) {
    getlength = http.getSize();
    DEBUG_SERIAL.printf("Response message size=%d\r\n", getlength);
    if ((getlength > SMLPAYLOADMAXSIZE) || (getlength == 0)) {
      http.end();
      return false;
    }
    WiFiClient *w = http.getStreamPtr();
    w->readBytes(smlpayload, getlength);
    // the OBIS codes for consumption (1-0:1.8.0*255) and power (1-0:16.7.0*255) are the same,
    // the SML message length might be different, but reading these should still work
    if (getlength != SML_PM_EMH_EHZB && getlength != SML_PM_EBZ_DD3) {
      DEBUG_SERIAL.printf("ERROR: SML data not in expected length! length=%d \r\n", getlength);
      // for extra debugging
      for (int i = 0; i < getlength; i++) {
          DEBUG_SERIAL.printf("%02xh ", smlpayload[i]);
      }
      DEBUG_SERIAL.println();
      ret = false;
    } else {
      int i = 0, iHandler = 0;
      unsigned char c;
      sml_states_t s;
      for (i = 0; i < getlength; ++i) {
        c = smlpayload[i];
        s = smlState(c);
        switch (s) {
        case SML_START:
          /* reset local vars */
          tibber_consumption = 0;
          tibber_production = 0;
          tibber_power = 0;
          break;
        case SML_LISTEND:
          for (
            iHandler = 0;
            OBISHandlers[iHandler].Handler != 0 && !(smlOBISCheck(OBISHandlers[iHandler].OBIS));
            iHandler++
          );
          if (OBISHandlers[iHandler].Handler != 0) {
            OBISHandlers[iHandler].Handler();
          }
          break;
        case SML_UNEXPECTED:
          DEBUG_SERIAL.printf(">>> Unexpected byte >%02X<! <<<\n", smlpayload[i]);
          break;
        case SML_FINAL:
          setEnergyData(tibber_consumption, tibber_production); // input and output energy from TibberPulse
          setPowerData(tibber_power);
          ret = true;
          break;
        default:
          break;
        }
      }
    }
  } else {
      DEBUG_SERIAL.printf("HTTP request failed, error code: %d\n", httpResponseCode);
      ret = false;
  }
  // Free resources
  http.end();
  return ret;
}
