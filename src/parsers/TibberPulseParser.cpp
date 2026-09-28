#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include <sml.h>

// functions for TibberPulse
double tibber_consumption = 0, tibber_production = 0, tibber_power = 0;
double tibber_power_l1 = 0, tibber_power_l2 = 0, tibber_power_l3 = 0;

typedef struct
{
  const unsigned char OBIS[6];
  void (*Handler)();
} OBISHandler;

// supports currently only:
// - consumption (OBIS 1-0:1.8.0)
// - production  (OBIS 1-0:2.8.0)
// - power       (OBIS 1-0:16.7.0)
// - power L1    (OBIS 1-0:36.7.0)
// - power L2    (OBIS 1-0:56.7.0)
// - power L3    (OBIS 1-0:76.7.0)
void Consumption() { smlOBISWh(tibber_consumption); }
void Production() { smlOBISWh(tibber_production); }
void Power() { smlOBISW(tibber_power); }
void PowerL1() { smlOBISW(tibber_power_l1); }
void PowerL2() { smlOBISW(tibber_power_l2); }
void PowerL3() { smlOBISW(tibber_power_l3); }

OBISHandler OBISHandlers[] = {
    {{0x01, 0x00, 0x01, 0x08, 0x00, 0xff}, &Consumption}, /* 1-0: 1. 8.0*255 (Consumption Total) */
    {{0x01, 0x00, 0x02, 0x08, 0x00, 0xff}, &Production},  /* 1-0: 2. 8.0*255 (Production Total) */
    {{0x01, 0x00, 0x10, 0x07, 0x00, 0xff}, &Power},       /* 1-0:16. 7.0*255 (power) */
    {{0x01, 0x00, 0x24, 0x07, 0x00, 0xff}, &PowerL1},     /* 1-0:36. 7.0*255 (power L1) */
    {{0x01, 0x00, 0x38, 0x07, 0x00, 0xff}, &PowerL2},     /* 1-0:56. 7.0*255 (power L2) */
    {{0x01, 0x00, 0x4c, 0x07, 0x00, 0xff}, &PowerL3},     /* 1-0:76. 7.0*255 (power L3) */
    {{0, 0}}};

static uint8_t guess = 0;
static uint8_t success_counter = 0;

void TibberPulse_URL_guesser(void)
{
  if (success_counter > 0)
  { // keep old URL fetch some time
    success_counter--;
  }
  else
  { // try another one
    guess++;
    guess %= 2;
  }
}

bool parseTibberPulse()
{
  bool ret = false;
  DEBUG_SERIAL.print(F("Querying TibberPulse raw SML: "));
  String url = "http://";
  url += String(tibber_host);
  url += String(tibber_rpc[guess]);
  url += String(tibber_nodeid);
  DEBUG_SERIAL.printf("URL:%s, user:%s\r\n", url.c_str(), tibber_user);
  http.begin(wifi_client, url);
  http.setAuthorization(tibber_user, tibber_password);
  http.setTimeout(5000);
  int httpResponseCode = http.GET();
  if (httpResponseCode > 0)
  {

    WiFiClient *w = http.getStreamPtr();    
    int iHandler = 0;
    sml_states_t s;
    unsigned int counter=0;
    unsigned long timeout = millis();
    // keep polling until exit condition met
    while (http.connected() && (w->available() || w->peek() != -1))
    {
      if (w->available())
      {
        counter++;
        unsigned char val = w->read();
        if (success_counter<10) DEBUG_SERIAL.printf("%02x ",val);  // if SML has been successfully parsed do not print bytes.
        s = smlState(val);
        switch (s)
        {
        case SML_START:
          /* reset local vars */
          tibber_consumption = 0;
          tibber_production = 0;
          tibber_power = 0;
          tibber_power_l1 = 0;
          tibber_power_l2 = 0;
          tibber_power_l3 = 0;
          break;
        case SML_LISTEND:
          for (
              iHandler = 0;
              OBISHandlers[iHandler].Handler != 0 && !(smlOBISCheck(OBISHandlers[iHandler].OBIS));
              iHandler++)
            ;
          if (OBISHandlers[iHandler].Handler != 0)
          {
            OBISHandlers[iHandler].Handler();
          }
          break;
        case SML_UNEXPECTED:
          DEBUG_SERIAL.printf(">>> Unexpected byte >%02X<! <<<\n", val);
          break;
        case SML_FINAL:
          setEnergyData(tibber_consumption, tibber_production);
          if (tibber_power_l1 != 0 || tibber_power_l2 != 0 || tibber_power_l3 != 0)
          {
            setPowerData(tibber_power_l1, tibber_power_l2, tibber_power_l3);
          }
          else
          {
            setPowerData(tibber_power);
          }
          success_counter = 10;
          ret = true;
          break;
        default:
          break;
        }
        timeout = millis(); // reset timeout, if data is comming in.
      }
      else {
         delay(1);
      }

      // Abort in case of inactivity
      if (millis() - timeout > 3000)
      {
        DEBUG_SERIAL.println(F("Error: Timeout during reading stream."));
        break;
      }

     
    }
    DEBUG_SERIAL.print(F("\nSML, Number of bytes parsed: "));
    DEBUG_SERIAL.println(counter);
  }
  else
  {
   
    DEBUG_SERIAL.print(F("HTTP request failed, error code:"));
    DEBUG_SERIAL.println(httpResponseCode);

    ret = false;
  }
  if (ret==false)  TibberPulse_URL_guesser();
  // Free resources
  http.end();
  return ret;
}
