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

// SML message length tested with following power meters:
enum
{
  // EMH eHZB
  SML_PM_EMH_EHZB = 248,
  // eBZ DD3
  SML_PM_EBZ_DD3 = 396,
  // Iskra MT631
  SML_PM_MT631 = 236,
  // EasyMeter ESY11
  SML_PM_ESY11 = 476,
  // EMH EHZ
  SML_PM_EMH_EHZ = 448,
  SML_PM_EBZDD3_B = 408,
  // Landis + Gyr E320
  SML_PM_LG_LE320 = 276,
  SMLPAYLOADMAXSIZE = 500
};
byte smlpayload[SMLPAYLOADMAXSIZE]{0};

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
  bool ret = true;
  int getlength = 0;
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
    getlength = http.getSize();
    DEBUG_SERIAL.printf("Response message size=%d\r\n", getlength);
    if ((getlength > SMLPAYLOADMAXSIZE) || (getlength == 0))
    {
      http.end();
      TibberPulse_URL_guesser();
      return false;
    }
   
  size_t bytesRead = 0;
  WiFiClient *w = http.getStreamPtr();
    if (getlength > 0) 
    {
      //size is known
      bytesRead = w->readBytes(smlpayload, getlength);
    } 
    else 
    {
      // unkown size
      unsigned long timeout = millis();
      
      // keep polling until exit condition met
      while (http.connected() && (w->available() || w->peek() != -1)) 
      {
        if (w->available()) 
        {
          // Buffer Overflow Protection
          if (bytesRead >= SMLPAYLOADMAXSIZE) 
          {
            DEBUG_SERIAL.println(F("Error: Payload exceeds SMLPAYLOADMAXSIZE!"));
            http.end();
            TibberPulse_URL_guesser();
            return false;
          }
          
          smlpayload[bytesRead] = w->read();
          bytesRead++;
          timeout = millis(); // reset timeout, if data is comming in.
        }
        
        // Abort in case of inactivity
        if (millis() - timeout > 3000) 
        {
          DEBUG_SERIAL.println(F("Error: Timeout during reading stream."));
          break;
        }
        
        delay(1); 
      }
      
    }
    getlength=bytesRead;
    // read some data?
    if (bytesRead == 0)
    {
      DEBUG_SERIAL.println(F("Error: No data received"));
      http.end();
      TibberPulse_URL_guesser();
      return false;
    }

    DEBUG_SERIAL.printf("successful %d bytes in smlpayload.\r\n", bytesRead);
  
       // the OBIS codes for consumption (1-0:1.8.0*255) and power (1-0:16.7.0*255) are the same,
    // the SML message length might be different, but reading these should still work
    //if (getlength != SML_PM_EMH_EHZB && getlength != SML_PM_EBZ_DD3 && getlength != SML_PM_MT631 && getlength != SML_PM_ESY11 && getlength != SML_PM_EMH_EHZ && getlength != SML_PM_LG_LE320)
    if (getlength<100)
    {
      DEBUG_SERIAL.printf("ERROR: SML data not in expected length! length=%d \r\n", getlength);
      // for extra debugging
      for (int i = 0; i < getlength; i++)
      {
        DEBUG_SERIAL.printf("%02xh ", smlpayload[i]);
      }
      DEBUG_SERIAL.println();
      TibberPulse_URL_guesser();
      ret = false;
    }
    else
    {
      success_counter = 10;
      int i = 0, iHandler = 0;
      unsigned char c;
      sml_states_t s;
      for (i = 0; i < getlength; ++i)
      {
        c = smlpayload[i];
        s = smlState(c);
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
          DEBUG_SERIAL.printf(">>> Unexpected byte >%02X<! <<<\n", smlpayload[i]);
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
          ret = true;
          break;
        default:
          break;
        }
      }
    }
  }
  else
  {
    TibberPulse_URL_guesser();
    DEBUG_SERIAL.print(F("HTTP request failed, error code:"));
    DEBUG_SERIAL.println(httpResponseCode);

    ret = false;
  }
  // Free resources
  http.end();
  return ret;
}
