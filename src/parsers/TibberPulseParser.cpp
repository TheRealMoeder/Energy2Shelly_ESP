#include "../config/Configuration.h"
#include "SmlCommon.h"

#define HTTP_TIMEOUT 5000
#define INACTIVITY_READ_TIMEOUT 3000

static uint8_t guess = 0;
static uint8_t success_counter = 0;

const char* tibber_rpc[] = {"/data.json?node_id=", "/node_data.json?node_id="}; // fixed rpc path

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
  char url[128];
  snprintf_P(url, sizeof(url), PSTR("http://%s%s%u"), tibber_host, tibber_rpc[guess], tibber_nodeid);
  DEBUG_SERIAL.printf("URL:%s, user:%s\r\n", url, tibber_user);
  http.begin(wifi_client, url);
  http.setAuthorization(tibber_user, tibber_password);
  http.setTimeout(HTTP_TIMEOUT);
  int httpResponseCode = http.GET();
  if (httpResponseCode > 0)
  {

    WiFiClient *w = http.getStreamPtr();    
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
        s = smlParseByte(val);
        if (s == SML_UNEXPECTED)
        {
          DEBUG_SERIAL.printf(">>> Unexpected byte >%02X<! <<<\n", val);
        }
        else if (s == SML_FINAL)
        {
          success_counter = 10;
          ret = true;
        }
        timeout = millis(); // reset timeout, if data is comming in.
      }
      else {
         delay(1);
      }

      // Abort in case of inactivity
      if (millis() - timeout > INACTIVITY_READ_TIMEOUT)
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
