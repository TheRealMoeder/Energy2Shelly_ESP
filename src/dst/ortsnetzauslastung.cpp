#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"
#include "../utils/utils.h"

#include <WiFiClientSecure.h>
static WiFiClientSecure ona_client;

const char* api_url = "https://www.ortsnetz-auslastung.de/v1/measurements";
const double LATITUDE = 52.520008;
const double LONGITUDE = 13.404954;

const char* ona_root_ca = 
"-----BEGIN CERTIFICATE-----\n"
"MIICGzCCAaGgAwIBAgIQQdKd0XLq7qeAwSxs6S+HUjAKBggqhkjOPQQDAzBPMQsw\n"
"CQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFyY2gg\n"
"R3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMjAeFw0yMDA5MDQwMDAwMDBaFw00\n"
"MDA5MTcxNjAwMDBaME8xCzAJBgNVBAYTAlVTMSkwJwYDVQQKEyBJbnRlcm5ldCBT\n"
"ZWN1cml0eSBSZXNlYXJjaCBHcm91cDEVMBMGA1UEAxMMSVNSRyBSb290IFgyMHYw\n"
"EAYHKoZIzj0CAQYFK4EEACIDYgAEzZvVn4CDCuwJSvMWSj5cz3es3mcFDR0HttwW\n"
"+1qLFNvicWDEukWVEYmO6gbf9yoWHKS5xcUy4APgHoIYOIvXRdgKam7mAHf7AlF9\n"
"ItgKbppbd9/w+kHsOdx1ymgHDB/qo0IwQDAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0T\n"
"AQH/BAUwAwEB/zAdBgNVHQ4EFgQUfEKWrt5LSDv6kviejM9ti6lyN5UwCgYIKoZI\n"
"zj0EAwMDaAAwZQIwe3lORlCEwkSHRhtFcP9Ymd70/aTSVaYgLXTWNLxBo1BfASdW\n"
"tL4ndQavEi51mI38AjEAi/V3bNTIZargCyzuFJ0nN6T5U6VR5CmD1/iQMVtCnwr1\n"
"/q4AaOeMSQ+2b1tbFfLn\n"
"-----END CERTIFICATE-----\n";

void ona_Setup()
{
     ona_client.setCACert(ona_root_ca);
}


void ona_sendMeasurement() {


    if  (phase_number!=3) {
        DEBUG_SERIAL.println(F("Ortsnetzauslastung: Phase number must be 3"));
    }

    HTTPClient http;
    http.begin(ona_client,api_url); 
    http.addHeader("Content-Type", "application/json");

    JsonDocument doc;
    doc["observed_at"] = getISO8601Time();
    doc["latitude"] = LATITUDE;
    doc["longitude"] = LONGITUDE;
    doc["l1_v"] = PhasePower[0].voltage;
    doc["l2_v"] = PhasePower[1].voltage;
    doc["l3_v"] = PhasePower[2].voltage;
    doc["smartmeter_model"] = "Energy2Shelly";
    doc["integration_version"] = VERSION_BUILD;
        
    //if (frequency_hz > 0) {// check if source delivers frequency, else skip it
    //      doc["grid_frequency_hz"] = frequency_hz;
    // }
    
    String requestBody;
    serializeJson(doc, requestBody);

    int httpResponseCode = http.POST(requestBody);

    if (httpResponseCode > 0) {
        String response = http.getString();
        DEBUG_SERIAL.printf("[HTTP] Code: %d\n", httpResponseCode);
        DEBUG_SERIAL.println("[HTTP] Answer:");
        DEBUG_SERIAL.println(response);
    } else {
        DEBUG_SERIAL.printf("[HTTP] POST, Error: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    http.end();
    
   
}