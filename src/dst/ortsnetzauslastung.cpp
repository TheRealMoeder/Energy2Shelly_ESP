#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"
#include "../utils/utils.h"
#ifdef ESP32
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

String ona_lastResponse = "";
int ona_lastResponseCode = 0;
volatile bool ona_responseReady = false; 

volatile TaskHandle_t onaTaskHandle = NULL;
void ona_sendMeasurementTask(void * pvParameters) {
    String *requestBodyPtr = (String *)pvParameters;
    String requestBody = *requestBodyPtr;
    delete requestBodyPtr;

    HTTPClient http;
    http.begin(ona_client, api_url); 
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(requestBody);

    ona_lastResponseCode = httpResponseCode;
    if (httpResponseCode > 0) {
        ona_lastResponse = http.getString();
    } else {
        ona_lastResponse = http.errorToString(httpResponseCode);
    }
    http.end();

    ona_responseReady = true;

    onaTaskHandle = NULL; 
    vTaskDelete(NULL);
}

void ona_sendMeasurement() {
    DEBUG_SERIAL.println("Ona");

    if (onaTaskHandle != NULL) {
        DEBUG_SERIAL.println("[Ona] Still processing");
        return; 
    }

    if (phase_number != 3) {
        DEBUG_SERIAL.println(F("Ona: Phase number must be 3"));
    }
    DEBUG_SERIAL.print("Ona, last answer:");
    DEBUG_SERIAL.println(ona_lastResponse);

    JsonDocument doc;
    doc["observed_at"] = getISO8601Time();
    doc["latitude"] = config_doc["ona"]["latitude"].as<double>();
    doc["longitude"] = config_doc["ona"]["longitude"].as<double>();
    doc["l1_v"] = PhasePower[0].voltage;
    doc["l2_v"] = PhasePower[1].voltage;
    doc["l3_v"] = PhasePower[2].voltage;
    doc["smartmeter_model"] = "Energy2Shelly";
    doc["integration_version"] = VERSION_BUILD;
    doc["grid_frequency_hz"] = PhasePower[0].frequency;
    
    String *requestBody = new String();
    serializeJson(doc, *requestBody);
    DEBUG_SERIAL.println(*requestBody);

    xTaskCreatePinnedToCore(
        ona_sendMeasurementTask,   
        "OnaSendTask",             
        16384,                      
        (void *)requestBody,       
        1,                         
        NULL,                      
        0                          
    );

}

#endif