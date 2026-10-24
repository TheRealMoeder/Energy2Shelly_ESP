#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"
#include "../utils/utils.h"
#ifdef ESP32
#include <WiFiClientSecure.h>
static WiFiClientSecure ona_client;

struct MeasurementData {
    char timeStr[25]; // Platz für ISO8601 String, z.B. "2026-10-24T16:00:00Z"
    double latitude;
    double longitude;
    float l1_v;
    float l2_v;
    float l3_v;
    float frequency;
};

QueueHandle_t onaQueue = NULL;
const int ONA_QUEUE_SIZE = 1;

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


String ona_lastResponse = "";
int ona_lastResponseCode = 0;
volatile bool ona_responseReady = false; 

volatile TaskHandle_t onaTaskHandle = NULL;
void ona_sendMeasurementTask(void * pvParameters) {
    HTTPClient http; 
    MeasurementData data; // Lokaler Speicher für die empfangenen Daten

    while (true) {
        // Wartet blockierungsfrei, bis eine neue Struktur reinkommt
        if (xQueueReceive(onaQueue, &data, portMAX_DELAY) == pdPASS) {
            
            // Erst hier im Hintergrund-Task bauen wir das JSON
            JsonDocument doc;
            doc["observed_at"] = data.timeStr;
            doc["latitude"] = data.latitude;
            doc["longitude"] = data.longitude;
            doc["l1_v"] = data.l1_v;
            doc["l2_v"] = data.l2_v;
            doc["l3_v"] = data.l3_v;
            doc["smartmeter_model"] = "Energy2Shelly";
            doc["integration_version"] = VERSION_BUILD;
            doc["grid_frequency_hz"] = data.frequency;

            String requestBody;
            serializeJson(doc, requestBody);

            // HTTP POST absenden
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
        }
    }
}

void ona_Setup()
{
    ona_client.setCACert(ona_root_ca);
    onaQueue = xQueueCreate(ONA_QUEUE_SIZE, sizeof(MeasurementData));
  
    xTaskCreate(ona_sendMeasurementTask, "OnaHTTPTask", 8192, NULL, 1, NULL);
}


void ona_sendMeasurement() {
    DEBUG_SERIAL.println("Ona");

    if (phase_number != 3) {
        DEBUG_SERIAL.println(F("Ona: Phase number must be 3"));
        return; 
    }
    if (ona_responseReady) {
        ona_responseReady=false;
        DEBUG_SERIAL.println(ona_lastResponse);
    }
   
    MeasurementData data;
    
    
    strncpy(data.timeStr, getISO8601Time().c_str(), sizeof(data.timeStr) - 1);
    data.timeStr[sizeof(data.timeStr) - 1] = '\0'; 

    data.latitude = config_doc["ona"]["latitude"].as<double>();
    data.longitude = config_doc["ona"]["longitude"].as<double>();
    data.l1_v = PhasePower[0].voltage;
    data.l2_v = PhasePower[1].voltage;
    data.l3_v = PhasePower[2].voltage;
    data.frequency = PhasePower[0].frequency;

    
    if (xQueueSend(onaQueue, &data, 0) != pdPASS) {
        DEBUG_SERIAL.println("[Ona] Queue full");
    } 
}
#endif