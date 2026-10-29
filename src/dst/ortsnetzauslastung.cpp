#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"
#include "../utils/utils.h"

#ifdef ONA_ACTIVE

#if defined(ESP8266)
    #include <ESP8266WiFi.h>
    #include <ESP8266HTTPClient.h>
    #include <WiFiClientSecure.h>
#elif defined(ESP32)
    #include <WiFiClientSecure.h>
    #include <HTTPClient.h>
#endif

#include <ArduinoJson.h>

struct MeasurementData {
    char timeStr[25]; 
    double latitude;
    double longitude;
    float l1_v;
    float l2_v;
    float l3_v;
    float frequency;
};


const char* api_url = "https://ortsnetz-auslastung.de";


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


#ifdef ESP32
QueueHandle_t onaQueue = NULL;
const int ONA_QUEUE_SIZE = 1;
static WiFiClientSecure ona_client;
volatile TaskHandle_t onaTaskHandle = NULL;

void ona_loop_state_machine() {
   
}

void ona_sendMeasurementTask(void * pvParameters) {
    HTTPClient http; 
    MeasurementData data; 
    JsonDocument doc;
    String requestBody;
    requestBody.reserve(512);

    for (;;) {
        
        if (xQueueReceive(onaQueue, &data, portMAX_DELAY) == pdPASS) {
            if (WiFi.status() == WL_CONNECTED) {
                doc.clear();
                doc["time"] = data.timeStr;
                doc["lat"] = config_doc["ona"]["latitude"].as<double>();
                doc["lon"] = config_doc["ona"]["longitude"].as<double>();
                doc["l1_v"] = data.l1_v;
                doc["l2_v"] = data.l2_v;
                doc["l3_v"] = data.l3_v;
                doc["freq"] = data.frequency;

                requestBody = "";
                serializeJson(doc, requestBody);

                if (http.begin(ona_client, api_url)) {
                    http.addHeader("Content-Type", "application/json");
                    
                    ona_lastResponseCode = http.POST(requestBody);
                    
                    if (ona_lastResponseCode > 0) {
                        ona_lastResponse = http.getString();
                    } else {
                        ona_lastResponse = http.errorToString(ona_lastResponseCode);
                    }
                    
                    ona_responseReady = true;
                    http.end();
                }
            }
        }
    }
}
#endif


#ifdef ESP8266
static MeasurementData ona_esp8266_buffer;
static volatile bool ona_esp8266_has_data = false;
static BearSSL::WiFiClientSecure ona_client;
static HTTPClient http_client;


static unsigned long ona_timeoutStart = 0;
const unsigned long ONA_TIMEOUT_MS = 5000;

enum ONA_State { 
    ONA_IDLE, 
    ONA_START_CONNECTION, 
    ONA_WAITING_FOR_RESPONSE 
};
static ONA_State ona_currentState = ONA_IDLE;

void ona_loop_state_machine() {
    static String requestBody;
    static JsonDocument doc;

    switch (ona_currentState) {
        case ONA_IDLE:
            if (ona_esp8266_has_data) {
                doc.clear();
                doc["time"] = ona_esp8266_buffer.timeStr;
               
                doc["lat"] = config_doc["ona"]["latitude"].as<double>();
                doc["lon"] = config_doc["ona"]["longitude"].as<double>();
                doc["l1_v"] = ona_esp8266_buffer.l1_v;
                doc["l2_v"] = ona_esp8266_buffer.l2_v;
                doc["l3_v"] = ona_esp8266_buffer.l3_v;
                doc["freq"] = ona_esp8266_buffer.frequency;

                requestBody = "";
                serializeJson(doc, requestBody);
                
                ona_currentState = ONA_START_CONNECTION;
            }
            break;

        case ONA_START_CONNECTION:
            if (WiFi.status() == WL_CONNECTED) {
              
                http_client.setTimeout(4000); 
                
                if (http_client.begin(ona_client, api_url)) {
                    http_client.addHeader("Content-Type", "application/json");
                    
                   
                    ona_timeoutStart = millis();
                    
                 
                    ona_lastResponseCode = http_client.POST(requestBody);
                    ona_currentState = ONA_WAITING_FOR_RESPONSE;
                } else {
                    ona_esp8266_has_data = false;
                    ona_currentState = ONA_IDLE;
                }
            }
            break;

        case ONA_WAITING_FOR_RESPONSE:
            if (ona_lastResponseCode != HTTPC_ERROR_CONNECTION_REFUSED) {
                if (ona_lastResponseCode > 0) {
                    ona_lastResponse = http_client.getString();
                } else {
                    ona_lastResponse = http_client.errorToString(ona_lastResponseCode);
                }
                
                ona_responseReady = true;
                ona_esp8266_has_data = false; 
                http_client.end();
                ona_currentState = ONA_IDLE;
            }
           
            else if (millis() - ona_timeoutStart > ONA_TIMEOUT_MS) {
                ona_lastResponseCode = -11;
                ona_lastResponse = "Software Timeout - No Server Response";
                ona_responseReady = true;
                
                ona_esp8266_has_data = false; 
                http_client.end();
                ona_currentState = ONA_IDLE;
            }
            break;
    }
}
#endif

bool ona_sendMeasurement() {
    MeasurementData data;
    
    strncpy(data.timeStr, getISO8601Time().c_str(), sizeof(data.timeStr) - 1);
    data.timeStr[sizeof(data.timeStr) - 1] = '\0'; 

    data.latitude = config_doc["ona"]["latitude"].as<double>();
    data.longitude = config_doc["ona"]["longitude"].as<double>();
    data.l1_v = PhasePower[0].voltage;
    data.l2_v = PhasePower[1].voltage;
    data.l3_v = PhasePower[2].voltage;
    data.frequency = PhasePower[0].frequency;

#ifdef ESP32
    if (onaQueue == NULL) return false;
    
    return (xQueueSend(onaQueue, &data, 0) == pdPASS);
#elif defined(ESP8266)
    if (ona_esp8266_has_data) return false; 
    ona_esp8266_buffer = data;
    ona_esp8266_has_data = true;
    return true;
#else
    return false;
#endif
}


void ona_Setup() {
#ifdef ESP32
    ona_client.setCACert(ona_root_ca);
    onaQueue = xQueueCreate(ONA_QUEUE_SIZE, sizeof(MeasurementData));
    
    xTaskCreatePinnedToCore(
        ona_sendMeasurementTask,
        "ONA_SendTask",
        4096,
        NULL,
        1,
        (TaskHandle_t*)&onaTaskHandle,
        1
    );
#elif defined(ESP8266)
    ona_client.setBufferSizes(512, 512);
    ona_client.setInsecure();
#endif
}

#endif // ONA_ACTIVE
