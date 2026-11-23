#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"
#include "../utils/utils.h"
#include "ortsnetzauslastung.h"

#ifdef ONA_ACTIVE

#if defined(ESP8266)
    #include <ESP8266WiFi.h>
    #include <ESP8266HTTPClient.h>
    #include <WiFiClientSecure.h>
#elif defined(ESP32)
    #include <WiFiClientSecure.h>
    #include <HTTPClient.h>
#endif

struct MeasurementData {
    char timeStr[25]; 
    float l1_v;
    float l2_v;
    float l3_v;
    float frequency;
};


double ONA_latitude;
double ONA_longitude;

const char api_url[] PROGMEM = "https://www.ortsnetz-auslastung.de/v1/measurements";

const char ona_root_ca[] PROGMEM = R"=====(
-----BEGIN CERTIFICATE-----
MIICGzCCAaGgAwIBAgIQQdKd0XLq7qeAwSxs6S+HUjAKBggqhkjOPQQDAzBPMQsw
CQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFyY2gg
R3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMjAeFw0yMDA5MDQwMDAwMDBaFw00
MDA5MTcxNjAwMDBaME8xCzAJBgNVBAYTAlVTMSkwJwYDVQQKEyBJbnRlcm5ldCBT
ZWN1cml0eSBSZXNlYXJjaCBHcm91cDEVMBMGA1UEAxMMSVNSRyBSb290IFgyMHYw
EAYHKoZIzj0CAQYFK4EEACIDYgAEzZvVn4CDCuwJSvMWSj5cz3es3mcFDR0HttwW
+1qLFNvicWDEukWVEYmO6gbf9yoWHKS5xcUy4APgHoIYOIvXRdgKam7mAHf7AlF9
ItgKbppbd9/w+kHsOdx1ymgHDB/qo0IwQDAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0T
AQH/BAUwAwEB/zAdBgNVHQ4EFgQUfEKWrt5LSDv6kviejM9ti6lyN5UwCgYIKoZI
zj0EAwMDaAAwZQIwe3lORlCEwkSHRhtFcP9Ymd70/aTSVaYgLXTWNLxBo1BfASdW
tL4ndQavEi51mI38AjEAi/V3bNTIZargCyzuFJ0nN6T5U6VR5CmD1/iQMVtCnwr1
/q4AaOeMSQ+2b1tbFfLn
-----END CERTIFICATE-----
)=====";

String ona_lastResponse = "";
int ona_lastResponseCode = 0;
volatile bool ona_responseReady = false;

static void collectMeasurementData(MeasurementData &data) {
    memset(data.timeStr, 0, sizeof(data.timeStr));
    getISO8601Time(data.timeStr, sizeof(data.timeStr) - 1);
    data.l1_v = fabs(PhasePower[0].voltage);
    data.l2_v = fabs(PhasePower[1].voltage);
    data.l3_v = fabs(PhasePower[2].voltage);
    data.validFrequency = ONA_validfrequency;
    data.frequency = ONA_validfrequency ? PhasePower[0].frequency : 0.0f;
}

static int buildMeasurementPayload(const MeasurementData &data, char *buffer, size_t bufferSize) {
    char freqPart[40] = "";
    if (data.validFrequency) {
        snprintf_P(freqPart, sizeof(freqPart), PSTR(",\"grid_frequency_hz\":%.2f"), data.frequency);
    }
    return snprintf_P(buffer, bufferSize,
        PSTR("{\"observed_at\":\"%s\",\"latitude\":%.6f,\"longitude\":%.6f,\"l1_v\":%.2f,\"l2_v\":%.2f,\"l3_v\":%.2f,\"smartmeter_model\":\"Energy2Shelly\",\"integration_version\":\"%s\"%s}"),
        data.timeStr, ONA_latitude, ONA_longitude, data.l1_v, data.l2_v, data.l3_v, VERSION_BUILD, freqPart);
}

static void executeHttpPost(HTTPClient &http, WiFiClientSecure &client, const char *payload, size_t len, uint16_t timeoutMs = 8000) {
    if (!http.begin(client, FPSTR(api_url))) return;
    http.setTimeout(timeoutMs);
    client.setTimeout(timeoutMs);
    http.addHeader("Content-Type", "application/json");
    ona_lastResponseCode = http.POST((uint8_t *)payload, len);
    ona_lastResponse = (ona_lastResponseCode > 0) ? http.getString() : http.errorToString(ona_lastResponseCode);
    ona_responseReady = true;
    http.end();
}

#ifdef ESP32
static QueueHandle_t onaQueue = NULL;
static const int ONA_QUEUE_SIZE = 1;
static WiFiClientSecure ona_client;

void ONA_loop_state_machine() {}

void ONA_sendMeasurementTask(void *pvParameters) {
    HTTPClient http;
    MeasurementData data;
    char payload[256];
    for (;;) {
        if (xQueueReceive(onaQueue, &data, portMAX_DELAY) == pdPASS) {
            if (WiFi.status() == WL_CONNECTED) {
                int len = buildMeasurementPayload(data, payload, sizeof(payload));
                if (len > 0 && len < (int)sizeof(payload)) executeHttpPost(http, ona_client, payload, len);
            }
        }
    }
}
#endif

#ifdef ESP8266
static volatile bool ona_esp8266_has_data = false;
static MeasurementData ona_esp8266_data;
static BearSSL::Session ona_ssl_session;
static BearSSL::WiFiClientSecure ona_client;
static BearSSL::X509List cert(ona_root_ca);
static HTTPClient http_client;
static bool ona_setup_active = false;

void ONA_loop_state_machine() {
    if (!ona_setup_active || !ona_esp8266_has_data) return;
    if (WiFi.status() == WL_CONNECTED) {
        char payload[256];
        int len = buildMeasurementPayload(ona_esp8266_data, payload, sizeof(payload));
        if (len > 0 && len < (int)sizeof(payload)) executeHttpPost(http_client, ona_client, payload, len);
    }
    ona_esp8266_has_data = false;
}
#endif

bool ONA_sendMeasurement() {
    DEBUG_SERIAL.println(F("Ona"));
#ifdef ESP8266
    if (extendedMillis() < (300ull * 1000ull)) return false;
    if (!ona_setup_active) ONA_Setup();
#endif
    if (phase_number != 3 || !ONA_validVoltage) return false;
    if (ona_responseReady) {
        ona_responseReady = false;
        DEBUG_SERIAL.println(ona_lastResponse);
    }
#ifdef ESP32
    if (onaQueue == NULL) return false;
    MeasurementData data;
    collectMeasurementData(data);
    return (xQueueSend(onaQueue, &data, 0) == pdPASS);
#elif defined(ESP8266)
    if (ona_esp8266_has_data) return false;
    collectMeasurementData(ona_esp8266_data);
    ona_esp8266_has_data = true;
    return true;
#else
    return false;
#endif
}

void ONA_Setup() {
    ONA_latitude = config_doc["ona"]["latitude"].as<double>();
    ONA_longitude = config_doc["ona"]["longitude"].as<double>();
#ifdef ESP32
    ona_client.setCACert(ona_root_ca);
    onaQueue = xQueueCreate(ONA_QUEUE_SIZE, sizeof(MeasurementData));
    xTaskCreate(ONA_sendMeasurementTask, "ONAHTTPTask", 8192, NULL, 1, NULL);
#elif defined(ESP8266)
    ona_client.setSession(&ona_ssl_session);
    ona_client.setBufferSizes(1024, 512);
    ona_client.setTrustAnchors(&cert);
    ona_client.probeMaxFragmentLength("www.ortsnetz-auslastung.de", 443, 1024);
    ona_setup_active = true;
#endif
}

#endif // ONA_ACTIVE
