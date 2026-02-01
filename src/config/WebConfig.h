#ifndef WEB_CONFIG_H
#define WEB_CONFIG_H

#include <Arduino.h>

// Forward declarations to avoid including ESPAsyncWebServer in header
class AsyncWebServerRequest;

// Web configuration page template processor
String processor(const String &var);

// HTTP request handlers
void handleConfig(AsyncWebServerRequest *request);
void handleSave(AsyncWebServerRequest *request);

#endif // WEB_CONFIG_H
