#ifndef CSRF_PROTECTION_H
#define CSRF_PROTECTION_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

// ============================================================================
// CSRF PROTECTION HELPER
// ============================================================================
// Validates that HTTP requests originated from the device itself by checking
// the Referer and Origin headers against the device's IP address.
// This protects against Cross-Site Request Forgery attacks.
// ============================================================================

inline bool validateCsrfToken(AsyncWebServerRequest *request) {
  String deviceUrl = "http://" + WiFi.localIP().toString();

  // Check Referer header first
  if (request->hasHeader("Referer")) {
    String referer = request->header("Referer");
    if (referer.startsWith(deviceUrl)) {
      return true;
    }
  }

  // Fallback: check Origin header
  if (request->hasHeader("Origin")) {
    String origin = request->header("Origin");
    if (origin == deviceUrl) {
      return true;
    }
  }

  return false;
}

#endif // CSRF_PROTECTION_H
