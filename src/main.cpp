// Energy2Shelly_ESP v0.6.1
#include <Arduino.h>

// Configuration & setup
#include "config/Configuration.h"

// Data structures & processing
#include "data/DataStructures.h"
#include "data/DataProcessing.h"

// Protocol parsers
#include "parsers/Parsers.h" 

// RPC handlers
#include "rpc/RpcHandlers.h"
#include "rpc/RpcComm.h"

void setup(void) {
  DEBUG_SERIAL.begin(115200);
  WifiManagerSetup();

  // Initialize time via NTP
#ifdef ESP32
  configTime(0, 0, ntp_server);
  setenv("TZ", timezone, 1);
  tzset();
#else
  // ESP8266
  configTime(timezone, ntp_server);
#endif
  while (!getLocalTime(&timeinfo))
  {
    DEBUG_SERIAL.println("Waiting for NTP time...");
    delay(500);
  }
  DEBUG_SERIAL.print("Current time: ");
  char time_buffer[20];
  strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  DEBUG_SERIAL.println(time_buffer);

  if (String(led_gpio).toInt() > 0) {
    led = String(led_gpio).toInt();
  }

  if (led > 0) {
    pinMode(led, OUTPUT);
    if (led_i) {
      digitalWrite(led, LOW);
    } else {
      digitalWrite(led, HIGH);
    }
  }

  // Set up web server and endpoints

  server.on("/", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "This is the Energy2Shelly for ESP converter!\r\nDevice and Energy status is available under /status\r\nTo reset configuration, goto /reset\r\n");
  });

  server.on("/shelly", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/status", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/reset", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head><title>Reset Confirmation</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial,sans-serif;text-align:center;padding:20px;}";
    html += ".btn{padding:10px 20px;margin:10px;cursor:pointer;text-decoration:none;display:inline-block;border-radius:5px;font-size:16px;}";
    html += ".btn-yes{background-color:#d9534f;color:white;border:none;}";
    html += ".btn-no{background-color:#5bc0de;color:white;border:none;}</style></head><body>";
    html += "<h2>Reset Configuration?</h2>";
    html += "<p>Are you sure you want to reset the WiFi configuration? This will clear all settings and restart the device.</p>";
    html += "<form method='POST' action='/reset' style='display:inline;'>";
    html += "<button type='submit' class='btn btn-yes'>Yes, Reset</button>";
    html += "</form>";
    html += "<a href='/' class='btn btn-no'>Cancel</a>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/reset", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {
    shouldResetConfig = true;
    request->send(200, "text/plain", "Resetting WiFi configuration, please log back into the hotspot to reconfigure...\r\n");
  });

// Offset settings page
server.on("/offset", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Offset & Lastverteilung</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<style>body{font-family:Arial,sans-serif;text-align:center;padding:20px;}";
  html += "input{padding:8px;margin:5px;width:200px;font-size:16px;border-radius:5px;border:1px solid #ccc;}";
  html += "label{display:block;font-weight:bold;margin-top:15px;text-align:left;}";
  html += ".info{font-size:12px;color:#888;margin-bottom:5px;text-align:left;}";
  html += "h3{border-bottom:1px solid #ccc;padding-bottom:5px;}";
  html += ".btn{padding:10px 30px;margin-top:20px;cursor:pointer;background-color:#5bc0de;color:white;border:none;border-radius:5px;font-size:16px;}";
  html += ".box{background:#f9f9f9;border:1px solid #ddd;border-radius:8px;padding:15px;margin:15px auto;max-width:400px;text-align:left;}";
  html += "</style></head><body>";
  html += "<h2>Energy2Shelly Einstellungen</h2>";
  html += "<form method='POST' action='/offset'>";

  
  
  // Lastverteilung Sektion
  html += "<div class='box'><h3>🔋 Akku2 Steuerung</h3>";
  //html += "<label>Anteil dieser Akku (%)</label>";
  //html += "<div class='info'>z.B. 25 bei 2kWh Akku2 vs 6kWh Akku1 (0 = deaktiviert)</div>";
  //html += "<input type='number' step='1' min='0' max='100' name='akku2_anteil' value='" + String(akku2AnteilProzent) + "'>";
  //html += "<label>Hysterese (W)</label>";
  //html += "<div class='info'>Erst eingreifen wenn Abweichung größer als dieser Wert</div>";
  //html += "<input type='number' step='1' name='hysterese_watt' value='" + String(hysteresWatt) + "'>";
    
  html += "<label>Zielwert Akku2 (W)</label>";
  html += "<div class='info'>Akku2 soll immer diesen Wert liefern. 0 = wie Netzbezug</div>";
  html += "<div class='info'>-0.99 bis -0.01 = Anteilsmässige Ausgabe des Netzbezugs. -0,80 = 80% des Netzbezuges</div>";
  html += "<input type='number' step='0.01' min='-0.99' max='900' name='akku2_zielwatt' value='" + String(akku2Zielwatt) + "'>";
  
  html += "<label>Timeout (Sekunden)</label>";
  html += "<div class='info'>Nach dieser Zeit ohne /setextern Update wird Korrektur deaktiviert</div>";
  html += "<input type='number' step='1' name='extern_timeout' value='" + String(externTimeout) + "'>";
  html += "<div class='info'>ℹ️ Wenn Akku1 = 0 emfangen wird, bekomt Akku2 den vollen Netzbezug/Zählerwert zugewiesen (z.B. bei leerem Akku1 oder Sonderfälle)</div>";
  
  html += "<label>Obere Grenze Akku1 (W)</label>";
  html += "<div class='info'>Ueber diesem Wert hilft Akku2 mit (z.B. 700W)</div>";
  html += "<input type='number' step='1' min='0' name='akku1_og' value='" + String(akku1ObereGrenze) + "'>";

  html += "<label>Untere Grenze Akku1 (W)</label>";
  html += "<div class='info'>Unter diesem Wert uebernimmt Akku2 (z.B. 15W)</div>";
  html += "<input type='number' step='1' min='0' name='akku1_ug' value='" + String(akku1UntereGrenze) + "'>";

  html += "<label>Abweichung Grenze (W)</label>";
  html += "<div class='info'>Wenn Akku1 leer (oder Akku1 = 0 emfängt) und Netzbezug größer ist als dieser Wert liefert Akku2 vollen Netzbezug (Standard: 10W)</div>";
  html += "<input type='number' step='1' min='0' name='abweichung_gz' value='" + String(abweichungGrenze) + "'>";

  //html += "<label>Einspeisung Schwellwert (W)</label>";
  //html += "<div class='info'>Bei mehr Einspeisung als dieser Wert wird Akku2 nicht erhöht (z.B. -50)</div>";
  //html += "<input type='number' step='1' name='einspeisung_schwelle' value='" + String(einspeisungSchwelle) + "'>";
  html += "<label>Akku1 Leistung aktuell (W)</label>";
  html += "<div class='info'>Zuletzt von /setextern power1 empfangen (nur Anzeige)</div>";
  html += "<input type='number' disabled value='" + String(externAkku1Power) + "'>";
  html += "<label>Akku2 Leistung aktuell (W)</label>";
  html += "<div class='info'>Zuletzt von /setextern power2 empfangen (nur Anzeige)</div>";
  html += "<input type='number' disabled value='" + String(externAkku2Power) + "'>";
  html += "<label>Echter Netzbezug (W)</label>";
  html += "<div class='info'>Aktueller Wert vom Stromzähler (nur Anzeige)</div>";
  html += "<input type='number' disabled value='" + String(echteNetPower) + "'>";

  html += "<label>Korrigierter Netzbezug (W) (+5W Puffer)</label>";
  html += "<div class='info'>Berechneter Wert der an Akku2 gesendet wird (nur Anzeige)</div>";
  html += "<input type='number' disabled value='" + String(korrigierteNetPower) + "'>";
  html += "</div>";

  html += "<button type='submit' class='btn'>Speichern</button>";
  html += "</form>";
  html += "<br><a href='/status'>Status</a> | <a href='/'>Home</a>";
  html += "</body></html>";
  request->send(200, "text/html", html);
});

// Externer Leistungswerte von FHEM
server.on("/setextern", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
  if (request->hasParam("power1")) {
    externAkku1Power = request->getParam("power1")->value().toDouble();
    externLastUpdate = millis();
    DEBUG_SERIAL.print("Extern Akku1 Power: ");
    DEBUG_SERIAL.println(externAkku1Power);
    korrekturGesendet = false;
  }
  if (request->hasParam("power2")) {
    externAkku2Power = request->getParam("power2")->value().toDouble();
    DEBUG_SERIAL.print("Extern Akku2 Power: ");
    DEBUG_SERIAL.println(externAkku2Power);
  }
  if (request->hasParam("power1") || request->hasParam("power2")) {
    request->send(200, "text/plain", "OK");
  } else {
    request->send(400, "text/plain", "Parameter 'power1' oder 'power2' fehlt");
  }
});

server.on("/offset", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {
  preferences.begin("e2s_config", false);
  
  if (request->hasParam("akku2_anteil", true)) {
    String val = request->getParam("akku2_anteil", true)->value();
    strncpy(akku2_anteil, val.c_str(), sizeof(akku2_anteil));
    akku2AnteilProzent = atof(akku2_anteil);
    preferences.putString("akku2_anteil", akku2_anteil);
  }
  //if (request->hasParam("hysterese_watt", true)) {
  //  String val = request->getParam("hysterese_watt", true)->value();
  //  strncpy(hysterese_watt, val.c_str(), sizeof(hysterese_watt));
  //  hysteresWatt = atof(hysterese_watt);
  //  preferences.putString("hysterese_watt", hysterese_watt);
  //}
  if (request->hasParam("extern_timeout", true)) {
    String val = request->getParam("extern_timeout", true)->value();
    strncpy(extern_timeout, val.c_str(), sizeof(extern_timeout));
    externTimeout = atol(extern_timeout);
    preferences.putString("extern_timeout", extern_timeout);
  }
  //if (request->hasParam("einspeisung_schwelle", true)) {
  //  String val = request->getParam("einspeisung_schwelle", true)->value();
  //  strncpy(einspeisung_schwelle, val.c_str(), sizeof(einspeisung_schwelle));
  //  einspeisungSchwelle = atof(einspeisung_schwelle);
  //  preferences.putString("einspeisung_schwelle", einspeisung_schwelle);
  //}
  if (request->hasParam("akku2_zielwatt", true)) {
    String val = request->getParam("akku2_zielwatt", true)->value();
    strncpy(akku2_zielwatt, val.c_str(), sizeof(akku2_zielwatt));
    akku2Zielwatt = atof(akku2_zielwatt);
    preferences.putString("akku2_zielwatt", akku2_zielwatt);
  }
if (request->hasParam("akku1_og", true)) {
    String val = request->getParam("akku1_og", true)->value();
    strncpy(akku1_obere_grenze, val.c_str(), sizeof(akku1_obere_grenze));
    akku1ObereGrenze = atof(akku1_obere_grenze);
    preferences.putString("akku1_og", akku1_obere_grenze);
}
if (request->hasParam("akku1_ug", true)) {
    String val = request->getParam("akku1_ug", true)->value();
    strncpy(akku1_untere_grenze, val.c_str(), sizeof(akku1_untere_grenze));
    akku1UntereGrenze = atof(akku1_untere_grenze);
    preferences.putString("akku1_ug", akku1_untere_grenze);
}
if (request->hasParam("abweichung_gz", true)) {
    String val = request->getParam("abweichung_gz", true)->value();
    strncpy(abweichung_grenze, val.c_str(), sizeof(abweichung_grenze));
    abweichungGrenze = atof(abweichung_grenze);
    preferences.putString("abweichung_gz", abweichung_grenze);
}
  preferences.end();
  request->redirect("/offset");
});

  
  // Shelly RPC endpoints called via HTTP GET method
  server.on("/rpc/EM.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/EM.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    EMGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/EMData.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    EMDataGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/Shelly.GetComponents", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetComponents();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Shelly.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Shelly.GetDeviceInfo", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetDeviceInfo();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Shelly.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    shellyGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/Sys.GetConfig", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sysGetConfig();
    request->send(200, "application/json", serJsonResponse);
  });
  server.on("/rpc/Sys.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    sysGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  server.on("/rpc/WiFi.GetStatus", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    wifiGetStatus();
    request->send(200, "application/json", serJsonResponse);
  });

  // Shelly RPC endpoint called via HTTP POST method with JSON-RPC body
  server.on("/rpc", AsyncWebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String rpcRequestBuffer;
      if (index == 0) {
        // New request, clear buffer
        rpcRequestBuffer = "";
      }
      // Append incoming data chunk to buffer
      rpcRequestBuffer += String((char *)data).substring(0, len);
      if (index + len >= total) {
        // All data received, process RPC request
        parseHttpRPC(rpcRequestBuffer, request);
      }
    }
  );

  webSocket.onEvent(webSocketEvent);
  server.addHandler(&webSocket);
  server.begin();

  // Set up RPC over UDP for Marstek users
  UdpRPC.begin(String(shelly_port).toInt()); 

  // Set up MQTT
  if (dataMQTT) {
    mqtt_client.setBufferSize(2048);
    if (isValidIPAddress(mqtt_server)) {
      mqtt_client.setServer(mqtt_server, String(mqtt_port).toInt());
    } else {
      mqtt_client.setServer(mqtt_server, String(mqtt_port).toInt());
    }
    mqtt_client.setCallback(mqtt_callback);
  }

  // Set Up Multicast for SMA Energy Meter
  if (dataSMA) {
    Udp.begin(multicastPort);
#ifdef ESP8266
    Udp.beginMulticast(WiFi.localIP(), multicastIP, multicastPort);
#else
    Udp.beginMulticast(multicastIP, multicastPort);
#endif
  }

  // Set Up UDP for SHRDZM smart meter interface
  if (dataSHRDZM) {
    Udp.begin(multicastPort);
  }

  // Set Up Modbus TCP for SUNSPEC register query
  if (dataSUNSPEC) {
    modbus1.client();
    modbus_ip.fromString(mqtt_server);
    if (!modbus1.isConnected(modbus_ip)) {  // reuse mqtt server adresss for modbus adress
      Serial.println("Trying to connect SUNSPEC powermeter data");
      modbus1.connect(modbus_ip, String(mqtt_port).toInt());
    }
  }

  // Set Up HTTP query
  if (dataHTTP) {
    period = atol(query_period);
    http.useHTTP10(true);
  }

  // Set up mDNS responder
  setupMdns();

  startMillis = millis();
}

void loop() {
  currentMillis = millis();
#ifndef ESP32
  MDNS.update();
#endif
  parseUdpRPC();
  if (shouldResetConfig) {
#ifdef ESP32
    WiFi.disconnect(true, true);
#else
    WiFi.disconnect(true);
#endif
    delay(1000);
    ESP.restart();
  }
  if (dataMQTT) {
    if (!mqtt_client.connected()) {
      mqtt_reconnect();
    }
    mqtt_client.loop();
  }
  if (dataSMA) {
    parseSMA();
  }
  if (dataSHRDZM) {
    parseSHRDZM();
  }
  if (dataSUNSPEC) {
    if (currentMillis - startMillis >= period) {
      parseSUNSPEC();
      startMillis = currentMillis;
    }
   
  }
  if (dataHTTP) {
    if (currentMillis - startMillis >= period) {
      queryHTTP();
      startMillis = currentMillis;
    }
  }
  handleblinkled();
}
