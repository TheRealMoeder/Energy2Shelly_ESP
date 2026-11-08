#include "RpcHandlers.h"
#include "../config/Configuration.h"
#include "../data/DataStructures.h"
#include "RpcComm.h"

void scriptGetCode() {
  JsonDocument jsonResponse;
  jsonResponse["data"] = "";
  jsonResponse["left"] = "0";
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("scriptGetCode: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

void scriptList() {
  JsonDocument jsonResponse;
  jsonResponse["scripts"].to<JsonArray>();
  serializeJson(jsonResponse, serJsonResponse);
  DEBUG_SERIAL.print("scriptList: ");
  DEBUG_SERIAL.println(serJsonResponse);
  blinkled(ledblinkduration);
}

void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  JsonDocument json;
  switch (type) {
    case WS_EVT_DISCONNECT:
      DEBUG_SERIAL.printf("[%u] Websocket: disconnected!\n", (unsigned int)client->id());
      break;
    case WS_EVT_CONNECT:
      DEBUG_SERIAL.printf("[%u] Websocket: connected from %s\n", (unsigned int)client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DATA:
      {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
          data[len] = 0;
          deserializeJson(json, data);
          rpcId = json["id"];
          if (json["method"] == "Shelly.GetDeviceInfo") {
            strcpy(rpcUser, "EMPTY");
            shellyGetDeviceInfo();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Shelly.GetComponents") {
            strcpy(rpcUser, "EMPTY");
            shellyGetComponents();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Shelly.GetConfig") {
            strcpy(rpcUser, "EMPTY");
            shellyGetConfig();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Shelly.GetStatus") {
            strcpy(rpcUser, "EMPTY");
            shellyGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Shelly.ListMethods") {
            strcpy(rpcUser, "EMPTY");
            shellyListMethods();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "EM.GetStatus") {
            strcpy(rpcUser, json["src"]);
            EMGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "EMData.GetStatus") {
            strcpy(rpcUser, json["src"]);
            EMDataGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "EM.GetConfig") {
            EMGetConfig();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Script.GetCode") {
            scriptGetCode();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "Script.List") {
            scriptList();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else if (json["method"] == "WiFi.GetStatus") {
            wifiGetStatus();
            rpcWrapper();
            webSocket.textAll(serJsonResponse);
          } else {
            DEBUG_SERIAL.printf("Websocket: unknown request: %s\n", data);
          }
        }
        break;
      }
    case WS_EVT_PING:
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// Push status notifications to connected websocket clients like a real Shelly Pro 3EM.
// Home Assistant only polls once on connect and relies on these notifications afterwards.
// Event driven: sent as soon as a data source has written new values. The minimum interval
// (NOTIFY_STATUS_MIN_INTERVAL) merges updates that arrive in several parts (e.g. one MQTT message per phase).
// Uses its own buffer instead of serJsonResponse, which is written by the HTTP/websocket handlers in the async_tcp task.
void notifyStatusWebSocket() {
  static PowerData lastPower[3];
  static EnergyData lastEnergy[3];
  static unsigned long lastNotify = 0;
  if (webSocket.count() == 0 || millis() - lastNotify < NOTIFY_STATUS_MIN_INTERVAL) {
    return;
  }
  bool powerChanged = memcmp(lastPower, PhasePower, sizeof(lastPower)) != 0;
  bool energyChanged = memcmp(lastEnergy, PhaseEnergy, sizeof(lastEnergy)) != 0;
  if (!powerChanged && !energyChanged) {
    return;
  }
  if (!webSocket.availableForWriteAll()) {
    return; // a client is not consuming its queue, try again on the next pass
  }
  lastNotify = millis();
  memcpy(lastPower, PhasePower, sizeof(lastPower));
  memcpy(lastEnergy, PhaseEnergy, sizeof(lastEnergy));

  char message[1024];
  size_t len = snprintf(message, sizeof(message),
      "{\"src\":\"%s\",\"method\":\"NotifyStatus\",\"params\":{\"ts\":%ld",
      shelly_name.c_str(), (long)time(nullptr));

  if (powerChanged) {
    char em[512];
    if (writeEMStatus(em, sizeof(em))) {
      len += snprintf(message + len, sizeof(message) - len, ",\"em:0\":%s", em);
    }
  }
  if (energyChanged) {
    char emdata[384];
    if (writeEMDataStatus(emdata, sizeof(emdata))) {
      len += snprintf(message + len, sizeof(message) - len, ",\"emdata:0\":%s", emdata);
    }
  }
  snprintf(message + len, sizeof(message) - len, "}}");

  DEBUG_SERIAL.print(F("NotifyStatus: "));
  DEBUG_SERIAL.println(message);
  webSocket.textAll(message);
}

void parseUdpRPC() {
  uint8_t buffer[1024];
  int packetSize = UdpRPC.parsePacket();
  if (packetSize) {
    JsonDocument json;
    int rSize = UdpRPC.read(buffer, 1024);
    buffer[rSize] = 0;
    DEBUG_SERIAL.print(F("Received UDP packet on port 1010: "));
    DEBUG_SERIAL.println((char *)buffer);
    deserializeJson(json, buffer);
    if (json["method"].is<JsonVariant>()) {
      rpcId = json["id"];
      strcpy(rpcUser, "EMPTY");
      UdpRPC.beginPacket(UdpRPC.remoteIP(), UdpRPC.remotePort());
      if (json["method"] == "Shelly.GetDeviceInfo") {
        shellyGetDeviceInfo();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "Shelly.GetComponents") {
        shellyGetComponents();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "Shelly.GetConfig") {
        shellyGetConfig();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "Shelly.GetStatus") {
        shellyGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "Shelly.ListMethods") {
        shellyListMethods();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "EM.GetStatus") {
        EMGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "EMData.GetStatus") {
        EMDataGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (json["method"] == "EM.GetConfig") {
        EMGetConfig();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else {
        DEBUG_SERIAL.printf("RPC over UDP: unknown request: %s\n", buffer);
      }
      UdpRPC.endPacket();
    }
  }
}

void parseHttpRPC(String requestBody, AsyncWebServerRequest *request) {
  if (request && requestBody) {
    JsonDocument json;
    DEBUG_SERIAL.print(F("Received HTTP RPC request: "));
    DEBUG_SERIAL.println(requestBody);
    deserializeJson(json, requestBody);
    if (json["method"].is<JsonVariant>()) {
      rpcId = json["id"];
      // strcpy(rpcUser, "EMPTY");
      if (json["method"] == "Shelly.GetDeviceInfo") {
        shellyGetDeviceInfo();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "Shelly.GetComponents") {
        shellyGetComponents();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "Shelly.GetConfig") {
        shellyGetConfig();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "Shelly.GetStatus") {
        shellyGetStatus();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "Shelly.ListMethods") {
        shellyListMethods();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "EM.GetStatus") {
        EMGetStatus();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "EMData.GetStatus") {
        EMDataGetStatus();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else if (json["method"] == "EM.GetConfig") {
        EMGetConfig();
        rpcWrapper();
        request->send(200, "application/json", serJsonResponse);
      } else {
        DEBUG_SERIAL.printf("RPC over HTTP: unknown request: %s\n", requestBody.c_str());
      }
    }
  }
}
