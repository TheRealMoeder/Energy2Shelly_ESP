#include "RpcHandlers.h"
#include "../config/Configuration.h"
#include "../data/DataStructures.h"

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
      DEBUG_SERIAL.printf("[%u] Websocket: disconnected!\n", client->id());
      break;
    case WS_EVT_CONNECT:
      DEBUG_SERIAL.printf("[%u] Websocket: connected from %s\n", client->id(), client->remoteIP().toString().c_str());
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

void parseUdpRPC() {
  uint8_t buffer[1024];
  int packetSize = UdpRPC.parsePacket();
  if (packetSize) {
    JsonDocument json;
    int rSize = UdpRPC.read(buffer, 1024);
    buffer[rSize] = 0;
    DEBUG_SERIAL.print("Received UDP packet on port 1010: ");
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
    DEBUG_SERIAL.print("Received HTTP RPC request: ");
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
