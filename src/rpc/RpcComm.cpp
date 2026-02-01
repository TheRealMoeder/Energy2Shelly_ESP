#include "RpcHandlers.h"
#include "../config/Configuration.h"
#include "../data/DataStructures.h"

// ============================================================================
// WEBSOCKET RPC HANDLER
// ============================================================================

void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                    AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
  case WS_EVT_DISCONNECT:
    DEBUG_SERIAL.printf("[%u] Websocket: disconnected!\n", client->id());
    break;
  case WS_EVT_CONNECT:
    DEBUG_SERIAL.printf("[%u] Websocket: connected from %s\n", client->id(),
                        client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DATA: {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len &&
        info->opcode == WS_TEXT) {
      data[len] = 0;
      deserializeJson(globalJsonDoc, data);
      rpcId = globalJsonDoc["id"];
      if (globalJsonDoc["method"] == "Shelly.GetDeviceInfo") {
        strncpy(rpcUser, "EMPTY", sizeof(rpcUser) - 1);
        rpcUser[sizeof(rpcUser) - 1] = '\0';
        GetDeviceInfo();
        rpcWrapper();
        webSocket.textAll(serJsonResponse);
      } else if (globalJsonDoc["method"] == "EM.GetStatus") {
        String src = globalJsonDoc["src"].as<String>();
        strncpy(rpcUser, src.c_str(), sizeof(rpcUser) - 1);
        rpcUser[sizeof(rpcUser) - 1] = '\0';
        EMGetStatus();
        rpcWrapper();
        webSocket.textAll(serJsonResponse);
      } else if (globalJsonDoc["method"] == "EMData.GetStatus") {
        String src = globalJsonDoc["src"].as<String>();
        strncpy(rpcUser, src.c_str(), sizeof(rpcUser) - 1);
        rpcUser[sizeof(rpcUser) - 1] = '\0';
        EMDataGetStatus();
        rpcWrapper();
        webSocket.textAll(serJsonResponse);
      } else if (globalJsonDoc["method"] == "EM.GetConfig") {
        EMGetConfig();
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

// ============================================================================
// UDP RPC HANDLER (for Marstek compatibility)
// ============================================================================

void parseUdpRPC() {
  int packetSize = UdpRPC.parsePacket();
  if (packetSize) {
    int rSize = UdpRPC.read(networkBuffer, 1024);
    networkBuffer[rSize] = 0;
    DEBUG_SERIAL.print("Received UDP packet on port 1010: ");
    DEBUG_SERIAL.println((char *)networkBuffer);
    deserializeJson(globalJsonDoc, networkBuffer);
    if (globalJsonDoc["method"].is<JsonVariant>()) {
      rpcId = globalJsonDoc["id"];
      strncpy(rpcUser, "EMPTY", sizeof(rpcUser) - 1);
      rpcUser[sizeof(rpcUser) - 1] = '\0';
      UdpRPC.beginPacket(UdpRPC.remoteIP(), UdpRPC.remotePort());
      if (globalJsonDoc["method"] == "Shelly.GetDeviceInfo") {
        GetDeviceInfo();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (globalJsonDoc["method"] == "EM.GetStatus") {
        EMGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (globalJsonDoc["method"] == "EMData.GetStatus") {
        EMDataGetStatus();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else if (globalJsonDoc["method"] == "EM.GetConfig") {
        EMGetConfig();
        rpcWrapper();
        UdpRPC.UDPPRINT(serJsonResponse.c_str());
      } else {
        DEBUG_SERIAL.printf("RPC over UDP: unknown request: %s\n",
                            networkBuffer);
      }
      UdpRPC.endPacket();
    }
    globalJsonDoc.clear();
  }
}
