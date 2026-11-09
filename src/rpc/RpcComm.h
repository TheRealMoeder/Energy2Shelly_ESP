#ifndef RPC_COMM_H
#define RPC_COMM_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

// minimum time between two NotifyStatus messages to websocket clients (ms)
#define NOTIFY_STATUS_MIN_INTERVAL 500

size_t scriptGetCode(char* buffer, size_t maxLen);
size_t scriptList(char* buffer, size_t maxLen);
size_t shellyListMethods(char* buffer, size_t maxLen);
void buildRpcResponse(const char* method, long rpcId, const char* src, String& output);
void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void notifyStatusWebSocket();
void parseUdpRPC();
void parseHttpRPC(String requestBody, AsyncWebServerRequest *request);

#endif // RPC_COMM_H