#ifndef RPC_COMM_H
#define RPC_COMM_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                    AwsEventType type, void *arg, uint8_t *data, size_t len);
void parseUdpRPC();

#endif // RPC_COMM_H
