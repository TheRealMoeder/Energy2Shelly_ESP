#include "RpcHandlers.h"
#include "../config/Configuration.h"
#include "../data/DataStructures.h"
#include "RpcComm.h"

// Aligned with Shelly API docs: Script.GetCode
size_t scriptGetCode(char* buffer, size_t maxLen) {
    int written = snprintf(buffer, maxLen, "{\"data\":\"\",\"left\":\"0\"}");
    blinkled(ledblinkduration);
    return finalize_buffer(written, maxLen);
}

// Aligned with Shelly API docs: Script.List
size_t scriptList(char* buffer, size_t maxLen) {
    int written = snprintf(buffer, maxLen, "{\"scripts\":[]}");
    blinkled(ledblinkduration);
    return finalize_buffer(written, maxLen);
}

// Aligned with Shelly API docs: Shelly.ListMethods
size_t shellyListMethods(char* buffer, size_t maxLen) {
    int written = snprintf(buffer, maxLen, 
        "{\"methods\":[\"EM.GetStatus\",\"EM.GetConfig\",\"EMData.GetStatus\",\"Shelly.GetComponents\","
        "\"Shelly.GetConfig\",\"Shelly.GetDeviceInfo\",\"Shelly.GetStatus\",\"Sys.GetConfig\",\"Sys.GetStatus\",\"WiFi.GetStatus\"]}"
    );
    blinkled(ledblinkduration);
    return finalize_buffer(written, maxLen);
}

// Router function allocating a local payload per request channel to prevent race conditions
void buildRpcResponse(const char* method, long rpcId, const char* src, String& output) {
    const size_t INNER_SIZE = 1536; 
    String innerJson;
    innerJson.reserve(INNER_SIZE);
    char* innerPtr = const_cast<char*>(innerJson.c_str());

    if (strcmp(method, "Shelly.GetDeviceInfo") == 0) shellyGetDeviceInfo(innerPtr, INNER_SIZE);
    else if (strcmp(method, "Shelly.GetComponents") == 0) shellyComponents(innerPtr, INNER_SIZE);
    else if (strcmp(method, "Shelly.GetConfig") == 0) shellyGetConfig(innerPtr, INNER_SIZE);
    else if (strcmp(method, "Shelly.GetStatus") == 0) shellyGetStatus(innerPtr, INNER_SIZE);
    else if (strcmp(method, "Shelly.ListMethods") == 0) shellyListMethods(innerPtr, INNER_SIZE);
    else if (strcmp(method, "EM.GetStatus") == 0) EMGetStatus(innerPtr, INNER_SIZE);
    else if (strcmp(method, "EMData.GetStatus") == 0) EMDataGetStatus(innerPtr, INNER_SIZE);
    else if (strcmp(method, "EM.GetConfig") == 0) EMGetConfig(innerPtr, INNER_SIZE);
    else if (strcmp(method, "Script.GetCode") == 0) scriptGetCode(innerPtr, INNER_SIZE);
    else if (strcmp(method, "Script.List") == 0) scriptList(innerPtr, INNER_SIZE);
    else if (strcmp(method, "WiFi.GetStatus") == 0) wifiGetStatus(innerPtr, INNER_SIZE);
    else if (strcmp(method, "Sys.GetStatus") == 0) sysGetStatus(innerPtr, INNER_SIZE);
    else if (strcmp(method, "Sys.GetConfig") == 0) sysGetConfig(innerPtr, INNER_SIZE);
    else {
        output = "{\"id\":" + String(rpcId) + ",\"src\":\"" + String(shelly_name.c_str()) + "\",\"error\":{\"code\":-32601,\"message\":\"Method not found\"}}";
        return;
    }
    innerJson = String(innerPtr);

    const size_t WRAPPER_SIZE = INNER_SIZE + 256;
    output.reserve(WRAPPER_SIZE);
    char* wrapPtr = const_cast<char*>(output.c_str());

    if (src != nullptr && strcmp(src, "EMPTY") != 0 && strlen(src) > 0) {
        snprintf(wrapPtr, WRAPPER_SIZE, "{\"id\":%ld,\"src\":\"%s\",\"dst\":\"%s\",\"result\":%s}", rpcId, shelly_name.c_str(), src, innerJson.c_str());
    } else {
        snprintf(wrapPtr, WRAPPER_SIZE, "{\"id\":%ld,\"src\":\"%s\",\"result\":%s}", rpcId, shelly_name.c_str(), innerJson.c_str());
    }
    output = String(wrapPtr);
}

void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type) {
    case WS_EVT_DISCONNECT:
      DEBUG_SERIAL.printf("[%u] Websocket: disconnected!\n", (unsigned int)client->id());
      break;
    case WS_EVT_CONNECT:
      DEBUG_SERIAL.printf("[%u] Websocket: connected from %s\n", (unsigned int)client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;

      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        JsonDocument json;
        DeserializationError error = deserializeJson(json, (const char *)data, len);
        if (error) {
          DEBUG_SERIAL.print(F("Websocket: deserializeJson() error: "));
          DEBUG_SERIAL.println(error.f_str());
          return;
        }
        long localRpcId = json["id"] | 0;
        const char* method = json["method"] | "";
        const char* src = json["src"] | "EMPTY";

        String wsAnswer;
        buildRpcResponse(method, localRpcId, src, wsAnswer);

        if (wsAnswer.length() > 0 && client->status() == WS_CONNECTED) {
          client->text(wsAnswer);
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

// Push live data packets to connected WebSockets completely string/JSON-document-free via stack buffering
// Home Assistant only polls once on connect and relies on these notifications afterwards.
// Event driven: sent as soon as a data source has written new values. The minimum interval
// (NOTIFY_STATUS_MIN_INTERVAL) merges updates that arrive in several parts (e.g. one MQTT message per phase).
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

  char emStatusBuffer[768] = "";
  char emDataStatusBuffer[512] = "";

  if (powerChanged) {
    float total_power = PhasePower[0].power + PhasePower[1].power + PhasePower[2].power;
    float total_apparent = PhasePower[0].apparentPower + PhasePower[1].apparentPower + PhasePower[2].apparentPower;
    
    snprintf(emStatusBuffer, sizeof(emStatusBuffer),
      "\"em:0\":{\"id\":0,\"a_current\":%.2f,\"a_voltage\":%.2f,\"a_act_power\":%.2f,\"a_aprt_power\":%.2f,\"a_pf\":%.2f,\"a_freq\":%.2f,"
      "\"b_current\":%.2f,\"b_voltage\":%.2f,\"b_act_power\":%.2f,\"b_aprt_power\":%.2f,\"b_pf\":%.2f,\"b_freq\":%.2f,"
      "\"c_current\":%.2f,\"c_voltage\":%.2f,\"c_act_power\":%.2f,\"c_aprt_power\":%.2f,\"c_pf\":%.2f,\"c_freq\":%.2f,"
      "\"n_current\":null,\"total_current\":%.2f,\"total_act_power\":%.2f,\"total_aprt_power\":%.2f,\"user_calibrated_phase\":[]}",
      PhasePower[0].current, PhasePower[0].voltage, PhasePower[0].power, PhasePower[0].apparentPower, PhasePower[0].powerFactor, PhasePower[0].frequency,
      PhasePower[1].current, PhasePower[1].voltage, PhasePower[1].power, PhasePower[1].apparentPower, PhasePower[1].powerFactor, PhasePower[1].frequency,
      PhasePower[2].current, PhasePower[2].voltage, PhasePower[2].power, PhasePower[2].apparentPower, PhasePower[2].powerFactor, PhasePower[2].frequency,
      total_power / defaultVoltage, total_power, total_apparent
    );
  }

  if (energyChanged) {
    snprintf(emDataStatusBuffer, sizeof(emDataStatusBuffer),
      "\"emdata:0\":{\"id\":0,\"a_total_act_energy\":%.2f,\"a_total_act_ret_energy\":%.2f,"
      "\"b_total_act_energy\":%.2f,\"b_total_act_ret_energy\":%.2f,"
      "\"c_total_act_energy\":%.2f,\"c_total_act_ret_energy\":%.2f,"
      "\"total_act\":%.2f,\"total_act_ret\":%.2f}",
      PhaseEnergy[0].consumption, PhaseEnergy[0].gridfeedin,
      PhaseEnergy[1].consumption, PhaseEnergy[1].gridfeedin,
      PhaseEnergy[2].consumption, PhaseEnergy[2].gridfeedin,
      PhaseEnergy[0].consumption + PhaseEnergy[1].consumption + PhaseEnergy[2].consumption,
      PhaseEnergy[0].gridfeedin + PhaseEnergy[1].gridfeedin + PhaseEnergy[2].gridfeedin
    );
  }

  const size_t FINAL_BUFFER_SIZE = 2048;
  String message;
  message.reserve(FINAL_BUFFER_SIZE);
  char* finalPtr = const_cast<char*>(message.c_str());

  const char* comma = (powerChanged && energyChanged) ? "," : "";

  snprintf(finalPtr, FINAL_BUFFER_SIZE,
    "{\"src\":\"%s\",\"method\":\"NotifyStatus\",\"params\":{\"ts\":%.0f,%s%s%s}}",
    shelly_name.c_str(),
    (double)time(nullptr),
    powerChanged ? emStatusBuffer : "",
    comma,
    energyChanged ? emDataStatusBuffer : ""
  );

  message = String(finalPtr);
  DEBUG_SERIAL.print(F("NotifyStatus: "));
  DEBUG_SERIAL.println(message);
  webSocket.textAll(message);
}

void parseUdpRPC() {
  uint8_t buffer[1024];
  int packetSize = UdpRPC.parsePacket();
  if (packetSize > 0) {
    JsonDocument json;
    int rSize = UdpRPC.read(buffer, sizeof(buffer) - 1);
    buffer[rSize] = '\0';
    DEBUG_SERIAL.print(F("Received UDP packet on port 1010: "));
    DEBUG_SERIAL.println((char *)buffer);
    deserializeJson(json, buffer);
    if (json["method"].is<JsonVariant>()) {
      long localRpcId = json["id"] | 0;
      const char* method = json["method"] | "";

      String udpAnswer;
      buildRpcResponse(method, localRpcId, "EMPTY", udpAnswer);

      UdpRPC.beginPacket(UdpRPC.remoteIP(), UdpRPC.remotePort());
      if (udpAnswer.length() > 0) {
        UdpRPC.UDPPRINT(udpAnswer.c_str());
      } else {
        DEBUG_SERIAL.printf("RPC over UDP: unknown request: %s\n", buffer);
      }
      UdpRPC.endPacket();
    }
  }
}

void parseHttpRPC(const String& requestBody, String& responseBody) {
  JsonDocument json;
  DEBUG_SERIAL.print(F("Received HTTP RPC request: "));
  DEBUG_SERIAL.println(requestBody);

  if (deserializeJson(json, requestBody) || !json["method"].is<const char*>()) {
    responseBody = "{\"error\":{\"code\":-32600,\"message\":\"Invalid request\"}}";
    return;
  }
  
  long localRpcId = json["id"] | 0;
  const char* method = json["method"] | "";
  buildRpcResponse(method, localRpcId, "EMPTY", responseBody);
}