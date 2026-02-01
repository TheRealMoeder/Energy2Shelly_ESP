#ifndef MQTT_PARSER_H
#define MQTT_PARSER_H

#include <Arduino.h>

void mqtt_callback(char *topic, byte *payload, unsigned int length);
void mqtt_reconnect();

#endif // MQTT_PARSER_H
