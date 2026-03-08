#ifndef PARSERS_H
#define PARSERS_H

#include <Arduino.h>

void queryHTTP();
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void mqtt_reconnect();
void parseSHRDZM();
void parseSMA();
void parseSUNSPEC();

#endif // PARSERS_H