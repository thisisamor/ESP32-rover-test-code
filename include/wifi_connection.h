#ifndef wifi_connection_h
#define wifi_connection_h

#include <string>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>

void wifi_connection(); 
bool check_wifi();
void client_post(String message); 
void client_get(); 

#endif