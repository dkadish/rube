//
// Created by David Kadish on 19/12/2017.
//

#include "wifi.h"
#include "Arduino.h"

void wifiResponse(char* response)
{
    Serial.printf("B%s\n", response);
    Serial1.printf("B%s\n", response);
    Serial1.println("NEWLINE");
}