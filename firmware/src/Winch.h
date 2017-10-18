//
// Created by David Kadish on 18/10/2017.
//

#ifndef FIRMWARE_WINCH_H
#define FIRMWARE_WINCH_H

#include "HX711.h"
#include <Encoder.h>
#include <SparkFun_TB6612.h>

class Winch {

    // Internal Variables
    int position;

    // Motor
    const int offset;
    Motor motor;

    // Encoder
    Encoder enc;

    // Scale
    HX711 scale;
};


#endif //FIRMWARE_WINCH_H
