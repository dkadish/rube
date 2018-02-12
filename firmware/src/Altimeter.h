//
// Created by David Kadish on 10/01/2018.
//

#ifndef FIRMWARE_ALTIMETER_H
#define FIRMWARE_ALTIMETER_H

#include <i2c_t3.h>
#include "SparkFunMPL3115A2.h"

class Altimeter {
public:
    Altimeter(i2c_t3 *wire);

    void setup();
    void loop();

    MPL3115A2 altimeter;
    float altitude_m; // Altitude in Metres

private:
    i2c_t3 * wire;
};


#endif //FIRMWARE_ALTIMETER_H
