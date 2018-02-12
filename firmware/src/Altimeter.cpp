//
// Created by David Kadish on 10/01/2018.
//

#include "Altimeter.h"

Altimeter::Altimeter(i2c_t3 *wire) : wire(wire) {
    altimeter(wire);
}

void Altimeter::setup() {

    altimeter.begin();
    altimeter.setModeAltimeter();
    altimeter.setOversampleRate(7); // Set Oversample to the recommended 128
    altimeter.enableEventFlags(); // Enable all three pressure and temp event flags
}

void Altimeter::loop() {
    altitude_m = altimeter.readAltitude();
}

