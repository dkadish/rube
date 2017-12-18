//
// Created by David Kadish on 17/12/2017.
//

#include "SDLogger.h"

SDLogger::SDLogger(String (*logFunction)()) : logFunction(logFunction), enabled(false), writeCounter(0) {

    csPin = BUILTIN_SDCARD;
/*#ifdef BUILTIN_SDCARD
    csPin = BUILTIN_SDCARD;
#else
    csPin = 10;
#endif*/
}

void SDLogger::setChipSelect(int cs) {
    csPin = cs;
}

bool SDLogger::setup() {
    if(SD.begin(csPin)) {
        file = SD.open("datalog.csv", FILE_WRITE);
        enabled = true;

        return true;
    }

    return false;
}

void SDLogger::loop() {
    if (enabled) {
        //file.open("datalog.csv", FILE_WRITE);
        file.println(logFunction());
        //file.close();
        //writeCounter++;
    }

    /*if( writeCounter > 10 ){
        file.flush();
        writeCounter = 0;
    }*/
}