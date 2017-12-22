//
// Created by David Kadish on 17/12/2017.
//

#include "SDLogger.h"

SDLogger::SDLogger(String (*logFunction)()) : logFunction(logFunction), enabled(false), writeTimer(0), fileTimer(0), writeInterval(100) {

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
        file = SD.open("datalog.txt", FILE_WRITE);
        enabled = true;

        return true;
    }

    return false;
}

void SDLogger::loop() {
    if (enabled && writeTimer > writeInterval) {

        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
        if(!file){
            file = SD.open("datalog.txt", FILE_WRITE);
            //Serial.println("Opening File");
        }

        // if the file is available, write to it:
        if (file) {
            file.println(logFunction());
            if(fileTimer > 1000){
                //Serial.println("Closing File");
                file.close();
                fileTimer = 0;
            }
            // print to the serial port too:
            Serial.println(logFunction());
        }
            // if the file isn't open, pop up an error:
        else {
            //Serial.println("error opening datalog.txt");
        }

        writeTimer = 0;
    }
}