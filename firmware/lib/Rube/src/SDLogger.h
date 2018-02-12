//
// Created by David Kadish on 17/12/2017.
//

#ifndef FIRMWARE_SDLOGGER_H
#define FIRMWARE_SDLOGGER_H

#include <SD.h>
#include <SPI.h>

class SDLogger {

public:
    SDLogger(String (*logFunction)());

    void setChipSelect(int cs);

    bool setup();
    void loop();

    bool enabled;

private:

    File file;

    int csPin;

    int writeInterval;

    elapsedMillis writeTimer, fileTimer;

    String (*logFunction)();

};


#endif //FIRMWARE_SDLOGGER_H
