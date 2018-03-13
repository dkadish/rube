//
// Created by David Kadish on 12/03/2018.
//

#ifndef RUBE_WINCHCONTROLLER_H
#define RUBE_WINCHCONTROLLER_H

#include "Controller.h"
#include "WinchDriver.h"

#include <elapsedMillis.h>

class WinchController: Controller {
public:
    WinchController(WinchDriver &winchDriver);

//    virtual void setup();
//    virtual void loop();

protected:
    WinchDriver &winchDriver;
    bool enabled;
};

class MinimumMotionController: WinchController {
public:
    MinimumMotionController(WinchDriver &winchDriver);

    virtual void setup(); /**< Run during the main setup() */
    virtual void loop(); /**< Run during the main loop() while MinimumMotionController is active */

    virtual void start(); /**< Run before each invovation of MinimumMotionController */
    virtual void end(); /**< Run at the end of each invovation of MinimumMotionController */

    void setDirection(bool positive);

    enum Status {
        STOPPED,
        RUNNING,
    };

    Status getStatus();

private:
    bool direction_positive;
    elapsedMillis rampTimer;
    int level;
    long lastPositioningTime;
    long lastPosition;
    Status status;
};

#endif //RUBE_WINCHCONTROLLER_H
