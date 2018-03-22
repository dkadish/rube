//
// Created by David Kadish on 12/03/2018.
//

#ifndef RUBE_WINCHCONTROLLER_H
#define RUBE_WINCHCONTROLLER_H

#include "Controller.h"
#include "WinchDriver.h"

#include <elapsedMillis.h>

class WinchController: protected Controller {
public:
    WinchController(WinchDriver &winchDriver);

//    virtual void setup();
//    virtual void loop();

protected:
    WinchDriver &winchDriver;
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

/** Ensures that the winch is kept under tension, otherwise ceases all motion.
 *
 *  @paragraph When the TensionMaintenanceController isEnabled, it will set the
 *      WinchDriver::stop_go to STOP if the line loses tension.
 */
class TensionMaintenanceController: WinchController {
public:
    TensionMaintenanceController(WinchDriver &winchDriver) : WinchController(winchDriver) {}

    virtual void setup(){}; /**< Run during the main setup() */
    virtual void loop(); /**< Run during the main loop() while TensionController is active */

    /** Run before each invocation of TensionController */
    virtual void start() {
        enabled = true;
    }

    /** Run at the end of each invocation of TensionController */
    virtual void end() {
        enabled = false;
    }
};


/** Returns the robot to tension when not under tension.
 *
 */
class RetensioningController: WinchController {
public:
    RetensioningController(WinchDriver &winchDriver, TensionMaintenanceController &tension) :
            WinchController(winchDriver), tensioner(tension) {}

    virtual void setup(){}; /**< Run during the main setup() */
    virtual void loop(); /**< Run during the main loop() while RetensioningController is active */

    /** Run before each invocation of RetensioningController */
    virtual void start() {
        enabled = true;
        tensioner.end(); // Turn off the tension maintenance controller so it doesn't stop motion.

        winchDriver.setSignal(100);
        winchDriver.go();
    }

    /** Run at the end of each invocation of RetensioningController */
    virtual void end() {
        enabled = false;
        tensioner.start();
    }

private:
    TensionMaintenanceController &tensioner;
};

#endif //RUBE_WINCHCONTROLLER_H
