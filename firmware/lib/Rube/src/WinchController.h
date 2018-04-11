//
// Created by David Kadish on 12/03/2018.
//

#ifndef RUBE_WINCHCONTROLLER_H
#define RUBE_WINCHCONTROLLER_H

#include "Controller.h"
#include "WinchDriver.h"
#include "logging.h"

#include <elapsedMillis.h>

class WinchController: public Controller {
public:
    WinchController(WinchDriver &winchDriver);

//    virtual void setup();
//    virtual void loop();

//protected:
    WinchDriver &winchDriver;
    //bool enabled = false;
};

class MinimumMotionController: public WinchController {
public:
    MinimumMotionController(WinchDriver &winchDriver);

    virtual void setup(); /**< Run during the main setup() */
    virtual void loop(); /**< Run during the main loop() while MinimumMotionController is active */

    virtual void start(); /**< Run before each invovation of MinimumMotionController */
    virtual void end(); /**< Run at the end of each invovation of MinimumMotionController */

    void setDirection(bool positive);

private:
    bool direction_positive;
    elapsedMillis rampTimer;
    int level;
    unsigned long lastPositioningTime;
    long lastPosition;
};

/** Ensures that the winch is kept under tension, otherwise ceases all motion.
 *
 *  @paragraph When the TensionMaintenanceController isEnabled, it will set the
 *      WinchDriver::stop_go to STOP if the line loses tension.
 */
class TensionMaintenanceController: public WinchController {
public:
    TensionMaintenanceController(WinchDriver &winchDriver) : WinchController(winchDriver) {}

    virtual void setup(){}; /**< Run during the main setup() */
    virtual void loop(); /**< Run during the main loop() while TensionController is active */

    /** Run before each invocation of TensionController */
    virtual void start() {
        INFO("Starting Tension Controller.");
        enabled = true;
    }

    /** Run at the end of each invocation of TensionController */
    virtual void end() {
        INFO("Ending Tension Controller.");
        enabled = false;
    }
};


/** Returns the robot to tension when not under tension.
 *
 */
class RetensioningController: public WinchController {
public:
    RetensioningController(WinchDriver &winchDriver, TensionMaintenanceController *tension) :
            WinchController(winchDriver), tensioner(tension) {}

    virtual void setup(){}; /**< Run during the main setup() */
    virtual void loop(); /**< Run during the main loop() while RetensioningController is active */

    /** Run before each invocation of RetensioningController */
    virtual void start() {
        enabled = true;
        tensioner->end(); // Turn off the tension maintenance controller so it doesn't stop motion.

        winchDriver.setSignal(100);
        winchDriver.go();
    }

    /** Run at the end of each invocation of RetensioningController */
    virtual void end() {
        enabled = false;
        tensioner->start();
    }

private:
    TensionMaintenanceController *tensioner;
};

#endif //RUBE_WINCHCONTROLLER_H
