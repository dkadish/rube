//
// Created by David Kadish on 12/03/2018.
//

#ifndef RUBE_WINCHCONTROLLER_H
#define RUBE_WINCHCONTROLLER_H

#include "Controller.h"
#include "WinchDriver.h"
#include "logging.h"
#include "rube_config.h"

#include <elapsedMillis.h>
#include <PID_v1.h>

struct PIDParams {
    double Kp;
    double Ki;
    double Kd;
};

class WinchController: public Controller {
public:
    WinchController(WinchDriver &winchDriver);

protected:
    WinchDriver &winchDriver;
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

    bool getErrorCondition(){
        if(lostTensionEvent){
            lostTensionEvent = false; // Only return true once.
            return true;
        }

        return false;
    };

    /** Run before each invocation of TensionController */
    virtual void start() {
        INFO("Starting Tension Controller.");
        enabled = true;
        lastLoopTension = winchDriver.isUnderTension();
    }

    /** Run at the end of each invocation of TensionController */
    virtual void end() {
        INFO("Ending Tension Controller.");
        enabled = false;
    }

private:

    bool lastLoopTension = true;
    bool lostTensionEvent = false;
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

/** Provides PID control to bring the motor to an encoder position.
 *
 *  The position is understood internally to be the distance in metres along
 *  the winch line from the starting point.
 */
class PIDPositionController: public WinchController {
public:
    PIDPositionController(WinchDriver &winchDriver, PIDParams pid_p):WinchController(winchDriver),
                            Kp(pid_p.Kp), Ki(pid_p.Ki), Kd(pid_p.Kd){

    }

    virtual void setup(); /**< Run during the main setup() */
    virtual void loop(); /**< Run during the main loop() while PIDPositionController is active */

    virtual void start(); /**< Run before each invovation of PIDPositionController */
    virtual void end(); /**< Run at the end of each invovation of PIDPositionController */

    void setTarget(float setpt){ setpoint = (double) setpt; }
    float getTarget(){ return (float) setpoint; }

    float getError(){ return (float)(setpoint) - winchDriver.getPosition(); }

    void setStopDistance(double distance){ stopOnDistance=true; distanceCondition=distance; };
    void setHoldDistance(double distance){ holdOnDistance=true; distanceCondition=distance; };

    // Builtin PID functions
    void doHold(){ setTarget(winchDriver.getPosition()); start(); } /**< Hold the current position */

private:
    // Define the parameters
    double Kp, Ki, Kd;

    // PID
    //Define Variables we'll be connecting to
    double in, out, setpoint;

    bool stopOnDistance=false, holdOnDistance=false;
    double distanceCondition=0.0; // Distance at which to stop/hold

    bool isWithinDistance(){
        if( getError() < distanceCondition && getError() > -distanceCondition ){
            return true;
        }
        return false;
    }

    //Specify the links and initial tuning parameters
    PID * position;
};

/** Provides PID control to bring the motor to an encoder speed.
 *
 *  THIS MAY NOT BE IMPLEMENTED. Mostly exists to re-capture only speed code
 *  used in shanghai to make sure it is not lost.
 */
class PIDSpeedController: public WinchController {
public:
    PIDSpeedController(WinchDriver &winchDriver);

    virtual void setup(); /**< Run during the main setup() */
    virtual void loop(); /**< Run during the main loop() while MinimumMotionController is active */

    virtual void start(); /**< Run before each invovation of MinimumMotionController */
    virtual void end(); /**< Run at the end of each invovation of MinimumMotionController */

private:

    // Define the parameters
    double pos_Kp;

    // PID
    //Define Variables we'll be connecting to
    double pos_in;
    // PID
    //Define Variables we'll be connecting to
    double pos_out;
    // PID
    //Define Variables we'll be connecting to
    double pos_setpt;
    //Specify the links and initial tuning parameters
    PID * position;
    // Define the parameters
    double spd_Ki;
    // Define the parameters
    double spd_Kp;
    double spd_in;
    double spd_out;
    double spd_setpt;
    PID * speed;
};

#endif //RUBE_WINCHCONTROLLER_H
