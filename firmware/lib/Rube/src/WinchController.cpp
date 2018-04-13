//
// Created by David Kadish on 12/03/2018.
//

#include "WinchController.h"
#include "logging.h"

#include <wifi.h>

WinchController::WinchController(WinchDriver &winchDriver): Controller(),
    winchDriver(winchDriver)
{

}

MinimumMotionController::MinimumMotionController(WinchDriver &winchDriver) :
        WinchController(winchDriver) {

}

void MinimumMotionController::setup() {
    rampTimer = 0;
    level = 0;
    lastPositioningTime = 0;
    lastPosition = winchDriver.getEncoderTicks();
}

void MinimumMotionController::loop() {

    if(enabled) {
        // Raise the level every once in a while IF it has not moved since last time.
        if (rampTimer / 5 > lastPositioningTime) {
            lastPositioningTime++;
            long winchPos = winchDriver.getEncoderTicks();

            /* The position goes negative as it rises. So, when the old position
             * is <= the new position, it hasn't gone anywhere.
             */
            if (lastPosition <= winchPos && direction_positive) {
                INFO("MM: Increasing level.");
                level++;
            } else if (lastPosition >= winchPos && !direction_positive) { // Unless we are going negative
                INFO("MM: Decreasing level.");
                level--;
            }
            lastPosition = winchPos;
        }

        winchDriver.setSignal(level);
    }
}

void MinimumMotionController::start() {
    INFO("Minimum Motion is beginning");
    enabled = true;

    rampTimer = 0;
    level = 0;
    lastPositioningTime = 0;
    lastPosition = winchDriver.getEncoderTicks();

    winchDriver.setSignal(level);
    winchDriver.go();
}

void MinimumMotionController::end() {
    INFO("Minimum Motion is ending");
    enabled = false;
}

/** Set the direction of motion.
 *
 * @param positive True for moving in a positive direction.
 */
void MinimumMotionController::setDirection(bool positive) {
    direction_positive = positive;
}

void TensionMaintenanceController::loop() {
    if(enabled){
        if (!winchDriver.isUnderTension()){
            winchDriver.setSignal(0);
            winchDriver.stop();

            // Check to see if this is the beginning of a lost tension event.
            if(lastLoopTension){
                WARNING("Stopped due to lack of tension.")
                lostTensionEvent = true;
            }
            lastLoopTension = false;
        }
    }
}

void RetensioningController::loop() {
    if(enabled){
        if(winchDriver.isUnderTension()){
            winchDriver.setSignal(0);
            winchDriver.stop();

            end();
        }
    }
}

void PIDPositionController::setup(){
    position = new PID(&in, &out, &setpoint, Kp, Ki, Kd, REVERSE);
    position->SetOutputLimits(MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);

    INFO("Started PID Controller with: Kp=%i, Ki=%i, Kd=%i",
         (int)Kp, (int)Ki, (int)Kd);

    // Start off by default
    PIDPositionController::end();
}

void PIDPositionController::loop(){

    in = (double) winchDriver.getPosition();
    position->Compute();

    if(enabled){
//        INFO("Setting signal to %i, given current position of %i", (int)(out), (int)(in));
        winchDriver.setSignal((int)out);

        if(stopOnDistance && isWithinDistance()){
            end();
        } else if (holdOnDistance && isWithinDistance()){
            setTarget(winchDriver.getPosition());
        }
    }
}

void PIDPositionController::start() {
    enabled = true;
    position->SetMode(AUTOMATIC);

    // Turn on the motors
    winchDriver.go();
}

void PIDPositionController::end() {
    enabled = false;
    position->SetMode(MANUAL);
    winchDriver.stop();

    // Reset Parameters
    stopOnDistance = false;
    holdOnDistance = false;
    distanceCondition = 0.0;
}

void PIDSpeedController::setup(){
    position = new PID(&pos_in, &pos_out, &pos_setpt, pos_Kp, 0.0, 0.0, DIRECT);
    position->SetMode(AUTOMATIC);
    position->SetOutputLimits(POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);

    speed = new PID(&spd_in, &spd_out, &pos_out, spd_Kp, spd_Ki, 0.0, DIRECT);
    speed->SetMode(AUTOMATIC);
    speed->SetOutputLimits(MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
}