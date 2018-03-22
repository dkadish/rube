//
// Created by David Kadish on 12/03/2018.
//

#include "WinchController.h"

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
    lastPosition = winchDriver.enc->read();
}

void MinimumMotionController::loop() {

    // Stop if tension is lost
    /*if (!winchDriver.isUnderTension()){
        winchDriver.go_signal(0);
        char response[255];
        sprintf(response, "Winch is lost tension.");
        wifiResponse(response);
        end();
    }*/

    // Raise the level every once in a while IF it has not moved since last time.
    if ( rampTimer/5 > lastPositioningTime ){
        lastPositioningTime++;
        long winchPos = winchDriver.enc->read();

        /* The position goes negative as it rises. So, when the old position
         * is <= the new position, it hasn't gone anywhere.
         */
        if( lastPosition <= winchPos && direction_positive){
            level++;
        } else if( lastPosition >= winchPos && !direction_positive){ // Unless we are going negative
            level--;
        }
        lastPosition = winchPos;
    }

    winchDriver.setSignal(level);
}

void MinimumMotionController::start() {
    enabled = true;

    rampTimer = 0;
    level = 0;
    lastPositioningTime = 0;
    lastPosition = winchDriver.enc->read();

    winchDriver.setSignal(level);
    winchDriver.go();
}

void MinimumMotionController::end() {
    enabled = false;
}

/** Set the direction of motion.
 *
 * @param positive True for moving in a positive direction.
 */
void MinimumMotionController::setDirection(bool positive) {
    direction_positive = positive;
}

MinimumMotionController::Status MinimumMotionController::getStatus(){
    return status;
}

void TensionMaintenanceController::loop() {
    if(enabled){
        if (!winchDriver.isUnderTension()){
            winchDriver.setSignal(0);
            winchDriver.stop();
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
