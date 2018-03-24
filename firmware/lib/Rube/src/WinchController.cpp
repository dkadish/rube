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

    // Stop if tension is lost
    /*if (!winchDriver.isUnderTension()){
        winchDriver.go_signal(0);
        char response[255];
        sprintf(response, "Winch is lost tension.");
        wifiResponse(response);
        end();
    }*/
    //INFO("MM: Looping. %i", enabled ? 1 : 0);
    //FIXME: Enabled is showing up false here. WHY
    //INFO("MM Enabled? %i %i %i %i", enabled ? 1 : 0, isEnabled() ? 1 : 0, Controller::enabled ? 1 : 0, Controller::isEnabled() ? 1 : 0);
    if(enabled) {
        INFO("MM: Looping.");
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

    //FIXME: Enabled true here
    INFO("Minimum Motion is enabled %i %i %i %i", enabled ? 1 : 0, isEnabled() ? 1 : 0, Controller::enabled ? 1 : 0, Controller::isEnabled() ? 1 : 0);
}

void MinimumMotionController::end() {
    INFO("Minimum Motion is ending");
    enabled = false;

    //FIXME: Enabled false here
    INFO("Minimum Motion is enabled %i %i %i %i", enabled ? 1 : 0, isEnabled() ? 1 : 0, Controller::enabled ? 1 : 0, Controller::isEnabled() ? 1 : 0);
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
