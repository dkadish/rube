//
// Created by David Kadish on 18/10/2017.
//

// TODO: Motion planning based on ETA using estimated velocity.

#include "Winch.h"
#include "WinchDriver.h"
#include "rube_config.h"
#include "logging.h"

#include <Arduino.h>
#include <elapsedMillis.h>

// TODO Get Winch/WinchDriver to compile.
// TODO Add states/modes

Winch::Winch(int index, int encA, int encB, int encIDX, int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
             int scale_dout, int scale_sck, long scale_offset, double Kp, double Ki, double Kd):
        driver(encA, encB, encIDX, motorIn1, motorIn2, motorPwm, motorOffset, motorStby,
                    scale_dout, scale_sck, scale_offset),
        pid_p{Kp, Ki, Kd},
        index(index)//, mm_ctrl(driver), tension_ctrl(driver), retension_ctrl(driver, tension_ctrl)
{

}

Winch::Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                         ScaleParams scale_p, PIDParams pid_p):
        driver(enc_p, motor_p, scale_p),
        pid_p(pid_p),
        index(index)//, mm_ctrl(driver), tension_ctrl(driver), retension_ctrl(driver, tension_ctrl)
{
}

Winch::Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                         ScaleParams scale_p, PIDParams pid_p, PositionParams pos_p):
        driver(enc_p, motor_p, scale_p),
        pid_p(pid_p),
        index(index)//, mm_ctrl(driver), tension_ctrl(driver), retension_ctrl(driver, tension_ctrl)
{
    origin.x = pos_p.origin.x;
    origin.y = pos_p.origin.y;
    origin.z = pos_p.origin.z;
    startLength = pos_p.length;
}

void Winch::setup() {
    mm_ctrl = new MinimumMotionController(driver);
    pidPos_ctrl = new PIDPositionController(driver, pid_p);
    tension_ctrl = new TensionMaintenanceController(driver);
    retension_ctrl = new RetensioningController(driver, tension_ctrl);

    controllers[0] = mm_ctrl;
    controllers[1] = pidPos_ctrl;
    controllers[2] = retension_ctrl;
    controllers[3] = tension_ctrl; // tension_ctrl is always last because it has priority.

    INFO("Setting Up Driver")
    driver.setup();

    INFO("Setting Up Controllers")
    for(int i=0; i<n_controllers; i++){
        INFO("Setting Up Controller")
        controllers[i]->setup();
    }

    // Tension control is on by default
    tension_ctrl->start();

}

void Winch::loop() {
    driver.loop();

    if( tension_ctrl->getErrorCondition() ){
        INFO("Winch %i lost tension.", index)
        WARNING("Winch %i lost tension.", index)
        for(Controller *controller: controllers){
            controller->end();
        }
        tension_ctrl->start();
    }

    for(int i=0; i<n_controllers; i++){
        controllers[i]->loop();
    }

}

/** Stop all motion. Ends all controllers and engages tension control.
 *
 */
void Winch::doStop() {
    for(int i=0; i<n_controllers; i++){
        controllers[i]->end();
    }

    driver.stop();

    tension_ctrl->start();
}

