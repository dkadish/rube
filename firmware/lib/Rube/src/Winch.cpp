//
// Created by David Kadish on 18/10/2017.
//

// TODO: Motion planning based on ETA using estimated velocity.

#include "Winch.h"
#include "WinchDriver.h"
#include "rube_config.h"
#include "logging.h"

#include <Arduino.h>
#include <PID_v1.h>
#include <elapsedMillis.h>

// TODO Get Winch/WinchDriver to compile.
// TODO Add states/modes

Winch::Winch(int index, int encA, int encB, int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
             int scale_dout, int scale_sck, long scale_offset, float positionKp, float speedKp, float speedKi):
        driver(encA, encB, motorIn1, motorIn2, motorPwm, motorOffset, motorStby,
                    scale_dout, scale_sck, scale_offset),
        pos_Kp(positionKp), spd_Kp(speedKp), spd_Ki(speedKi),
        index(index)//, mm_ctrl(driver), tension_ctrl(driver), retension_ctrl(driver, tension_ctrl)
{

}

Winch::Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                         ScaleParams scale_p, FilterParams filter_p):
        driver(enc_p, motor_p, scale_p),
        pos_Kp(filter_p.positionKp), spd_Kp(filter_p.speedKp), spd_Ki(filter_p.speedKi),
        index(index)//, mm_ctrl(driver), tension_ctrl(driver), retension_ctrl(driver, tension_ctrl)
{
}

Winch::Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                         ScaleParams scale_p, FilterParams filter_p, PositionParams pos_p):
        driver(enc_p, motor_p, scale_p),
        pos_Kp(filter_p.positionKp), spd_Kp(filter_p.speedKp), spd_Ki(filter_p.speedKi),
        index(index)//, mm_ctrl(driver), tension_ctrl(driver), retension_ctrl(driver, tension_ctrl)
{
    origin.x = pos_p.origin.x;
    origin.y = pos_p.origin.y;
    origin.z = pos_p.origin.z;
    startLength = pos_p.length;
}

void Winch::setup() {
    mm_ctrl = new MinimumMotionController(driver);
    tension_ctrl = new TensionMaintenanceController(driver);
    retension_ctrl = new RetensioningController(driver, tension_ctrl);

    controllers[0] = mm_ctrl;
    controllers[1] = tension_ctrl;
    controllers[2] = retension_ctrl;

    INFO("Setting Up Driver")
    driver.setup();

    INFO("Setting Up Controllers")
    for(int i=0; i<n_controllers; i++){
        INFO("Setting Up Controller")
        controllers[i]->setup();
    }

    // Tension control is on by default
    tension_ctrl->start();

    pid_setup();
}

void Winch::loop() {
    driver.loop();

    if(printTimer > 500){
//        INFO("Controllers: 0 (%i/%i), 1 (%i/%i), 2 (%i/%i)\n",
//             controllers[0]->isEnabled(), mm_ctrl->isEnabled(),
//             controllers[1]->isEnabled(), tension_ctrl->isEnabled(),
//             controllers[2]->isEnabled(), retension_ctrl->isEnabled());
//
//        INFO("Controller Addresses: 0 (%ld/%ld), 1 (%ld/%ld), 2 (%ld/%ld)\n",
//             (long)(controllers[0]), (long)(&mm_ctrl),
//             (long)(controllers[1]), (long)(&tension_ctrl),
//             (long)(controllers[2]), (long)(&retension_ctrl));
//
//        INFO("Controller Driver Addresses: %ld, 0 (%ld), 1 (%ld), 2 (%ld)\n",
//             (long)(&driver), (long)(&(mm_ctrl->winchDriver)), (long)(&(tension_ctrl->winchDriver)), (long)(&(retension_ctrl->winchDriver))
//        );
        printTimer = 0;
    }

    for(int i=0; i<n_controllers; i++){
        controllers[i]->loop();
    }


    pid_loop();
}

float Winch::getPosition() {
    return driver.getEncoderTurns()*METRES_PER_REVOLUTION;
}


float Winch::getLength() {
    return startLength - getPosition();
}

void Winch::pid_setup() {

    position = new PID(&pos_in, &pos_out, &pos_setpt, pos_Kp, 0.0, 0.0, DIRECT);
    position->SetMode(AUTOMATIC);
    position->SetOutputLimits(POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);

    speed = new PID(&spd_in, &spd_out, &pos_out, spd_Kp, spd_Ki, 0.0, DIRECT);
    speed->SetMode(AUTOMATIC);
    speed->SetOutputLimits(MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);

}

void Winch::pid_loop() {

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


