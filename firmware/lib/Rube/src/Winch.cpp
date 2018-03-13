//
// Created by David Kadish on 18/10/2017.
//

// TODO: Motion planning based on ETA using estimated velocity.

#include "Winch.h"
#include "WinchDriver.h"
#include "rube_config.h"

#include <PID_v1.h>
#include <elapsedMillis.h>

// TODO Get Winch/WinchDriver to compile.
// TODO Add states/modes

Winch::Winch(int index, int encA, int encB, int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
             int scale_dout, int scale_sck, long scale_offset, float positionKp, float speedKp, float speedKi):
        WinchDriver(encA, encB, motorIn1, motorIn2, motorPwm, motorOffset, motorStby,
                    scale_dout, scale_sck, scale_offset),
        pos_Kp(positionKp), spd_Kp(speedKp), spd_Ki(speedKi),
        index(index)
{

}

Winch::Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                         ScaleParams scale_p, FilterParams filter_p):
        WinchDriver(enc_p, motor_p, scale_p),
        pos_Kp(filter_p.positionKp), spd_Kp(filter_p.speedKp), spd_Ki(filter_p.speedKi),
        index(index)
{

}

Winch::Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                         ScaleParams scale_p, FilterParams filter_p, PositionParams pos_p):
        WinchDriver(enc_p, motor_p, scale_p),
        pos_Kp(filter_p.positionKp), spd_Kp(filter_p.speedKp), spd_Ki(filter_p.speedKi),
        index(index)
{
    origin.x = pos_p.origin.x;
    origin.y = pos_p.origin.y;
    origin.z = pos_p.origin.z;
    startLength = pos_p.length;
}

void Winch::setup() {
    WinchDriver::setup();

    pid_setup();
}

void Winch::loop() {
    WinchDriver::loop();

    pid_loop();
}

double Winch::current_position() {
    return ((double)enc->read())/TICKS_PER_REVOLUTION;
}


double Winch::getLength() {
    return startLength - current_position()*METRES_PER_REVOLUTION;
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


