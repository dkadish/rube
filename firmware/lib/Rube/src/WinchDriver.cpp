//
// Created by David Kadish on 09/03/2018.
//

#include "WinchDriver.h"
#include "wifi.h"
#include "rube_config.h"
#include <Encoder.h>
#include <usb_serial.h>

WinchDriver::WinchDriver(int encA, int encB,
                          int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
                          int scale_dout, int scale_sck, long offset
                          //float positionKp, float speedKp, float speedKi
): encA(encA), encB(encB), motor(motorIn1, motorIn2, motorPwm, motorOffset, motorStby),
   dout(scale_dout), sck(scale_sck), scale_offset(offset),
   //Winch::pos_Kp(positionKp), Winch::spd_Kp(speedKp), Winch::spd_Ki(speedKi),
   stop_go(STOP)
{

}

WinchDriver::WinchDriver(EncoderParams enc_p, MotorParams motor_p,
                          ScaleParams scale_p
                          //FilterParams filter_p
):
        encA(enc_p.A), encB(enc_p.B),
        motor(motor_p.in1, motor_p.in2, motor_p.pwm, motor_p.offset, motor_p.standby),
        dout(scale_p.dout), sck(scale_p.sck), scale_offset(scale_p.offset),
        //Winch::pos_Kp(filter_p.positionKp), Winch::spd_Kp(filter_p.speedKp), Winch::spd_Ki(filter_p.speedKi),
        stop_go(STOP){

}
void WinchDriver::motor_setup() {
    // Make the motor stop
    motor.brake();
}

void WinchDriver::enc_setup() {
    enc = new Encoder(encA, encB);

    enc->write(0);

    enc_pos = 0;

    // Set velocities to 0
    spd_est = 0.0;

    encTimer = 0;
}

void WinchDriver::scale_setup() {
    scale.begin(dout, sck);
    //TODO Fix scale calibration and taring.
    scale.set_scale(2280.f);
    scale.set_offset(scale_offset);
}

void WinchDriver::setup(){

    //scale_setup();
//    char resp[100];
//    sprintf(resp, "Winch %i Setting up motor.", index);
//    wifiResponse(resp);
    motor_setup();

//    sprintf(resp, "Winch %i Setting up encoder.", index);
//    wifiResponse(resp);
    enc_setup();

//    sprintf(resp, "Winch %i Setting up scale.", index);
//    wifiResponse(resp);
    scale_setup();

    /*sprintf(resp, "Winch %i Setting up pid.", index);
    wifiResponse(resp);
    Winch::pid_setup();*/

}

void WinchDriver::motor_loop() {

    // If it is position controlled, calculate the position
    /*if( control_mode == POSITION ) {
        pos_in = (double) enc_pos;
        position.Compute();
    }*/

    if (stop_go == STOP) { // Something about integral windup here.
        motor.brake();
    } /*else if(control_mode == POSITION || control_mode == SPEED ) {
        Winch::pos_in = (double) enc_pos;
        Winch::position->Compute();

        Winch::spd_in = spd_est;
        Winch::speed->Compute();
        motor.drive((int) Winch::spd_out);
    } */ //else if (control_mode == SIGNAL) {
    else {
        motor.drive(signal);
    }

    /*if(printTimer > 1000){
        Serial.printf("PID %i > Position: %i (%i/%i), Speed: %i (%i/%i)\n", index,
                      (int)(Winch::pos_out), (int)(enc_pos), (int)(Winch::pos_setpt),
                      (int) Winch::spd_out, (int)(spd_est), (int)(Winch::pos_out));
        /*Serial.printf("Estimation: Pos: %i, PosErr: %i, SpdInt: %i, Spd: %i\n",
                      (int)(pos_est*100.0), (int)((enc_pos-pos_est)*100.0), (int)(spd_int*100.0), (int)(spd_est*100.0)
        );
        Serial.printf("Position %i > x: %i\n", index, (int)(enc_pos));
        printTimer=0;
    }*/
}

void WinchDriver::enc_loop() {
    // TODO Should this run faster than the control loop?

    double dt = ((double) encTimer);

    enc_pos = ((double)enc->read()) / TICKS_PER_REVOLUTION;

    // Estimate the velocity using a tracking loop (https://www.embeddedrelated.com/showarticle/530.php)
    pos_est += (spd_est * dt) / 1000000.0;
    double pos_est_err = enc_pos - pos_est;
    spd_int += pos_est_err * spd_tracking_ki * dt / 1000000.0;
    spd_est = pos_est_err * spd_tracking_kp + spd_int;

    /*if(printTimer > 1000){
        Serial.printf("Estimation %i > Pos: %i, PosErr: %i, SpdInt: %i, Spd: %i, encTimer: %i, pos_est_error: %i\n", index,
                      (int)(pos_est), (int)(enc_pos - pos_est), (int)(spd_int),
                      (int)(spd_est), (int) encTimer, (int)(pos_est_err)
        );
    }*/

    encTimer = 0;

}

void WinchDriver::scale_loop() {
    if (scale.is_ready())
        tension = scale.get_units(1);
}

void WinchDriver::comm_loop() {

}

void WinchDriver::loop(){
    motor_loop();
    enc_loop();
    scale_loop();
    comm_loop();
}

void WinchDriver::go(){
    stop_go = GO;
}

void WinchDriver::go_signal(int s) {
    stop_go = GO;
    signal = s;
}

void WinchDriver::setSignal(int s) {
    signal = s;
}

void WinchDriver::stop(){ // remove integral windup here
    stop_go = STOP;
    motor.brake();
}

// Winch States
bool WinchDriver::isUnderTension() {
    if(tension > STRING_TENSION_THRESHOLD)
        return true;

    return false;
}