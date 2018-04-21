//
// Created by David Kadish on 09/03/2018.
//

#include "WinchDriver.h"
#include "wifi.h"
#include "logging.h"
#include "rube_config.h"
#include <Encoder.h>
#include <usb_serial.h>
#include <Arduino.h>

// Index Interrupt Functions
/*volatile bool idx_tick_23=false, idx_tick_24=false, idx_tick_36=false;
void isr23(){ idx_tick_23 = true; };
void isr24(){ idx_tick_24 = true; };
void isr36(){ idx_tick_36 = true; };*/

WinchDriver::WinchDriver(int encA, int encB, int IDX,
                          int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
                          int scale_dout, int scale_sck, long offset
                          //float positionKp, float speedKp, float speedKi
): encA(encA), encB(encB), encIDX(IDX), motor(motorIn1, motorIn2, motorPwm, motorOffset, motorStby),
   dout(scale_dout), sck(scale_sck), scale_offset(offset),
   //Winch::pos_Kp(positionKp), Winch::spd_Kp(speedKp), Winch::spd_Ki(speedKi),
   stop_go(STOP)
{

}

WinchDriver::WinchDriver(EncoderParams enc_p, MotorParams motor_p,
                          ScaleParams scale_p
                          //PIDParams filter_p
):
        encA(enc_p.A), encB(enc_p.B), encIDX(enc_p.IDX),
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
    INFO("Setting up encoder.")
    enc = new Encoder(encA, encB);

    enc->write(0);

    /*pinMode(encIDX, INPUT_PULLUP);
    INFO("Attaching interrupt to pin %i.", encIDX)
    switch(encIDX) {
        case 23:
            attachInterrupt(digitalPinToInterrupt(23), isr23, RISING);
            INFO("Attached interrupt to pin 23.")
            break;
        case 24:
            attachInterrupt(digitalPinToInterrupt(24), isr24, RISING);
            INFO("Attached interrupt to pin 24.")
            break;
        case 36:
            attachInterrupt(digitalPinToInterrupt(36), isr36, RISING);
            INFO("Attached interrupt to pin 36.")
            break;
    }*/

    // Set velocities to 0
    //spd_est = 0.0;

    //encTimer = 0;
}

void WinchDriver::scale_setup() {
    scale.begin(dout, sck);
    //TODO Fix scale calibration and taring.
    scale.set_scale(2280.f);
    scale.set_offset(scale_offset);
}

void WinchDriver::setup(){

    motor_setup();

    enc_setup();

    scale_setup();

}

void WinchDriver::motor_loop() {


    if (stop_go == STOP) { // Something about integral windup here.
        motor.brake();
    }
    else {
        motor.drive(signal);
    }

}

void WinchDriver::StopEncoder() {
    INFO("Encoder is stopped at %i ticks.", encoder.ticks)
    encoder.active = false;
}

void WinchDriver::StartEncoder() {
    INFO("Encoder is starting. Resetting encoder from %i to %i ticks.", enc->read(), encoder.ticks)
    encoder.active = true;
    //encoder.index_sync = false;

    enc->write((int32_t) encoder.ticks);
}

void WinchDriver::enc_loop() {

    //double dt = ((double) encTimer);

    // Ignore everything if the encoder is not active.
    if(encoder.active){
        long ticks = enc->read();

        // Verify the number
        long change = ticks - encoder.ticks;
        if( change > MAX_TICKS_PER_LOOP || change < -MAX_TICKS_PER_LOOP ){
            WARNING("Jump in encoder detected. Ticks moved from %i to %i in one loop, a change of %i. Ignoring movement.", encoder.ticks, ticks, change);
            // Ignore the recorded motion and reset the ticks to last count
            enc->write(encoder.ticks); // + encoder.direction); // Also add one in the direction of motion
        } else {
            // Update ticks
            encoder.ticks_prev = encoder.ticks;
            encoder.ticks = ticks;

            // Update direction
            int new_direction = 0;
            if(encoder.ticks > encoder.ticks_prev){ new_direction = 1; }
            else if(encoder.ticks < encoder.ticks_prev){ new_direction = -1; }

            if(encoder.direction != new_direction && new_direction != 0){
                DEBUG("Changed direction from %i to %i.", encoder.direction, new_direction)
                encoder.direction = new_direction;
            }

        }

        // Check for index
        /*switch (encIDX){
            case 23:
                if(idx_tick_23){
                    encoder.index_tick = true;
                    idx_tick_23 = false;
                }
                break;
            case 24:
                if(idx_tick_24){
                    encoder.index_tick = true;
                    idx_tick_24 = false;
                }
                break;
            case 36:
                if(idx_tick_36){
                    encoder.index_tick = true;
                    idx_tick_36 = false;
                }
                break;
        }

        if( encoder.index_tick ){
            INFO("Encoder index captured. Index %i.", encoder.index)
            if( encoder.index_sync ){ // Check to see if index is in sync
                // Check number of ticks since last sync
                long idx_change = encoder.ticks - encoder.index_prev_tick;
                bool jump_up = idx_change > TICKS_PER_REVOLUTION + 10;
                bool jump_down = idx_change < - TICKS_PER_REVOLUTION - 10;
                bool jump
                if(  ){ // FIXME Test for unreasonable jumps
                    WARNING("Jump in encoder detected by index. Ticks moved from %i to %i in one index, a change of %i. The count is off.", encoder.index_prev_tick, encoder.ticks, idx_change);
                    if(idx_change > 0){
                        // Reset the encoder count and ticks
                    } else {
                        // Reset the encoder count and ticks
                    }
                } else {
                    if(encoder.direction > 0) {
                        encoder.index++;
                    } else {
                        encoder.index--;
                    }
                }
            } else {
                INFO("Encoder index sync starting.")
                encoder.index_sync = true;
                encoder.index_prev_tick = encoder.ticks;
                if(encoder.direction > 0) {
                    encoder.index++;
                } else {
                    encoder.index--;
                }
            }

            encoder.index_tick = false;
        }

        encoder.turns = ((float)encoder.ticks) / ((float)TICKS_PER_REVOLUTION);*/
    }

    // Estimate the velocity using a tracking loop (https://www.embeddedrelated.com/showarticle/530.php)
    /*pos_est += (spd_est * dt) / 1000000.0;
    double pos_est_err = enc_turns - pos_est;
    spd_int += pos_est_err * spd_tracking_ki * dt / 1000000.0;
    spd_est = pos_est_err * spd_tracking_kp + spd_int;*/

    /*if(printTimer > 1000){
        Serial.printf("Estimation %i > Pos: %i, PosErr: %i, SpdInt: %i, Spd: %i, encTimer: %i, pos_est_error: %i\n", index,
                      (int)(pos_est), (int)(enc_turns - pos_est), (int)(spd_int),
                      (int)(spd_est), (int) encTimer, (int)(pos_est_err)
        );
    }*/

    //encTimer = 0;

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