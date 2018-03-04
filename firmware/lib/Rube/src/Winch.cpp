//
// Created by David Kadish on 18/10/2017.
//

// TODO: Motion planning based on ETA using estimated velocity.

#include "Winch.h"
#include "wifi.h"
#include "config.h"
#include "../../../../../../../../.platformio/packages/framework-arduinoteensy/cores/teensy3/elapsedMillis.h"

Winch::Winch(int index, int encA, int encB,
             int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
             int scale_dout, int scale_sck, long offset,
             double positionKp, double speedKp, double speedKi
    ): index(index), encA(encA), encB(encB), motor(motorIn1, motorIn2, motorPwm, motorOffset, motorStby),
       dout(scale_dout), sck(scale_sck), scale_offset(offset),
       pos_Kp(positionKp), spd_Kp(speedKp), spd_Ki(speedKi),
       //position(&pos_in, &pos_out, &pos_setpt, positionKp, 0.0, 0.0, DIRECT),
       //speed(&spd_in, &spd_out, &pos_out, speedKp, speedKi, 0.0, DIRECT),
       stop_go(STOP)
{

}

void Winch::motor_setup() {
    // Make the motor stop
    motor.brake();
}

void Winch::enc_setup() {
    enc = new Encoder(encA, encB);

    enc->write(0);

    enc_pos = 0;

    // Set velocities to 0
    spd_est = 0.0;

    encTimer = 0;
}

void Winch::pid_setup() {

    position = new PID(&pos_in, &pos_out, &pos_setpt, pos_Kp, 0.0, 0.0, DIRECT);
    position->SetMode(AUTOMATIC);
    position->SetOutputLimits(POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);

    speed = new PID(&spd_in, &spd_out, &pos_out, spd_Kp, spd_Ki, 0.0, DIRECT);
    speed->SetMode(AUTOMATIC);
    speed->SetOutputLimits(MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);

}

void Winch::scale_setup() {
    scale.begin(dout, sck);
    //TODO Fix scale calibration and taring.
    scale.set_scale(2280.f);
    scale.set_offset(scale_offset);
}

void Winch::setup(){

    //scale_setup();
    char resp[100];
    sprintf(resp, "Winch %i Setting up motor.", index);
    wifiResponse(resp);
    motor_setup();

    sprintf(resp, "Winch %i Setting up encoder.", index);
    wifiResponse(resp);
    enc_setup();

    sprintf(resp, "Winch %i Setting up scale.", index);
    wifiResponse(resp);
    scale_setup();

    sprintf(resp, "Winch %i Setting up pid.", index);
    wifiResponse(resp);
    pid_setup();

}

void Winch::go(){
    stop_go = GO;
}

void Winch::go_signal(int s) {
    control_mode = SIGNAL;
    stop_go = GO;
    signal = s;
}

void Winch::stop(){ // remove integral windup here
    stop_go = STOP;

    control_mode = POSITION;
    motor.brake();
    pos_setpt = current_position();
}

void Winch::motor_loop() {

    // If it is position controlled, calculate the position
    /*if( control_mode == POSITION ) {
        pos_in = (double) enc_pos;
        position.Compute();
    }*/

    if (stop_go == STOP) { // Something about integral windup here.
        motor.brake();
    } else if( control_mode == POSITION || control_mode == SPEED ) {
        pos_in = (double) enc_pos;
        position->Compute();

        spd_in = spd_est;
        speed->Compute();
        motor.drive((int) spd_out);
    } else if (control_mode == SIGNAL) {
        motor.drive(signal);
    }

    if(printTimer > 1000){
        Serial.printf("PID %i > Position: %i (%i/%i), Speed: %i (%i/%i)\n", index,
                      (int)(pos_out), (int)(enc_pos), (int)(pos_setpt),
                      (int)spd_out, (int)(spd_est), (int)(pos_out));
        /*Serial.printf("Estimation: Pos: %i, PosErr: %i, SpdInt: %i, Spd: %i\n",
                      (int)(pos_est*100.0), (int)((enc_pos-pos_est)*100.0), (int)(spd_int*100.0), (int)(spd_est*100.0)
        );*/
        Serial.printf("Position %i > x: %i\n", index, (int)(enc_pos));
        printTimer=0;
    }
}

void Winch::enc_loop() {
    // TODO Should this run faster than the control loop?

    double dt = ((double)encTimer);

    enc_pos = ((double)enc->read())/TICKS_PER_REVOLUTION;

    /*if( (micros > 100000) ) {

        enc_speed = VELOCITY_SCALE_FACTOR*((double) (enc_pos - last_pos)) / ((double) micros);

        /*if( enc_pos != last_pos ){
            Serial.printf("Scale factor: %i, PDiff: %i, Micros: %i, Speed: %i.%i\n",
                          (int)VELOCITY_SCALE_FACTOR, (enc_pos - last_pos), (int)micros, (int)enc_speed, ((int)(enc_speed*100.0))%100
            );
        }

        encTimer = 0;
        last_pos = enc_pos;
    }*/

    // Estimate the velocity using a tracking loop (https://www.embeddedrelated.com/showarticle/530.php)
    pos_est += (spd_est * dt) / 1000000.0;
    double pos_est_err = enc_pos - pos_est;
    spd_int += pos_est_err * spd_tracking_ki * dt / 1000000.0;
    spd_est = pos_est_err * spd_tracking_kp + spd_int;

    if(printTimer > 1000){
        Serial.printf("Estimation %i > Pos: %i, PosErr: %i, SpdInt: %i, Spd: %i, encTimer: %i, pos_est_error: %i\n", index,
                      (int)(pos_est), (int)(enc_pos-pos_est), (int)(spd_int),
                      (int)(spd_est), (int)encTimer, (int)(pos_est_err)
        );
    }

    encTimer = 0;

}

void Winch::scale_loop() {
    tension = scale.get_units();

    if (printTimer > 1000) {
        Serial.printf("Weight %i: ", index);
        Serial.println(scale.get_units());
    }
}

void Winch::pid_loop() {
}

void Winch::comm_loop() {

}

void Winch::loop(){
//    elapsedMicros loopTimer=0;
    motor_loop();
//    Serial.print(loopTimer);
//    Serial.println(" microseconds.");
//    loopTimer=0;
    enc_loop();
//    Serial.print(loopTimer);
//    Serial.println(" microseconds.");
//     loopTimer=0;
    scale_loop();
//    Serial.print(loopTimer);
//    Serial.println(" microseconds.");
//     loopTimer=0;
    pid_loop();
//    Serial.print(loopTimer);
//    Serial.println(" microseconds.");
//     loopTimer=0;
    comm_loop();
//    Serial.print("Loop took ");
//    Serial.print(loopTimer);
//    Serial.println(" microseconds.");
}

double Winch::current_position() {
    return ((double)enc->read())/TICKS_PER_REVOLUTION;
}

// Winch States
bool Winch::isUnderTension() {
    if(tension > STRING_TENSION_THRESHOLD)
        return true;

    return false;
}