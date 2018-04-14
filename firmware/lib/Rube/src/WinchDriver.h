//
// Created by David Kadish on 09/03/2018.
//

#ifndef RUBE_WINCHDRIVER_H
#define RUBE_WINCHDRIVER_H


#include <SparkFun_TB6612.h>
#include <Encoder.h>
#include <HX711.h>
#include <elapsedMillis.h>

struct EncoderParams {
    int A;
    int B;
};

struct MotorParams {
    int in1;
    int in2;
    int pwm;
    int offset;
    int standby;
};

struct ScaleParams {
    int dout;
    int sck;
    int offset;
};

class WinchDriver {

public:
    WinchDriver(int encA, int encB,
                int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
                int scale_dout, int scale_sck, long scale_offset
                //float positionKp, float speedKp, float speedKi
    );

    WinchDriver(EncoderParams enc_p, MotorParams motor_p,
                ScaleParams scale_p
                //PIDParams filter_p
    );

    WinchDriver();

    enum State {
        STOP,
        GO
    };

    Motor motor;
    // Encoder
    Encoder* enc;
    // Scale
    HX711 scale;
    long scale_offset;

    void setup();

    int signal = 0;

    void loop();

    // Control functions
    void stop();
    void go();
    void go_signal(int signal);
    void setSignal(int signal);
    int getSignal(){ return signal; }

    // Winch Functions
    bool isUnderTension();
    float getTension(){ return tension; }

    //TODO: Should not be getting from estimate here and from tracker in Winch
    //float getPosition(){ return pos_est; }
    float getEncoderTurns(){ return enc_turns; }
    long getEncoderTicks(){ return enc->read(); }
    //float getSpeed(){ return spd_est; }

    bool isOn(){return stop_go == State::GO; }

private:
    // Setup functions
    void motor_setup();

    void enc_setup();

    void scale_setup();

    // Loop functions
    void motor_loop();

    void enc_loop();

    void scale_loop();

    void comm_loop();

    State stop_go = State::STOP ; // Is it in stop mode or go mode

    int encA, encB; /**< Pins for the magnetic encoder readings */
    double enc_turns; // Encoder position, in revolutions. 1 rev = 1.0.
    //double enc_speed; // Encoder speed in revolutions/sec.
    //elapsedMicros encTimer;

    // Scale Variables
    int dout, sck;

    //(&pos_in, &pos_out, &pos_setpt, 0.0, 0.0, 0.0, DIRECT);
    //(&spd_in, &spd_out, &spd_setpt, 0.0, 0.0, 0.0, DIRECT);

    // Estimation (see https://www.embeddedrelated.com/showarticle/530.php)
    /*double pos_est = 0; // Position estimate
    double spd_est = 0; // Speed estimate
    double spd_int = 0; // Speed integrator
    double spd_tracking_ki = 5.0;
    double spd_tracking_kp = 10.0;*/

    // Printing variables
    long printTimer = 0;

    float tension; // Stores the tension on the line. Updated in loop. Measured in no particular unit at the moment.
};


#endif //RUBE_WINCHDRIVER_H
