//
// Created by David Kadish on 18/10/2017.
//

#ifndef FIRMWARE_WINCH_H
#define FIRMWARE_WINCH_H

#include "Geometry.h"
#include "HX711.h"
#include <Encoder.h>
#include <elapsedMillis.h>
#include <SparkFun_TB6612.h>
#include <PID_v1.h>

#if defined(__MK66FX1M0__) || defined(__MK64FX512__)
#define MOTOR_UPPER_LIMIT 150
#define MOTOR_LOWER_LIMIT -255

#define POSITION_UPPER_LIMIT 100
#define POSITION_LOWER_LIMIT -100

// Used to calculate velocity from encoder motion and microseconds
#define VELOCITY_SCALE_FACTOR 1000.0
#define TICKS_PER_REVOLUTION 1024 // TODO is this right
#define METRES_PER_REVOLUTION (0.01*2*3.14159) // A = r * theta
#endif
#ifdef __MK20DX256__ // Teensy 3.2 (Test Rig)
#define MOTOR_UPPER_LIMIT 100
#define MOTOR_LOWER_LIMIT -255

#define POSITION_UPPER_LIMIT 20000
#define POSITION_LOWER_LIMIT -20000

// Used to calculate velocity from encoder motion and microseconds
#define VELOCITY_SCALE_FACTOR 100000.0
#define TICKS_PER_REVOLUTION 1024.0 // TODO is this right
#endif

#define POSITION 0
#define SPEED 1
#define SIGNAL 2
#define STOP 0
#define GO 1

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

struct FilterParams {
    double positionKp;
    double speedKp;
    double speedKi;
};

struct PositionParams {
    Point3D origin;
    float length;
};

class Winch {

public:
    Winch(int index, int encA, int encB,
          int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
          int scale_dout, int scale_sck, long scale_offset,
          double positionKp, double speedKp, double speedKi
        );

    Winch(int index, EncoderParams enc_p, MotorParams motor_p,
          ScaleParams scale_p, FilterParams filter_p);

    Winch(int index, EncoderParams enc_p, MotorParams motor_p,
          ScaleParams scale_p, FilterParams filter_p, PositionParams pos_p);


    // Position Variables
    Point3D origin = {0.0,0.0,0.0};
    double startLength = 0.0;
    double getLength();

    // Internal Variables
    int cycles;
    int index; // Which winch am I? [0 == A, 1 == B, ...]

    // Motor
    int offset;
    Motor motor;

    // Encoder
    Encoder* enc;
    int encA, encB;
    double enc_pos; // Encoder position, in revolutions. 1 rev = 1.0.
    //float last_pos;
    double enc_speed; // Encoder speed in revolutions/sec.
    elapsedMicros encTimer;

    // Scale
    HX711 scale;
    long scale_offset;
    float tension; // Stores the tension on the line. Updated in loop. Measured in no particular unit at the moment.

    // PID
    //Define Variables we'll be connecting to
    double pos_setpt, pos_in, pos_out;
    double spd_setpt, spd_in, spd_out;
    // Define the parameters
    double pos_Kp, spd_Kp, spd_Ki;

    int control_mode = POSITION; // Is it position- or speed-controlled
    int stop_go = STOP; // Is it in stop mode or go mode

    int signal = 0;

    //Specify the links and initial tuning parameters
    PID * position;//(&pos_in, &pos_out, &pos_setpt, 0.0, 0.0, 0.0, DIRECT);
    PID * speed;//(&spd_in, &spd_out, &spd_setpt, 0.0, 0.0, 0.0, DIRECT);

    // Estimation (see https://www.embeddedrelated.com/showarticle/530.php)
    double pos_est = 0; // Position estimate
    double spd_est = 0; // Speed estimate
    double spd_int = 0; // Speed integrator
    double spd_tracking_ki = 5.0;
    double spd_tracking_kp = 10.0;

    void setup();
    void loop();

    double current_position(); /**< Current position in number of revolutions */
    void go_signal(int signal);
    void stop();
    void go();

    // Winch Functions
    bool isUnderTension();

private:
    // Setup functions
    void motor_setup();
    void enc_setup();
    void scale_setup();
    void pid_setup();

    // Loop functions
    void motor_loop();
    void enc_loop();
    void scale_loop();
    void pid_loop();
    void comm_loop();

    // Scale Variables
    int dout, sck;

    // Printing variables
    long printTimer = 0;
    //elapsedMillis printTimer = 0;
};


#endif //FIRMWARE_WINCH_H
