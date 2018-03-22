//
// Created by David Kadish on 18/10/2017.
//

#ifndef FIRMWARE_WINCH_H
#define FIRMWARE_WINCH_H

#include "Geometry.h"
#include "WinchDriver.h"
#include "WinchController.h"

#include <HX711.h>
#include <PID_v1.h>
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
#define METRES_PER_REVOLUTION (0.01*2*3.14159) // A = r * theta
#endif
#ifdef __MK20DX256__ // Teensy 3.2 (Test Rig)
#define MOTOR_UPPER_LIMIT 100
#define MOTOR_LOWER_LIMIT -255

#define POSITION_UPPER_LIMIT 20000
#define POSITION_LOWER_LIMIT -20000

// Used to calculate velocity from encoder motion and microseconds
#define VELOCITY_SCALE_FACTOR 100000.0
#endif

#define POSITION 0
#define SPEED 1
#define SIGNAL 2

struct PositionParams {
    Point3D origin;
    float length;
};

struct FilterParams {
    double positionKp;
    double speedKp;
    double speedKi;
};

class Winch : public WinchDriver {

    double spd_setpt;

public:

    Winch(int index, int encA, int encB,
                int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
                int scale_dout, int scale_sck, long scale_offset,
                float positionKp, float speedKp, float speedKi
    );

    Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                ScaleParams scale_p, FilterParams filter_p);

    Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                ScaleParams scale_p, FilterParams filter_p, PositionParams pos_p);

    // Main Functions
    void setup();
    void loop();

    // Controllers
    MinimumMotionController mm_ctrl;
    TensionMaintenanceController tension_ctrl;
    RetensioningController retension_ctrl;

    // Position Variables
    Point3D origin = {0.0,0.0,0.0};
    double startLength = 0.0;
    double getLength();

    // Internal Variables
    int cycles;
    int index; // Which winch am I? [0 == A, 1 == B, ...]

    // Motor
    int offset;


    int control_mode = POSITION; // Is it position- or speed-controlled

    double current_position(); /**< Current position in number of revolutions */
    void go_signal(int signal);
    void stop();
    void go();

    void pid_loop();

    void pid_setup();

    // Define the parameters
    double pos_Kp;
    // PID
    //Define Variables we'll be connecting to
    double pos_in;
    // PID
    //Define Variables we'll be connecting to
    double pos_out;
    // PID
    //Define Variables we'll be connecting to
    double pos_setpt;
    //Specify the links and initial tuning parameters
    PID * position;
    // Define the parameters
    double spd_Ki;
    // Define the parameters
    double spd_Kp;
    double spd_in;
    double spd_out;
    PID * speed;
};

#endif //FIRMWARE_WINCH_H
