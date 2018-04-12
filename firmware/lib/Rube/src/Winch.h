//
// Created by David Kadish on 18/10/2017.
//

#ifndef FIRMWARE_WINCH_H
#define FIRMWARE_WINCH_H

#include "Geometry.h"
#include "WinchDriver.h"
#include "WinchController.h"
#include "rube_config.h"

#include <HX711.h>
#include <Encoder.h>
#include <elapsedMillis.h>
#include <SparkFun_TB6612.h>

struct PositionParams {
    Point3D origin;
    float length;
};

class Winch {

    WinchDriver driver;
    elapsedMillis printTimer=0;

    PIDParams pid_p;

public:

    Winch(int index, int encA, int encB,
          int motorIn1, int motorIn2, int motorPwm, int motorOffset, int motorStby,
          int scale_dout, int scale_sck, long scale_offset,
          double positionKp, double speedKp, double speedKi
    );

    Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                ScaleParams scale_p, PIDParams pid_p);

    Winch(int index, EncoderParams enc_p, MotorParams motor_p,
                ScaleParams scale_p, PIDParams pid_p, PositionParams pos_p);

    // Main Functions
    void setup();

    void loop();

    // Controllers
    MinimumMotionController *mm_ctrl;
    TensionMaintenanceController *tension_ctrl;
    RetensioningController *retension_ctrl;
    PIDPositionController *pidPos_ctrl;

    const static int n_controllers = 4;
    Controller *controllers[n_controllers]; // = {&mm_ctrl, &tension_ctrl, &retension_ctrl};

    // Motion commands
    void doRetension(){ retension_ctrl->start(); }

    void doSlowUp(){ mm_ctrl->setDirection(true); mm_ctrl->start(); }
    void doSlowDown(){ mm_ctrl->setDirection(false); mm_ctrl->start(); }
    void doStop();
    void doGoTo(float position){ pidPos_ctrl->setTarget(position); pidPos_ctrl->start(); }
    void doGoTo(float position, float stoppingError) { /**< Go to a position, stop when nearby */
        pidPos_ctrl->setTarget(position);
        pidPos_ctrl->start();
        pidPos_ctrl->setStopDistance((double)stoppingError);
    }
    void doHoldAt(float position, float stoppingError) { /**< Go to a position, hold when nearby */
        pidPos_ctrl->setTarget(position);
        pidPos_ctrl->start();
        pidPos_ctrl->setHoldDistance((double) stoppingError);
    }


    // Position Variables
    Point3D origin = {0.0,0.0,0.0};

    double startLength = 0.0;
    float getLength();
    // Internal Variables
    int cycles;

    int index; // Which winch am I? [0 == A, 1 == B, ...]
    // Motor
    int offset;

    float getPosition(); /**< Current position in distance from start */

    float getSpeed(){ driver.getSpeed(); }
    int getSignal(){ driver.getSignal(); }

    //*** Driver access functions ***//
    // Scale
    void scaleTare(){ driver.scale.tare(10); } /**< Zero the scale */
    long getScaleOffset(){ return driver.scale.get_offset(); } /**< Get the offset from a tared scale */

    // Encoder
    long getEncoderTicks(){ return driver.getEncoderTicks(); } /**< Get the number of ticks of the quadrature encoder */

    // Tension
    bool isUnderTension(){ return driver.isUnderTension(); }; /**< Check if the winch is under tension. */
    float getTension(){ return driver.getTension(); } /**< Get the current tension value */

    // Motor
    void doGo(){ driver.go(); };
    void doGo(int signal){ driver.go_signal(signal); };

};

#endif //FIRMWARE_WINCH_H
