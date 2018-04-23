//
// Created by David Kadish on 18/10/2017.
//

#ifndef FIRMWARE_WINCH_H
#define FIRMWARE_WINCH_H

#include "Geometry.h"
#include "WinchDriver.h"
#include "WinchController.h"
#include "rube_config.h"

#include <Arduino.h>
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

    Point3D origin = {0.0,0.0,0.0};

    /** Convert from line length (distance from line origin to robot) in metres
     *  to the number of turns from the starting position.
     *
     * @param length
     * @return
     */
    float LengthToTurns(float length){
        INFO("Relocating to %i.%i, position %i.%i", FLOAT(length), FLOAT((length - startLength) / METRES_PER_REVOLUTION))
        return (length - startLength) / METRES_PER_REVOLUTION;
    }

    bool errorCondition = false;

public:

    Winch(int index, int encA, int encB, int encIDX,
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
    void doGoTo(float length){ pidPos_ctrl->setTarget(LengthToTurns(length)); pidPos_ctrl->start(); }
    void doGoTo(float length, float stoppingError) { /**< Go to a position, stop when nearby */
        pidPos_ctrl->setTarget(LengthToTurns(length));
        pidPos_ctrl->start();
        pidPos_ctrl->setStopDistance((double)stoppingError);
    }
    void doHoldAt(float length, float stoppingError) { /**< Go to a position, hold when nearby */
        pidPos_ctrl->setTarget(LengthToTurns(length));
        pidPos_ctrl->start();
        pidPos_ctrl->setHoldDistance((double) stoppingError);
    }

    bool getErrorCondition();

    // Position Variables

    float startLength = 0.0;
    /**< Current position in distance from start
     * Up (shortening the line) is negative.
     */
    float getPosition(){ return driver.getEncoderTurns()*METRES_PER_REVOLUTION; }
    float getLength(){ return startLength + getPosition(); } /**< Get the length of the winch line. */
    void setStartLength(float length){ startLength = length; } /**< Set the initial length of the winch line. */
    void setOrigin(Point3D o){origin = o;}
    void setOrigin(float x, float y, float z){origin.x = x; origin.y = y; origin.z = z;}
    Point3D getOrigin(){ return origin; }

    // Internal Variables
    int cycles;
    int index; // Which winch am I? [0 == A, 1 == B, ...]

    // Motor
    int offset;

    //float getSpeed(){ return driver.getSpeed(); }
    int getSignal(){
        if( driver.isOn() ) {
            return driver.getSignal();
        } else {
            return 0;
        }
    }

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

    String print(){
        // Time
        String winch_print = millis();
        winch_print += ", ";

        // Winch Info
        winch_print += "Winch: ";
        winch_print += index;
        winch_print += ", ";

        // Position/Encoding
        winch_print += "P: ";
        winch_print += getPosition();
        winch_print += ", ";
        winch_print += getLength();
        winch_print += ", ";
        winch_print += getEncoderTicks();
        winch_print += ", ";
        winch_print += driver.getEncoderTurns();

        // Motor
        winch_print += ", M: ";
        winch_print += driver.isOn();
        winch_print += ", ";
        winch_print += getSignal();
        winch_print += ", ";
        winch_print += driver.getSignal();

        // Scale
        winch_print += ", S: ";
        winch_print += getTension();
        winch_print += ", ";
        winch_print += isUnderTension();

        // PID
        winch_print += ", D: ";
        winch_print += pidPos_ctrl->isEnabled();
        winch_print += ", ";
        winch_print += pidPos_ctrl->getTarget();
        winch_print += ", ";
        winch_print += pidPos_ctrl->getError();

        return winch_print;
    }

};

#endif //FIRMWARE_WINCH_H
