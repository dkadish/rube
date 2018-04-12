//
// Created by David Kadish on 02/03/2018.
//

#ifndef MANUALCONTROLDIAGNOSTICS_MANUALCONTROLDIAGNOSTICS_H
#define MANUALCONTROLDIAGNOSTICS_MANUALCONTROLDIAGNOSTICS_H

#include <Arduino.h>
#include <pindefs.h>
#include <rube_config.h>
#include <wifi.h>
#include <Winch.h>
#include <robot.h>

#define KP 255.0
#define KI 50.0
#define KD 0.0

// Winches
//TODO Is this the right naming
Winch O(0, ENC1A, ENC1B,
        IN1_0, IN2_0, PWM_0, 1, STBY_0,
        DOUT_A, SCK_A, SCALE1_OFFSET,
        KP, KI, KD);
Winch Q(1, ENC2A, ENC2B,
        IN1_1, IN2_1, PWM_1, 1, STBY_1,
        DOUT_B, SCK_B, SCALE2_OFFSET,
        KP, KI, KD);
Winch P(2, ENC3A, ENC3B,
        IN1_2, IN2_2, PWM_2, 1, STBY_2,
        DOUT_C, SCK_C, SCALE3_OFFSET,
        KP, KI, KD);
Winch *winches[3];// = {O, Q, P}; // FIXME: Why is the order this way? B/C the index...
const int N_WINCHES = 3;

// Robot
RobotSetupParameters robotParams = {
    2.5,    /**< The height of the bottom of the mounts from the ground (m). */
    4.7,     /**< The length of the line from O to P (m). */
    4.884,   /**< The length of the line from P to Q (m). */
    3.863,    /**< The length of the line from O to Q (m). */
    2.38, /**< Length of the cable from O to robot (m). */
    3.13, /**< Length of the cable from P to robot (m).*/
    3.54 /**< Length of the cable from Q to robot (m). */
};

RobotPosition position = RobotPosition();

// Communication
Stream* cmdSerial=&Serial; // Commands sent to the robot
Stream* msgSerial=&Serial; // Messages from the robot
Stream* datSerial=&Serial; // Data logs from the robot
Stream* logSerial=&Serial; // Log messages from the

String command = ""; // Commands received over serial

void position_setup();

void robot_loop();
void serial_loop();
void runDiagnostics();
void printSensors();
void handleSerialInput(String serial_in);

// Configuration
void printConfig();
void tareScales();

// Winch Manual Control Functions
void doWinchStop(int winch_i);
void doSetWinchSignal(int winch_i, int signal);
void doSetWinchPositionSetpoint(int winch_i, float position);
void doDisplayWinchState(int winch_i);
void doTensionLine(int winch_i);
void doRelaxLine(int winch_i);
void doRampUp(int winch_i, int ms);
void doRampDown(int winch_i, int ms);
void _doRamp(int winch_i, int ms);

int decimalDigits(float number);

#endif //MANUALCONTROLDIAGNOSTICS_MANUALCONTROLDIAGNOSTICS_H
