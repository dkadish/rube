//
// Created by David Kadish on 2017-07-05.
//

// Printing
#include <Metro.h>
Metro printTimer = Metro(500);

#include <elapsedMillis.h>
elapsedMicros debounceTimer;
elapsedMicros lastRead;
elapsedMillis pauseTimer;

// Load Cell
#include "HX711.h"
#define LC_DAT 14
#define LC_CLK 15
HX711 scale(LC_DAT, LC_CLK);
float calibration_factor = 73.0; //-7050 worked for my 440lb max scale setup

// Encoder
#include <Encoder.h>
Encoder enc1(5, 6);

// Loop initializations
void enc_loop();
void motor_loop();
void lc_loop();

// Motor Control
// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include <SparkFun_TB6612.h>

#define M1_STBY	11 //Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
// Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
#define M1_AIN1 8
#define M1_BIN1 10
//Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
#define M1_AIN2 7
#define M1_BIN2 9
//PWM input for channels A/B	Input	PWM input that controls the speed
#define M1_PWMA 3
#define M1_PWMB 4

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(M1_AIN1, M1_AIN2, M1_PWMA, offsetA, M1_STBY);
Motor motor2 = Motor(M1_BIN1, M1_BIN2, M1_PWMB, offsetB, M1_STBY);

// PID
#include <PID_v1.h>
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
bool dirUp; // Is the motor travelling up

//Specify the links and initial tuning parameters
double Kp=10.0, Ki=1.0, Kd=0.0;
PID posPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

void setup() {
    Serial.begin(9600);
    delay(1000);

    // Load Cell
    scale.set_scale();
    scale.tare(); //Reset the scale to 0
    long zero_factor = scale.read_average(); //Get a baseline reading
    Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
    Serial.println(zero_factor);

    motor1.brake();
    motor2.brake();

    Setpoint = 500.0;
    dirUp = false;
    posPID.SetMode(AUTOMATIC);
    posPID.SetOutputLimits(-255, 255);

    printTimer.reset();
}

long pos1  = -999;

void loop() {
    //enc_loop();
    //motor_loop();
    lc_loop();

    // if a character is sent from the serial monitor,
    // reset both back to zero.
    /*if (Serial.available()) {
        int newSetpt = Serial.parseInt();
        Serial.printf("New Setpoint %i", newSetpt);
        Serial.println();
        Setpoint = (double) newSetpt;
        Serial.read();
    }*/

    if( pos1 <= 1 && dirUp ){ // Hit the top, switch direction
        pauseTimer = 0;
        while (pauseTimer < 10000L){
            enc_loop();
            motor_loop();
        }
        Setpoint = 500.0;
        dirUp = false;
    } else if ( pos1 >= 499 && !dirUp ){
        pauseTimer = 0;
        while (pauseTimer < 10000L){
            enc_loop();
            motor_loop();
        }
        Setpoint = 0.0;
        dirUp = true;
    }
}

void enc_loop() {
    if( debounceTimer > 250 ){
        debounceTimer = 0;
        long newPos1;
        newPos1 = enc1.read();
        if (newPos1 != pos1) {
            Serial.print(newPos1);
            Serial.print(",");
            Serial.print(newPos1-pos1);
            Serial.print(",");
            Serial.print(1000.0*((float)newPos1-pos1)/((float)lastRead));
            Serial.println();
            pos1 = newPos1;
            lastRead = 0;
        }
    }
}

void motor_loop(){

    Input = (double) pos1;
    posPID.Compute();
    motor1.drive((int) Output);
}

void lc_loop(){
    scale.set_scale(calibration_factor); //Adjust to this calibration factor

    Serial.print("Reading: ");
    Serial.print(scale.get_units(), 1);
    Serial.print(" g"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
    Serial.print(" calibration_factor: ");
    Serial.print(calibration_factor);
    Serial.println();

    if(Serial.available())
    {
        char temp = Serial.read();
        if(temp == '+' || temp == 'a')
            calibration_factor += 1;
        else if(temp == '-' || temp == 'z')
            calibration_factor -= 1;
    }
}