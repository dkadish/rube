/******************************************************************************
 *
 * Created by David Kadish on 2017-07-05.
 *
 *
 *
 * TODO: Wire load cell circuit into place.

******************************************************************************/

// Printing
#include <Metro.h>
Metro printTimer = Metro(500);

#include <elapsedMillis.h>
elapsedMicros debounceTimer1, debounceTimer2, debounceTimer3;
elapsedMicros lastRead1, lastRead2, lastRead3;
elapsedMillis pauseTimer;
elapsedMillis requestTimer;
elapsedMillis motorPrintTimer;

// Load Cell
#include "HX711.h"
#define LC_DAT 14
#define LC_CLK 15
HX711 scale(LC_DAT, LC_CLK);
float calibration_factor = -7050.0; //-7050 worked for my 440lb max scale setup

// Encoder
#include <Encoder.h>
#ifdef  __MK66FX1M0__
#define ENC1A 23
#define ENC1B 22

#define ENC2A 21
#define ENC2B 20

#define ENC3A 19
#define ENC3B 18

#endif
#ifdef __MK20DX256__
#define ENC1A 5
#define ENC1B 6

#define ENC2A ENC1A
#define ENC2B ENC1B

#define ENC3A ENC1A
#define ENC3B ENC1B
#endif

Encoder enc1(ENC1A, ENC1B);
Encoder enc2(ENC2A, ENC2B);
Encoder enc3(ENC3A, ENC3B);

// Loop initializations
void enc_loop();
void motor_loop();
void lc_calibration_loop();

// Motor Control
// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include <SparkFun_TB6612.h>

#define MOTOR_UPPER_LIMIT 75
#define MOTOR_LOWER_LIMIT -255
#ifdef __MK66FX1M0__
    // Pin settings for teensy 3.6 on stripboard

    //********************** MOTOR DRIVER 1 **********************//
    #define M1_STBY	10 //Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
    // Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M1_AIN1 9
    #define M1_BIN1 11
    //Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M1_AIN2 8
    #define M1_BIN2 12
    //PWM input for channels A/B	Input	PWM input that controls the speed
    #define M1_PWMA 3
    #define M1_PWMB 4


    //********************** MOTOR DRIVER 2 **********************//
    #define M2_STBY	26 //Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
    // Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M2_AIN1 25
    //#define M2_BIN1 27
    //Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M2_AIN2 24
    //#define M2_BIN2 28
    //PWM input for channels A/B	Input	PWM input that controls the speed
    #define M2_PWMA 29
    //#define M2_PWMB 30

#endif
#ifdef __MK20DX256__
    // Pin settings for teensy 3.2 on solderless breadboard
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

    // Pin settings for teensy 3.2 on solderless breadboard
    #define M2_STBY	M1_STBY	 //Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
    // Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M2_AIN1 M1_AIN1
    //#define M2_BIN1 M1_BIN1
    //Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M2_AIN2 M1_AIN2
    //#define M2_BIN2 M1_BIN2
    //PWM input for channels A/B	Input	PWM input that controls the speed
    #define M2_PWMA M1_PWMA
    //#define M2_PWMB M2_PWMB
#endif

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offset1A = -1;
const int offset1B = -1;
const int offset2A = -1;
//const int offset2B = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguments you can either write new functions or
// call the function more than once.
Motor motor1A = Motor(M1_AIN1, M1_AIN2, M1_PWMA, offset1A, M1_STBY);
Motor motor1B = Motor(M1_BIN1, M1_BIN2, M1_PWMB, offset1B, M1_STBY);
Motor motor2A = Motor(M2_AIN1, M2_AIN2, M2_PWMA, offset2A, M2_STBY);
//Motor motor2B = Motor(M2_BIN1, M2_BIN2, M2_PWMB, offset2B, M2_STBY);

// PID
#include <PID_v1.h>
//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1,
        Setpoint2, Input2, Output2,
        Setpoint3, Input3, Output3;
bool dirUp; // Is the motor travelling up
int NextPoint1, NextPoint2, NextPoint3;

//Specify the links and initial tuning parameters
double Kp=0.5, Ki=0.1, Kd=0.075; //, Ki=0.05, Kd=0.1;
PID posPID1(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID posPID2(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);
PID posPID3(&Input3, &Output3, &Setpoint3, Kp, Ki, Kd, DIRECT);

void motor_setup(){
    motor1A.brake();
    motor1B.brake();
    motor2A.brake();

    // Move up to start position
    /*motor1A.drive(-175);
    motor1B.drive(-175);
    motor2A.drive(-175);
    delay(1000);

    motor1A.brake();
    motor1B.brake();
    motor2A.brake();*/
}

void scale_setup(){
    Serial.printf("Setting up scale.");
    // Load Cell
    scale.set_scale();
    scale.tare(); //Reset the scale to 0
    long zero_factor = scale.read_average(); //Get a baseline reading
    Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
    Serial.println(zero_factor);
    scale.set_scale(calibration_factor); //Adjust to this calibration factor
}

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(9600);

    delay(2000);
    //Serial1.begin(115200);

    Serial.printf("Pins\nAIn2: %i\nAIn1: %i\nSTBY: %i\nBIn1: %i\nBIn2: %i\n", M1_AIN2, M1_AIN1, M1_STBY, M1_BIN1, M1_BIN2);

    delay(1000);

    scale_setup();

    motor_setup();

    // Reset the encoders
    enc1.write(0);
    enc2.write(0);
    enc3.write(0);
    dirUp = false;
    posPID1.SetMode(AUTOMATIC);
    posPID1.SetOutputLimits(MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
    posPID2.SetMode(AUTOMATIC);
    posPID2.SetOutputLimits(MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
    posPID3.SetMode(AUTOMATIC);
    posPID3.SetOutputLimits(MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);

    printTimer.reset();
}

long pos1  = -999;
long pos2  = -999;
long pos3  = -999;

String WifiRequest = "";

void wifiResponse(char* response)
{
    Serial.printf("B%s\n", response);
}

bool firstStop = true;

void loop() {

    enc_loop();

    motor_loop();

    //scale_loop();

    //lc_calibration_loop();

    while (Serial.available())
    {
        char character = Serial.read();
        if (character != '\n')
            WifiRequest.concat(character);
        else
        {
            if (WifiRequest[0] == 'G'){
                int newSetPt = WifiRequest.substring(1).toInt();

                Setpoint1 = (float)newSetPt;
                Setpoint2 = (float)newSetPt;
                Setpoint3 = (float)newSetPt;
            }
            else if (WifiRequest[0] == 'S'){
                motor1A.brake();
                motor1B.brake();
                motor2A.brake();

                Setpoint1 = (float)enc1.read();
                Setpoint2 = (float)enc2.read();
                Setpoint3 = (float)enc3.read();
            }
            else if (WifiRequest[0] == 'A'){
                int newSetPt = WifiRequest.substring(1).toInt();
                NextPoint1 = newSetPt;
            }
            else if (WifiRequest[0] == 'B'){
                int newSetPt = WifiRequest.substring(1).toInt();
                NextPoint2 = newSetPt;
            }
            else if (WifiRequest[0] == 'C'){
                int newSetPt = WifiRequest.substring(1).toInt();
                NextPoint3 = newSetPt;
            }
            else if (WifiRequest[0] == 'H'){
                Setpoint1 = (float)NextPoint1;
                Setpoint2 = (float)NextPoint2;
                Setpoint3 = (float)NextPoint3;
            }
            else if (WifiRequest == "ON")
            {
                wifiResponse("Stuff turned on");
                digitalWrite(LED_BUILTIN, HIGH);
            }
            else if (WifiRequest == "OFF")
            {
                wifiResponse("Stuff turned off");
                digitalWrite(LED_BUILTIN, LOW);
            }
            else
            {
                wifiResponse("Unknown command");
            }

            WifiRequest = "";
        }
        requestTimer = 0;
    }

}

void enc_loop() {
    pos1 = enc1.read();
    pos2 = enc2.read();
    pos3 = enc3.read();
}

void motor_loop(){

    Input1 = (double) pos1;
    posPID1.Compute();
    motor1A.drive((int) Output1);

    Input2 = (double) pos2;
    posPID2.Compute();
    motor1B.drive((int) Output2);

    Input3 = (double) pos3;
    posPID3.Compute();
    motor2A.drive((int) Output3);

    if(motorPrintTimer > 1000){
        Serial.printf("Motor signals: %i (%i/%i), %i (%i/%i), %i (%i/%i)\n",
                      (int)Output1, pos1, (int)Setpoint1,
                      (int)Output2, pos2, (int)Setpoint2,
                      (int)Output3, pos3, (int)Setpoint3);
        motorPrintTimer=0;
        Serial.printf("Scale readings: %i, %i, %i", 0, scale.get_units(), 0);
    }
}

void lc_calibration_loop(){
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