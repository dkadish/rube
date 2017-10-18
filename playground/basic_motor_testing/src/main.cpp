//
// Created by David Kadish on 18/10/2017.
//
// 18/10/2017 NOTE
// Motors: Negative is up.
// Encoder: Up is positive on the encoder.
//

#include <Encoder.h>
#define ENC1A 23
#define ENC1B 22

#define ENC2A 21
#define ENC2B 20

#define ENC3A 19
#define ENC3B 18

Encoder enc1(ENC1A, ENC1B);
Encoder enc2(ENC2A, ENC2B);
Encoder enc3(ENC3A, ENC3B);

#include <SparkFun_TB6612.h>

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

Motor motors[] = {motor1A, motor1B, motor2A};


#include <elapsedMillis.h>
elapsedMillis motorTimer;
elapsedMillis textTimer;

bool fwd = true;

void setup(){

    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.printf("Starting...");

    /*elapsedMillis startUp;
    while(startUp < 1000) {
        motor1A.drive(-50);
        motor1B.drive(-50);
        motor2A.drive(-50);
    }*/

    motorTimer = 0;
}


String WifiRequest = "";

void wifiResponse(char* response)
{
    Serial.printf("B%s\n", response);
}

bool go = false;
int speed = 0;
int motor = 0;
long duration = 0;
elapsedMillis goTimer = 0;

void loop(){

    while(Serial.available()) {

        char character = Serial.read();
        if (character != '\n')
            WifiRequest.concat(character);
        else {
            if (WifiRequest[0] == 'G') { // Go
                wifiResponse("GO!");
                go = true;
                goTimer = 0;

            } else if (WifiRequest[0] == 'H') {
                motor1A.brake();
                motor1B.brake();
                motor2A.brake();

                /*Setpoint1 = (float) enc1.read();
                Setpoint2 = (float) enc2.read();
                Setpoint3 = (float) enc3.read();*/
            } else if (WifiRequest[0] == 'M') { // Select Motor 0,1,2
                wifiResponse("Motor set");
                motor = WifiRequest.substring(1).toInt();
            } else if (WifiRequest[0] == 'S') { // Set speed 0-255
                wifiResponse("Speed set");
                speed = WifiRequest.substring(1).toInt();
            } else if (WifiRequest[0] == 'D') { // Set duration 0-inf
                wifiResponse("Duration set");
                duration = (long) (WifiRequest.substring(1).toInt());
            } else if (WifiRequest[0] == 'P') {

            } else if (WifiRequest == "ON") {
                wifiResponse("Stuff turned on");
                digitalWrite(LED_BUILTIN, HIGH);
            } else if (WifiRequest == "OFF") {
                wifiResponse("Stuff turned off");
                digitalWrite(LED_BUILTIN, LOW);
            } else {
                wifiResponse("Unknown command");
            }

            WifiRequest = "";
        }
    }

    if(goTimer > duration){
        go = false;

        motor1A.brake();
        motor1B.brake();
        motor2A.brake();
    } else {
        motors[motor].drive(speed);
    }

    /*if( motorTimer > 500){
        fwd = !fwd;
        motorTimer = 0;
    }

    if( fwd ){
        digitalWrite(LED_BUILTIN, HIGH);
        motor1A.drive(50);
        motor1B.drive(50);
        motor2A.drive(50);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
        motor1A.drive(-100);
        motor1B.drive(-100);
        motor2A.drive(-100);
    }*/

    if(textTimer > 250) {
        Serial.printf("Motor Encoders (fwd=%i): %i, %i, %i\n", (int)fwd, enc1.read(), enc2.read(), enc3.read());
        textTimer = 0;
    }
}

