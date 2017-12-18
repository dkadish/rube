/******************************************************************************
 *
 * Created by David Kadish on 2017-07-05.
 *
 *
 *
 * TODO: Wire load cell circuit into place.
 *
 * Nov 28: Created a velocity measurement from encoder.
 * TODO: Use for doing cascaded PID control. Position outside velocity loop.
 *
 * Dec 17
 * TODO: - Finish assembling board
 *       - Reprogram PINDEFs
 *       - Add winches B and C
 *       - Calibrate PID control
 *       - Store variables to SD card
 *       - Real-time clock?

******************************************************************************/

#include "pindefs.h"

#include "Winch.h"

// Load Cell
#include "HX711.h"
#include "SDLogger.h"

#define LC_DAT 14
#define LC_CLK 15
HX711 scale(LC_DAT, LC_CLK);
float calibration_factor = -7050.0; //-7050 worked for my 440lb max scale setup

// Loop initializations
void lc_calibration_loop();
void serial_loop();

int offset1A = -1;

Winch A(0, ENC1A, ENC1B,
        M1_AIN1, M1_AIN2, M1_PWMA, offset1A, M1_STBY,
        0.0, 0.0, 0.0);

int NextPoint1;


// SD Card
String logLoop();
SDLogger logger = SDLogger(logLoop);

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(9600);

    delay(2000);
    //Serial1.begin(115200);

    // SD Card
    if(logger.setup()){
        Serial.printf("STATUS: SD Card initialised.\n");
    } else {
        Serial.printf("STATUS: SD initialization failed.\n");
    }

    Serial.printf("Pins\nAIn2: %i\nAIn1: %i\nSTBY: %i\nBIn1: %i\nBIn2: %i\n", M1_AIN2, M1_AIN1, M1_STBY, M1_BIN1, M1_BIN2);


    A.setup();
}

String WifiRequest = "";

void wifiResponse(char* response)
{
    Serial.printf("B%s\n", response);
}

void loop() {

    A.loop();

    serial_loop();
}

void serial_loop(){
    while (Serial.available())
    {
        char character = Serial.read();
        if (character != '\n')
            WifiRequest.concat(character);
        else
        {
            if (WifiRequest[0] == 'G'){
                int newSetPt = WifiRequest.substring(1).toInt();

                A.pos_setpt = (float)newSetPt;
                /*Setpoint2 = (float)newSetPt;
                Setpoint3 = (float)newSetPt;*/
            }
            else if (WifiRequest[0] == 'S'){
                A.control_mode = POSITION;
                A.motor.brake();
                /*motor1B.brake();
                motor2A.brake();*/

                A.pos_setpt = A.current_position();
            }
            else if (WifiRequest[0] == 'D'){
                double newSpeed = ((double)WifiRequest.substring(1).toInt()) / 1000.0;
                A.control_mode = SPEED;

                A.pos_out = newSpeed;
            }
            else if (WifiRequest[0] == 'A'){
                int newSetPt = WifiRequest.substring(1).toInt();
                NextPoint1 = newSetPt;
            }
            /*else if (WifiRequest[0] == 'B'){
                int newSetPt = WifiRequest.substring(1).toInt();
                NextPoint2 = newSetPt;
            }
            else if (WifiRequest[0] == 'C'){
                int newSetPt = WifiRequest.substring(1).toInt();
                NextPoint3 = newSetPt;
            }*/
            else if (WifiRequest[0] == 'H'){
                A.pos_setpt = (float)NextPoint1;
                /*Setpoint2 = (float)NextPoint2;
                Setpoint3 = (float)NextPoint3;*/
            }
            else if (WifiRequest[0] == 'P'){
                int kp = WifiRequest.substring(1).toInt();
                float kp_float = ((float)kp)/100.0;
                Serial.printf("Setting Kp to %i.%i\n", (int)kp_float, ((int)(kp_float*100.0))%100);
                A.position.SetTunings(kp_float, 0.0, 0.0);
            }
            else if (WifiRequest[0] == 'V'){
                int kv = WifiRequest.substring(1).toInt();
                float kv_float = ((float)kv)/100.0;
                Serial.printf("Setting Kv to %i.%i\n", (int)kv_float, ((int)(kv_float*100.0))%100);
                A.speed.SetTunings(kv_float, A.speed.GetKi(), 0.0);
            }
            else if (WifiRequest[0] == 'I'){
                int ki = WifiRequest.substring(1).toInt();
                float ki_float = ((float)ki)/100.0;
                Serial.printf("Setting Ki to %i.%i\n", (int)ki_float, ((int)(ki_float*100.0))%100);
                A.speed.SetTunings(A.speed.GetKp(), ki_float, 0.0);
            }
            else if (WifiRequest[0] == 'K'){
                Serial.printf("Param > Kp: %i.%i, Kv: %i.%i, Ki: %i.%i\n",
                    (int)A.position.GetKp(), ((int)(A.position.GetKp()*100.0))%100,
                    (int)A.speed.GetKp(), ((int)(A.speed.GetKp()*100.0))%100,
                    (int)A.speed.GetKi(), ((int)(A.speed.GetKi()*100.0))%100
                );
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
    }

    logger.loop();
}

String logLoop(){
    return "Hello World!";
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