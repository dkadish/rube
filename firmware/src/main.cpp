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
#include "wifi.h"

#include "Winch.h"
#define N_WINCHES 3

// Load Cell
#include "HX711.h"
#include "SDLogger.h"
#include "robot.h"

#define LC_DAT 14
#define LC_CLK 15
HX711 scale(LC_DAT, LC_CLK);
float calibration_factor = -7050.0; //-7050 worked for my 440lb max scale setup

// Loop initializations
void lc_calibration_loop();
void serial_loop();

int offset1A = -1;
int offset1B = -1;
int offset2A = -1;

RobotPosition robot_pos = RobotPosition();
RobotSetupParameters robot_pos_params = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
Winch A(0, ENC1A, ENC1B,
        M1_AIN1, M1_AIN2, M1_PWMA, offset1A, M1_STBY,
        0.0, 0.0, 0.0);
Winch B(1, ENC2A, ENC2B,
        M1_BIN1, M1_BIN2, M1_PWMB, offset1B, M1_STBY,
        0.0, 0.0, 0.0);
Winch C(2, ENC3A, ENC3B,
        M2_AIN1, M2_AIN2, M2_PWMA, offset2A, M2_STBY,
        0.0, 0.0, 0.0);

Winch winches[] = {A, B, C};

int NextPoint1;

#define BRIGHT_LED 7

// SD Card
String logLoop();
SDLogger logger = SDLogger(logLoop);

String WifiRequest = "";
String SerialRequest = "";

// Function templates
void handleSerialInput(String serial_in8);

void doOnLED();
void doOffLED();
void doBrightPWM(int pwm);

void doStop();
void doGo();
void doSetMode(char mode);
void doSetXYZ(String xyz_string);
void doSetVelocity(String xyz_string);
void doDisplayRobotPositioning();
void doSetRobotPosParams(String param_string);
void doDisplayRobotParams();

void doWinchStop(int winch_i);
void doSetWinchSignal(int winch_i, int signal);

void doSetKp(float k, int winch_i);
void doSetKi(float k, int winch_i);
void doSetKv(float k, int winch_i);
void doDisplayWinchPIDParams(int winch_i);

void setup() {
    // Pin 14 causes boot issues.
    //pinMode(14, OUTPUT_OPENDRAIN);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(9600);
    Serial1.begin(115200);

    delay(2000);

    Serial.printf("Encoder pins: %i, %i | %i, %i | %i, %i\n", ENC1A, ENC1B, ENC2A, ENC2B, ENC3A, ENC3B);

    // SD Card
    if(logger.setup()){
        wifiResponse("STATUS: SD Card initialised.\n");
    } else {
        wifiResponse("STATUS: SD initialization failed.\n");
    }

    Serial.printf("Pins\nAIn2: %i\nAIn1: %i\nSTBY: %i\nBIn1: %i\nBIn2: %i\n", M1_AIN2, M1_AIN1, M1_STBY, M1_BIN1, M1_BIN2);

    robot_pos.CalibrateInitialPosition(robot_pos_params);
    for(int i=0; i < N_WINCHES; i++){
        winches[i].setup();
    }

    pinMode(BRIGHT_LED, OUTPUT);
    digitalWrite(BRIGHT_LED, HIGH);
    delay(200);
    digitalWrite(BRIGHT_LED, LOW);
}

void loop() {

    for(int i=0; i < N_WINCHES; i++){
        winches[i].loop();
    }

    serial_loop();
}

/* Command Structure
 *
 * L > LED
 *  0 > Off
 *  1 > On
 *  P > Bright LED PWM
 *
 * R > Robot
 *  S > Stop
 *  G > Go
 *  M > Mode (S: speed, P: Position)
 *  P > Position (X###Y######Z####)
 *  V > Velocity (X###Y######Z####)
 *  D > Display All
 *
 * W > Winch
 *  # > Winch Number
 *      V > Set velocity
 *      S > Stop
 *
 * P > Parameters
 *  # > Winch Control
 *      P > Kp
 *      I > Ki
 *      V > Kv
 *      K > Print K
 *      A > All (P#####V#####I#####)
 *      D > Display All
 *
 *  H > Height of the mounts
 *  OP > O-P Distance
 *  PQ > P-Q Distance
 *  OQ > O-Q Distance
 *  OR > O-R Distance
 *  PR > P-R Distance
 *  QR > Q-R Distance
 *  D > Display All
 *
 */

void serial_loop(){
   while (Serial1.available())
    {
        char character = Serial1.read();
        if (character != '\n') {
            WifiRequest.concat(character);
        } else {
            handleSerialInput(WifiRequest);

            WifiRequest = "";

            Serial1.flush();
        }
    }

    while (Serial.available())
    {
        char character = Serial.read();
        if (character != '\n') {
            SerialRequest.concat(character);
        } else {
            handleSerialInput(SerialRequest);

            SerialRequest = "";

            Serial.flush();
        }
    }

    //logger.loop();
}

/*void SerialEvent1(){
    char character = Serial1.read();
    if (character != '\n') {
        WifiRequest.concat(character);
    } else {
        handleSerialInput(WifiRequest);

        WifiRequest = "";
    }
}*/

void handleSerialInput(String serial_in){
    if( serial_in[0] == 'L' ){ // LED Functions
        wifiResponse("Performing LED function...");
        if( serial_in[1] == '0' ){
            doOffLED();
        } else if ( serial_in[1] == '1' ){
            doOnLED();
        } else if ( serial_in[1] == 'P' ){
            doBrightPWM(serial_in.substring(2).toInt());
        } else {
            wifiResponse("ERR: Invalid LED Command");
        }
    } else if ( serial_in[0] == 'R' ) { // Robot motion functions
        wifiResponse("Performing Robot motion function...");
        if( serial_in[1] == 'S' ){
            doStop();
        } else if ( serial_in[1] == 'G' ){
            doGo();
        } else if ( serial_in[1] == 'M' ){
            doSetMode(0);
        } else if ( serial_in[1] == 'P' ){ // In the format X######Y#####Z####
            doSetXYZ(serial_in.substring(2));
        } else if ( serial_in[1] == 'V' ){
            doSetVelocity(serial_in.substring(2));
        } else if ( serial_in[1] == 'D' ){
            doDisplayRobotPositioning();
        } else {
            wifiResponse("ERR: Invalid Robot Command");
        }
    } else if ( serial_in[0] == 'W' ) { // Winch motion functions
        wifiResponse("Performing Winch function...");
        int winch_i = serial_in.substring(1, 2).toInt();
        Serial.printf("Actuating winch %i, %c.\n", winch_i, serial_in.charAt(1));
        if( serial_in[2] == 'S' ){
            doWinchStop(winch_i);
        } else if ( serial_in[2] == 'V' ){
            doSetWinchSignal(winch_i,serial_in.substring(3).toInt());
        } else {
            wifiResponse("ERR: Invalid Robot Command");
        }
    } else if ( serial_in[0] == 'P' ) { // Parameter functions
        wifiResponse("Performing Parameter function...");
        if( isDigit(serial_in[1]) ) {
            int winch_i = serial_in.substring(1, 2).toInt();
            if (serial_in[2] == 'P') {
                doSetKp(serial_in.substring(3).toFloat(), winch_i);
            } else if (serial_in[2] == 'V') {
                doSetKv(serial_in.substring(3).toFloat(), winch_i);
            } else if (serial_in[2] == 'I') {
                doSetKi(serial_in.substring(3).toFloat(), winch_i);
            } else if ( serial_in[2] == 'D' ){
                doDisplayWinchPIDParams(winch_i);
            } else {
                wifiResponse("ERR: Invalid PID Parameter Command");
            }
        } else if (serial_in[1] == 'P') {
            doSetRobotPosParams(serial_in.substring(2));
        } else if ( serial_in[1] == 'D' ){
            doDisplayRobotParams();
        } else {
            wifiResponse("ERR: Invalid Parameter Command");
        }
    } else {
        wifiResponse("ERR: Unknown command");
    }
}

// Robot functions
void doStop(){
    for( int i = 0; i < 3; i++ ){
        winches[i].stop();
    }
}

void doGo(){
    for( int i = 0; i < 3; i++ ){
        winches[i].go();
    }
}

void doSetMode(char mode){
    for( int i = 0; i < 3; i++ ){
        if( mode == 'S') {
            winches[i].control_mode = SPEED;
        } else if ( mode == 'P' ){
            winches[i].control_mode = POSITION;
        } else {
            wifiResponse("ERR: Invalid Control Mode Command");
        }
    }
}

void _str_to_xyz(String xyz_string, float *x, float *y, float *z){
    int x_i, y_i, z_i;
    x_i = xyz_string.indexOf('X');
    y_i = xyz_string.indexOf('Y');
    z_i = xyz_string.indexOf('Z');

    *x = xyz_string.substring(x_i+1,y_i).toFloat();
    *y = xyz_string.substring(y_i+1,z_i).toFloat();
    *z = xyz_string.substring(z_i+1).toFloat();
}

void doSetXYZ(String xyz_string){
    float x, y, z;
    _str_to_xyz(xyz_string, &x, &y, &z);
    robot_pos.setpoint = {x, y, z};
}

void doSetVelocity( String xyz_string ){
    wifiResponse("ERR: Velocity control not implemented");
}

void doDisplayRobotPositioning(){
    String pos = String("Mode: ") + String(winches[0].control_mode);
    pos += String(", Position(") + String(robot_pos.lineLengths.OR)  + String(",") + String(robot_pos.lineLengths.PR) +
            String(",") + String(robot_pos.lineLengths.QR) + String(")");
    pos += String(", Velocity: Coming Soon!") /*+ String(robot_pos.OR)  + String(",") + String(robot_pos.lineLengths.PR) +
           String(",") + String(robot_pos.lineLengths.QR) + String(")")*/;
    char response[255];
    pos.toCharArray(response, 255);
    wifiResponse(response);
}

// Parameter Functions
void doSetKp(float k, int winch_i){
    Serial.printf("Setting Kp to %i.%i\n", (int)k, ((int)(k*100.0))%100);
    winches[winch_i].position.SetTunings(k, 0.0, 0.0);
}

void doSetKi(float k, int winch_i){
    Serial.printf("Setting Ki to %i.%i\n", (int)k, ((int)(k*100.0))%100);
    winches[winch_i].speed.SetTunings(winches[winch_i].speed.GetKp(), k, 0.0);
}

void doSetKv(float k, int winch_i){
    Serial.printf("Setting Kv to %i.%i\n", (int)k, ((int)(k*100.0))%100);
    winches[winch_i].speed.SetTunings(k, winches[winch_i].speed.GetKi(), 0.0);
}

void doDisplayWinchPIDParams(int winch_i){
    /*Serial.printf("Param > Kp: %i.%i, Kv: %i.%i, Ki: %i.%i\n",
                  (int)winches[winch_i].position.GetKp(), ((int)(winches[winch_i].position.GetKp()*100.0))%100,
                  (int)winches[winch_i].speed.GetKp(), ((int)(winches[winch_i].speed.GetKp()*100.0))%100,
                  (int)winches[winch_i].speed.GetKi(), ((int)(winches[winch_i].speed.GetKi()*100.0))%100
    );*/

    String pid = String("Param > Kp: ") + String(winches[winch_i].position.GetKp());
    pid += String(", Kv:") + String(winches[winch_i].speed.GetKp());
    pid += String(", Ki") + String(winches[winch_i].speed.GetKi());
    char response[255];
    pid.toCharArray(response, 255);
    wifiResponse(response);
}

void doSetRobotPosParams(String pos_string){
    RobotSetupParameters params;

    int paramBounds[7];
    String paramStrings[] = {"H", "OP", "PQ", "OQ", "OR", "PR", "QR"};
    for( int i = 0; i < 7; i++){
        paramBounds[i] = pos_string.indexOf(paramStrings[i]);
        char ps[2];
        paramStrings[i].toCharArray(ps,2);
        Serial.printf("%s %i, ", ps, paramBounds[i]);
    }
    Serial.println();

    params.height = pos_string.substring(paramBounds[0]+1,paramBounds[1]).toFloat();
    Serial.print(params.height);
    Serial.print(", ");
    params.OP = pos_string.substring(paramBounds[1]+2,paramBounds[2]).toFloat();
    Serial.print(params.OP);
    Serial.print(", ");
    params.PQ = pos_string.substring(paramBounds[2]+2,paramBounds[3]).toFloat();
    Serial.print(params.PQ);
    Serial.print(", ");
    params.OQ = pos_string.substring(paramBounds[3]+2,paramBounds[4]).toFloat();
    Serial.print(params.OQ);
    Serial.print(", ");
    params.OR = pos_string.substring(paramBounds[4]+2,paramBounds[5]).toFloat();
    Serial.print(params.OR);
    Serial.print(", ");
    params.PR = pos_string.substring(paramBounds[5]+2,paramBounds[6]).toFloat();
    Serial.print(params.PR);
    Serial.print(", ");
    params.QR = pos_string.substring(paramBounds[6]+2).toFloat();
    Serial.print(params.QR);
    Serial.println();

    robot_pos.CalibrateInitialPosition(params);

}

void doDisplayRobotParams(){
    String pos = String("Height: ") + String(robot_pos.mount_height);
    pos += String(", Mount Points: OP ") + String(robot_pos.lineLengths.OP)  + String(", PQ") + String(robot_pos.lineLengths.PQ) +
           String(", OQ") + String(robot_pos.lineLengths.OQ) + String("");
    pos += String(", Robot Location: OR") + String(robot_pos.lineLengths.OR)  + String(", OP") + String(robot_pos.lineLengths.OP) +
           String(", PQ") + String(robot_pos.lineLengths.PQ) + String("");
    char response[255];
    pos.toCharArray(response, 255);
    wifiResponse(response);
}

// Winch Functions
void doWinchStop(int winch_i){
    winches[winch_i].stop();
}

void doSetWinchSignal(int winch_i, int signal){
    winches[winch_i].go_signal(signal);
}

// LED Functions
void doOnLED(){
    wifiResponse("Stuff turned on");
    digitalWrite(LED_BUILTIN, HIGH);
}

void doOffLED(){
    wifiResponse("Stuff turned off");
    digitalWrite(LED_BUILTIN, LOW);
}

void doBrightPWM(int pwm){
    analogWrite(BRIGHT_LED, pwm);

    char resp[100];
    sprintf(resp, "Setting bright LED to PWM %i", pwm);
    wifiResponse(resp);
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