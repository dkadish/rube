/*
 * Created by David Kadish on 23/02/2018.
 *
 * Negative speed relaxes lines. Positive tensions.
 *
 */

#include "ManualControlDiagnostics.h"
#include "../.piolibdeps/RUBE/src/robot.h"
#include <Winch.h>
#include <elapsedMillis.h>

#include <logging.h>

// IMU
#include <NXPMotionSense.h>
#include <i2c_t3.h>
NXPMotionSense imu(&Wire1, &Wire3);
NXPSensorFusion imu_filter;

// Timer for print functions
elapsedMillis printTimer=0;

void setup(){
    delay(2000);

    winches[0] = &O;
    winches[1] = &P;
    winches[2] = &Q;

    Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000); // Teensy
    Wire3.begin(I2C_MASTER, 0x00, I2C_PINS_56_57, I2C_PULLUP_EXT, 400000); // Teensy

    // Serial connection
    Serial.begin(9600);
    Serial1.begin(9600);
    delay(2000);
    INFO("Finished initializing Serial");

    // Winch
    for( int i=0; i < N_WINCHES; i++) {
        winches[i]->setup();
        INFO("Finished setting up 1 Winch");
    }
    INFO("Finished setting up Winch");

    // IMU
    imu.begin();
    imu_filter.begin(100);
    INFO("Finished setting up IMU");

    /* NOTE
     * It seems like the first read never reads, so you need to do this to get it ready to send data.
     */
    while(!imu.available()) {
        DEBUG("Waiting for IMU...");
        delay(1000);
    }

    position_setup();

    INFO("Staring Loop...");

}

void position_setup(){

    msgSerial->printf("Setting up position with robot params | height: %i, OP: %i, PQ:%i, OQ: %i, OR: %i, PR: %i, QR:%i.\n",
                      (int)robotParams.height, (int)robotParams.OP, (int)robotParams.PQ, (int)robotParams.OQ,
                      (int)robotParams.OR, (int)robotParams.PR, (int)robotParams.QR
    );


    msgSerial->printf("The current line lengths are: ");
    for( int i=0; i < N_WINCHES; i++) {
        msgSerial->print(winches[i]->getLength());
        if( i < 2 )
            msgSerial->print(", ");
    }
    msgSerial->println();

    Point3D pos = position.getXYZ();
    msgSerial->printf("The current XYZ is : (%i.%i, %i.%i, %i.%i)\n",
                      (int)(pos.x), decimalDigits(pos.x),
                      (int)(pos.y), decimalDigits(pos.y),
                      (int)(pos.z), decimalDigits(pos.z)
    );

    // Positioning
    position.CalibrateInitialPosition(robotParams);
    O.setOrigin(position.getOriginO());
    P.setOrigin(position.getOriginP());
    Q.setOrigin(position.getOriginQ());
    O.setStartLength(robotParams.OR);
    P.setStartLength(robotParams.PR);
    Q.setStartLength(robotParams.QR);

    msgSerial->printf("The current line lengths are: ");
    for( int i=0; i < N_WINCHES; i++) {
        msgSerial->print(winches[i]->getLength());
        if( i < 2 )
            msgSerial->print(", ");
    }
    msgSerial->println();

    pos = position.getXYZ();
    msgSerial->printf("The current XYZ is : (%i.%i, %i.%i, %i.%i)\n",
                      (int)(pos.x), decimalDigits(pos.x),
                      (int)(pos.y), decimalDigits(pos.y),
                      (int)(pos.z), decimalDigits(pos.z)
    );
}

void loop(){
    robot_loop();

    serial_loop();
}

void robot_loop(){
    position.update(O.getLength(), P.getLength(), Q.getLength());

    for( int i=0; i < N_WINCHES; i++) {
        winches[i]->loop();
    }

    // IMU
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float altitude;

    if (imu.available()) {
        // Read the motion sensors
        imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz, altitude);

        // Update the SensorFusion filter
        imu_filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    }
}

void serial_loop(){
    while (cmdSerial->available())
    {
        char character = cmdSerial->read();
        if (character != '\n') {
            command.concat(character);
        } else {
            msgSerial->println(command);
            handleSerialInput(command);
            msgSerial->println("Clearing Command Buffer...");
            command = "";
            cmdSerial->flush();
        }
    }
}

void handleSerialInput(String serial_in){
    if( serial_in[0] == 'S' ){
        msgSerial->println("Printing Sensors");
        printSensors();
    } else if (serial_in[0] == 'C'){ // Configuration
        if( serial_in[1] == 'P' ){
            printConfig();
        } else if( serial_in[1] == 'T' ){
            tareScales();
        }
    }else if (serial_in[0] == 'D'){
        runDiagnostics();
    } else if (serial_in[0] == 'W'){
        wifiResponse("Performing Winch function...");
        int winch_i = serial_in.substring(1, 2).toInt();//int winch_i = serial_in[1] - 48; // Subtract the ASCII char code for '0' == 48
        Serial.printf("Actuating winch %i, %c.\n", winch_i, serial_in[1]);
        if( serial_in[2] == 'S' ){
            doWinchStop(winch_i);
        } else if ( serial_in[2] == 'V' ){
            doSetWinchSignal(winch_i,serial_in.substring(3).toInt());
        } else if ( serial_in[2] == 'P' ){
            float pt = serial_in.substring(3).toFloat();
            INFO("Repositioning to %i.%i", (int)(pt), decimalDigits(pt));
            doSetWinchPositionSetpoint(winch_i, pt);
        } else if ( serial_in[2] == 'D' ){
            doDisplayWinchState(winch_i);
        } else if ( serial_in[2] == 'T' ){
            INFO("Tensioning winch %i.", winch_i);
            doTensionLine(winch_i);
        } else if ( serial_in[2] == 'R' ){
            INFO("Releasing winch %i.", winch_i);
            doRelaxLine(winch_i);
        } else if ( serial_in[2] == 'W' ){ // Wind
            int ms = serial_in.substring(3).toInt(); // Time in ms to wind for
            doRampUp(winch_i, ms);
        } else if ( serial_in[2] == 'U' ){ // Unwind
            int ms = serial_in.substring(3).toInt(); // Time in ms to unwind for
            doRampDown(winch_i, ms);
        } else {
            wifiResponse("ERR: Invalid Robot Command");
        }
    }
    else {
        wifiResponse("ERR: Unknown command");
    }
}

void runDiagnostics(){
    // Run diagnostic loop for each winch
    /*INFO("Running diagnostic loop...")
    int startPos=0, midPos=0, endPos=0;
    float downWeight=0.0, upWeight=0.0;
    int weightCounter=0;
    for(int i=0; i < N_WINCHES; i++){
        INFO("Running diagnostics for motor: Starting motor...")
        elapsedMillis runTimer = 0;
        startPos = winches[i]->enc->read();
        winches[i]->stop_go = GO;
        winches[i]->control_mode = SIGNAL;
        winches[i]->signal = -150;
        while(runTimer < 1000){
            winches[i]->loop();
            downWeight += winches[i]->scale.get_units(5);
            weightCounter++;
            delayMicroseconds(1);
        }
        downWeight /= (float)weightCounter;
        midPos = winches[i]->enc->read();
        weightCounter = 0;
        delay(100);
        runTimer = 0;
        winches[i]->signal = 200;
        while(runTimer < 1000){
            winches[i]->loop();
            upWeight += winches[i]->scale.get_units(5);
            weightCounter++;
            delayMicroseconds(1);
        }
        upWeight /= (float)weightCounter;
        winches[i]->signal = 0;
        winches[i]->stop_go = STOP;
        winches[i]->loop();
        endPos = winches[i]->enc->read();
        msgSerial->printf("Ran from %i > %i > %i\n", startPos, midPos, endPos);
        msgSerial->print("Weight: Down = ");
        msgSerial->print(downWeight);
        msgSerial->print(", Up = ");
        msgSerial->println(upWeight);
    }
    INFO("Running diagnostic loop: COMPLETE...")*/
}

void printConfig(){
    // Winch

    // Scales
    for( int i=0; i < N_WINCHES; i++) {
        msgSerial->printf("Scale offsets: ");
        msgSerial->print(winches[i]->getScaleOffset());
        if( i < 2 )
            msgSerial->print(", ");
    }
    msgSerial->println();

    // IMU

    if( printTimer > 1000 ) printTimer = 0;
}

void tareScales(){
    msgSerial->print("Scale offsets: ");
    for( int i=0; i < N_WINCHES; i++) {
        winches[i]->scaleTare();
        msgSerial->printf("%i: ", i);
        msgSerial->print(winches[i]->getScaleOffset());
        msgSerial->print("L");
        if( i < 2 )
            msgSerial->print(", ");
    }
    msgSerial->println();
}

void printSensors(){
    // Winch
    for( int i=0; i < N_WINCHES; i++) {
        winches[i]->loop();
    }
    msgSerial->printf("The current positions of the winches are: ");
    for( int i=0; i < N_WINCHES; i++) {
        msgSerial->print(winches[i]->getPosition());
        if( i < 2 )
            msgSerial->print(", ");
    }
    msgSerial->println();
    msgSerial->printf("The current ticks of the winches are: ");
    for( int i=0; i < N_WINCHES; i++) {
        msgSerial->print(winches[i]->getEncoderTicks());
        if( i < 2 )
            msgSerial->print(", ");
    }
    msgSerial->println();
    msgSerial->printf("The current line lengths are: ");
    for( int i=0; i < N_WINCHES; i++) {
        msgSerial->print(winches[i]->getLength());
        if( i < 2 )
            msgSerial->print(", ");
    }
    msgSerial->println();

    Point3D pos = position.getXYZ();
//    msgSerial->printf("The current XYZ is : (%i.%i, %i.%i, %i.%i)\n",
//                      (int)(pos.x), decimalDigits(pos.x),
//                      (int)(pos.y), decimalDigits(pos.y),
//                      (int)(pos.z), decimalDigits(pos.z)
//    );
//
//    INFO("Updating with %i, %i, %i", (int)O.getLength(), (int)P.getLength(), (int)Q.getLength());
//    position.update(O.getLength(), P.getLength(), Q.getLength());
//
//    pos = position.getXYZ();
    msgSerial->printf("The current XYZ is : (%i.%i, %i.%i, %i.%i)\n",
                      (int)(pos.x), decimalDigits(pos.x),
                      (int)(pos.y), decimalDigits(pos.y),
                      (int)(pos.z), decimalDigits(pos.z)
    );

    // Scales
    msgSerial->printf("The current weights are: ");
    for( int i=0; i < N_WINCHES; i++) {
        msgSerial->printf("%i: ", i);
        msgSerial->print(winches[i]->getTension());
        msgSerial->print("(");
        msgSerial->print(winches[i]->isUnderTension());
        msgSerial->print(")");
        if( i < 2 )
            msgSerial->print(", ");
    }
    msgSerial->println();

    // IMU
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float roll, pitch, heading;
    float temperature;
    float altitude;
    //long altint;

    if (imu.available()) {
        // Read the motion sensors
        imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
        imu.readAltitudeAndTemperature(altitude, temperature);
        //imu.readMotionSensor(altint); //TODO Why is this wrong?

        // Update the SensorFusion filter
        imu_filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    }

    // print the heading, pitch and roll
    roll = imu_filter.getRoll();
    pitch = imu_filter.getPitch();
    heading = imu_filter.getYaw();
    msgSerial->print("Orientation: ");
    msgSerial->print(heading);
    msgSerial->print(" ");
    msgSerial->print(pitch);
    msgSerial->print(" ");
    msgSerial->print(roll);
    msgSerial->print(", Temperature:  ");
    msgSerial->print(temperature);
    msgSerial->print(", Altitude:  ");
    msgSerial->println(altitude);
    //msgSerial->print(", Altitude:  ");
    //msgSerial->println(*altint);
}

// Winch Functions
void doWinchStop(int winch_i){
    winches[winch_i]->doStop();
}

void doSetWinchSignal(int winch_i, int signal){
    winches[winch_i]->doGo(signal);
}

void doSetWinchPositionSetpoint(int winch_i, float setpoint){
    winches[winch_i]->doGoTo(setpoint);//, 0.1);
//    INFO("Error is %i.%i. Stopping.", (int)(winches[winch_i]->pidPos_ctrl->getError()), winches[winch_i]->pidPos_ctrl->getError());
}

void doDisplayWinchState(int winch_i){
    char response[255];
    sprintf(response, "Winch %i Position: %i.%i, Est. Speed: %i.%i", winch_i,
            (int)(winches[winch_i]->getPosition()), decimalDigits(winches[winch_i]->getPosition()),
            (int)(winches[winch_i]->getSpeed()), decimalDigits(winches[winch_i]->getSpeed())
    );
    wifiResponse(response);
}

void doTensionLine(int winch_i){
    if(winches[winch_i]->isUnderTension()) {
        msgSerial->printf("Winch %i is already under tension. Position: %i.%i, Tension: %i.%i.\n", winch_i,
                (int)(winches[winch_i]->getPosition()), decimalDigits(winches[winch_i]->getPosition()),
                (int)(winches[winch_i]->getTension()), decimalDigits(winches[winch_i]->getTension())
        );
        return;
    }

    winches[winch_i]->doRetension();
    while(!winches[winch_i]->isUnderTension()){
        loop();
    }

    msgSerial->printf("Winch %i Tensioned. Position: %i.%i, Tension: %i.%i.\n", winch_i,
            (int)(winches[winch_i]->getPosition()), decimalDigits(winches[winch_i]->getPosition()),
            (int)(winches[winch_i]->getTension()), decimalDigits(winches[winch_i]->getTension())
    );
}

void doRelaxLine(int winch_i){
    // Set the motor speed
    if(winches[winch_i]->isUnderTension()) {
        winches[winch_i]->doGo(-100);
    } else {
        msgSerial->printf("Winch %i is already relaxed. Position: %i.%i, Tension: %i.%i.\n", winch_i,
                (int)(winches[winch_i]->getPosition()), decimalDigits(winches[winch_i]->getPosition()),
                (int)(winches[winch_i]->getTension()), decimalDigits(winches[winch_i]->getTension())
        );
        return;
    }

    while(winches[winch_i]->isUnderTension()){
        loop();
    }
    // Stop the motor
    winches[winch_i]->doGo(0);

    msgSerial->printf("Winch %i relaxed. Position: %i.%i, Tension: %i.%i.\n", winch_i,
            (int)(winches[winch_i]->getPosition()), decimalDigits(winches[winch_i]->getPosition()),
            (int)(winches[winch_i]->getTension()), decimalDigits(winches[winch_i]->getTension())
    );
}

void doRampUp(int winch_i, int ms){
    INFO("Doing RampUp");
    winches[winch_i]->doSlowUp();
    _doRamp(winch_i, ms);
}

void doRampDown(int winch_i, int ms) {
    winches[winch_i]->doSlowDown();
    _doRamp(winch_i, ms);
}

void _doRamp(int winch_i, int ms){
    msgSerial->printf("Ramping winch %i for %i ms.\n", winch_i, ms);

    elapsedMillis rampTimer = 0;
    long lastPrint = 0;

    while(rampTimer < ms) {

        loop();

        // Print the tension every once in a while
        if( rampTimer/50 > lastPrint ){
            lastPrint++;
            msgSerial->printf("Ramp is at %i. Position: %i.%i, Tension: %i.%i\n", winches[winch_i]->getSignal(),
                              (int)(winches[winch_i]->getPosition()), decimalDigits(winches[winch_i]->getPosition()),
                              (int)(winches[winch_i]->getTension()), decimalDigits(winches[winch_i]->getTension()));
        }
    }
    // Stop the motor
    winches[winch_i]->doStop();

    char response[255];
    sprintf(response, "Winch %i ramped. Position: %i.%i, Tension: %i.%i.", winch_i,
            (int)(winches[winch_i]->getPosition()), decimalDigits(winches[winch_i]->getPosition()),
            (int)(winches[winch_i]->getTension()), decimalDigits(winches[winch_i]->getTension())
    );
    wifiResponse(response);

}

int decimalDigits(float number){
    int whole = ((int)number) * 100;
    int frac = (int)(number * 100 - whole);
    frac = abs(frac);

    return frac;
}