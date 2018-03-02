/***********************************************************************************************************************
 * Created by David Kadish on 14/02/2018.
 *
 *
 * NOTES:
 * [x] Pin problems with ENCODER 3. Works on 1. Non on 3. One of the pins is good (I think, 12?). Other busted.
 * [x] Possible pin problems with 18/19 (HX711 sensors). Check them.
 * [ ] Altimeter reading is incorrect
 */

#include <Arduino.h>
#include <logging.h>
#include <pindefs.h>

// Winch
#include <Winch.h>
//TODO: Check motor and encoder mappings. Are these right?
Winch A(0, ENC1A, ENC1B,
        M1_BIN1, M1_BIN2, M1_PWMB, 1, M1_STBY,
        DOUT_A, SCK_A,
        10.0, 5.0, 0.5);
Winch B(1, ENC2A, ENC2B,
        M2_AIN1, M2_AIN2, M2_PWMA, 1, M2_STBY,
        DOUT_B, SCK_B,
        10.0, 5.0, 0.5);
Winch C(2, ENC3A, ENC3B,
        M1_AIN1, M1_AIN2, M1_PWMA, 1, M1_STBY,
        DOUT_C, SCK_C,
        10.0, 5.0, 0.5);
Winch winches[] = {A, B, C};
const int N_WINCHES = 3;

// Scales
HX711 A_scale, B_scale, C_scale;
HX711 scales[] = {A_scale, B_scale, C_scale};
int scales_dout[] = {DOUT_A, DOUT_B, DOUT_C};
int scales_sck[] = {SCK_A, SCK_B, SCK_C};

// IMU
#include <NXPMotionSense.h>
#include <i2c_t3.h>
NXPMotionSense imu(&Wire1, &Wire3);
NXPSensorFusion imu_filter;

// Timer for print functions
elapsedMillis printTimer=0;

void setup(){
    Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000); // Teensy
    Wire3.begin(I2C_MASTER, 0x00, I2C_PINS_56_57, I2C_PULLUP_EXT, 400000); // Teensy

    // Serial connection
    Serial.begin(9600);
    delay(2000);
    /*while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }*/
    INFO("Finished initializing Serial");

    // Winch
    for( int i=0; i < N_WINCHES; i++) {
        winches[i].setup();
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

}

void loop(){

    // Winch
    for( int i=0; i < N_WINCHES; i++) {
        winches[i].loop();
    }
    if (printTimer > 1000) {
        Serial.printf("The current positions of the winches are: ");
        for( int i=0; i < N_WINCHES; i++) {
            Serial.print(winches[i].enc->read());
            if( i < 2 )
                Serial.print(", ");
        }
        Serial.println();
    }

    // Scales
    if (printTimer > 1000) {
        for( int i=0; i < N_WINCHES; i++) {
            Serial.printf("The current weights are: ");
            Serial.print(winches[i].scale.get_units());
            if( i < 2 )
                Serial.print(", ");
        }
        Serial.println();

    }

    // IMU
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float roll, pitch, heading;
    float altitude;

    if (imu.available()) {
        // Read the motion sensors
        imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz, altitude);

        // Update the SensorFusion filter
        imu_filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    }

    if(printTimer > 1000) {
        // print the heading, pitch and roll
        roll = imu_filter.getRoll();
        pitch = imu_filter.getPitch();
        heading = imu_filter.getYaw();
        Serial.print("Orientation: ");
        Serial.print(heading);
        Serial.print(" ");
        Serial.print(pitch);
        Serial.print(" ");
        Serial.print(roll);
        Serial.print(", Altitude:  ");
        Serial.println(altitude);
    }

    if( printTimer > 1000 ) printTimer = 0;

}