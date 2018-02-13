/***********************************************************************************************************************
 * Created by David Kadish on 12/02/2018.
 *
 * TODO:
 * [x] Winch
 * [x] IMU
 * [ ] SD Card Logging
 * [ ] XBee Communication
 * [ ] Add interval bump to winch code
 */

#include <Arduino.h>
#include <logging.h>
#include <pindefs.h>

// Winch
#include <Winch.h>
Winch winch(0, ENC1A, ENC1B,
        M1_BIN1, M1_BIN2, M1_PWMB, 1, M1_STBY,
        10.0, 5.0, 0.5);

// IMU
#include <NXPMotionSense.h>
#include <i2c_t3.h>
NXPMotionSense imu(&Wire1, &Wire3);
NXPSensorFusion imu_filter;

// Function to run the "loop" once
void loopOnce();

void setup(){
    Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000); // Teensy
    Wire3.begin(I2C_MASTER, 0x00, I2C_PINS_56_57, I2C_PULLUP_EXT, 400000); // Teensy

    // Serial connection
    Serial.begin(9600);
    delay(2000);
    Serial.printf("Finished initializing Serial (%i ms)\n", millis());

    // Winch
    winch.setup();
    Serial.printf("Finished setting up Winch (%i ms)\n", millis());

    // IMU
    imu.begin();
    imu_filter.begin(100);
    Serial.printf("Finished setting up IMU (%i ms)\n", millis());

    /* NOTE
     * It seems like the first read never reads, so you need to do this to get it ready to send data.
     */
    while(!imu.available()) {
        Serial.printf("Waiting for IMU...(%i ms)\n", millis());
        delay(100);
    }

    Serial.printf("Looping once (%i ms)\n", millis());
    loopOnce();
    Serial.printf("Finished loop (%i ms)\n", millis());

    Serial.println();
    Serial.printf("Testing Log Messages (%i ms)\n", millis());
    TRACE("This is a TRACE message.");
    DEBUG("This is a DEBUG message.");
    INFO("This is a INFO message.");
    WARNING("This is a WARNING message.");
    ERROR("This is a ERROR message.");
    FATAL("This is a FATAL message.");
}

void loopOnce(){

    // Winch
    winch.loop();
    Serial.print("The current position of the winch is ");
    Serial.println(winch.current_position());

    // IMU
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float roll, pitch, heading;

    if (imu.available()) {
        // Read the motion sensors
        imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

        // Update the SensorFusion filter
        imu_filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

        // print the heading, pitch and roll
        roll = imu_filter.getRoll();
        pitch = imu_filter.getPitch();
        heading = imu_filter.getYaw();
        Serial.print("Orientation: ");
        Serial.print(heading);
        Serial.print(" ");
        Serial.print(pitch);
        Serial.print(" ");
        Serial.println(roll);
    } else {
        Serial.println("Oops, IMU not available. Something is wrong.");
    }


}

void loop(){

}