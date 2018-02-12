//
// Created by David Kadish on 12/02/2018.
//

#include <Arduino.h>
#include <logging.h>
#include <pindefs.h>
#include <Winch.h>

Winch winch(0, ENC1A, ENC1B,
        M1_BIN1, M1_BIN2, M1_PWMB, 1, M1_STBY,
        10.0, 5.0, 0.5);


// Function to run the "loop" once
void loopOnce();

void setup(){

    // Serial connection
    Serial.begin(9600);
    delay(2000);
    Serial.printf("Finished initializing Serial (%i ms)\n", millis());

    // Winch
    winch.setup();
    Serial.printf("Finished setting up Winch (%i ms)\n", millis());

    Serial.printf("Looping once (%i ms)\n", millis());
    loopOnce();
    Serial.printf("Finished loop (%i ms)\n", millis());

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


}

void loop(){

}