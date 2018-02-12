//
// Created by David Kadish on 27/10/2017.
//

#include <Encoder.h>
#define ENC1A 2
#define ENC1B 3
#define ENC1INDEX 4

volatile bool indexTriggered = false;

#define ENC2A 21
#define ENC2B 20

#define ENC3A 19
#define ENC3B 18

Encoder enc1(ENC1A, ENC1B);
//Encoder enc2(ENC2A, ENC2B);
//Encoder enc3(ENC3A, ENC3B);

#include <elapsedMillis.h>
elapsedMillis textTimer;
elapsedMicros debounceTimer;

void indexInterrupt(){
    indexTriggered = true;
}

void setup(){

    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);

    attachInterrupt(digitalPinToInterrupt(ENC1INDEX), indexInterrupt, RISING);

    Serial.printf("Starting...");

    enc1.write(0);

}

void loop(){

    if( indexTriggered ){
        indexTriggered = false;
        Serial.println("1 cycle...");
    }


    if (textTimer > 250) {
        Serial.printf("Motor Encoders: %i, %i, %i\n", enc1.read(), -1, -1);//, enc2.read(), enc3.read());
        textTimer = 0;
    }
}