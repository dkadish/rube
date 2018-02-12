#define BRIGHT_LED 7

elapsedMillis ledTimer = 0;
bool ledOn = false;

// Droplet Servo motor
#include <Servo.h>
Servo dropperServo;  // create servo object to control a servo

void setup() {
  // put your setup code here, to run once:
  pinMode(BRIGHT_LED, OUTPUT);

  dropperServo.attach(6);
  dropperServo.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  if( ledOn && ledTimer > 5000L ){ // After 5 seconds
    if(random(100) < 5){
      digitalWrite(BRIGHT_LED, LOW);
      ledTimer = 0;
      ledOn = false;
    }
  } else if (!ledOn && ledTimer > 300000L) { // After 5 minutes
    if(random(100) < 5){
      digitalWrite(BRIGHT_LED, HIGH);
      ledTimer = 0;
      ledOn = true;
    }
  }
  delay(1000);
}
