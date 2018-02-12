/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder * myEnc1; // Example where it is not initialized immediately
Encoder myEnc2; // DOES NOT WORK
Encoder myEnc3(22, 23);
//   avoid using pins with LEDs attached

#define BRIGHT_LED 7

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  
  pinMode(BRIGHT_LED, OUTPUT); 
  analogWrite(BRIGHT_LED, 0);

  myEnc1 = new Encoder(18, 19);
  myEnc2 = Encoder(20, 21); // DOES NOT WORK
}

long oldPosition  = -999;

void loop() {
  long newPosition1 = myEnc1->read();
  long newPosition2 = myEnc2.read();
  long newPosition3 = myEnc3.read();
    Serial.print(newPosition1);
    Serial.print(", ");
    Serial.print(newPosition2);
    Serial.print(", ");
    Serial.println(newPosition3);
  delay(50);
}
