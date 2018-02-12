/*
  Reading a serial ASCII-encoded string.

 This sketch demonstrates the Serial parseInt() function.
 It looks for an ASCII string of comma-separated values.
 It parses them into ints, and uses those to fade an RGB LED.

 Circuit: Common-Cathode RGB LED wired like so:
 * Red anode: digital pin 3
 * Green anode: digital pin 5
 * Blue anode: digital pin 6
 * Cathode : GND

 created 13 Apr 2012
 by Tom Igoe
 
 modified 14 Mar 2016
 by Arturo Guadalupi

 This example code is in the public domain.
 */

void setup() {
  // initialize serial:
  Serial.begin(9600);

}

String SerialRequest = "";

void loop() {
  while (Serial.available()) {

    char character = Serial.read();
    
    if (character != '\n') {
      SerialRequest.concat(character);
    } else {
      //SerialRequest.trim();
      char sr[20];
      SerialRequest.toCharArray(sr, 20);
      Serial.printf("STR: %s, ", sr);
      float f = SerialRequest.toFloat();
      Serial.printf("INT: %i, FLOAT: ", (int)f);
      Serial.println(f);
      SerialRequest = "";
    }
  }
}








