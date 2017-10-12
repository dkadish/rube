
#include <Arduino.h>

#include "genann.h"

void setup(){
    Serial.begin(9600);
    delay(1000);

    Serial.printf("GENANN example 1.\n");
    Serial.printf("Train a small ANN to the XOR function using backpropagation.\n");

    /* Input and expected out data for the XOR function. */
    const float input[4][2] = {{0, 0}, {0, 1}, {1, 0}, {1, 1}};
    const float output[4] = {0, 1, 1, 0};
    int i;

    /* New network with 2 inputs,
     * 1 hidden layer of 2 neurons,
     * and 1 output. */
    genann *ann = genann_init(2, 1, 2, 1);

    /* Train on the four labeled data points many times. */
    for (int j = 0; j < 5; j++){
        for (i = 0; i < 100; ++i) {
            genann_train(ann, input[0], output + 0, 3);
            genann_train(ann, input[1], output + 1, 3);
            genann_train(ann, input[2], output + 2, 3);
            genann_train(ann, input[3], output + 3, 3);
        }

        char instr1[10], instr2[10], outstr[10];

        /* Run the network and see what it predicts. */
        Serial.println("Training Round: ");
        dtostrf(input[0][0],3,3,instr1); dtostrf(input[0][1],3,3,instr2); dtostrf(*genann_run(ann, input[0]),3,3,outstr);
        Serial.printf("Output for [%s, %s] is %s.\n", instr1, instr2, outstr);
        dtostrf(input[1][0],3,3,instr1); dtostrf(input[1][1],3,3,instr2); dtostrf(*genann_run(ann, input[1]),3,3,outstr);
        Serial.printf("Output for [%s, %s] is %s.\n", instr1, instr2, outstr);
        dtostrf(input[2][0],3,3,instr1); dtostrf(input[2][1],3,3,instr2); dtostrf(*genann_run(ann, input[2]),3,3,outstr);
        Serial.printf("Output for [%s, %s] is %s.\n", instr1, instr2, outstr);
        dtostrf(input[3][0],3,3,instr1); dtostrf(input[3][1],3,3,instr2); dtostrf(*genann_run(ann, input[3]),3,3,outstr);
        Serial.printf("Output for [%s, %s] is %s.\n", instr1, instr2, outstr);
        Serial.println();

    }

    genann_free(ann);
}

void loop(){

}