//
// Created by David Kadish on 24/03/2018.
//

#ifndef INHERITANCE_TESTING_COMPOSITESHAPE_H
#define INHERITANCE_TESTING_COMPOSITESHAPE_H


#include "Shape.h"

class CompositeShape {
public:
    CompositeShape(Circle smallCircle, Circle bigCircle, Rectangle rectangle);

    Circle sCirc, bCirc;
    Rectangle rect;

    Shape *shapes[3] = {&sCirc, &bCirc, &rect};

    void setup(){
        Serial.println("Setting up composite shape.");

        for(int i = 0; i < 3; i++){
            Serial.printf("Shape %i at (%i, %i)\n", i, shapes[i]->getX(), shapes[i]->getY());
        }
        Serial.println();

        sCirc.moveLeft();
        bCirc.moveLeft();
        rect.moveLeft();

        /*********************************************************************************/

        for(int i = 0; i < 3; i++){
            Serial.printf("Shape %i at (%i, %i)\n", i, shapes[i]->getX(), shapes[i]->getY());
        }
        Serial.println();

        Serial.printf("Shape %i at (%i, %i)\n", 0, sCirc.getX(), sCirc.getY());
        Serial.printf("Shape %i at (%i, %i)\n", 1, bCirc.getX(), bCirc.getY());
        Serial.printf("Shape %i at (%i, %i)\n", 2, rect.getX(), rect.getY());
        Serial.println();

        /*********************************************************************************/

        for(int i = 0; i < 3; i++){
            shapes[i]->setX(10);
            Serial.printf("Shape %i at (%i, %i)\n", i, shapes[i]->getX(), shapes[i]->getY());
        }
        Serial.println();

        Serial.printf("Shape %i at (%i, %i)\n", 0, sCirc.getX(), sCirc.getY());
        Serial.printf("Shape %i at (%i, %i)\n", 1, bCirc.getX(), bCirc.getY());
        Serial.printf("Shape %i at (%i, %i)\n", 2, rect.getX(), rect.getY());

        /*********************************************************************************/

        for(int i = 0; i < 3; i++){
            Serial.printf("Shape %i is the %ith of %i.\n", i, shapes[i]->getIndex(), shapes[i]->getCount());
        }
        Serial.println();

        Serial.printf("Shape %i is the %ith of %i.\n", 0, sCirc.getIndex(), sCirc.getCount());
        Serial.printf("Shape %i is the %ith of %i.\n", 1, bCirc.getIndex(), bCirc.getCount());
        Serial.printf("Shape %i is the %ith of %i.\n", 2, rect.getIndex(), rect.getCount());
        Serial.println();

        for(int i = 0; i < 3; i++){
            Serial.printf("Shape %i is the %ith of %i.\n", i, shapes[i]->getIndex(), shapes[i]->getCount());
        }
        Serial.println();

        /*********************************************************************************/
    }
    void loop(){

    }
};


#endif //INHERITANCE_TESTING_COMPOSITESHAPE_H
