//
// Created by David Kadish on 24/03/2018.
//

#include "InheritanceTesting.h"
#include "Shape.h"
#include "CompositeShape.h"

#include <vector>

size_t Shape::count = 0;

Circle small(1,1,1), big(1, 2, 5);
Rectangle rectangle(0,0,8,8);

CompositeShape compositeShape(small, big, rectangle);

std::vector<Shape*> shapes = {&small, &big, &rectangle};

void setup(){
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);

    compositeShape.setup();

    Serial.println("************************************");
    Serial.println();

    for(Shape * s: shapes){
        Serial.printf("Shape %i of %i is at (%i, %i)\n", s->getIndex(), s->getCount(), s->getX(), s->getY());
    }
    Serial.println();

    Serial.printf("Shape %i of %i is at (%i, %i)\n", small.getIndex(), small.getCount(), small.getX(), small.getY());
    Serial.printf("Shape %i of %i is at (%i, %i)\n", big.getIndex(), big.getCount(), big.getX(), big.getY());
    Serial.printf("Shape %i of %i is at (%i, %i)\n", rectangle.getIndex(), rectangle.getCount(), rectangle.getX(), rectangle.getY());
    Serial.println();

    /*****************************************************/

    for(Shape * s: shapes){
        s->setX(10+s->getIndex());
        Serial.printf("Shape %i of %i is at (%i, %i)\n", s->getIndex(), s->getCount(), s->getX(), s->getY());
    }
    Serial.println();

    Serial.printf("Shape %i of %i is at (%i, %i)\n", small.getIndex(), small.getCount(), small.getX(), small.getY());
    Serial.printf("Shape %i of %i is at (%i, %i)\n", big.getIndex(), big.getCount(), big.getX(), big.getY());
    Serial.printf("Shape %i of %i is at (%i, %i)\n", rectangle.getIndex(), rectangle.getCount(), rectangle.getX(), rectangle.getY());
    Serial.println();


}

void loop(){

}