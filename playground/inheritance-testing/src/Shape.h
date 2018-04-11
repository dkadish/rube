//
// Created by David Kadish on 24/03/2018.
//

#ifndef INHERITANCE_TESTING_SHAPE_H
#define INHERITANCE_TESTING_SHAPE_H

#include <Arduino.h>

class Shape {

public:
    Shape(int x, int y, int width, int height):x(x), y(y), width(width), height(height) {count++; index=count;}

    virtual void setup() = 0;
    virtual void loop() = 0;

    virtual int getArea() = 0;
    virtual int getPerimeter() = 0;

    int getX(){ return x; }
    int getY(){ return y; }

    void setX(int X){ x = X; }
    void setY(int Y){ x = Y; }

    size_t getIndex(){return index;}
    static size_t getCount(){return count;}

protected:

    int x, y;
    int width, height;

    size_t index;

    static size_t count;

};

class Rectangle: public Shape {
public:

    Rectangle(int x, int y, int width, int height):Shape(x, y, width, height ) {}

    void setup() {
        Serial.println("Setting up rectangle.");
    }

    void loop() {}

    int getArea(){ return width*height; }
    int getPerimeter(){ return 2*(width+height); }

    int getLeft(){ return x - width/2; }
    int getRight(){ return x + width/2; }
    int getTop(){ return y - height/2; }
    int getBottom(){ return y + height/2; }

    void moveLeft(){ x-=1; }
    void moveRight(){ x+=1; }

};

class Circle: public Shape {
public:

    Circle(int x, int y, int radius):Shape(x, y, radius, radius ) {}

    void setup() {
        Serial.println("Setting up rectangle.");
    }

    void loop() {}

    int getRadius(){ return width/2; }

    int getArea(){ return 3*getRadius()*getRadius(); }
    int getPerimeter(){ return 2*3*getRadius(); }

    void moveLeft(){ x-=1; }
    void moveRight(){ x+=1; }

};


#endif //INHERITANCE_TESTING_SHAPE_H
