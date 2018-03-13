//
// Created by David Kadish on 08/03/2018.
//

#ifndef RUBE_CONTROLLER_H
#define RUBE_CONTROLLER_H


class Controller {
public:

    virtual void setup() = 0;
    virtual void loop() = 0;


    virtual void start() = 0;
    virtual void end() = 0;

protected:
    bool enabled;
};


#endif //RUBE_CONTROLLER_H
