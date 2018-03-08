//
// Created by David Kadish on 08/03/2018.
//

#ifndef RUBE_CONTROLLER_H
#define RUBE_CONTROLLER_H


class Controller {
public:
    bool enabled;

    virtual void setup() = 0;
    virtual void loop() = 0;
};


#endif //RUBE_CONTROLLER_H
