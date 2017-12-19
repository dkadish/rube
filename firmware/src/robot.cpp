//
// Created by David Kadish on 18/12/2017.
//

#include "robot.h"
#include "math.h"


RobotPosition::RobotPosition():mount_height(0) {

    lineLengths = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // Initialize as array { 1.0, 2.4, ... }
    lineLengthSetpoints = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    // Location of Q in XYZ
    Q = {1.0, 1.0, 1.0};//, O, P;

    // Setpoint in XYZ
    setpoint = {0.0, 0.0, 0.0};

    // RobotPosition position in XYZ
    R = {0.0, 0.0, 0.0};
}

void RobotPosition::CalibrateInitialPosition(RobotSetupParameters params) {

    // Set parameters from calibration settings
    mount_height = params.height;
    lineLengths.OP = params.OP;
    lineLengths.PQ = params.PQ;
    lineLengths.OQ = params.OQ;
    lineLengths.OR = params.OR;
    lineLengths.PR = params.PR;
    lineLengths.QR = params.QR;

    //first finding the XY position of point Q
    float r1=lineLengths.OQ;
    float r2=lineLengths.PQ;
    float d=lineLengths.OP;

    // Calculate the position of Q
    Q.x = ((d*d)-(r2*r2)+(r1*r1))/(2*d);
    Q.y = ((1/d) * sqrt((-d+r2-r1)*(-d-r2+r1)*(-d+r2+r1)*(d+r2+r1)))/2;
    Q.z = mount_height;

    //now finding the pointR based on the three other lineLengths OPQ and the distances from these to pointR
    r1=lineLengths.OR;
    r2=lineLengths.PR;
    int r3=lineLengths.QR;
    d=lineLengths.OP;
    float i=Q.x; //pointQ.x
    float j=Q.y; //pointQ.y

    R.x=int(( ( (r1*r1) - (r2*r2) + (d*d) ) /  (2*d) ));
    R.y=int(( ( (r1*r1) - (r3*r3) + (i*i) + (j*j) ) / (2*j) ) - ( (i/j)*R.x )); // ((r1*r1)-(r3*r3)-(xc*xc)+( (xc-i)*(xc-i) )+(j*j) / (2*j));
    R.z=int( sqrt( (r1*r1) - (R.x*R.x) - (R.y*R.y) ))  ;
    R.z=mount_height-R.z;
}

void RobotPosition::CalculateLineOPQ(int x, int y, int z){
    setpoint.x=x;
    setpoint.y=y;
    setpoint.z=z;
    float d= lineLengths.OP;
    float i= Q.x;
    float j= Q.y;
    float h=mount_height;
    lineLengthSetpoints.OR = int(sqrt((setpoint.x*setpoint.x)+(setpoint.y*setpoint.y)+((setpoint.z-h)*(setpoint.z-h))));
    lineLengthSetpoints.PR = int(sqrt(((setpoint.x-d)*(setpoint.x-d))+(setpoint.y*setpoint.y)+((setpoint.z-h)*(setpoint.z-h)))) ;
    lineLengthSetpoints.QR = int(sqrt(((setpoint.x-i)*(setpoint.x-i))+((setpoint.y-j)*(setpoint.y-j))+((setpoint.z-h)*(setpoint.z-h)))) ;
}