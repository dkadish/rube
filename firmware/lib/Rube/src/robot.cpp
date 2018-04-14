//
// Created by David Kadish on 18/12/2017.
//

#include "robot.h"
#include "wifi.h"
#include "math.h"
#include "Arduino.h"


RobotPosition::RobotPosition():mount_height(0) {

    lineLengths = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // Initialize as array { 1.0, 2.4, ... }

    // Location of Q in XYZ
    O = {0.0, 0.0, 1.0};
    P = {0.0, 1.0, 1.0};
    Q = {1.0, 1.0, 1.0};

    // Setpoint in XYZ
    setpoint = {0.0, 0.0, 0.0};
    setpoint_lengths = {0.0, 0.0, 0.0};

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

    // Set Position of O, P
    O.z = mount_height;
    P.y = lineLengths.OP;
    P.z = mount_height;

    //first finding the XY position of point Q
    float r1=lineLengths.OQ;
    float r2=lineLengths.PQ;
    float d=lineLengths.OP;

    //FIXME correct the calculations in here to account for new axes and the inverted Z-axis

    // Calculate the position of Q
    char response[255];
    Q.x = ((1/d) * sqrt((-d+r2-r1)*(-d-r2+r1)*(-d+r2+r1)*(d+r2+r1)))/2;
    sprintf(response, "Q.x: %i.%i\n", (int)Q.x, (int)(Q.x*100.0));
    wifiResponse(response);
    Q.y = ((d*d)-(r2*r2)+(r1*r1))/(2*d);
    sprintf(response, "Q.y: %i.%i\n", (int)Q.y, (int)(Q.y*100.0));
    wifiResponse(response);
    Q.z = mount_height;

    CalculateXYZ(lineLengths.OR, lineLengths.PR, lineLengths.QR);

//    //now finding the pointR based on the three other lineLengths OPQ and the distances from these to pointR
//    r1=lineLengths.OR;
//    r2=lineLengths.PR;
//    float r3=lineLengths.QR;
//    d=lineLengths.OP;
//    float i=Q.x; //pointQ.x
//    float j=Q.y; //pointQ.y
//
//    R.x=(( ( (r1*r1) - (r2*r2) + (d*d) ) /  (2*d) ));
//    sprintf(response, "R.x: %i.%i\n", (int)R.x, (int)(R.x*100.0));
//    wifiResponse(response);
//    R.y=(( ( (r1*r1) - (r3*r3) + (i*i) + (j*j) ) / (2*j) ) - ( (i/j)*R.x )); // ((r1*r1)-(r3*r3)-(xc*xc)+( (xc-i)*(xc-i) )+(j*j) / (2*j));
//    sprintf(response, "R.y: %i.%i\n", (int)R.y, (int)(R.y*100.0));
//    wifiResponse(response);
//    R.z=( sqrt( (r1*r1) - (R.x*R.x) - (R.y*R.y) ))  ;
//    sprintf(response, "R.z (down): %i.%i\n", (int)R.z, (int)(R.z*100.0));
//    wifiResponse(response);
//    R.z=mount_height-R.z;

    INFO("Origins initialized as O(%i.%i,%i.%i,%i.%i), P(%i.%i,%i.%i,%i.%i), Q(%i.%i,%i.%i,%i.%i)",
        FLOAT(O.x), FLOAT(O.y), FLOAT(O.z),
         FLOAT(P.x), FLOAT(P.y), FLOAT(P.z),
         FLOAT(Q.x), FLOAT(Q.y), FLOAT(Q.z)
    )
}

void RobotPosition::CalculateLines(float x, float y, float z){
    setpoint.x = x;
    setpoint.y = y;
    setpoint.z = z;

    setpoint_lengths.O = sqrtf(powf(setpoint.x-O.x,2)+powf(setpoint.y-O.y,2)+powf(setpoint.z-O.z,2));
    setpoint_lengths.P = sqrtf(powf(setpoint.x-P.x,2)+powf(setpoint.y-P.y,2)+powf(setpoint.z-P.z,2));
    setpoint_lengths.Q = sqrtf(powf(setpoint.x-Q.x,2)+powf(setpoint.y-Q.y,2)+powf(setpoint.z-Q.z,2));
}

void RobotPosition::CalculateLines(Point3D point) {
    RobotPosition::CalculateLines(point.x, point.y, point.z);
}

/**
 * @brief Calculate the XYZ position of the robot given the winch geometries.
 *
 * Knowing the origin point of each winch and the length of each line, calculate
 * the current position of the robot. This assumes that O is located at (0,0,0),
 * P is at (d,0,0) and Q is at (i,j,0). In other words, the origins are planar
 * and P is on the x-axis.
 *
 * Assumes that the three mount points are level and returns R's z position wrt ground.
 */
void RobotPosition::CalculateXYZ(float length_O, float length_P, float length_Q) {
    float d=P.y, i=Q.y, j=Q.x;
    float l_O = length_O, l_P = length_P, l_Q = length_Q;

    float y = (powf(l_O,2) - powf(l_P,2) + powf(d,2))/(2.0*d);//(powf(lengths.O, 2) - powf(lengths.P, 2) + powf(d, 2))/(2*d);

    float x = (powf(l_O,2)-powf(l_Q,2)+powf(i,2)+powf(j,2))/(2*j) - (i*y)/(j);

    float z = Q.z - sqrtf(powf(l_O,2) - powf(x,2) - powf(y,2));

    R.x = (float)x;
    R.y = (float)y;
    R.z = (float)z;
}

// Find Position
void Robot::getPosition(Point3D &position) {
    float d=P.getOrigin().x, i=Q.getOrigin().x, j=Q.getOrigin().y;

    position.x = getX();

    position.y = _getY(position.x);

    position.z = _getz(position.x, position.y);
}

float Robot::getX(){
    float d=P.getOrigin().x;

    float x = (pow(O.getLength(), 2) - pow(P.getLength(), 2) + pow(d, 2))/(2*d);
    return x;
}

float Robot::_getY(float x) {
    float d=P.getOrigin().x, i=Q.getOrigin().x, j=Q.getOrigin().y;

    float y = (pow(O.getLength(),2)-pow(Q.getLength(),2)+pow(i,2)+pow(j,2))/(2*j) - (i*x)/(j);
    return y;
}

float Robot::getY() {
    float y = _getY(getX());
    return y;
}

float Robot::_getz(float x, float y) {
    float d=P.getOrigin().x, i=Q.getOrigin().x, j=Q.getOrigin().y;

    float z = -sqrt(pow(O.getLength(),2) - pow(x,2) - pow(y,2));
    return z;
}

float Robot::getZ() {
    float x = getX();
    float z = _getz(x, _getY(x));
    return z;
}

void Robot::setPositionTarget(Point3D &position) {}
void Robot::setXTarget(double x) {}
void Robot::setYTarget(double y) {}
void Robot::setZTarget(double z) {}

void Robot::setup() {

    // Set up the lengths of the lines
    // winch.setStartLenght()

}

void RobotPosition::update(float length_O, float length_P, float length_Q){
    CalculateXYZ(length_O, length_P, length_Q);

//    INFO("Length comparisons %i (%i), %i (%i), %i (%i)",
//         (int)(length_O*1000),(int)(lineLengths.OR*1000),
//         (int)(length_P*1000),(int)(lineLengths.PR*1000),
//         (int)(length_Q*1000),(int)(lineLengths.QR*1000)
//    )
//
//    CalculateXYZ(lineLengths.OR, lineLengths.PR, lineLengths.QR);
}