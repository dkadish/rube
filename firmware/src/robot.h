//
// Created by David Kadish on 18/12/2017.
//

#ifndef FIRMWARE_ROBOT_H
#define FIRMWARE_ROBOT_H

struct RobotSetupParameters {
    float height;

    float OP;
    float PQ;
    float OQ;

    float OR; // Origin to robot
    float PR;
    float QR;
};

struct TetrahedronLengths {
    // Mount point distances
    float OP; // Origin to first point
    float PQ; // First point to second point
    float OQ; // Origin to second pointw

    // Line lengths
    float OR; // Origin to robot
    float PR;
    float QR;
};

struct Point3D {
    float x;
    float y;
    float z;
};

class RobotPosition {

public:
    RobotPosition();

    float mount_height; // The height (in metres) that the mount lineLengths are at.

    TetrahedronLengths lineLengths; // Initialize as array { 1.0, 2.4, ... }
    TetrahedronLengths lineLengthSetpoints;

    // Location of Q in XYZ
    Point3D Q;//, O, P;

    // Setpoint in XYZ
    Point3D setpoint;

    // RobotPosition position in XYZ
    Point3D R;

    void CalibrateInitialPosition(RobotSetupParameters params);
    void CalculateLineOPQ(int x, int y, int z);
};


#endif //FIRMWARE_ROBOT_H
