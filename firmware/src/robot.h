//
// Created by David Kadish on 18/12/2017.
//

#ifndef FIRMWARE_ROBOT_H
#define FIRMWARE_ROBOT_H

struct TetrahedronLengths {
    // Mount point distances
    float OP; // Origin to first point
    float PQ; // First point to second point
    float OQ; // Origin to second point

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

class Robot {

    float mount_height; // The height (in metres) that the mount lineLengths are at.

    TetrahedronLengths lineLengths; // Initialize as array { 1.0, 2.4, ... }
    TetrahedronLengths lineLengthSetpoints;

    // Location of Q in XYZ
    Point3D Q;

    // Setpoint in XYZ
    Point3D setpoint;

    // Robot position in XYZ
    Point3D R;

    void CalibrateInitialPosition();
    void CalculateLineOPQ(int x, int y, int z);
};


#endif //FIRMWARE_ROBOT_H
