//
// Created by David Kadish on 18/12/2017.
//

#ifndef FIRMWARE_ROBOT_H
#define FIRMWARE_ROBOT_H

#include "Geometry.h"
#include "Winch.h"
#include <i2c_t3.h>
#include <NXPMotionSense.h>

/**
 * @brief Holds initial parameters for robot setup
 *
 * Robot is set up in a system so that            O-----------P
 *                                                 \         /
 *                                                  \   R   /
 *                                                   \     /
 *                                                    \   /
 *                                                     \ /
 *                                                      Q
 */
struct RobotSetupParameters {
    float height; /**< The height of the bottom of the robot from the ground (m). */

    float OP; /**< The length of the line from O to P (m). */
    float PQ; /**< The length of the line from P to Q (m). */
    float OQ; /**< The length of the line from O to Q (m). */

    float OR; /**< Length of the cable from O to robot (m). */
    float PR; /**< Length of the cable from P to robot (m).*/
    float QR; /**< Length of the cable from Q to robot (m). */
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

struct WinchGeometry {
    Point3D origin;
    float length;
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

    WinchGeometry winches[3];

    void CalibrateInitialPosition(RobotSetupParameters params);
    void CalculateLines(float x, float y, float z);
    void CalculateLines(Point3D xyz);
    static Point3D CalculateXYZ(WinchGeometry O, WinchGeometry P, WinchGeometry Q);

    void update();
};

class Robot{
public:
    Robot();

    void setup(); /**< Run once when the main Arduino setup() function is run. */
    void loop(); /**< Run once per cycle when the main Arduino loop() function is run. */

    Winch O, P, Q;
    Winch winches[3]; /**< The order here is switched (O-Q-P instead of O-P-Q) to reflect the mapping of electronics to physical space and the orientation of the robot. */

    //TODO: Build error here!
    NXPMotionSense imu(&Wire1, &Wire3);
    NXPSensorFusion imu_filter;

    // Position
    void getPosition(Point3D &position);
    float getX();
    float getY();
    float getZ();

    void setPositionTarget(Point3D &position);
    void setXTarget(double x);
    void setYTarget(double y);
    void setZTarget(double z);

private:

    float _getY(float x);
    float _getz(float x, float y);
};


#endif //FIRMWARE_ROBOT_H
