//
// Created by David Kadish on 04/03/2018.
//

#ifndef RUBE_CONFIG_H
#define RUBE_CONFIG_H

// Encoder offset definitions
#ifndef SCALE1_OFFSET
    #define SCALE1_OFFSET 394996L
#endif
#ifndef SCALE2_OFFSET
    #define SCALE2_OFFSET 1356954L
#endif
#ifndef SCALE3_OFFSET
    #define SCALE3_OFFSET -1780478L
#endif

#ifndef STRING_TENSION_THRESHOLD
    #define STRING_TENSION_THRESHOLD 20.0
#endif

#ifndef TICKS_PER_REVOLUTION
    #define TICKS_PER_REVOLUTION 1024 // TODO is this right
#endif

#if defined(__MK66FX1M0__) || defined(__MK64FX512__)
#define MOTOR_UPPER_LIMIT 255
#define MOTOR_LOWER_LIMIT -100

#define POSITION_UPPER_LIMIT 100
#define POSITION_LOWER_LIMIT -100

// Used to calculate velocity from encoder motion and microseconds
#define VELOCITY_SCALE_FACTOR 1000.0
#define METRES_PER_REVOLUTION (0.0129*3.14159) // A = r * theta, adjusted through calibration
#endif
#ifdef __MK20DX256__ // Teensy 3.2 (Test Rig)
#define MOTOR_UPPER_LIMIT 100
#define MOTOR_LOWER_LIMIT -255

#define POSITION_UPPER_LIMIT 20000
#define POSITION_LOWER_LIMIT -20000

// Used to calculate velocity from encoder motion and microseconds
#define VELOCITY_SCALE_FACTOR 100000.0
#endif

#endif //RUBE_CONFIG_H
