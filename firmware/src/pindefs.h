//
// Created by David Kadish on 29/11/2017.
//

#ifndef FIRMWARE_PINDEFS_H
#define FIRMWARE_PINDEFS_H

#if defined(__MK66FX1M0__) || defined(__MK64FX512__)
#define ENC1A 23
#define ENC1B 22

#define ENC2A 21
#define ENC2B 20

#define ENC3A 19
#define ENC3B 18

#endif
#ifdef __MK20DX256__
#define ENC1A 5
#define ENC1B 6

#define ENC2A ENC1A
#define ENC2B ENC1B

#define ENC3A ENC1A
#define ENC3B ENC1B
#endif

#if defined(__MK66FX1M0__) || defined(__MK64FX512__)
// Pin settings for teensy 3.6 on stripboard

    //********************** MOTOR DRIVER 1 **********************//
    #define M1_STBY	10 //Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
    // Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M1_AIN1 9
    #define M1_BIN1 11
    //Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M1_AIN2 8
    #define M1_BIN2 12
    //PWM input for channels A/B	Input	PWM input that controls the speed
    #define M1_PWMA 3
    #define M1_PWMB 4


    //********************** MOTOR DRIVER 2 **********************//
    #define M2_STBY	26 //Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
    // Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M2_AIN1 25
    //#define M2_BIN1 27
    //Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M2_AIN2 24
    //#define M2_BIN2 28
    //PWM input for channels A/B	Input	PWM input that controls the speed
    #define M2_PWMA 29
    //#define M2_PWMB 30

#endif
#ifdef __MK20DX256__
// Pin settings for teensy 3.2 on solderless breadboard
    #define M1_STBY	11 //Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
    // Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M1_AIN1 8
    #define M1_BIN1 10
    //Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M1_AIN2 7
    #define M1_BIN2 9
    //PWM input for channels A/B	Input	PWM input that controls the speed
    #define M1_PWMA 3
    #define M1_PWMB 4

    // Pin settings for teensy 3.2 on solderless breadboard
    #define M2_STBY	M1_STBY	 //Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
    // Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M2_AIN1 M1_AIN1
    //#define M2_BIN1 M1_BIN1
    //Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
    #define M2_AIN2 M1_AIN2
    //#define M2_BIN2 M1_BIN2
    //PWM input for channels A/B	Input	PWM input that controls the speed
    #define M2_PWMA M1_PWMA
    //#define M2_PWMB M2_PWMB
#endif

#endif //FIRMWARE_PINDEFS_H
