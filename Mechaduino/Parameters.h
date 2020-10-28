//Contains the Mechaduino parameter declarations
#include "stdint.h"
#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#define firmware_version "0.1.5"    // firmware version
#define identifier "x"              // change this to help keep track of multiple mechaduinos (printed on startup)

// Motor control settings

#define ANTI_COGG_CAL true

#if ANTI_COGG_CAL
	#define CALC_VELOCITY_ASYNC false
#else 
	#define CALC_VELOCITY_ASYNC true
#endif

#define USE_FIXED_POINT true
#define USE_10_BIT true	
#define SAMPLE_ON_REQUEST false
#define VELOCITY_TEST false
#define LATENCY_DEBUG false
#define BUFFER_ANGLE_READINGS false
#define USE_MOTOR_CALIBRATION false
#define FIXED_POINT_FRACTIONAL_BITS 6
#define HAPTIC_LOOP_FREQ 1000
#define FPID_FREQ 6000
#define HW2

typedef int32_t fixed_point_t;

//----Current Parameters-----

extern volatile float Ts;
extern const int Fs;

extern volatile float pKp;
extern volatile float pKi;
extern volatile float pKd;
extern volatile float pLPF;


extern volatile float vKp;
extern volatile float vKi;
extern volatile float vKd;
extern volatile float vLPF;

extern const float lookup[];
extern const int16_t lookup_fixed[];


extern volatile float pLPFa;
extern volatile float pLPFb;
extern volatile float vLPFa;
extern volatile float vLPFb;

// Fixed point variables
extern fixed_point_t fixed_180;
extern fixed_point_t fixed_neg_180;
extern fixed_point_t fixed_360;
extern fixed_point_t vLPFa_fixed;
extern fixed_point_t vLPFb_fixed;


extern const int spr; //  200 steps per revolution
extern const float aps; // angle per step
extern int cpr; //counts per rev
extern const float stepangle;

extern volatile float PA;  //

extern const float iMAX;
extern const float rSense;
extern volatile int uMAX;
extern float max_torque;


extern const int sin_1[];
extern const int anti_cogg_fw[];
extern const int anti_cogg_bw[];
extern bool use_anti_cogg;


//Defines for pins:

#define IN_4  6
#define IN_3  5
#define VREF_2 4
#define VREF_1 9
#define IN_2  7
#define IN_1  8
#define ledPin  13
#define chipSelectPin A2 //output to chip select

#define step_pin 1
#define dir_pin 0
#define enable_pin 2

//for faster digitalWrite:
#define IN_1_HIGH() (REG_PORT_OUTSET0 = PORT_PA06)
#define IN_1_LOW() (REG_PORT_OUTCLR0 = PORT_PA06)
#define IN_2_HIGH() (REG_PORT_OUTSET0 = PORT_PA21)
#define IN_2_LOW() (REG_PORT_OUTCLR0 = PORT_PA21)
#define IN_3_HIGH() (REG_PORT_OUTSET0 = PORT_PA15)
#define IN_3_LOW() (REG_PORT_OUTCLR0 = PORT_PA15)
#define IN_4_HIGH() (REG_PORT_OUTSET0 = PORT_PA20)
#define IN_4_LOW() (REG_PORT_OUTCLR0 = PORT_PA20)
#define ledPin_HIGH() (REG_PORT_OUTSET0 = PORT_PA17)
#define ledPin_LOW() (REG_PORT_OUTCLR0 = PORT_PA17)
#define CHIPSELECT_HIGH() (REG_PORT_OUTSET1 = PORT_PB09)
#define CHIPSELECT_LOW() (REG_PORT_OUTCLR1 = PORT_PB09)

#define ENABLE_PROFILE_IO    // Define to enable profiling I/O pins

#ifdef ENABLE_PROFILE_IO
#define TEST1   3
#define TEST2   2

#define TEST1_HIGH() (REG_PORT_OUTSET0 = PORT_PA09)
#define TEST1_LOW() (REG_PORT_OUTCLR0 = PORT_PA09)
#define TEST2_HIGH() (REG_PORT_OUTSET0 = PORT_PA14)
#define TEST2_LOW() (REG_PORT_OUTCLR0 = PORT_PA14)

#else
#define TEST1_HIGH()
#define TEST1_LOW()
#endif



#endif
