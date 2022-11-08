/*
********************************************************************************
*
* Created on: March 15, 2022
*     Author: Nipun Dhananjaya Weerakkodi -> nipun.dhananjaya@gmail.com
********************************************************************************
*/

#ifndef __SERVO_CONTROLLER_H__
#define __SERVO_CONTROLLER_H__

#include "stdint.h"

#include "pca9685.h"
#include "i2c.h"
#include "variables.h"

#define FR_HIP_OFFSET   113
#define FR_ULEG_OFFSET   65
#define FR_LLEG_OFFSET   0

#define FL_HIP_OFFSET   116
#define FL_ULEG_OFFSET   197
#define FL_LLEG_OFFSET   290

#define BR_HIP_OFFSET   123
#define BR_ULEG_OFFSET   70
#define BR_LLEG_OFFSET   -35

#define BL_HIP_OFFSET   117
#define BL_ULEG_OFFSET   198
#define BL_LLEG_OFFSET   295

#define MIN_ANG_L2_L3		15
#define MAX_ANG_L2_L3		110

#define TH1_MIN         -90
#define TH1_MAX         90
#define TH2_MIN         -70
#define TH2_MAX         185
#define TH3_MIN         -90
#define TH3_MAX         130



// bool start = false;



void drive_servos();

#endif