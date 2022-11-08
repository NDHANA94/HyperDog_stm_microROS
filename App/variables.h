/*
********************************************************************************
*
* Created on: March 15, 2022
*     Author: Nipun Dhananjaya Weerakkodi -> nipun.dhananjaya@gmail.com
********************************************************************************
*/

#ifndef __VARIABLES_H__
#define __VARIABLES_H__

#ifdef __cplusplus
extern "C" {
#endif



float joint_ang[12];
float servo_ang[12];

float joint_ang_feedback[4][4];
float current_sense[4][3];


#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

