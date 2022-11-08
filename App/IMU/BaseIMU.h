/*
    STM32 library for IMU-amperka

*/


#ifndef __BASE_IMU_H__
#define __BASE_IMU_H__




#include "i2c.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

// Registers address
#define BASE_IMU_WHO_AM_I           0x0F    // device identification register
#define BASE_IMU_CTRL_REG1          0x20    // controll register 1
#define BASE_IMU_CTRL_REG2          0x21    // controll register 2
#define BASE_IMU_CTRL_REG3          0x22    // controll register 3
#define BASE_IMU_CTRL_REG4          0x23    // controll register 4
#define BASE_IMU_CTRL_REG5          0x24    // controll register 5

#define BASE_IMU_OUT_X_L            0x28    // output reg X
#define BASE_IMU_OUT_X_H            0x29    // output reg X
#define BASE_IMU_OUT_Y_L            0x2A    // output reg Y
#define BASE_IMU_OUT_Y_H            0x2B    // output reg Y
#define BASE_IMU_OUT_Z_L            0x2C    // output reg Z
#define BASE_IMU_OUT_Z_H            0x2D    // output reg Z

#define DEVICE_ID                   0xF5    // 
// typedef enum
// {
//     IMU_OK      = 0,
//     IMU_ERROR   = 1
// }IMU_STATUS;



void IMU_Init();

void init(I2C_HandleTypeDef);
uint8_t readDeviceID();
int16_t readX();
int16_t readY();
int16_t readZ();
void readXYZ(int16_t* x, int16_t* y, int16_t* z);


uint16_t IMU_ADDRESS;

#ifdef __cplusplus
}
#endif

#endif // __BASE_IMU_H__