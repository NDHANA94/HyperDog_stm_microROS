#include "BaseIMU.h"


I2C_HandleTypeDef *imu_i2c;



uint8_t _readByte(uint16_t regAddress) {
    uint8_t data;

    if (HAL_OK != HAL_I2C_Mem_Read(imu_i2c, DEVICE_ID, regAddress, 1, &data, 1, 10)){
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    }
    // _wire->beginTransmission(_slaveAddress);
    // _wire->write(regAddress);
    // _wire->endTransmission();
    // _wire->requestFrom(_slaveAddress, 1u);
    // data = _wire->read();
    return data;
}


uint8_t readDeviceID(){
    return _readByte(BASE_IMU_WHO_AM_I);
}

int16_t readX() {
    return _readByte(BASE_IMU_OUT_X_H) << 8 | _readByte(BASE_IMU_OUT_X_L);
}

int16_t readY() {
    return _readByte(BASE_IMU_OUT_Y_H) << 8 | _readByte(BASE_IMU_OUT_Y_L);
}

int16_t readZ() {
    return _readByte(BASE_IMU_OUT_Z_H) << 8 | _readByte(BASE_IMU_OUT_Z_L);
}

void readXYZ(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t data[6];
    _readBytes(0x80 | BASE_IMU_OUT_X_L, data, 6);
    x = data[1] << 8 | data[0];
    y = data[3] << 8 | data[2];
    z = data[5] << 8 | data[4];
}



void _writeByte(uint16_t regAddress, uint8_t data) {
    
    if(HAL_OK != HAL_I2C_Mem_Write(imu_i2c, DEVICE_ID, regAddress, 1, &data, 1, 10)){
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    }
}

void _readBytes(uint16_t regAddress, uint8_t* data, uint8_t length) {
    if(HAL_OK != HAL_I2C_Mem_Write(imu_i2c, DEVICE_ID, regAddress, 1, data, length, 10)){
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    }
}

void IMU_Init()
{
    MX_I2C1_Init();
    imu_i2c  = &hi2c1;
}