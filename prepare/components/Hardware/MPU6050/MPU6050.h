#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>
#include <esp_err.h>
#include <stdbool.h>
#include "MPU6050_reg.h"



// 陀螺仪量程选项
typedef enum {
    MPU6050_GYRO_RANGE_250DPS  = 0x00,  // ±250°/s
    MPU6050_GYRO_RANGE_500DPS  = 0x08,  // ±500°/s
    MPU6050_GYRO_RANGE_1000DPS = 0x10,  // ±1000°/s
    MPU6050_GYRO_RANGE_2000DPS = 0x18   // ±2000°/s
} mpu6050_gyro_range_t;

// 加速度计量程选项
typedef enum {
    MPU6050_ACCEL_RANGE_2G  = 0x00,  // ±2g
    MPU6050_ACCEL_RANGE_4G  = 0x08,  // ±4g
    MPU6050_ACCEL_RANGE_8G  = 0x10,  // ±8g
    MPU6050_ACCEL_RANGE_16G = 0x18   // ±16g
} mpu6050_accel_range_t;

// MPU6050 数据结构体
typedef struct {
    int16_t accel_x;  // 原始加速度计X数据
    int16_t accel_y;  // 原始加速度计Y数据
    int16_t accel_z;  // 原始加速度计Z数据
    int16_t gyro_x;   // 原始陀螺仪X数据
    int16_t gyro_y;   // 原始陀螺仪Y数据
    int16_t gyro_z;   // 原始陀螺仪Z数据
} mpu6050_raw_data_t;

typedef struct {
    float accel_x;  // 加速度计X (g)
    float accel_y;  // 加速度计Y (g)
    float accel_z;  // 加速度计Z (g)
    float gyro_x;   // 陀螺仪X (°/s)
    float gyro_y;   // 陀螺仪Y (°/s)
    float gyro_z;   // 陀螺仪Z (°/s)
} mpu6050_data_t;

// 姿态数据结构体（传感器融合后的角度）
typedef struct {
    float pitch;  // 俯仰角 (-90° ~ +90°，向前为正)
    float roll;   // 横滚角 (-90° ~ +90°，向右为正)
} mpu6050_attitude_t;

esp_err_t MPU6050_Init(uint8_t dev_addr);
esp_err_t MPU6050_FuseData(const mpu6050_data_t *raw_data, mpu6050_attitude_t *attitude);
esp_err_t MPU6050_CalibrateGyro(uint8_t dev_addr);
bool MPU6050_CheckConnection(uint8_t dev_addr);
esp_err_t MPU6050_SetGyroRange(uint8_t dev_addr, mpu6050_gyro_range_t range);
esp_err_t MPU6050_SetAccelRange(uint8_t dev_addr, mpu6050_accel_range_t range);
esp_err_t MPU6050_ReadRawData(uint8_t dev_addr, mpu6050_raw_data_t *data);
esp_err_t MPU6050_ReadData(uint8_t dev_addr, mpu6050_data_t *data);
esp_err_t MPU6050_ReadID(uint8_t dev_addr, uint8_t *id);

#endif /* __MPU6050_H__ */
