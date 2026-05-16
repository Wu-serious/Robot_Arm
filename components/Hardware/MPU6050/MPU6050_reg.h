#ifndef __MPU6050_REG_H__
#define __MPU6050_REG_H__
#include <stdint.h>

// MPU6050 默认 I2C 地址
#define MPU6050_I2C_ADDR_DEFAULT    0x68
#define MPU6050_I2C_ADDR_ALT        0x69  // AD0=1 时的地址

// MPU6050 寄存器地址
#define MPU6050_WHO_AM_I            0x75  // 设备ID寄存器
#define MPU6050_PWR_MGMT_1          0x6B  // 电源管理寄存器1
#define MPU6050_PWR_MGMT_2          0x6C  // 电源管理寄存器2
#define MPU6050_CONFIG               0x1A  // 配置寄存器
#define MPU6050_GYRO_CONFIG          0x1B  // 陀螺仪配置寄存器
#define MPU6050_ACCEL_CONFIG         0x1C  // 加速度计配置寄存器
#define MPU6050_ACCEL_XOUT_H         0x3B  // 加速度计X轴高字节
#define MPU6050_ACCEL_XOUT_L         0x3C  // 加速度计X轴低字节
#define MPU6050_ACCEL_YOUT_H         0x3D  // 加速度计Y轴高字节
#define MPU6050_ACCEL_YOUT_L         0x3E  // 加速度计Y轴低字节
#define MPU6050_ACCEL_ZOUT_H         0x3F  // 加速度计Z轴高字节
#define MPU6050_ACCEL_ZOUT_L         0x40  // 加速度计Z轴低字节
#define MPU6050_GYRO_XOUT_H          0x43  // 陀螺仪X轴高字节
#define MPU6050_GYRO_XOUT_L          0x44  // 陀螺仪X轴低字节
#define MPU6050_GYRO_YOUT_H          0x45  // 陀螺仪Y轴高字节
#define MPU6050_GYRO_YOUT_L          0x46  // 陀螺仪Y轴低字节
#define MPU6050_GYRO_ZOUT_H          0x47  // 陀螺仪Z轴高字节
#define MPU6050_GYRO_ZOUT_L          0x48  // 陀螺仪Z轴低字节

#endif