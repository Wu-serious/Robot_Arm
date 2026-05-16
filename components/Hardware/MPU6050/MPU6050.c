#include "MPU6050_reg.h"
#include "MPU6050.h"
#include "I2C.h"
#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "MPU6050";

static bool s_address_initialized = false;
static uint8_t s_active_dev_addr = 0;

static uint8_t MPU6050_GetEffectiveAddress(uint8_t dev_addr)
{
    return s_address_initialized ? s_active_dev_addr : dev_addr;
}

// 静态变量存储当前量程设置
static mpu6050_gyro_range_t s_gyro_range = MPU6050_GYRO_RANGE_250DPS;
static mpu6050_accel_range_t s_accel_range = MPU6050_ACCEL_RANGE_2G;

// 陀螺仪零偏校准值
static float s_gyro_bias_x = 0.0f;
static float s_gyro_bias_y = 0.0f;
static float s_gyro_bias_z = 0.0f;

// 是否已校准
static bool s_gyro_calibrated = false;

/**
 * @brief 初始化 MPU6050
 * @param dev_addr I2C设备地址
 * @return ESP_OK成功，其他失败
 */
esp_err_t MPU6050_Init(uint8_t dev_addr)
{
    esp_err_t err;
    uint8_t active_addr = dev_addr;

    ESP_LOGI(TAG, "Initializing MPU6050 with address 0x%02X...", active_addr);
    bool connected = MPU6050_CheckConnection(active_addr);
    if (!connected) {
        ESP_LOGW(TAG, "MPU6050 not connected at 0x%02X, trying alternative address 0x%02X...", active_addr, MPU6050_I2C_ADDR_ALT);
        if (active_addr == MPU6050_I2C_ADDR_DEFAULT) {
            active_addr = MPU6050_I2C_ADDR_ALT;
            connected = MPU6050_CheckConnection(active_addr);
            if (!connected) {
                ESP_LOGE(TAG, "MPU6050 not found on both addresses 0x%02X and 0x%02X", MPU6050_I2C_ADDR_DEFAULT, MPU6050_I2C_ADDR_ALT);
                return ESP_ERR_NOT_FOUND;
            }
            ESP_LOGI(TAG, "MPU6050 connected successfully at alternate address 0x%02X", active_addr);
        } else {
            ESP_LOGE(TAG, "MPU6050 not connected at 0x%02X", active_addr);
            return ESP_ERR_NOT_FOUND;
        }
    } else {
        ESP_LOGI(TAG, "MPU6050 connected successfully at address 0x%02X", active_addr);
    }
    
    // 第二步：唤醒MPU6050（清除睡眠位）
    err = I2C_MasterWriteToReg(active_addr, MPU6050_PWR_MGMT_1, (uint8_t[]){0x00}, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t pwr_state = 0;
    err = I2C_MasterReadFromReg(active_addr, MPU6050_PWR_MGMT_1, &pwr_state, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read back PWR_MGMT_1: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "PWR_MGMT_1 after wake = 0x%02X", pwr_state);
    if (pwr_state & 0x40) {
        ESP_LOGW(TAG, "MPU6050 still appears in sleep mode after wake command");
    }

    s_active_dev_addr = active_addr;
    s_address_initialized = true;

    // 第三步：设置陀螺仪量程（默认±250°/s）
    err = MPU6050_SetGyroRange(active_addr, MPU6050_GYRO_RANGE_250DPS);
    if (err != ESP_OK) {
        return err;
    }
    
    // 第四步：设置加速度计量程（默认±2g）
    err = MPU6050_SetAccelRange(active_addr, MPU6050_ACCEL_RANGE_2G);
    if (err != ESP_OK) {
        return err;
    }
    
    // 第五步：陀螺仪校准
    err = MPU6050_CalibrateGyro(active_addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gyro calibration failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

/**
 * @brief 陀螺仪校准
 * @param dev_addr I2C设备地址
 * @return ESP_OK成功，其他失败
 * @note 校准时请保持传感器静止
 */
esp_err_t MPU6050_CalibrateGyro(uint8_t dev_addr)
{
    const int sample_count = 100;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    mpu6050_data_t data;
    
    ESP_LOGI(TAG, "Calibrating gyroscope... Keep sensor still!");
    
    // 等待传感器稳定
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 采集样本
    for (int i = 0; i < sample_count; i++) {
        esp_err_t err = MPU6050_ReadData(dev_addr, &data);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read data during calibration: %s", esp_err_to_name(err));
            return err;
        }
        sum_x += data.gyro_x;
        sum_y += data.gyro_y;
        sum_z += data.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // 计算平均零偏
    s_gyro_bias_x = sum_x / sample_count;
    s_gyro_bias_y = sum_y / sample_count;
    s_gyro_bias_z = sum_z / sample_count;
    s_gyro_calibrated = true;
    
    ESP_LOGI(TAG, "Gyro calibration completed");
    ESP_LOGI(TAG, "Gyro bias: X=%.2f, Y=%.2f, Z=%.2f", 
             s_gyro_bias_x, s_gyro_bias_y, s_gyro_bias_z);
    
    return ESP_OK;
}

/**
 * @brief 检查 MPU6050 是否连接
 * @param dev_addr I2C设备地址
 * @return true表示设备已连接，false表示未连接或通信失败
 */
bool MPU6050_CheckConnection(uint8_t dev_addr)
{
    uint8_t id;
    esp_err_t err = MPU6050_ReadID(dev_addr, &id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I from 0x%02X: %s", dev_addr, esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "WHO_AM_I from 0x%02X = 0x%02X", dev_addr, id);
    return (id == 0x68);
}

/**
 * @brief 设置陀螺仪量程
 * @param dev_addr I2C设备地址
 * @param range 陀螺仪量程
 * @return ESP_OK成功，其他失败
 */
esp_err_t MPU6050_SetGyroRange(uint8_t dev_addr, mpu6050_gyro_range_t range)
{
    uint8_t addr = MPU6050_GetEffectiveAddress(dev_addr);
    esp_err_t err = I2C_MasterWriteToReg(addr, MPU6050_GYRO_CONFIG, (uint8_t[]){range}, 1);
    if (err == ESP_OK) {
        s_gyro_range = range;
    }
    return err;
}

/**
 * @brief 设置加速度计量程
 * @param dev_addr I2C设备地址
 * @param range 加速度计量程
 * @return ESP_OK成功，其他失败
 */
esp_err_t MPU6050_SetAccelRange(uint8_t dev_addr, mpu6050_accel_range_t range)
{
    uint8_t addr = MPU6050_GetEffectiveAddress(dev_addr);
    esp_err_t err = I2C_MasterWriteToReg(addr, MPU6050_ACCEL_CONFIG, (uint8_t[]){range}, 1);
    if (err == ESP_OK) {
        s_accel_range = range;
    }
    return err;
}

/**
 * @brief 读取原始数据
 * @param dev_addr I2C设备地址
 * @param data 输出参数，原始数据结构体
 * @return ESP_OK成功，其他失败
 */
esp_err_t MPU6050_ReadRawData(uint8_t dev_addr, mpu6050_raw_data_t *data)
{
    uint8_t addr = MPU6050_GetEffectiveAddress(dev_addr);
    uint8_t buffer[14];
    
    // 从ACCEL_XOUT_H开始读取14字节数据，包括温度数据
    esp_err_t err = I2C_MasterReadFromReg(addr, MPU6050_ACCEL_XOUT_H, buffer, sizeof(buffer));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw data: %s", esp_err_to_name(err));
        return err;
    }

    // 组合高低字节
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    return ESP_OK;
}

/**
 * @brief 读取转换后的数据
 * @param dev_addr I2C设备地址
 * @param data 输出参数，转换后的数据结构体
 * @return ESP_OK成功，其他失败
 */
esp_err_t MPU6050_ReadData(uint8_t dev_addr, mpu6050_data_t *data)
{
    mpu6050_raw_data_t raw;
    esp_err_t err = MPU6050_ReadRawData(dev_addr, &raw);
    if (err != ESP_OK) {
        return err;
    }
    
    // 计算陀螺仪灵敏度系数
    float gyro_scale;
    switch (s_gyro_range) {
        case MPU6050_GYRO_RANGE_250DPS:  gyro_scale = 131.0f;  break;
        case MPU6050_GYRO_RANGE_500DPS:  gyro_scale = 65.5f;   break;
        case MPU6050_GYRO_RANGE_1000DPS: gyro_scale = 32.8f;   break;
        case MPU6050_GYRO_RANGE_2000DPS: gyro_scale = 16.4f;   break;
        default:                         gyro_scale = 131.0f;  break;
    }
    
    // 计算加速度计灵敏度系数
    float accel_scale;
    switch (s_accel_range) {
        case MPU6050_ACCEL_RANGE_2G:  accel_scale = 16384.0f; break;
        case MPU6050_ACCEL_RANGE_4G:  accel_scale = 8192.0f;  break;
        case MPU6050_ACCEL_RANGE_8G:  accel_scale = 4096.0f;  break;
        case MPU6050_ACCEL_RANGE_16G: accel_scale = 2048.0f;  break;
        default:                      accel_scale = 16384.0f; break;
    }
    
    // 转换数据
    data->accel_x = (float)raw.accel_x / accel_scale;
    data->accel_y = (float)raw.accel_y / accel_scale;
    data->accel_z = (float)raw.accel_z / accel_scale;
    data->gyro_x = (float)raw.gyro_x / gyro_scale;
    data->gyro_y = (float)raw.gyro_y / gyro_scale;
    data->gyro_z = (float)raw.gyro_z / gyro_scale;
    
    // 如果已校准，减去零偏
    if (s_gyro_calibrated) {
        data->gyro_x -= s_gyro_bias_x;
        data->gyro_y -= s_gyro_bias_y;
        data->gyro_z -= s_gyro_bias_z;
    }
    
    return ESP_OK;
}

/**
 * @brief 读取设备ID
 * @param dev_addr I2C设备地址
 * @param id 输出参数，设备ID（应为0x68）
 * @return ESP_OK成功，其他失败
 */
esp_err_t MPU6050_ReadID(uint8_t dev_addr, uint8_t *id)
{
    uint8_t addr = MPU6050_GetEffectiveAddress(dev_addr);
    return I2C_MasterReadFromReg(addr, MPU6050_WHO_AM_I, id, 1);
}

// 静态变量用于传感器融合
static float s_pitch = 0.0f;  // 当前俯仰角
static TickType_t s_last_time = 0;  // 上一次更新时间

/**
 * @brief 传感器融合（互补滤波）
 * @param raw_data 原始传感器数据
 * @param attitude 输出的姿态数据
 * @return ESP_OK成功
 * 
 * 使用互补滤波融合陀螺仪和加速度计数据，获得稳定的姿态估计
 * 陀螺仪提供快速响应，加速度计提供绝对角度参考
 */
esp_err_t MPU6050_FuseData(const mpu6050_data_t *raw_data, mpu6050_attitude_t *attitude)
{
    if (raw_data == NULL || attitude == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 获取当前时间
    TickType_t current_time = xTaskGetTickCount();
    
    // 计算时间差（秒）
    float dt = 0.0f;
    if (s_last_time != 0) {
        dt = (float)(current_time - s_last_time) / (float)configTICK_RATE_HZ;
    }
    s_last_time = current_time;

    // 从加速度计计算角度（X轴为pitch，左右倾斜）
    float accel_pitch = atan2(raw_data->accel_x, sqrt(raw_data->accel_y * raw_data->accel_y + raw_data->accel_z * raw_data->accel_z)) * (180.0f / 3.1415926535f);

    // 互补滤波权重系数
    // alpha越接近1，越信任陀螺仪；越接近0，越信任加速度计
    float alpha = 0.98f;

    if (dt > 0.0f)
    {
        // 使用陀螺仪积分更新角度
        s_pitch += raw_data->gyro_x * dt;

        // 融合加速度计数据（消除漂移）
        s_pitch = alpha * s_pitch + (1.0f - alpha) * accel_pitch;
    }
    else
    {
        // 首次初始化，使用加速度计数据
        s_pitch = accel_pitch;
    }

    // 设置输出
    attitude->pitch = s_pitch;

    return ESP_OK;
}