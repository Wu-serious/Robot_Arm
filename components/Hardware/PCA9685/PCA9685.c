#include "PCA9685.h"
#include "PCA9685_reg.h"
#include "I2C.h"
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

/** 日志标签 */
static const char *TAG = "PCA9685";

/**
 * @brief 向PCA9685寄存器写入数据
 * @param dev_addr I2C设备地址
 * @param reg 寄存器地址
 * @param value 要写入的值
 * @return ESP_OK成功，其他失败
 */
static esp_err_t PCA9685_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    return I2C_MasterWriteToReg(dev_addr, reg, &value, 1);
}

/**
 * @brief 从PCA9685寄存器读取数据
 * @param dev_addr I2C设备地址
 * @param reg 寄存器地址
 * @param value 读取到的值
 * @return ESP_OK成功，其他失败
 */
static esp_err_t PCA9685_ReadReg(uint8_t dev_addr, uint8_t reg, uint8_t *value)
{
    return I2C_MasterReadFromReg(dev_addr, reg, value, 1);
}

/**
 * @brief 初始化PCA9685
 * @param dev_addr I2C设备地址
 * @return ESP_OK成功，其他失败
 * 
 * 配置MODE1和MODE2寄存器，设置默认PWM频率为50Hz
 */
esp_err_t PCA9685_Init(uint8_t dev_addr)
{
    /** 配置MODE1寄存器: 启用地址自动递增(AI位)
     * 使能AI后，连续写入多个寄存器时地址会自动递增
     */
    esp_err_t err = PCA9685_WriteReg(dev_addr, PCA9685_MODE1, PCA9685_MODE1_AI);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Init MODE1 failed: %s", esp_err_to_name(err));
        return err;
    }

    /** 配置MODE2寄存器: 设置推挽输出模式(OUTDRV位)
     * 推挽模式可直接驱动舵机、LED等负载，提供更大的驱动能力
     */
    err = PCA9685_WriteReg(dev_addr, PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Init MODE2 failed: %s", esp_err_to_name(err));
        return err;
    }

    /** 设置默认PWM频率为50Hz(舵机常用频率) */
    return PCA9685_SetPWMFreq(dev_addr, 50.0f);
}

/**
 * @brief 设置PWM频率
 * @param dev_addr I2C设备地址
 * @param freq_hz 目标频率(Hz)，范围约24Hz-1526Hz
 * @return ESP_OK成功，其他失败
 * 
 * 计算公式: prescale = round(25MHz / (4096 * freq)) - 1
 * 内部使用25MHz晶振，PWM分辨率为12位(4096级)
 */
esp_err_t PCA9685_SetPWMFreq(uint8_t dev_addr, float freq_hz)
{
    if (freq_hz <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const float osc_clock = 25000000.0f;
    float prescale_val = (osc_clock / (4096.0f * freq_hz)) - 1.0f;
    uint8_t prescale = (uint8_t)(floorf(prescale_val + 0.5f));

    uint8_t old_mode;// 保存当前MODE1寄存器值
    esp_err_t err = PCA9685_ReadReg(dev_addr, PCA9685_MODE1, &old_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read MODE1 failed: %s", esp_err_to_name(err));
        return err;
    }

    /** 进入睡眠模式以设置预分频器 */
    uint8_t sleep_mode = (old_mode & ~PCA9685_MODE1_RESTART)\
     | PCA9685_MODE1_SLEEP;//清除RESTART位，进入睡眠模式
    err = PCA9685_WriteReg(dev_addr, PCA9685_MODE1, sleep_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Enter sleep failed: %s", esp_err_to_name(err));
        return err;
    }

    /** 写入预分频值 */
    err = PCA9685_WriteReg(dev_addr, PCA9685_PRE_SCALE, prescale);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write PRE_SCALE failed: %s", esp_err_to_name(err));
        return err;
    }

    /** 唤醒芯片 */
    err = PCA9685_WriteReg(dev_addr, PCA9685_MODE1, old_mode | PCA9685_MODE1_AI);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wake from sleep failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    
    /** 重启PWM控制器 */
    err = PCA9685_WriteReg(dev_addr, PCA9685_MODE1, (old_mode | \
        PCA9685_MODE1_RESTART | PCA9685_MODE1_AI) & ~PCA9685_MODE1_SLEEP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Restart failed: %s", esp_err_to_name(err));
    }

    return err;
}

/**
 * @brief 设置单个通道PWM
 * @param dev_addr I2C设备地址
 * @param channel 通道号(0-15)
 * @param on 开启时间(0-4096)，4096表示常开
 * @param off 关闭时间(0-4096)，4096表示常闭
 * @return ESP_OK成功，其他失败
 * 
 * PWM周期为4096个时钟周期，on和off决定一个周期内高电平的开始和结束位置
 */
esp_err_t PCA9685_SetPWM(uint8_t dev_addr, uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15 || on > 4096 || off > 4096) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t on_l = (uint8_t)(on & 0xFF);
    uint8_t on_h = (uint8_t)(on >> 8);
    uint8_t off_l = (uint8_t)(off & 0xFF);
    uint8_t off_h = (uint8_t)(off >> 8);

    /** 值为4096时表示特殊标志位(全开或全关) */
    if (on == 4096) {
        on_l = 0;
        on_h = 0x10;
    }
    if (off == 4096) {
        off_l = 0;
        off_h = 0x10;
    }

    uint8_t data[4] = {
        on_l,
        on_h,
        off_l,
        off_h
    };

    return I2C_MasterWriteToReg(dev_addr, PCA9685_LED0_ON_L + 4 * channel, data, sizeof(data));
}

/**
 * @brief 设置单个通道占空比
 * @param dev_addr I2C设备地址
 * @param channel 通道号(0-15)
 * @param duty_percent 占空比(0-100%)
 * @return ESP_OK成功，其他失败
 * 
 * 将百分比转换为12位PWM值，0%对应全关，100%对应全开
 */
esp_err_t PCA9685_SetDuty(uint8_t dev_addr, uint8_t channel, float duty_percent)
{
    if (channel > 15 || duty_percent < 0.0f || duty_percent > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (duty_percent <= 0.0f) {
        return PCA9685_SetPWM(dev_addr, channel, 0, 4096);
    }

    if (duty_percent >= 100.0f) {
        return PCA9685_SetPWM(dev_addr, channel, 4096, 0);
    }

    uint16_t off = (uint16_t)((duty_percent / 100.0f) * 4095.0f);
    return PCA9685_SetPWM(dev_addr, channel, 0, off);
}

/**
 * @brief 同时设置所有通道占空比
 * @param dev_addr I2C设备地址
 * @param duty_percent 占空比(0-100%)
 * @return ESP_OK成功，其他失败
 * 
 * 通过ALL_LED寄存器同时控制16个通道
 */
esp_err_t PCA9685_SetAllDuty(uint8_t dev_addr, float duty_percent)
{
    if (duty_percent < 0.0f || duty_percent > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (duty_percent <= 0.0f) {
        uint8_t data[4] = {0, 0, 0, 0x10};
        return I2C_MasterWriteToReg(dev_addr, PCA9685_ALL_LED_ON_L, data, sizeof(data));
    }

    if (duty_percent >= 100.0f) {
        uint8_t data[4] = {0, 0x10, 0, 0};
        return I2C_MasterWriteToReg(dev_addr, PCA9685_ALL_LED_ON_L, data, sizeof(data));
    }

    uint16_t off = (uint16_t)((duty_percent / 100.0f) * 4095.0f);
    uint8_t data[4] = {
        0,
        0,
        (uint8_t)(off & 0xFF),
        (uint8_t)(off >> 8)
    };

    return I2C_MasterWriteToReg(dev_addr, PCA9685_ALL_LED_ON_L, data, sizeof(data));
}
