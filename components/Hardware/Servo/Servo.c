#include "Servo.h"
#include "I2C.h"
#include "PCA9685.h"
#include <esp_log.h>

static const char *TAG = "Servo";

esp_err_t Servo_Init(void)
{
    esp_err_t err = PCA9685_Init(PCA9685_I2C_ADDR_DEFAULT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685_Init failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = PCA9685_SetPWMFreq(PCA9685_I2C_ADDR_DEFAULT, 50.0f);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685_SetPWMFreq failed: %s", esp_err_to_name(err));
        return err;
    }
    
    /* 初始化使用的 5 个舵机通道（0=底座, 1=肩, 2=肘, 3=腕, 4=夹爪） */
    for (uint8_t channel = 0; channel < 5; channel++) {
        err = Servo_SetAngle(channel, 90.0f);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Servo_SetAngle(%d, 90) failed: %s", channel, esp_err_to_name(err));
            return err;
        }
    }
    
    ESP_LOGI(TAG, "Servo initialized successfully");
    return ESP_OK;
}

esp_err_t Servo_SetAngle(uint8_t channel, float angle)
{
    if (channel > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float pulse_width_ms = 0.5f + (angle / 180.0f) * 2.0f;
    uint16_t off_value = (uint16_t)((pulse_width_ms / 20.0f) * 4096.0f);
    
    return PCA9685_SetPWM(PCA9685_I2C_ADDR_DEFAULT, channel, 0, off_value);
}
