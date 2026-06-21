#ifndef __PCA9685_H__
#define __PCA9685_H__

#include <stdint.h>
#include <esp_err.h>
#include "PCA9685_reg.h"


esp_err_t PCA9685_Init(uint8_t dev_addr);
esp_err_t PCA9685_SetPWMFreq(uint8_t dev_addr, float freq_hz);
esp_err_t PCA9685_SetPWM(uint8_t dev_addr, uint8_t channel, uint16_t on, uint16_t off);


#endif
