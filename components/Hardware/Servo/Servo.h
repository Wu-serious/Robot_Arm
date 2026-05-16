#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>
#include <esp_err.h>

typedef struct {
    uint8_t channel;
    float angle;
} ServoCommand;

typedef struct {
    uint8_t count;
    ServoCommand commands[5];
} ServoCommandGroup;

esp_err_t Servo_Init(void);
esp_err_t Servo_SetAngle(uint8_t channel, float angle);

#endif
