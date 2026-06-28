#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float pitch;
    float roll;
    float base_angle;
    float shoulder_angle;
    float elbow_angle;
    float wrist_angle;
    float gripper_angle;
    int num_stations;
    int mode; /* 0=manual, 1=Phone IMU+PID */

    /* PID反馈数据（仅pitch有闭环） */
    float pid_pitch_error;
    float pid_pitch_output;

    /* 目标姿态 */
    float target_pitch;
    float target_roll;
} servo_status_t;

typedef void (*web_command_callback_t)(const char *command, float value);

esp_err_t WebServer_Init(web_command_callback_t callback);
void WebServer_SendStatus(const servo_status_t *status);

#ifdef __cplusplus
}
#endif

#endif
