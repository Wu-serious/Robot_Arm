#ifndef __SCRIPT_H__
#define __SCRIPT_H__

#include <stdint.h>
#include "Kinematics.h"

/* 前置声明，避免在组件公共头文件中引入 FreeRTOS 头文件依赖 */
typedef struct QueueDefinition * QueueHandle_t;

/** 两点抓取移动：从起点抓取物体，移动到终点放置 */
void KinematicScript_MoveObject(const EndEffectorPos *start_pos,
                                const EndEffectorPos *end_pos,
                                uint16_t step_delay_ms,
                                QueueHandle_t control_queue);

/** 挥手动画：右臂(J2)左右摆动 3 次 */
void Wave_Animate(QueueHandle_t control_queue);

/** 全部舵机归位到 90°（底座→肩→肘→腕→夹爪顺序） */
void Arm_Reset(QueueHandle_t control_queue);

/** 单舵机角度设置：指定通道和目标角度 */
void Servo_SetAngle_Single(QueueHandle_t control_queue, uint8_t channel, float angle);

/** 夹爪开/关：angle=0 张开, angle=90 闭合 */
void Gripper_Control(QueueHandle_t control_queue, float angle);

/** 夹爪张开（便捷封装） */
void Gripper_Open(QueueHandle_t control_queue);

/** 夹爪闭合（便捷封装） */
void Gripper_Closed(QueueHandle_t control_queue);

#endif /* __SCRIPT_H__ */
