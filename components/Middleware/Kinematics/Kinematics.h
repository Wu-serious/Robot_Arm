#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <stdint.h>
#include <stdbool.h>

// 机械臂关节角度结构体
typedef struct {
    float joint0;  // 底座旋转角度 (0-180度)
    float joint1;  // 肩关节角度 (0-180度)
    float joint2;  // 肘关节角度 (0-180度)
    float joint3;  // 腕关节角度 (0-180度)
} JointAngles;

// 末端执行器坐标结构体
typedef struct {
    float x;  // X坐标 (mm)
    float y;  // Y坐标 (mm)
    float z;  // Z坐标 (mm)
} EndEffectorPos;

// 机械臂结构参数（可根据实际硬件调整）
typedef struct {
    float l1;  // 肩关节到肘关节长度 (mm)
    float l2;  // 肘关节到腕关节长度 (mm)
    float l3;  // 腕关节到末端执行器长度 (mm)
    float base_height;  // 底座高度 (mm)
} ArmParams;

// 全局机械臂参数（使用默认值初始化）
extern ArmParams g_arm_params;

/**
 * @brief 初始化逆运动学模块，设置机械臂结构参数
 * @param params 机械臂结构参数
 */
void Kinematics_Init(const ArmParams *params);

/**
 * @brief 逆运动学求解：从末端坐标计算关节角度
 * @param pos 末端执行器坐标
 * @param angles 输出的关节角度
 * @return true表示求解成功，false表示目标点不可达
 */
bool Kinematics_Inverse(const EndEffectorPos *pos, JointAngles *angles);

/**
 * @brief 正运动学求解：从关节角度计算末端坐标
 * @param angles 关节角度
 * @param pos 输出的末端执行器坐标
 */
void Kinematics_Forward(const JointAngles *angles, EndEffectorPos *pos);

/**
 * @brief 检查目标点是否在机械臂工作空间内
 * @param pos 目标坐标
 * @return true表示可达，false表示不可达
 */
bool Kinematics_CheckReachable(const EndEffectorPos *pos);

#endif /* __KINEMATICS_H__ */