#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <stdint.h>
#include <stdbool.h>

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

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
    float l3;  // 腕关节到摇臂末端长度 (mm)，不包含夹爪
    float gripper_length;  // 夹爪长度（摇臂末端到夹爪尖端）(mm)
    float base_height;  // 底座高度 (mm)
} ArmParams;

// 全局机械臂参数（使用默认值初始化）
extern ArmParams g_arm_params;

// 逆运动学姿态配置
typedef enum {
    KINEMATIC_ANY = 0,      // 任意有效解（首个匹配即返回）
    KINEMATIC_GRASP_A,      // 抓取姿态A: J1<90°, J2>90°, J3<90°（从上方向下抓取）
    KINEMATIC_GRASP_B,      // 抓取姿态B: J1>90°, J2<90°, J3>90°（从下方向上抓取）
} KinematicConfig;

/**
 * @brief 初始化逆运动学模块，设置机械臂结构参数
 * @param params 机械臂结构参数
 */
void Kinematics_Init(const ArmParams *params);

/**
 * @brief 逆运动学求解：从末端坐标计算关节角度
 * @param pos 末端执行器坐标
 * @param angles 输出的关节角度
 * @param config 姿态配置偏好（KINEMATIC_ANY=任意，GRASP_A/B=特定抓取姿态）
 * @return true表示求解成功，false表示目标点不可达
 */
bool Kinematics_Inverse(const EndEffectorPos *pos, JointAngles *angles, KinematicConfig config);

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

/**
 * @brief 逆运动学求解：使用迭代逼近方法（雅可比矩阵）
 * @param pos 末端执行器坐标
 * @param angles 输出的关节角度
 * @param max_iter 最大迭代次数
 * @param tolerance 收敛容差（mm）
 * @return true表示求解成功，false表示未收敛或目标点不可达
 */
bool Kinematics_Inverse_Iterative(const EndEffectorPos *pos, JointAngles *angles,
                                   int max_iter, float tolerance);

#endif /* __KINEMATICS_H__ */
