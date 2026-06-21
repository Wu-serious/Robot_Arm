#include "Kinematics.h"
#include <math.h>
#include <esp_log.h>

static const char *TAG = "Kinematics";

// 全局机械臂参数（根据实际模型设置）
// 工作空间范围：min_dist = |l1-l2| = 33mm, max_dist = l1+l2 = 123mm
ArmParams g_arm_params = {
    .l1 = 78.0f,    // 肩关节到肘关节长度 (mm)
    .l2 = 45.0f,    // 肘关节到腕关节长度 (mm)
    .l3 = 68.0f,    // 腕关节到末端执行器长度 (mm)
    .base_height = 70.0f  // 底座高度 (mm)
};

// ==========================================================================
// 初始化逆运动学模块
// ==========================================================================
void Kinematics_Init(const ArmParams *params)
{
    if (params != NULL)
    {
        g_arm_params = *params;
    }
    
    ESP_LOGI(TAG, "Kinematics initialized: l1=%.1f, l2=%.1f, l3=%.1f, base=%.1f",
             g_arm_params.l1, g_arm_params.l2, g_arm_params.l3, g_arm_params.base_height);
}

// ==========================================================================
// 逆运动学求解：末端坐标 -> 关节角度
// ==========================================================================
bool Kinematics_Inverse(const EndEffectorPos *pos, JointAngles *angles)
{
    if (pos == NULL || angles == NULL)
    {
        ESP_LOGE(TAG, "Invalid input parameters");
        return false;
    }

    float x = pos->x;
    float y = pos->y;
    float z = pos->z - g_arm_params.base_height;

    // ========== 关节0：底座旋转角度 ==========
    // 坐标系定义：X轴向前，Y轴向右
    // 当y=0, x>0时，J0=90°（面向前方）
    // roll>0（右倾）→ Y>0 → J0>90°（右转）
    // roll<0（左倾）→ Y<0 → J0<90°（左转）
    float joint0_rad = atan2(y, x);
    angles->joint0 = 90.0f - (joint0_rad * (180.0f / M_PI));
    
    // 限制在0-180度范围内
    if (angles->joint0 < 0) angles->joint0 = 0;
    if (angles->joint0 > 180) angles->joint0 = 180;

    // ========== 工作空间检查 ==========
    float r = sqrt(x * x + y * y);
    float d = sqrt(r * r + z * z);
    
    float min_dist = fabsf(g_arm_params.l1 - g_arm_params.l2);
    float max_dist = g_arm_params.l1 + g_arm_params.l2;
    
    // 扩大范围检查，避免误判
    if (d < min_dist - 5.0f || d > max_dist + 5.0f)
    {
        ESP_LOGW(TAG, "Out of workspace: d=%.2f (min=%.2f, max=%.2f)", d, min_dist, max_dist);
        return false;
    }

    // ========== 关节2：肘关节角度 ==========
    float cos_theta2 = (g_arm_params.l1 * g_arm_params.l1 + g_arm_params.l2 * g_arm_params.l2 - d * d) / 
                       (2.0f * g_arm_params.l1 * g_arm_params.l2);
    
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;
    
    float theta2_rad = acos(cos_theta2);
    angles->joint2 = theta2_rad * (180.0f / M_PI);

    // ========== 关节1：肩关节角度 ==========
    float alpha = atan2(z, r);
    float beta_input = (g_arm_params.l1 * g_arm_params.l1 + d * d - g_arm_params.l2 * g_arm_params.l2) / 
                      (2.0f * g_arm_params.l1 * d);
    
    if (beta_input > 1.0f) beta_input = 1.0f;
    if (beta_input < -1.0f) beta_input = -1.0f;
    float beta = acos(beta_input);
    
    float theta1_rad = alpha + beta;
    angles->joint1 = theta1_rad * (180.0f / M_PI);

    // ========== 关节3：腕关节角度 ==========
    // 保持末端水平，腕关节角度需要补偿肩关节和肘关节
    // 当J1=90°, J2=90°时，J3=0°（保持水平）
    // 公式: J3 = 90° - J1 - J2 + 180° = 270° - J1 - J2
    angles->joint3 = 270.0f - angles->joint1 - angles->joint2;

    // ========== 角度范围限制（舵机安全范围）==========
    // 关节0：底座旋转 0-180
    if (angles->joint0 < 0) angles->joint0 = 0;
    if (angles->joint0 > 180) angles->joint0 = 180;
    
    // 关节1：肩关节 30-150
    if (angles->joint1 < 30) angles->joint1 = 30;
    if (angles->joint1 > 150) angles->joint1 = 150;
    
    // 关节2：肘关节 30-150
    if (angles->joint2 < 30) angles->joint2 = 30;
    if (angles->joint2 > 150) angles->joint2 = 150;
    
    // 关节3：腕关节 0-180
    if (angles->joint3 < 0) angles->joint3 = 0;
    if (angles->joint3 > 180) angles->joint3 = 180;

    ESP_LOGD(TAG, "Solved: J0=%.1f J1=%.1f J2=%.1f J3=%.1f",
             angles->joint0, angles->joint1, angles->joint2, angles->joint3);

    return true;
}

// ==========================================================================
// 正运动学求解
// ==========================================================================
void Kinematics_Forward(const JointAngles *angles, EndEffectorPos *pos)
{
    if (angles == NULL || pos == NULL)
    {
        ESP_LOGE(TAG, "Invalid input parameters");
        return;
    }

    float theta0 = angles->joint0 * (M_PI / 180.0f);
    float theta1 = angles->joint1 * (M_PI / 180.0f);
    float theta2 = angles->joint2 * (M_PI / 180.0f);
    float theta3 = angles->joint3 * (M_PI / 180.0f);

    float shoulder_z = g_arm_params.base_height;

    float elbow_x = g_arm_params.l1 * cos(theta1) * cos(theta0);
    float elbow_y = g_arm_params.l1 * cos(theta1) * sin(theta0);
    float elbow_z = shoulder_z + g_arm_params.l1 * sin(theta1);

    float wrist_x = elbow_x + g_arm_params.l2 * cos(theta1 + theta2) * cos(theta0);
    float wrist_y = elbow_y + g_arm_params.l2 * cos(theta1 + theta2) * sin(theta0);
    float wrist_z = elbow_z + g_arm_params.l2 * sin(theta1 + theta2);

    pos->x = wrist_x + g_arm_params.l3 * cos(theta1 + theta2 + theta3) * cos(theta0);
    pos->y = wrist_y + g_arm_params.l3 * cos(theta1 + theta2 + theta3) * sin(theta0);
    pos->z = wrist_z + g_arm_params.l3 * sin(theta1 + theta2 + theta3);

    ESP_LOGD(TAG, "Forward: X=%.2f Y=%.2f Z=%.2f", pos->x, pos->y, pos->z);
}

// ==========================================================================
// 检查目标点是否可达
// ==========================================================================
bool Kinematics_CheckReachable(const EndEffectorPos *pos)
{
    if (pos == NULL)
    {
        return false;
    }

    float x = pos->x;
    float y = pos->y;
    float z = pos->z - g_arm_params.base_height;

    float r = sqrt(x * x + y * y);
    float d = sqrt(r * r + z * z);

    float min_dist = fabsf(g_arm_params.l1 - g_arm_params.l2);
    float max_dist = g_arm_params.l1 + g_arm_params.l2;

    return (d >= min_dist - 5.0f) && (d <= max_dist + 5.0f);
}
