#include "Kinematics.h"
#include <math.h>
#include <esp_log.h>

static const char *TAG = "Kinematics";

// 全局机械臂参数
ArmParams g_arm_params = {
    .l1 = 100.0f,   // 肩关节到肘关节长度 (mm)
    .l2 = 100.0f,   // 肘关节到腕关节长度 (mm)
    .l3 = 50.0f,    // 腕关节到末端长度 (mm)
    .base_height = 50.0f  // 底座高度 (mm)
};

void Kinematics_Init(const ArmParams *params)
{
    if (params != NULL) {
        g_arm_params = *params;
    }
    
    ESP_LOGI(TAG, "Kinematics initialized with params: l1=%.1fmm, l2=%.1fmm, l3=%.1fmm, base=%.1fmm",
             g_arm_params.l1, g_arm_params.l2, g_arm_params.l3, g_arm_params.base_height);
}

bool Kinematics_Inverse(const EndEffectorPos *pos, JointAngles *angles)
{
    if (pos == NULL || angles == NULL) {
        ESP_LOGE(TAG, "Invalid input parameters");
        return false;
    }

    float x = pos->x;
    float y = pos->y;
    float z = pos->z - g_arm_params.base_height;  // 相对于肩关节的Z坐标

    // 关节0：底座旋转角度（绕Z轴）
    float joint0_rad = atan2(y, x);
    angles->joint0 = joint0_rad * (180.0f / M_PI);

    // 计算投影到XY平面的距离
    float r = sqrt(x * x + y * y);

    // 关节1和关节2：使用几何方法求解
    // 首先计算腕关节位置到肩关节的距离
    float d = sqrt(r * r + z * z);

    // 检查是否在工作空间内
    float min_dist = fabsf(g_arm_params.l2 - g_arm_params.l3);
    float max_dist = g_arm_params.l2 + g_arm_params.l3;
    
    if (d < min_dist - 1.0f || d > max_dist + 1.0f) {
        ESP_LOGE(TAG, "Target position out of workspace: d=%.2f, min=%.2f, max=%.2f", d, min_dist, max_dist);
        return false;
    }

    // 计算肘关节角度 (关节2)
    float cos_theta2 = (g_arm_params.l1 * g_arm_params.l1 + g_arm_params.l2 * g_arm_params.l2 - d * d) / 
                       (2.0f * g_arm_params.l1 * g_arm_params.l2);
    
    // 限制cos_theta2在[-1, 1]范围内
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;
    
    float theta2_rad = acos(cos_theta2);
    angles->joint2 = theta2_rad * (180.0f / M_PI);

    // 计算肩关节角度 (关节1)
    float alpha = atan2(z, r);
    float beta = acos((g_arm_params.l1 * g_arm_params.l1 + d * d - g_arm_params.l2 * g_arm_params.l2) / 
                     (2.0f * g_arm_params.l1 * d));
    
    // 限制beta的输入范围
    float beta_input = (g_arm_params.l1 * g_arm_params.l1 + d * d - g_arm_params.l2 * g_arm_params.l2) / 
                      (2.0f * g_arm_params.l1 * d);
    if (beta_input > 1.0f) beta_input = 1.0f;
    if (beta_input < -1.0f) beta_input = -1.0f;
    beta = acos(beta_input);
    
    float theta1_rad = alpha + beta;
    angles->joint1 = theta1_rad * (180.0f / M_PI);

    // 关节3：腕关节角度，保持末端执行器水平
    // theta3 = -(theta1 + theta2)
    angles->joint3 = -(angles->joint1 + angles->joint2 - 180.0f);

    // 将角度限制在0-180度范围内
    angles->joint0 = fmod(angles->joint0, 180.0f);
    if (angles->joint0 < 0) angles->joint0 += 180.0f;
    
    if (angles->joint1 < 0) angles->joint1 = 0;
    if (angles->joint1 > 180) angles->joint1 = 180;
    
    if (angles->joint2 < 0) angles->joint2 = 0;
    if (angles->joint2 > 180) angles->joint2 = 180;
    
    if (angles->joint3 < 0) angles->joint3 = 0;
    if (angles->joint3 > 180) angles->joint3 = 180;

    ESP_LOGD(TAG, "Inverse kinematics solved: J0=%.1f°, J1=%.1f°, J2=%.1f°, J3=%.1f°",
             angles->joint0, angles->joint1, angles->joint2, angles->joint3);

    return true;
}

void Kinematics_Forward(const JointAngles *angles, EndEffectorPos *pos)
{
    if (angles == NULL || pos == NULL) {
        ESP_LOGE(TAG, "Invalid input parameters");
        return;
    }

    // 将角度转换为弧度
    float theta0 = angles->joint0 * (M_PI / 180.0f);
    float theta1 = angles->joint1 * (M_PI / 180.0f);
    float theta2 = angles->joint2 * (M_PI / 180.0f);
    float theta3 = angles->joint3 * (M_PI / 180.0f);

    // 计算各关节位置
    // 肩关节位置（相对于底座）
    float shoulder_x = 0;
    float shoulder_y = 0;
    float shoulder_z = g_arm_params.base_height;

    // 肘关节位置
    float elbow_x = shoulder_x + g_arm_params.l1 * cos(theta1) * cos(theta0);
    float elbow_y = shoulder_y + g_arm_params.l1 * cos(theta1) * sin(theta0);
    float elbow_z = shoulder_z + g_arm_params.l1 * sin(theta1);

    // 腕关节位置
    float wrist_x = elbow_x + g_arm_params.l2 * cos(theta1 + theta2) * cos(theta0);
    float wrist_y = elbow_y + g_arm_params.l2 * cos(theta1 + theta2) * sin(theta0);
    float wrist_z = elbow_z + g_arm_params.l2 * sin(theta1 + theta2);

    // 末端执行器位置
    pos->x = wrist_x + g_arm_params.l3 * cos(theta1 + theta2 + theta3) * cos(theta0);
    pos->y = wrist_y + g_arm_params.l3 * cos(theta1 + theta2 + theta3) * sin(theta0);
    pos->z = wrist_z + g_arm_params.l3 * sin(theta1 + theta2 + theta3);

    ESP_LOGD(TAG, "Forward kinematics: X=%.2f, Y=%.2f, Z=%.2f", pos->x, pos->y, pos->z);
}

bool Kinematics_CheckReachable(const EndEffectorPos *pos)
{
    if (pos == NULL) {
        return false;
    }

    float x = pos->x;
    float y = pos->y;
    float z = pos->z - g_arm_params.base_height;

    float r = sqrt(x * x + y * y);
    float d = sqrt(r * r + z * z);

    float min_dist = fabsf(g_arm_params.l2 - g_arm_params.l3);
    float max_dist = g_arm_params.l2 + g_arm_params.l3;

    return (d >= min_dist - 1.0f) && (d <= max_dist + 1.0f);
}