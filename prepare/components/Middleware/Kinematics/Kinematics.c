// ==========================================================================
// 机械臂运动学模块
// 坐标系定义：
// - 底座中心为原点(0,0,0)
// - X轴向前，Y轴向右，Z轴向上
//
// 关节角度定义（物理舵机角度，0-180°）：
// - joint0: 底座旋转，90°=向前
// - joint1: 肩关节，90°=向上，0°=向前，180°=向后
// - joint2: 肘关节，90°=伸直，0°=向后弯，180°=向前弯
// - joint3: 腕关节，90°=保持末端竖直向上
//
// 初始状态（所有关节=90°）：手臂笔直向上，夹爪竖直向上
//   x = 0, y = 0, z = base + l1 + l2 + l3 + gripper = 70+78+45+68+58 = 319mm
// ==========================================================================

#include "Kinematics.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"

#define TAG "Kinematics"

#define MAX_DELTA_PER_STEP_DEG 5.0f  // 迭代 IK 单步最大角度变化（度），防奇异点发散

ArmParams g_arm_params = {
    .l1 = 78.0f,
    .l2 = 45.0f,
    .l3 = 68.0f,              // 腕关节到摇臂末端
    .gripper_length = 58.0f,  // 夹爪长度 摇臂末端→夹爪尖端（需实测调整）
    .base_height = 70.0f
};

static void ComputeJacobian(const JointAngles *angles, float J[3][4]);

void Kinematics_Init(const ArmParams *params)
{
    if (params != NULL)
    {
        memcpy(&g_arm_params, params, sizeof(ArmParams));
    }
    ESP_LOGI(TAG, "Kinematics initialized: l1=%.1f l2=%.1f l3=%.1f grip=%.1f end=%.1f base=%.1f",
             g_arm_params.l1, g_arm_params.l2, g_arm_params.l3,
             g_arm_params.gripper_length,
             g_arm_params.l3 + g_arm_params.gripper_length,
             g_arm_params.base_height);
}

// ==========================================================================
// 正运动学求解
// 角度转换：
//   j0 = (joint0 - 90°) * PI/180
//   j1 = (90° - joint1) * PI/180
//   j2 = (joint2 - 90°) * PI/180
//   j3 = (joint3 - 90°) * PI/180
//
// 末端位置：
//   x = (l1*sin(j1) + l2*sin(j1+j2) + l3*sin(j1+j2+j3)) * cos(j0)
//   y = (l1*sin(j1) + l2*sin(j1+j2) + l3*sin(j1+j2+j3)) * sin(j0)
//   z = base_z + l1*cos(j1) + l2*cos(j1+j2) + l3*cos(j1+j2+j3)
// ==========================================================================
void Kinematics_Forward(const JointAngles *angles, EndEffectorPos *pos)
{
    if (angles == NULL || pos == NULL)
    {
        ESP_LOGE(TAG, "Invalid input parameters");
        return;
    }

    float l1 = g_arm_params.l1;
    float l2 = g_arm_params.l2;
    float l3 = g_arm_params.l3;
    float gripper_len = g_arm_params.gripper_length;
    float base_z = g_arm_params.base_height;

    // 总末端长度 = 摇臂长度 + 夹爪长度
    float end_len = l3 + gripper_len;

    float j0 = (angles->joint0 - 90.0f) * (M_PI / 180.0f);
    float j1 = (90.0f - angles->joint1) * (M_PI / 180.0f);
    float j2 = (angles->joint2 - 90.0f) * (M_PI / 180.0f);
    float j3 = (angles->joint3 - 90.0f) * (M_PI / 180.0f);

    float c0 = cosf(j0);
    float s0 = sinf(j0);

    float j12 = j1 + j2;
    float j123 = j12 + j3;

    // 使用总末端长度计算夹爪尖端位置
    float horizontal = l1 * sinf(j1) + l2 * sinf(j12) + end_len * sinf(j123);

    pos->x = horizontal * c0;
    pos->y = horizontal * s0;
    pos->z = base_z + l1 * cosf(j1) + l2 * cosf(j12) + end_len * cosf(j123);
}

// ==========================================================================
// 逆运动学求解：几何方法
//
// 核心公式推导：
//   腕关节到肩关节的距离：wrist_d² = l1² + l2² + 2*l1*l2*cos(j2)
//   三角形内角（肘关节）：θ = acos((l1²+l2²-wrist_d²)/(2*l1*l2))
//   关系：j2 = π - θ（肘部向上），j2 = θ - π（肘部向下）
//   joint2 = 90 + j2*180/π
//
//   关节2范围 [0°, 180°] → j2 范围 [-π/2, π/2]
//   → 最小腕关节距离 = sqrt(l1²+l2²)，最大 = l1+l2
// ==========================================================================
// 对一组关节解打分：J2/J3姿态匹配为主（硬约束），J1由运动偏好决定（软约束）
// J1不限制方向，只要求尽量接近90°（肩关节活动最小）
// GRASP_A: J2>90°(肘部向前), J3<90°(腕关节向前)
// GRASP_B: J2<90°(肘部向后), J3>90°(腕关节向后)
static float ScoreConfig(float j1_rad, float j2_rad, float j3_rad,
                         float joint1, float joint2, float joint3, KinematicConfig config)
{
    if (config == KINEMATIC_ANY) return 1.0f;

    int config_score = 0;
    float penalty = 1.0f;

    if (config == KINEMATIC_GRASP_A) {
        // J2>90°(肘部向前) — 核心抓取约束
        if (j2_rad > 0.0f) config_score += 3;
        else penalty *= 0.1f;

        // J3<90°(腕关节向前/朝下) — 核心抓取约束
        if (j3_rad < 0.0f) config_score += 3;
        else penalty *= 0.1f;
    } else if (config == KINEMATIC_GRASP_B) {
        // J2<90°(肘部向后) — 核心抓取约束
        if (j2_rad < 0.0f) config_score += 2;
        else penalty *= 0.3f;

        // J3>90°(腕关节向后/朝上) — 核心抓取约束
        if (j3_rad > 0.0f) config_score += 2;
        else penalty *= 0.3f;
    }

    // 运动偏好：J1由接近度决定（不论方向），J1越接近90°分越高
    float j1_closeness = 1.0f - fabsf(joint1 - 90.0f) / 90.0f;  // 0~1

    // J2中等变化：在|J2-90|=45°附近得分最高
    float j2_diff = fabsf(joint2 - 90.0f);
    float j2_mid_range = 1.0f - (j2_diff - 45.0f) * (j2_diff - 45.0f) / (45.0f * 45.0f);
    j2_mid_range = fmaxf(0.0f, j2_mid_range);

    // J3偏离度：|J3-90|越大越好（腕关节承担主要定位工作）
    float j3_deviation = fabsf(joint3 - 90.0f) / 90.0f;  // 0~1

    // J1接近度55% + J2中等10% + J3偏离35%
    float movement_bonus = (j1_closeness * 0.55f + j2_mid_range * 0.10f + j3_deviation * 0.35f) * 0.099f;

    return ((float)config_score + movement_bonus) * penalty;
}

bool Kinematics_Inverse(const EndEffectorPos *pos, JointAngles *angles, KinematicConfig config)
{
    if (pos == NULL || angles == NULL)
    {
        ESP_LOGE(TAG, "Invalid input parameters");
        return false;
    }

    float x = pos->x;
    float y = pos->y;
    float z = pos->z;
    float l1 = g_arm_params.l1;
    float l2 = g_arm_params.l2;
    float l3 = g_arm_params.l3;
    float gripper_len = g_arm_params.gripper_length;
    float base_z = g_arm_params.base_height;

    // 总末端长度 = 摇臂长度 + 夹爪长度（用于逆运动学求解）
    float end_len = l3 + gripper_len;

    // ========== Y轴对称优化 ==========
    // 对于 y<0 的目标，将坐标翻转到 y>0 侧求解 J1/J2/J3，
    // 得到姿态后把 J0 对称映射：J0' = 180° - J0。
    // 这保证 (x,y) 和 (x,-y) 得到完全一致的臂姿态。
    bool flipped_y = (y < 0.0f);
    float solve_x = x;
    float solve_y = flipped_y ? -y : y;

    // ========== 关节0：底座旋转角度 ==========
    float j0_raw = atan2f(solve_y, solve_x);
    float j0_candidates[2];
    float r_ee_signs[2];
    int n_candidates = 0;

    float j0_1 = j0_raw;
    float joint0_1 = 90.0f + j0_1 * (180.0f / M_PI);
    if (joint0_1 >= 0.0f && joint0_1 <= 180.0f) {
        j0_candidates[n_candidates] = j0_1;
        r_ee_signs[n_candidates] = 1.0f;
        n_candidates++;
    }

    float j0_2 = j0_raw + (j0_raw >= 0.0f ? -M_PI : M_PI);
    float joint0_2 = 90.0f + j0_2 * (180.0f / M_PI);
    if (joint0_2 >= 0.0f && joint0_2 <= 180.0f) {
        j0_candidates[n_candidates] = j0_2;
        r_ee_signs[n_candidates] = -1.0f;
        n_candidates++;
    }

    // ========== 计算目标点在臂平面中的坐标 ==========
    float r = sqrtf(solve_x * solve_x + solve_y * solve_y);
    float ee_z = z - base_z;
    float ee_d = sqrtf(r * r + ee_z * ee_z);

    float arm_max = l1 + l2 + end_len;
    if (ee_d > arm_max + 10.0f)  // 放宽边界条件
    {
        ESP_LOGW(TAG, "Out of reach: ee_d=%.2f > arm_max=%.2f", ee_d, arm_max);
        return false;
    }

    float l1l2_min_sq = l1 * l1 + l2 * l2;
    float l1l2_max = l1 + l2;

    // 追踪最优解（浮点分：整数=姿态匹配，小数=运动偏好）
    float best_score = 0.0f;
    float best_j0_rad = 0.0f;
    float best_j1 = 0.0f, best_j2 = 0.0f, best_j3 = 0.0f;

    for (int c = 0; c < n_candidates; c++)
    {
        float j0_rad = j0_candidates[c];
        float ee_angle = atan2f(r * r_ee_signs[c], ee_z);

        for (int j2_int = 0; j2_int <= 180; j2_int++)
        {
            float joint2 = (float)j2_int;
            float j2_rad = (joint2 - 90.0f) * (M_PI / 180.0f);
            float wd_sq = l1 * l1 + l2 * l2 + 2.0f * l1 * l2 * cosf(j2_rad);
            if (wd_sq < l1l2_min_sq - 0.01f) continue;

            float wd = sqrtf(wd_sq);
            if (wd > l1l2_max + 0.5f) continue;

            float cos_alpha = (wd_sq + ee_d * ee_d - end_len * end_len) / (2.0f * wd * ee_d);
            if (cos_alpha < -1.0f || cos_alpha > 1.0f) continue;
            float alpha = acosf(cos_alpha);

            for (int sign = -1; sign <= 1; sign += 2)
            {
                float wrist_angle = ee_angle + (float)sign * alpha;
                float w_r = wd * sinf(wrist_angle);
                float w_z = wd * cosf(wrist_angle);

                float r_eff = r * r_ee_signs[c];

                float w2e_dx = r_eff - w_r;
                float w2e_dz = ee_z - w_z;
                float check_dist = sqrtf(w2e_dx * w2e_dx + w2e_dz * w2e_dz);
                if (fabsf(check_dist - end_len) > 0.5f) continue;

                float cos_beta = (l1 * l1 + wd_sq - l2 * l2) / (2.0f * l1 * wd);
                if (cos_beta < -1.0f || cos_beta > 1.0f) continue;

                float A = l1 + l2 * cosf(j2_rad);
                float B_xy = l2 * sinf(j2_rad);
                float R = sqrtf(A * A + B_xy * B_xy);
                if (R < 0.01f) continue;

                float max_r = wd;
                if (w_r > max_r + 0.5f) continue;

                float ratio = w_r / R;
                if (ratio < -1.0f || ratio > 1.0f) continue;

                float j1_v1 = asinf(ratio) - atan2f(B_xy, A);
                float j1_v2 = M_PI - asinf(ratio) - atan2f(B_xy, A);

                for (int v = 0; v < 2; v++)
                {
                    float j1_rad = (v == 0) ? j1_v1 : j1_v2;
                    if (j1_rad < -M_PI / 2.0f || j1_rad > M_PI / 2.0f) continue;

                    float fk_wz = l1 * cosf(j1_rad) + l2 * cosf(j1_rad + j2_rad);
                    if (fabsf(fk_wz - w_z) > 0.5f) continue;

                    float j12_rad = j1_rad + j2_rad;

                    // 腕关节有两种可能的姿态（原始和相差π的备选），尝试两种解
                    float j3_base = atan2f(w2e_dx, w2e_dz) - j12_rad;
                    for (int wrist_sign = 0; wrist_sign <= 1; wrist_sign++)
                    {
                        float j3_rad = j3_base;
                        if (wrist_sign == 1) {
                            // 备选：翻转腕关节方向（±π）
                            if (j3_rad >= 0.0f) j3_rad -= M_PI;
                            else j3_rad += M_PI;
                        }

                        float joint1 = 90.0f - j1_rad * (180.0f / M_PI);
                        float joint3 = 90.0f + j3_rad * (180.0f / M_PI);

                        if (joint1 >= 0.0f && joint1 <= 180.0f && joint3 >= 0.0f && joint3 <= 180.0f)
                        {
                            float score = ScoreConfig(j1_rad, j2_rad, j3_rad,
                                                      joint1, joint2, joint3, config);
                            if (score > best_score)
                            {
                                best_score = score;
                                best_j0_rad = j0_rad;
                                best_j1 = joint1;
                                best_j2 = joint2;
                                best_j3 = joint3;

                                // 完美姿态匹配(≥7.0)或ANY模式立即返回
                                if (score >= 7.0f || config == KINEMATIC_ANY)
                                {
                                    angles->joint0 = 90.0f + best_j0_rad * (180.0f / M_PI);
                                    if (flipped_y) angles->joint0 = 180.0f - angles->joint0;
                                    angles->joint1 = best_j1;
                                    angles->joint2 = best_j2;
                                    angles->joint3 = best_j3;
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (best_score > 0.0f)
    {
        angles->joint0 = 90.0f + best_j0_rad * (180.0f / M_PI);
        if (flipped_y) angles->joint0 = 180.0f - angles->joint0;
        angles->joint1 = best_j1;
        angles->joint2 = best_j2;
        angles->joint3 = best_j3;
        if (best_score < 3.0f)
        {
            ESP_LOGW(TAG, "Partial config match (score=%.1f/3) for (%.1f,%.1f,%.1f)",
                     best_score, x, y, z);
        }
        return true;
    }

    ESP_LOGW(TAG, "No valid solution for (%.1f, %.1f, %.1f), ee_d=%.1f", x, y, z, ee_d);
    return false;
}

bool Kinematics_CheckReachable(const EndEffectorPos *pos)
{
    if (pos == NULL)
    {
        return false;
    }

    float x = pos->x;
    float y = pos->y;
    float z = pos->z - g_arm_params.base_height;

    float d = sqrtf(x * x + y * y + z * z);
    float max_dist = g_arm_params.l1 + g_arm_params.l2 + g_arm_params.l3 + g_arm_params.gripper_length;

    return (d <= max_dist + 5.0f);
}

static void ComputeJacobian(const JointAngles *angles, float J[3][4])
{
    float l1 = g_arm_params.l1;
    float l2 = g_arm_params.l2;
    float l3 = g_arm_params.l3;
    float gripper_len = g_arm_params.gripper_length;
    float end_len = l3 + gripper_len;

    float j0 = (angles->joint0 - 90.0f) * (M_PI / 180.0f);
    float j1 = (90.0f - angles->joint1) * (M_PI / 180.0f);
    float j2 = (angles->joint2 - 90.0f) * (M_PI / 180.0f);
    float j3 = (angles->joint3 - 90.0f) * (M_PI / 180.0f);

    float c0 = cosf(j0);
    float s0 = sinf(j0);

    float j12 = j1 + j2;
    float j123 = j12 + j3;

    float s_sum = l1 * sinf(j1) + l2 * sinf(j12) + end_len * sinf(j123);
    float c1 = cosf(j1);
    float c12 = cosf(j12);
    float c123 = cosf(j123);

    J[0][0] = -s_sum * s0;
    J[1][0] = s_sum * c0;
    J[2][0] = 0.0f;

    J[0][1] = (l1 * c1 + l2 * c12 + end_len * c123) * c0;
    J[1][1] = (l1 * c1 + l2 * c12 + end_len * c123) * s0;
    J[2][1] = -s_sum;

    J[0][2] = (l2 * c12 + end_len * c123) * c0;
    J[1][2] = (l2 * c12 + end_len * c123) * s0;
    J[2][2] = -(l2 * sinf(j12) + end_len * sinf(j123));

    J[0][3] = end_len * c123 * c0;
    J[1][3] = end_len * c123 * s0;
    J[2][3] = -end_len * sinf(j123);
}

bool Kinematics_Inverse_Iterative(const EndEffectorPos *pos, JointAngles *angles,
                                   int max_iter, float tolerance)
{
    if (pos == NULL || angles == NULL)
    {
        ESP_LOGE(TAG, "Invalid input parameters");
        return false;
    }

    if (!Kinematics_CheckReachable(pos))
    {
        ESP_LOGW(TAG, "Target position is out of workspace");
        return false;
    }

    JointAngles initial_angles;
    if (Kinematics_Inverse(pos, &initial_angles, KINEMATIC_ANY))
    {
        *angles = initial_angles;
    }
    else
    {
        float dx = sqrtf(pos->x * pos->x + pos->y * pos->y);
        float dz = pos->z - g_arm_params.base_height;
        float target_d = sqrtf(dx * dx + dz * dz);

        initial_angles.joint0 = 90.0f + atan2f(pos->y, pos->x) * (180.0f / M_PI);

        if (target_d > 1.0f)
        {
            float angle = atan2f(dx, dz);
            float shoulder = angle * (180.0f / M_PI);
            initial_angles.joint1 = CLAMP(90.0f - shoulder, 10.0f, 170.0f);
            float ratio = CLAMP(target_d / (g_arm_params.l1 + g_arm_params.l2 + g_arm_params.l3 + g_arm_params.gripper_length), 0.0f, 1.0f);
            initial_angles.joint2 = CLAMP(90.0f + 90.0f * ratio, 30.0f, 150.0f);
            initial_angles.joint3 = CLAMP(90.0f, 10.0f, 170.0f);
        }
        else
        {
            initial_angles.joint0 = 90.0f;
            initial_angles.joint1 = 90.0f;
            initial_angles.joint2 = 90.0f;
            initial_angles.joint3 = 90.0f;
        }

        initial_angles.joint0 = CLAMP(initial_angles.joint0, 0.0f, 180.0f);
        *angles = initial_angles;
    }

    float J[3][4];
    float error[3];
    float delta_theta[4];
    float prev_delta[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    EndEffectorPos current_pos;

    float alpha = 0.5f;
    float momentum = 0.3f;

    for (int iter = 0; iter < max_iter; iter++)
    {
        Kinematics_Forward(angles, &current_pos);

        error[0] = pos->x - current_pos.x;
        error[1] = pos->y - current_pos.y;
        error[2] = pos->z - current_pos.z;

        float error_norm = sqrtf(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);

        if (error_norm < tolerance)
        {
            return true;
        }

        ComputeJacobian(angles, J);

        for (int i = 0; i < 4; i++)
        {
            float grad = 0.0f;
            for (int j = 0; j < 3; j++)
            {
                grad += J[j][i] * error[j];
            }
            delta_theta[i] = alpha * grad + momentum * prev_delta[i];
            // 限制单步最大角度变化，防奇异点处发散
            float delta_deg = delta_theta[i] * (180.0f / M_PI);
            if (delta_deg > MAX_DELTA_PER_STEP_DEG)  delta_deg  = MAX_DELTA_PER_STEP_DEG;
            if (delta_deg < -MAX_DELTA_PER_STEP_DEG) delta_deg = -MAX_DELTA_PER_STEP_DEG;
            delta_theta[i] = delta_deg * (M_PI / 180.0f);
            prev_delta[i] = delta_theta[i];
        }

        angles->joint0 += delta_theta[0] * (180.0f / M_PI);
        angles->joint1 -= delta_theta[1] * (180.0f / M_PI);
        angles->joint2 += delta_theta[2] * (180.0f / M_PI);
        angles->joint3 += delta_theta[3] * (180.0f / M_PI);

        angles->joint0 = CLAMP(angles->joint0, 0.0f, 180.0f);
        angles->joint1 = CLAMP(angles->joint1, 0.0f, 180.0f);
        angles->joint2 = CLAMP(angles->joint2, 0.0f, 180.0f);
        angles->joint3 = CLAMP(angles->joint3, 0.0f, 180.0f);

        if (error_norm > 50.0f)
        {
            alpha = 0.8f;
            momentum = 0.5f;
        }
        else if (error_norm > 20.0f)
        {
            alpha = 0.5f;
            momentum = 0.4f;
        }
        else if (error_norm > 5.0f)
        {
            alpha = 0.2f;
            momentum = 0.3f;
        }
        else
        {
            alpha = 0.05f;
            momentum = 0.1f;
        }
    }

    Kinematics_Forward(angles, &current_pos);
    float dx = pos->x - current_pos.x;
    float dy = pos->y - current_pos.y;
    float dz = pos->z - current_pos.z;
    float final_error = sqrtf(dx * dx + dy * dy + dz * dz);

    ESP_LOGW(TAG, "Did not converge after %d iterations, final error=%.3fmm",
             max_iter, final_error);
    return false;
}
