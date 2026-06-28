/**
 * @file main.c
 * @brief 四自由度机械臂 手机IMU体感控制 + MPU6050末端PID闭环反馈
 *
 * 系统架构：
 * - ESP32-P4 主控 (Host)
 * - ESP32-C5 协处理器 (Slave) - 通过SDIO接口连接，提供WiFi功能
 * - 手机IMU：通过WebSocket发送手机姿态，作为目标输入
 * - MPU6050（末端）：安装在机械臂末端，测量实际姿态用于PID闭环反馈
 * - PCA9685 驱动 5 路舵机
 *
 * 控制模式：
 * 0 = Manual  手动滑条控制
 * 1 = Phone IMU  手机IMU体感控制 + MPU6050末端PID闭环修正
 *
 * 任务结构：
 * 1. IMU_Feedback_Task  - 100Hz 末端MPU6050反馈数据采集
 * 2. Control_Task       - 50Hz  PID闭环控制 + 命令生成
 * 3. Servo_Task         - 50Hz  舵机指令执行
 * 4. WebStatus_Task     - 20Hz  Web状态发布
 *
 * PID闭环控制逻辑：
 *   - pitch: 手机IMU → 目标pitch → map_IMU_To_Servos → MPU6050末端pitch反馈 → PID修正腕关节(ch3)
 *   - roll:  手机IMU → 直接映射底座旋转(ch0)，无需PID（底座旋转不改变MPU6050 roll）
 *   - 夹爪: 独立按键控制张/关(ch4)
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "WiFi.h"
#include "WebServer.h"
#include "I2C.h"
#include "Servo.h"
#include "MPU6050.h"
#include "Kinematics.h"
#include "Script.h"

/* WiFi AP credentials — override via sdkconfig.defaults if desired */
#ifndef WIFI_SSID
#define WIFI_SSID      "Arm_AP"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD  "12345678"
#endif

// ==========================================================================
// PID 控制器
// ==========================================================================

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
} PIDController;

static void PID_Init(PIDController *pid, float kp, float ki, float kd, float i_limit, float o_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = i_limit;
    pid->output_limit = o_limit;
}

static float PID_Compute(PIDController *pid, float error, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float p = pid->kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }
    float i = pid->ki * pid->integral;

    float d = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = p + i + d;
    if (output > pid->output_limit)
    {
        output = pid->output_limit;
    }
    if (output < -pid->output_limit)
    {
        output = -pid->output_limit;
    }
    return output;
}

static void PID_Reset(PIDController *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// ==========================================================================
// 数据结构
// ==========================================================================

/* IMU 反馈数据结构：与 mpu6050_attitude_t 相同，供队列使用 */
typedef mpu6050_attitude_t ImuData;

// ==========================================================================
// 系统状态封装（替代分散的全局变量）
// ==========================================================================

/* 舵机通道索引常量 */
#define CH_BASE     0
#define CH_SHOULDER 1
#define CH_ELBOW    2
#define CH_WRIST    3
#define CH_GRIPPER  4
#define NUM_CHANNELS 5

/* 共享状态结构体：所有任务通过此结构体通信，避免裸全局变量 */
typedef struct {
    /* 控制模式 */
    volatile int  imu_mode;           /* 0=手动, 1=手机IMU+PID */

    /* 手机IMU目标角度 */
    volatile float phone_pitch;
    volatile float phone_roll;

    /* 当前舵机角度（Servo_Task 写入，WebStatus_Task 读取） */
    float servo_angle[NUM_CHANNELS];  /* [base, shoulder, elbow, wrist, gripper] */

    /* MPU6050 末端姿态（IMU_Feedback_Task 写入，WebStatus_Task 读取） */
    float mpu_pitch;
    float mpu_roll;

    /* PID 状态（Control_Task 写入，WebStatus_Task 读取） */
    volatile float pid_pitch_error;
    volatile float pid_pitch_output;
} SystemState_t;

/* 全局系统状态实例 */
static SystemState_t g_sys = {
    .imu_mode       = 0,
    .phone_pitch    = 0.0f,
    .phone_roll     = 0.0f,
    .servo_angle    = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f},
    .mpu_pitch      = 0.0f,
    .mpu_roll       = 0.0f,
    .pid_pitch_error= 0.0f,
    .pid_pitch_output= 0.0f,
};

/* 同步原语（在 app_main 中创建） */
static SemaphoreHandle_t xI2CMutex = NULL;
static QueueHandle_t xFeedbackQueue = NULL;
static QueueHandle_t xControlQueue = NULL;

// ==========================================================================
// 角度映射模块（手机IMU → 目标舵机角度）
// ==========================================================================

/* 映射系数：通过实验标定，使 IMU 倾斜角度均匀分配到各关节 */
#define IMU_SHOULDER_RATIO 0.5f
#define IMU_ELBOW_RATIO    0.8f
#define IMU_WRIST_RATIO    1.2f

/* Δ-limiter 阈值：陀螺仪在 ±90° 附近符号翻转（-89°→+89°），
 * 超过此值的跳变视为伪影，直接丢弃 */
#define DELTA_LIMIT_THRESHOLD 170.0f

/* 手机IMU姿态 → 舵机目标角度
 *   roll → 底座(ch0) : 90+roll ，roll=-90→底座0°   roll=0→底座90°   roll=90→底座180°
 *   pitch→ 肩(ch1) : 90-pitch*IMU_SHOULDER_RATIO ，pitch>0（前倾）→肩<90°（前伸）
 *   pitch→ 肘(ch2) : 90+pitch*IMU_ELBOW_RATIO ，pitch>0（前倾）→肘>90°（外扩）
 *   pitch→ 腕(ch3) : 90-pitch*IMU_WRIST_RATIO ，腕关节幅度最大，保持末端水平
 *
 *   Δ-limiter: 手机陀螺仪在 ±90° 附近会符号翻转（-89→+89），
 *   限制底座每步最大变化量，防止底盘猛烈甩动                           */
static void map_IMU_To_Servos(float pitch, float roll, ServoCommandGroup *group)
{
    /* 底座：roll直接叠加，clamp到[0, 180]，跳变>170° 判定为传感器翻转，丢弃 */
    static float s_prev_raw_base  = 90.0f;

    float raw_base = 90.0f + roll;
    float delta = raw_base - s_prev_raw_base;
    if (delta > DELTA_LIMIT_THRESHOLD || delta < -DELTA_LIMIT_THRESHOLD) {
        raw_base = s_prev_raw_base;   /* 传感器翻转 artifact → 保持原位 */
    } else {
        s_prev_raw_base = raw_base;   /* 正常移动 → 更新记忆 */
    }

    float base = raw_base;
    if (base <   0.0f) base =   0.0f;
    if (base > 180.0f) base = 180.0f;

    /* 肩关节：pitch反向，clamp到[60, 120] */
    float shoulder = 90.0f - pitch * IMU_SHOULDER_RATIO;
    if (shoulder <  60.0f) shoulder =  60.0f;
    if (shoulder > 120.0f) shoulder = 120.0f;

    /* 肘关节：pitch同向，clamp到[40, 140] */
    float elbow = 90.0f + pitch * IMU_ELBOW_RATIO;
    if (elbow <  40.0f) elbow =  40.0f;
    if (elbow > 140.0f) elbow = 140.0f;

    /* 腕关节：pitch反向，幅度最大，clamp到[20, 160] */
    float wrist = 90.0f - pitch * IMU_WRIST_RATIO;
    if (wrist <  20.0f) wrist =  20.0f;
    if (wrist > 160.0f) wrist = 160.0f;

    group->count = 4;
    group->commands[0].channel = CH_BASE;    group->commands[0].angle = base;
    group->commands[1].channel = CH_SHOULDER; group->commands[1].angle = shoulder;
    group->commands[2].channel = CH_ELBOW;    group->commands[2].angle = elbow;
    group->commands[3].channel = CH_WRIST;    group->commands[3].angle = wrist;
}

// ==========================================================================
// 任务：末端MPU6050反馈数据采集（100Hz）
// ==========================================================================

void IMU_Feedback_Task(void *pvParameters)
{
    (void)pvParameters;

    mpu6050_data_t imu_raw;
    mpu6050_attitude_t attitude;

    static float filtered_pitch = 0.0f;
    static float filtered_roll = 0.0f;
    const float imu_filter_alpha = 0.3f;

    while (1)
    {
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            esp_err_t err = MPU6050_ReadData(MPU6050_I2C_ADDR_DEFAULT, &imu_raw);
            if (err == ESP_OK)
            {
                MPU6050_FuseData(&imu_raw, &attitude);

                filtered_pitch = imu_filter_alpha * attitude.pitch + (1 - imu_filter_alpha) * filtered_pitch;
                filtered_roll = imu_filter_alpha * attitude.roll + (1 - imu_filter_alpha) * filtered_roll;

                g_sys.mpu_pitch = filtered_pitch;
                g_sys.mpu_roll  = filtered_roll;

                xQueueOverwrite(xFeedbackQueue, &attitude);
            }
            xSemaphoreGive(xI2CMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================================================
// 任务：PID闭环控制（50Hz）
// ==========================================================================

void Control_Task(void *pvParameters)
{
    (void)pvParameters;

    mpu6050_attitude_t feedback;
    ServoCommandGroup target_group;

    /* 舵机平滑滤波 */
    static float filtered_base = 90.0f;
    static float filtered_shoulder = 90.0f;
    static float filtered_elbow = 90.0f;
    static float filtered_wrist = 90.0f;
    const float servo_filter_alpha = 0.8f;

    /* 防抖参数 */
    float last_base = 90.0f;
    float last_shoulder = 90.0f;
    float last_elbow = 90.0f;
    float last_wrist = 90.0f;
    const float threshold = 1.0f;
    const TickType_t min_interval = pdMS_TO_TICKS(60);
    TickType_t last_tick = xTaskGetTickCount();

    /* PID控制器：基于雅可比伪逆的4自由度姿态闭环
     * error = [phone_pitch - mpu_pitch] (1×1)
     * Jacobian = [∂pitch/∂J1, ∂pitch/∂J2, ∂pitch/∂J3] (1×3)
     * 伪逆 J⁺ = Jᵀ / (J·Jᵀ)  (3×1)
     * joint_correction = J⁺ * PID_output  (3×1) */
    PIDController pid_pitch; /* 末端姿态误差 → 修正量 */
    PID_Init(&pid_pitch, 1.0f, 0.02f, 0.05f, 30.0f, 15.0f);

    /* 雅可比伪逆缩放因子：控制修正强度，避免奇异点处放大过度 */
    const float jacobi_gain = 0.8f;

    TickType_t prev_pid_tick = xTaskGetTickCount();

    static bool imu_first_entry = true;

    while (1)
    {
        bool feedback_ok = (xQueueReceive(xFeedbackQueue, &feedback, pdMS_TO_TICKS(50)) == pdTRUE);

        if (g_sys.imu_mode == 1)
        {
            /* ========== 手机IMU + PID闭环 ========== */
            if (imu_first_entry) {
                /* 首次进入IMU模式：reset PID + 用当前IMU值初始化滤波状态 */
                PID_Reset(&pid_pitch);
                filtered_base     = 90.0f + g_sys.phone_roll;
                filtered_shoulder = 90.0f - g_sys.phone_pitch * IMU_SHOULDER_RATIO;
                filtered_elbow    = 90.0f + g_sys.phone_pitch * IMU_ELBOW_RATIO;
                filtered_wrist    = 90.0f - g_sys.phone_pitch * IMU_WRIST_RATIO;
                imu_first_entry = false;
            }

            map_IMU_To_Servos(g_sys.phone_pitch, g_sys.phone_roll, &target_group);

            /* PID闭环修正：基于雅可比伪逆将末端pitch误差分配到J1/J2/J3 */
            if (feedback_ok)
            {
                float dt = (float)(xTaskGetTickCount() - prev_pid_tick) * portTICK_PERIOD_MS / 1000.0f;
                if (dt <= 0.0f)
                {
                    dt = 0.02f;
                }
                prev_pid_tick = xTaskGetTickCount();

                /* 末端姿态误差 */
                float pitch_error = g_sys.phone_pitch - feedback.pitch;
                float pid_output = PID_Compute(&pid_pitch, pitch_error, dt);

                /* 构建当前目标角度的 JointAngles 结构 */
                JointAngles current_target;
                current_target.joint0 = target_group.commands[0].angle;
                current_target.joint1 = target_group.commands[1].angle;
                current_target.joint2 = target_group.commands[2].angle;
                current_target.joint3 = target_group.commands[3].angle;

                /* 计算 pitch 对 J1/J2/J3 的偏导（复用运动学参数） */
                float dPitch_dJ[3];
                {
                    float l1 = g_arm_params.l1;
                    float l2 = g_arm_params.l2;
                    float end_len = g_arm_params.l3 + g_arm_params.gripper_length;

                    float j1 = (90.0f - current_target.joint1) * (M_PI / 180.0f);
                    float j2 = (current_target.joint2 - 90.0f) * (M_PI / 180.0f);
                    float j3 = (current_target.joint3 - 90.0f) * (M_PI / 180.0f);
                    float j12 = j1 + j2;
                    float j123 = j12 + j3;

                    float h  = l1 * sinf(j1) + l2 * sinf(j12) + end_len * sinf(j123);
                    float ez = l1 * cosf(j1) + l2 * cosf(j12) + end_len * cosf(j123);
                    float denom_sq = h * h + ez * ez;

                    if (denom_sq < 0.01f)
                    {
                        dPitch_dJ[0] = dPitch_dJ[1] = dPitch_dJ[2] = 0.0f;
                    }
                    else
                    {
                        float dh_dj1 =  l1 * cosf(j1) + l2 * cosf(j12) + end_len * cosf(j123);
                        float dh_dj2 =  l2 * cosf(j12) + end_len * cosf(j123);
                        float dh_dj3 =  end_len * cosf(j123);
                        float dez_dj1 = -l1 * sinf(j1) - l2 * sinf(j12) - end_len * sinf(j123);
                        float dez_dj2 = -l2 * sinf(j12) - end_len * sinf(j123);
                        float dez_dj3 = -end_len * sinf(j123);

                        dPitch_dJ[0] = (ez * dh_dj1 - h * dez_dj1) / denom_sq;
                        dPitch_dJ[1] = (ez * dh_dj2 - h * dez_dj2) / denom_sq;
                        dPitch_dJ[2] = (ez * dh_dj3 - h * dez_dj3) / denom_sq;
                    }
                }

                /* 雅可比伪逆：J⁺ = Jᵀ / (J·J) */
                float jacobi_dot = dPitch_dJ[0] * dPitch_dJ[0]
                                 + dPitch_dJ[1] * dPitch_dJ[1]
                                 + dPitch_dJ[2] * dPitch_dJ[2];

                if (jacobi_dot > 1e-6f)  /* 非奇异 */
                {
                    /* 伪逆 × PID输出 × 缩放 */
                    float correction[3];
                    for (int i = 0; i < 3; i++)
                    {
                        correction[i] = jacobi_gain * dPitch_dJ[i] * pid_output / jacobi_dot;
                    }

                    /* 将修正量叠加到 J1/J2/J3 */
                    target_group.commands[1].angle += correction[0];
                    target_group.commands[2].angle += correction[1];
                    target_group.commands[3].angle += correction[2];

                    /* 范围钳位 */
                    if (target_group.commands[1].angle < 60.0f) target_group.commands[1].angle = 60.0f;
                    if (target_group.commands[1].angle > 120.0f) target_group.commands[1].angle = 120.0f;
                    if (target_group.commands[2].angle < 40.0f) target_group.commands[2].angle = 40.0f;
                    if (target_group.commands[2].angle > 140.0f) target_group.commands[2].angle = 140.0f;
                    if (target_group.commands[3].angle < 20.0f) target_group.commands[3].angle = 20.0f;
                    if (target_group.commands[3].angle > 160.0f) target_group.commands[3].angle = 160.0f;
                }

                /* 更新PID状态供Web显示 */
                g_sys.pid_pitch_error = pitch_error;
                g_sys.pid_pitch_output = pid_output;
            }

            /* 舵机角度平滑滤波 */
            filtered_base = servo_filter_alpha * target_group.commands[0].angle + (1 - servo_filter_alpha) * filtered_base;
            filtered_shoulder = servo_filter_alpha * target_group.commands[1].angle + (1 - servo_filter_alpha) * filtered_shoulder;
            filtered_elbow = servo_filter_alpha * target_group.commands[2].angle + (1 - servo_filter_alpha) * filtered_elbow;
            filtered_wrist = servo_filter_alpha * target_group.commands[3].angle + (1 - servo_filter_alpha) * filtered_wrist;

            target_group.commands[0].angle = filtered_base;
            target_group.commands[1].angle = filtered_shoulder;
            target_group.commands[2].angle = filtered_elbow;
            target_group.commands[3].angle = filtered_wrist;

            /* 防抖发送 — 任一舵机角度变化超过阈值，或超过最小间隔则发送 */
            bool need_send = false;
            const float last_angles[4] = {last_base, last_shoulder, last_elbow, last_wrist};
            for (int i = 0; i < 4; i++) {
                float delta = target_group.commands[i].angle - last_angles[i];
                if (delta > threshold || delta < -threshold) {
                    need_send = true;
                    break;
                }
            }
            if (!need_send && (xTaskGetTickCount() - last_tick >= min_interval))
            {
                need_send = true;
            }

            if (need_send)
            {
                /* 只发送4个舵机角度（不包含夹爪） */
                target_group.count = 4;
                xQueueSend(xControlQueue, &target_group, pdMS_TO_TICKS(10));
                last_base = target_group.commands[0].angle;
                last_shoulder = target_group.commands[1].angle;
                last_elbow = target_group.commands[2].angle;
                last_wrist = target_group.commands[3].angle;
                last_tick = xTaskGetTickCount();

                static int cnt = 0;
                if (cnt++ % 200 == 0)
                {
                    printf("[Control] PhoneP=%.1f Roll=%.1f | FB_P=%.1f R=%.1f | Base=%.1f SH=%.1f EL=%.1f WR=%.1f\n",
                           g_sys.phone_pitch, g_sys.phone_roll,
                           feedback.pitch, feedback.roll,
                           last_base, last_shoulder, last_elbow, last_wrist);
                    printf("[PID]     P_Err=%.1f Out=%.1f\n",
                           g_sys.pid_pitch_error, g_sys.pid_pitch_output);
                }
            }
        }
        else
        {
            /* 手动模式下重置PID积分，标记下次进入IMU需重新初始化 */
            imu_first_entry = true;
            if (feedback_ok)
            {
                PID_Reset(&pid_pitch);
                g_sys.pid_pitch_error = 0.0f;
                g_sys.pid_pitch_output = 0.0f;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==========================================================================
// 任务：舵机执行（50Hz）
// ==========================================================================

/* 舵机指令发送：每次 I2C 写操作单独加锁，避免长时间持有锁导致 IMU 任务超时丢帧 */
static void SendServoCommand(uint8_t channel, float angle)
{
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Servo_SetAngle(channel, angle);
        g_sys.servo_angle[channel] = angle;
        xSemaphoreGive(xI2CMutex);
    }
    else
    {
        ESP_LOGW("Servo", "I2C lock timeout, dropping channel=%d angle=%.1f", channel, angle);
    }
}

void Servo_Task(void *pvParameters)
{
    (void)pvParameters;

    ServoCommandGroup group;

    while (1)
    {
        /* 阻塞等待队列：有指令才唤醒，无指令时让出 CPU */
        if (xQueueReceive(xControlQueue, &group, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            /* 逐舵机加锁发送，每个 I2C 写操作只持锁 ~1ms */
            for (uint8_t i = 0; i < group.count; i++)
            {
                SendServoCommand(group.commands[i].channel, group.commands[i].angle);
            }
        }
    }
}

// ==========================================================================
// Web命令回调函数
// ==========================================================================

static void web_command_handler(const char *command, float value)
{
    if (strcmp(command, "phone_pitch") != 0 && strcmp(command, "phone_roll") != 0 && strcmp(command, "heartbeat") != 0)
        ESP_LOGI("Web", "Cmd: %s val=%.2f", command, value);

    if (strcmp(command, "reset") == 0)
    {
        /* 全部舵机归位 */
        g_sys.imu_mode = 0;
        Arm_Reset(xControlQueue);
    }
    else if (strcmp(command, "wave") == 0)
    {
        /* 触发挥手动画 */
        g_sys.imu_mode = 0;
        Wave_Animate(xControlQueue);
    }
    else if (strcmp(command, "mode") == 0)
    {
        g_sys.imu_mode = (int)value;
        if (g_sys.imu_mode < 0) g_sys.imu_mode = 0;
        if (g_sys.imu_mode > 1) g_sys.imu_mode = 1;
        const char *names[] = {"Manual", "Phone IMU+PID"};
        printf("Mode: %s\n", names[g_sys.imu_mode]);
    }
    else if (strcmp(command, "phone_pitch") == 0)
    {
        g_sys.phone_pitch = value;
    }
    else if (strcmp(command, "phone_roll") == 0)
    {
        g_sys.phone_roll = value;
    }
    else if (strncmp(command, "servo", 5) == 0)
    {
        /* "servo0"~"servo4" → 单舵机控制 */
        int ch = command[5] - '0';
        if (ch >= 0 && ch <= 4) {
            g_sys.imu_mode = 0;
            Servo_SetAngle_Single(xControlQueue, (uint8_t)ch, value);
        }
    }
    else if (strcmp(command, "gripper_open") == 0)
    {
        /* 夹爪控制不退出 IMU 模式，允许在体感控制中随时开合夹爪 */
        Gripper_Open(xControlQueue);
    }
    else if (strcmp(command, "gripper_close") == 0)
    {
        /* 夹爪控制不退出 IMU 模式，允许在体感控制中随时开合夹爪 */
        Gripper_Closed(xControlQueue);
    }
    else if (strcmp(command, "kinematic_move") == 0)
    {
        /* 无坐标参数：使用默认硬编码位置 */
        EndEffectorPos start_pos = {0.0f, -80.0f, 50.0f};
        EndEffectorPos end_pos   = {80.0f, 0.0f, 50.0f};
        KinematicScript_MoveObject(&start_pos, &end_pos, 500, xControlQueue);
    }
    else if (strncmp(command, "kmove ", 6) == 0)
    {
        /* WebSocket传入坐标：格式 "kmove sx sy sz ex ey ez" */
        g_sys.imu_mode = 0;
        EndEffectorPos start_pos, end_pos;
        if (sscanf(command, "kmove %f %f %f %f %f %f",
                   &start_pos.x, &start_pos.y, &start_pos.z,
                   &end_pos.x, &end_pos.y, &end_pos.z) == 6) {
            ESP_LOGI("Web", "Kinematic move: (%.0f,%.0f,%.0f) -> (%.0f,%.0f,%.0f)",
                     start_pos.x, start_pos.y, start_pos.z,
                     end_pos.x, end_pos.y, end_pos.z);
            KinematicScript_MoveObject(&start_pos, &end_pos, 500, xControlQueue);
        } else {
            ESP_LOGW("Web", "Invalid kmove format: %s", command);
        }
    }
}

// ==========================================================================
// 任务：Web状态发布（20Hz）
// ==========================================================================

void WebStatus_Task(void *pvParameters)
{
    (void)pvParameters;

    servo_status_t status;
    wifi_status_t wifi_status;

    while (1)
    {
        status.pitch = g_sys.mpu_pitch;
        status.roll = g_sys.mpu_roll;
        status.base_angle = g_sys.servo_angle[CH_BASE];
        status.shoulder_angle = g_sys.servo_angle[CH_SHOULDER];
        status.elbow_angle = g_sys.servo_angle[CH_ELBOW];
        status.wrist_angle = g_sys.servo_angle[CH_WRIST];
        status.gripper_angle = g_sys.servo_angle[CH_GRIPPER];
        status.mode = g_sys.imu_mode;

        /* PID反馈数据 */
        status.pid_pitch_error = g_sys.pid_pitch_error;
        status.pid_pitch_output = g_sys.pid_pitch_output;

        status.target_pitch = g_sys.phone_pitch;

        WiFi_AP_GetStatus(&wifi_status);
        status.num_stations = wifi_status.num_stations;

        WebServer_SendStatus(&status);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==========================================================================
// 主程序入口
// ==========================================================================

void app_main(void)
{
    esp_err_t ret = ESP_OK;

    /* 关闭 ESP-IDF HTTP 库的冗余 WARN 日志 */
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_ws", ESP_LOG_ERROR);

    printf("Initializing WiFi AP via ESP32-C5 co-processor...\n");
    ret = WiFi_AP_Init(WIFI_SSID, WIFI_PASSWORD);
    if (ret != ESP_OK)
    {
        printf("Warning: WiFi AP initialization failed, continuing with hardware...\n");
    }
    else
    {
        printf("WiFi AP started: SSID=%s, IP=192.168.4.1\n", WIFI_SSID);
    }

    /* 创建同步原语 */
    xI2CMutex = xSemaphoreCreateMutex();
    if (xI2CMutex == NULL)
    {
        printf("Error: Failed to create I2C mutex!\n");
        return;
    }

    /* 创建队列 */
    xFeedbackQueue = xQueueCreate(1, sizeof(ImuData));
    if (xFeedbackQueue == NULL)
    {
        printf("Error: Failed to create feedback queue!\n");
        return;
    }

    xControlQueue = xQueueCreate(30, sizeof(ServoCommandGroup));
    if (xControlQueue == NULL)
    {
        printf("Error: Failed to create control queue!\n");
        return;
    }

    /* 初始化硬件 */
    printf("Initializing hardware...\n");

    ret = I2C_Init();
    if (ret != ESP_OK)
    {
        printf("Error: I2C_Init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    /* 初始化末端MPU6050（用于PID闭环反馈） */
    printf("Initializing MPU6050 (end-effector feedback)...\n");
    ret = MPU6050_Init(MPU6050_I2C_ADDR_DEFAULT);
    if (ret != ESP_OK)
    {
        printf("Error: MPU6050_Init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    printf("Initializing Servo...\n");
    ret = Servo_Init();
    if (ret != ESP_OK)
    {
        printf("Error: Servo_Init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    printf("Initializing Kinematics...\n");
    ArmParams arm_params = {
        .l1 = 78.0f,
        .l2 = 45.0f,
        .l3 = 68.0f,
        .gripper_length = 58.0f,
        .base_height = 70.0f
    };
    Kinematics_Init(&arm_params);

    printf("Hardware initialization complete!\n");

    /* 初始化Web服务器 */
    printf("Initializing Web Server...\n");
    ret = WebServer_Init(web_command_handler);
    if (ret != ESP_OK)
    {
        printf("Warning: Web Server initialization failed!\n");
    }
    else
    {
        printf("Web Server started: http://192.168.4.1\n");
    }

    printf("System ready!  Modes: 0=Manual  1=Phone IMU+PID\n");

    /* 创建任务 */
    xTaskCreate(IMU_Feedback_Task, "IMU_FB_Task", 4096, NULL, 5, NULL);
    xTaskCreate(Control_Task, "Control_Task", 4096, NULL, 5, NULL);
    xTaskCreate(Servo_Task, "Servo_Task", 4096, NULL, 5, NULL);
    xTaskCreate(WebStatus_Task, "WebStatus_Task", 4096, NULL, 4, NULL);

    vTaskDelete(NULL);
}
