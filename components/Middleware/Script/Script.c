#include "Script.h"
#include "Servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>

static const char *TAG = "KinematicScript";

#define SERVO_CHAN_J0 0
#define SERVO_CHAN_J1 1
#define SERVO_CHAN_J2 2
#define SERVO_CHAN_J3 3
#define SERVO_CHAN_GRIPPER 4  // 夹爪舵机通道


// 通过队列发送单舵机指令的辅助函数
static bool GripperViaQueue(QueueHandle_t q, float angle)
{
    ServoCommandGroup g;
    g.count = 1;
    g.commands[0].channel = SERVO_CHAN_GRIPPER;
    g.commands[0].angle = angle;
    return (xQueueSend(q, &g, pdMS_TO_TICKS(100)) == pdTRUE);
}

static bool JointViaQueue(QueueHandle_t q, uint8_t ch, float angle)
{
    ServoCommandGroup g;
    g.count = 1;
    g.commands[0].channel = ch;
    g.commands[0].angle = angle;
    return (xQueueSend(q, &g, pdMS_TO_TICKS(100)) == pdTRUE);
}

// 发送舵机指令组，失败时打印警告
static void SendGroup(QueueHandle_t q, ServoCommandGroup *g)
{
    if (!xQueueSend(q, g, pdMS_TO_TICKS(100))) {
        ESP_LOGW(TAG, "Queue send failed (count=%d), skipping", g->count);
    }
}

// ===========================================================================
// Script 公共接口实现
// ===========================================================================

/** 单舵机角度设置（非阻塞队列发送） */
void Servo_SetAngle_Single(QueueHandle_t control_queue, uint8_t channel, float angle)
{
    JointViaQueue(control_queue, channel, angle);
}

/** 夹爪控制：angle=0 张开, angle=90 闭合 */
void Gripper_Control(QueueHandle_t control_queue, float angle)
{
    GripperViaQueue(control_queue, angle);
    vTaskDelay(pdMS_TO_TICKS(500));
}

/** 夹爪张开 */
void Gripper_Open(QueueHandle_t control_queue)
{
    Gripper_Control(control_queue, 0.0f);
}

/** 夹爪闭合 */
void Gripper_Closed(QueueHandle_t control_queue)
{
    Gripper_Control(control_queue, 90.0f);
}

/** 全部舵机归位到 90°（底座→肩→肘→腕→夹爪顺序） */
void Arm_Reset(QueueHandle_t control_queue)
{
    ESP_LOGI(TAG, "Arm_Reset: 全部舵机归位到 90°");

    ServoCommandGroup g;
    g.count = 1;

    const uint8_t channels[] = {0, 1, 2, 3, 4};
    for (int i = 0; i < 5; i++) {
        g.commands[0].channel = channels[i];
        g.commands[0].angle = 90.0f;
        SendGroup(control_queue, &g);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* 无需维护 s_current_angles（Servo_Task 已跟踪实际舵机状态） */
}

/** 挥手动画：J2(肘关节) 左右摆动 3 次，每次 600ms */
void Wave_Animate(QueueHandle_t control_queue)
{
    ESP_LOGI(TAG, "Wave_Animate: 挥手动画启动");

    ServoCommandGroup g;
    g.count = 1;
    g.commands[0].channel = SERVO_CHAN_J2;

    for (int i = 0; i < 6; i++) {
        g.commands[0].angle = (i % 2 == 0) ? 50.0f : 130.0f;
        SendGroup(control_queue, &g);
        vTaskDelay(pdMS_TO_TICKS(600));
    }

    // 最后回到 90°
    g.commands[0].angle = 90.0f;
    SendGroup(control_queue, &g);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "Wave_Animate: 挥手动画完成");
}

// 两点抓取移动：从起点抓取物体，移动到终点放置
// 所有舵机指令通过 control_queue 发送，避免与体感控制任务竞争I2C总线
void KinematicScript_MoveObject(const EndEffectorPos *start_pos,
                                const EndEffectorPos *end_pos,
                                uint16_t step_delay_ms,
                                QueueHandle_t control_queue)
{
    JointAngles angles;
    static const char *TAG = "MoveObject";

    KinematicConfig start_config = (start_pos->x >= 0.0f) ? KINEMATIC_GRASP_A : KINEMATIC_GRASP_B;
    KinematicConfig end_config   = (end_pos->x   >= 0.0f) ? KINEMATIC_GRASP_A : KINEMATIC_GRASP_B;

    ESP_LOGI(TAG, "=== 两点抓取任务 ===");
    ESP_LOGI(TAG, "  起点(%.1f,%.1f,%.1f) → %s",
             start_pos->x, start_pos->y, start_pos->z,
             start_config == KINEMATIC_GRASP_A ? "GRASP_A(向前)" : "GRASP_B(向后)");
    ESP_LOGI(TAG, "  终点(%.1f,%.1f,%.1f) → %s",
             end_pos->x, end_pos->y, end_pos->z,
             end_config == KINEMATIC_GRASP_A ? "GRASP_A(向前)" : "GRASP_B(向后)");

    // 步骤1：IK求解抓取位置，打开夹爪 + 只转动J0底盘
    ESP_LOGI(TAG, "步骤 1：底盘转向抓取方向");
    if (!Kinematics_Inverse(start_pos, &angles, start_config))
    {
        ESP_LOGE(TAG, "  抓取位置不可达");
        return;
    }
    GripperViaQueue(control_queue, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
    JointViaQueue(control_queue, SERVO_CHAN_J0, angles.joint0);
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

    // 步骤2：依次伸出J1/J2/J3
    ESP_LOGI(TAG, "步骤 2：伸出机械臂 J1=%.1f J2=%.1f J3=%.1f",
             angles.joint1, angles.joint2, angles.joint3);
    if (!JointViaQueue(control_queue, SERVO_CHAN_J1, angles.joint1))
        ESP_LOGW(TAG, "  步骤 2: J1 发送失败，跳过");
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    if (!JointViaQueue(control_queue, SERVO_CHAN_J2, angles.joint2))
        ESP_LOGW(TAG, "  步骤 2: J2 发送失败，跳过");
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    if (!JointViaQueue(control_queue, SERVO_CHAN_J3, angles.joint3))
        ESP_LOGW(TAG, "  步骤 2: J3 发送失败，跳过");
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

    // 步骤3：闭合夹爪
    ESP_LOGI(TAG, "步骤 3：闭合夹爪抓取");
    if (!GripperViaQueue(control_queue, 90.0f))
        ESP_LOGW(TAG, "  步骤 3: 夹爪闭合发送失败");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 步骤4：肩关节和肘关节同时恢复90°
    ESP_LOGI(TAG, "步骤 4：肩关节+肘关节恢复90°");
    {
        ServoCommandGroup g;
        g.count = 2;
        g.commands[0].channel = SERVO_CHAN_J1;
        g.commands[0].angle = 90.0f;
        g.commands[1].channel = SERVO_CHAN_J2;
        g.commands[1].angle = 90.0f;
        SendGroup(control_queue, &g);
    }
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

    // 步骤5：IK求解放置位置，只转动J0底盘
    ESP_LOGI(TAG, "步骤 5：底盘转向放置方向");
    if (!Kinematics_Inverse(end_pos, &angles, end_config))
    {
        ESP_LOGE(TAG, "  放置位置不可达");
        return;
    }
    if (!JointViaQueue(control_queue, SERVO_CHAN_J0, angles.joint0))
        ESP_LOGW(TAG, "  步骤 5: J0 发送失败，跳过");
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

    // 步骤6：伸出J1/J2/J3并释放物品
    ESP_LOGI(TAG, "步骤 6：伸出机械臂并释放 J1=%.1f J2=%.1f J3=%.1f",
             angles.joint1, angles.joint2, angles.joint3);
    if (!JointViaQueue(control_queue, SERVO_CHAN_J1, angles.joint1))
        ESP_LOGW(TAG, "  步骤 6: J1 发送失败，跳过");
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    if (!JointViaQueue(control_queue, SERVO_CHAN_J2, angles.joint2))
        ESP_LOGW(TAG, "  步骤 6: J2 发送失败，跳过");
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    if (!JointViaQueue(control_queue, SERVO_CHAN_J3, angles.joint3))
        ESP_LOGW(TAG, "  步骤 6: J3 发送失败，跳过");
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    if (!GripperViaQueue(control_queue, 0.0f))
        ESP_LOGW(TAG, "  步骤 6: 夹爪张开发送失败");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 步骤7：恢复初始姿态
    ESP_LOGI(TAG, "步骤 7：恢复初始姿态");
    Arm_Reset(control_queue);

    ESP_LOGI(TAG, "=== 抓取任务完成 ===");
}
