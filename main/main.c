/**
 * @file main.c
 * @brief 四自由度机械臂 MPU6050 体感控制主程序
 *
 * 系统架构：
 * - ESP32-P4 主控
 * - MPU6050 六轴传感器读取姿态
 * - PCA9685 驱动 5 路舵机
 *
 * 任务结构：
 * 1. IMU_Task     - 100Hz IMU数据采集
 * 2. Control_Task - 50Hz 姿态解算与命令生成
 * 3. Servo_Task   - 50Hz 舵机指令执行
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "I2C.h"
#include "Servo.h"
#include "MPU6050.h"

// ==========================================================================
// 数据结构
// ==========================================================================

typedef struct {
    float pitch;
    float roll;
    float yaw;
} ImuData;

// ==========================================================================
// 全局变量
// ==========================================================================

static SemaphoreHandle_t xI2CMutex = NULL;
static QueueHandle_t xIMUQueue = NULL;
static QueueHandle_t xControlQueue = NULL;

// ==========================================================================
// 角度映射模块
// ==========================================================================

static void map_IMU_To_Servos(float pitch, ServoCommandGroup *group)
{
    float shoulder = -pitch + 90.0f;
    if (shoulder < 30.0f) shoulder = 30.0f;
    if (shoulder > 150.0f) shoulder = 150.0f;

    float elbow = 180.0f - shoulder;
    if (elbow < 45.0f) elbow = 45.0f;
    if (elbow > 135.0f) elbow = 135.0f;

    float wrist = shoulder;
    if (wrist < 20.0f) wrist = 20.0f;
    if (wrist > 160.0f) wrist = 160.0f;

    group->count = 3;
    group->commands[0].channel = 1;
    group->commands[0].angle = shoulder;
    group->commands[1].channel = 2;
    group->commands[1].angle = elbow;
    group->commands[2].channel = 3;
    group->commands[2].angle = wrist;
}

// ==========================================================================
// 任务：IMU数据采集
// ==========================================================================

void IMU_Task(void *pvParameters)
{
    (void)pvParameters;

    mpu6050_data_t imu_raw;
    mpu6050_attitude_t attitude;
    ImuData imu_data;

    while (1)
    {
        // 获取I2C锁读取IMU数据
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            // 读取传感器数据
            esp_err_t err = MPU6050_ReadData(MPU6050_I2C_ADDR_DEFAULT, &imu_raw);
            if (err == ESP_OK)
            {
                // 传感器融合计算姿态
                MPU6050_FuseData(&imu_raw, &attitude);

                imu_data.pitch = attitude.pitch;
                imu_data.roll = 0.0f;
                imu_data.yaw = 0.0f;

                // 发送到队列
                xQueueOverwrite(xIMUQueue, &imu_data);
            }
            xSemaphoreGive(xI2CMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================================================
// 任务：控制逻辑
// ==========================================================================

void Control_Task(void *pvParameters)
{
    (void)pvParameters;

    ImuData imu_data;
    ServoCommandGroup group;

    float last_shoulder = 90.0f;
    float last_elbow = 90.0f;
    float last_wrist = 90.0f;
    const float threshold = 1.0f;
    const TickType_t min_interval = pdMS_TO_TICKS(100);
    TickType_t last_tick = xTaskGetTickCount();

    // 发送初始姿态（归位）
    group.count = 5;
    group.commands[0].channel = 0;
    group.commands[0].angle = 90.0f;
    group.commands[1].channel = 1;
    group.commands[1].angle = 90.0f;
    group.commands[2].channel = 2;
    group.commands[2].angle = 90.0f;
    group.commands[3].channel = 3;
    group.commands[3].angle = 90.0f;
    group.commands[4].channel = 4;
    group.commands[4].angle = 180.0f;
    xQueueSend(xControlQueue, &group, portMAX_DELAY);

    while (1)
    {
        // 从队列读取IMU数据
        if (xQueueReceive(xIMUQueue, &imu_data, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            // 将pitch映射为舵机角度
            map_IMU_To_Servos(imu_data.pitch, &group);

            // 防抖判断
            bool need_send = false;
            if (group.commands[0].angle > last_shoulder + threshold ||
                group.commands[0].angle < last_shoulder - threshold)
            {
                need_send = true;
            }
            if (group.commands[1].angle > last_elbow + threshold ||
                group.commands[1].angle < last_elbow - threshold)
            {
                need_send = true;
            }
            if (group.commands[2].angle > last_wrist + threshold ||
                group.commands[2].angle < last_wrist - threshold)
            {
                need_send = true;
            }
            if (xTaskGetTickCount() - last_tick >= min_interval)
            {
                need_send = true;
            }

            // 发送命令到舵机任务
            if (need_send)
            {
                xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
                last_shoulder = group.commands[0].angle;
                last_elbow = group.commands[1].angle;
                last_wrist = group.commands[2].angle;
                last_tick = xTaskGetTickCount();

                // 日志输出（每20次输出一次）
                static int cnt = 0;
                if (cnt++ % 20 == 0)
                {
                    printf("[Control] Pitch=%.1f | Shoulder=%.1f Elbow=%.1f Wrist=%.1f\n",
                           imu_data.pitch, last_shoulder, last_elbow, last_wrist);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==========================================================================
// 任务：舵机执行
// ==========================================================================

void Servo_Task(void *pvParameters)
{
    (void)pvParameters;

    ServoCommandGroup group;

    while (1)
    {
        // 从队列接收命令组
        if (xQueueReceive(xControlQueue, &group, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            // 获取I2C锁，批量执行舵机命令
            if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                for (uint8_t i = 0; i < group.count; i++)
                {
                    Servo_SetAngle(group.commands[i].channel, group.commands[i].angle);
                }
                xSemaphoreGive(xI2CMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==========================================================================
// 主程序入口
// ==========================================================================

void app_main(void)
{
    // 1. 创建同步原语
    xI2CMutex = xSemaphoreCreateMutex();
    if (xI2CMutex == NULL)
    {
        printf("Error: Failed to create I2C mutex!\n");
        return;
    }

    // 2. 创建队列
    xIMUQueue = xQueueCreate(1, sizeof(ImuData));
    if (xIMUQueue == NULL)
    {
        printf("Error: Failed to create IMU queue!\n");
        return;
    }

    xControlQueue = xQueueCreate(10, sizeof(ServoCommandGroup));
    if (xControlQueue == NULL)
    {
        printf("Error: Failed to create control queue!\n");
        return;
    }

    // 3. 初始化硬件
    printf("Initializing hardware...\n");

    esp_err_t err = I2C_Init();
    if (err != ESP_OK)
    {
        printf("Error: I2C_Init failed: %s\n", esp_err_to_name(err));
        return;
    }

    // 初始化MPU6050
    printf("Initializing MPU6050...\n");
    err = MPU6050_Init(MPU6050_I2C_ADDR_DEFAULT);
    if (err != ESP_OK)
    {
        printf("Error: MPU6050_Init failed: %s\n", esp_err_to_name(err));
        return;
    }

    // 初始化Servo
    printf("Initializing Servo...\n");
    err = Servo_Init();
    if (err != ESP_OK)
    {
        printf("Error: Servo_Init failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("Hardware initialization complete!\n");
    printf("System ready!\n");

    // 4. 创建任务
    xTaskCreate(IMU_Task, "IMU_Task", 4096, NULL, 5, NULL);
    xTaskCreate(Control_Task, "Control_Task", 4096, NULL, 5, NULL);
    xTaskCreate(Servo_Task, "Servo_Task", 4096, NULL, 5, NULL);

    vTaskDelete(NULL);
}
