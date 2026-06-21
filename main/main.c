/**
 * @file main.c
 * @brief 四自由度机械臂 MPU6050 体感控制主程序
 *
 * 系统架构：
 * - ESP32-P4 主控 (Host)
 * - ESP32-C5 协处理器 (Slave) - 通过SDIO接口连接，提供WiFi/BLE功能
 * - MPU6050 六轴传感器读取姿态
 * - PCA9685 驱动 5 路舵机
 *
 * 任务结构：
 * 1. IMU_Task     - 100Hz IMU数据采集
 * 2. Control_Task - 50Hz 姿态解算与命令生成
 * 3. Servo_Task   - 50Hz 舵机指令执行
 * 4. WiFi_Task    - WiFi连接与状态管理
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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

#define WIFI_SSID      "ServoArm_AP"
#define WIFI_PASSWORD  "12345678"

// ==========================================================================
// 数据结构
// ==========================================================================

typedef struct {
    float pitch;
    float roll;
} ImuData;

// ==========================================================================
// 全局变量
// ==========================================================================

static SemaphoreHandle_t xI2CMutex = NULL;
static QueueHandle_t xIMUQueue = NULL;
static QueueHandle_t xControlQueue = NULL;

static int  g_imu_mode = 0;   /* 0=manual, 1=MPU6050, 2=Phone IMU */
static float g_phone_pitch = 0.0f;
static float g_phone_roll = 0.0f;
static float current_base = 90.0f;
static float current_shoulder = 90.0f;
static float current_elbow = 90.0f;
static float current_wrist = 90.0f;
static float current_gripper = 180.0f;
static float current_pitch = 0.0f;
static float current_roll = 0.0f;

// ==========================================================================
// 角度映射模块
// ==========================================================================

static void map_IMU_To_Servos(float pitch, float roll, ServoCommandGroup *group)
{
    // 底座角度：roll<0 → <90°，roll>0 → >90°
    float base = 90.0f + roll;
    if (base < 0.0f) base = 0.0f;
    if (base > 180.0f) base = 180.0f;

    // 根据pitch按比例分配角度变化
    // shoulder变化最小，elbow中等，wrist最大
    // 再次增加比例，提高响应灵敏度
    const float shoulder_ratio = 0.5f;   // 肩关节响应比例
    const float elbow_ratio = 0.8f;      // 肘关节响应比例
    const float wrist_ratio = 1.2f;      // 腕关节响应比例（最大）
    // 总变化量 = pitch × (0.5 + 0.8 + 1.2) = pitch × 2.5

    // 肩关节：90°为中心，变化方向与pitch相反
    float shoulder = 90.0f - pitch * shoulder_ratio;
    if (shoulder < 60.0f) shoulder = 60.0f;
    if (shoulder > 120.0f) shoulder = 120.0f;

    // 肘关节：90°为中心，变化方向与pitch相同
    float elbow = 90.0f + pitch * elbow_ratio;
    if (elbow < 40.0f) elbow = 40.0f;
    if (elbow > 140.0f) elbow = 140.0f;

    // 腕关节：90°为中心，变化方向与pitch相反
    float wrist = 90.0f - pitch * wrist_ratio;
    if (wrist < 20.0f) wrist = 20.0f;
    if (wrist > 160.0f) wrist = 160.0f;

    group->count = 4;
    group->commands[0].channel = 0;
    group->commands[0].angle = base;
    group->commands[1].channel = 1;
    group->commands[1].angle = shoulder;
    group->commands[2].channel = 2;
    group->commands[2].angle = elbow;
    group->commands[3].channel = 3;
    group->commands[3].angle = wrist;
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

    // IMU数据平滑滤波
    static float filtered_pitch = 0.0f;
    static float filtered_roll = 0.0f;
    const float imu_filter_alpha = 0.3f;  // 滤波系数

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

                // 一阶低通滤波
                filtered_pitch = imu_filter_alpha * attitude.pitch + (1 - imu_filter_alpha) * filtered_pitch;
                filtered_roll = imu_filter_alpha * attitude.roll + (1 - imu_filter_alpha) * filtered_roll;

                imu_data.pitch = filtered_pitch;
                imu_data.roll = filtered_roll;

                // 更新全局变量供Web显示
                current_pitch = filtered_pitch;
                current_roll = filtered_roll;

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

    // 舵机平滑滤波
    static float filtered_base = 90.0f;
    static float filtered_shoulder = 90.0f;
    static float filtered_elbow = 90.0f;
    static float filtered_wrist = 90.0f;
    const float servo_filter_alpha = 0.5f;  // 舵机滤波系数

    float last_base = 90.0f;
    float last_shoulder = 90.0f;
    float last_elbow = 90.0f;
    float last_wrist = 90.0f;
    const float threshold = 1.5f;  // 适当增大阈值
    const TickType_t min_interval = pdMS_TO_TICKS(150);  // 延长最小间隔
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
            // IMU mode: 1=MPU6050, 2=Phone
            if (g_imu_mode == 1 || g_imu_mode == 2)
            {
                float p = (g_imu_mode == 2) ? g_phone_pitch : imu_data.pitch;
                float r = (g_imu_mode == 2) ? g_phone_roll  : imu_data.roll;
                map_IMU_To_Servos(p, r, &group);

                // 舵机角度平滑滤波
                filtered_base = servo_filter_alpha * group.commands[0].angle + (1 - servo_filter_alpha) * filtered_base;
                filtered_shoulder = servo_filter_alpha * group.commands[1].angle + (1 - servo_filter_alpha) * filtered_shoulder;
                filtered_elbow = servo_filter_alpha * group.commands[2].angle + (1 - servo_filter_alpha) * filtered_elbow;
                filtered_wrist = servo_filter_alpha * group.commands[3].angle + (1 - servo_filter_alpha) * filtered_wrist;

                // 更新为滤波后的值
                group.commands[0].angle = filtered_base;
                group.commands[1].angle = filtered_shoulder;
                group.commands[2].angle = filtered_elbow;
                group.commands[3].angle = filtered_wrist;

                // 防抖判断
                bool need_send = false;
                if (group.commands[0].angle > last_base + threshold ||
                    group.commands[0].angle < last_base - threshold)
                {
                    need_send = true;
                }
                if (group.commands[1].angle > last_shoulder + threshold ||
                    group.commands[1].angle < last_shoulder - threshold)
                {
                    need_send = true;
                }
                if (group.commands[2].angle > last_elbow + threshold ||
                    group.commands[2].angle < last_elbow - threshold)
                {
                    need_send = true;
                }
                if (group.commands[3].angle > last_wrist + threshold ||
                    group.commands[3].angle < last_wrist - threshold)
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
                    last_base = group.commands[0].angle;
                    last_shoulder = group.commands[1].angle;
                    last_elbow = group.commands[2].angle;
                    last_wrist = group.commands[3].angle;
                    last_tick = xTaskGetTickCount();

                    // 日志输出（每200次输出一次，约4秒一次）
                    static int cnt = 0;
                    if (cnt++ % 200 == 0)
                    {
                        printf("[Control] Pitch=%.1f Roll=%.1f | Base=%.1f Shoulder=%.1f Elbow=%.1f Wrist=%.1f\n",
                               imu_data.pitch, imu_data.roll,
                               last_base, last_shoulder, last_elbow, last_wrist);
                    }
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
                    
                    // 更新当前角度
                    switch (group.commands[i].channel) {
                        case 0: current_base = group.commands[i].angle; break;
                        case 1: current_shoulder = group.commands[i].angle; break;
                        case 2: current_elbow = group.commands[i].angle; break;
                        case 3: current_wrist = group.commands[i].angle; break;
                        case 4: current_gripper = group.commands[i].angle; break;
                    }
                }
                xSemaphoreGive(xI2CMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==========================================================================
// Web命令回调函数
// ==========================================================================

static void web_command_handler(const char *command, float value)
{
    ServoCommandGroup group;

    ESP_LOGI("Web", "Received command: %s, value: %.2f", command, value);

    if (strcmp(command, "reset") == 0)
    {
        /* Home all servos: base/shoulder/elbow/wrist=90, gripper=180 */
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
        g_imu_mode = 0;
        xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
    }
    else if (strcmp(command, "wave") == 0)
    {
        /* Simple wave sequence using shoulder and wrist */
        g_imu_mode = 0;
        for (int i = 0; i < 3; i++)
        {
            group.count = 2;
            group.commands[0].channel = 1;
            group.commands[0].angle = 60.0f;
            group.commands[1].channel = 3;
            group.commands[1].angle = 50.0f;
            xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
            vTaskDelay(pdMS_TO_TICKS(400));

            group.commands[0].angle = 120.0f;
            group.commands[1].angle = 130.0f;
            xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
            vTaskDelay(pdMS_TO_TICKS(400));
        }
        /* Return to home */
        group.count = 2;
        group.commands[0].channel = 1;
        group.commands[0].angle = 90.0f;
        group.commands[1].channel = 3;
        group.commands[1].angle = 90.0f;
        xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
    }
    else if (strcmp(command, "mode") == 0)
    {
        g_imu_mode = (int)value;
        if (g_imu_mode < 0) g_imu_mode = 0;
        if (g_imu_mode > 2) g_imu_mode = 2;
        const char *names[] = {"Manual", "MPU6050", "Phone IMU"};
        printf("Mode: %s\n", names[g_imu_mode]);
    }
    else if (strcmp(command, "phone_pitch") == 0)
    {
        g_phone_pitch = value;
    }
    else if (strcmp(command, "phone_roll") == 0)
    {
        g_phone_roll = value;
    }
    else if (strcmp(command, "servo0") == 0)
    {
        group.count = 1;
        group.commands[0].channel = 0;
        group.commands[0].angle = value;
        xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
    }
    else if (strcmp(command, "servo1") == 0)
    {
        group.count = 1;
        group.commands[0].channel = 1;
        group.commands[0].angle = value;
        xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
    }
    else if (strcmp(command, "servo2") == 0)
    {
        group.count = 1;
        group.commands[0].channel = 2;
        group.commands[0].angle = value;
        xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
    }
    else if (strcmp(command, "servo3") == 0)
    {
        group.count = 1;
        group.commands[0].channel = 3;
        group.commands[0].angle = value;
        xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
    }
    else if (strcmp(command, "servo4") == 0)
    {
        group.count = 1;
        group.commands[0].channel = 4;
        group.commands[0].angle = value;
        xQueueSend(xControlQueue, &group, pdMS_TO_TICKS(10));
    }
}

// ==========================================================================
// 任务：Web状态发布
// ==========================================================================

void WebStatus_Task(void *pvParameters)
{
    (void)pvParameters;

    servo_status_t status;
    wifi_status_t wifi_status;

    while (1)
    {
        status.pitch = current_pitch;
        status.roll = current_roll;
        status.base_angle = current_base;
        status.shoulder_angle = current_shoulder;
        status.elbow_angle = current_elbow;
        status.wrist_angle = current_wrist;
        status.mode = g_imu_mode;

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

    // 2. 创建同步原语
    xI2CMutex = xSemaphoreCreateMutex();
    if (xI2CMutex == NULL)
    {
        printf("Error: Failed to create I2C mutex!\n");
        return;
    }

    // 3. 创建队列
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

    // 4. 初始化硬件
    printf("Initializing hardware...\n");

    ret = I2C_Init();
    if (ret != ESP_OK)
    {
        printf("Error: I2C_Init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // 初始化MPU6050
    printf("Initializing MPU6050...\n");
    ret = MPU6050_Init(MPU6050_I2C_ADDR_DEFAULT);
    if (ret != ESP_OK)
    {
        printf("Error: MPU6050_Init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // 初始化Servo
    printf("Initializing Servo...\n");
    ret = Servo_Init();
    if (ret != ESP_OK)
    {
        printf("Error: Servo_Init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    printf("Hardware initialization complete!\n");

    // 5. 初始化Web服务器
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

    printf("System ready!\n");

    // 6. 创建任务
    xTaskCreate(IMU_Task, "IMU_Task", 4096, NULL, 5, NULL);
    xTaskCreate(Control_Task, "Control_Task", 4096, NULL, 5, NULL);
    xTaskCreate(Servo_Task, "Servo_Task", 4096, NULL, 5, NULL);
    xTaskCreate(WebStatus_Task, "WebStatus_Task", 4096, NULL, 4, NULL);

    vTaskDelete(NULL);
}
