#include "Script.h"
#include "Servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void PickAndPlace(uint16_t step_delay_ms)
{
    const uint16_t small_delay_ms = 500;  // 小步操作延时
    Servo_Init();  // 确保舵机已初始化

    static const char *TAG = "ROBOT_ARM";
    
    ESP_LOGI(TAG, "步骤 1：准备目标上方姿态");
    Servo_SetAngle(0, 0.0f);    // 底座旋转到目标方向
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(1, 90.0f);   // 肩关节提高到准备高度
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(2, 90.0f);   // 肘关节伸直
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(3, 90.0f);   // 手腕保持水平
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(4, 90.0f);   // 夹爪关闭
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

    ESP_LOGI(TAG, "步骤 2：下降到物体位置");
    Servo_SetAngle(4, 0.0f);    // 夹爪打开 
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(1, 65.0f);   // 肩关节下降
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(2, 140.0f);  // 肘关节调整至抓取角度
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(3, 20.0f);   // 手腕调整至抓取姿态    
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

    ESP_LOGI(TAG, "步骤 3：闭合夹爪抓取");  
    Servo_SetAngle(4, 90.0f);   // 夹爪闭合
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));

    ESP_LOGI(TAG, "步骤 4：抬起物体");
    Servo_SetAngle(1, 90.0f);   // 肩关节抬起   
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

    ESP_LOGI(TAG, "步骤 5：移动到放置位置");
    Servo_SetAngle(0, 90.0f);   // 底座转到放置方向
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

    ESP_LOGI(TAG, "步骤 6：放下物体");
   
    Servo_SetAngle(1, 65.0f);   // 肩关节下降
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    Servo_SetAngle(4, 0.0f);    // 夹爪打开
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));

    ESP_LOGI(TAG, "步骤 7：回到初始姿态");
    Servo_SetAngle(0, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(1, 90.0f);
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(2, 90.0f);
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(3, 90.0f);
    vTaskDelay(pdMS_TO_TICKS(small_delay_ms));
    Servo_SetAngle(4, 90.0f);
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

    ESP_LOGI(TAG, "抓取动作完成\n");
}