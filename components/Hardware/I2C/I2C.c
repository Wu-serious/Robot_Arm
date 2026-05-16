#include "I2C.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "I2C";

/**
 * @brief 初始化 I2C 主机驱动。
 *
 * 配置 I2C 主机模式、SDA/SCL 引脚和时钟频率，并安装驱动。
 *
 * @return 成功返回 ESP_OK，失败返回对应 esp_err_t 错误码。
 */
esp_err_t I2C_Init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    //参数配置
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }
    //安装驱动  
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
    }

    return err;
}

/**
 * @brief 反初始化 I2C 驱动。
 *
 * 删除驱动并释放 I2C 控制器资源。
 */
esp_err_t I2C_Deinit(void)
{
    return i2c_driver_delete(I2C_MASTER_NUM);
}

/**
 * @brief 发送 I2C 起始信号并发送从机地址。
 *
 * @param cmd I2C 命令链句柄。
 * @param dev_addr 7 位从机地址。
 * @param read true 表示读操作，false 表示写操作。
 * @return ESP_OK 表示成功。
 */
static esp_err_t I2C_MasterStart(i2c_cmd_handle_t cmd, uint8_t dev_addr, bool read)
{
    esp_err_t err = i2c_master_start(cmd);//发送起始信号
    if (err != ESP_OK) {
        return err;
    }

    uint8_t address = (dev_addr << 1) | (read ? I2C_MASTER_READ : I2C_MASTER_WRITE);
    //发送7位从机地址+读/写位
    return i2c_master_write_byte(cmd, address, true);
}

/**
 * @brief 向从机寄存器写入数据。
 *
 * 先发送寄存器地址，再发送数据负载。
 */
esp_err_t I2C_MasterWriteToReg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, size_t data_len)
{
    if (data == NULL || data_len == 0) {                               // 参数合法性检查
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                       // 创建 I2C 命令链
    esp_err_t err = I2C_MasterStart(cmd, dev_addr, false);              // 发送 START + 从机地址(写)
    if (err != ESP_OK) {                                                // 如果发送失败
        i2c_cmd_link_delete(cmd);                                       // 释放命令链内存
        return err;
    }

    err = i2c_master_write_byte(cmd, reg, true);                        // 发送寄存器地址
    if (err == ESP_OK) {                                                // 如果发送成功
        err = i2c_master_write(cmd, data, data_len, true);             // 发送数据字节
    }
    if (err == ESP_OK) {                                                // 如果发送成功
        err = i2c_master_stop(cmd);                                     // 发送 STOP 信号
    }
    if (err == ESP_OK) {                                                // 如果之前都成功
        err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));  // 执行命令链
    }

    i2c_cmd_link_delete(cmd);                                           // 释放命令链内存
    return err;
}

/**
 * @brief 从从机寄存器读取数据。
 *
 * 先写入寄存器地址，然后发送重复启动并读取数据。
 */
esp_err_t I2C_MasterReadFromReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t data_len)
{
    if (data == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = I2C_MasterStart(cmd, dev_addr, false);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return err;
    }

    err = i2c_master_write_byte(cmd, reg, true);
    if (err == ESP_OK) {
        err = i2c_master_start(cmd);
    }
    if (err == ESP_OK) {
        err = i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    }
    if (err == ESP_OK) {
        err = i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
    }
    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    }

    i2c_cmd_link_delete(cmd);
    return err;
}