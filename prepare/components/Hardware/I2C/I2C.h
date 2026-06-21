#ifndef __I2C_H__
#define __I2C_H__

#include <esp_err.h>
#include <driver/i2c.h>


// I2C 硬件配置
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           51      // SDA 引脚
#define I2C_MASTER_SCL_IO           52      // SCL 引脚
#define I2C_MASTER_FREQ_HZ          100000  // I2C 时钟频率 100kHz
#define I2C_MASTER_TX_BUF_DISABLE   0       // 主机模式不使用 TX 缓冲区
#define I2C_MASTER_RX_BUF_DISABLE   0       // 主机模式不使用 RX 缓冲区
#define I2C_MASTER_TIMEOUT_MS       1000    // 命令超时，单位毫秒

/**
 * @brief 初始化 I2C 主机控制器。
 *
 * 配置 SDA/SCL 引脚并安装 I2C 驱动。
 * SDA 使用 GPIO51，SCL 使用 GPIO52。
 *
 * @return 成功返回 ESP_OK，否则返回 esp_err_t 错误码。
 */
esp_err_t I2C_Init(void);

/**
 * @brief 向 I2C 设备写入原始数据。
 *
 * @param dev_addr 7 位从机地址。
 * @param data 发送数据指针。
 * @param data_len 发送字节数。
 * @return 成功返回 ESP_OK。
 */
esp_err_t I2C_MasterWriteToReg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, size_t data_len);

/**
 * @brief 从 I2C 设备寄存器读取数据。
 *
 * 先写入寄存器地址，然后发送重复启动并读取数据。
 */
esp_err_t I2C_MasterReadFromReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t data_len);


#endif
