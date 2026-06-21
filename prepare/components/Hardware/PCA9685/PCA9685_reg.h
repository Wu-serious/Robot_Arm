#ifndef __PCA9685_REG_H__
#define __PCA9685_REG_H__


#define PCA9685_I2C_ADDR_DEFAULT   0x40    // PCA9685 默认I2C设备地址

#define PCA9685_MODE1              0x00    // Mode 寄存器1 - 控制模式
#define PCA9685_MODE2              0x01    // Mode 寄存器2 - 输出模式
#define PCA9685_SUBADR1            0x02    // 子地址1
#define PCA9685_SUBADR2            0x03    // 子地址2
#define PCA9685_SUBADR3            0x04    // 子地址3
#define PCA9685_ALL_CALL_ADDR      0x05    // 全调用地址
#define PCA9685_LED0_ON_L          0x06    // LED0 ON 低字节
#define PCA9685_LED0_ON_H          0x07    // LED0 ON 高字节
#define PCA9685_LED0_OFF_L         0x08    // LED0 OFF 低字节
#define PCA9685_LED0_OFF_H         0x09    // LED0 OFF 高字节
#define PCA9685_ALL_LED_ON_L       0xFA    // 所有LED ON 低字节
#define PCA9685_ALL_LED_ON_H       0xFB    // 所有LED ON 高字节
#define PCA9685_ALL_LED_OFF_L      0xFC    // 所有LED OFF 低字节
#define PCA9685_ALL_LED_OFF_H      0xFD    // 所有LED OFF 高字节
#define PCA9685_PRE_SCALE          0xFE    // 预分频器 - 设置PWM频率

// MODE1 寄存器位定义
#define PCA9685_MODE1_RESTART      0x80    // Bit7: 重启位
#define PCA9685_MODE1_SLEEP        0x10    // Bit4: 睡眠模式
#define PCA9685_MODE1_AI           0x20    // Bit5: 自动递增使能

// MODE2 寄存器位定义
#define PCA9685_MODE2_OUTDRV       0x04    // Bit2: 推挽输出模式

#endif 
