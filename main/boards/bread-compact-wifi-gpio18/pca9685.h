#ifndef __PCA9685_H__
#define __PCA9685_H__

#include "i2c_device.h"
#include <driver/i2c_master.h>

/*
 * PCA9685 - 16 通道 12-bit I2C PWM/Servo 驱动板
 *
 * 默认 I2C 地址：0x70（A0~A5 全部拉低）
 * 可改地址：0x70~0x7F（通过 A0~A5 6 个地址引脚）
 *
 * 频率计算：prescale = round(25MHz / (4096 * freq)) - 1
 *   50Hz: prescale = 121
 *   60Hz: prescale = 101
 *   100Hz: prescale = 60
 *
 * 占空比：12-bit (0~4095)
 *   SG90 0°:   102 (≈ 0.5ms / 20ms × 4096)
 *   SG90 90°:  307 (≈ 1.5ms)
 *   SG90 180°: 511 (≈ 2.5ms)
 */
class Pca9685 : public I2cDevice {
public:
    Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x70);
    bool Init();
    void SetPwmFreq(float freq_hz);
    void SetPwm(uint8_t channel, uint16_t on, uint16_t off);
    void SetServoAngle(uint8_t channel, float angle);
    void Sleep();
    void Wakeup();

private:
    uint8_t ReadMode1();
    uint8_t addr_;  // 保存 I2C 地址供诊断日志使用

    enum {
        REG_MODE1 = 0x00,
        REG_MODE2 = 0x01,
        REG_LED0_ON_L = 0x06,
        REG_ALL_LED_ON_L = 0xFA,
        REG_ALL_LED_OFF_L = 0xFC,
        REG_PRESCALE = 0xFE,
    };

    enum {
        MODE1_RESTART = 0x80,
        MODE1_EXTCLK = 0x40,
        MODE1_AI = 0x20,        // Auto-Increment
        MODE1_SLEEP = 0x10,
    };
};

#endif // __PCA9685_H__
