/*
 * PCA9685 - 16 通道 12-bit I2C PWM/Servo 驱动板
 *
 * 配套：I2cDevice 基类（虾哥 main/boards/common/i2c_device.h）
 * 适用：ESP-IDF v5+ 新 I2C API
 */

#include "pca9685.h"
#include "freertos/FreeRTOS.h"
#include <esp_log.h>
#include <math.h>

#define TAG "Pca9685"
#define I2C_TIMEOUT_MS 100

Pca9685::Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr), addr_(addr) {
}

bool Pca9685::Init() {
    // 1. 唤醒（清除 SLEEP 位）
    Wakeup();

    // 1.5 使能 Auto-Increment（关键！否则 SetPwm 的 4 字节多寄存器写入只有第 1 个字节生效）
    WriteReg(REG_MODE1, ReadMode1() | MODE1_AI);

    // 2. 配置 MODE2：输出为 totem-pole（默认）
    WriteReg(0x01, 0x04);  // MODE2: INVRT=0, OCH=0, OUTDRV=1, OUTNE1=0, OUTNE0=1

    // 3. 设置 PWM 频率为 50Hz（舵机标准）
    SetPwmFreq(50.0f);

    // 4. 全部通道初始为 0
    for (int i = 0; i < 16; i++) {
        SetPwm(i, 0, 0);
    }

    // 诊断：直接用低层 API 探测 PCA9685（绕过基类 ReadReg，避免 ESP_ERROR_CHECK abort 整个系统）
    uint8_t reg = REG_MODE1;
    uint8_t mode1_val = 0;
    esp_err_t probe = i2c_master_transmit_receive(i2c_device_, &reg, 1, &mode1_val, 1, I2C_TIMEOUT_MS);
    if (probe != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 探测失败（地址 0x%02X）: %s — 请检查共地/V+ 5V/跳线地址",
                 addr_, esp_err_to_name(probe));
        return false;
    }
    ESP_LOGI(TAG, "PCA9685 探测成功（地址 0x%02X, MODE1=0x%02X）", addr_, mode1_val);

    ESP_LOGI(TAG, "PCA9685 初始化完成（地址 0x%02X, 50Hz）", addr_);
    return true;
}

uint8_t Pca9685::ReadMode1() {
    return ReadReg(REG_MODE1);
}

void Pca9685::Sleep() {
    uint8_t mode1 = ReadMode1();
    WriteReg(REG_MODE1, (mode1 & 0x7F) | MODE1_SLEEP);
}

void Pca9685::Wakeup() {
    uint8_t mode1 = ReadMode1();
    WriteReg(REG_MODE1, mode1 & ~MODE1_SLEEP);
    // 唤醒后需等待振荡器稳定（最多 500us）
    vTaskDelay(pdMS_TO_TICKS(1));
}

void Pca9685::SetPwmFreq(float freq_hz) {
    // PCA9685 频率范围 24Hz ~ 1526Hz
    if (freq_hz < 24.0f) freq_hz = 24.0f;
    if (freq_hz > 1526.0f) freq_hz = 1526.0f;

    // prescale = round(25MHz / (4096 × freq)) - 1
    float prescale_val = 25000000.0f / (4096.0f * freq_hz) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_val + 0.5f);

    // 改 prescale 必须先 SLEEP
    uint8_t old_mode1 = ReadMode1();
    Sleep();
    WriteReg(REG_PRESCALE, prescale);
    WriteReg(REG_MODE1, old_mode1);
    vTaskDelay(pdMS_TO_TICKS(1));

    // restart（清除 SLEEP 后需要 restart 恢复 PWM 输出）
    WriteReg(REG_MODE1, old_mode1 | MODE1_RESTART);
}

void Pca9685::SetPwm(uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) return;

    uint8_t reg_base = REG_LED0_ON_L + (channel * 4);
    uint8_t buffer[5] = {
        reg_base,
        (uint8_t)(on & 0xFF),
        (uint8_t)((on >> 8) & 0x0F),
        (uint8_t)(off & 0xFF),
        (uint8_t)((off >> 8) & 0x0F),
    };
    esp_err_t ret = i2c_master_transmit(i2c_device_, buffer, 5, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SetPwm 失败: CH=%d reg=0x%02X on=%u off=%u err=%s",
                 channel, reg_base, on, off, esp_err_to_name(ret));
    }
}

void Pca9685::SetServoAngle(uint8_t channel, float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // SG90: 0.5ms~2.5ms 对应 0°~180°
    // 12-bit: 4096 = 20ms → 1ms = 204.8 count
    // 0.5ms ≈ 102, 2.5ms ≈ 512
    uint16_t off = 102 + (uint16_t)((angle * 410.0f) / 180.0f);
    SetPwm(channel, 0, off);
}
