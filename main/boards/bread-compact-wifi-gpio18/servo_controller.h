#ifndef __SERVO_CONTROLLER_H__
#define __SERVO_CONTROLLER_H__

#include "pca9685.h"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

/*
 * ServoController - 基于 PCA9685 的 2 自由度云台 MCP 控制器
 *
 * 适配板型：bread-compact-wifi-gpio18（ESP32-S3-WROOM-1-N16R8）
 *
 * 接线：
 *   PCA9685 SDA -> ESP32 GPIO9  (I2C1_SDA)
 *   PCA9685 SCL -> ESP32 GPIO10 (I2C1_SCL)
 *   PCA9685 V+  -> 5V/3A 独立电源（18650 串联 + 降压板）
 *   PCA9685 GND -> 5V 电源 GND + ESP32 GND（共地）
 *   舵机 X 信号 -> PCA9685 CH0
 *   舵机 Y 信号 -> PCA9685 CH1
 *   舵机 VCC/GND -> PCA9685 V+/GND
 *
 * 提供 6 个 MCP 工具（LLM 可主动调用）：
 *   self.servo.set_x        - 设定 X 角度（0-180）
 *   self.servo.set_y        - 设定 Y 角度（0-180）
 *   self.servo.look_around  - 摇头（左右看看）
 *   self.servo.nod          - 点头
 *   self.servo.center       - 回中
 *   self.servo.get_status   - 查状态（moving/idle）
 */

#define SERVO_I2C_SDA_GPIO    GPIO_NUM_9
#define SERVO_I2C_SCL_GPIO    GPIO_NUM_10
#define SERVO_I2C_FREQ_HZ     400000    // 400kHz fast mode
#define SERVO_PCA9685_ADDR    0x70
#define SERVO_PCA9685_CH_X    0
#define SERVO_PCA9685_CH_Y    1

// 角度中心点与限位
#define SERVO_X_CENTER        90
#define SERVO_X_MIN           30
#define SERVO_X_MAX           150
#define SERVO_Y_CENTER        90
#define SERVO_Y_MIN           60
#define SERVO_Y_MAX           120

class ServoController {
public:
    ServoController();
    ~ServoController();

private:
    Pca9685* pca9685_ = nullptr;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_ = nullptr;
    bool is_action_in_progress_ = false;

    enum ActionType {
        ACTION_SET_X = 1,        // param1 = 目标角度
        ACTION_SET_Y = 2,        // param1 = 目标角度
        ACTION_LOOK_AROUND = 3,  // param1 = 速度 ms/帧（默认 300）
        ACTION_NOD = 4,          // param1 = 速度 ms/帧（默认 300）
        ACTION_CENTER = 5,       // 立即回中
    };

    struct ServoActionParams {
        int action_type;
        int param1;
        int param2;
    };

    static void ActionTask(void* arg);
    void QueueAction(int type, int p1 = 0, int p2 = 0);
    void StartActionTaskIfNeeded();
    void SetServoAngle(uint8_t channel, int angle);
    void RegisterMcpTools();
};

void InitializeServoController();

#endif // __SERVO_CONTROLLER_H__
