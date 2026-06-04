#ifndef __SERVO_CONTROLLER_H__
#define __SERVO_CONTROLLER_H__

#include "pca9685.h"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <string>
#include <vector>
#include <map>

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
 * 设计哲学（v2 - 情感驱动）：
 *   LLM 不直接控制舵机角度，而是声明"情感"，由内部 emotion 映射表
 *   自动播放预制动作序列。每个动作序列自带步进式插值（10ms/步，1°/步），
 *   实现丝滑过渡，告别硬跳变。
 *
 * 提供 3 个 MCP 工具（LLM 可主动调用）：
 *   self.servo.play_emotion  - 播预制情感动作（10 种 emotion）
 *   self.servo.set_look      - 设置视线方向（X/Y 角度 + 步进速度）
 *   self.servo.center        - 立即回中
 *   self.servo.get_status    - 查状态（moving/idle）
 *
 * 队列策略（v2.1 改进）：
 *   - 队列满时丢弃最旧 → 插入最新（保持最新情感优先，避免 MCP 阻塞）
 *   - 每个 emotion 独立指定 step_ms（不受 set_look 干扰）
 *
 * 参考实现：ideamark/desk-emoji v2.0.1（head_move 步进式插值）
 * 但改造成"情感驱动 + 队列异步 + 失败可感知"，不阻塞 LLM 工具调用。
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

// 步进插值默认参数（desk-emoji 同款）
#define SERVO_STEP_DEG        1         // 每步 1°
#define SERVO_STEP_MS         10        // 每步 10ms

/*
 * Emotion 动作片段定义（顶层 struct，方便 std::map 使用）
 * 每个 emotion 由多个 step 组成，每个 step 是 (x, y, hold_ms, step_ms)
 *   - x / y: 该 step 目标角度（= current 表示不动）
 *   - hold_ms: 在该位置停留时间
 *   - step_ms: 本步进的步进间隔（= 0 用 SERVO_STEP_MS 默认）
 * 步进插值由 MoveSmooth() 自动处理
 */
struct EmotionStep {
    int x;          // 目标 X 角度
    int y;          // 目标 Y 角度
    int hold_ms;    // 到达目标后停留时间
    int step_ms;    // 本步进速度（0 = 用默认 10ms）
};

class ServoController {
public:
    ServoController();
    ~ServoController();

private:
    Pca9685* pca9685_ = nullptr;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_ = nullptr;
    bool is_action_running_ = false;  // 当前是否有 emotion/move 在执行

    // 当前实际角度（步进插值用，初始为中心）
    volatile int current_x_ = SERVO_X_CENTER;
    volatile int current_y_ = SERVO_Y_CENTER;

    // v2.2 抗循环：play_emotion 节奏控制
    // 背景：LLM 收到 "立刻调+调用后再说话" 这种 prompt 后会陷进
    //       "调 play_emotion → 说话 → 再调 → 再说" 的循环，导致
    //       语音重复播报 + 舵机不停跳。设备端加最小间隔 + 同名额外冷却
    //       来强制 LLM 慢下来。
    static constexpr int64_t kPlayEmotionMinIntervalMs = 2000;     // 任意 emotion 最小间隔
    static constexpr int64_t kPlayEmotionSameCooldownMs = 5000;    // 同一 emotion 额外冷却
    int64_t last_play_emotion_ms_ = 0;       // 上一次 play_emotion 时间戳（ms）
    std::string last_play_emotion_name_;     // 上一次 play_emotion 名字

    enum ActionType {
        ACTION_PLAY_EMOTION = 1,   // str_param = emotion 名字
        ACTION_SET_LOOK = 2,       // param1 = x, param2 = y, param3 = step_ms（每次独立）
        ACTION_CENTER = 3,         // 立即回中
        ACTION_MOVE_SMOOTH_X = 4,  // param1 = 目标 X
        ACTION_MOVE_SMOOTH_Y = 5,  // param1 = 目标 Y
    };

    struct ServoActionParams {
        int action_type;
        int param1;
        int param2;
        int param3;            // 用于 set_look 的 step_ms
        char str_param[32];    // emotion 名字（避免 const char* 指针悬挂）
    };

    static void ActionTask(void* arg);

    // QueueAction 现在不阻塞：队列满时丢弃最旧，插入最新
    // 返回 true 表示成功入队，false 表示 emotion 名字错（仅 play_emotion 用）
    bool QueueAction(int type, int p1 = 0, int p2 = 0, int p3 = 0, const char* str = nullptr);

    void StartActionTaskIfNeeded();

    // 步进式平滑移动（desk-emoji head_move 移植）
    void MoveSmooth(uint8_t ch, int target_angle, int step_ms = SERVO_STEP_MS);

    // 角度限位辅助
    int ConstrainAngle(uint8_t ch, int angle);

    // Emotion 库查询（返回是否找到，strdup/parse 失败也返回 false）
    bool PlayEmotion(const char* name);

    // 屏幕表情联动 hook（默认空实现，主人可重写）
    void OnEmotionTriggered(const char* name);

    void RegisterMcpTools();
};

void InitializeServoController();

#endif // __SERVO_CONTROLLER_H__
