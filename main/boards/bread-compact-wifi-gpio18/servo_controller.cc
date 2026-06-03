/*
 * ServoController - 基于 PCA9685 的 2 自由度云台 MCP 控制器
 *
 * 配套驱动：pca9685.cc（PCA9685 I2C PWM 驱动）
 * 配套基类：main/boards/common/i2c_device.{h,cc}（虾哥 I2cDevice）
 *
 * 架构优势：
 *   1. ESP32 与舵机电源完全隔离（USB 5V + 18650 5V 分别供电）
 *   2. PCA9685 硬件 PWM 精度 12-bit（4096 级）
 *   3. I2C 总线独立（I2C1，不跟 OLED 抢 I2C0）
 *   4. 16 通道可扩展（未来加 4/6 自由度机器人）
 */

#include "servo_controller.h"
#include "mcp_server.h"
#include "freertos/FreeRTOS.h"
#include <esp_log.h>

#define TAG "ServoController"

// === CH2 诊断测试任务（启动后 3 分钟内不停摆动）===
// 用途：插上舵机到 CH2 看是否动，用于对比测试 CH0/CH1 是不是硬件问题
// 3 分钟后自动停止（想再测就按 RST 键重启 ESP32）
// CH2 与 CH0/CH1 通道独立，I2C 总线驱动内部互斥，不会冲突
static void Ch2DiagnosticTest(void* arg) {
    Pca9685* pca = static_cast<Pca9685*>(arg);
    const int target_ch = 2;          // CH2 专用
    const int half_period_ms = 800;   // 0.8 秒一切
    const int total_seconds = 180;    // 3 分钟
    const int total_cycles = (total_seconds * 1000) / (half_period_ms * 2);

    ESP_LOGI(TAG, "CH2 诊断测试启动（持续 %d 秒，0° ↔ 180° 摆动）", total_seconds);
    for (int i = 0; i < total_cycles; i++) {
        // 0° (off_count=102 ≈ 0.5ms)
        pca->SetPwm(target_ch, 0, 102);
        vTaskDelay(pdMS_TO_TICKS(half_period_ms));
        // 180° (off_count=512 ≈ 2.5ms)
        pca->SetPwm(target_ch, 0, 512);
        vTaskDelay(pdMS_TO_TICKS(half_period_ms));
    }
    // 3 分钟结束，回到中位 1.5ms (off_count=307)
    pca->SetPwm(target_ch, 0, 307);
    ESP_LOGI(TAG, "CH2 诊断测试结束（CH2 停止摆动）");
    vTaskDelete(NULL);
}

ServoController::ServoController() {
    // 1. 创建独立的 I2C1 总线（不跟 OLED 抢 I2C0）
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = (i2c_port_t)I2C_NUM_1,
        .sda_io_num = SERVO_I2C_SDA_GPIO,
        .scl_io_num = SERVO_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,  // I2C 上拉（PCA9685 板通常无上拉）
        },
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &bus_handle));

    // 2. 创建 PCA9685 驱动
    pca9685_ = new Pca9685(bus_handle, SERVO_PCA9685_ADDR);
    pca9685_->Init();

    // 3. 启动后回中
    SetServoAngle(SERVO_PCA9685_CH_X, SERVO_X_CENTER);
    SetServoAngle(SERVO_PCA9685_CH_Y, SERVO_Y_CENTER);

    // 4. 创建动作队列
    action_queue_ = xQueueCreate(10, sizeof(ServoActionParams));

    // 5. 注册 MCP 工具
    RegisterMcpTools();
    ESP_LOGI(TAG, "Servo 控制器已初始化（PCA9685 @ 0x%02X, X=CH%d, Y=CH%d）",
             SERVO_PCA9685_ADDR, SERVO_PCA9685_CH_X, SERVO_PCA9685_CH_Y);
    ESP_LOGI(TAG, "Servo 启动完成（PCA9685 通信层已验证）");

    // 6. 启动 CH2 诊断测试任务（3 分钟内不停摆动，用于对比测试 CH0/CH1 是否硬件故障）
    if (pca9685_ != nullptr) {
        xTaskCreate(Ch2DiagnosticTest, "servo_ch2_test", 2048, pca9685_, 1, nullptr);
        ESP_LOGI(TAG, "CH2 诊断测试已启动（CH2 上插舵机会自动摆动 3 分钟）");
    }
}

ServoController::~ServoController() {
    if (action_task_handle_ != nullptr) {
        vTaskDelete(action_task_handle_);
        action_task_handle_ = nullptr;
    }
    if (action_queue_ != nullptr) {
        vQueueDelete(action_queue_);
    }
    if (pca9685_ != nullptr) {
        pca9685_->Sleep();  // 休眠 PCA9685 节省功耗
        delete pca9685_;
    }
}

void ServoController::SetServoAngle(uint8_t channel, int angle) {
    if (channel == SERVO_PCA9685_CH_X) {
        angle = angle < SERVO_X_MIN ? SERVO_X_MIN : (angle > SERVO_X_MAX ? SERVO_X_MAX : angle);
    } else {
        angle = angle < SERVO_Y_MIN ? SERVO_Y_MIN : (angle > SERVO_Y_MAX ? SERVO_Y_MAX : angle);
    }
    // 诊断：打印通道/角度/pulse 信息（公式与 pca9685.cc::SetServoAngle 保持一致）
    // SG90: 0.5ms~2.5ms → 0°~180°；12-bit @ 50Hz: 1 count = 20ms/4096
    uint16_t off_count = 102 + (uint16_t)((angle * 410.0f) / 180.0f);
    float pulse_us = (float)off_count * 20.0f * 1000.0f / 4096.0f;
    ESP_LOGI(TAG, "SetServoAngle: CH=%d angle=%d pulse_us=%.1f off_count=%d",
             channel, angle, pulse_us, off_count);
    pca9685_->SetServoAngle(channel, (float)angle);
}

void ServoController::QueueAction(int type, int p1, int p2) {
    ServoActionParams params = { type, p1, p2 };
    xQueueSend(action_queue_, &params, portMAX_DELAY);
    StartActionTaskIfNeeded();
}

void ServoController::StartActionTaskIfNeeded() {
    if (action_task_handle_ == nullptr) {
        xTaskCreate(ActionTask, "servo_action", 1024 * 3, this,
                    configMAX_PRIORITIES - 2, &action_task_handle_);
    }
}

void ServoController::ActionTask(void* arg) {
    ServoController* self = static_cast<ServoController*>(arg);
    ServoActionParams params;

    while (true) {
        if (xQueueReceive(self->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
            self->is_action_in_progress_ = true;

            switch (params.action_type) {
                case ACTION_SET_X:
                    self->SetServoAngle(SERVO_PCA9685_CH_X, params.param1);
                    break;
                case ACTION_SET_Y:
                    self->SetServoAngle(SERVO_PCA9685_CH_Y, params.param1);
                    break;
                case ACTION_LOOK_AROUND: {
                    // 摇头：左 -> 中 -> 右 -> 中
                    int delay = params.param1 > 0 ? params.param1 : 300;
                    int seq[] = { SERVO_X_MIN, SERVO_X_CENTER, SERVO_X_MAX, SERVO_X_CENTER };
                    for (int a : seq) {
                        self->SetServoAngle(SERVO_PCA9685_CH_X, a);
                        vTaskDelay(pdMS_TO_TICKS(delay));
                    }
                    break;
                }
                case ACTION_NOD: {
                    // 点头：低 -> 高 -> 低
                    int delay = params.param1 > 0 ? params.param1 : 300;
                    int seq[] = { SERVO_Y_MIN, SERVO_Y_CENTER, SERVO_Y_MAX, SERVO_Y_CENTER };
                    for (int a : seq) {
                        self->SetServoAngle(SERVO_PCA9685_CH_Y, a);
                        vTaskDelay(pdMS_TO_TICKS(delay));
                    }
                    break;
                }
                case ACTION_CENTER:
                    self->SetServoAngle(SERVO_PCA9685_CH_X, SERVO_X_CENTER);
                    self->SetServoAngle(SERVO_PCA9685_CH_Y, SERVO_Y_CENTER);
                    break;
            }
            self->is_action_in_progress_ = false;
        }
    }
}

void ServoController::RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();

    // 设定 X 轴角度
    mcp_server.AddTool(
        "self.servo.set_x",
        "水平摆头到指定角度。0=最左, 90=正前, 180=最右。常用范围 30-150。",
        PropertyList({
            Property("angle", kPropertyTypeInteger, 90, 0, 180)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int angle = properties["angle"].value<int>();
            ESP_LOGI(TAG, "set_x: %d°", angle);
            QueueAction(ACTION_SET_X, angle);
            return true;
        });

    // 设定 Y 轴角度
    mcp_server.AddTool(
        "self.servo.set_y",
        "垂直低头/抬头到指定角度。0=最低, 90=正前, 180=最高。常用范围 60-120。",
        PropertyList({
            Property("angle", kPropertyTypeInteger, 90, 0, 180)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int angle = properties["angle"].value<int>();
            ESP_LOGI(TAG, "set_y: %d°", angle);
            QueueAction(ACTION_SET_Y, angle);
            return true;
        });

    // 摇头
    mcp_server.AddTool(
        "self.servo.look_around",
        "左右看看（摇头动作）。speed_ms: 每步停留毫秒数（100-1000，数值越小越快）",
        PropertyList({
            Property("speed_ms", kPropertyTypeInteger, 300, 100, 1000)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int speed = properties["speed_ms"].value<int>();
            ESP_LOGI(TAG, "look_around (speed=%dms)", speed);
            QueueAction(ACTION_LOOK_AROUND, speed);
            return true;
        });

    // 点头
    mcp_server.AddTool(
        "self.servo.nod",
        "点头。speed_ms: 每步停留毫秒数（100-1000，数值越小越快）",
        PropertyList({
            Property("speed_ms", kPropertyTypeInteger, 300, 100, 1000)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int speed = properties["speed_ms"].value<int>();
            ESP_LOGI(TAG, "nod (speed=%dms)", speed);
            QueueAction(ACTION_NOD, speed);
            return true;
        });

    // 回中
    mcp_server.AddTool(
        "self.servo.center",
        "回中（看向正前方）。",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "center");
            QueueAction(ACTION_CENTER);
            return true;
        });

    // 获取状态
    mcp_server.AddTool(
        "self.servo.get_status",
        "获取云台状态，返回 moving（正在动）或 idle（静止）。",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            return is_action_in_progress_ ? "moving" : "idle";
        });

    ESP_LOGI(TAG, "Servo MCP 工具注册完成（6 个）");
}

void InitializeServoController() {
    static ServoController servos;
}
