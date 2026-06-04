/*
 * ServoController v2.1 - 情感驱动的 2 自由度云台
 *
 * v2.1 改进（vs v2 复查后）：
 *   ✅ P0 PlayEmotion 找不到 emotion 时返回 false（让 LLM 知道）
 *   ✅ P0 QueueAction 不再阻塞（队列满时丢最旧、插最新）
 *   ✅ P0 agree/deny 时长从 4s 压到 ~1.5s（减 step + 加快 step_ms）
 *   ✅ P1 set_look 改为每次独立 step_ms（不再污染全局）
 *   ✅ P1 sad 重复 step 合并 / confused 拆段
 *   ✅ P1 命名 is_action_in_progress_ → is_action_running_
 *   ✅ P1 EmotionStep 增加 step_ms 字段（每 step 独立速度）
 *
 * 参考：
 *   - desk-emoji v2.0.1 head.cpp::head_move()（步进插值算法）
 *   - desk-emoji v2.0.1 act.cpp::speech_act()（emotion 编排思路）
 */

#include "servo_controller.h"
#include "mcp_server.h"
#include "freertos/FreeRTOS.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <cstring>
#include <string>
#include <vector>
#include <map>

#define TAG "ServoController"

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
            .enable_internal_pullup = true,
        },
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &bus_handle));

    // 2. 创建 PCA9685 驱动
    pca9685_ = new Pca9685(bus_handle, SERVO_PCA9685_ADDR);
    pca9685_->Init();

    // 3. 初始化舵机到中心（步进插值起点）
    current_x_ = SERVO_X_CENTER;
    current_y_ = SERVO_Y_CENTER;
    pca9685_->SetServoAngle(SERVO_PCA9685_CH_X, (float)current_x_);
    pca9685_->SetServoAngle(SERVO_PCA9685_CH_Y, (float)current_y_);

    // 4. 创建动作队列
    action_queue_ = xQueueCreate(10, sizeof(ServoActionParams));

    // 5. 注册 MCP 工具
    RegisterMcpTools();
    ESP_LOGI(TAG, "Servo 控制器 v2.1 已初始化（情感驱动 + 步进插值 + 队列非阻塞）");
    ESP_LOGI(TAG, "PCA9685 @ 0x%02X, X=CH%d(%d°-%d°), Y=CH%d(%d°-%d°)",
             SERVO_PCA9685_ADDR,
             SERVO_PCA9685_CH_X, SERVO_X_MIN, SERVO_X_MAX,
             SERVO_PCA9685_CH_Y, SERVO_Y_MIN, SERVO_Y_MAX);
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
        pca9685_->Sleep();
        delete pca9685_;
    }
}

int ServoController::ConstrainAngle(uint8_t ch, int angle) {
    if (ch == SERVO_PCA9685_CH_X) {
        if (angle < SERVO_X_MIN) return SERVO_X_MIN;
        if (angle > SERVO_X_MAX) return SERVO_X_MAX;
    } else {
        if (angle < SERVO_Y_MIN) return SERVO_Y_MIN;
        if (angle > SERVO_Y_MAX) return SERVO_Y_MAX;
    }
    return angle;
}

/*
 * MoveSmooth - 步进式平滑移动（核心）
 *
 * 从 current_x_/current_y_ 出发，按 SERVO_STEP_DEG (1°) 步进，
 * 每步 step_ms 毫秒，直到 target_angle。
 * 类似 desk-emoji v2.0.1 head.cpp::head_move() 的实现。
 */
void ServoController::MoveSmooth(uint8_t ch, int target_angle, int step_ms) {
    int* current = (ch == SERVO_PCA9685_CH_X) ?
                   const_cast<int*>(&current_x_) :
                   const_cast<int*>(&current_y_);

    target_angle = ConstrainAngle(ch, target_angle);

    if (step_ms <= 0) step_ms = SERVO_STEP_MS;

    while (*current != target_angle) {
        *current += (*current < target_angle) ? SERVO_STEP_DEG : -SERVO_STEP_DEG;
        pca9685_->SetServoAngle(ch, (float)*current);
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
}

/*
 * QueueAction - v2.1 改进：永不阻塞
 *
 * 队列满时策略：丢弃最旧，插入最新
 * 理由：情感驱动的场景下，"最新" 永远比"过去"重要
 *       旧 emotion 还没播完，LLM 来了新 emotion → 应该立即切到新的
 *       否则队列满时 MCP lambda 会被 portMAX_DELAY 阻塞，影响 LLM 流
 */
bool ServoController::QueueAction(int type, int p1, int p2, int p3, const char* str) {
    ServoActionParams params = {};
    params.action_type = type;
    params.param1 = p1;
    params.param2 = p2;
    params.param3 = p3;
    if (str != nullptr) {
        strncpy(params.str_param, str, sizeof(params.str_param) - 1);
        params.str_param[sizeof(params.str_param) - 1] = '\0';
    }

    // 如果队列满，丢一个最旧的腾位置
    if (xQueueSend(action_queue_, &params, 0) != pdTRUE) {
        ServoActionParams dropped;
        if (xQueueReceive(action_queue_, &dropped, 0) == pdTRUE) {
            ESP_LOGW(TAG, "QueueAction: 队列满，丢弃 %d (新 %d)",
                     dropped.action_type, type);
        }
        // 再试一次（可能仍满，但不会死等）
        if (xQueueSend(action_queue_, &params, 0) != pdTRUE) {
            ESP_LOGE(TAG, "QueueAction: 队列仍满，丢弃新动作 %d", type);
            return false;
        }
    }

    StartActionTaskIfNeeded();
    return true;
}

void ServoController::StartActionTaskIfNeeded() {
    if (action_task_handle_ == nullptr) {
        xTaskCreate(ActionTask, "servo_action", 1024 * 4, this,
                    configMAX_PRIORITIES - 2, &action_task_handle_);
    }
}

/*
 * Emotion 库 v2.1 - 10 种预制情感动作序列
 *
 * 设计原则（v2.1 改进）：
 *   - X 范围 30-150（中心 90，幅度 ±60）
 *   - Y 范围 60-120（中心 90，幅度 ±30）
 *   - 每个 step 包含 (x, y, hold_ms, step_ms)
 *   - step_ms=0 用 SERVO_STEP_MS 默认（10ms）
 *   - 同意/否定只用 1 次点头/摇头（4s 太长，1.5s 够表达）
 *   - confused 拆 X/Y 两段（避免两舵机并发视觉乱）
 *   - sad 重复 step 合并（hold_ms 累加）
 *   - 所有 emotion 总时长控制在 1.0~2.5s
 *
 * 触发场景（提示词中会告诉 LLM）：
 *   agree    - 同意、肯定、收到
 *   deny     - 反对、否定、不行
 *   thinking - 思考、让我想想
 *   happy    - 开心、哈、太棒了
 *   curious  - 好奇、咦、真的吗
 *   surprise - 惊讶、哇、不会吧
 *   sad      - 难过、遗憾、唉
 *   shy      - 害羞、不好意思
 *   proud    - 得意、骄傲
 *   confused - 困惑、糊涂、什么
 */
static const std::map<std::string, std::vector<EmotionStep>> kEmotionMap = {
    // 点头 1 次（短促坚定，约 1.0s）
    {"agree", {
        {90, 105, 150, 0},   // 低头
        {90, 75,  150, 0},   // 抬头
        {90, 90,  100, 0},   // 回到中位
    }},

    // 摇头 1 次（明显反对，约 1.2s）
    {"deny", {
        {55,  90, 200, 0},   // 左
        {125, 90, 200, 0},   // 右
        {90,  90, 100, 0},   // 回到中位
    }},

    // 思考：左右看 + 抬头凝视（约 2.0s）
    {"thinking", {
        {55,  90, 350, 0},   // 左看
        {125, 90, 350, 0},   // 右看
        {90,  75, 500, 0},   // 抬头（凝视远方）
        {90,  90, 100, 0},   // 回到中位
    }},

    // 开心：轻摇 + 微抬（约 1.5s）
    {"happy", {
        {75,  85, 150, 0},   // 左上轻摇
        {105, 85, 150, 0},   // 右上轻摇
        {90,  80, 250, 0},   // 微抬保持
        {90,  90, 100, 0},   // 回到中位
    }},

    // 好奇：头微右倾 + 微抬，约 1.4s
    {"curious", {
        {110, 85, 300, 0},   // 右倾
        {110, 85, 600, 0},   // 保持看
        {90,  90, 100, 0},   // 回到中位
    }},

    // 惊讶：快速大幅抬头（约 1.0s）
    {"surprise", {
        {90,  65, 250, 0},   // 猛抬头（加快）
        {90,  75, 400, 0},   // 微回（惊讶未消）
        {90,  90, 100, 0},   // 回到中位
    }},

    // 难过：低头保持（合并重复 step，约 1.5s）
    {"sad", {
        {90,  115, 1400, 0}, // 低头 1.4s（之前拆 800+600 等价）
        {90,  90,  100,  0}, // 慢回中
    }},

    // 害羞：微左倾 + 微低（约 1.4s）
    {"shy", {
        {75,  100, 300, 0},  // 微左倾 + 微低
        {75,  100, 800, 0},  // 保持
        {90,  90,  100, 0},  // 回到中位
    }},

    // 得意：单次抬头（约 1.0s）
    {"proud", {
        {90,  75, 350, 0},   // 抬
        {90,  80, 300, 0},   // 略回（得意保持）
        {90,  90, 100, 0},   // 回到中位
    }},

    // 困惑：拆 X/Y 两段（避免两舵机并发），约 2.0s
    // 段 1：水平 S 形扫动（2 步）
    // 段 2：回中
    {"confused", {
        {65,  90, 250, 0},   // 左扫
        {115, 90, 250, 0},   // 右扫
        {80,  90, 250, 0},   // 再左扫
        {100, 90, 250, 0},   // 再右扫
        {90,  90, 200, 0},   // 回中
    }},
};

bool ServoController::PlayEmotion(const char* name) {
    if (name == nullptr || name[0] == '\0') {
        ESP_LOGW(TAG, "PlayEmotion: null or empty name");
        return false;
    }
    std::string key(name);
    auto it = kEmotionMap.find(key);
    if (it == kEmotionMap.end()) {
        ESP_LOGW(TAG, "Unknown emotion: '%s' (支持: agree/deny/thinking/happy/curious/surprise/sad/shy/proud/confused)", name);
        return false;
    }

    ESP_LOGI(TAG, "PlayEmotion: %s (%d steps)", name, it->second.size());

    // 触发屏幕表情联动 hook（默认空）
    OnEmotionTriggered(name);

    // 串行执行 emotion 的 step 序列
    for (const auto& step : it->second) {
        // 步进插值到目标 X（每 step 独立 step_ms）
        if (step.x != current_x_) {
            MoveSmooth(SERVO_PCA9685_CH_X, step.x, step.step_ms);
        }
        // 步进插值到目标 Y
        if (step.y != current_y_) {
            MoveSmooth(SERVO_PCA9685_CH_Y, step.y, step.step_ms);
        }
        // 在该位置停留
        if (step.hold_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(step.hold_ms));
        }
    }
    return true;
}

/*
 * 屏幕表情联动 hook
 *
 * 默认空实现。如果主人想让 emotion 触发时同步切换 OLED 表情，
 * 在这里加 1-2 行代码即可。
 */
void ServoController::OnEmotionTriggered(const char* name) {
    // TODO: 联动屏幕表情（128x64 OLED 支持 emoji 切换）
    (void)name;
}

void ServoController::ActionTask(void* arg) {
    ServoController* self = static_cast<ServoController*>(arg);
    ServoActionParams params;

    while (true) {
        if (xQueueReceive(self->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
            self->is_action_running_ = true;

            switch (params.action_type) {
                case ACTION_PLAY_EMOTION:
                    self->PlayEmotion(params.str_param);
                    break;

                case ACTION_SET_LOOK: {
                    int x = self->ConstrainAngle(SERVO_PCA9685_CH_X, params.param1);
                    int y = self->ConstrainAngle(SERVO_PCA9685_CH_Y, params.param2);
                    int step = params.param3 > 0 ? params.param3 : SERVO_STEP_MS;
                    ESP_LOGI(TAG, "set_look: x=%d y=%d step_ms=%d (current x=%d y=%d)",
                             x, y, step, self->current_x_, self->current_y_);
                    if (x != self->current_x_) self->MoveSmooth(SERVO_PCA9685_CH_X, x, step);
                    if (y != self->current_y_) self->MoveSmooth(SERVO_PCA9685_CH_Y, y, step);
                    break;
                }

                case ACTION_CENTER:
                    ESP_LOGI(TAG, "center");
                    if (self->current_x_ != SERVO_X_CENTER)
                        self->MoveSmooth(SERVO_PCA9685_CH_X, SERVO_X_CENTER);
                    if (self->current_y_ != SERVO_Y_CENTER)
                        self->MoveSmooth(SERVO_PCA9685_CH_Y, SERVO_Y_CENTER);
                    break;

                case ACTION_MOVE_SMOOTH_X:
                    self->MoveSmooth(SERVO_PCA9685_CH_X, params.param1);
                    break;

                case ACTION_MOVE_SMOOTH_Y:
                    self->MoveSmooth(SERVO_PCA9685_CH_Y, params.param1);
                    break;
            }
            self->is_action_running_ = false;
        }
    }
}

/*
 * MCP 工具注册（v2.1 - 4 个工具）
 *
 * 设计原则：LLM 不直接控制角度，只声明"情感"或"视线方向"
 */
void ServoController::RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();

    // 工具 1: play_emotion（主用）
    mcp_server.AddTool(
        "self.servo.play_emotion",
        "播一个预制情感动作（动作和对话内容强匹配）。\n"
        "支持的 emotion（注意拼写，写错会调用失败）:\n"
        "  agree    - 同意、肯定、收到（点头）\n"
        "  deny     - 反对、否定、不行（摇头）\n"
        "  thinking - 思考、让我想想（左看→右看→抬头）\n"
        "  happy    - 开心、哈、太棒了（轻摇 + 微抬）\n"
        "  curious  - 好奇、咦、真的吗（头微右倾 + 微抬）\n"
        "  surprise - 惊讶、哇、不会吧（快速大幅抬头）\n"
        "  sad      - 难过、遗憾、唉（低头保持）\n"
        "  shy      - 害羞、不好意思（微左倾 + 微低）\n"
        "  proud    - 得意、骄傲（单次抬头）\n"
        "  confused - 困惑、什么鬼（S 形扫动）\n"
        "\n"
        "使用规则（v2.2 重写，防止 LLM 重复调造成语音循环）:\n"
        "1. **一整轮回复最多调一次** play_emotion\n"
        "2. 如果回复中可能出现多种情感，**只挑最强的那一个**，别每个情绪都调\n"
        "3. 把文字想好（整句/整段）后再调一次 emotion，让动作和文字自然衔接\n"
        "4. 中性对话不需要调用（云台保持中位）\n"
        "5. 设备端有去重：同一 emotion 5s 内 / 不同 emotion 2s 内重复调\n"
        "   会被直接拒掉（返回 ok=false error=throttled），不是 bug 是保护\n"
        "6. play_emotion 不会阻塞你的回复（队列非阻塞）\n"
        "7. **不要在调完 play_emotion 后立刻再调一次来「补动作」**——\n"
        "   这正是造成「声音重复播报最后一段」的根因",
        PropertyList({
            Property("emotion", kPropertyTypeString)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            std::string emotion = properties["emotion"].value<std::string>();
            ESP_LOGI(TAG, "MCP play_emotion: %s", emotion.c_str());

            // 尝试同步查找 emotion 名字（不入队），无效时直接返回 false
            std::string key(emotion);
            if (kEmotionMap.find(key) == kEmotionMap.end()) {
                ESP_LOGE(TAG, "MCP play_emotion: 不支持的 emotion '%s'", emotion.c_str());
                return "{\"ok\":false,\"error\":\"unknown emotion: " + emotion +
                       "（支持: agree/deny/thinking/happy/curious/surprise/sad/shy/proud/confused）\"}";
            }

            // v2.2 抗循环：throttling 检查
            // 不同 emotion 最小间隔 2s；同一 emotion 额外冷却到 5s
            // 目的：强制 LLM 不要在短窗口内反复调 play_emotion + 说话
            int64_t now_ms = esp_timer_get_time() / 1000;
            int64_t elapsed = now_ms - last_play_emotion_ms_;
            if (last_play_emotion_ms_ > 0) {
                bool is_same = (emotion == last_play_emotion_name_);
                int64_t cooldown = is_same ? kPlayEmotionSameCooldownMs : kPlayEmotionMinIntervalMs;
                if (elapsed < cooldown) {
                    int64_t remain_ms = cooldown - elapsed;
                    ESP_LOGW(TAG, "MCP play_emotion throttled: %lldms since last '%s' (requested '%s'), remain %lldms",
                             (long long)elapsed, last_play_emotion_name_.c_str(), emotion.c_str(),
                             (long long)remain_ms);
                    return std::string("{\"ok\":false,\"error\":\"throttled: ") +
                           (is_same ? std::string("同一 emotion '") + emotion + "' 5s 冷却中"
                                    : std::string("play_emotion 2s 内重复调用")) +
                           "（剩余 " + std::to_string(remain_ms / 1000) +
                           "s），一整轮回复最多调一次\"}";
                }
            }

            last_play_emotion_ms_ = now_ms;
            last_play_emotion_name_ = emotion;

            QueueAction(ACTION_PLAY_EMOTION, 0, 0, 0, emotion.c_str());
            return "{\"ok\":true}";
        });

    // 工具 2: set_look（视线方向，v2.1 改进：step_ms 每次独立）
    mcp_server.AddTool(
        "self.servo.set_look",
        "把视线平滑移动到指定方向（**只动视线，不表达情感**）。\n"
        "X: 0-180（0=最左, 90=正前, 180=最右，推荐 30-150）\n"
        "Y: 0-180（0=最低, 90=正前, 180=最高，推荐 60-120）\n"
        "speed_ms: 步进间隔毫秒（5-50，越小越快，默认 10）\n"
        "\n"
        "使用规则：\n"
        "1. 需要'看着某个方向'时使用（不表达情感）\n"
        "2. 不影响 play_emotion 的步进速度（每次独立）",
        PropertyList({
            Property("x", kPropertyTypeInteger, 90, 0, 180),
            Property("y", kPropertyTypeInteger, 90, 0, 180),
            Property("speed_ms", kPropertyTypeInteger, 10, 5, 50)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int x = properties["x"].value<int>();
            int y = properties["y"].value<int>();
            int speed = properties["speed_ms"].value<int>();
            ESP_LOGI(TAG, "MCP set_look: x=%d y=%d speed=%dms", x, y, speed);
            QueueAction(ACTION_SET_LOOK, x, y, speed);
            return "{\"ok\":true}";
        });

    // 工具 3: center（回中）
    mcp_server.AddTool(
        "self.servo.center",
        "立即回中（看向正前方）。通常不需要调用，play_emotion 内部会自动回中。",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "MCP center");
            QueueAction(ACTION_CENTER);
            return "{\"ok\":true}";
        });

    // 工具 4: get_status（状态查询）
    mcp_server.AddTool(
        "self.servo.get_status",
        "获取云台状态，返回 'moving'（正在动）或 'idle'（静止）。",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            return is_action_running_ ? "moving" : "idle";
        });

    ESP_LOGI(TAG, "Servo MCP 工具 v2.1 注册完成（4 个: play_emotion/set_look/center/get_status）");
    ESP_LOGI(TAG, "Emotion 库: 10 种（agree/deny/thinking/happy/curious/surprise/sad/shy/proud/confused）");
}

void InitializeServoController() {
    static ServoController servos;
}
