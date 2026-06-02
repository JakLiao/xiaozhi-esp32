# Bread Compact WiFi GPIO18 - 麦克风引脚修复版

> 适用情况：ESP32-S3 的 GPIO5 / GPIO6 引脚损坏，导致麦克风（INMP441）无法拾音。

## 问题原因

ESP32-S3 模组的 GPIO4/5/6 同时是 SPI Flash/PSRAM 总线引脚，在面包板反复插拔后容易损坏。

`bread-compact-wifi` 原版固件中，麦克风 INMP441 使用 **I2S0**，引脚为：
- WS  → GPIO4
- SCK → **GPIO5 ❌ 损坏**
- DIN → **GPIO6 ❌ 损坏**

修复方案：将麦克风迁移到 **I2S0 专用 native pins** GPIO1/2/3，绕过损坏的 GPIO5/6。

## 接线变更（相比原 bread-compact-wifi）

### 麦克风 INMP441（仅修改引脚）

| 信号 | 原引脚（损坏）| 新引脚 | 说明 |
|------|------------|--------|------|
| WS | GPIO4 | **GPIO1** | I2S0 native WS |
| SCK | GPIO5 ❌ | **GPIO2** | I2S0 native BCK |
| DIN | GPIO6 ❌ | **GPIO3** | I2S0 native DIN |
| L/R GND | GND | GND | 不变 |
| VDD | 3.3V | 3.3V | 不变 |

### 扬声器 MAX98357（完全不变）

| 信号 | 引脚 | 说明 |
|------|------|------|
| DIN | GPIO7 | 音频数据输入 |
| BCLK | GPIO15 | 位时钟 |
| LRCK | GPIO16 | 声道时钟 |
| GND | GND | 地 |
| GAIN | GND | 短接 GND，默认增益 |
| SD | GND | 短接 GND，禁用关机 |
| VIN | 5V | 电源 |
| OUT+ | 喇叭正极 | 音频输出 |
| OUT- | 喇叭负极 | 音频输出 |

### 灯泡 LAMP（仅修改引脚）

| 信号 | 原引脚 | 新引脚 |
|------|--------|--------|
| 控制 | GPIO18 | **GPIO8** |

> 注：GPIO18 现用于扬声器 BCLK，灯泡改接 GPIO8（ESP32-S3 直连引脚，可安全使用）

### 显示屏 SSD1306 / 其他（不变）

| 信号 | 引脚 |
|------|------|
| SDA | GPIO41 |
| SCL | GPIO42 |
| 按钮/LED | 0/40/47/48 |

## 完整接线对照表（新板型）

| 硬件 | 信号 | GPIO | 备注 |
|------|------|------|------|
| INMP441 | WS | **1** | 新（原 GPIO4）|
| INMP441 | SCK | **2** | 新（原 GPIO5）|
| INMP441 | DIN | **3** | 新（原 GPIO6）|
| INMP441 | GND | GND | - |
| INMP441 | VDD | 3.3V | - |
| MAX98357 | DIN | GPIO7 | 不变 |
| MAX98357 | BCLK | GPIO15 | 不变 |
| MAX98357 | LRCK | GPIO16 | 不变 |
| MAX98357 | GND | GND | - |
| MAX98357 | VIN | 5V | - |
| SSD1306 | SDA | 41 | 不变 |
| SSD1306 | SCL | 42 | 不变 |
| 按钮/LED | - | 0/40/47/48 | 不变 |
| LAMP | 控制 | **8** | 新（原 GPIO18）|

## 编译方法

```bash
# 进入项目目录
cd E:\Code\EDA-robot\xiaozhi-esp32

# 设置目标芯片（首次配置）
idf.py set-target esp32s3

# 进入菜单选择板型
idf.py menuconfig
# → Xiaozhi Assistant → Board Type → Bread Compact WiFi GPIO18

# 编译
idf.py build

# 烧录（COM5 换成你的实际串口）
idf.py -p COM5 flash monitor
```

或使用 release.py 脚本（无需手动 menuconfig）：
```bash
python scripts/release.py bread-compact-wifi-gpio18
```

## 验证方法

1. 烧录后观察串口日志，确认 INMP441 初始化无报错
2. 对设备说"你好小智"，应有语音应答
3. 如无应答，检查接线是否正确（尤其 GND 共地）

## 已知约束

- GPIO1 已被占用（USB-DM），但在 ESP32-S3 WROOM 模组上不影响 I2S 功能
- GPIO18 同时是 ESP32-S3 native I2S1_BCK 引脚，MAX98357 BCLK 直连无 Matrix 延迟
