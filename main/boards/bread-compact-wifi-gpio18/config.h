#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 如果使用 Duplex I2S 模式，请注释下面一行
#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX

// ============================================================
// 修复说明：
// 原 bread-compact-wifi 麦克风在 GPIO4/5/6，其中 GPIO5/6 损坏。
// GPIO4/5/6 是 ESP32-S3 SPI Flash 总线引脚（PSRAM/Flash 共享），
// 部分模组上这些引脚也兼做其他用途，易损坏。
//
// 麦克风改到 GPIO1/2/3（I2S0 专用 native pins），避开损坏引脚。
// 扬声器保持在 GPIO7/15/16（I2S1，完好）。
// ============================================================

// 麦克风 INMP441：GPIO1(WS) / GPIO2(SCK) / GPIO3(DIN)
// - GPIO1/2/3 是 ESP32-S3 I2S0 专用 native pins
// - 全部完好，避开损坏的 GPIO5/6
// - I2S0 作为 Master，输出 WS/SCK，INMP441 作为 Slave
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_1
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_2
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_3

// 扬声器 MAX98357：保持 GPIO7/BCLK / GPIO15/LRCK / GPIO16/DIN
// - 这些引脚在原 bread-compact-wifi 配置下正常工作
// - GPIO7 = I2S1 BCK（native），GPIO15 = I2S1 WS（native），GPIO16 = I2S1 DOUT（native）
// - 不变，不碰
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16

#else

#define AUDIO_I2S_GPIO_WS GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7

#endif


#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define TOUCH_BUTTON_GPIO       GPIO_NUM_47
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_40
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_39

// OLED SSD1306：保持在 GPIO41/42（I2C，不变）
#define DISPLAY_SDA_PIN GPIO_NUM_41
#define DISPLAY_SCL_PIN GPIO_NUM_42
#define DISPLAY_WIDTH   128

#if CONFIG_OLED_SSD1306_128X32
#define DISPLAY_HEIGHT  32
#elif CONFIG_OLED_SSD1306_128X64
#define DISPLAY_HEIGHT  64
#elif CONFIG_OLED_SH1106_128X64
#define DISPLAY_HEIGHT  64
#define SH1106
#else
#error "OLED display type is not selected"
#endif

#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true


// 灯泡控制：原来在 GPIO18（BCLK 直连引脚），改为 GPIO8
// 注意：GPIO8 是 ESP32-S3 直连引脚（非 PSRAM/Flash），可安全使用
#define LAMP_GPIO GPIO_NUM_8

#endif // _BOARD_CONFIG_H_
