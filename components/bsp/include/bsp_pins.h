// components/bsp/include/bsp_pins.h
// 硬件引脚与参数的【单一事实来源】。换板/改硬件只需改这一个文件。
// 每项都注明"为什么是这个值",便于二次开发时判断能不能改。
//
// ⚠ 本文件同时服务两块板:
//   - ESP32-C3 → FoloToy-Card(默认分支,下方全部原定义)
//   - ESP32    → M5Stack Core2(用 #if CONFIG_IDF_TARGET_ESP32 隔离)
// 两套代码互不干扰,切回 C3 只需 idf.py set-target esp32c3。
#pragma once

#include "driver/spi_master.h"
#include "driver/i2c_types.h"
#include "hal/adc_types.h"

// ============================================================================
// M5StickC Plus 1.1(ESP32-PICO-D4)引脚
// 屏幕 ST7789V2 135x240,音频经 AXP192 供电,实体按键 GPIO37。
// 引脚值来自 M5Stack 官方原理图与 Zephyr m5stickc_plus 板级支持。
// ============================================================================
#if CONFIG_IDF_TARGET_ESP32

// --- 显示:ST7789V2 135x240,4-line SPI ---
#define BSP_LCD_W            135
#define BSP_LCD_H            240
#define BSP_LCD_SPI_HOST     SPI3_HOST   // M5StickC 用 VSPI(TFT_eSPI 默认)
#define BSP_LCD_MOSI         15
#define BSP_LCD_MISO         (-1)
#define BSP_LCD_SCLK         13
#define BSP_LCD_CS           5
#define BSP_LCD_DC           23
#define BSP_LCD_RST          18          // 实体复位脚,GPIO 直接驱动
// ⚠ M5StickC Plus 无 GPIO 背光:背光在 AXP192 的 LDO3 电源轨上(bsp_display.c 的 axp 分支)。
#define BSP_LCD_BL           (-1)
#define BSP_LCD_PCLK_HZ      (80 * 1000 * 1000)   // 优化A1: 40->80MHz,推屏耗时减半
#define BSP_LCD_SPI_MODE     0
// M5StickC Plus 的 ST7789V2 需要反色(M5 官方 TFT 库默认 INVON)。
#define BSP_LCD_INVERT_COLOR 1

// 背光 LEDC 参数
#define BSP_BL_LEDC_TIMER    LEDC_TIMER_0
#define BSP_BL_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define BSP_BL_LEDC_CHANNEL  LEDC_CHANNEL_0
#define BSP_BL_LEDC_RES      LEDC_TIMER_10_BIT
#define BSP_BL_LEDC_FREQ_HZ  5000

// --- 按键:M5StickC Plus 只有实体按键(BtnA=GPIO37,低有效) ---
#define BSP_BTN_COUNT        1
#define BSP_BTN_GPIO         37        // 低电平有效
#define BSP_BTN_MV_TABLE     { {0, 0} }   // 无 ADC 电压,占位

// --- I2C:AXP192/BM8563/MPU6886 共用内部总线 ---
#define BSP_I2C_PORT         I2C_NUM_0
#define BSP_I2C_SDA          21
#define BSP_I2C_SCL          22
#define BSP_I2C_ES8311_ADDR  0x18   // M5StickC Plus 无 ES8311;保留地址定义
#define BSP_I2C_CW2017_ADDR  0x63   // 无 CW2017;保留定义,扫描时它不应答即可
#define BSP_I2C_AXP192_ADDR  0x34   // 电源管理
#define BSP_I2C_FT6336U_ADDR 0x38   // 无触摸屏;保留定义

// --- 音频:M5StickC Plus 喇叭经 I2S 直驱(无 codec 芯片) ---
// ⚠ 引脚经官方 FactoryTest 固件确认:麦克风 WS(CLK)=GPIO0, DATA_IN=GPIO34。
//   喇叭与麦克风共用 I2S_NUM_0。DOUT 候选 GPIO2/26/25,见 bsp_audio.c 注释。
#define BSP_I2S_PORT         I2S_NUM_0
#define BSP_I2S_MCLK         (-1)   // NC:无外部 MCLK
#define BSP_I2S_BCLK         26      // ★ 待验证:候选 GPIO26(HA 社区记录)或 0
#define BSP_I2S_WS           0       // ★ 官方确认:WS=GPIO0(FactoryTest 固件)
#define BSP_I2S_DOUT         2       // ★ 待验证:候选 GPIO2/26/25
#define BSP_I2S_DIN          (-1)    // 麦克风 DATA_IN=GPIO34(未用)
#define BSP_I2S_PA_CTRL      (-1)

#else  // ---- FoloToy-Card(ESP32-C3)原定义,勿动 ----

// ============================================================================
// 显示:ST7789P3 240x320,4-line SPI
// ============================================================================
#define BSP_LCD_W            240
#define BSP_LCD_H            320
#define BSP_LCD_SPI_HOST     SPI2_HOST
#define BSP_LCD_MOSI         9
#define BSP_LCD_SCLK         8
#define BSP_LCD_CS           1
#define BSP_LCD_DC           20
// -1 = 复位脚未接 MCU(硬接 3.3V),由 esp_lcd_panel_reset() 走 SWRESET 软复位。
#define BSP_LCD_RST          (-1)
#define BSP_LCD_BL           21          // 背光,LEDC PWM 调光
#define BSP_LCD_PCLK_HZ      (80 * 1000 * 1000)   // 优化A1: 40->80MHz,推屏耗时减半
// ST7789 SCK 空闲低、上升沿采样 → SPI mode 0。
#define BSP_LCD_SPI_MODE     0
// 本屏出厂即需反色(参考例程 TFT_init() 末尾无条件发 0x21 INVON)。
// 若换屏后画面呈负片,把这里改成 0。
#define BSP_LCD_INVERT_COLOR 1

// 背光 LEDC 参数
#define BSP_BL_LEDC_TIMER    LEDC_TIMER_0
#define BSP_BL_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define BSP_BL_LEDC_CHANNEL  LEDC_CHANNEL_0
#define BSP_BL_LEDC_RES      LEDC_TIMER_10_BIT
#define BSP_BL_LEDC_FREQ_HZ  5000

// ============================================================================
// 按键:三键共用一个 ADC 引脚,靠分压电阻区分
//
// 电路:3.3V ── 外部上拉 10k ──┬── ADC 节点(GPIO0 / ADC1_CH0)
//                              └── 按键 ── 分压电阻 ── GND
//
//   上   : 0Ω    → 3.3 x 0/10k       =   0 mV
//   下   : 1k    → 3.3 x 1k/11k      ≈ 300 mV
//   确定 : 2.2k  → 3.3 x 2.2k/12.2k  ≈ 595 mV
//   松开 : 无通路 → 上拉到 3300 mV
//
// ⚠ 不能改用【内部上拉】:约 45kΩ 且精度差,会把三档全挤到 0~154mV 并随温漂重叠。
//
// ★ 换了分压/上拉阻值怎么办:进 demo 的 Button 页,它实时显示当前 ADC 电压;
//   逐个按住三个键记下读数,取相邻两档的中点作为窗口边界,改下面的 BSP_BTN_MV 即可。
// ============================================================================
#define BSP_BTN_ADC_UNIT     ADC_UNIT_1
#define BSP_BTN_ADC_CHANNEL  ADC_CHANNEL_0    // GPIO0
#define BSP_BTN_COUNT        3

// 每键的电压窗口 {min_mV, max_mV};边界取相邻档中点。
// 确定键上界留宽到 1900,是为了和松开态的 3300mV 拉开距离。
#define BSP_BTN_MV_TABLE  { {0, 150}, {150, 447}, {447, 1900} }

// ============================================================================
// I2C:ES8311(音频 codec)与 CW2017(电量计)共用一条总线
// ============================================================================
#define BSP_I2C_PORT         I2C_NUM_0
#define BSP_I2C_SDA          10
#define BSP_I2C_SCL          7
#define BSP_I2C_ES8311_ADDR  0x18    // 7 位地址(8 位形式为 0x30)
#define BSP_I2C_CW2017_ADDR  0x63    // 7 位地址

// ============================================================================
// 音频:ES8311,I2S 全双工(同端口一 tx 一 rx,共用 MCLK/BCLK/WS)
// ============================================================================
#define BSP_I2S_PORT         I2S_NUM_0
#define BSP_I2S_MCLK         6
#define BSP_I2S_BCLK         5
#define BSP_I2S_WS           3
#define BSP_I2S_DOUT         2       // 播放:MCU → codec
#define BSP_I2S_DIN          4       // 录音:codec → MCU
// -1 = 功放使能脚未接 MCU(常通)。若你的板子接了,填对应 GPIO。
#define BSP_I2S_PA_CTRL      (-1)

#endif  // CONFIG_IDF_TARGET_ESP32 / FoloToy-Card(ESP32-C3)
