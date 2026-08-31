// components/bsp/src/bsp_display.c
// ST7789 系面板 SPI 初始化 + 厂商寄存器 + 背光。
//
// 双板支持:
//   ESP32-C3 (FoloToy-Card) : ST7789P3 240x320,背光 GPIO21(LEDC)
//   ESP32    (M5StickC Plus): ST7789V2 135x240(偏移 52,40),LCD 电源/背光走 AXP192
// 引脚与分辨率在 bsp_pins.h 按 CONFIG_IDF_TARGET_ESP32 区分。
#include "bsp_display.h"
#include "bsp_pins.h"
#include "bsp_i2c.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp_disp";

static esp_lcd_panel_handle_t    s_panel;
static esp_lcd_panel_io_handle_t s_io;
static bool                      s_bl_ready;

// ---------------------------------------------------------------------------
// ST7789 厂商专属初始化序列(porch / power / gamma)。
// 这些是【面板厂给的参考例程 TFT_init() 里的值】,不是 ST7789 通用默认值 ——
// 换面板必须找对应厂商要新的一份,照抄这份大概率显示异常。
//
// 以下四条由 esp_lcd 内置驱动完成,故此处不重复:
//   0x11 SLPOUT / 0x3A COLMOD → esp_lcd_panel_init()
//   0x21 INVON                → esp_lcd_panel_invert_color()
//   0x29 DISPON               → esp_lcd_panel_disp_on_off()
//   0x36 MADCTL               → esp_lcd_panel_mirror()(⚠ 别再手动写 0x36,会被它覆盖)
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t  cmd;
    uint8_t  data[16];
    uint8_t  len;
    uint16_t delay_ms;
} st_init_cmd_t;

static const st_init_cmd_t ST7789_CMDS[] = {
    {0xB2, {0x05, 0x05, 0x00, 0x33, 0x33}, 5, 0},   // PORCTRL 帧率 porch
    {0xB7, {0x35}, 1, 0},                            // GCTRL 栅极
    {0xBB, {0x21}, 1, 0},                            // VCOMS
    {0xC0, {0x2C}, 1, 0},                            // LCMCTRL
    {0xC2, {0x01}, 1, 0},                            // VDVVRHEN
    {0xC3, {0x0B}, 1, 0},                            // VRHS
    {0xC4, {0x20}, 1, 0},                            // VDVSET
    {0xC6, {0x0F}, 1, 0},                            // FRCTRL2 60Hz 点反转
    {0xD0, {0xA7, 0xA1}, 2, 0},                      // PWCTRL1
    {0xD0, {0xA4, 0xA1}, 2, 0},                      // PWCTRL1(参考例程重发,覆盖上一条)
    {0xD6, {0xA1}, 1, 0},
    {0xE0, {0xD0, 0x04, 0x08, 0x0A, 0x09, 0x05, 0x2D, 0x43,
            0x49, 0x09, 0x16, 0x15, 0x26, 0x2B}, 14, 0},   // PVGAMCTRL 正伽马
    {0xE1, {0xD0, 0x03, 0x09, 0x0A, 0x0A, 0x06, 0x2E, 0x44,
            0x40, 0x3A, 0x15, 0x15, 0x26, 0x2A}, 14, 10},  // NVGAMCTRL 负伽马
};

// ---------------------------------------------------------------------------
// M5StickC Plus:LCD 逻辑电源(LDO2)与背光(LDO3)都由 AXP192 管理,无 GPIO 背光。
// 序列照抄 nopnop2002/esp-idf-m5stickC-Plus 的 AXP192_PowerOn(实机验证)。
// ---------------------------------------------------------------------------
#if CONFIG_IDF_TARGET_ESP32
static esp_err_t axp_write_reg(uint8_t reg, uint8_t val) {
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) return ESP_ERR_INVALID_STATE;
    i2c_master_dev_handle_t dev;
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BSP_I2C_AXP192_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t e = i2c_master_bus_add_device(bus, &dc, &dev);
    if (e != ESP_OK) return e;
    uint8_t b[2] = { reg, val };
    e = i2c_master_transmit(dev, b, 2, 1000);
    i2c_master_bus_rm_device(dev);
    return e;
}

static esp_err_t axp_read_reg(uint8_t reg, uint8_t *out) {
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) return ESP_ERR_INVALID_STATE;
    i2c_master_dev_handle_t dev;
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BSP_I2C_AXP192_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t e = i2c_master_bus_add_device(bus, &dc, &dev);
    if (e != ESP_OK) return e;
    e = i2c_master_transmit_receive(dev, &reg, 1, out, 1, 1000);
    i2c_master_bus_rm_device(dev);
    return e;
}

// AXP192 上电:LDO2 & LDO3 = 3.0V → LCD 逻辑电源 + 背光电源轨
static void axp_display_power_init(void) {
    if (bsp_i2c_init() != ESP_OK) { ESP_LOGE(TAG, "AXP192 需要 I2C,初始化失败"); return; }

    // 0x28: LDO2(高4位) & LDO3(低4位) 电压 = 0xC → 3.0V
    axp_write_reg(0x28, 0xCC);
    // 0x84: ADC 采样率 200Hz
    axp_write_reg(0x84, 0xF2);
    // 0x82: ADC 全使能
    axp_write_reg(0x82, 0xFF);
    // 0x33: 充电电流 100mA
    axp_write_reg(0x33, 0xC0);
    // 0x12: 使能 LDO2/LDO3/DCDC1/DCDC3(保留现有位)(专版: (data&0xEF)|0x4D)
    uint8_t v = 0;
    if (axp_read_reg(0x12, &v) == ESP_OK) axp_write_reg(0x12, (uint8_t)((v & 0xEF) | 0x4D));
    // 0x36: 电源键 128ms 开 / 4s 关
    axp_write_reg(0x36, 0x0C);
    // 0x91: RTC 电压 3.3V
    axp_write_reg(0x91, 0xF0);
    // 0x90: GPIO0 → LDO
    axp_write_reg(0x90, 0x02);
    // 0x30: 关闭 vbus 限流
    axp_write_reg(0x30, 0x80);
    // 0x35: 使能 RTC 电池充电
    axp_write_reg(0x35, 0xA2);
    // 0x32: 使能电池检测
    axp_write_reg(0x32, 0x46);

    s_bl_ready = true;
    ESP_LOGI(TAG, "AXP192 上电完成:LDO2/LDO3 = LCD 电源+背光");
}
#endif

// M5StickC Plus 背光亮度 = AXP192 0x28 高 4 位(LDO3 电压档 0~12)。
#if CONFIG_IDF_TARGET_ESP32
static void axp_set_backlight(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint8_t bri = (uint8_t)((percent * 12u) / 100u);   // 0..12
    uint8_t v = 0;
    if (axp_read_reg(0x28, &v) == ESP_OK) {
        axp_write_reg(0x28, (uint8_t)((v & 0x0F) | (bri << 4)));
    }
}
#endif

static void backlight_init(void) {
#if CONFIG_IDF_TARGET_ESP32
    // M5StickC Plus:电源/背光在 AXP192,需先于面板初始化调用
    axp_display_power_init();
    return;
#endif
    if (BSP_LCD_BL < 0) { ESP_LOGW(TAG, "背光引脚未接 MCU,亮度不可调"); return; }
    ledc_timer_config_t t = {
        .speed_mode      = BSP_BL_LEDC_MODE,
        .timer_num       = BSP_BL_LEDC_TIMER,
        .duty_resolution = BSP_BL_LEDC_RES,
        .freq_hz         = BSP_BL_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t e = ledc_timer_config(&t);
    if (e != ESP_OK) { ESP_LOGE(TAG, "ledc_timer_config 失败: %s", esp_err_to_name(e)); return; }

    ledc_channel_config_t ch = {
        .gpio_num   = BSP_LCD_BL,
        .speed_mode = BSP_BL_LEDC_MODE,
        .channel    = BSP_BL_LEDC_CHANNEL,
        .timer_sel  = BSP_BL_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    e = ledc_channel_config(&ch);
    if (e != ESP_OK) { ESP_LOGE(TAG, "ledc_channel_config 失败: %s", esp_err_to_name(e)); return; }

    s_bl_ready = true;
    ESP_LOGI(TAG, "背光 LEDC 就绪 gpio=%d", BSP_LCD_BL);
}

esp_err_t bsp_display_init(void) {
    if (s_panel) return ESP_OK;

#if CONFIG_IDF_TARGET_ESP32
    // ⚠ 顺序关键:必须先给面板上电(AXP192 LDO2),再初始化 SPI/面板。
    axp_display_power_init();
#endif

    spi_bus_config_t bus = {
        .mosi_io_num = BSP_LCD_MOSI,
        .sclk_io_num = BSP_LCD_SCLK,
        .miso_io_num = -1, .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = BSP_LCD_W * 80 * 2,
    };
    esp_err_t e = spi_bus_initialize(BSP_LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "SPI 总线初始化失败 (%s) —— 检查 MOSI=GPIO%d / SCLK=GPIO%d 是否冲突",
                 esp_err_to_name(e), BSP_LCD_MOSI, BSP_LCD_SCLK);
        return e;
    }

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = BSP_LCD_CS,
        .dc_gpio_num = BSP_LCD_DC,
        .pclk_hz = BSP_LCD_PCLK_HZ,
        .spi_mode = BSP_LCD_SPI_MODE,
        .lcd_cmd_bits = 8, .lcd_param_bits = 8,
        .trans_queue_depth = 10,
    };
    e = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_HOST, &io_cfg, &s_io);
    if (e != ESP_OK) { ESP_LOGE(TAG, "panel_io 创建失败: %s", esp_err_to_name(e)); return e; }

    esp_lcd_panel_dev_config_t dev = {
        .reset_gpio_num = BSP_LCD_RST,          // GPIO18 实体复位
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    e = esp_lcd_new_panel_st7789(s_io, &dev, &s_panel);
    if (e != ESP_OK) { ESP_LOGE(TAG, "面板创建失败: %s", esp_err_to_name(e)); return e; }

    esp_lcd_panel_reset(s_panel);   // 实体 RST(GPIO18)或 SWRESET
    esp_lcd_panel_init(s_panel);    // SLPOUT / COLMOD / RAMCTRL

    for (size_t i = 0; i < sizeof(ST7789_CMDS) / sizeof(ST7789_CMDS[0]); i++) {
        const st_init_cmd_t *c = &ST7789_CMDS[i];
        esp_err_t r = esp_lcd_panel_io_tx_param(s_io, c->cmd, c->data, c->len);
        if (r != ESP_OK) ESP_LOGE(TAG, "厂商初始化命令 0x%02X 失败: %s", c->cmd, esp_err_to_name(r));
        if (c->delay_ms) vTaskDelay(pdMS_TO_TICKS(c->delay_ms));
    }

    esp_lcd_panel_invert_color(s_panel, BSP_LCD_INVERT_COLOR);   // 0x21 / 0x20
#if CONFIG_IDF_TARGET_ESP32
    // ⚠ M5StickC Plus 的 ST7789V2 是 135x240 面板,但控制器 RAM 是 240x320,
    //   可见区域居中于 RAM → 必须设置偏移 x=52, y=40(esphome/专版驱动同值)。
    //   不设偏移 = 内容画到面板不可见区域 = 黑屏(实测!)。
    esp_lcd_panel_mirror(s_panel, false, false);
    esp_lcd_panel_set_gap(s_panel, 52, 40);
#else
    esp_lcd_panel_mirror(s_panel, false, false);                 // 0x36 MADCTL:默认不需镜像
    esp_lcd_panel_set_gap(s_panel, 0, 0);
#endif
    esp_lcd_panel_disp_on_off(s_panel, true);                    // 0x29 DISPON

    backlight_init();
    ESP_LOGI(TAG, "显示就绪 %dx%d", BSP_LCD_W, BSP_LCD_H);
    return ESP_OK;
}

esp_lcd_panel_handle_t bsp_display_panel(void) { return s_panel; }

esp_lcd_panel_io_handle_t bsp_display_io(void) { return s_io; }

void bsp_display_backlight(uint8_t percent) {
    if (!s_bl_ready) return;
    if (percent > 100) percent = 100;
#if CONFIG_IDF_TARGET_ESP32
    axp_set_backlight(percent);
#else
    uint32_t max_duty = (1u << BSP_BL_LEDC_RES) - 1u;
    uint32_t duty = (max_duty * percent) / 100u;
    ledc_set_duty(BSP_BL_LEDC_MODE, BSP_BL_LEDC_CHANNEL, duty);
    ledc_update_duty(BSP_BL_LEDC_MODE, BSP_BL_LEDC_CHANNEL);
#endif
}
