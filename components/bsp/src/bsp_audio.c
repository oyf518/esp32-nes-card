// components/bsp/src/bsp_audio.c
//
// 双板支持:
//   ESP32-C3 (FoloToy-Card) : ES8311 codec + I2S 全双工
//   ESP32    (M5StickC Plus): 内置喇叭 = GPIO2 单线 buzzer(LEDC 方波)
//
// ⚠ M5StickC Plus 音频引脚实测结论(M5Unified 官方源码确认):
//   内置喇叭不是标准三线 I2S,而是 pin_data_out = GPIO2 的 buzzer 模式。
//   之前按 I2S(BCLK/WS/DOUT)驱动全部无声,改 LEDC 方波后立即出声。
#include "bsp_audio.h"
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdint.h>

static const char *TAG = "bsp_audio";

#if CONFIG_IDF_TARGET_ESP32
// ===========================================================================
// M5StickC Plus:buzzer 喇叭 = LEDC 方波驱动 GPIO2
// ===========================================================================
#include "driver/ledc.h"

#define BUZZER_GPIO     BSP_I2S_DOUT    // GPIO2
#define BUZZER_TIMER    LEDC_TIMER_0
#define BUZZER_MODE     LEDC_LOW_SPEED_MODE
#define BUZZER_CHANNEL  LEDC_CHANNEL_0

static bool     s_buzzer_ready;
static uint32_t s_rate;                 // 当前采样率(默认 16000)
static uint32_t s_cur_freq;             // 当前实际发声频率(平滑用)

// 过零检测:统计本次 chunk 内正→负过零次数 → 平均频率。
// 频率公式:每周期 2 次过零,采样数 samples 覆盖时间 = samples/rate 秒
//   freq = (crossings / 2) / (samples / rate) = crossings * rate / (2 * samples)
// 平滑:频率跳变限速(每 16ms 最多变化 ±150Hz),避免方波切换爆音。
static void buzzer_play(const int16_t *pcm, size_t samples) {
    if (!s_buzzer_ready || samples < 4) return;
    int crossings = 0;
    for (size_t i = 1; i < samples; i++) {
        if (pcm[i-1] >= 0 && pcm[i] < 0) crossings++;
    }
    uint32_t freq = 0;
    if (crossings > 0) {
        freq = (uint32_t)(((uint64_t)crossings * s_rate) / (2 * samples));
    }
    // 限幅到 buzzer 可发声范围(100Hz ~ 8kHz);静音/超范围则停
    if (freq < 100 || freq > 8000) freq = 0;

    // 平滑:新频率与当前频率差异限速,静音时直接停
    if (freq == 0) {
        if (s_cur_freq) {
            ledc_stop(BUZZER_MODE, BUZZER_CHANNEL, 0);
            s_cur_freq = 0;
        }
        return;
    }
    int32_t diff = (int32_t)freq - (int32_t)s_cur_freq;
    if (diff > 150) diff = 150;
    if (diff < -150) diff = -150;
    freq = (s_cur_freq == 0) ? freq : (uint32_t)((int32_t)s_cur_freq + diff);
    s_cur_freq = freq;

    ledc_timer_config_t t = {
        .speed_mode = BUZZER_MODE,
        .timer_num = BUZZER_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = freq,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&t) != ESP_OK) return;
    // 50% 占空比
    ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 128);
    ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
}

esp_err_t bsp_audio_init(void) {
    if (s_buzzer_ready) return ESP_OK;

    // 功放/喇叭电源:AXP192 GPIO2 拉高(M5 官方 Speaker 电源控制)
    bsp_i2c_init();
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (bus) {
        i2c_master_dev_handle_t axp;
        i2c_device_config_t dc = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = BSP_I2C_AXP192_ADDR,
            .scl_speed_hz    = 400000,
        };
        if (i2c_master_bus_add_device(bus, &dc, &axp) == ESP_OK) {
            uint8_t reg = 0x94, v = 0;
            if (i2c_master_transmit_receive(axp, &reg, 1, &v, 1, 100) == ESP_OK) {
                uint8_t w[2] = { 0x94, (uint8_t)((v | 0x04) | 0xF0) };
                i2c_master_transmit(axp, w, 2, 100);
                ESP_LOGI(TAG, "AXP192 功放电源已开启");
            }
            i2c_master_bus_rm_device(axp);
        }
    }

    // LEDC 初始化 buzzer 通道
    ledc_timer_config_t t = {
        .speed_mode = BUZZER_MODE,
        .timer_num = BUZZER_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&t) != ESP_OK) { ESP_LOGE(TAG, "LEDC 定时器配置失败"); return ESP_FAIL; }
    ledc_channel_config_t ch = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = BUZZER_MODE,
        .channel = BUZZER_CHANNEL,
        .timer_sel = BUZZER_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    if (ledc_channel_config(&ch) != ESP_OK) { ESP_LOGE(TAG, "LEDC 通道配置失败"); return ESP_FAIL; }

    s_rate = 16000;
    s_buzzer_ready = true;
    ESP_LOGI(TAG, "buzzer 就绪:GPIO%d 方波喇叭", BUZZER_GPIO);
    return ESP_OK;
}

esp_err_t bsp_audio_set_format(uint32_t hz, uint8_t bits, uint8_t ch) {
    (void)bits; (void)ch;
    s_rate = hz ? hz : 16000;
    return ESP_OK;
}

esp_err_t bsp_audio_write(const void *pcm, size_t bytes) {
    if (!s_buzzer_ready) return ESP_ERR_INVALID_STATE;
    buzzer_play((const int16_t *)pcm, bytes / 2);
    return ESP_OK;
}

esp_err_t bsp_audio_read(void *pcm, size_t bytes) {
    (void)pcm; (void)bytes;
    return ESP_ERR_NOT_SUPPORTED;   // buzzer 无录音
}

void bsp_audio_set_volume(uint8_t percent) {
    (void)percent;                  // 方波无音量控制
}

// 直接驱动 GPIO2 方波喇叭。freq_hz=0 停止。
// ★ 用 ledc_timer_config 重建 timer(实测比 ledc_set_freq 波形干净,
//   运行中改分频会有相位不连续爆音)。频率相同跳过。
void bsp_audio_tone(uint32_t freq_hz) {
    if (!s_buzzer_ready) return;
    if (freq_hz == 0 || freq_hz < 100 || freq_hz > 8000) {
        if (s_cur_freq) { ledc_stop(BUZZER_MODE, BUZZER_CHANNEL, 0); s_cur_freq = 0; }
        return;
    }
    if (freq_hz == s_cur_freq) return;                 // 同频率不重复配置
    ledc_timer_config_t t = {
        .speed_mode = BUZZER_MODE,
        .timer_num = BUZZER_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&t) != ESP_OK) return;
    ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 128);   // 50% 占空比
    ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
    s_cur_freq = freq_hz;
}

#else  // ---- FoloToy-Card(ESP32-C3):ES8311 + I2S ----

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"
#include "driver/i2s_std.h"

static esp_codec_dev_handle_t s_dev;
static i2s_chan_handle_t      s_tx, s_rx;
static uint32_t s_hz;
static uint8_t  s_bits, s_ch;
static bool     s_opened;

static esp_err_t i2s_full_duplex_init(void) {
    i2s_chan_config_t chan = {
        .id = BSP_I2S_PORT,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    esp_err_t e = i2s_new_channel(&chan, &s_tx, &s_rx);
    if (e != ESP_OK) { ESP_LOGE(TAG, "i2s_new_channel 失败: %s", esp_err_to_name(e)); return e; }

    i2s_std_config_t std = {
        .clk_cfg = {
            .sample_rate_hz = 16000,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK, .bclk = BSP_I2S_BCLK, .ws = BSP_I2S_WS,
            .dout = BSP_I2S_DOUT, .din = BSP_I2S_DIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    if ((e = i2s_channel_init_std_mode(s_tx, &std)) != ESP_OK) {
        ESP_LOGE(TAG, "i2s tx 初始化失败: %s", esp_err_to_name(e)); return e;
    }
    if ((e = i2s_channel_init_std_mode(s_rx, &std)) != ESP_OK) {
        ESP_LOGE(TAG, "i2s rx 初始化失败: %s", esp_err_to_name(e)); return e;
    }
    i2s_channel_enable(s_tx);
    i2s_channel_enable(s_rx);
    return ESP_OK;
}

esp_err_t bsp_audio_init(void) {
    if (s_dev) return ESP_OK;

    esp_err_t e = bsp_i2c_init();
    if (e != ESP_OK) return e;

    const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&(audio_codec_i2c_cfg_t){
        .port = BSP_I2C_PORT,
        .addr = BSP_I2C_ES8311_ADDR << 1,
        .bus_handle = bsp_i2c_bus(),
    });
    if (!ctrl) {
        ESP_LOGE(TAG, "ES8311 控制口创建失败");
        return ESP_FAIL;
    }

    if ((e = i2s_full_duplex_init()) != ESP_OK) return e;

    const audio_codec_data_if_t *data = audio_codec_new_i2s_data(&(audio_codec_i2s_cfg_t){
        .port = BSP_I2S_PORT, .tx_handle = s_tx, .rx_handle = s_rx,
    });
    if (!data) { ESP_LOGE(TAG, "I2S 数据口创建失败"); return ESP_FAIL; }

    const audio_codec_if_t *codec = es8311_codec_new(&(es8311_codec_cfg_t){
        .ctrl_if     = ctrl,
        .gpio_if     = audio_codec_new_gpio(),
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin      = BSP_I2S_PA_CTRL,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk    = true,
        .hw_gain     = { .pa_voltage = 5.0f, .codec_dac_voltage = 3.3f },
        .no_dac_ref  = true,
    });
    if (!codec) { ESP_LOGE(TAG, "es8311_codec_new 失败"); return ESP_FAIL; }

    s_dev = esp_codec_dev_new(&(esp_codec_dev_cfg_t){
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = codec,
        .data_if  = data,
    });
    if (!s_dev) { ESP_LOGE(TAG, "esp_codec_dev_new 失败"); return ESP_FAIL; }

    ESP_LOGI(TAG, "ES8311 就绪");
    return ESP_OK;
}

esp_err_t bsp_audio_set_format(uint32_t hz, uint8_t bits, uint8_t ch) {
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    if (s_opened && s_hz == hz && s_bits == bits && s_ch == ch) return ESP_OK;

    if (s_opened) {
        esp_codec_dev_close(s_dev);
        s_opened = false;
        if (s_tx) i2s_channel_enable(s_tx);
        if (s_rx) i2s_channel_enable(s_rx);
    }

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = bits,
        .channel = ch,
        .channel_mask = ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0),
        .sample_rate = hz,
        .mclk_multiple = 0,
    };
    int r = esp_codec_dev_open(s_dev, &fs);
    if (r != 0) { ESP_LOGE(TAG, "esp_codec_dev_open 失败: %d", r); return ESP_FAIL; }

    esp_codec_dev_set_in_gain(s_dev, 30.0f);

    s_opened = true; s_hz = hz; s_bits = bits; s_ch = ch;
    ESP_LOGI(TAG, "codec 打开 %luHz/%ubit/%uch", (unsigned long)hz, bits, ch);
    return ESP_OK;
}

esp_err_t bsp_audio_write(const void *pcm, size_t bytes) {
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    return esp_codec_dev_write(s_dev, (void *)pcm, bytes) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t bsp_audio_read(void *pcm, size_t bytes) {
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    return esp_codec_dev_read(s_dev, pcm, bytes) == 0 ? ESP_OK : ESP_FAIL;
}

void bsp_audio_set_volume(uint8_t percent) {
    if (s_dev) esp_codec_dev_set_out_vol(s_dev, percent);
}

// C3(ES8311)无 buzzer,空操作。
void bsp_audio_tone(uint32_t freq_hz) { (void)freq_hz; }

#endif  // CONFIG_IDF_TARGET_ESP32
