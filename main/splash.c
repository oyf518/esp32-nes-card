// main/splash.c
// 开机画面:splash.bin(240x320 RGB565,生成见 tools/gen_splash.py)逐带推显,
// 背光 PWM 淡入/淡出。不做逐像素渐变:一帧 150KB@40MHz SPI 约 25ms,
// 逐帧混合跑不出流畅动画,背光调光是 C3 上唯一流畅的淡出方式。
#include "splash.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "bsp_display.h"
#include "nes_video.h"   // nes_video_scratch:借用半帧缓冲作推屏暂存
#include "music.h"       // 开屏音效

// EMBED_FILES 嵌入的 splash.bin(main/CMakeLists.txt)
extern const uint8_t splash_start[] asm("_binary_splash_bin_start");
extern const uint8_t splash_end[]   asm("_binary_splash_bin_end");

static const char *TAG = "splash";

#define SPLASH_W 240
#define SPLASH_H 320
// 整帧 150KB 直接从 flash rodata 推屏,驱动要为非 DMA 地址现分配私有中转缓冲,
// 开机早期堆分配不到 → 推屏半途失败、GRAM 下半屏残影花屏(实测)。
// 改为:借用 nes_video 的半帧缓冲(nes_video_scratch,静态 SRAM、DMA-capable),
// 逐带 memcpy + 推屏。开机阶段 SPI 独占,一带 19.2KB@40MHz 约 4ms,
// 固定延时 15ms 等 DMA 读完后安全覆写,无需回调同步;也不新增常驻内存。
#define SPLASH_BAND_ROWS 40   // 每带 40 行 = 19.2KB,单笔 SPI 传输内(max_transfer_sz=38.4KB)

// 开屏音效:原创 C 大调上行琶音"锵!"——三连音爬升 + 高八度定格 + 低八度回声,
// 约 0.75s,跟淡入同时响起,惊喜感靠"快 + 亮"。MIDI 音高:C6=84 E6=88 G6=91 C7=96。
static const melody_note_t k_jingle_lead[] = {
    {   0,  90, 84 }, { 100,  90, 88 }, { 200,  90, 91 }, { 300, 400, 96 },
};
static const melody_note_t k_jingle_echo[] = {
    { 100,  90, 72 }, { 200,  90, 76 }, { 300,  90, 79 }, { 400, 300, 84 },
};
static const uint16_t k_jingle_lens[2] = {
    sizeof(k_jingle_lead) / sizeof(k_jingle_lead[0]),
    sizeof(k_jingle_echo) / sizeof(k_jingle_echo[0]),
};
static const melody_note_t *const k_jingle_voices[2] = { k_jingle_lead, k_jingle_echo };
static const tune_t k_splash_jingle = {
    .voices = k_jingle_voices, .lens = k_jingle_lens,
    .voice_count = 2, .total_ms = 750,
};

void splash_show(int hold_ms) {
    const size_t need = SPLASH_W * SPLASH_H * 2;
    if ((size_t)(splash_end - splash_start) != need) {
        ESP_LOGE(TAG, "splash.bin 尺寸 %u != 预期 %u(重跑 tools/gen_splash.py)",
                 (unsigned)(splash_end - splash_start), (unsigned)need);
        return;
    }
    esp_lcd_panel_handle_t panel = bsp_display_panel();
    if (NULL == panel) {
        ESP_LOGE(TAG, "面板未初始化");
        return;
    }

    // 背光先压到 0 再推屏,避免上电残帧闪现(推屏为异步 DMA,~25ms 完成)
    bsp_display_backlight(0);
    uint16_t *band = nes_video_scratch();   // 240x120 像素,足够 40 行一带
    for (int y = 0; y < SPLASH_H; y += SPLASH_BAND_ROWS) {
        memcpy(band, splash_start + (size_t)y * SPLASH_W * 2,
               (size_t)SPLASH_W * SPLASH_BAND_ROWS * 2);
        esp_lcd_panel_draw_bitmap(panel, 0, y, SPLASH_W, y + SPLASH_BAND_ROWS, band);
        vTaskDelay(pdMS_TO_TICKS(15));   // 等本带 DMA 读完再覆写缓冲
    }

    // 音效跟淡入一起响(推屏阶段屏幕还黑着,响早了没有惊喜感)
    music_play_once(&k_splash_jingle);
    // 淡入 ~150ms
    for (int p = 0; p <= 100; p += 25) {
        bsp_display_backlight((uint8_t)p);
        vTaskDelay(pdMS_TO_TICKS(35));
    }
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    // 淡出 ~500ms
    for (int p = 100; p >= 0; p -= 10) {
        bsp_display_backlight((uint8_t)p);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
