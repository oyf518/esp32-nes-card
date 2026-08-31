// main/splash.c
// 开机画面:splash.bin(240x320 RGB565,生成见 tools/gen_splash.py)整屏推显,
// 背光 PWM 淡入/淡出。不做逐像素渐变:一帧 150KB@40MHz SPI 约 25ms,
// 逐帧混合跑不出流畅动画,背光调光是 C3 上唯一流畅的淡出方式。
#include "splash.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "bsp_display.h"

// EMBED_FILES 嵌入的 splash.bin(main/CMakeLists.txt)
extern const uint8_t splash_start[] asm("_binary_splash_bin_start");
extern const uint8_t splash_end[]   asm("_binary_splash_bin_end");

static const char *TAG = "splash";

#define SPLASH_W 240
#define SPLASH_H 320

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
    esp_lcd_panel_draw_bitmap(panel, 0, 0, SPLASH_W, SPLASH_H, splash_start);

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
