// main/osd_esp.c
// nofrendo 的 osd_* 回调实现(视频/音频/输入/日志)+ 内核引用的 GUI 桩。
//
// 头文件包含顺序敏感:先引 ESP-IDF/标准头(其中 stdbool 把 bool/true/false 定义成宏),
// 再 undef 掉这三件套,最后才引 nofrendo 头 —— noftypes.h 是 C89 风格,
// 自己 typedef enum {false,true} bool,与 stdbool 宏直接冲突(esp32-nesemu 原版同款处理)。
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "bsp_audio.h"
#include "bsp_button.h"
#include "bsp_pins.h"


#include <noftypes.h>
#include <nofrendo.h>     // nofrendo_ticks extern 声明
#include <bitmap.h>
#include <vid_drv.h>
#include <osd.h>
#include <gui.h>          // GUI_* 颜色枚举 + 桩函数原型
#include <log.h>
#include <nes.h>
#include <nesinput.h>

#include "nes_video.h"
#include "nes_pad_map.h"
#include "ble_pad.h"        // BLE 手柄输入;只引 stdint/stdbool/esp_err,无 bool 冲突

static const char *TAG = "osd";

#define NES_SAMPLE_RATE   22050
// 每帧采样数 = 22050/60 = 367.5,APU 内部对小数部分有缓冲,这里每帧取整 367
// (音高偏差 0.1% 量级,听不出;节拍由 bsp_audio_write 阻塞兜底)。
#define NES_FRAG_SIZE     (NES_SAMPLE_RATE / NES_REFRESH_RATE)   // 367

// nofrendo.c(原版 main)被裁掉了,它定义的帧节拍计数器由这里补上。
// 由 osd_installtimer 装的 60Hz esp_timer 递增,nes_emulate() 靠它决定何时渲染。
volatile int nofrendo_ticks = 0;

// ---------------------------------------------------------------------------
// 日志:内核 log_printf -> ESP_LOG
// ---------------------------------------------------------------------------
static int logprint(const char *string) {
    // 内核日志多为状态/报错,量不大,直出即可
    return printf("%s", string);
}

// ---------------------------------------------------------------------------
// 音频:osd_setsound 拿到 APU 回调,帧末(custom_blit)统一生成一帧采样并阻塞写。
// bsp_audio_write 阻塞到 I2S 有空间,天然把模拟器限到 60fps。
// ---------------------------------------------------------------------------
static void (*audio_callback)(void *buffer, int size) = NULL;
static int16_t s_audio_frame[NES_FRAG_SIZE + 2];   // 367/368 交替,留 2 采样余量
static float s_audio_carry = 0.0f;

void osd_setsound(void (*playfunc)(void *buffer, int size)) {
    audio_callback = playfunc;
}

void osd_getsoundinfo(sndinfo_t *info) {
    info->sample_rate = NES_SAMPLE_RATE;
    info->bps = 16;
}

static void do_audio_frame(void) {
    if (NULL == audio_callback) {
        return;
    }

    // 每帧精确产 22050/60 = 367.5 个采样:367/368 交替。
    // 恒写 367 = 22020 采样/秒,比 I2S 消耗(22050)每秒亏 30 个,
    // DMA 周期性断流 → 持续"破音"。余数累积保证平均速率精确一致。
    s_audio_carry += (float)NES_SAMPLE_RATE / (float)NES_REFRESH_RATE;
    int n = (int)s_audio_carry;
    s_audio_carry -= (float)n;
    audio_callback(s_audio_frame, n);
    bsp_audio_write(s_audio_frame, n * sizeof(int16_t));

}

// 供内核(nes.c)跳帧分支调用:跳过推屏的帧也必须产音,否则掉帧时 I2S 断流
void osd_audio_frame(void) {
    do_audio_frame();
}

// ---------------------------------------------------------------------------
// 视频:viddriver_t 实现。内核渲染进 vid_drv 的 8bpp 主缓冲,
// 帧末 vid_flush() -> custom_blit 交给我们裁剪推屏 + 出音频。
// ---------------------------------------------------------------------------
static int  vid_init_stub(int width, int height);
static void vid_shutdown_stub(void);
static int  vid_set_mode_stub(int width, int height);
static void vid_set_palette(rgb_t *pal);
static void vid_clear(uint8 color);
static bitmap_t *vid_lock_write(void);
static void vid_free_write(int num_dirties, rect_t *dirty_rects);
static void vid_custom_blit(bitmap_t *bmp, int num_dirties, rect_t *dirty_rects);

static viddriver_t s_esp_driver = {
    "esp32c3-st7789",
    vid_init_stub,
    vid_shutdown_stub,
    vid_set_mode_stub,
    vid_set_palette,
    vid_clear,
    vid_lock_write,
    vid_free_write,
    vid_custom_blit,
    false,               // invalidate
};

// vid_drv 的 vid_findmode 会对 lock_write 返回的"屏幕"做日志读取;
// 我们提供了 clear(),它不会真的往这块缓冲写数据,故给个 1 字节假缓冲即可。
static uint8_t s_screen_dummy[1];
static bitmap_t *s_screen_bmp;

void osd_getvideoinfo(vidinfo_t *info) {
    info->default_width = NES_SCREEN_WIDTH;
    info->default_height = NES_VISIBLE_HEIGHT;
    info->driver = &s_esp_driver;
}

static int vid_init_stub(int width, int height) {
    (void)width; (void)height;
    return 0;
}

static void vid_shutdown_stub(void) {
}

static int vid_set_mode_stub(int width, int height) {
    (void)width; (void)height;
    return 0;
}

static void vid_set_palette(rgb_t *pal) {
    // rgb_t -> RGB565,并预交换高低字节:
    // ST7789 走 SPI 要求大端,而 esp_lcd_panel_draw_bitmap 原样发内存数据
    // (BSP 的 LVGL 路径同理用 swap_bytes=true)。
    static uint16_t pal565[256];
    for (int i = 0; i < 256; i++) {
        uint16_t c = (pal[i].b >> 3) | ((pal[i].g >> 2) << 5) | ((pal[i].r >> 3) << 11);
        pal565[i] = (c >> 8) | (c << 8);
    }
    nes_video_set_palette(pal565);
}

static void vid_clear(uint8 color) {
    (void)color;   // 清屏由 nes_video_init 做过;游戏区每帧全覆盖,无需再清
}

static bitmap_t *vid_lock_write(void) {
    if (NULL == s_screen_bmp) {
        s_screen_bmp = bmp_createhw(s_screen_dummy, NES_SCREEN_WIDTH,
                                    NES_VISIBLE_HEIGHT, NES_SCREEN_WIDTH);
    }
    return s_screen_bmp;
}

static void vid_free_write(int num_dirties, rect_t *dirty_rects) {
    (void)num_dirties; (void)dirty_rects;
}

static void vid_custom_blit(bitmap_t *bmp, int num_dirties, rect_t *dirty_rects) {
    (void)num_dirties; (void)dirty_rects;
    nes_video_blit(bmp);   // 内部 2:1 抽帧
    do_audio_frame();      // 紧跟一帧音频,阻塞写充当帧节拍
}

// ---------------------------------------------------------------------------
// 音量:UP/DOWN 直接作音量键(不再映射游戏输入,方向靠 BLE 手柄)。
// 按下沿立即调一档,按住 0.4s 后连发;屏幕底部显示音量条。
// 音量存 NVS(ble_pad_init 已确保 NVS 就绪),重启保留。
// ---------------------------------------------------------------------------
#define VOL_REPEAT_DELAY_US 400000   // 按住多久后开始连发
#define VOL_REPEAT_US       180000   // 连发间隔
#define VOL_STEP            10
#define VOL_DEFAULT         70

static int s_volume = VOL_DEFAULT;
static bool s_vol_pressed[2] = {false};   // UP/DOWN 当前按住状态
static int64_t s_vol_next_us = 0;         // 下次连发时刻
static bool s_vol_dirty = false;          // 调整过但未落盘

static void volume_apply(void) {
    if (s_volume < 0) s_volume = 0;
    if (s_volume > 100) s_volume = 100;
    bsp_audio_set_volume((uint8_t)s_volume);
    nes_video_osd_volume(s_volume);
}

// 松开按键时落盘一次,避免连发过程中频繁写 flash
static void volume_save(void) {
    nvs_handle_t h;
    if (nvs_open("nes", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "volume", (uint8_t)s_volume);
        nvs_commit(h);
        nvs_close(h);
    }
}

// app_main 在 ble_pad_init 之后调用(NVS 已就绪,codec 已打开)
void osd_volume_init(void) {
    nvs_handle_t h;
    if (nvs_open("nes", NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = VOL_DEFAULT;
        if (nvs_get_u8(h, "volume", &v) == ESP_OK && v <= 100) s_volume = v;
        nvs_close(h);
    }
    volume_apply();
}

// ---------------------------------------------------------------------------
// 输入:轮询 ADC 电压自维护按键状态机。
// 不用 bsp 的 iot_button 事件(它没有 release 事件,不适合模拟器按住移动)。
// ---------------------------------------------------------------------------
static nesinput_t s_pad = { INP_JOYPAD0, 0 };

void osd_getinput(void) {
    int mv = bsp_button_read_mv();
    int btn = -1;
    if (mv >= 0) {
        // 电压窗口与 bsp_pins.h 的 BSP_BTN_MV_TABLE 一致:{0,150}=UP、
        // {150,447}=DOWN、{447,1900}=OK,松开≈3300 不命中任何窗口。
        static const uint16_t WIN[BSP_BTN_COUNT][2] = BSP_BTN_MV_TABLE;
        for (int i = 0; i < BSP_BTN_COUNT; i++) {
            if (mv >= WIN[i][0] && mv < WIN[i][1]) {
                btn = i;
                break;
            }
        }
    }

    // UP/DOWN = 音量键:按下沿立即调一档,按住连发;松开时 NVS 落盘一次。
    int64_t now_us = esp_timer_get_time();
    for (int i = 0; i < 2; i++) {   // 0=UP(+音量), 1=DOWN(-音量)
        if (btn != i) {
            if (s_vol_pressed[i] && s_vol_dirty) {
                volume_save();
                s_vol_dirty = false;
            }
            s_vol_pressed[i] = false;
            continue;
        }
        if (!s_vol_pressed[i]) {
            s_vol_pressed[i] = true;
            s_volume += (i == BSP_BTN_UP) ? VOL_STEP : -VOL_STEP;
            volume_apply();
            s_vol_dirty = true;
            s_vol_next_us = now_us + VOL_REPEAT_DELAY_US;
        } else if (now_us >= s_vol_next_us) {
            s_volume += (i == BSP_BTN_UP) ? VOL_STEP : -VOL_STEP;
            volume_apply();
            s_vol_dirty = true;
            s_vol_next_us = now_us + VOL_REPEAT_US;
        }
    }

    // 双击 OK -> Start:两次按下沿间隔 <400ms 即认为本次按下是 Start。
    // 标题画面普遍要 Start 才能进游戏,三键没有富余键位,只能借双击。
    // 代价:快速连打 A 会被误判成 Start,横版游戏里跳跃按不到这么快,可接受。
    static int s_prev_btn = -1;
    static int64_t s_last_ok_press_us = 0;
    static bool s_ok_as_start = false;
    if (btn == BSP_BTN_OK && s_prev_btn != BSP_BTN_OK) {
        int64_t now = esp_timer_get_time();
        s_ok_as_start = (now - s_last_ok_press_us) < 400000;
        s_last_ok_press_us = now;
    }
    s_prev_btn = btn;

    // 任一时刻只可能按住一个键(分压硬件决定),状态直接整字节重建。
    // (UP/DOWN 已改作音量键,NES_PAD_MAP 中映射为 0;物理游戏输入只剩 OK)
    if (btn == BSP_BTN_OK && s_ok_as_start) {
        s_pad.data = INP_PAD_START;
    } else {
        s_pad.data = (btn >= 0) ? NES_PAD_MAP[btn] : 0;
    }

    // BLE 手柄输入与三键取或:位掩码互补,谁按都生效;未连接时 ble_pad_state()=0
    uint8_t pad_bits = ble_pad_state();

    // SELECT = 返回菜单:长按约 0.8 秒触发热切换退出(等同长按 OK),短按无效果。
    // SELECT 位始终不传给游戏(需要游戏内 SELECT 功能时改回即可)。
    static int s_sel_held = 0;
    if (pad_bits & 0x04) {                   // NES 位:SELECT=0x04
        s_sel_held++;
        if (s_sel_held == 50) {              // 50 帧 @60Hz ≈ 0.83s,== 保证只触发一次
            ESP_LOGI(TAG, "长按 SELECT,返回菜单");
            nes_poweroff();
        }
    } else {
        s_sel_held = 0;
    }
    s_pad.data |= pad_bits & ~0x04;          // SELECT 位不进游戏输入
}

void osd_getmouse(int *x, int *y, int *button) {
    (void)x; (void)y; (void)button;   // 无光枪/鼠标
}

// ---------------------------------------------------------------------------
// 定时器:60Hz 递增 nofrendo_ticks(nes_emulate 的帧节拍源)。
// ---------------------------------------------------------------------------
// nes_emulate() 主循环是纯忙等(等 nofrendo_ticks 变化),prio 5 下会饿死 idle
// 任务触发 TWDT、连累按键组件的定时器任务。内核里打了补丁每轮调一次本函数,
// 让出 1 个 tick(1ms)给低优先级任务;模拟器追不上帧时损失约 6% CPU,可接受。
void osd_yield(void) {
    vTaskDelay(1);
}


int osd_installtimer(int frequency, void *func, int funcsize, void *counter, int countersize) {
    (void)funcsize; (void)counter; (void)countersize;
    const esp_timer_create_args_t tcfg = {
        .callback = (esp_timer_cb_t)func,
        .name = "nes_tick",
    };
    esp_timer_handle_t t;
    if (esp_timer_create(&tcfg, &t) != ESP_OK) {
        return -1;
    }
    return esp_timer_start_periodic(t, 1000000 / frequency);
}

// ---------------------------------------------------------------------------
// 文件名/存档接口:ROM 从嵌入符号读,无存档功能,全部给最小实现。
// ---------------------------------------------------------------------------
void osd_fullname(char *fullname, const char *shortname) {
    strncpy(fullname, shortname, PATH_MAX);
}

char *osd_newextension(char *string, char *ext) {
    (void)ext;
    return string;   // 电池存档不落盘
}

int osd_makesnapname(char *filename, int len) {
    (void)filename; (void)len;
    return -1;       // 不做 PCX 截图
}

// ---------------------------------------------------------------------------
// 关机/启动
// ---------------------------------------------------------------------------
void osd_shutdown(void) {
    audio_callback = NULL;
}

int osd_init(void) {
    log_chain_logfunc(logprint);
    // 手柄注册进内核输入表(原桌面版由 event.c 代劳,我们裁掉了 event 系统)
    input_register(&s_pad);
    return 0;
}

// ---------------------------------------------------------------------------
// GUI 桩:内核(nes.c/nes_rom.c/nesstate.c/nes_ppu.c)引用这些符号。
// 桌面版 gui.c 已裁掉,这里给最小实现:消息转日志,帧叠加什么都不画。
// ---------------------------------------------------------------------------
rgb_t gui_pal[GUI_TOTALCOLORS];   // nes_ppu.c 会把它拷进调色板 192+ 项,全 0 即可

void gui_tick(int ticks) {
    (void)ticks;
}

void gui_frame(bool draw) {
    (void)draw;
}

void gui_sendmsg(int color, char *format, ...) {
    (void)color;
    char buf[256];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
    ESP_LOGI(TAG, "%s", buf);
}
