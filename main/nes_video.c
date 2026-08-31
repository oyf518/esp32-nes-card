// main/nes_video.c
// 帧缓冲裁剪 + 推屏,布局决策见 nes_video.h 顶部注释。
#include "nes_video.h"

#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "bsp_display.h"

static const char *TAG = "nes_video";

static esp_lcd_panel_handle_t s_panel;

// draw_bitmap 是异步 DMA(BSP 的 trans_queue_depth=10),推屏返回后传输仍在途。
// 半帧推屏要复用 s_fb,必须等上一笔 COLOR_TRANS_DONE 才能改写,否则
// 上半片在屏上读到的是被下半片覆盖后的数据(实测:上下两半画面重复)。
static SemaphoreHandle_t s_dma_done;

IRAM_ATTR static bool s_on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                            esp_lcd_panel_io_event_data_t *edata,
                                            void *user_ctx) {
    (void)io; (void)edata; (void)user_ctx;
    BaseType_t awake = pdFALSE;
    xSemaphoreGiveFromISR(s_dma_done, &awake);
    return awake == pdTRUE;
}

// 等待在途的一笔推屏 DMA 完成(协议:每次 draw 前 take 一次)。
// 100ms 超时容错:回调万一丢失/多余,牺牲一片画面并告警,绝不让模拟任务卡死。
static bool wait_trans_done(void) {
    if (xSemaphoreTake(s_dma_done, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "推屏完成回调超时,丢弃本片(不卡死)");
        return false;
    }
    return true;
}

// RGB565 半帧缓冲 240x120x2 = 57.6KB。C3 无 PSRAM,放 SRAM 静态区;
// 一帧拆上下两半转换并推屏(两次 draw_bitmap,总像素不变),省出一半 DRAM ——
// 曾因整帧 115KB 静态区把堆挤到内核 malloc 渲染缓冲失败而首帧崩机。
static uint16_t s_fb[NES_FB_W * NES_FB_SLICE];

// NES 调色板的 RGB565 展开表(osd set_palette 时由 nes_video_set_palette 写入)
static uint16_t s_pal[256];

// 音量 OSD:调整音量后的短时间内,在游戏区最底部画音量条
static int s_osd_volume = 0;
static int s_osd_left = 0;   // 剩余显示的推屏帧数(30fps 计,45 帧 ≈ 1.5s)

void nes_video_osd_volume(int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    s_osd_volume = percent;
    s_osd_left = 45;
}

int nes_video_init(void) {
    s_panel = bsp_display_panel();
    ESP_RETURN_ON_FALSE(s_panel != NULL, -1, TAG, "面板未初始化,请先 bsp_display_init()");

    // 推屏完成同步:每完成一笔 DMA give 一次,每次 draw 前 take 一次
    s_dma_done = xSemaphoreCreateCounting(4, 0);
    ESP_RETURN_ON_FALSE(s_dma_done != NULL, -1, TAG, "信号量创建失败");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = s_on_color_trans_done,
    };
    ESP_RETURN_ON_FALSE(esp_lcd_panel_io_register_event_callbacks(bsp_display_io(), &cbs, NULL) == ESP_OK,
                        -1, TAG, "推屏回调注册失败");

    // 上下黑边只在此清一次,之后每帧只刷中间 240x240。
    // 借用 s_fb 头部一段零数据当全 0 块(s_fb 是 static,此处必为全 0);
    // 黑边 40 行 < 半帧 120 行,头部够用。
    memset(s_fb, 0, sizeof(s_fb));
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, NES_FB_W, NES_LCD_Y, s_fb);               // 上黑边
    esp_lcd_panel_draw_bitmap(s_panel, 0, NES_LCD_Y + NES_FB_H, NES_FB_W, 320, s_fb);  // 下黑边
    wait_trans_done();   // 等两笔黑边 DMA 完成,期间 s_fb 不得改写
    wait_trans_done();
    // 关键:此刻已无在途传输,必须归还一个"空闲令牌",否则首帧 blit 的 take
    // 永远等不到回调 → 模拟任务卡死在第一帧(实测:重启后花屏卡死)。
    xSemaphoreGive(s_dma_done);
    ESP_LOGI(TAG, "推屏就绪:%dx%d @ y=%d(裁剪 x+8,上下两片各 %d 行)",
             NES_FB_W, NES_FB_H, NES_LCD_Y, NES_FB_SLICE);
    return 0;
}

void nes_video_set_palette(const uint16_t *pal565) {
    memcpy(s_pal, pal565, sizeof(s_pal));
}

void nes_video_blit(const bitmap_t *bmp) {
    // 每模拟 2 帧推 1 次屏(30fps);被推掉的帧数据直接丢弃,游戏逻辑不受影响。
    static int s_frame = 0;
    if (++s_frame & 1) {
        return;
    }

    // 8bpp 调色板索引 -> RGB565,逐行裁剪 x in [8,248);
    // 按上下两半转换,转完一片立刻推屏。
    for (int slice = 0; slice < 2; slice++) {
        if (!wait_trans_done()) {
            return;   // 回调超时:本帧弃推,下帧重试
        }
        for (int y = 0; y < NES_FB_SLICE; y++) {
            const uint8_t *src = bmp->line[slice * NES_FB_SLICE + y] + NES_CROP_X;
            uint16_t *dst = &s_fb[y * NES_FB_W];
            for (int x = 0; x < NES_FB_W; x++) {
                dst[x] = s_pal[src[x]];
            }
        }
        // 音量 OSD:画在游戏区最底部 8 行(属于下半片),转换完再覆盖
        if (slice == 1 && s_osd_left > 0) {
            s_osd_left--;
            const int bh = 8;                          // 条高 8 行
            const int y0 = NES_FB_SLICE - bh;
            const int x0 = 10, x1 = NES_FB_W - 10;     // 左右各留 10px
            const int fill_w = x1 - x0 - 2;
            const int fill_end = x0 + 1 + fill_w * s_osd_volume / 100;
            for (int x = x0; x < x1; x++) {
                for (int y = 0; y < bh; y++) {
                    uint16_t c;
                    if (x == x0 || x == x1 - 1 || y == 0 || y == bh - 1) {
                        c = 0xFFFF;                    // 白边框
                    } else {
                        c = (s_osd_volume > 0 && x <= fill_end) ? 0x07E0 : 0x0000;
                    }
                    s_fb[(y0 + y) * NES_FB_W + x] = c;
                }
            }
        }
        // draw_bitmap 的 x/y_end 为开区间:传 x+w / y+h
        if (esp_lcd_panel_draw_bitmap(s_panel, 0, NES_LCD_Y + slice * NES_FB_SLICE,
                                      NES_FB_W, NES_LCD_Y + (slice + 1) * NES_FB_SLICE,
                                      s_fb) != ESP_OK) {
            // 入队失败 = 没有在途传输产生,归还令牌防止令牌数永久亏空
            ESP_LOGW(TAG, "推屏入队失败,归还令牌");
            xSemaphoreGive(s_dma_done);
            return;
        }
    }
    static bool s_first_frame = true;
    if (s_first_frame) {
        s_first_frame = false;
        ESP_LOGI(TAG, "首帧推屏完成,模拟循环运行中");
    }
}

// ---------------------------------------------------------------------------
// 菜单渲染:现代扁平风,全屏 240x320(推屏 3 片:120+120+80 行)。
// 纯色深底,纵向卡片轮转(cover-flow):选中卡片居中放大(圆角 + 纯色橙描边),
// 上下各三级邻近卡片逐级缩小压暗(最远处只露出一截),切换选择时整列滑动
// 过渡(指数平滑)。顶部状态栏:BLE 图标(连接=橙 / 未连接=灰)+ 标题
// 「小小游戏机」+ 电池;底部进度条指示选中项在库中的位置。上下各留 10px 边距。
// 卡片绘制被裁剪在状态栏分隔线与进度条之间,滑动时也不会盖住栏。
// 16x16 点阵(menufont.h,tools/gen_menufont.py 生成,字符集受 romlist 限制,
// 新增 UI 文案只能用字库里已有的字,否则要先改 gen_menufont.py 的 UI_TEXT)。
// rom_menu_run 每帧整页重画(约 30fps,受推屏 DMA 限速),动效按墙钟相位计算。
// ---------------------------------------------------------------------------
#include "menufont.h"
#include "esp_timer.h"
#include "bsp_battery.h"
#include "ble_pad.h"

#define MENU_LCD_H      320        // 菜单全屏高
#define MENU_FOCUS_CY   168        // 选中卡片中心 y
#define MENU_STEP       46         // 相邻卡片中心距
#define MENU_CARD_W     216        // 选中卡片 216x64,圆角 12
#define MENU_CARD_H     64
#define MENU_CARD_R     12
#define MENU_NEAR_W     196        // 邻近卡片(±1)196x38
#define MENU_NEAR_H     38
#define MENU_FAR_W      176        // 次邻近卡片(±2)176x30
#define MENU_FAR_H      30
#define MENU_PEEK_W     160        // 远景卡片(±3)160x26,只露出一截
#define MENU_PEEK_H     26
#define MENU_CLIP_Y     40         // 卡片绘制上边界(状态栏分隔线以下)
#define MENU_CLIP_B     296        // 卡片绘制下边界(进度条以上)
#define MENU_PROG_Y     302        // 底部进度条 y(3px 高)

// 扁平配色(RGB565,纯色无渐变):中性炭灰底 + 琥珀橙点缀。
// 注意:s_fb 里的颜色必须预交换高低字节(ST7789 走 SPI 要大端,
// draw_bitmap 原样发内存数据,与 osd_esp.c 的调色板处理一致)。
#define SWAP565(x) ((uint16_t)((((x) & 0xFF) << 8) | ((x) >> 8)))
#define COL_BG       SWAP565(0x18C3) // 页面底:rgb(26,26,30) 石墨灰,中性不带蓝紫倾向
#define COL_DIV      SWAP565(0x4229) // 分隔线:rgb(70,70,76)
#define COL_ACCENT   SWAP565(0xFCE1) // 点缀色琥珀橙:#FF9F0A
#define COL_TEXT_S   0xFFFF          // 选中卡片文字:白(对称值无需交换)
#define COL_CARD_F   SWAP565(0x3186) // 选中卡底:rgb(48,48,54)
#define COL_TEXT_N1  SWAP565(0xC618) // ±1 文字:浅灰
#define COL_BG_N1    SWAP565(0x2104) // ±1 卡底:rgb(34,34,38)
#define COL_BD_N1    SWAP565(0x4A4A) // ±1 卡描边:rgb(74,74,80)
#define COL_TEXT_N2  SWAP565(0x8410) // ±2 文字:中灰
#define COL_BG_N2    SWAP565(0x18E3) // ±2 卡底:rgb(28,28,32)
#define COL_BD_N2    SWAP565(0x39E7) // ±2 卡描边:rgb(56,56,62)
#define COL_TEXT_N3  SWAP565(0x630C) // ±3 文字:暗灰
#define COL_BG_N3    SWAP565(0x18C3) // ±3 卡底:rgb(24,24,28)
#define COL_BD_N3    SWAP565(0x2966) // ±3 卡描边:rgb(44,44,50)
#define COL_ICON     SWAP565(0x9CF3) // 状态栏图标浅灰
#define COL_BATT_G   SWAP565(0x2E4B) // 电量 >50% 绿
#define COL_BATT_Y   SWAP565(0xFD62) // 20~50% 黄
#define COL_BATT_R   SWAP565(0xEA88) // <20% 红
#define COL_BLE_OFF  SWAP565(0x630C) // BLE 未连接:暗灰
#define COL_PROG_TK  SWAP565(0x31A7) // 进度条轨道:rgb(52,52,58)

// BLE 图标:蓝牙如尼文 12x16(1bpp,MSB-first)
static const uint16_t k_ble_glyph[16] = {
    0x0600, 0x0500, 0x8480, 0x4440, 0x2420, 0x1440, 0x0D00, 0x0600,
    0x0600, 0x0D00, 0x1440, 0x2420, 0x4440, 0x8480, 0x0500, 0x0600,
};

// 卡片绘制裁剪窗口;画卡片时设为 [MENU_CLIP_Y, MENU_CLIP_B)
static int s_clip_y, s_clip_b = MENU_LCD_H;

static int isqrt(int v) {
    int r = 0;
    while ((r + 1) * (r + 1) <= v) r++;
    return r;
}

// 圆角矩形第 dy 行的左右缩进(圆角区按圆计算,直边区为 0)
static int rounded_inset(int dy, int h, int R) {
    int q = dy < R ? R - 1 - dy : (dy >= h - R ? dy - (h - R) : -1);
    if (q < 0) return 0;
    return R - isqrt(R * R - q * q);
}

// utf8 字符数(1 字节 ASCII / 3 字节 CJK)
static int utf8_len(const char *s) {
    int n = 0;
    for (; *s; n++) s += ((uint8_t)*s >= 0xE0) ? 3 : 1;
    return n;
}

// utf8 解码,返回字形索引,-1 = 字库没有
static int menu_glyph_find(uint32_t ucs) {
    for (int i = 0; i < MENU_GLYPH_COUNT; i++) {
        if (k_menu_glyphs[i].ucs == ucs) return i;
    }
    return -1;
}

static inline void mput(uint16_t *fb, int slice, int ys, int xs, uint16_t c) {
    if (ys >= s_clip_y && ys < s_clip_b &&
        ys >= slice * NES_FB_SLICE && ys < (slice + 1) * NES_FB_SLICE &&
        (unsigned)xs < NES_FB_W) {
        fb[(ys - slice * NES_FB_SLICE) * NES_FB_W + xs] = c;
    }
}

// 透明底文字:只画笔画,卡底透出来
static void mtext(uint16_t *fb, int slice, int x0, int y0, const char *txt, uint16_t fg) {
    int cx = x0;
    for (const char *p = txt; *p && cx + 16 <= NES_FB_W; ) {
        uint32_t ucs = (uint8_t)*p;
        int len = 1;
        if (ucs >= 0xE0) {
            ucs = ((ucs & 0x0F) << 12) | (((uint8_t)p[1] & 0x3F) << 6) |
                  ((uint8_t)p[2] & 0x3F);
            len = 3;
        }
        p += len;
        int gi = menu_glyph_find(ucs);
        if (gi >= 0) {
            for (int gy = 0; gy < 16; gy++) {
                uint16_t bits = k_menu_glyphs[gi].glyph[gy];
                for (int gx = 0; gx < 16; gx++)
                    if (bits & (0x8000 >> gx)) mput(fb, slice, y0 + gy, cx + gx, fg);
            }
        }
        cx += 16;
    }
}

// 选中卡片:纯色底 + 2px 主色描边(圆角)
static void draw_card_focus(uint16_t *fb, int slice, int cx, int cy) {
    const int w = MENU_CARD_W, h = MENU_CARD_H, R = MENU_CARD_R;
    const int x0 = cx - w / 2, y0 = cy - h / 2;
    for (int dy = 0; dy < h; dy++) {
        int ins = rounded_inset(dy, h, R);
        if (dy < 2 || dy >= h - 2) {
            for (int x = ins; x < w - ins; x++) mput(fb, slice, y0 + dy, x0 + x, COL_ACCENT);
        } else {
            for (int t = 0; t < 2; t++) {
                mput(fb, slice, y0 + dy, x0 + ins + t, COL_ACCENT);
                mput(fb, slice, y0 + dy, x0 + w - 1 - ins - t, COL_ACCENT);
            }
            for (int x = ins + 2; x < w - ins - 2; x++) mput(fb, slice, y0 + dy, x0 + x, COL_CARD_F);
        }
    }
}

// 邻近卡片:纯色圆角底 + 1px 暗描边
static void draw_card_plain(uint16_t *fb, int slice, int cx, int cy,
                            int w, int h, int R, uint16_t bg, uint16_t bd) {
    const int x0 = cx - w / 2, y0 = cy - h / 2;
    for (int dy = 0; dy < h; dy++) {
        int ins = rounded_inset(dy, h, R);
        if (dy == 0 || dy == h - 1) {
            for (int x = ins; x < w - ins; x++) mput(fb, slice, y0 + dy, x0 + x, bd);
        } else {
            for (int x = ins; x < w - ins; x++) mput(fb, slice, y0 + dy, x0 + x, bg);
            mput(fb, slice, y0 + dy, x0 + ins, bd);
            mput(fb, slice, y0 + dy, x0 + w - 1 - ins, bd);
        }
    }
}

// 状态栏:BLE 图标(已连接 = 橙;未连接 = 暗灰,不闪烁)
static void draw_ble(uint16_t *fb, int slice) {
    uint16_t c = ble_pad_connected() ? COL_ACCENT : COL_BLE_OFF;
    for (int gy = 0; gy < 16; gy++)
        for (int gx = 0; gx < 12; gx++)
            if (k_ble_glyph[gy] & (0x8000 >> gx)) mput(fb, slice, 15 + gy, 10 + gx, c);
}

// 状态栏:电池图标(壳 + 电极 + 按电量填充;读不到画空壳)
static void draw_battery(uint16_t *fb, int slice, int soc) {
    const int x0 = 208, y0 = 18;     // 壳 20x11,电极 2x5
    for (int x = x0; x < x0 + 20; x++) {
        mput(fb, slice, y0, x, COL_ICON);
        mput(fb, slice, y0 + 10, x, COL_ICON);
    }
    for (int y = y0; y < y0 + 11; y++) {
        mput(fb, slice, y, x0, COL_ICON);
        mput(fb, slice, y, x0 + 19, COL_ICON);
    }
    for (int y = y0 + 3; y < y0 + 8; y++) {
        mput(fb, slice, y, x0 + 20, COL_ICON);
        mput(fb, slice, y, x0 + 21, COL_ICON);
    }
    if (soc < 0) return;
    uint16_t c = soc > 50 ? COL_BATT_G : soc > 20 ? COL_BATT_Y : COL_BATT_R;
    int fw = 16 * soc / 100;
    for (int y = y0 + 2; y < y0 + 9; y++)
        for (int x = x0 + 2; x < x0 + 2 + fw; x++)
            mput(fb, slice, y, x, c);
}

// 菜单全屏绘制过上下边区;进游戏前调本函数把 y<40 / y>=280 清回黑色。
void nes_video_clear_edges(void) {
    memset(s_fb, 0, sizeof(s_fb));
    if (wait_trans_done())
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, NES_FB_W, NES_LCD_Y, s_fb);
    if (wait_trans_done())
        esp_lcd_panel_draw_bitmap(s_panel, 0, NES_LCD_Y + NES_FB_H, NES_FB_W, MENU_LCD_H, s_fb);
}

void nes_video_menu_draw(const char *const *names, int count, int sel) {
    uint16_t *fb = s_fb;
    uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);

    // 视觉位置(×256 定点):朝选中项指数平滑,回绕(距离超半库)时直接跳变
    static int s_pos_fp = -1;
    int target = sel << 8;
    if (s_pos_fp < 0) s_pos_fp = target;
    int diff = target - s_pos_fp;
    if (diff > count * 128 || diff < -count * 128) {
        s_pos_fp = target;
    } else if (diff != 0) {
        int d4 = diff / 4;
        s_pos_fp += d4 ? d4 : (diff > 0 ? 1 : -1);
    }

    // 电量:I2C 读取按 2s 节流缓存
    static int s_soc = -1;
    static uint32_t s_soc_ms = 0;
    if (ms - s_soc_ms > 2000) {
        s_soc_ms = ms;
        s_soc = bsp_battery_soc();
    }

    for (int slice = 0; slice < 3; slice++) {
        if (!wait_trans_done()) return;
        int y0 = slice * NES_FB_SLICE;
        int rows = MENU_LCD_H - y0 < NES_FB_SLICE ? MENU_LCD_H - y0 : NES_FB_SLICE;

        // 1) 背景:纯色
        for (int i = 0; i < NES_FB_W * NES_FB_SLICE; i++) fb[i] = COL_BG;

        // 2) 状态栏一行:BLE + 标题 + 电池,下沿分隔线
        s_clip_y = 0;
        s_clip_b = MENU_LCD_H;
        draw_ble(fb, slice);
        mtext(fb, slice, (NES_FB_W - 5 * 16) / 2, 15, "小小游戏机", 0xFFFF);
        draw_battery(fb, slice, s_soc);
        for (int x = 10; x < NES_FB_W - 10; x++) mput(fb, slice, 36, x, COL_DIV);

        // 3) 卡片:远 → 近 → 选中,逐层叠画;裁剪在两栏之间
        s_clip_y = MENU_CLIP_Y;
        s_clip_b = MENU_CLIP_B;
        for (int pass = 3; pass >= 0; pass--) {
            for (int i = 0; i < count; i++) {
                int dfp = (i << 8) - s_pos_fp;
                int adf = dfp < 0 ? -dfp : dfp;
                int depth = adf < 128 ? 0 : (adf < 384 ? 1 : (adf < 640 ? 2 : 3));
                if (depth != pass) continue;
                int cy = MENU_FOCUS_CY + dfp * MENU_STEP / 256;
                int ch = depth == 0 ? MENU_CARD_H :
                         depth == 1 ? MENU_NEAR_H :
                         depth == 2 ? MENU_FAR_H : MENU_PEEK_H;
                if (cy + ch / 2 < MENU_CLIP_Y || cy - ch / 2 > MENU_CLIP_B) continue;
                if (depth == 0) draw_card_focus(fb, slice, NES_FB_W / 2, cy);
                else if (depth == 1)
                    draw_card_plain(fb, slice, NES_FB_W / 2, cy,
                                    MENU_NEAR_W, MENU_NEAR_H, 10, COL_BG_N1, COL_BD_N1);
                else if (depth == 2)
                    draw_card_plain(fb, slice, NES_FB_W / 2, cy,
                                    MENU_FAR_W, MENU_FAR_H, 8, COL_BG_N2, COL_BD_N2);
                else
                    draw_card_plain(fb, slice, NES_FB_W / 2, cy,
                                    MENU_PEEK_W, MENU_PEEK_H, 8, COL_BG_N3, COL_BD_N3);
                int tw = utf8_len(names[i]) * 16;
                mtext(fb, slice, (NES_FB_W - tw) / 2, cy - 8, names[i],
                      depth == 0 ? COL_TEXT_S :
                      depth == 1 ? COL_TEXT_N1 :
                      depth == 2 ? COL_TEXT_N2 : COL_TEXT_N3);
            }
        }
        s_clip_y = 0;
        s_clip_b = MENU_LCD_H;

        // 4) 底部进度条:轨道 + 选中位置(随滑动平滑移动)
        int pw = (s_pos_fp + 256) * 220 / (count << 8);   // 0..220
        if (pw > 220) pw = 220;
        for (int y = MENU_PROG_Y; y < MENU_PROG_Y + 3; y++)
            for (int x = 10; x < NES_FB_W - 10; x++)
                mput(fb, slice, y, x, x - 10 < pw ? COL_ACCENT : COL_PROG_TK);

        if (esp_lcd_panel_draw_bitmap(s_panel, 0, y0, NES_FB_W, y0 + rows,
                                      fb) != ESP_OK) {
            xSemaphoreGive(s_dma_done);
            break;
        }
    }
}
