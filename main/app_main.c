// main/app_main.c
// 入口:初始化 BSP(显示/音频/按键)、ROM 库菜单、起模拟器任务。
//
// nofrendo 原版 main(nofrendo.c)已裁掉,这里按它 internal_insert() 的顺序直接驱动:
//   osd_init -> vid_init -> nes_create -> nes_insertcart -> vid_setmode -> 装 60Hz 定时器 -> nes_emulate()
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "bsp_display.h"
#include "bsp_audio.h"
#include "bsp_button.h"
#include "bsp_battery.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

// 同 osd_esp.c:nofrendo 头与 stdbool 冲突,先引 ESP 头再 undef 三件套

#include <noftypes.h>
#include <osd.h>
#include <vid_drv.h>
#include <nes.h>

#include "nes_video.h"
#include "ble_pad.h"
#include "rom_menu.h"
#include "splash.h"

static const char *TAG = "app";

// ROM 来源:storage 分区 ROM 库(rom_menu.c),菜单选中后 mmap 到此。
// osd_getromdata 保留(内核 nes_rom.c 的取数口),实现转调 rombank_ptr。
char *osd_getromdata(void) {
    uint32_t sz = 0;
    const void *p = rombank_ptr(&sz);
    ESP_LOGI(TAG, "ROM: %u bytes @ %p", (unsigned)sz, p);
    return (char *)p;
}

// 帧节拍:60Hz 递增 nofrendo_ticks(nes_emulate 的主时钟)
static void timer_isr(void) {
    extern volatile int nofrendo_ticks;
    nofrendo_ticks++;
}

// 按键事件回调:轮询负责游戏输入(见 osd_getinput),这里只处理长按 OK 退出。
static void on_button(bsp_btn_t btn, bsp_btn_ev_t ev, void *user) {
    (void)user;
    if (btn == BSP_BTN_OK && ev == BSP_BTN_LONG) {
        ESP_LOGW(TAG, "长按 OK,退出游戏回菜单(热切换,不断开手柄)");
        nes_poweroff();   // 内核公开 API:置 poweroff 标志,nes_emulate 循环退出
    }
}

extern void osd_volume_init(void);   // osd_esp.c:音量 NVS 恢复 + 生效

static void emu_task(void *arg) {
    (void)arg;
    vidinfo_t video;

    // ROM 库 + 选择菜单(机身三键 / BLE 手柄,菜单内音量键只用于选择)
    if (rombank_init() <= 0) {
        ESP_LOGE(TAG, "ROM 库为空:请烧录 rompack.bin 到 0x190000");
        // 背光点亮到错误状态(残显开屏画面),否则一直黑屏像死机
        bsp_display_backlight(80);
        vTaskDelete(NULL);
    }

    osd_init();

    osd_getvideoinfo(&video);
    if (vid_init(video.default_width, video.default_height, video.driver)) {
        ESP_LOGE(TAG, "vid_init 失败");
        vTaskDelete(NULL);
    }

    if (osd_installtimer(NES_REFRESH_RATE, (void *)timer_isr, 0, NULL, 0)) {
        ESP_LOGE(TAG, "定时器安装失败");
        vTaskDelete(NULL);
    }

    // 菜单 <-> 游戏 热切换循环:游戏内长按 OK -> nes_poweroff() -> nes_emulate
    // 返回 -> 销毁本局 -> 回菜单。全程不走重启,BLE 手柄连接原封不动。
    for (;;) {
        ESP_LOGI(TAG, "heap: free=%lu largest8bit=%lu min_free=%lu",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                 (unsigned long)esp_get_minimum_free_heap_size());
        int sel = rom_menu_run();
        // 菜单 BGM 把 I2S 切到了 16kHz,进游戏前恢复 NES 音频格式
        ESP_ERROR_CHECK(bsp_audio_set_format(22050, 16, 1));
        nes_video_clear_edges();   // 菜单是全屏绘制,清掉游戏区上下边带的菜单残留
        if (rombank_open(sel) != 0) {
            ESP_LOGE(TAG, "打开游戏 %d 失败,回菜单", sel);
            continue;
        }

        nes_t *machine = nes_create();
        if (NULL == machine) {
            ESP_LOGE(TAG, "nes_create 失败,回菜单");
            continue;
        }

        if (nes_insertcart("rom", machine)) {
            ESP_LOGE(TAG, "ROM 加载失败,回菜单");
            nes_destroy(&machine);
            continue;
        }

        // 整 240 行(PPU 渲染 0..239,nes_video_blit 也按 240 行索引;
        // 用 NES_VISIBLE_HEIGHT=224 会少 16 行 → 推屏越界读 line[224..239])
        if (vid_setmode(NES_SCREEN_WIDTH, NES_SCREEN_HEIGHT)) {
            ESP_LOGE(TAG, "vid_setmode 失败,回菜单");
            nes_destroy(&machine);
            continue;
        }

        ESP_LOGI(TAG, "开始模拟: %s", rombank_name(sel));
        nes_emulate();          // 游戏内长按 OK -> nes_poweroff() -> 到这里返回

        nes_destroy(&machine);  // 释放本局全部资源(cpu/apu/ppu/mmc/rom)
        // 循环回菜单;BLE 连接保持,手柄可立即选下一个游戏
    }
}

void app_main(void) {
    // 显示:SPI 面板 + 背光。开机画面淡入->停留->淡出;背光保持熄灭,
    // 直到菜单第一帧完整上屏才点亮(nes_video_menu_draw)——过早点亮的话,
    // 亮起来的瞬间 GRAM 里还是开屏画面,菜单要再过一阵才画出来。
    ESP_ERROR_CHECK(bsp_display_init());

    // 音频:先于开屏初始化,开屏音效要跟淡入一起响;音量先给个能听见的值,
    // osd_volume_init 稍后从 NVS 恢复用户设置
    ESP_ERROR_CHECK(bsp_audio_init());
    bsp_audio_set_volume(70);

    splash_show(4000);   // 期间播开屏音效(音效任务把 codec 切到了 16kHz)

    // 恢复 NES 音频格式:22050Hz 16bit 单声道,bsp_audio_set_format 内部会先 close 再 open
    ESP_ERROR_CHECK(bsp_audio_set_format(22050, 16, 1));

    // 按键:回调只接长按退出;方向/A 键在 osd_getinput 里轮询 ADC 电压
    ESP_ERROR_CHECK(bsp_button_init(on_button, NULL));

    // 电量计:菜单状态栏显示用;芯片不应答时返回错误,忽略(菜单画空壳电池)
    bsp_battery_init();

    // 紧急出口:开机按住 OK -> 清空 NVS(删全部 BLE 配对 + 音量设置)。
    // 手柄与板子密钥失配时会陷入"发现->连接失败"死循环,只能删配对重建。
    {
        int mv = bsp_button_read_mv();
        if (mv >= 447 && mv < 1900) {          // OK 档,电压窗口见 bsp_pins.h
            ESP_LOGW(TAG, "开机按住 OK:清空 NVS(删 BLE 配对 + 音量设置)");
            esp_err_t nret = nvs_flash_erase();
            ESP_LOGW(TAG, "nvs_flash_erase: %s", esp_err_to_name(nret));
        }
    }

    // BLE 手柄(可选):初始化失败只警告,三键照样能玩
    {
        esp_err_t bret = ble_pad_init();
        if (bret != ESP_OK) {
            ESP_LOGW(TAG, "BLE 手柄初始化失败: %s(仅三键输入)", esp_err_to_name(bret));
        }
    }

    // 音量:从 NVS 恢复上次值并生效(ble_pad_init 已确保 NVS 就绪;codec 已打开)。
    // UP/DOWN 长按调音量,见 osd_esp.c。
    osd_volume_init();

    // 视频推屏层(清屏 + 黑边)
    if (nes_video_init() != 0) {
        ESP_LOGE(TAG, "nes_video_init 失败");
        return;
    }

    // 模拟器任务:nes_emulate 调用链较深,给足栈;优先级高于 idle 即可。
    // 必须检查返回值:堆不足时创建失败若不报错,设备会"静默黑屏"毫无线索。
    if (xTaskCreate(emu_task, "nes_emu", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "模拟任务创建失败(内存不足)");
        return;
    }
}
