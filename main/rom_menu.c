// main/rom_menu.c
// storage 分区 ROM 库 + 菜单选择页。格式见 rom_menu.h。
#include "rom_menu.h"
#include "nes_video.h"
#include "ble_pad.h"
#include "music.h"
#include "menu_tune.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "bsp_button.h"   // bsp_button_read_mv

static const char *TAG = "rommenu";

// NES 手柄位定义(与 osd_esp.c 一致)
#define NESPAD_A      0x01
#define NESPAD_START  0x08
#define NESPAD_UP     0x10
#define NESPAD_DOWN   0x20

#define MENU_MAX_GAMES   64

typedef struct {
    char     name[48];     // UTF-8,\0 结尾
    uint32_t size;
    uint32_t offset;       // 分区内偏移(4KB 对齐)
    uint32_t crc32;
    uint32_t _pad;         // 槽位定长 64B,与打包器 ENT_SIZE 严格一致
} rom_ent_t;

static const esp_partition_t *s_part;
static int   s_count;
static char  s_names[MENU_MAX_GAMES][47];   // 显示用,仍是 47+\0
static rom_ent_t s_ents[MENU_MAX_GAMES];

static esp_partition_mmap_handle_t s_mmap;
static const void *s_rom_ptr;
static uint32_t    s_rom_size;

int rombank_count(void) { return s_count; }
const char *rombank_name(int idx) { return (idx >= 0 && idx < s_count) ? s_names[idx] : NULL; }

int rombank_init(void) {
    s_count = 0;
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");
    if (NULL == s_part) {
        ESP_LOGE(TAG, "找不到 storage 分区");
        return 0;
    }
    uint8_t hdr[16];
    if (esp_partition_read(s_part, 0, hdr, sizeof(hdr)) != ESP_OK) return 0;
    if (memcmp(hdr, "NESPACK1", 8) != 0) {
        ESP_LOGW(TAG, "storage 无 ROM 库(magic 不符),需烧录 rompack.bin");
        return 0;
    }
    uint32_t count, hdrsz;
    memcpy(&count, hdr + 8, 4);
    memcpy(&hdrsz, hdr + 12, 4);
    if (count == 0 || count > MENU_MAX_GAMES || hdrsz < 16 + count * 64) {
        ESP_LOGE(TAG, "目录非法: count=%lu hdrsz=%lu", (unsigned long)count, (unsigned long)hdrsz);
        return 0;
    }
    static rom_ent_t ents[MENU_MAX_GAMES];
    if (esp_partition_read(s_part, 16, ents, count * sizeof(rom_ent_t)) != ESP_OK) return 0;
    for (uint32_t i = 0; i < count; i++) {
        ents[i].name[sizeof(ents[i].name) - 1] = '\0';
        memcpy(s_names[i], ents[i].name, sizeof(s_names[i]) - 1);
        s_names[i][sizeof(s_names[i]) - 1] = 0;
        s_ents[i] = ents[i];
    }
    s_count = (int)count;
    ESP_LOGI(TAG, "ROM 库:%d 个游戏,%lu KB", s_count,
             (unsigned long)(s_part->size / 1024));
    return s_count;
}

int rombank_open(int idx) {
    if (idx < 0 || idx >= s_count) return -1;
    rom_ent_t *e = &s_ents[idx];
    if (s_rom_ptr) {
        esp_partition_munmap(s_mmap);
        s_rom_ptr = NULL;
    }
    esp_partition_mmap_handle_t mh;
    const void *ptr;
    esp_err_t err = esp_partition_mmap(s_part, e->offset, e->size,
                                       ESP_PARTITION_MMAP_DATA, &ptr, &mh);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mmap %s 失败: %s", e->name, esp_err_to_name(err));
        return -1;
    }
    s_mmap = mh;
    s_rom_ptr = ptr;
    s_rom_size = e->size;
    ESP_LOGI(TAG, "打开 [%d] %s (%luB @0x%06lx)",
             idx, e->name, (unsigned long)e->size, (unsigned long)e->offset);
    return 0;
}

void rombank_close(void) {
    if (s_rom_ptr) {
        esp_partition_munmap(s_mmap);
        s_rom_ptr = NULL;
    }
}

// nofrendo 内核 nes_rom.c 经 app_main.c 的 osd_getromdata() 取此指针
const void *rombank_ptr(uint32_t *size_out) {
    if (size_out) *size_out = s_rom_size;
    return s_rom_ptr;
}

// ---------------------------------------------------------------------------
// 按键轮询:机身 ADC 三键 + BLE 手柄,统一成三事件(UP/DOWN/OK)的边沿+连发
// ---------------------------------------------------------------------------
typedef enum { KEY_NONE, KEY_UP, KEY_DOWN, KEY_OK } menu_key_t;

static menu_key_t poll_keys(void) {
    // 机身键优先(手柄方向键同时按也不冲突,菜单阶段模拟循环未启动)
    int mv = bsp_button_read_mv();
    if (mv >= 0 && mv < 150)    return KEY_UP;
    if (mv >= 150 && mv < 447)  return KEY_DOWN;
    if (mv >= 447 && mv < 1900) return KEY_OK;
    // BLE 手柄(NES 位:UP=0x10 DOWN=0x20 A=0x01 START=0x08)
    uint8_t b = ble_pad_state();
    if (b & NESPAD_UP)                       return KEY_UP;
    if (b & NESPAD_DOWN)                     return KEY_DOWN;
    if (b & (NESPAD_A | NESPAD_START))       return KEY_OK;
    return KEY_NONE;
}

// 边沿检测 + 按住连发(首延时 400ms,连发间隔 120ms)
static menu_key_t key_edge(void) {
    static menu_key_t last = KEY_NONE;
    static int hold_ms = 0;   // 粗略按住时长
    menu_key_t k = poll_keys();
    if (k == KEY_NONE) {
        last = KEY_NONE;
        hold_ms = 0;
        return KEY_NONE;
    }
    if (k != last) {          // 新按下:立即响应
        last = k;
        hold_ms = 0;
        return k;
    }
    vTaskDelay(pdMS_TO_TICKS(15));
    hold_ms += 15;
    if (hold_ms > 400 && (hold_ms - 400) % 120 < 15) return k;   // 连发
    return KEY_NONE;
}

int rom_menu_run(void) {
    const int n = s_count;
    if (n <= 0) return 0;
    int sel = 0;
    ESP_LOGI(TAG, "进菜单:%d 个游戏", n);
    // 从游戏长按 OK 热切换回来时,手指可能还按着:先等所有键松开,
    // 否则残留的 KEY_OK 会瞬间"确认"又回到同一个游戏
    while (poll_keys() != KEY_NONE) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(50));   // 松键消抖
    music_play_loop(&k_menu_tune);   // 菜单 BGM:循环播放,确认进游戏时停止
    const char *name_ptrs[MENU_MAX_GAMES];   // 二维数组不能直接转 char**,构造真指针数组
    for (int i = 0; i < n; i++) name_ptrs[i] = s_names[i];
    for (;;) {
        nes_video_menu_draw(name_ptrs, n, sel);
        menu_key_t k = key_edge();
        if (k == KEY_UP)    sel = (sel + n - 1) % n;
        else if (k == KEY_DOWN) sel = (sel + 1) % n;
        else if (k == KEY_OK) {
            // 确认去抖:松开再返回,避免进游戏瞬间还读到按键
            while (poll_keys() != KEY_NONE) vTaskDelay(pdMS_TO_TICKS(15));
            music_stop();
            return sel;
        }
    }
}
