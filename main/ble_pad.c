// main/ble_pad.c
// BLE HID 游戏手柄支持:esp_hid host(BLE 模式)+ NimBLE,只当 central/observer。
//
// 结构:
//   ble_pad_init()   —— NVS + BT 控制器 + NimBLE + esp_hidh,起 scan_task
//   scan_task        —— 循环:扫 30s -> 挑最佳候选 -> 连接 -> 等到断开 -> 重扫
//   hidh_callback    —— esp_hidh 事件:OPEN(置已连接)/ INPUT(解析报告)/ CLOSE(清状态重扫)
//   handle_input()   —— 按 s_layout 预设布局把 HID 报告解析成 NES 8 位位掩码
//
// 真机校准:不同手柄 HID 报告布局不同,内置预设不可能通吃。首次接手柄时
// 看日志 "RAW INPUT"(前 20 条原始 hex)和 "PAD"(按键变化)对照实际按下的键,
// 改下面的 s_layout 预设再烧录即可。
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_bt.h"

#include "esp_hidh.h"
#include "esp_hidh_nimble.h"        // esp_hidh_dev_open 的 NimBLE 版原型

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_nimble_hci.h"         // esp_nimble_enable

#include "ble_pad.h"

static const char *TAG = "ble_pad";

// NimBLE 自带 bond 存储(store/config)的初始化函数没有公开头文件,例程同样手声明
extern void ble_store_config_init(void);

// ---------------------------------------------------------------------------
// NES 手柄位掩码:与 nofrendo nesinput.h 的 INP_PAD_* 保持一致(见 ble_pad.h 注释)
// ---------------------------------------------------------------------------
#define NES_A       0x01
#define NES_B       0x02
#define NES_SELECT  0x04
#define NES_START   0x08
#define NES_UP      0x10
#define NES_DOWN    0x20
#define NES_LEFT    0x40
#define NES_RIGHT   0x80

// ---------------------------------------------------------------------------
// HID 报告布局预设(真机校准改这里)
// ---------------------------------------------------------------------------
typedef enum {
    DPAD_XY_AXIS,   // X/Y 各一个字节(0~255,中值 128),死区转方向
    DPAD_HAT,       // 4bit hat:0~7 自正北顺时针,>=8 回中
    DPAD_BITS,      // 独立 4bit:bit0=UP bit1=DOWN bit2=LEFT bit3=RIGHT
} dpad_type_t;

typedef struct {
    uint8_t report_id;      // 只解析该 report id;0xFF = 不检查(通吃无 ID 报告)
    dpad_type_t dpad_type;
    uint8_t dpad_byte;      // DPAD_HAT / DPAD_BITS 的字节偏移
    uint8_t x_byte;         // DPAD_XY_AXIS 的 X 轴字节偏移
    uint8_t y_byte;         // DPAD_XY_AXIS 的 Y 轴字节偏移
    uint8_t btn_byte;       // 按钮位掩码起始字节(占 2 字节共 16bit)
    uint8_t btn_map[16];    // 按钮位号 -> NES 位,0 = 不映射
} pad_layout_t;

// 预设:通用 Android/D-input 手柄布局(8BitDo D-input 模式、廉价蓝牙手柄多见)
//   byte0 = X 轴,byte1 = Y 轴(0~255 中值 128)
//   byte4~5 = 16 个按钮位:btn1(南键)=bit0, btn2(东键)=bit1, ..., btn9/bit8=Select, btn10/bit9=Start
// 注意:esp_hidh 的 INPUT 数据不含 report id 字节(HOGP 里 ID 由特征值隐式给出),
// 所以偏移从 0 起算。对不上就照 "RAW INPUT" 日志改本表。
// 实测校准(JZ-V4 BFM,2025 抓包):
//   byte1 = HAT(0x00上 02右 04下 06左 0F回中)
//   byte2 bit0 = 选择  bit1 = A   (bit2/3 推测为 B 兜底)
//   byte3 bit3 = 开始(实测两轮一致;此手柄 B 键与开始疑似同位,待真机验证)
static const pad_layout_t s_layout_default = {
    .report_id = 0xFF,
    .dpad_type = DPAD_HAT,
    .dpad_byte = 1,
    .btn_byte = 2,
    .btn_map = {
        [0] = NES_SELECT,   // byte2 bit0:选择键(实测)
        [1] = NES_A,        // byte2 bit1:A(实测)
        [2] = NES_B,        // byte2 bit2:B(推测位)
        [3] = NES_B,        // byte2 bit3:B(推测位兜底)
        [11] = NES_START,   // byte3 bit3:开始键(实测)
    },
};
// 用户校准布局(网页校准程序经串口写入 NVS,开机加载);无则用默认预设
static pad_layout_t s_layout_ram;
static const pad_layout_t *s_layout = &s_layout_default;
static volatile bool s_cal_mode;    // 校准模式:handle_input 额外输出 EVT 原始报文行

// XY 轴死区:距中值 128 超过该值才算方向按下(摇杆漂移/回中抖动防护)
#define AXIS_CENTER   128
#define AXIS_DEADZONE 64

// ---------------------------------------------------------------------------
// 运行状态
// ---------------------------------------------------------------------------
static volatile uint8_t s_pad_bits;     // 当前按键位掩码(事件任务写,emu 读)
static volatile bool    s_connected;
static esp_hidh_dev_t  *s_cur_dev;       // 当前连接的 HID 设备(优雅断开用)
static uint8_t          s_lastpad[6];    // 上次成功连接的手柄地址(快速重连)
static bool             s_have_lastpad;

static SemaphoreHandle_t s_scan_done;   // 扫描结束(SCAN_SECONDS 到点或提前取消)
static SemaphoreHandle_t s_dev_closed;  // 手柄断开(CLOSE 事件)

#define SCAN_SECONDS  10

// 扫描候选:只记得分最高的一台,不定长列表,内存有界
#define HID_SVC_UUID  0x1812            // HID over GATT
static struct {
    bool     valid;
    int      score;
    int8_t   rssi;
    uint8_t  addr[6];
    uint8_t  addr_type;
    uint16_t appearance;
    char     name[32];
} s_best;

// ---------------------------------------------------------------------------
// GAP 扫描回调(NimBLE host 任务上下文)
// ---------------------------------------------------------------------------
static int score_candidate(uint16_t appearance, const char *name)
{
    // 基础分 1:只要是 HID 设备就能当候选;gamepad/joystick appearance 优先,
    // 名字含常见手柄关键词的再加权;键盘外观不加分(防止误连键盘,但不排除)。
    int score = 1;
    if (appearance == ESP_HID_APPEARANCE_GAMEPAD ||
        appearance == ESP_HID_APPEARANCE_JOYSTICK) {
        score += 4;
    }
    if (name != NULL &&
        (strstr(name, "8BitDo") || strstr(name, "Gamepad") ||
         strstr(name, "gamepad") || strstr(name, "Controller"))) {
        score += 2;
    }
    return score;
}

static void handle_disc(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0) {
        return;
    }

    // 只认广播里带 HID 服务 UUID(0x1812)的设备
    bool is_hid = false;
    for (int i = 0; i < fields.num_uuids16; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == HID_SVC_UUID) {
            is_hid = true;
            break;
        }
    }
    if (!is_hid) {
        return;
    }

    char name[32] = {0};
    if (fields.name != NULL && fields.name_len < sizeof(name)) {
        memcpy(name, fields.name, fields.name_len);
    }

    uint16_t appearance = fields.appearance_is_present ? fields.appearance : 0;
    int score = score_candidate(appearance, name[0] ? name : NULL);

    ESP_LOGI(TAG, "发现 HID 设备: %02x:%02x:%02x:%02x:%02x:%02x RSSI=%d APP=0x%04x 分=%d 名='%s'",
             disc->addr.val[5], disc->addr.val[4], disc->addr.val[3],
             disc->addr.val[2], disc->addr.val[1], disc->addr.val[0],
             disc->rssi, appearance, score, name);

    // 快速重连:发现上次连接过的手柄,立即结束扫描直接进入连接
    if (s_have_lastpad && memcmp(disc->addr.val, s_lastpad, 6) == 0) {
        ESP_LOGI(TAG, "发现已配对手柄,提前结束扫描");
        ble_gap_disc_cancel();
    }

    // 得分高的优先;同分取信号强的
    if (!s_best.valid || score > s_best.score ||
        (score == s_best.score && disc->rssi > s_best.rssi)) {
        s_best.valid = true;
        s_best.score = score;
        s_best.rssi = disc->rssi;
        memcpy(s_best.addr, disc->addr.val, 6);
        s_best.addr_type = disc->addr.type;
        s_best.appearance = appearance;
        // snprintf 保证收尾 \0,避开 -Werror=stringop-truncation
        snprintf(s_best.name, sizeof(s_best.name), "%s", name);
    }
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        handle_disc(&event->disc);
        break;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        xSemaphoreGive(s_scan_done);
        break;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // 已有 bond 但对端要求重新配对:删旧 bond 重配,省事优先(同官方例程)
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        // 无屏无键盘,固定 passkey 应答(同官方例程策略)
        struct ble_sm_io pkey = {0};
        pkey.action = event->passkey.params.action;
        pkey.passkey = 123456;
        ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        break;
    }
    default:
        break;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// HID 报告解析(esp_hidh 事件任务上下文)
// ---------------------------------------------------------------------------
static const char *bit_name(uint8_t bit)
{
    switch (bit) {
    case NES_A:      return "A";
    case NES_B:      return "B";
    case NES_SELECT: return "SELECT";
    case NES_START:  return "START";
    case NES_UP:     return "UP";
    case NES_DOWN:   return "DOWN";
    case NES_LEFT:   return "LEFT";
    case NES_RIGHT:  return "RIGHT";
    }
    return "?";
}

static void log_bits_change(uint8_t old_bits, uint8_t new_bits)
{
    uint8_t diff = old_bits ^ new_bits;
    for (int i = 0; i < 8; i++) {
        uint8_t b = 1 << i;
        if (diff & b) {
            ESP_LOGI(TAG, "PAD: %s %s", bit_name(b), (new_bits & b) ? "按下" : "松开");
        }
    }
}

static uint8_t parse_axes(uint8_t v, uint8_t neg_bit, uint8_t pos_bit)
{
    if (v < AXIS_CENTER - AXIS_DEADZONE) {
        return neg_bit;
    }
    if (v > AXIS_CENTER + AXIS_DEADZONE) {
        return pos_bit;
    }
    return 0;
}

static uint8_t parse_report(const uint8_t *data, uint16_t len)
{
    const pad_layout_t *L = s_layout;
    uint8_t bits = 0;

    switch (L->dpad_type) {
    case DPAD_XY_AXIS:
        if (len > L->x_byte && len > L->y_byte) {
            bits |= parse_axes(data[L->x_byte], NES_LEFT, NES_RIGHT);
            bits |= parse_axes(data[L->y_byte], NES_UP, NES_DOWN);   // HID Y 轴向下为正
        }
        break;
    case DPAD_HAT:
        if (len > L->dpad_byte) {
            switch (data[L->dpad_byte] & 0x0F) {
            case 0: bits |= NES_UP; break;
            case 1: bits |= NES_UP | NES_RIGHT; break;
            case 2: bits |= NES_RIGHT; break;
            case 3: bits |= NES_DOWN | NES_RIGHT; break;
            case 4: bits |= NES_DOWN; break;
            case 5: bits |= NES_DOWN | NES_LEFT; break;
            case 6: bits |= NES_LEFT; break;
            case 7: bits |= NES_UP | NES_LEFT; break;
            default: break;   // 8/15 等 = 回中
            }
        }
        break;
    case DPAD_BITS:
        if (len > L->dpad_byte) {
            uint8_t d = data[L->dpad_byte];
            if (d & 0x01) bits |= NES_UP;
            if (d & 0x02) bits |= NES_DOWN;
            if (d & 0x04) bits |= NES_LEFT;
            if (d & 0x08) bits |= NES_RIGHT;
        }
        break;
    }

    // 按钮:btn_byte 起 2 字节 16bit,逐位查映射表
    if (len > L->btn_byte + 1) {
        uint16_t btn = data[L->btn_byte] | (data[L->btn_byte + 1] << 8);
        for (int i = 0; i < 16; i++) {
            if (btn & (1 << i)) {
                bits |= L->btn_map[i];
            }
        }
    }
    return bits;
}

static void handle_input(uint16_t report_id, const uint8_t *data, uint16_t len)
{
    // 校准模式:每条报文输出一行 EVT,网页校准程序靠它识别按键位号
    if (s_cal_mode) {
        char hex[3 * 32 + 4];
        int n = 0;
        uint16_t n2 = len > 32 ? 32 : len;
        for (uint16_t i = 0; i < n2; i++) n += snprintf(hex + n, sizeof(hex) - n, "%02X", data[i]);
        printf("EVT %u %u %s\n", report_id, n2, hex);
    }

    // 原始报告日志:前 20 条全打(真机校准用),之后每分钟最多一条防刷屏
    static int s_raw_count;
    static int64_t s_last_raw_log;
    if (s_raw_count < 20) {
        s_raw_count++;
        ESP_LOGI(TAG, "RAW INPUT id=%u len=%u data:", report_id, len);
        ESP_LOG_BUFFER_HEX(TAG, data, len);
    } else if (esp_timer_get_time() - s_last_raw_log > 60 * 1000 * 1000) {
        s_last_raw_log = esp_timer_get_time();
        ESP_LOGI(TAG, "RAW INPUT id=%u len=%u data:", report_id, len);
        ESP_LOG_BUFFER_HEX(TAG, data, len);
    }

    if (s_layout->report_id != 0xFF && report_id != s_layout->report_id) {
        return;
    }

    uint8_t bits = parse_report(data, len);
    if (bits != s_pad_bits) {
        log_bits_change(s_pad_bits, bits);
        s_pad_bits = bits;
    }
}

// ---------------------------------------------------------------------------
// esp_hidh 事件回调(事件任务上下文,做日志和解析可以,不能长时间阻塞)
// ---------------------------------------------------------------------------
static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    (void)handler_args; (void)base;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch ((esp_hidh_event_t)id) {
    case ESP_HIDH_OPEN_EVENT:
        if (param->open.status == ESP_OK) {
            s_connected = true;
            s_pad_bits = 0;
            s_cur_dev = param->open.dev;
            ESP_LOGI(TAG, "已连接: %s", esp_hidh_dev_name_get(param->open.dev));
            if (s_cal_mode) printf("CALCONN 1\n");
            // 打印 report map(按键布局校准的关键参考;对不上时看它推偏移)
            esp_hidh_dev_dump(param->open.dev, stdout);
        } else {
            ESP_LOGW(TAG, "连接失败(status=%d),继续扫描", param->open.status);
        }
        break;
    case ESP_HIDH_INPUT_EVENT:
        handle_input(param->input.report_id, param->input.data, param->input.length);
        break;
    case ESP_HIDH_BATTERY_EVENT:
        ESP_LOGI(TAG, "电量: %d%%", param->battery.level);
        break;
    case ESP_HIDH_CLOSE_EVENT:
        ESP_LOGW(TAG, "已断开: %s,重新扫描", esp_hidh_dev_name_get(param->close.dev));
        s_connected = false;
        s_pad_bits = 0;
        s_cur_dev = NULL;
        if (s_cal_mode) printf("CALCONN 0\n");
        esp_hidh_dev_free(param->close.dev);    // API 要求:CLOSE 事件里必须释放
        xSemaphoreGive(s_dev_closed);           // 唤醒 scan_task 重扫
        break;
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// 扫描任务:扫描 -> 连接 -> 守到断开 -> 重扫,全在本任务闭环,不阻塞 emu 任务
// ---------------------------------------------------------------------------
static void start_scan(void)
{
    uint8_t own_addr_type = 0;
    struct ble_gap_disc_params disc_params = {0};

    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        ESP_LOGE(TAG, "无法确定本机地址类型");
        return;
    }
    disc_params.filter_duplicates = 1;      // 同一设备重复广播只报一次
    disc_params.passive = 0;                // 主动扫描,拿 scan rsp 里的名字
    disc_params.itvl = 0x50;
    disc_params.window = 0x30;

    int rc = ble_gap_disc(own_addr_type, SCAN_SECONDS * 1000, &disc_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "启动扫描失败: rc=%d", rc);
    }
}

static void scan_task(void *arg)
{
    (void)arg;

    // 等 NimBLE host 与控制器同步完(esp_hidh 注册的 sync_cb 置位)
    while (!ble_hs_synced()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "NimBLE 就绪,开始扫描 BLE 手柄(%ds 一轮)", SCAN_SECONDS);

    for (;;) {
        s_best.valid = false;
        s_best.score = 0;
        start_scan();
        // 扫描时长固定 30s,DISC_COMPLETE 会 give 信号量;加个超时兜底
        xSemaphoreTake(s_scan_done, pdMS_TO_TICKS((SCAN_SECONDS + 5) * 1000));

        if (s_best.valid) {
            ESP_LOGI(TAG, "连接手柄: '%s' APP=0x%04x", s_best.name, s_best.appearance);
            // open 内部阻塞到 GATT 连接完成/失败(最长约 30s),在本任务跑无碍
            esp_hidh_dev_t *dev = esp_hidh_dev_open(s_best.addr, ESP_HID_TRANSPORT_BLE, s_best.addr_type);
            (void)dev;
            if (s_connected) {
                // 记住本次手柄地址:下次重启/掉线后扫描一旦见到它,秒连
                memcpy(s_lastpad, s_best.addr, 6);
                s_have_lastpad = true;
                nvs_handle_t h;
                if (nvs_open("nes", NVS_READWRITE, &h) == ESP_OK) {
                    uint8_t rec[7] = {0};
                    memcpy(rec, s_best.addr, 6);
                    rec[6] = (uint8_t)s_best.addr_type;
                    nvs_set_blob(h, "lastpad", rec, sizeof(rec));
                    nvs_commit(h);
                    nvs_close(h);
                }
            }
            if (dev != NULL) {
                // 守着连接,断开后 CLOSE 事件 give 信号量,出去重扫
                xSemaphoreTake(s_dev_closed, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(500));     // 断开后稍等再扫,避免狂转
            } else {
                ESP_LOGW(TAG, "open 失败,删除该设备旧配对信息,2s 后重扫");
                // 陈旧 bond 会让对端拿旧密钥来加密,必然失败 -> 每轮都连不上。
                // 删掉本端保存的这台设备配对信息,下一轮强制重新配对。
                ble_addr_t stale = { .type = s_best.addr_type };
                memcpy(stale.val, s_best.addr, 6);
                ble_store_util_delete_peer(&stale);
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        } else {
            // 一轮没扫到手柄,歇 5s 再扫(省电;emu 不受影响)
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

// ---------------------------------------------------------------------------
// 手柄校准(串口协议,配网页校准程序 nes/tools/padcal.py):
//   CAL ON / CAL OFF      开关 EVT 报文流(ON 后回 CALREADY)
//   EVT <id> <len> <hex>  每条 HID 输入报文一行
//   CALCONN <0/1>         连接状态变化
//   CAL GET               回 CALLAY <21字节hex> CONN=<0/1>
//   CAL SAVE <21字节hex>  写布局到 NVS 并生效,回 CALOK / CALERR
//   CAL FORGET            清空本端全部 bond(重新配对手柄用),回 CALOK
// 21 字节 = dpad_type, dpad_byte, x_byte, y_byte, btn_byte, btn_map[16]
// ---------------------------------------------------------------------------
#define CAL_BLOB_LEN 21
#define CAL_NVS_NS   "padcal"
#define CAL_NVS_KEY  "layout"

static void layout_to_blob(const pad_layout_t *L, uint8_t *b)
{
    b[0] = (uint8_t) L->dpad_type;
    b[1] = L->dpad_byte;
    b[2] = L->x_byte;
    b[3] = L->y_byte;
    b[4] = L->btn_byte;
    memcpy(b + 5, L->btn_map, 16);
}

static bool layout_from_blob(const uint8_t *b, pad_layout_t *L)
{
    if (b[0] > DPAD_BITS || b[1] > 15 || b[4] > 15) {
        return false;   // 明显非法,拒绝(保护解析端不越界)
    }
    memset(L, 0, sizeof(*L));
    L->report_id = 0xFF;
    L->dpad_type = (dpad_type_t) b[0];
    L->dpad_byte = b[1];
    L->x_byte = b[2];
    L->y_byte = b[3];
    L->btn_byte = b[4];
    memcpy(L->btn_map, b + 5, 16);
    return true;
}

static void layout_load(void)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    uint8_t blob[CAL_BLOB_LEN];
    size_t len = sizeof(blob);
    if (nvs_get_blob(h, CAL_NVS_KEY, blob, &len) == ESP_OK && len == sizeof(blob)
            && layout_from_blob(blob, &s_layout_ram)) {
        s_layout = &s_layout_ram;
        ESP_LOGI(TAG, "已加载用户校准布局: dpad_type=%d dpad_byte=%d btn_byte=%d",
                 s_layout_ram.dpad_type, s_layout_ram.dpad_byte, s_layout_ram.btn_byte);
    }
    nvs_close(h);
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static void cal_task(void *arg)
{
    (void)arg;
    char line[160];

    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (!strncmp(line, "CAL ON", 6)) {
            s_cal_mode = true;
            printf("CALREADY\n");
        } else if (!strncmp(line, "CAL OFF", 7)) {
            s_cal_mode = false;
        } else if (!strncmp(line, "CAL GET", 7)) {
            uint8_t blob[CAL_BLOB_LEN];
            char hex[CAL_BLOB_LEN * 2 + 1];
            layout_to_blob(s_layout, blob);
            for (int i = 0; i < CAL_BLOB_LEN; i++) {
                snprintf(hex + i * 2, 3, "%02X", blob[i]);
            }
            printf("CALLAY %s CONN=%d\n", hex, s_connected ? 1 : 0);
        } else if (!strncmp(line, "CAL SAVE ", 9)) {
            uint8_t blob[CAL_BLOB_LEN];
            const char *hx = line + 9;
            bool ok = true;
            for (int i = 0; i < CAL_BLOB_LEN && ok; i++) {
                int hi = hexval(hx[i * 2]), lo = hexval(hx[i * 2 + 1]);
                if (hi < 0 || lo < 0) ok = false;
                else blob[i] = (uint8_t) (hi * 16 + lo);
            }
            if (!ok || !layout_from_blob(blob, &s_layout_ram)) {
                printf("CALERR\n");
                continue;
            }
            nvs_handle_t h;
            if (nvs_open(CAL_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
                printf("CALERR\n");
                continue;
            }
            if (nvs_set_blob(h, CAL_NVS_KEY, blob, sizeof(blob)) != ESP_OK
                    || nvs_commit(h) != ESP_OK) {
                nvs_close(h);
                printf("CALERR\n");
                continue;
            }
            nvs_close(h);
            s_layout = &s_layout_ram;
            ESP_LOGI(TAG, "已保存并启用新校准布局: dpad_type=%d dpad_byte=%d btn_byte=%d",
                     s_layout_ram.dpad_type, s_layout_ram.dpad_byte, s_layout_ram.btn_byte);
            printf("CALOK\n");
        } else if (!strncmp(line, "CAL FORGET", 10)) {
            // 注意:不要在任务里直接调 ble_store_clear(),实测会与蓝牙栈互锁卡死。
            // 改为直接擦 NimBLE bond 的 NVS 命名空间后重启,干净可靠。
            nvs_handle_t h;
            esp_err_t erc = nvs_open("nimble_bond", NVS_READWRITE, &h);
            if (erc == ESP_OK) {
                nvs_erase_all(h);
                nvs_commit(h);
                nvs_close(h);
                printf("CALOK\n");
                ESP_LOGW(TAG, "配对信息已清除,3 秒后重启;请让手柄进配对模式重连");
                vTaskDelay(pdMS_TO_TICKS(3000));
                esp_restart();
            } else {
                printf("CALERR\n");   // 没有 bond 命名空间 = 本来就没有配对
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 对外接口
// ---------------------------------------------------------------------------
void ble_pad_graceful_reboot(void)
{
    // 先给手柄一个正常的 HCI 断开事件(直接 esp_restart 会让手柄状态机
    // 卡在"已连接",不再响应重连),等 CLOSE 事件收尾后再重启。
    esp_hidh_dev_t *dev = s_cur_dev;
    if (dev != NULL) {
        ESP_LOGW(TAG, "优雅断开手柄后重启");
        esp_hidh_dev_close(dev);
        xSemaphoreTake(s_dev_closed, pdMS_TO_TICKS(600));
        vTaskDelay(pdMS_TO_TICKS(200));   // 给手柄留出处理断开的时间
    }
    esp_restart();
}

uint8_t ble_pad_state(void)
{
    return s_pad_bits;
}

bool ble_pad_connected(void)
{
    return s_connected;
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();      // nimble_port_stop() 前不返回
    nimble_port_freertos_deinit();
}

esp_err_t ble_pad_init(void)
{
    esp_err_t ret;

    // NVS:BLE bond 信息要落 NVS,重连免配对
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs_flash_init 失败");

    layout_load();

    s_scan_done = xSemaphoreCreateBinary();
    s_dev_closed = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_scan_done && s_dev_closed, ESP_ERR_NO_MEM, TAG, "信号量创建失败");

    // 控制器 + NimBLE 协议栈(C3 只有 BLE,经典蓝牙内存直接释放)
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    ESP_RETURN_ON_ERROR(ret, TAG, "释放经典蓝牙内存失败");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_RETURN_ON_ERROR(ret, TAG, "BT 控制器初始化失败");
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    ESP_RETURN_ON_ERROR(ret, TAG, "BT 控制器使能失败");
    ret = esp_nimble_init();
    ESP_RETURN_ON_ERROR(ret, TAG, "esp_nimble_init 失败");

    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ret = esp_hidh_init(&config);
    ESP_RETURN_ON_ERROR(ret, TAG, "esp_hidh_init 失败");

    // SMP:启用 bonding(配对信息存 NVS),无屏无键盘 -> NoInputNoOutput,
    // 对端发起配对时走 Just Works;重复配对直接删旧 bond(见 gap_event_cb)
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ret = esp_nimble_enable(nimble_host_task);
    ESP_RETURN_ON_ERROR(ret, TAG, "esp_nimble_enable 失败");

    // 扫描任务:优先级低于 emu(5),栈留足给 esp_hidh_dev_open 的阻塞等待
    // 读上次成功连接的手柄地址(快速重连:扫描一见到它就提前收网)
    nvs_handle_t h;
    if (nvs_open("nes", NVS_READONLY, &h) == ESP_OK) {
        uint8_t rec[7] = {0};
        size_t len = sizeof(rec);
        if (nvs_get_blob(h, "lastpad", rec, &len) == ESP_OK && len == sizeof(rec)) {
            memcpy(s_lastpad, rec, 6);
            s_have_lastpad = true;
            ESP_LOGI(TAG, "上次手柄:%02x:%02x:%02x:%02x:%02x:%02x(优先重连)",
                     rec[5], rec[4], rec[3], rec[2], rec[1], rec[0]);
        }
        nvs_close(h);
    }

    xTaskCreate(scan_task, "ble_scan", 4096, NULL, 2, NULL);

    // 校准任务:读串口命令(网页校准程序用)
    xTaskCreate(cal_task, "padcal", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "BLE 手柄初始化完成,后台扫描中");
    return ESP_OK;
}
