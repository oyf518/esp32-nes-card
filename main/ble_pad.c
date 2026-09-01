// main/ble_pad.c
// BLE HID 游戏手柄支持:纯 NimBLE GAP/GATT 自实现 host(不用 esp_hidh),只当 central/observer。
//
// 不用 esp_hidh 的原因:它连接后必读 HID Report Map(ATT 长读),实测部分手柄
// (IINE)固件在此步让自身链路层停止应答 -> C3 控制器 HCI 超时(BLE_HS_ETIMEOUT)
// -> 整个 BLE 栈静默死亡。本文件的按键解析由 handle_input + s_layout 布局表
// 独立完成,只需要原始 input report 字节流,因此绕开 esp_hidh:
//   连接 -> 加密 -> 服务发现(0x1812) -> 给 input 特征值的 CCCD 写 0x0003 开通知,
// 全程只做 Find Information / Read By Type 这类短交互,绝不长读 Report Map。
//
// 结构:
//   ble_pad_init()   —— NVS + BT 控制器 + NimBLE,起 scan_task
//   scan_task        —— 循环:扫描 -> 挑最佳候选 -> ble_gap_connect -> 等到断开 -> 重扫
//   gap_event_cb     —— GAP 事件:扫描 + CONNECT/ENC_CHANGE/NOTIFY_RX/DISCONNECT
//   svc/chr/dsc_disc_cb —— GATT 服务发现链,订阅 HID input report 通知
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

#include "esp_hid_common.h"         // ESP_HID_APPEARANCE_*(扫描候选打分用)

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_mbuf.h"       // ble_hs_mbuf_to_flat(notify 数据拷出)
#include "host/ble_hs_adv.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_nimble_hci.h"         // esp_nimble_enable
#include "nimble/ble.h"             // BLE_ERR_REM_USER_CONN_TERM(HCI 断开原因码)

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
// 注意:HOGP 的通知载荷不含 report id 字节(HOGP 里 ID 由特征值隐式给出),
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
static volatile uint8_t s_pad_bits;     // 当前按键位掩码(GAP 回调写,emu 读)
static volatile bool    s_connected;
static uint16_t         s_conn_handle;   // 当前连接句柄(优雅断开用,配 s_connected)
static uint8_t          s_own_addr_type; // 本机地址类型(start_scan 里推断,连接复用)
static uint8_t          s_lastpad[6];    // 上次成功连接的手柄地址(快速重连)
static bool             s_have_lastpad;

static SemaphoreHandle_t s_scan_done;   // 扫描结束(SCAN_SECONDS 到点或提前取消)
static SemaphoreHandle_t s_dev_closed;  // 手柄断开/连接失败(唤醒 scan_task)

// GATT 发现进度(HID 0x1812 服务 -> input 特征值 -> CCCD),都在 NimBLE host
// 任务回调里顺序推进,一次连接内只走一遍,用 static 传递中间结果即可
static uint16_t s_hid_start_handle;     // HID 服务句柄范围
static uint16_t s_hid_end_handle;
static bool     s_hid_svc_found;        // 已找到 0x1812 服务
static int      s_cccd_count;           // 已写入的 CCCD 数(订阅成功的输入报告数)
static uint16_t s_proto_mode_handle;    // HID Protocol Mode 特征值(0x2A4E),0=未发现
static uint16_t s_map_handle;           // Report Map 特征值(0x2A4B),0=未发现
static int      s_input_chr_count;      // 发现的可通知输入特征值数
static uint16_t s_output_handles[4];    // 输出报告特征值(LED 等,开机写一次解锁上报)
static int      s_output_count;

// ---------------------------------------------------------------------------
// BLE 看门狗:部分手柄(实测 IINE Phone 模式)会在 GATT 发现阶段触发 HCI 超时
// (status=261 BLE_HS_ETIMEOUT),主机与控制器失联后整个 BLE 栈静默死亡 ——
// 扫描无声停止、连接流程永不收尾,应用层无任何回调可救。
// 对策:监控任务发现"未连接 && BLE 超过 60s 无活动"即重启(约 6s 起完,
// 手柄自动回连,用户几乎无感)。活动时间戳由扫描循环与 DISC 事件持续刷新;
// 连接态不监控(正常游戏数小时不动也合法)。
#define BLE_WDT_TIMEOUT_MS  60000
static volatile uint32_t s_ble_activity_ms;   // 最近一次 BLE 活动的 uptime ms

static void ble_wdt_feed(void) {
    s_ble_activity_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

static void ble_wdt_task(void *arg) {
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (s_connected) continue;
        uint32_t idle = (uint32_t)(esp_timer_get_time() / 1000) - s_ble_activity_ms;
        if (idle > BLE_WDT_TIMEOUT_MS) {
            ESP_LOGE(TAG, "BLE %lus 无活动且未连接,判定栈挂死,重启自愈",
                     (unsigned long)(idle / 1000));
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
    }
}

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
    ble_wdt_feed();   // 收到广播 = BLE 栈活着
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

// ---------------------------------------------------------------------------
// GATT 服务发现链(加密成功后,运行在 NimBLE host 任务上下文)。
// 回调间用上面那组 static 变量传递进度,链式发起下一步发现。
// 只做服务/特征值/描述符发现 + 写 CCCD,绝不读 Report Map 长读(会触发部分
// 手柄固件挂死,见文件头)。
// ---------------------------------------------------------------------------
#define BLE_PAD_WRITE_PROTO_MODE 0   // 实测 IINE 手柄该写操作会断数据流,关闭
#define BLE_PAD_WRITE_OUTPUT    0   // 实测盲写输出报告会让 JZ-V4 手柄持续震动,关闭;
                                    // 个别手柄若需 LED 写入才上报,置 1 单独验证
static void gatt_disc_state_reset(void)
{
    s_hid_start_handle = 0;
    s_hid_end_handle = 0;
    s_hid_svc_found = false;
    s_cccd_count = 0;
    s_proto_mode_handle = 0;
    s_map_handle = 0;
    s_input_chr_count = 0;
    s_output_count = 0;
}

// 描述符回调:给 HID 服务里所有 CCCD(0x2902)写 0x0003。发现范围覆盖整个
// HID 服务,把 report / boot input 的 CCCD 一并打开——多写无害,手柄只在
// 自己的报告通道上发通知。
static int dsc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)arg; (void)chr_val_handle;
    if (error->status != 0) {   // BLE_HS_EDONE:遍历结束
        ESP_LOGI(TAG, "订阅完成:共开启 %d 个输入报告通知", s_cccd_count);
#if BLE_PAD_WRITE_OUTPUT
        // 输出报告(LED):部分手柄收到主机的 LED 写入后才开始上报。
        // 注意:盲写输出报告会触发部分手柄震动电机(JZ-V4 实测),默认关闭。
        for (int i = 0; i < s_output_count; i++) {
            const uint8_t off = 0x00;
            int rc = ble_gattc_write_flat(conn_handle, s_output_handles[i],
                                          &off, 1, NULL, NULL);
            ESP_LOGI(TAG, "写输出报告 handle=%d rc=%d", s_output_handles[i], rc);
        }
#endif
#if BLE_PAD_WRITE_PROTO_MODE
        // HID 协议模式 = Report(0x01):不少手柄开机处于 Boot 模式且不发
        // input report,要主机写入后才正式开始上报。
        // 注意:实测 IINE Gamepad 模式下这一写会让手柄数据流当场停止
        // (其固件对该写操作脆弱),故默认关闭,除非确认手柄需要。
        if (s_proto_mode_handle != 0) {
            const uint8_t report_mode = 0x01;
            struct os_mbuf *om = ble_hs_mbuf_from_flat(&report_mode, 1);
            if (om != NULL) {
                int rc = ble_gattc_write_no_rsp(conn_handle, s_proto_mode_handle, om);
                ESP_LOGI(TAG, "写 Protocol Mode=Report handle=%d rc=%d",
                         s_proto_mode_handle, rc);
            }
        }
#endif
        return 0;
    }
    if (dsc->uuid.u.type != BLE_UUID_TYPE_16 ||
            ble_uuid_u16(&dsc->uuid.u) != BLE_GATT_DSC_CLT_CFG_UUID16) {
        return 0;   // 只认 CCCD
    }
    static const uint8_t cccd_val[2] = {0x03, 0x00};    // 0x0003:notify + indicate
    int rc = ble_gattc_write_flat(conn_handle, dsc->handle, cccd_val, sizeof(cccd_val),
                                  NULL, NULL);
    s_cccd_count++;
    ESP_LOGI(TAG, "写 CCCD handle=%d rc=%d", dsc->handle, rc);
    return 0;
}

// Report Map 安全读回调:只用普通 ATT_READ(单包,最多 MTU-3 字节),
// 绝不发 ATT_READ_BLOB 长读 —— 长读会触发部分手柄固件链路层挂死(实测)。
// 读它不为用内容(布局解析不用),而是很多手柄要见到"主机读过 map"才开始上报。
static int map_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       struct ble_gatt_attr *attr, void *arg)
{
    (void)arg;
    if (error->status == 0) {
        uint8_t head[16];
        uint16_t n = sizeof(head);
        ble_hs_mbuf_to_flat(attr->om, head, sizeof(head), &n);
        ESP_LOGI(TAG, "Report Map 普通读成功(仅取前 %u 字节): %02x%02x%02x%02x...",
                 (unsigned)n, head[0], head[1],
                 n > 2 ? head[2] : 0, n > 3 ? head[3] : 0);
    } else {
        ESP_LOGW(TAG, "Report Map 读失败 status=%d,继续订阅", error->status);
    }
    ble_gattc_disc_all_dscs(conn_handle, s_hid_start_handle, s_hid_end_handle,
                            dsc_disc_cb, NULL);
    return 0;
}

// 特征值回调:收集全部可通知特征值(HID input report)与 Protocol Mode(0x2A4E)。
// 描述符发现必须等特征值遍历 EDONE 后再发起——NimBLE 同一时刻只允许一个
// GATT 过程,中途另起会打断特征值遍历(实测只订阅到部分 CCCD)。
static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if (error->status != 0) {   // BLE_HS_EDONE:遍历结束
        if (!s_hid_svc_found) return 0;
        // 先安全读 Report Map(解锁部分手柄的上报),再进入 CCCD 订阅
        if (s_map_handle != 0) {
            ESP_LOGI(TAG, "普通读 Report Map handle=%d(不用长读)", s_map_handle);
            ble_gattc_read(conn_handle, s_map_handle, map_read_cb, NULL);
            return 0;
        }
        ESP_LOGI(TAG, "无 Report Map,直接发起描述符发现");
        ble_gattc_disc_all_dscs(conn_handle, s_hid_start_handle, s_hid_end_handle,
                                dsc_disc_cb, NULL);
        return 0;
    }
    if (ble_uuid_u16(&chr->uuid.u) == 0x2A4B) {
        s_map_handle = chr->val_handle;
        ESP_LOGI(TAG, "Report Map 特征值 handle=%d", chr->val_handle);
    }
    if (chr->properties & (BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE)) {
        s_input_chr_count++;
        ESP_LOGI(TAG, "输入特征值 val_handle=%d props=0x%02x",
                 chr->val_handle, chr->properties);
    }
    if (ble_uuid_u16(&chr->uuid.u) == 0x2A4E) {
        s_proto_mode_handle = chr->val_handle;
        ESP_LOGI(TAG, "Protocol Mode 特征值 handle=%d", chr->val_handle);
    }
    if ((chr->properties & (BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP)) &&
            s_output_count < 4) {
        s_output_handles[s_output_count++] = chr->val_handle;
        ESP_LOGI(TAG, "输出特征值 val_handle=%d props=0x%02x",
                 chr->val_handle, chr->properties);
    }
    return 0;
}

// 服务回调:锁定 0x1812 的句柄范围,继续发现其下特征值
static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg)
{
    (void)arg;
    if (error->status != 0) {
        if (!s_hid_svc_found) {
            ESP_LOGW(TAG, "对端无 HID 服务(0x1812)");
        }
        return 0;
    }
    if (ble_uuid_u16(&service->uuid.u) == HID_SVC_UUID) {
        s_hid_svc_found = true;
        s_hid_start_handle = service->start_handle;
        s_hid_end_handle = service->end_handle;
        ESP_LOGI(TAG, "HID 服务句柄范围 %d~%d", s_hid_start_handle, s_hid_end_handle);
        ble_gattc_disc_all_chrs(conn_handle, s_hid_start_handle, s_hid_end_handle,
                                chr_disc_cb, NULL);
    }
    return 0;
}

static void handle_input(uint16_t report_id, const uint8_t *data, uint16_t len);

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
    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            s_pad_bits = 0;
            gatt_disc_state_reset();
            ble_wdt_feed();
            ESP_LOGI(TAG, "已连接 conn_handle=%d", s_conn_handle);
            if (s_cal_mode) printf("CALCONN 1\n");
            // HID 特征值要求加密链路,由本端发起配对/加密(已有 bond 则直接加密)
            int rc = ble_gap_security_initiate(event->connect.conn_handle);
            if (rc != 0) {
                ESP_LOGW(TAG, "发起加密失败 rc=%d,等待链路断开重扫", rc);
            }
        } else {
            ESP_LOGW(TAG, "连接建立失败(status=%d),唤醒重扫", event->connect.status);
            xSemaphoreGive(s_dev_closed);
        }
        break;
    }
    case BLE_GAP_EVENT_ENC_CHANGE: {
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "链路已加密,开始 GATT 服务发现");
            ble_uuid16_t hid_uuid = BLE_UUID16_INIT(HID_SVC_UUID);
            int rc = ble_gattc_disc_svc_by_uuid(event->enc_change.conn_handle, &hid_uuid.u,
                                                svc_disc_cb, NULL);
            if (rc != 0) {
                ESP_LOGW(TAG, "发起服务发现失败 rc=%d", rc);
            }
        } else {
            ESP_LOGW(TAG, "加密失败 status=%d", event->enc_change.status);
        }
        break;
    }
    case BLE_GAP_EVENT_NOTIFY_RX: {
        // 按键数据主通道:HID input report 的 notify/indicate 载荷。
        // report id 在 HOGP 里由特征值隐式给出,载荷里没有,统一按 0 传
        // (布局表 report_id=0xFF 通吃,EVT 行的 id 字段为 0)。
        uint8_t buf[128];
        uint16_t out_len = 0;
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, buf, sizeof(buf), &out_len) == 0 &&
                out_len > 0) {
            handle_input(0, buf, out_len);
        }
        break;
    }
    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGW(TAG, "已断开: %02x:%02x:%02x:%02x:%02x:%02x (reason=%d),重新扫描",
                 event->disconnect.conn.peer_ota_addr.val[5],
                 event->disconnect.conn.peer_ota_addr.val[4],
                 event->disconnect.conn.peer_ota_addr.val[3],
                 event->disconnect.conn.peer_ota_addr.val[2],
                 event->disconnect.conn.peer_ota_addr.val[1],
                 event->disconnect.conn.peer_ota_addr.val[0],
                 event->disconnect.reason);
        s_connected = false;
        s_pad_bits = 0;
        gatt_disc_state_reset();
        ble_wdt_feed();   // 断开后时间戳从零起算,否则立刻误判挂死
        if (s_cal_mode) printf("CALCONN 0\n");
        xSemaphoreGive(s_dev_closed);           // 唤醒 scan_task 重扫
        break;
    }
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
// HID 报告解析(NimBLE host 任务上下文,做日志和解析可以,不能长时间阻塞)
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
// 扫描任务:扫描 -> 连接 -> 守到断开 -> 重扫,全在本任务闭环,不阻塞 emu 任务
// ---------------------------------------------------------------------------
static void start_scan(void)
{
    struct ble_gap_disc_params disc_params = {0};

    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
        ESP_LOGE(TAG, "无法确定本机地址类型");
        return;
    }
    disc_params.filter_duplicates = 1;      // 同一设备重复广播只报一次
    disc_params.passive = 0;                // 主动扫描,拿 scan rsp 里的名字
    disc_params.itvl = 0x50;
    disc_params.window = 0x30;

    int rc = ble_gap_disc(s_own_addr_type, SCAN_SECONDS * 1000, &disc_params, gap_event_cb, NULL);
    if (rc != 0) {
        // 常见于上一轮发现过程没被正确收尾(BUSY):取消残留过程,下轮重试。
        // 不取消的话 disc 会一直失败,扫描从此哑掉。
        ESP_LOGE(TAG, "启动扫描失败: rc=%d,取消残留过程", rc);
        ble_gap_disc_cancel();
    }
}

static void scan_task(void *arg)
{
    (void)arg;

    // 等 NimBLE host 与控制器同步完
    while (!ble_hs_synced()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "NimBLE 就绪,开始扫描 BLE 手柄(%ds 一轮)", SCAN_SECONDS);

    for (;;) {
        s_best.valid = false;
        s_best.score = 0;
        ble_wdt_feed();   // 每轮循环喂狗:连接卡死/扫描死掉都会被看门狗抓到
        // 防令牌漂移:DISC_COMPLETE 偶发多发一次(取消/超时竞争),计数累积后
        // 会让后面的轮次瞬间"拿到假令牌",扫描实际还在跑就贸然去连接。
        while (xSemaphoreTake(s_scan_done, 0) == pdTRUE) {}
        start_scan();
        // 扫描时长固定 30s,DISC_COMPLETE 会 give 信号量;加个超时兜底
        xSemaphoreTake(s_scan_done, pdMS_TO_TICKS((SCAN_SECONDS + 5) * 1000));

        if (s_best.valid) {
            ESP_LOGI(TAG, "连接手柄: '%s' APP=0x%04x", s_best.name, s_best.appearance);
            ble_addr_t peer = { .type = s_best.addr_type };
            memcpy(peer.val, s_best.addr, 6);
            // 连接参数:30/50ms 间隔、无 latency、4s 监督超时,手柄通用
            struct ble_gap_conn_params conn_params = {
                .scan_itvl = 0x50,          // 连接期间回扫,与扫描参数一致
                .scan_window = 0x30,
                .itvl_min = 24,             // 30ms
                .itvl_max = 40,             // 50ms
                .latency = 0,
                .supervision_timeout = 400, // 400 * 10ms = 4s
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            // 非阻塞发起;结果经 gap_event_cb 回报(成功置 s_connected,失败 give s_dev_closed)
            int rc = ble_gap_connect(s_own_addr_type, &peer, 30000, &conn_params,
                                     gap_event_cb, NULL);
            if (rc != 0) {
                // 常见于上一轮连接过程没收尾(BUSY):取消残留过程,下轮重试
                ESP_LOGE(TAG, "发起连接失败: rc=%d,取消残留过程", rc);
                ble_gap_conn_cancel();
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            // 轮询等连接结果(控制器 30s 连接超时兜底,不会永久卡住)
            int waited_ms = 0;
            while (!s_connected && waited_ms < 32000 &&
                   xSemaphoreTake(s_dev_closed, pdMS_TO_TICKS(200)) != pdTRUE) {
                waited_ms += 200;
            }
            if (s_connected) {
                ble_wdt_feed();
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
                // 守着连接,断开后出去重扫。
                // 不能死等 s_dev_closed(portMAX_DELAY):NimBLE 偶发不回调
                // DISCONNECT(实测:手柄配对中长按配对键这类硬断链),任务会
                // 在此永久阻塞,扫描从此静默挂死。改为在 GAP 层轮询链路是否
                // 还存在,任一侧发现断开即退出。
                while (s_connected) {
                    struct ble_gap_conn_desc desc;
                    if (ble_gap_conn_find(s_conn_handle, &desc) != 0) {
                        ESP_LOGW(TAG, "GAP 层链路已消失(未收到断开回调),继续重扫");
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                // 消费可能残留/迟到的断开令牌,避免计数累积
                xSemaphoreTake(s_dev_closed, 0);
                vTaskDelay(pdMS_TO_TICKS(500));     // 断开后稍等再扫,避免狂转
            } else {
                ESP_LOGW(TAG, "连接失败,删除该设备旧配对信息,2s 后重扫");
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
    // 卡在"已连接",不再响应重连),等断开事件收尾后再重启。
    if (s_connected) {
        ESP_LOGW(TAG, "优雅断开手柄后重启");
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
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

    // 扫描任务:优先级低于 emu(5),栈留足给 GATT 服务发现回调链
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
    xTaskCreate(ble_wdt_task, "ble_wdt", 3072, NULL, 1, NULL);

    // 校准任务:读串口命令(网页校准程序用)
    xTaskCreate(cal_task, "padcal", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "BLE 手柄初始化完成,后台扫描中");
    return ESP_OK;
}
