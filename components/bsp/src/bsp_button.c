// components/bsp/src/bsp_button.c
// 移植自 trae_card/components/platform/platform_esp32/src/btn_iot_button.c
//
// 双板支持:
//   ESP32-C3 (FoloToy-Card) : ADC 分压三键(iot_button 组件)
//   ESP32    (M5StickC Plus): GPIO37 实体按键(低有效),按下=OK,长按=重启
// 原 C3 逻辑保留在 #else 分支。
#include "bsp_button.h"
#include "bsp_pins.h"
#if CONFIG_IDF_TARGET_ESP32
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include "iot_button.h"
#include "button_adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#endif
#include "esp_log.h"

static const char *TAG = "bsp_btn";

// ---------------------------------------------------------------------------
// M5StickC Plus:GPIO37 实体按键(按下为低电平)
// ---------------------------------------------------------------------------
#if CONFIG_IDF_TARGET_ESP32
static bsp_btn_cb_t    s_cb;
static void           *s_user;
static TaskHandle_t    s_task;
static bool            s_pressed;
static bool            s_long_fired;

// 长按判定阈值:按住超过该时间视为"OK 长按"(触发 esp_restart)。
#define STICKC_LONG_PRESS_MS   1500

static bool btn_pressed(void) {
    // 低电平有效:按下=0
    return gpio_get_level(BSP_BTN_GPIO) == 0;
}

static void btn_task(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        bool down = btn_pressed();
        if (down && !s_pressed) {           // 新按下
            s_pressed = true;
            s_long_fired = false;
            last = xTaskGetTickCount();
            if (s_cb) s_cb(BSP_BTN_OK, BSP_BTN_PRESS, s_user);
        } else if (down && s_pressed && !s_long_fired) {
            if ((xTaskGetTickCount() - last) >= pdMS_TO_TICKS(STICKC_LONG_PRESS_MS)) {
                s_long_fired = true;        // 只触发一次,直到松开再按
                ESP_LOGI(TAG, "按键长按 OK,触发重启");
                if (s_cb) s_cb(BSP_BTN_OK, BSP_BTN_LONG, s_user);
            }
        } else if (!down && s_pressed) {    // 松开
            s_pressed = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *user) {
    s_cb = cb; s_user = user;
    if (s_task) return ESP_OK;              // 幂等
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BSP_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        // ⚠ GPIO37 是输入专用 pad,没有内部上拉;板上已有外部上拉(按下=低)。
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&cfg) != ESP_OK) { ESP_LOGE(TAG, "按键 GPIO 配置失败"); return ESP_FAIL; }
    BaseType_t ok = xTaskCreate(btn_task, "btn_gpio", 2048, NULL, 5, &s_task);
    if (ok != pdPASS) { ESP_LOGE(TAG, "按键任务创建失败"); return ESP_FAIL; }
    ESP_LOGI(TAG, "M5StickC Plus 按键就绪:GPIO%d 按下=OK,长按 %dms 触发重启",
             BSP_BTN_GPIO, STICKC_LONG_PRESS_MS);
    return ESP_OK;
}

int bsp_button_read_mv(void) { return -1; }   // 无 ADC 按键电压

#else  // ---- FoloToy-Card(ESP32-C3) 原实现 ----

static const uint16_t BTN_MV[BSP_BTN_COUNT][2] = BSP_BTN_MV_TABLE;

static button_handle_t s_btn[BSP_BTN_COUNT];
static bsp_btn_cb_t    s_cb;
static void           *s_user;

// ADC1 是 unit 级独占资源:iot_button 与 bsp_button_read_mv() 必须共用同一个 oneshot
// 句柄。谁第二个调 adc_oneshot_new_unit() 谁就拿到 "adc1 is already in use"。
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;

// 电压读取的衰减档必须与 button 组件内部的 ADC_BUTTON_ATTEN 一致 —— 通道只被配置一次
// (由组件在 iot_button_new_adc_device() 里下发),两边对不上会让读数与按键阈值错位。
// managed_components/espressif__button/button_adc.c:26 在 C3 上取 ADC_ATTEN_DB_6+1。
#define BSP_BTN_ATTEN  ADC_ATTEN_DB_12       // 量程约 0~3100mV,覆盖松开态

// 每个按键把"哪个键"随回调带回来。button 组件的回调签名固定,故用 usr_data 传索引。
static void on_event(void *arg, void *usr_data, bsp_btn_ev_t ev) {
    (void)arg;
    if (!s_cb) return;
    s_cb((bsp_btn_t)(intptr_t)usr_data, ev, s_user);
}
static void cb_press (void *a, void *u) { on_event(a, u, BSP_BTN_PRESS);  }
static void cb_click (void *a, void *u) { on_event(a, u, BSP_BTN_CLICK);  }
static void cb_double(void *a, void *u) { on_event(a, u, BSP_BTN_DOUBLE); }
static void cb_long  (void *a, void *u) { on_event(a, u, BSP_BTN_LONG);   }

esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *user) {
    s_cb = cb; s_user = user;

    // 先由 BSP 建 unit,再把句柄交给 button 组件(button_adc.h:adc_handle 非 NULL 即复用),
    // 这样本文件的 bsp_button_read_mv() 也能读同一路 ADC。
    const adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = BSP_BTN_ADC_UNIT };
    esp_err_t ae = adc_oneshot_new_unit(&ucfg, &s_adc);
    if (ae != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit 创建失败 (%s)", esp_err_to_name(ae));
        s_adc = NULL;
        return ae;
    }

    for (int i = 0; i < BSP_BTN_COUNT; i++) {
        const button_adc_config_t ac = {
            .adc_handle   = &s_adc,          // 复用上面这一个,别让组件自建
            .unit_id      = BSP_BTN_ADC_UNIT,
            .adc_channel  = BSP_BTN_ADC_CHANNEL,
            .button_index = i,
            .min          = BTN_MV[i][0],
            .max          = BTN_MV[i][1],
        };
        const button_config_t bc = { 0 };
        esp_err_t e = iot_button_new_adc_device(&bc, &ac, &s_btn[i]);
        if (e != ESP_OK || !s_btn[i]) {
            ESP_LOGE(TAG, "按键 %d 创建失败 (%s) —— 检查 GPIO%d 的 ADC 配置与分压电阻",
                     i, esp_err_to_name(e), BSP_BTN_ADC_CHANNEL);
            return e == ESP_OK ? ESP_FAIL : e;
        }
        void *idx = (void *)(intptr_t)i;
        iot_button_register_cb(s_btn[i], BUTTON_PRESS_DOWN,      NULL, cb_press,  idx);
        iot_button_register_cb(s_btn[i], BUTTON_SINGLE_CLICK,    NULL, cb_click,  idx);
        iot_button_register_cb(s_btn[i], BUTTON_DOUBLE_CLICK,    NULL, cb_double, idx);
        iot_button_register_cb(s_btn[i], BUTTON_LONG_PRESS_START,NULL, cb_long,   idx);
    }

    // 通道已由组件配置好,这里只补一份校准句柄给 bsp_button_read_mv() 用。
    // 失败不致命:按键照常工作,只是读不出电压(标定分压电阻时才需要)。
    // 经典 ESP32 只能用 line fitting 方案(C3 等才有 curve fitting)。
#if CONFIG_IDF_TARGET_ESP32
    const adc_cali_line_fitting_config_t cal = {
        .unit_id  = BSP_BTN_ADC_UNIT,
        .atten    = BSP_BTN_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cal, &s_cali) != ESP_OK) {
#else
    const adc_cali_curve_fitting_config_t cal = {
        .unit_id  = BSP_BTN_ADC_UNIT,
        .chan     = BSP_BTN_ADC_CHANNEL,
        .atten    = BSP_BTN_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) != ESP_OK) {
#endif
        ESP_LOGW(TAG, "ADC 校准创建失败,Button 页将无法显示电压");
        s_cali = NULL;
    }

    ESP_LOGI(TAG, "按键就绪:ADC1_CH%d 三键分压", BSP_BTN_ADC_CHANNEL);
    return ESP_OK;
}

int bsp_button_read_mv(void) {
    // 读的是 bsp_button_init() 建好、并与 iot_button 共用的那一路 ADC。
    // 单次采样与组件的按键轮询互不干扰(oneshot 内部自带锁)。
    if (!s_adc || !s_cali) return -1;

    int raw = 0, mv = 0;
    if (adc_oneshot_read(s_adc, BSP_BTN_ADC_CHANNEL, &raw) != ESP_OK) return -1;
    if (adc_cali_raw_to_voltage(s_cali, raw, &mv) != ESP_OK) return -1;
    return mv;
}

#endif  // CONFIG_IDF_TARGET_ESP32 / FoloToy-Card(ESP32-C3)
