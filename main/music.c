// main/music.c —— 菜单 BGM 循环播放器:方波合成播放 tune_t 里的多声部音符表。
// 25% 占空比仿 NES 脉冲声道;相位用整数累加,避免逐采样浮点(C3 无 FPU)。
// 移植自 menu_app/main/music.c:去掉音效任务与音量覆盖,改为单曲循环。
#include "music.h"
#include "bsp_audio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "music";

#define SAMPLE_RATE   16000
#define CHUNK_SAMPLES 256                 // 每次送 16ms,控制任务栈内缓冲
#define MAX_VOICES    4
#define VOICE_AMP     5500                // 单声部振幅;四声部同响 22000 < 32767
#define RELEASE_MS    8                   // 音尾线性衰减,去爆音

static volatile bool s_playing;
static volatile bool s_stop_req;

// MIDI 音高 -> 整数相位步进(2^32 / 采样率)
static uint32_t note_step(uint8_t note) {
    // freq = 440 * 2^((note-69)/12);2^(k/12)*256 查表,再按八度移位
    static const uint16_t k_semi[12] = {
        256, 271, 287, 304, 322, 341, 362, 383, 406, 430, 456, 483,
    };
    int rel = note - 69;                  // 相对 A4
    int oct = rel / 12, semi = rel % 12;
    if (semi < 0) { semi += 12; oct -= 1; }
    uint32_t freq_x256 = 440u * k_semi[semi];
    if (oct >= 0) freq_x256 <<= oct; else freq_x256 >>= -oct;
    // step = freq / SAMPLE_RATE * 2^32 = freq_x256 * 2^32 / (256 * SAMPLE_RATE)
    return (uint32_t)(((uint64_t)freq_x256 << 24) / SAMPLE_RATE);
}

typedef struct {
    const melody_note_t *notes;
    uint16_t len;
    uint16_t idx;                         // 当前音符下标
    uint32_t phase;                       // 方波相位累加器
    uint32_t step;                        // 当前音符相位步进
} voice_t;

static void music_task(void *arg) {
    const tune_t *tune = arg;
    if (bsp_audio_set_format(SAMPLE_RATE, 16, 1) != ESP_OK) {
        ESP_LOGE(TAG, "set_format 失败");
        s_playing = false;
        vTaskDelete(NULL);
        return;
    }

    int nv = tune->voice_count < MAX_VOICES ? tune->voice_count : MAX_VOICES;
    voice_t v[MAX_VOICES];

    int16_t buf[CHUNK_SAMPLES];
    uint32_t total = ((uint32_t)tune->total_ms + 50) * (SAMPLE_RATE / 1000);
    while (!s_stop_req) {                 // 循环整曲
        memset(v, 0, sizeof(v));
        for (int i = 0; i < nv; i++) {
            v[i].notes = tune->voices[i];
            v[i].len = tune->lens[i];
        }
        for (uint32_t base = 0; base < total && !s_stop_req; base += CHUNK_SAMPLES) {
            for (int i = 0; i < CHUNK_SAMPLES; i++) {
                uint32_t t = (base + i) * 1000 / SAMPLE_RATE;  // 当前采样时刻(ms)
                int32_t mix = 0;
                for (int k = 0; k < nv; k++) {
                    voice_t *vc = &v[k];
                    while (vc->idx < vc->len) {
                        const melody_note_t *n = &vc->notes[vc->idx];
                        if (t < (uint32_t)n->start_ms + n->dur_ms) break;
                        vc->idx++;
                        vc->phase = 0;    // 换音符相位归零,占空比对齐
                    }
                    if (vc->idx >= vc->len) continue;
                    const melody_note_t *n = &vc->notes[vc->idx];
                    if (t < n->start_ms) continue;
                    if (vc->phase == 0) vc->step = note_step(n->note);
                    vc->phase += vc->step;
                    int amp = (vc->phase < 0x40000000u) ? VOICE_AMP : -VOICE_AMP;  // 25% 占空比
                    uint32_t left = (uint32_t)n->start_ms + n->dur_ms - t;
                    if (left < RELEASE_MS) amp = amp * (int)left / RELEASE_MS;
                    mix += amp;
                }
                buf[i] = (int16_t)mix;
            }
            bsp_audio_write(buf, sizeof(buf));
        }
    }

    memset(buf, 0, sizeof(buf));          // 尾部送一段静音,防止 codec 停在直流电平
    bsp_audio_write(buf, sizeof(buf));
    ESP_LOGI(TAG, "BGM 停止");
    s_playing = false;
    vTaskDelete(NULL);
}

void music_play_loop(const tune_t *tune) {
    if (s_playing || !tune) return;
    s_stop_req = false;
    s_playing = true;
    xTaskCreate(music_task, "music", 4096, (void *)tune, 4, NULL);
}

// 请求停止并等待音乐任务退出(通常 <20ms)
void music_stop(void) {
    if (!s_playing) return;
    s_stop_req = true;
    while (s_playing) vTaskDelay(pdMS_TO_TICKS(10));
}
