// components/bsp/include/bsp_audio.h
//
// 双板差异:
//   ESP32-C3 (FoloToy-Card) : ES8311 codec + I2S 全双工(PCM 播放)
//   ESP32    (M5StickC Plus): GPIO2 方波喇叭(buzzer),只认频率
//
// ⚠ M5StickC Plus 用 bsp_audio_tone() 直接写频率(推荐),不要用 bsp_audio_write(PCM):
//   多声部混音波形做过零检测会破音(实测)。
#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

// 初始化音频(双板通用)。内部会调 bsp_i2c_init()(幂等)。
esp_err_t bsp_audio_init(void);

// 设置采样格式。同格式重复调用是廉价的(直接复用已打开的 codec)。
//
// ⚠ 这里有个必须绕开的坑:esp_codec_dev_open() 在 codec【已打开】时会直接返回 OK 且
//   【不重新配置采样率】。若不先 close,16kHz 播完再播 8kHz 会以 16k 时钟送出 ——
//   音调和速度都快一倍。故本函数在格式变化时先 close 再 open。
esp_err_t bsp_audio_set_format(uint32_t hz, uint8_t bits, uint8_t ch);

// 播放 / 录音。bytes 为字节数(16bit 单声道时 = 采样数 x 2)。
// ⚠ M5StickC Plus(buzzer)不支持 PCM 播放,请用 bsp_audio_tone()。
esp_err_t bsp_audio_write(const void *pcm, size_t bytes);
esp_err_t bsp_audio_read(void *pcm, size_t bytes);

// 输出音量 0..100(%)。
void bsp_audio_set_volume(uint8_t percent);

// M5StickC Plus 专用:直接驱动 GPIO2 方波喇叭发声。
// freq_hz=0 表示停止。C3(ES8311)上此函数为空操作。
void bsp_audio_tone(uint32_t freq_hz);
