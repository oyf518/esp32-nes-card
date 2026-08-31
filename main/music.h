// main/music.h —— 菜单 BGM / 短音效播放器:方波合成(25% 占空比仿 NES 脉冲声道),
// 多声部混音经 ES8311 播放,独立任务运行,不阻塞调用方。
// 移植自 menu_app/main/music.c,裁剪为仅循环播放;
// 音量沿用当前设置(游戏音量由 osd_esp.c 管理,这里不碰)。
#pragma once

#include <stdint.h>

// {起始 ms, 时长 ms, MIDI 音高}
typedef struct {
    uint32_t start_ms;
    uint32_t dur_ms;
    uint8_t  note;
} melody_note_t;

// 一首曲子:若干单声部音轨 + 总时长
typedef struct {
    const melody_note_t *const *voices;
    const uint16_t *lens;
    uint8_t  voice_count;
    uint32_t total_ms;
} tune_t;

// 循环播放一首曲子,直到 music_stop()。已在播时再次调用被忽略。
// 内部把 I2S 切到 16kHz/16bit/单声道;停止后调用方负责恢复游戏格式。
void music_play_loop(const tune_t *tune);

// 播放一遍即停(开屏音效用)。已在播时忽略;内部同样把 I2S 切到
// 16kHz/16bit/单声道,播完后停在 16kHz,调用方负责恢复所需格式。
void music_play_once(const tune_t *tune);

// 立即停止播放(未在播则空操作)。阻塞到音乐任务退出,通常 <20ms。
void music_stop(void);

// 已播时长 ms(自本次播放起点计);未在播返回 0。
// 菜单动效用它对齐 BGM 节拍(波纹律动与音乐同源,不会各走各的)。
uint32_t music_pos_ms(void);
