// main/nes_video.h
// NES 帧缓冲裁剪 + 推屏。
//
// 显示方案(定稿,勿随意改):
//   NES 输出 256x240,本机竖屏 240x320。左右各裁 8 像素(正好是 NES overscan 区)
//   得 240x240,推到屏幕 y in [40,280) 居中,上下黑边。不旋转、不缩放。
//   SPI 40MHz 推一整帧约 23ms 跑不满 60fps,故每模拟 2 帧推 1 次屏(30fps),
//   游戏逻辑与音频仍按 60fps 跑。
#pragma once

#include <stdint.h>
#include <bitmap.h>   // nofrendo bitmap_t

// 裁剪后尺寸与屏幕落点
#define NES_FB_W      240           // 256 - 左右各 8
#define NES_FB_H      240
#define NES_FB_SLICE  120           // 半帧推屏:s_fb 拆上下两片各 120 行,省一半 DRAM
#define NES_CROP_X    8             // NES overscan 左缘
#define NES_LCD_Y     40            // 屏幕上缘(上下各留 40 行黑边)

// 初始化:取 LCD 面板句柄、整屏清黑(上下黑边只在此清一次)。
// 返回 0 成功。
int nes_video_init(void);

// nofrendo 帧末回调:把 8bpp 调色板帧缓冲裁剪转换成 RGB565 并按 2:1 抽帧推屏。
// 内部持有调色板(nes_video_set_palette 设置的 RGB565 表)。
void nes_video_blit(const bitmap_t *bmp);

// 设置 NES 调色板(256 项的 RGB565 版本,含内核附加在 192+ 的 GUI 色)。
void nes_video_set_palette(const uint16_t *pal565);

// 音量 OSD:调整音量时调用,在游戏区底部画音量条,约 1.5s 后自动消失。
void nes_video_osd_volume(int percent);

// ---------------------------------------------------------------------------
// 菜单页渲染(ROM 选择菜单专用,与 nes_video_blit 共享半帧缓冲,不新增大内存)
// ---------------------------------------------------------------------------
// 整页重画并推屏:现代扁平风,全屏 240x320 —— 顶部状态栏(BLE 连接状态、
// 标题「小小游戏机」、电池电量)+ 纵向卡片轮转(选中卡居中放大带纯色描边,
// 上下三级邻近卡缩小压暗,切换时滑动过渡;卡片裁剪在两栏之间)
// + 底部选中位置进度条。
// names[i] 为 UTF-8 中文/ASCII;sel 为选中项。阻塞推屏完成才返回。
// 滑动动效依赖周期性重画(rom_menu_run 已循环调用)。
void nes_video_menu_draw(const char *const *names, int count, int sel);

// 菜单覆盖了全屏(含游戏区的上下黑边);进游戏前调用,把 y<40 / y>=280
// 清回黑色,避免菜单残留。
void nes_video_clear_edges(void);

// 借出半帧缓冲 s_fb 作通用推屏暂存(开机画面逐带推屏用),容量 240x120 像素。
// 仅限 nes_video_init 之前或菜单/游戏未在推屏时使用,借出方不得越界写。
uint16_t *nes_video_scratch(void);
