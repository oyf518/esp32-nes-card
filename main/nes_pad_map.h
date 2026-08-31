// main/nes_pad_map.h
// 三键(ADC 分压)-> NES 手柄 1 的映射表,集中在此方便按游戏改。
//
// 当前(UP/DOWN 已改作音量键,见 osd_esp.c):
//   UP 键 -> 音量+, DOWN 键 -> 音量-, OK 键 -> A(跳跃)
//   双击 OK -> Start(标题画面开始游戏用);长按 OK -> esp_restart 退出
//   游戏方向(左/右)不再用实体键,靠 BLE 手柄
//
// 已知硬件限制:UP/DOWN 同按时分压电压会落到 DOWN 档,读出来只有 DOWN 生效,
// 接受即可(三键本来也玩不了需要组合键的游戏)。
#pragma once

#include <stdint.h>
#include <nesinput.h>          // INP_PAD_* 位定义
#include "bsp_button.h"        // bsp_btn_t
#include "bsp_pins.h"          // BSP_BTN_COUNT

// 按下表把物理按键翻译成 NES 手柄位;0 表示该键不映射。
static const uint8_t NES_PAD_MAP[BSP_BTN_COUNT] = {
    [BSP_BTN_UP]   = 0,           // 音量 + (osd_esp.c)
    [BSP_BTN_DOWN] = 0,           // 音量 - (osd_esp.c)
    [BSP_BTN_OK]   = INP_PAD_A,
};
