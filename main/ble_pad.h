// main/ble_pad.h
// BLE HID 游戏手柄(esp_hid host + NimBLE,BLE-only;ESP32-C3 无经典蓝牙)。
//
// ble_pad_state() 返回的 8 位位掩码与 nofrendo components/nofrendo/nes/nesinput.h
// 的 INP_PAD_* 布局保持一致(A=0x01 B=0x02 SELECT=0x04 START=0x08
// UP=0x10 DOWN=0x20 LEFT=0x40 RIGHT=0x80),由 ble_pad.c 自己重定义一份,
// 不引 nofrendo 头 —— 避免 noftypes.h 的 C89 enum bool 与 stdbool 冲突。
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// 初始化 NVS + NimBLE + esp_hid host,起后台扫描任务。返回 ESP_OK 即已在跑;
// 失败不影响三键输入,调用方按警告处理即可。
esp_err_t ble_pad_init(void);

// 当前手柄按键状态(NES 8 位位掩码,未连接返回 0)。
// 由 esp_hidh 事件任务写、emu 任务读,uint8_t 单字节读写,volatile 保证可见性。
uint8_t ble_pad_state(void);

// 优雅断开当前手柄链接后软件重启(游戏内长按 OK 返回菜单用)。
// 直接 esp_restart 会让手柄状态机卡死不再重连。
void ble_pad_graceful_reboot(void);

// 是否已连接(供日志/UI)
bool ble_pad_connected(void);
