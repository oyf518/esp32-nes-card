// main/rom_menu.h
// ROM 库:storage 分区裸格式(目录表 + 连续 ROM)+ 菜单选择页。
//
// 分区格式(tools/rompack.py 生成,esptool write_flash 0x190000 烧录):
//   0x0000  magic "NESPACK1"
//   0x0008  u32 游戏数 count
//   0x000C  u32 目录区大小(定长 4096)
//   0x0010  目录项 × 64B: name[47](UTF-8) + u32 size + u32 offset + u32 crc32
//   0x1000  ROM 数据连续存放,每个文件 4KB 对齐
// 读取方式:esp_partition_mmap 直接映射到虚拟地址,内核 XIP 零拷贝,不占 RAM。
#pragma once
#include <stdint.h>

// 找 storage 分区并解析目录。返回游戏数(0 = 空库/格式不对)。
int rombank_init(void);

int rombank_count(void);

// 游戏名(UTF-8,打包时的显示名),idx 越界返回 NULL。
const char *rombank_name(int idx);

// 打开第 idx 个游戏:mmap 分区该 ROM 段,之后 osd_getromdata() 返回其映射地址。
// 返回 0 成功;-1 参数/映射失败(调用方可回菜单另选)。
int rombank_open(int idx);

// 释放当前 mmap(目前游戏内常驻,不做动态换)。
void rombank_close(void);

// 菜单循环(阻塞直到玩家选中):渲染列表到 LCD,轮询机身三键 + BLE 手柄。
//   UP/DOWN 选择(机身与手柄方向键均有效,支持按住连滚),OK/A/START 确认。
// 菜单期间不触碰音量(游戏内的音量键逻辑不在此路径)。
// 返回选中的游戏索引(仅当 rombank_count()>0 时被调用才有意义)。
int rom_menu_run(void);

// 取当前已打开 ROM 的映射地址与大小(未打开返回 NULL)。
// app_main.c 的 osd_getromdata() 转调此口,供 nofrendo 内核读 ROM。
const void *rombank_ptr(uint32_t *size_out);
