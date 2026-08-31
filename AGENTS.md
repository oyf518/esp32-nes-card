# AGENTS.md — AI 辅助开发指南

本文件给 AI 编码助手（以及新加入的人类）快速建立项目心智模型。
改代码前先读完；改完记得让本文件与实际保持一致。

## 项目是什么

ESP32-C3（RISC-V 单核 160MHz，**无 PSRAM**）上的 NES/FC 模拟器固件：

- 内核：`components/nofrendo`（Matthew Conte 的 Nofrendo，LGPL-2.0，**勿动其许可头**）
- 板级支持：`components/bsp`（SPI 屏 240x320、I2S 音频、ADC 三键、电量计；同时支持 ESP32/M5StickC Plus，按 `CONFIG_IDF_TARGET` 编译分支区分）
- 应用层：`main/`（开机画面、ROM 库菜单、BLE HID 手柄、OSD 层、帧缓冲推屏、菜单 BGM 合成）
- 工具链：`tools/`（ROM 打包、中文字库生成、测试 ROM 生成、手柄校准、开机画面生成）

## 硬约束（违反即事故）

1. **任何 `.nes`/`.bin` ROM 不得进入 git 仓库**（`.gitignore` 已拦，别绕过）。ROM 一律从 storage 分区的裸分区 ROM 库读取，固件不内嵌 ROM。
2. `components/nofrendo/` 保留 LGPL-2.0 与版权声明，不要删除或改写。
3. 本项目原创代码（`main/`、`components/bsp/`、`tools/`）以 MIT 发布。

## 常用命令

```bash
# 环境激活(本机): source ~/esp/esp-idf/export.sh  (ESP-IDF v5.5.x)

idf.py set-target esp32c3      # 首次
idf.py build                   # 构建验证(改代码后必跑)
idf.py -p PORT flash monitor   # 烧录 + 看日志

# ROM 库流程
python3 tools/gen_test_rom.py                       # 生成零版权测试 ROM -> tools/test_rom.nes
python3 tools/rompack.py -o rompack.bin -l tools/romlist.txt   # 打包
python3 -m esptool --chip esp32c3 --port PORT write_flash 0x190000 rompack.bin

# 改了 tools/romlist.txt 的显示名后,必须重跑:
python3 tools/gen_menufont.py   # 菜单中文字库跟随清单字符集自动生成 -> main/menufont.h

# 改了开机画面(assets/splash_src.png)后,必须重跑:
python3 tools/gen_splash.py     # 240x320 RGB565 位图 -> main/splash.bin(EMBED_FILES 嵌入固件)
```

## 关键架构知识（二开最常踩的点）

- **ROM 加载链**：`rom_menu.c` 读 storage 分区的 NESPACK1 格式（目录表 + 4KB 对齐 ROM），`esp_partition_mmap` 零拷贝映射；nofrendo 内核经 `app_main.c` 的 `osd_getromdata()` 取数据指针。改 ROM 库格式时，`rom_menu.c` 的 `rom_ent_t`（64B 槽位）与 `tools/rompack.py` 的目录项布局必须严格同步。
- **头文件包含顺序**：`osd_esp.c`/`app_main.c` 必须先引 ESP-IDF/标准头，再 `#undef` bool/true/false，最后才引 nofrendo 头（nofrtypes.h 是 C89 风格，与 stdbool 冲突）。新建引用 nofrendo 的文件照抄此模式。
- **热切换**：游戏内长按 OK → `nes_poweroff()` → `nes_emulate()` 返回 → 销毁本局 → 回菜单，全程不走重启，BLE 连接保持。新增游戏内功能时别破坏此循环。
- **内存**：无 PSRAM，帧缓冲全在 SRAM；推屏按整 240 行索引（`NES_VISIBLE_HEIGHT=224` 会越界，见 `nes_video.c` 注释）。
- **编译优化**：模拟器 CPU 密集，全局 `-O2`（`sdkconfig.defaults` 的 `CONFIG_COMPILER_OPTIMIZATION_PERF`），不要改回默认 `-Og`。
- **双板支持**：`components/bsp` 的引脚/分辨率按 `CONFIG_IDF_TARGET_ESP32` 区分 ESP32-C3(FoloToy-Card) 与 ESP32(M5StickC Plus)，改引脚看 `bsp_pins.h`。
- **开机画面**：`main/splash.c` 整屏推 EMBED 的 splash.bin，淡入淡出用背光 PWM（逐像素混合一帧 150KB@40MHz SPI 约 25ms，跑不出流畅动画）。换图重跑 `gen_splash.py`，字节序是 MSB-first RGB565（与 nes_video.c 的 SWAP565 约定一致）。
- **注释风格**：全项目中文注释、文件头一段说明设计意图，新代码保持一致。

## 改动验证清单

- [ ] `idf.py build` 通过（零 warning 新增为佳，nofrendo 老代码警告可忽略）
- [ ] 改了 ROM 库格式 → 重新 `rompack.py` 打包烧录，实机菜单能列出游戏
- [ ] 改了 `romlist.txt` 显示名 → `gen_menufont.py` 已重跑，菜单无豆腐块
- [ ] 改了 BLE 手柄解析 → `tools/padcal.py` 校准后实机按键正常
- [ ] 改了 `assets/splash_src.png` → `gen_splash.py` 已重跑，实机开机画面方向/色彩正常
- [ ] 没有把任何 `.nes`/`.bin` 带进 git（`main/splash.bin` 是唯一豁免）

## 仓库结构

```
├── components/bsp/        # 板级支持包(MIT,可复用)
├── components/nofrendo/   # 模拟器内核(LGPL-2.0)
├── main/                  # 应用固件(MIT)
├── tools/                 # 开发工具(MIT,Python3)
├── assets/                # 美术源图(splash_src.png -> main/splash.bin)
├── partitions.csv         # 8MB flash:app 1.5MB + storage 5.4MB(ROM 库,0x190000 起)
└── sdkconfig.defaults     # 全部关键配置都在这里,不依赖手工改 sdkconfig
```
