# esp32-nes-card · 名片大小的 FC 游戏机

把一块 FoloToy AI 卡片(FoloToy-Card,ESP32-C3)刷成 NES/FC 模拟器掌机:
nofrendo 内核 + ROM 库菜单 + BLE 无线手柄,8MB Flash,全程中文注释。

> ESP32-C3 card-sized NES emulator based on nofrendo, with an on-device
> ROM library menu and BLE gamepad support. See [构建](#构建) below.
> (简体中文为主, English summary at the end)

<!-- TODO: 放一张实机照片 / 30 秒演示视频链接 -->

## 硬件

| 部件 | 说明 |
|---|---|
| 主控 | ESP32-C3,RISC-V 单核 160MHz,无 PSRAM,帧缓冲全放 SRAM |
| Flash | 8MB:固件 + 5.4MB 裸分区 ROM 库(约可放 20+ 个游戏) |
| 屏幕 | ST7789P3 2.0" SPI 240x320 |
| 输入 | 机身 ADC 三键(上/下/OK);可选 BLE HID 手柄(八位堂等) |
| 音频 | I2S + 功放,22050Hz 16bit 单声道 |

其他 ESP32 / ESP32-C3 板子改 `components/bsp/include/bsp_pins.h` 的引脚表即可适配。

## 构建与烧录

需要 [ESP-IDF](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.5.3/esp32c3/get-started/) v5.5.x:

```bash
idf.py set-target esp32c3
idf.py build flash monitor
```

## ROM 库

固件不含任何 ROM,游戏从 storage 分区的裸分区 ROM 库读取(格式见 `main/rom_menu.h`,
打包器见 `tools/rompack.py`)。

```bash
# 1. 仓库自带一个零版权测试 ROM,也可以换成自己的合法 ROM
python3 tools/gen_test_rom.py

# 2. 编辑 tools/romlist.txt(每行 "路径|显示名"),然后打包
python3 tools/rompack.py -o rompack.bin -l tools/romlist.txt

# 3. 烧到 0x190000(storage 分区起始,见 partitions.csv)
python3 -m esptool --chip esp32c3 --port /dev/cu.usbmodemXXXX write_flash 0x190000 rompack.bin
```

**ROM 合法性自担**:请使用自制 ROM、免费 homebrew,或你合法拥有实体卡带的转储。
商业 ROM 的再分发是侵权,请勿把打包产物传上网。

改了清单里的显示名后,记得重跑 `python3 tools/gen_menufont.py`
(菜单中文字库按清单字符集自动生成)再重新烧录固件。

## BLE 手柄

开机自动扫描并连接 BLE HID 手柄,断线自动重扫,配对信息存 NVS。
不同手柄的 HID 报文布局不同:

- 修改 `main/ble_pad.c` 里的 `s_layout` 预设,或
- 用配套校准工具在线识别按键布局并写入板子 NVS:

```bash
python3 tools/padcal.py    # 浏览器打开 http://127.0.0.1:8788
```

游戏内**长按 OK** 退回菜单,热切换不断开手柄连接。

## 目录结构

```
├── components/
│   ├── bsp/        # 板级支持包:SPI 屏/I2S 音频/ADC 按键/电量计(可复用到其他项目)
│   └── nofrendo/   # NES 模拟器内核(LGPL-2.0, © 1998-2000 Matthew Conte)
├── main/           # 固件:启动/ROM 库菜单/BLE 手柄/OSD 层/推屏/菜单 BGM
├── tools/
│   ├── rompack.py       # ROM 库打包器(目录表 + 4KB 对齐裸分区镜像)
│   ├── gen_menufont.py  # 菜单中文字库生成(按 romlist.txt 字符集)
│   ├── gen_test_rom.py  # 零版权测试 ROM 生成(内置极简 6502 汇编器)
│   ├── padcal.py        # BLE 手柄按键布局网页校准器
│   └── romlist.txt      # ROM 库清单(示例)
├── partitions.csv
└── sdkconfig.defaults
```

## 致谢

- Nofrendo — Matthew Conte 的 NES 模拟器内核,本项目的核心
- Espressif 的 esp32-nesemu 移植 — `osd_*` 层的 stdbool/nofrendo 头文件冲突处理方式参考了它

## 许可证

- 本仓库原创代码(main/、components/bsp/、tools/):[MIT](LICENSE)
- `components/nofrendo/`:Nofrendo 内核,保留其 [LGPL-2.0](components/nofrendo/LICENSE) 许可与版权声明

## English summary

An ESP32-C3 (RISC-V, no PSRAM) NES emulator firmware: nofrendo core driving a
240x320 ST7789 SPI panel over a framebuffer blit path, game ROMs served from a
custom raw-partition ROM library (no filesystem overhead, `mmap`-ed on select),
BLE HID gamepad with auto reconnect, menu BGM synthesized on-chip. Build with
ESP-IDF v5.5.x (`idf.py set-target esp32c3 && idf.py build flash`). Pack ROMs
with `tools/rompack.py` and flash to offset `0x190000`. Bring your own ROMs —
none are included or distributed here.
