# esp32-nes-card · Card-sized FC/NES Game Console

[English](README_EN.md) | [简体中文](README.md)

Turn a FoloToy AI card (FoloToy-Card, ESP32-C3) into a NES/FC handheld:
nofrendo core + on-device ROM library menu + BLE wireless gamepad, 8MB flash,
fully commented in Chinese.

## Hardware

| Part | Description |
|---|---|
| SoC | ESP32-C3, single-core RISC-V @ 160MHz, **no PSRAM** — framebuffers live in SRAM |
| Flash | 8MB: firmware + 5.4MB raw-partition ROM library (20+ games) |
| Display | ST7789P3 2.0" SPI, 240x320 |
| Input | 3 on-board ADC buttons (UP/DOWN/OK); optional BLE HID gamepad (8BitDo etc.) |
| Audio | I2S + amplifier, 22050Hz 16-bit mono |

Other ESP32 / ESP32-C3 boards can be adapted by editing the pin table in
`components/bsp/include/bsp_pins.h`.

## Build & Flash

Requires [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32c3/get-started/) v5.5.x:

```bash
idf.py set-target esp32c3
idf.py build flash monitor
```

## ROM Library

The firmware ships with **no ROMs**. Games are read from a raw-partition ROM
library in the storage partition (format: `main/rom_menu.h`, packer:
`tools/rompack.py`). **Drop your own games into the `roms/` folder** (its
contents are git-ignored), then add a line `roms/yourgame.nes|Display Name`
to `tools/romlist.txt`:

```bash
# 1. No games yet? Generate a zero-copyright test ROM
python3 tools/gen_test_rom.py          # -> roms/test_rom.nes

# 2. Edit tools/romlist.txt (one "path|display name" per line), then pack
python3 tools/rompack.py -o rompack.bin -l tools/romlist.txt

# 3. Flash to 0x190000 (storage partition start, see partitions.csv)
python3 -m esptool --chip esp32c3 --port /dev/cu.usbmodemXXXX write_flash 0x190000 rompack.bin
```

**You are responsible for ROM legality**: use homebrew, freely distributed
ROMs, or dumps of cartridges you legally own. Redistributing commercial ROMs
is infringement — do not upload packed `rompack.bin` files anywhere.

After changing display names in the list, re-run
`python3 tools/gen_menufont.py` (the Chinese menu font is generated from the
character set of the list) and re-flash the firmware.

## BLE Gamepad

The device scans and connects to a BLE HID gamepad on boot, reconnects
automatically on disconnect, and stores pairing info in NVS. HID report
layouts differ between gamepads:

- Edit the `s_layout` preset in `main/ble_pad.c`, or
- Use the companion calibration tool to identify the layout and write it to
  the device's NVS from a web page:

```bash
python3 tools/padcal.py    # open http://127.0.0.1:8788 in a browser
```

**Long-press OK** in-game to return to the menu — a hot switch that keeps the
BLE connection alive.

## Project Layout

```
├── components/
│   ├── bsp/        # Board support: SPI panel / I2S audio / ADC buttons / fuel gauge (reusable)
│   └── nofrendo/   # NES emulator core (LGPL-2.0, © 1998-2000 Matthew Conte)
├── main/           # Firmware: boot / ROM library menu / BLE gamepad / OSD layer / blit / menu BGM
├── tools/
│   ├── rompack.py       # ROM library packer (directory table + 4KB-aligned raw image)
│   ├── gen_menufont.py  # Chinese menu font generator (driven by romlist.txt)
│   ├── gen_test_rom.py  # Zero-copyright test ROM generator (built-in minimal 6502 assembler)
│   ├── padcal.py        # Web-based BLE gamepad layout calibrator
│   └── romlist.txt      # ROM library list (example)
├── partitions.csv
└── sdkconfig.defaults
```

See [AGENTS.md](AGENTS.md) for architecture notes if you want to hack on it
with an AI coding assistant.

## Credits

- Nofrendo — Matthew Conte's NES emulator core, the heart of this project
- Espressif's esp32-nesemu port — reference for the stdbool/nofrendo header
  conflict handling in the `osd_*` layer

## License

- Original code in this repo (`main/`, `components/bsp/`, `tools/`): [MIT](LICENSE)
- `components/nofrendo/`: the Nofrendo core, kept under its original
  [LGPL-2.0](components/nofrendo/LICENSE) license and copyright notice
