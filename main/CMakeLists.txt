idf_component_register(
    SRCS "app_main.c"
         "rom_menu.c"
         "osd_esp.c"
         "nes_video.c"
         "ble_pad.c"
         "music.c"
    INCLUDE_DIRS "."
    # esp_hid:ESP-IDF v5.5.3 内置组件(BLE HID host,NimBLE 后端)
    # nvs_flash:BLE bond 配对信息存储;bt:NimBLE 头文件
    REQUIRES bsp nofrendo esp_timer esp_hid nvs_flash bt
    # ROM 一律走 storage 分区的 ROM 库(见 tools/rompack.py),固件不内嵌任何 .nes
)
