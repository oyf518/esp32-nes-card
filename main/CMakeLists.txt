idf_component_register(
    SRCS "app_main.c"
         "rom_menu.c"
         "osd_esp.c"
         "nes_video.c"
         "ble_pad.c"
         "music.c"
         "splash.c"
    INCLUDE_DIRS "."
    # esp_hid:ESP-IDF v5.5.3 内置组件(BLE HID host,NimBLE 后端)
    # nvs_flash:BLE bond 配对信息存储;bt:NimBLE 头文件
    REQUIRES bsp nofrendo esp_timer esp_hid nvs_flash bt
    # 开机画面位图(tools/gen_splash.py 生成;ROM 一律走 storage 分区 ROM 库,不内嵌 .nes)
    EMBED_FILES "splash.bin"
)
