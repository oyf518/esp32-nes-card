# nofrendo 2.0 (Matthew Conte) NES 模拟器核心。
# 来源:https://github.com/espressif/esp32-nesemu 的 components/nofrendo
# (乐鑫已做过嵌入式裁剪:nes.c 直接渲染进 vid 主缓冲、nes_rom.c 改走 osd_getromdata()
#  从内存读 ROM,不再依赖文件系统)。
#
# 相对上游删除:nofrendo.c(原版 main,我们自己在 app_main 里驱动)、config/event/gui/
# intro/pcx 等桌面端代码。gui 仅保留 gui.h(内核引用 GUI_BLACK 等常量),
# gui_frame/gui_tick/gui_sendmsg/gui_pal 在 main/osd_esp.c 里以桩实现。

set(NOFRENDO_SRCS
    "bitmap.c"
    "vid_drv.c"
    "log.c"
    "memguard.c"
    "cpu/nes6502.c"
    "cpu/dis6502.c"
    "nes/mmclist.c"
    "nes/nes.c"
    "nes/nes_mmc.c"
    "nes/nes_pal.c"
    "nes/nes_ppu.c"
    "nes/nes_rom.c"
    "nes/nesinput.c"
    "nes/nesstate.c"
    "sndhrdw/nes_apu.c"
    "sndhrdw/fds_snd.c"
    "sndhrdw/mmc5_snd.c"
    "sndhrdw/vrcvisnd.c"
    "libsnss/libsnss.c"
    # mappers:全部保留,与 nes/mmclist.c 的注册表一一对应;裁剪文件必须同步裁剪 mmclist
    "mappers/map000.c"
    "mappers/map001.c"
    "mappers/map002.c"
    "mappers/map003.c"
    "mappers/map004.c"
    "mappers/map005.c"
    "mappers/map007.c"
    "mappers/map008.c"
    "mappers/map009.c"
    "mappers/map011.c"
    "mappers/map015.c"
    "mappers/map016.c"
    "mappers/map018.c"
    "mappers/map019.c"
    "mappers/map024.c"
    "mappers/map032.c"
    "mappers/map033.c"
    "mappers/map034.c"
    "mappers/map040.c"
    "mappers/map041.c"
    "mappers/map042.c"
    "mappers/map046.c"
    "mappers/map050.c"
    "mappers/map064.c"
    "mappers/map065.c"
    "mappers/map066.c"
    "mappers/map070.c"
    "mappers/map073.c"
    "mappers/map075.c"
    "mappers/map078.c"
    "mappers/map079.c"
    "mappers/map085.c"
    "mappers/map087.c"
    "mappers/map093.c"
    "mappers/map094.c"
    "mappers/map099.c"
    "mappers/map160.c"
    "mappers/map229.c"
    "mappers/map231.c"
    "mappers/mapvrc.c"
)

idf_component_register(
    SRCS ${NOFRENDO_SRCS}
    INCLUDE_DIRS "." "cpu" "nes" "sndhrdw" "mappers" "libsnss"
    PRIV_REQUIRES esp_timer   # 帧级性能剖析(nes_renderframe 计时)
)

# 老代码,警告铺天盖地:全部压掉(-w),不挨个改源码。
# 优化B5: -O3 + 重开跳转表(全局参数里禁了,模拟核心的 switch 密集,吃这个亏)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -w)

# 渲染完整 240 行(默认 NES_VISIBLE_HEIGHT=224 只渲染 NTSC 可见区)。
# PUBLIC:main 组件的 osd 层也要按 240 行裁剪/推屏,两边必须一致。
target_compile_definitions(${COMPONENT_LIB} PUBLIC NES_VISIBLE_HEIGHT=240)
