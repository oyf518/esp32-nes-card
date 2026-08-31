// main/splash.h
// 开机画面:整屏位图(assets/splash_src.png -> tools/gen_splash.py -> main/splash.bin)
#pragma once

// 在已 bsp_display_init() 的屏幕上显示开机画面:
// 背光淡入 -> 停留 hold_ms -> 淡出到全黑。随后由调用方恢复正常背光。
void splash_show(int hold_ms);
