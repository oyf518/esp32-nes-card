#!/usr/bin/env python3
# 开机画面生成:把源图(PNG,方形)转成 240x320 RGB565 裸位图 -> main/splash.bin
# 淡入淡出由固件用背光 PWM 实现(main/splash.c),这里只管位图。
#
#   用法: python3 tools/gen_splash.py [源图]   # 默认 assets/splash_src.png
#
# 布局:源图缩放到 240x240 居中,上下各 40px 用源图左上角取色填充
# (源图自带暗角边框,取角部色衔接自然)。字节序为 MSB-first RGB565,
# 与 nes_video.c 的 SWAP565 推屏约定一致。
import os, struct, sys
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "assets", "splash_src.png")
OUT = os.path.join(ROOT, "main", "splash.bin")

W, H = 240, 320
SQ = 240

img = Image.open(SRC).convert("RGB")
# 非方形先居中裁方
w, h = img.size
if w != h:
    s = min(w, h)
    img = img.crop(((w - s) // 2, (h - s) // 2, (w + s) // 2, (h + s) // 2))
img = img.resize((SQ, SQ), Image.LANCZOS)

bg = img.getpixel((2, 2))   # 角部取色作上下边带
print(f"边带取色: rgb{bg}")

def px565_be(r, g, b):
    v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    return struct.pack(">H", v)   # MSB first,匹配 SWAP565 后的内存布局

out = bytearray()
band = px565_be(*bg) * W
out += band * ((H - SQ) // 2)          # 上边带
pix = img.load()
for y in range(SQ):
    out += b"".join(px565_be(*pix[x, y]) for x in range(SQ))
out += band * (H - SQ - (H - SQ) // 2) # 下边带

assert len(out) == W * H * 2
with open(OUT, "wb") as f:
    f.write(out)
print(f"OK: {OUT} ({len(out)} bytes, {W}x{H} RGB565)")
