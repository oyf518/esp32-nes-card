#!/usr/bin/env python3
# 菜单字库生成:字符集自动派生自 ROM 清单的显示名 + 固定 UI 文案。
# 清单加游戏后重跑本脚本即可,显示名里的新字自动进字库。
#
#   用法: python3 tools/gen_menufont.py [清单路径]   # 默认 tools/romlist.txt
#         私人清单(不入库): python3 tools/gen_menufont.py tools/romlist.local.txt
#         注意:用私人清单生成后 main/menufont.h 会含私人游戏名,勿提交!
import os, re, sys
from PIL import Image, ImageFont, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIST = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "tools", "romlist.txt")
OUT = os.path.join(ROOT, "main", "menufont.h")

UI_TEXT = "选择游戏游戏库OK键开始无小小游戏机"
chars = set(UI_TEXT)
for line in open(LIST, encoding="utf-8"):
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    disp = line.split("|", 1)[1] if "|" in line else ""
    chars.update(disp)
chars = sorted(chars)

FONT_CANDIDATES = [
    "/System/Library/Fonts/PingFang.ttc",
    "/System/Library/Fonts/Hiragino Sans GB.ttc",
    "/System/Library/Fonts/STHeiti Light.ttc",
    "/System/Library/Fonts/Supplemental/Songti.ttc",
]
font = None
for path in FONT_CANDIDATES:
    try:
        font = ImageFont.truetype(path, 16)
        print("字体:", path)
        break
    except Exception:
        continue
assert font, "找不到可用的中文字体"

CELL = 16
rows = []
for ch in chars:
    img = Image.new("1", (CELL, CELL), 0)
    d = ImageDraw.Draw(img)
    d.text((CELL // 2, CELL // 2), ch, font=font, fill=1, anchor="mm")
    bits = []
    for y in range(CELL):
        v = 0
        for x in range(CELL):
            if img.getpixel((x, y)):
                v |= 0x8000 >> x
        bits.append(v)
    rows.append((ch, bits))

with open(OUT, "w") as f:
    f.write("// 自动生成:tools/gen_menufont.py(字符集来自 tools/romlist.txt + UI 文案)\n")
    f.write("// 16x16 1bpp 点阵,行主序 MSB-first。手工修改会被覆盖。\n")
    f.write("#pragma once\n#include <stdint.h>\n\n")
    f.write("typedef struct { uint32_t ucs; const uint16_t glyph[16]; } menu_glyph_t;\n\n")
    f.write(f"#define MENU_GLYPH_COUNT {len(rows)}\n")
    f.write("static const menu_glyph_t k_menu_glyphs[] = {\n")
    for ch, bits in rows:
        hexbits = ",".join(f"0x{b:04x}" for b in bits)
        f.write(f'    {{ 0x{ord(ch):04x}, {{{hexbits}}} }},  /* {ch} */\n')
    f.write("};\n")
print(f"生成 {len(rows)} 个字形 -> main/menufont.h")
