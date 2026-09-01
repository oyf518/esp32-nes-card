#!/usr/bin/env python3
# tools/gen_test_rom.py
# 生成自制 NES 测试 ROM(无版权问题),放进 roms/ 目录后由 ROM 库清单
# (tools/romlist.txt)打包,或用来验证固件菜单/加载链路。
#
#   用法: python3 tools/gen_test_rom.py [输出路径]   # 默认 roms/test_rom.nes
#
# 产物:NROM-128 iNES 文件 = 16 字节头 + 16KB PRG + 8KB CHR,mapper 0,垂直镜像,
#       NMI/RESET/IRQ 向量在 $FFFA-$FFFF(即 16KB PRG 的尾部)。
#
# 内置一个极简两遍 6502 汇编器,仅支持以下子集(对本 ROM 够用):
#   LDA/LDX/LDY: imm/zp/abs/abs,X/abs,Y(STX/STY 无 abs,X,按真实 6502 校验)
#   STA/STX/STY: zp/abs/abs,X/abs,Y(同上,非法组合报错)
#   INX/INY/DEX/DEY、TAX/TXA/TAY/TYA/TSX/TXS
#   CMP/CPX/CPY imm、AND/ORA/EOR/ADC/SBC imm(外加 zp/abs 以备用)
#   ASL/LSR A
#   BNE/BEQ/BPL/BMI/BCC/BCS rel(标签)
#   JMP abs、JSR abs、RTS、SEI/CLI/CLD、NOP
#   (另加 RTI:NMI/IRQ 返回必需,任务清单的子集漏了它)
#   伪指令:.org / .byte / .word、NAME = 表达式(zp 变量用)、标签 name:
#
# ROM 行为:棋盘格背景 + 顶部一行数字 0-9;方向键移动笑脸精灵(边界钳制),
# A 键循环切换背景色;NMI 里做 OAM DMA 与调色板更新。

import os
import sys

# ---------------------------------------------------------------------------
# 极简两遍 6502 汇编器
# ---------------------------------------------------------------------------

# 寻址模式 -> 操作码。zp 与 abs 的区分规则:操作数是数值/常量且 < $100 用 zp,
# 是代码/数据标签(>= $C000)或前向引用用 abs。
OPS = {
    'LDA': {'imm': 0xA9, 'zp': 0xA5, 'abs': 0xAD, 'absx': 0xBD, 'absy': 0xB9},
    'LDX': {'imm': 0xA2, 'zp': 0xA6, 'abs': 0xAE, 'absy': 0xBE},
    'LDY': {'imm': 0xA0, 'zp': 0xA4, 'abs': 0xAC, 'absx': 0xBC},
    'STA': {'zp': 0x85, 'abs': 0x8D, 'absx': 0x9D, 'absy': 0x99},
    'STX': {'zp': 0x86, 'abs': 0x8E, 'absy': 0x96},          # 6502 无 STX abs,X
    'STY': {'zp': 0x84, 'abs': 0x8C, 'absx': 0x94},          # 6502 无 STY abs,Y
    'CMP': {'imm': 0xC9, 'zp': 0xC5, 'abs': 0xCD},
    'CPX': {'imm': 0xE0, 'zp': 0xE4, 'abs': 0xEC},
    'CPY': {'imm': 0xC0, 'zp': 0xC4, 'abs': 0xCC},
    'AND': {'imm': 0x29, 'zp': 0x25, 'abs': 0x2D},
    'ORA': {'imm': 0x09, 'zp': 0x05, 'abs': 0x0D},
    'EOR': {'imm': 0x49, 'zp': 0x45, 'abs': 0x4D},
    'ADC': {'imm': 0x69, 'zp': 0x65, 'abs': 0x6D},
    'SBC': {'imm': 0xE9, 'zp': 0xE5, 'abs': 0xED},
    'ASL': {'acc': 0x0A},
    'LSR': {'acc': 0x4A},
    'BNE': {'rel': 0xD0}, 'BEQ': {'rel': 0xF0},
    'BPL': {'rel': 0x10}, 'BMI': {'rel': 0x30},
    'BCC': {'rel': 0x90}, 'BCS': {'rel': 0xB0},
    'JMP': {'abs': 0x4C},
    'JSR': {'abs': 0x20},
}
IMPLIED = {  # 无操作数指令
    'INX': 0xE8, 'INY': 0xC8, 'DEX': 0xCA, 'DEY': 0x88,
    'TAX': 0xAA, 'TXA': 0x8A, 'TAY': 0xA8, 'TYA': 0x98,
    'TSX': 0xBA, 'TXS': 0x9A,
    'RTS': 0x60, 'RTI': 0x40, 'SEI': 0x78, 'CLI': 0x58, 'CLD': 0xD8, 'NOP': 0xEA,
}

PRG_BASE = 0xC000   # NROM-128:16KB PRG 镜像在 $8000/$C000,取高半区写向量方便


class AsmError(Exception):
    pass


def parse_value(tok, symbols, pass2):
    """解析 $hex / %bin / 十进制 / 符号。pass1 里未定义符号按 0 估(占 abs 两字节)。"""
    tok = tok.strip()
    if tok.startswith('$'):
        return int(tok[1:], 16), True
    if tok.startswith('%'):
        return int(tok[1:], 2), True
    if tok.isdigit():
        return int(tok), True
    if tok in symbols:
        return symbols[tok], True
    if pass2:
        raise AsmError(f'未定义符号: {tok}')
    return 0, False   # pass1 未知符号:按 abs 处理


class Assembler:
    def __init__(self, source):
        self.lines = source.splitlines()
        self.symbols = {}
        self.prg = bytearray(PRG_BASE and 0x4000)  # 16KB
        self.pc = PRG_BASE

    def _offset(self, addr):
        off = addr - PRG_BASE
        if not (0 <= off < 0x4000):
            raise AsmError(f'地址 ${addr:04X} 超出 PRG 范围 ($C000-$FFFF)')
        return off

    def emit(self, *bytes_):
        off = self._offset(self.pc)
        for b in bytes_:
            self.prg[off] = b & 0xFF
            off += 1
            self.pc += 1

    def assemble(self):
        for pass2 in (False, True):
            self.pc = PRG_BASE
            for lineno, raw in enumerate(self.lines, 1):
                try:
                    self._line(raw, pass2)
                except AsmError as e:
                    raise AsmError(f'第 {lineno} 行 `{raw.strip()}`: {e}') from e
        return bytes(self.prg)

    def _line(self, raw, pass2):
        line = raw.split(';', 1)[0].strip()
        if not line:
            return
        # 标签(可独占一行或前缀在指令前)
        while True:
            head, sep, rest = line.partition(':')
            if sep and head.strip().isidentifier() and not head.strip().upper() in OPS \
                    and not head.strip().upper() in IMPLIED:
                name = head.strip()
                if not pass2:
                    self.symbols[name] = self.pc
                line = rest.strip()
                if not line:
                    return
            else:
                break
        # 常量定义: NAME = expr
        if '=' in line and not line.startswith('.'):
            name, _, expr = line.partition('=')
            val, _ = parse_value(expr, self.symbols, pass2)
            self.symbols[name.strip()] = val
            return
        # 伪指令
        if line.startswith('.'):
            parts = line.split(None, 1)
            directive = parts[0].lower()
            args = parts[1] if len(parts) > 1 else ''
            if directive == '.org':
                val, _ = parse_value(args, self.symbols, pass2)
                self.pc = val
            elif directive == '.byte':
                for tok in args.split(','):
                    val, _ = parse_value(tok, self.symbols, pass2)
                    self.emit(val)
            elif directive == '.word':
                for tok in args.split(','):
                    val, _ = parse_value(tok, self.symbols, pass2)
                    self.emit(val & 0xFF, val >> 8)
            else:
                raise AsmError(f'不支持的伪指令 {directive}')
            return
        # 指令
        parts = line.split(None, 1)
        op = parts[0].upper()
        operand = parts[1].strip() if len(parts) > 1 else ''
        if op in IMPLIED:
            if operand:
                raise AsmError(f'{op} 不需要操作数')
            self.emit(IMPLIED[op])
            return
        if op not in OPS:
            raise AsmError(f'不支持的指令 {op}')
        modes = OPS[op]
        # 累加器模式: ASL A / LSR A
        if operand.upper() == 'A':
            if 'acc' not in modes:
                raise AsmError(f'{op} 不支持累加器模式')
            self.emit(modes['acc'])
            return
        # 分支:相对跳转,操作数必须是标签
        if 'rel' in modes:
            target, _ = parse_value(operand, self.symbols, pass2)
            if pass2:
                rel = target - (self.pc + 2)
                if not (-128 <= rel <= 127):
                    raise AsmError(f'分支超出范围: {rel}')
                self.emit(modes['rel'], rel & 0xFF)
            else:
                self.emit(modes['rel'], 0)
            return
        # imm / zp / abs / abs,X / abs,Y
        if operand.startswith('#'):
            if 'imm' not in modes:
                raise AsmError(f'{op} 不支持立即数')
            val, _ = parse_value(operand[1:], self.symbols, pass2)
            self.emit(modes['imm'], val & 0xFF)
            return
        suffix = ''
        base = operand
        for sfx, mode in ((',X', 'absx'), (',x', 'absx'), (',Y', 'absy'), (',y', 'absy')):
            if operand.endswith(sfx):
                base = operand[:-2].strip()
                suffix = mode
                break
        val, known = parse_value(base, self.symbols, pass2)
        if suffix:
            if suffix not in modes:
                raise AsmError(f'{op} 不支持 {operand} 这种寻址(查 6502 手册)')
            self.emit(modes[suffix], val & 0xFF, val >> 8)
            return
        # 无后缀:已知且 < $100 且支持 zp 时用 zp,否则 abs
        if known and val < 0x100 and 'zp' in modes:
            self.emit(modes['zp'], val)
        elif 'abs' in modes:
            self.emit(modes['abs'], val & 0xFF, val >> 8)
        else:
            raise AsmError(f'{op} 不支持该寻址模式')


# ---------------------------------------------------------------------------
# ROM 程序(6502 汇编)
# ---------------------------------------------------------------------------

ROM_SOURCE = r"""
; ---- zeropage 变量 ----
buttons  = $10        ; 本帧手柄位:A=$80 B=$40 SEL=$20 STA=$10 上=$08 下=$04 左=$02 右=$01
prev_a   = $11        ; A 键上一帧状态(边沿检测)
sprite_x = $12
sprite_y = $13
bgcolor  = $14        ; 背景色(A 键循环切换)
paldirty = $15        ; 非 0:NMI 里把 bgcolor 写入 $3F00
nmidone  = $16        ; 帧同步标志
save_a   = $17        ; NMI 现场保护(指令子集无 PHA/PLA,存 zp)
save_x   = $18
save_y   = $19

        .org $C000

; ---- RESET:标准初始化 ----
reset:
        SEI
        CLD
        LDX #$FF
        TXS
        LDA #$00
        STA $2000             ; 关 NMI
        STA $2001             ; 关渲染
        LDX #$00
        TXA
clr_ram:                      ; 清 2KB RAM
        STA $0000,X
        STA $0100,X
        STA $0200,X
        STA $0300,X
        STA $0400,X
        STA $0500,X
        STA $0600,X
        STA $0700,X
        INX
        BNE clr_ram
wait_v1:                      ; 等两次 vblank(PPU 预热)
        LDA $2002
        BPL wait_v1
wait_v2:
        LDA $2002
        BPL wait_v2
        ; 写 32 字节调色板 -> $3F00
        LDA #$3F
        STA $2006
        LDA #$00
        STA $2006
        LDX #$00
pal_loop:
        LDA palette,X
        STA $2007
        INX
        CPX #$20
        BNE pal_loop
        ; nametable0:$2000 起 1024 字节填 tile2(像素级棋盘格),属性区随后清零
        LDA #$20
        STA $2006
        LDA #$00
        STA $2006
        LDX #$00
        LDY #$04
        LDA #$02
nt_loop:
        STA $2007
        INX
        BNE nt_loop
        DEY
        BNE nt_loop
        ; 属性表 $23C0 清零(全部走 palette0)
        LDA #$23
        STA $2006
        LDA #$C0
        STA $2006
        LDX #$40
        LDA #$00
attr_loop:
        STA $2007
        DEX
        BNE attr_loop
        ; 第 2 行写数字 0-9(tile $10..$19)
        LDA #$20
        STA $2006
        LDA #$41
        STA $2006
        LDX #$00
dig_loop:
        LDA digits,X
        STA $2007
        INX
        CPX #$0A
        BNE dig_loop
        ; 精灵 0:笑脸 tile $05,初始 (120,120)
        LDA #$78
        STA sprite_x
        STA sprite_y
        LDA #$05
        STA $0201             ; tile
        LDA #$00
        STA $0202             ; attr
        ; scroll 归 0,开 NMI + 背景 + 精灵(含左列)
        LDA #$00
        STA $2005
        STA $2005
        STA bgcolor
        LDA #$88
        STA $2000
        LDA #$1E
        STA $2001

; ---- 主循环:读手柄 -> 移动精灵 -> 等 NMI ----
main_loop:
        JSR read_pad
        ; -- 上(位定义见 read_pad:上=$08 下=$04 左=$02 右=$01 A=$80) --
        LDA buttons
        AND #$08
        BEQ no_up
        LDA sprite_y
        BEQ no_up             ; y==0 到顶
        CMP #$00              ; 子集无 SEC:CMP #0 恒置 C=1,供 SBC 用
        SBC #$02
        STA sprite_y
no_up:
        ; -- 下(上限 231) --
        LDA buttons
        AND #$04
        BEQ no_down
        LDA sprite_y
        CMP #$E7
        BCS no_down
        LDA #$00              ; 子集无 CLC:A=0 后 LSR 恒清 C
        LSR A
        LDA sprite_y
        ADC #$02
        STA sprite_y
no_down:
        ; -- 左 --
        LDA buttons
        AND #$02
        BEQ no_left
        LDA sprite_x
        BEQ no_left
        CMP #$00
        SBC #$02
        STA sprite_x
no_left:
        ; -- 右(上限 239) --
        LDA buttons
        AND #$01
        BEQ no_right
        LDA sprite_x
        CMP #$EF
        BCS no_right
        LDA #$00
        LSR A
        LDA sprite_x
        ADC #$02
        STA sprite_x
no_right:
        ; -- A 键($80)边沿:循环切换背景色 --
        LDA buttons
        AND #$80
        BEQ a_released
        LDA prev_a
        BNE a_done            ; 已按住,忽略
        LDA #$01
        STA prev_a
        LDA #$00
        LSR A                 ; 清 C
        LDA bgcolor
        ADC #$01
        AND #$3F              ; 0..63 循环
        STA bgcolor
        LDA #$01
        STA paldirty
        JMP a_done
a_released:
        LDA #$00
        STA prev_a
a_done:
        ; 写回精灵坐标($0200 页,NMI 做 OAM DMA)
        LDA sprite_y
        STA $0200
        LDA sprite_x
        STA $0203
        ; 帧同步:等 NMI 置标志
        LDA #$00
        STA nmidone
wait_nmi:
        LDA nmidone
        BEQ wait_nmi
        JMP main_loop

; ---- 读手柄:标准锁存 + 移位读 8 位 ----
; 子集无 ROL 也无 abs,X,改为 buttons 左移累积:第 i 次读到的位最终落在 bit(7-i),
; 即 A=$80 B=$40 SEL=$20 STA=$10 上=$08 下=$04 左=$02 右=$01。
read_pad:
        LDA #$01
        STA $4016
        LDA #$00
        STA $4016             ; 锁存
        STA buttons
        LDX #$08
pad_loop:
        LDA buttons
        ASL A
        STA buttons
        LDA $4016
        LSR A                 ; C = 当前位(顺序 A,B,SEL,STA,U,D,L,R)
        BCC pad_next
        LDA buttons
        ORA #$01
        STA buttons
pad_next:
        DEX
        BNE pad_loop
        RTS

; ---- NMI:OAM DMA + 调色板更新 + 帧标志 ----
nmi:
        STA save_a
        STX save_x
        STY save_y
        LDA #$02
        STA $4014             ; OAM DMA:$0200 页 -> PPU
        LDA paldirty
        BEQ nmi_done
        LDA #$3F              ; 写背景色 -> $3F00
        STA $2006
        LDA #$00
        STA $2006
        LDA bgcolor
        STA $2007
        LDA #$00
        STA paldirty
        STA $2005             ; $2006 写破坏了内部锁存,恢复 scroll=0
        STA $2005
nmi_done:
        LDA #$01
        STA nmidone
        LDA save_a
        LDX save_x
        LDY save_y
        RTI

irq:
        RTI

; ---- 数据 ----
palette:                      ; 4 组背景 + 4 组精灵
        .byte $0F,$00,$10,$30 ; bg0 黑/灰/浅灰/白(棋盘格与数字用)
        .byte $0F,$16,$27,$18
        .byte $0F,$09,$19,$29
        .byte $0F,$0C,$1C,$2C
        .byte $0F,$16,$27,$30 ; sp0(笑脸:红/橙/白)
        .byte $0F,$12,$22,$32
        .byte $0F,$14,$24,$34
        .byte $0F,$1A,$2A,$3A
digits:
        .byte $10,$11,$12,$13,$14,$15,$16,$17,$18,$19

; ---- 向量($FFFA-$FFFF = 16KB PRG 尾部) ----
        .org $FFFA
        .word nmi, reset, irq
"""

# ---------------------------------------------------------------------------
# CHR 生成:tile0 空白、tile1 实心、tile2/3 棋盘格、tile4 边框、
# tile5 笑脸精灵、tile $10..$19 数字 0-9(3x5 点阵)
# ---------------------------------------------------------------------------

def tile_from_pixels(px):
    """8x8 像素(值 0..3)-> 16 字节 tile(plane0 八行 + plane1 八行,bit7=最左)。"""
    out = bytearray()
    for plane in (0, 1):
        for y in range(8):
            b = 0
            for x in range(8):
                if (px[y][x] >> plane) & 1:
                    b |= 0x80 >> x
            out.append(b)
    return bytes(out)


def rows(*patterns):
    """把 '3...2...' 之类的字符串行转成像素值列表(字符 '0'..'3' 或 '.'=0)。"""
    px = []
    for p in patterns:
        assert len(p) == 8, f'行宽必须 8: {p!r}'
        px.append([0 if c == '.' else int(c) for c in p])
    return px


def digit_tile(bitmap5):
    """3x5 数字点阵放进 8x8(x 偏移 2,y 偏移 1),像素值 3。"""
    px = [[0] * 8 for _ in range(8)]
    for y, rowbits in enumerate(bitmap5):
        for x in range(3):
            if (rowbits >> (2 - x)) & 1:
                px[1 + y][2 + x] = 3
    return tile_from_pixels(px)


DIGITS_3X5 = [  # 每个数字 5 行,每行 3 bit
    [0b111, 0b101, 0b101, 0b101, 0b111],  # 0
    [0b010, 0b110, 0b010, 0b010, 0b111],  # 1
    [0b111, 0b001, 0b111, 0b100, 0b111],  # 2
    [0b111, 0b001, 0b111, 0b001, 0b111],  # 3
    [0b101, 0b101, 0b111, 0b001, 0b001],  # 4
    [0b111, 0b100, 0b111, 0b001, 0b111],  # 5
    [0b111, 0b100, 0b111, 0b101, 0b111],  # 6
    [0b111, 0b001, 0b001, 0b010, 0b010],  # 7
    [0b111, 0b101, 0b111, 0b101, 0b111],  # 8
    [0b111, 0b101, 0b111, 0b001, 0b111],  # 9
]


def gen_chr():
    tiles = [bytes(16)] * 512                      # 8KB CHR = 512 tiles,默认全空
    tiles[0] = bytes(16)                           # tile0 空白
    tiles[1] = tile_from_pixels(rows(              # tile1 实心块
        '33333333', '33333333', '33333333', '33333333',
        '33333333', '33333333', '33333333', '33333333'))
    checker1 = [[1 if (x + y) % 2 == 0 else 2 for x in range(8)] for y in range(8)]
    checker2 = [[2 if (x + y) % 2 == 0 else 1 for x in range(8)] for y in range(8)]
    tiles[2] = tile_from_pixels(checker1)          # tile2 棋盘格(背景用)
    tiles[3] = tile_from_pixels(checker2)          # tile3 反相棋盘格
    tiles[4] = tile_from_pixels(rows(              # tile4 边框
        '33333333', '3.......', '3.......', '3.......',
        '3.......', '3.......', '3.......', '33333333'))
    tiles[5] = tile_from_pixels(rows(              # tile5 笑脸(精灵)
        '........', '.11..11.', '.11..11.', '........',
        '........', '3......3', '.333333.', '........'))
    for i in range(10):                            # tile $10..$19 数字 0-9
        tiles[0x10 + i] = digit_tile(DIGITS_3X5[i])
    return b''.join(tiles)


# ---------------------------------------------------------------------------
# 组装 iNES 文件 + 自检
# ---------------------------------------------------------------------------

def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'roms', 'test_rom.nes')

    prg = Assembler(ROM_SOURCE).assemble()
    chr_data = gen_chr()
    assert len(prg) == 0x4000 and len(chr_data) == 0x2000

    # iNES 头:'NES\x1A', 1x16KB PRG, 1x8KB CHR, flags6 bit0=1 垂直镜像, mapper 0
    header = b'NES\x1A' + bytes([1, 1, 0x01, 0x00]) + bytes(8)
    rom = header + prg + chr_data

    # ---- 自检 ----
    assert len(rom) == 16 + 16384 + 8192, f'文件大小错误: {len(rom)}'
    assert rom[:4] == b'NES\x1A', 'iNES 魔数错误'
    assert rom[4] == 1 and rom[5] == 1, 'PRG/CHR  bank 数错误'
    # 向量在 PRG 尾部($FFFA = offset 16+0x3FFA),三向量必须落在 $C000-$FFFF
    vbase = 16 + 0x3FFA
    for name, off in (('NMI', 0), ('RESET', 2), ('IRQ', 4)):
        vec = rom[vbase + off] | (rom[vbase + off + 1] << 8)
        assert 0xC000 <= vec <= 0xFFFF, f'{name} 向量非法: ${vec:04X}'
    print(f'OK: {out_path} ({len(rom)} bytes, NMI=${rom[vbase] | rom[vbase+1] << 8:04X})')

    with open(out_path, 'wb') as f:
        f.write(rom)


if __name__ == '__main__':
    main()
