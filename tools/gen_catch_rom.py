#!/usr/bin/env python3
# tools/gen_catch_rom.py
# 生成第二个自制 demo ROM:接宝石(无版权问题)。
#   宝石从顶部随机落下,左右移动底部的挡板接住,接住"叮"一声并换位置。
#   左/右方向键移动挡板。复用 gen_test_rom.py 的极简 6502 汇编器。
#
#   用法: python3 tools/gen_catch_rom.py [输出路径]   # 默认 roms/catch.nes

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_test_rom import Assembler, PRG_BASE

ROM_SOURCE = """
PPUCTRL   = $2000
PPUMASK   = $2001
PPUSTATUS = $2002
OAMADDR   = $2003
OAMDMA    = $4014
PAD1      = $4016

gemY  = $00
gemX  = $01
padX  = $02
frame = $03
tmp   = $04
BTR   = $0300
BTL   = $0301
BTD   = $0302
BTU   = $0303

  .org $C000

RESET:
  sei
  cld
  ldx #$40
  stx $4017
  ldx #$FF
  txs
w1:
  lda PPUSTATUS
  bpl w1
w2:
  lda PPUSTATUS
  bpl w2
  lda PPUSTATUS
  lda #$3F
  sta $2006
  lda #$00
  sta $2006
  ldx #$00
pal:
  lda PalData,X
  sta $2007
  inx
  cpx #$20
  bne pal
  lda #$20
  sta $2006
  lda #$00
  sta $2006
  ldy #$04
ntr:
  ldx #$00
ntc:
  lda #$00
  sta $2007
  inx
  bne ntc
  dey
  bne ntr
  lda #$70
  sta gemY
  lda #$70
  sta gemX
  lda #$70
  sta padX
  lda #$00
  sta frame
  ldx #$00
oi:
  lda #$F8
  sta $0200,X
  txa
  cmp #$00
  sbc #$FC
  tax
  bne oi
  lda #$01
  sta $0201
  lda #$00
  sta $0202
  lda #$02
  sta $0205
  sta $0209
  lda #$01
  sta $0206
  sta $020A
  lda #$80
  sta PPUCTRL
  lda #$1E
  sta PPUMASK
loop:
  lda PPUSTATUS
  bpl loop
  jmp loop

NMI:
  pha
  txa
  pha
  lda #$00
  sta OAMADDR
  lda #$02
  sta OAMDMA
  lda #$00
  lsr a
  lda frame
  adc #$01
  sta frame
  jsr ReadPad
  jsr UpdatePad
  jsr UpdateGem
  lda gemY
  sta $0200
  lda gemX
  sta $0203
  lda #$D0
  sta $0204
  sta $0208
  lda padX
  sta $0207
  lda #$00
  lsr a
  lda padX
  adc #$08
  sta $020B
  pla
  tax
  pla
  rti

ReadPad:
  lda #$01
  sta PAD1
  lda #$00
  sta PAD1
  ldx #$08
rp:
  lda PAD1
  and #$01
  sta $02FF,X
  dex
  bne rp
  rts

UpdatePad:
  lda BTL
  beq noL
  lda padX
  cmp #$10
  bcc noL
  lda padX
  cmp #$00
  sbc #$02
  sta padX
noL:
  lda BTR
  beq noR
  lda padX
  cmp #$E0
  bcs noR
  lda #$00
  lsr a
  lda padX
  adc #$02
  sta padX
noR:
  rts

UpdateGem:
  lda #$00
  lsr a
  lda gemY
  adc #$02
  sta gemY
  cmp #$E8
  bcs Respawn
  cmp #$C8
  bcc DoneG
  lda gemX
  cmp #$00
  sbc padX
  bcs gr
  cmp #$F8
  bcs Catch
  jmp DoneG
gr:
  cmp #$09
  bcc Catch
  jmp DoneG
Catch:
  lda #$B8
  sta $4000
  lda #$DF
  sta $4002
  lda #$0F
  sta $4003
Respawn:
  lda #$08
  sta gemY
  lda frame
  eor gemX
  eor padX
  and #$7F
  sta tmp
  lda #$00
  lsr a
  lda tmp
  adc #$30
  sta gemX
DoneG:
  rts

PalData:
  .byte $0F,$0F,$0F,$0F, $0F,$0F,$0F,$0F
  .byte $0F,$0F,$0F,$0F, $0F,$0F,$0F,$0F
  .byte $0F,$27,$0F,$0F, $0F,$30,$0F,$0F
  .byte $0F,$0F,$0F,$0F, $0F,$0F,$0F,$0F

  .org $FFFA
  .word NMI
  .word RESET
  .word $C060
"""


def build_chr():
    """tile0=空 tile1=宝石(菱形) tile2=挡板(实心条),其余铺空。1bpp+0 平面。"""
    blank = bytes(8)
    gem = bytes([0x18, 0x3C, 0x7E, 0xFF, 0xFF, 0x7E, 0x3C, 0x18])
    bar = bytes([0xFF] * 8)
    def tile(a, b):
        return a + b
    base = tile(blank, blank) + tile(gem, bytes(8)) + tile(bar, bytes(8)) + tile(blank, blank)
    return (base * 128)[:0x2000]


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'roms', 'catch.nes')

    prg = Assembler(ROM_SOURCE).assemble()
    chr_data = build_chr()
    assert len(prg) == 0x4000 and len(chr_data) == 0x2000

    header = b'NES\x1A' + bytes([1, 1, 0x01, 0x00]) + bytes(8)
    rom = header + prg + chr_data
    assert len(rom) == 16 + 16384 + 8192

    vbase = 16 + 0x3FFA
    for name, off in (('NMI', 0), ('RESET', 2), ('IRQ', 4)):
        vec = rom[vbase + off] | (rom[vbase + off + 1] << 8)
        assert 0xC000 <= vec <= 0xFFFF, f'{name} 向量非法: ${vec:04X}'
    print(f'OK: {out_path} ({len(rom)} bytes, NMI=${rom[vbase] | rom[vbase+1] << 8:04X})')

    with open(out_path, 'wb') as f:
        f.write(rom)


if __name__ == '__main__':
    main()
