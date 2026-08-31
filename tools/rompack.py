#!/usr/bin/env python3
# ROM 库打包器:把多个 .nes 打成裸分区镜像(目录表 + 连续 ROM,4KB 对齐)
# 用法: rompack.py -o rompack.bin 游戏1.nes 游戏2.nes ...
#       rompack.py -o rompack.bin -l romlist.txt   (每行: 文件路径)
# 烧录: esptool write_flash 0x190000 rompack.bin
import argparse, struct, zlib, os, sys

MAGIC = b"NESPACK1"
HDR_SIZE = 4096          # 目录区定长,ROM 数据从这里开始
ENT_SIZE = 64            # 目录项 64B: name[47]+size(4)+offset(4)+crc32(4)+pad
NAME_MAX = 47        # 可见名字最长 47 字节(结构体槽 48B,含保底\0)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("-l", "--list", help="每行一个 ROM 文件的清单(可带 |显示名)")
    ap.add_argument("roms", nargs="*", help="ROM 文件列表")
    args = ap.parse_args()

    items = []
    if args.list:
        for line in open(args.list, encoding="utf-8"):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "|" in line:
                path, disp = line.split("|", 1)
                items.append((path.strip(), disp.strip()))
            else:
                items.append((line, None))
    for p in args.roms:
        items.append((p, None))

    entries, blob = [], bytearray()
    cur = HDR_SIZE
    for path, disp in items:
        data = open(path, "rb").read()
        assert data[:4] == b"NES\x1a", f"非 iNES 文件: {path}"
        name = disp or os.path.splitext(os.path.basename(path))[0]
        nb = name.encode("utf-8")[:NAME_MAX - 1]
        off = cur
        crc = zlib.crc32(data) & 0xFFFFFFFF
        entries.append((nb, len(data), off, crc))
        # 4KB 对齐存放(esp_partition_mmap 页对齐友好)
        pad = (-len(data)) % 4096
        blob += data + b"\xff" * pad
        cur = HDR_SIZE + len(blob)
        print(f"  + {name:<20} {len(data):>9,}B  @0x{off:06x}  crc={crc:08x}")

    total = HDR_SIZE + len(blob)
    with open(args.out, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<II", len(entries), HDR_SIZE))
        f.write(b"\xff" * (HDR_SIZE - 16))
        f.seek(16)
        for nb, sz, off, crc in entries:
            # 槽位布局(与固件 rom_ent_t 对齐后完全一致): name[48] + u32 size + u32 offset + u32 crc32 = 60B,pad 到 64
            ent = nb + b"\x00" * (48 - len(nb)) + struct.pack("<III", sz, off, crc)
            f.write(ent + b"\x00" * (ENT_SIZE - len(ent)))
        f.seek(HDR_SIZE)
        f.write(blob)
    print(f"=> {args.out}: {len(entries)} 游戏, 镜像 {total:,}B ({total/1024:.0f} KB), 分区可用 5.44MB")

if __name__ == "__main__":
    main()
