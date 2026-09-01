# roms/ — 放你自己的游戏

把你**合法拥有**的 ROM 放到本目录（自制、免费 homebrew，或你拥有实体卡带的游戏转储），
然后在 `tools/romlist.txt` 里加一行 `roms/文件名.nes|显示名` 即可打包进 ROM 库。

- 本目录内容已被 .gitignore 排除，**任何 ROM 都不会进入 git 仓库**
- 商业 ROM 的再分发是侵权，请勿分享打包产物 rompack.bin
- 没有游戏？先跑 `python3 tools/gen_test_rom.py` 生成一个零版权测试 ROM
