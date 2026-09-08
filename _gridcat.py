#!/usr/bin/env python3
"""把多帧三栏对比图(cmp3_fNNNN.png)拼成网格总览图, 每格标注帧号, 方便横向比较动作序列。
用法: python3 _gridcat.py <src_dir> <out.png> <frames_csv> [cols]"""
import sys, os
from PIL import Image, ImageDraw

src, out, frames_csv = sys.argv[1], sys.argv[2], sys.argv[3]
cols = int(sys.argv[4]) if len(sys.argv) > 4 else 6
frames = [int(x) for x in frames_csv.split(",")]

imgs = []
for f in frames:
    fp = f"{src}/cmp3_f{f:04d}.png"
    if not os.path.exists(fp):
        print(f"skip f{f}: {fp} not found")
        continue
    imgs.append((f, Image.open(fp).convert("RGB")))
if not imgs:
    sys.exit("no images")

CELL_W = 480                      # 每格宽(三栏缩略)
th = imgs[0][1].height
CELL_H = round(th * CELL_W / imgs[0][1].width)
PAD, LABEL_H = 6, 34
rows = (len(imgs) + cols - 1) // cols
W = cols * (CELL_W + PAD) + PAD
H = rows * (CELL_H + LABEL_H + PAD) + PAD
canvas = Image.new("RGB", (W, H), (24, 24, 28))
draw = ImageDraw.Draw(canvas)
for i, (f, im) in enumerate(imgs):
    r, c = divmod(i, cols)
    x = PAD + c * (CELL_W + PAD)
    y = PAD + r * (CELL_H + LABEL_H + PAD)
    draw.text((x + 4, y + 6), f"frame {f}", fill=(255, 210, 90))
    canvas.paste(im.resize((CELL_W, CELL_H), Image.LANCZOS), (x, y + LABEL_H))
canvas.save(out)
print("saved", out, f"({len(imgs)} frames, {cols} cols)")
