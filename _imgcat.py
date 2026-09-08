"""拼接竖幅栏为横幅对比图(自动检测 L/M/R 栏存在性, 两栏→cmp2_, 三栏→cmp3_)
可选第 5 参数 prefix, 第 6 参数栏标签(如 "源BVH|HS2骨架|HS2模型", 顶部加标签条)"""
import sys, os
from PIL import Image, ImageDraw, ImageFont

tmp = sys.argv[1]
out = sys.argv[2]
frames = [int(x) for x in sys.argv[3].split(",")]
prefix = sys.argv[4] if len(sys.argv) > 4 and sys.argv[4] else None
labels = sys.argv[5].split("|") if len(sys.argv) > 5 else None

try:
    _font = ImageFont.truetype("/System/Library/Fonts/PingFang.ttc", 28)
except Exception:
    try:
        _font = ImageFont.truetype("/System/Library/Fonts/Hiragino Sans GB.ttc", 28)
    except Exception:
        _font = ImageFont.load_default()

for f in frames:
    bars = []
    for suf in ("L", "M", "R"):
        fp = f"{tmp}/f{f:04d}_{suf}.png"
        if os.path.exists(fp):
            bars.append(Image.open(fp).convert("RGB"))
    if not bars:
        print(f"skip f{f}: no bars found")
        continue
    w = sum(b.width for b in bars)
    h = max(b.height for b in bars)
    LB = 44 if labels else 0
    canvas = Image.new("RGB", (w, h + LB), (24, 24, 28))
    draw = ImageDraw.Draw(canvas)
    x = 0
    for i, b in enumerate(bars):
        if labels and i < len(labels):
            tw = draw.textlength(labels[i], font=_font)
            draw.text((x + (b.width - tw) / 2, 6), labels[i],
                      fill=(235, 235, 240), font=_font)
        canvas.paste(b, (x, LB + (h - b.height) // 2))
        x += b.width
    p = prefix or f"cmp{len(bars)}"
    fp = f"{out}/{p}_f{f:04d}.png"
    canvas.save(fp)
    print("saved", fp)
