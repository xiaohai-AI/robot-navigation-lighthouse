#!/usr/bin/env python3
"""程序化生成 dan_house 的真实感纹理 (numpy + PIL)"""
import os
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'worlds', 'textures')
os.makedirs(OUT, exist_ok=True)
rng = np.random.default_rng(7)

def to_img(a):
    return Image.fromarray(np.clip(a, 0, 255).astype('uint8'))

def smooth_noise(n, amp, kernel=15):
    x = rng.normal(0, amp, n)
    return np.convolve(x, np.ones(kernel) / kernel, mode='same')

def wood(base, size=512, planks=16):
    img = np.zeros((size, size, 3))
    ph = size // planks
    for i in range(planks):
        shade = 0.82 + 0.36 * rng.random()
        block = np.ones((ph, size, 3)) * np.array(base) * shade
        block += smooth_noise(size, 6)[None, :, None]          # 木纹
        block += rng.normal(0, 2.5, (ph, size, 3))
        block[-2:, :, :] *= 0.5                                   # 板缝
        img[i * ph:(i + 1) * ph] = block
    return to_img(img)

def flat(base, size=512, amp=4):
    a = np.ones((size, size, 3)) * np.array(base) + rng.normal(0, amp, (size, size, 3))
    return to_img(a)

def wallpaper(size=512):
    a = np.ones((size, size, 3)) * np.array((233, 227, 214))
    for x in range(0, size, 64):
        a[:, x:x + 32] += 5
    a += rng.normal(0, 3, (size, size, 3))
    return to_img(a)

def tile(size=512, n=8):
    a = np.ones((size, size, 3)) * np.array((214, 214, 218)) + rng.normal(0, 3, (size, size, 3))
    step = size // n
    for i in range(n):
        for j in range(n):
            a[i*step:(i+1)*step, j*step:(j+1)*step] *= 0.96 + 0.08 * rng.random()
    a[::step, :, :] = (150, 150, 155)
    a[:, ::step, :] = (150, 155, 150)
    return to_img(a)

def carpet(size=512):
    a = np.ones((size, size, 3)) * np.array((128, 48, 48)) + rng.normal(0, 13, (size, size, 3))
    return to_img(a).filter(ImageFilter.GaussianBlur(1))

def fabric(base=(74, 92, 132), size=512):
    xs = np.sin(np.arange(size) * 0.9) * 3
    a = np.ones((size, size, 3)) * np.array(base) + xs[None, :, None] + xs[:, None, None]
    a += rng.normal(0, 5, (size, size, 3))
    return to_img(a)

def metal(size=512):
    a = np.ones((size, size, 3)) * np.array((203, 205, 209))
    a += smooth_noise(size, 5)[:, None, None]                    # 横向拉丝
    a += rng.normal(0, 2, (size, size, 3))
    return to_img(a)

def screen(size=512):
    y, x = np.mgrid[0:size, 0:size]
    d = np.sqrt((x - size/2)**2 + (y - size/2)**2) / (size/2)
    a = np.ones((size, size, 3)) * np.array((16, 18, 24)) + (1 - d)[:, :, None] * 26
    return to_img(a)

def bedding(size=512):
    a = np.ones((size, size, 3)) * np.array((226, 223, 216)) + rng.normal(0, 3, (size, size, 3))
    step = size // 8
    a[::step, :, :] *= 0.94
    a[:, ::step, :] *= 0.94
    return to_img(a)

def rug(size=512):
    a = np.ones((size, size, 3)) * np.array((132, 42, 42)) + rng.normal(0, 10, (size, size, 3))
    img = to_img(a)
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, size - 1, size - 1], outline=(70, 20, 20), width=36)
    dr.rectangle([48, 48, size - 49, size - 49], outline=(212, 180, 120), width=6)
    return img

def painting(size=512):
    img = Image.new('RGB', (size, size), (240, 238, 230))
    dr = ImageDraw.Draw(img)
    palette = [(180, 60, 50), (40, 80, 140), (220, 180, 60), (60, 120, 80), (90, 60, 120)]
    for _ in range(14):
        c = palette[int(rng.integers(0, len(palette)))]
        x0, y0 = [int(v) for v in rng.integers(0, size, 2)]
        w, h = [int(v) for v in rng.integers(60, 200, 2)]
        dr.rectangle([x0, y0, x0 + w, y0 + h], fill=c)
    return img


wood_floor = wood((152, 108, 66), planks=16)
wood_dark  = wood((96, 62, 38), planks=10)
files = {
    'wood_floor.jpg': wood_floor,
    'wood_dark.jpg': wood_dark,
    'wallpaper.jpg': wallpaper(),
    'tile.jpg': tile(),
    'carpet.jpg': carpet(),
    'fabric.jpg': fabric(),
    'metal.jpg': metal(),
    'screen.jpg': screen(),
    'bedding.jpg': bedding(),
    'rug.jpg': rug(),
    'painting.jpg': painting(),
    'leaf.jpg': flat((58, 118, 48), 256, 16),
    'terracotta.jpg': flat((172, 92, 58), 256, 8),
}
for name, img in files.items():
    img.save(os.path.join(OUT, name), quality=90)
print(f'✅ 已生成 {len(files)} 张纹理 -> {OUT}')
