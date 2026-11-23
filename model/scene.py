"""
Synthetic surveillance-like scene with exact ground truth.

Contains: textured static background, sensor noise, slow global
illumination drift, a bimodal "flickering" region (e.g. a monitor or
waving foliage) that a single-Gaussian model cannot learn, and several
textured objects moving with constant velocity and bouncing at borders.
"""
import numpy as np


class Scene:
    def __init__(self, width=160, height=120, n_obj=3, seed=1,
                 noise_sigma=3.0, warmup=20, drift=6.0):
        self.w, self.h = width, height
        self.rng = np.random.default_rng(seed)
        self.noise = noise_sigma
        self.warmup = warmup
        self.drift = drift
        yy, xx = np.mgrid[0:height, 0:width]
        base = 70 + 80 * xx / width + 30 * np.sin(yy / height * 3.1)
        tex = self.rng.normal(0, 12, (height // 4 + 1, width // 4 + 1))
        tex = np.kron(tex, np.ones((4, 4)))[:height, :width]
        bg = np.stack([base + tex, base * 0.9 + tex, base * 0.8 + 0.5 * tex], -1)
        self.bg = np.clip(bg, 0, 255)
        # bimodal region (top-right corner)
        self.bim = np.zeros((height, width), bool)
        self.bim[height // 12: height // 3, int(width * 0.70): int(width * 0.92)] = True
        self.bim_alt = np.clip(self.bg + np.array([60, 55, 40]), 0, 255)
        # moving objects
        self.objs = []
        sz = max(6, min(width, height) // 8)
        for i in range(n_obj):
            ow = int(sz * self.rng.uniform(0.8, 1.6))
            oh = int(sz * self.rng.uniform(1.0, 2.0))
            x = self.rng.uniform(0, width - ow)
            y = self.rng.uniform(height * 0.35, height - oh)
            vx = self.rng.choice([-1, 1]) * self.rng.uniform(0.6, 1.6) * width / 160
            vy = self.rng.choice([-1, 1]) * self.rng.uniform(0.2, 0.8) * height / 120
            col = self.rng.uniform(0, 255, 3)
            otex = self.rng.normal(0, 10, (oh, ow, 1))
            ell = i % 2 == 1
            self.objs.append(dict(x=x, y=y, w=ow, h=oh, vx=vx, vy=vy,
                                  col=col, tex=otex, ell=ell, t0=warmup + 4 * i))
        self.t = 0

    def frame(self):
        """-> (rgbx uint8 (H,W,4), gt mask bool (H,W))"""
        t = self.t
        img = self.bg.copy()
        flick = self.rng.random((self.h, self.w)) < 0.5
        sel = self.bim & flick
        img[sel] = self.bim_alt[sel]
        img = img + self.drift * np.sin(2 * np.pi * t / 400.0)
        gt = np.zeros((self.h, self.w), bool)
        self.last_boxes = []
        for o in self.objs:
            if t < o["t0"]:
                continue
            x0, y0 = int(round(o["x"])), int(round(o["y"]))
            yy, xx = np.mgrid[0:o["h"], 0:o["w"]]
            if o["ell"]:
                m = ((xx - o["w"] / 2 + .5) / (o["w"] / 2)) ** 2 + \
                    ((yy - o["h"] / 2 + .5) / (o["h"] / 2)) ** 2 <= 1
            else:
                m = np.ones((o["h"], o["w"]), bool)
            sl = (slice(y0, y0 + o["h"]), slice(x0, x0 + o["w"]))
            img[sl][m] = (o["col"] + o["tex"])[m]
            gt[sl] |= m
            ys, xs = np.nonzero(m)
            self.last_boxes.append((x0 + xs.min(), x0 + xs.max(), y0 + ys.min(), y0 + ys.max()))
            # move
            o["x"] += o["vx"]
            o["y"] += o["vy"]
            if o["x"] < 0 or o["x"] > self.w - o["w"]:
                o["vx"] = -o["vx"]
                o["x"] = np.clip(o["x"], 0, self.w - o["w"])
            if o["y"] < self.h * 0.2 or o["y"] > self.h - o["h"]:
                o["vy"] = -o["vy"]
                o["y"] = np.clip(o["y"], self.h * 0.2, self.h - o["h"])
        img = img + self.rng.normal(0, self.noise, img.shape)
        img = np.clip(np.round(img), 0, 255).astype(np.uint8)
        rgbx = np.zeros((self.h, self.w, 4), np.uint8)
        rgbx[..., :3] = img
        self.t += 1
        return rgbx, gt
