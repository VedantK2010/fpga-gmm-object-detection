"""
PYNQ driver for the moving-object-detection overlay
(OpenCV GMM in the hardware formulation of Genovese & Napoli
 + morphology + single-pass CCL).

Copy mod_overlay.bit, mod_overlay.hwh, mod_driver.py and mod_model.py into
one folder on the board (e.g. /home/xilinx/jupyter_notebooks/mod/).
"""
import time

import numpy as np
from pynq import Overlay, allocate

# register map (byte offsets)
REG_CTRL, REG_STATUS, REG_WIDTH, REG_HEIGHT = 0x000, 0x004, 0x008, 0x00C
REG_SHIFTS, REG_LAMBDA2, REG_T_BG, REG_W_INIT = 0x010, 0x014, 0x018, 0x01C
REG_VAR_INIT, REG_VAR_MIN, REG_VAR_MAX, REG_MIN_AREA = 0x020, 0x024, 0x028, 0x02C
REG_NUM_OBJ, REG_FG_COUNT, REG_ID, REG_CAPS = 0x030, 0x034, 0x038, 0x03C
REG_GMM_FMT = 0x040
REG_OBJ_BASE = 0x200
CORE_ID = 0x4D4F4401

DEFAULTS = dict(alpha_sh=7,          # alpha_w = 2^-7 (OpenCV default 1/200)
                lambda2=100,         # Q4.4, lambda = 2.5
                t_bg=2867,           # Q0.12, T = 0.7
                var_init=900,        # sigma_init = 30 (OpenCV)
                var_min=16, var_max=16383,
                min_area=64,
                morph_mode=3,        # 0 none 1 open 2 close 3 close+open
                warmup=True)         # alpha_w ~ 1/t during the first 2^a frames


def alpha_schedule(t, a):
    """alpha_w(t) = 2^-min(a, ceil(log2(t+1))) (same as mod_model.alpha_schedule)."""
    return min(a, t.bit_length() if t > 0 else 0)


class GmmFormat:
    """Word lengths of the hardware GMM, read from the GMM_FMT register."""

    def __init__(self, reg):
        self.WB = reg & 0xF
        self.FM = (reg >> 4) & 0xF
        self.VB = ((reg >> 8) & 0xF) + 8
        self.SB = (reg >> 12) & 0xF
        self.MB = 8 + self.FM
        self.VS = 14 - self.VB
        self.GW = self.WB + self.MB + self.VB + self.SB

    def __repr__(self):
        return "w%d mu%d s2%d ms%d (%d bits/pixel)" % (self.WB, self.MB, self.VB, self.SB, 3 * self.GW)


def _sync_to_device(buf):
    if hasattr(buf, "flush"):
        buf.flush()


def _sync_from_device(buf):
    if hasattr(buf, "invalidate"):
        buf.invalidate()


class MovingObjectDetector:
    """Frame-by-frame interface to the hardware pipeline."""

    def __init__(self, bitfile="mod_overlay.bit", width=640, height=480,
                 download=True, **cfg):
        self.ol = Overlay(bitfile, download=download)
        self.core = self.ol.mod_top_0
        self.dma_pix = self.ol.axi_dma_pix
        self.dma_par = self.ol.axi_dma_par
        ident = self.core.read(REG_ID)
        if ident != CORE_ID:
            raise RuntimeError("unexpected core ID 0x%08x" % ident)
        caps = self.core.read(REG_CAPS)
        self.max_w = caps >> 16
        self.max_obj = 1 << ((caps >> 8) & 0xFF)
        self.fmt = GmmFormat(self.core.read(REG_GMM_FMT))
        self.w, self.h = width, height
        if not (5 <= width <= self.max_w and 2 <= height <= 1024):
            raise ValueError("unsupported frame size")
        self.cfg = dict(DEFAULTS)
        self.cfg.update(cfg)
        n = width * height
        self.pix = allocate(shape=(height, width, 4), dtype=np.uint8)
        self.mask = allocate(shape=(height, width), dtype=np.uint8)
        self.par = [allocate(shape=(n, 2), dtype=np.uint64) for _ in range(2)]
        self.cur = 0
        self.initialised = False
        self.configure()

    # ------------------------------------------------------------------
    def configure(self, **cfg):
        self.cfg.update(cfg)
        c, w = self.cfg, self.core.write
        w(REG_CTRL, 1)                                   # soft reset
        w(REG_WIDTH, self.w)
        w(REG_HEIGHT, self.h)
        w(REG_SHIFTS, c["alpha_sh"])
        w(REG_LAMBDA2, c["lambda2"])
        w(REG_T_BG, c["t_bg"])
        w(REG_VAR_INIT, c["var_init"])
        w(REG_VAR_MIN, c["var_min"])
        w(REG_VAR_MAX, c["var_max"])
        w(REG_MIN_AREA, c["min_area"])
        w(REG_CTRL, (c["morph_mode"] & 3) << 2)            # release reset

    # ------------------------------------------------------------------
    def _load_frame(self, frame_bgr):
        """frame_bgr: (H,W,3) uint8 as returned by OpenCV."""
        self.pix[..., 0] = frame_bgr[..., 2]
        self.pix[..., 1] = frame_bgr[..., 1]
        self.pix[..., 2] = frame_bgr[..., 0]

    def init_model(self, frame_bgr):
        """Bootstrap the background model from one frame:
        G0 = (w=1, mu=x, s2=var_init, matchsum=1), G1 = G2 = (0, 0, var_init, 0)."""
        f = self.fmt
        r = frame_bgr[..., 2].astype(np.uint32)
        g = frame_bgr[..., 1].astype(np.uint32)
        b = frame_bgr[..., 0].astype(np.uint32)
        gray = ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint64).ravel()
        vinit = min(self.cfg["var_init"] >> f.VS, (1 << f.VB) - 1)
        u = np.uint64
        vfield = u(vinit) << u(f.WB + f.MB)
        g0 = u((1 << f.WB) - 1) | ((gray << u(f.FM)) << u(f.WB)) | vfield | \
            (u(1) << u(f.WB + f.MB + f.VB))
        words = [g0, np.full_like(g0, vfield), np.full_like(g0, vfield)]
        lo = np.zeros_like(g0)
        hi = np.zeros_like(g0)
        for k, gw in enumerate(words):
            off = f.GW * k
            if off < 64:
                lo |= gw << u(off)
                if off + f.GW > 64:
                    hi |= gw >> u(64 - off)
            else:
                hi |= gw << u(off - 64)
        p = self.par[self.cur]
        p[:, 0] = lo
        p[:, 1] = hi
        self.t = 0
        self.initialised = True

    def params(self):
        """Current model as a (N,2) uint64 array (lo, hi words)."""
        return np.array(self.par[self.cur])

    # ------------------------------------------------------------------
    def process(self, frame_bgr):
        """Run one frame. Returns (mask view, list of objects, stats)."""
        if not self.initialised:
            self.init_model(frame_bgr)
        self._load_frame(frame_bgr)
        src, dst = self.par[self.cur], self.par[1 - self.cur]
        a = self.cfg["alpha_sh"]
        self.core.write(REG_SHIFTS, alpha_schedule(self.t, a) if self.cfg["warmup"] else a)
        self.t += 1
        fc0 = self.core.read(REG_STATUS) >> 16
        _sync_to_device(self.pix)
        _sync_to_device(src)

        t0 = time.perf_counter()
        self.dma_par.recvchannel.transfer(dst)
        self.dma_pix.recvchannel.transfer(self.mask)
        self.dma_par.sendchannel.transfer(src)
        self.dma_pix.sendchannel.transfer(self.pix)
        self.dma_par.sendchannel.wait()
        self.dma_pix.sendchannel.wait()
        self.dma_par.recvchannel.wait()
        self.dma_pix.recvchannel.wait()
        while (self.core.read(REG_STATUS) >> 16) == fc0:
            pass
        t_hw = time.perf_counter() - t0

        _sync_from_device(self.mask)
        _sync_from_device(dst)
        self.cur = 1 - self.cur
        objs = self.read_objects()
        st = self.core.read(REG_STATUS)
        stats = dict(t_hw=t_hw, fg_pixels=self.core.read(REG_FG_COUNT),
                     overflow=st & 1, frame=st >> 16)
        return self.mask, objs, stats

    def read_objects(self):
        n = self.core.read(REG_NUM_OBJ)
        objs = []
        for i in range(n):
            a = REG_OBJ_BASE + 16 * i
            wx, wy, area = self.core.read(a), self.core.read(a + 4), self.core.read(a + 8)
            objs.append(dict(x0=wx & 0x3FF, x1=(wx >> 16) & 0x3FF,
                             y0=wy & 0x3FF, y1=(wy >> 16) & 0x3FF, area=area))
        return objs

    def close(self):
        for b in [self.pix, self.mask] + self.par:
            b.freebuffer()


def draw_objects(frame_bgr, objs, color=(0, 255, 0)):
    import cv2
    out = frame_bgr.copy()
    for o in objs:
        cv2.rectangle(out, (o["x0"], o["y0"]), (o["x1"], o["y1"]), color, 2)
        cx, cy = (o["x0"] + o["x1"]) // 2, (o["y0"] + o["y1"]) // 2
        cv2.circle(out, (cx, cy), 2, (0, 0, 255), -1)
    return out
