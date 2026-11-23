"""
Bit-exact reference model of the FPGA moving-object-detection pipeline.

    RGB -> Gray -> GMM background identification (K=3, fixed point)
        -> binary morphology (3x3 closing and/or opening)
        -> single-pass connected-component labelling (8-connectivity)
        -> object table (bounding box + area)

The GMM follows the hardware-oriented formulation of the OpenCV GMM by
M. Genovese and E. Napoli (IEEE TVLSI 22(3), 2014):
  * every Gaussian carries weight w, mean mu, variance s2 and a match
    counter (matchsum);
  * learning rates are powers of two: alpha_w = 2^-a and
    alpha_k = alpha_w / w_k, rounded to the nearest power of two by a
    look-up on w_k (paper eq. 4, 12);
  * Gaussians are ordered by IF_k = s2_k * 2^(2(n_k - n_w)) = s2_k / w_k^2,
    which is equivalent to ordering by the fitness w/sigma (eq. 10-14);
  * match: (x - mu)^2 < lambda^2 * s2 (eq. 15);
  * matched Gaussian: mu, s2 updated with alpha_k, w += alpha_w (1 - w),
    matchsum += 1; other weights decay w -= alpha_w w (eq. 3, 6);
  * no match: the Gaussian with the largest IF is replaced by
    (w = 1/msumtot, mu = x, s2 = v_init, matchsum = 1), msumtot being the
    matchsum of the other two Gaussians (eq. 7);
  * background: the pixel matches one of the first B Gaussians in fitness
    order, B = argmin_b sum_{k<=b} w_k > T (eq. 8).

Every integer operation mirrors the RTL in ../rtl exactly, so model and RTL
simulation must agree bit for bit.

Word lengths are parameters (class Fmt):
    WB : weight bits,     w  = code / 2^WB              (U-1,WB)
    FM : mean fraction,   mu = code / 2^FM, MB = 8 + FM (U7,FM)
    VB : variance bits,   s2 = code * 2^VS, VS = 14 - VB (U13,VS)
    SB : matchsum bits                                   (U(SB-1),0)
Fmt.PAPER is Table I of the paper (8/10/11/4 bits = 99 bits per pixel);
Fmt.WIDE (default build, 12/12/14/4 = 126 bits) still fits one 128-bit
DMA word. Gaussian k sits at bits [GW*k + GW-1 : GW*k] as
{matchsum, s2, mu, w} (w in the LSBs).
"""
import numpy as np

K = 3


class Fmt:
    def __init__(self, WB=12, FM=4, VB=14, SB=4):
        assert 1 <= WB <= 12 and 0 <= FM <= 8 and 8 <= VB <= 14 and 2 <= SB <= 6
        self.WB, self.FM, self.VB, self.SB = WB, FM, VB, SB
        self.MB = 8 + FM
        self.VS = 14 - VB
        self.E = self.VS + 2 * FM          # (x-mu)^2 code -> s2 code shift
        self.GW = WB + self.MB + VB + SB
        self.fits_128 = K * self.GW <= 128   # required by the RTL / DMA word
        self.WMAX = (1 << WB) - 1
        self.VMAX = (1 << VB) - 1
        self.SMAX = (1 << SB) - 1

    def __repr__(self):
        return "Fmt(WB=%d, FM=%d, VB=%d, SB=%d)" % (self.WB, self.FM, self.VB, self.SB)


Fmt.WIDE = Fmt(12, 4, 14, 4)
Fmt.PAPER = Fmt(8, 2, 11, 4)


class Cfg:
    def __init__(self, width=160, height=120, alpha_sh=7, lambda2=100,
                 t_bg=2867, var_init=900, var_min=16, var_max=16383,
                 min_area=16, gray_mode=0, morph_mode=3, max_lbl=128,
                 max_obj=32, fmt=None):
        self.width, self.height = width, height
        self.alpha_sh = alpha_sh        # alpha_w = 2^-alpha_sh (OpenCV: 1/200)
        self.lambda2 = lambda2          # Q4.4  (100 -> 6.25 -> lambda = 2.5)
        self.t_bg = t_bg                # Q0.12 (2867 -> 0.70)
        self.var_init = var_init        # integer sigma^2 (OpenCV: 30^2)
        self.var_min = var_min          # integer sigma^2
        self.var_max = var_max
        self.min_area = min_area
        self.gray_mode = gray_mode
        self.morph_mode = morph_mode    # 0 none 1 open 2 close 3 close+open
        self.max_lbl = max_lbl
        self.max_obj = max_obj
        self.fmt = fmt if fmt is not None else Fmt.WIDE
        assert self.alpha_sh <= self.fmt.WB


# --------------------------------------------------------------------------
# Colour conversion
# --------------------------------------------------------------------------
def rgb2gray(rgbx, gray_mode=0):
    """rgbx: (H,W,4) uint8 with byte0=R, byte1=G, byte2=B."""
    r = rgbx[..., 0].astype(np.int64)
    if gray_mode:
        return r
    g = rgbx[..., 1].astype(np.int64)
    b = rgbx[..., 2].astype(np.int64)
    return (77 * r + 150 * g + 29 * b) >> 8


# --------------------------------------------------------------------------
# Parameter packing.  p has shape (4, K, H, W): fields w, mu, s2, matchsum
# --------------------------------------------------------------------------
def init_params(gray, cfg):
    """Bootstrap from one frame: G0 = (1, x, v_init, 1); G1, G2 empty."""
    f = cfg.fmt
    h, w = gray.shape
    p = np.zeros((4, K, h, w), dtype=np.int64)
    p[0, 0] = f.WMAX
    p[1, 0] = gray.astype(np.int64) << f.FM
    p[2, :] = min(cfg.var_init >> f.VS, f.VMAX)
    p[3, 0] = 1
    return p


def _gword(p, k, f):
    w, mu, v, ms = p[0, k], p[1, k], p[2, k], p[3, k]
    return w | (mu << f.WB) | (v << (f.WB + f.MB)) | (ms << (f.WB + f.MB + f.VB))


def pack_params(p, f):
    """-> list of python ints (128-bit words), raster order."""
    assert f.fits_128
    g = [_gword(p.astype(object), k, f).ravel() for k in range(K)]
    return [int(a) | (int(b) << f.GW) | (int(c) << (2 * f.GW)) for a, b, c in zip(*g)]


def unpack_params(words, h, w, f):
    arr = np.array(words, dtype=object)
    p = np.zeros((4, K, h * w), dtype=np.int64)
    m = (1 << f.GW) - 1
    for k in range(K):
        g = (arr >> (f.GW * k)) & m
        p[0, k] = np.array(g & f.WMAX, dtype=np.int64)
        p[1, k] = np.array((g >> f.WB) & ((1 << f.MB) - 1), dtype=np.int64)
        p[2, k] = np.array((g >> (f.WB + f.MB)) & f.VMAX, dtype=np.int64)
        p[3, k] = np.array((g >> (f.WB + f.MB + f.VB)) & f.SMAX, dtype=np.int64)
    return p.reshape(4, K, h, w)


def pack_params_u64(p, f):
    """Vectorised packing into (N,2) uint64 (lo, hi) - used by the drivers."""
    lo = np.zeros(p[0, 0].size, np.uint64)
    hi = np.zeros(p[0, 0].size, np.uint64)
    for k in range(K):
        g = _gword(p.astype(np.uint64), k, f).ravel().astype(np.uint64)
        off = f.GW * k
        if off < 64:
            lo |= g << np.uint64(off)
            if off + f.GW > 64:
                hi |= g >> np.uint64(64 - off)
        else:
            hi |= g << np.uint64(off - 64)
    return np.stack([lo, hi], axis=1)


# --------------------------------------------------------------------------
# GMM core (Genovese & Napoli formulation of the OpenCV GMM)
# --------------------------------------------------------------------------
def log2_round(c, WB):
    """round(log2(c)) for 1 <= c < 2^WB, as the hardware computes it:
    number of r in [0, WB) with c^2 >= 2^(2r+1)."""
    c2 = c * c
    L = np.zeros_like(c)
    for r in range(WB):
        L += (c2 >= (1 << (2 * r + 1))).astype(np.int64)
    return L


def alpha_shift(w, a, WB):
    """alpha_k = alpha_w / w_k = 2^-s, s = clamp(a - WB + round(log2 w_code), 0, a)."""
    s = np.clip(a - WB + log2_round(w, WB), 0, a)
    return np.where(w == 0, 0, s)


def alpha_schedule(t, a):
    """Warm-up schedule applied by software through the SHIFTS register:
    alpha_w(t) = 2^-min(a, ceil(log2(t+1))) for frame t = 0, 1, 2, ...
    i.e. alpha_w ~ 1/t during the first 2^a frames (as in OpenCV's initial
    learning phase), then the constant 2^-a."""
    return min(a, (t).bit_length() if t > 0 else 0)


def rshr(x, s):
    """arithmetic shift right with round-half-up: (x + 2^(s-1)) >> s."""
    return (x + ((1 << s) >> 1)) >> s


def recip(n, f):
    """weight code of 1/n (eq. 7), n = msumtot."""
    n = np.asarray(n)
    r = ((1 << f.WB) + n // 2) // np.maximum(n, 1)
    return np.where(n == 0, f.WMAX, np.minimum(r, f.WMAX))


def if_max(f, a_max=12):
    return (1 << (f.VB + 2 * a_max)) - 1


def gmm_step(gray, p, cfg):
    """One frame of the fixed-point GMM. Returns (fg bool, new params)."""
    f = cfg.fmt
    a = cfg.alpha_sh
    w, mu, v, ms = [x.copy() for x in p]
    xs = gray.astype(np.int64) << f.FM
    d = xs[None] - mu
    d2 = d * d
    match = (d2 << 4) < ((cfg.lambda2 * v) << f.E)

    # learning rate per Gaussian and inverse fitness
    s = np.stack([alpha_shift(w[k], a, f.WB) for k in range(K)])
    IF = np.where(w == 0, if_max(f), v << (2 * (a - s)))

    # stable ascending sort of IF -> rank of every Gaussian
    b01 = IF[0] <= IF[1]
    b02 = IF[0] <= IF[2]
    b12 = IF[1] <= IF[2]
    rank = np.stack([(~b01).astype(int) + (~b02),
                     b01.astype(int) + (~b12),
                     b02.astype(int) + b12])

    # first matched Gaussian in fitness order
    big = 9
    mr = np.where(match, rank, big)
    m = np.argmin(mr, axis=0)
    found = match.any(axis=0)

    # background test: sum of weights ranked before the matched one <= T
    sbefore = np.stack([sum(np.where(rank[j] < rank[k], w[j], 0) for j in range(K))
                        for k in range(K)])
    bg = (sbefore << (12 - f.WB)) <= cfg.t_bg
    fg = ~(found & np.take_along_axis(bg, m[None], 0)[0])

    # candidate updates
    vmin = cfg.var_min >> f.VS
    vmax = min(cfg.var_max >> f.VS, f.VMAX)
    d2q = d2 >> f.E
    inc = 1 << (f.WB - a)
    w_hit = np.minimum(w + inc - rshr(w, a), f.WMAX)
    w_mis = w - rshr(w, a)
    mu_hit = np.stack([mu[k] + rshr(d[k], s[k]) for k in range(K)])
    v_hit = np.stack([np.clip(v[k] + rshr(d2q[k] - v[k], s[k]), vmin, vmax) for k in range(K)])
    ms_hit = np.minimum(ms + 1, f.SMAX)

    msumtot = sum(np.where(rank[j] < 2, ms[j], 0) for j in range(K))
    w_new = recip(msumtot, f)
    v_init = min(cfg.var_init >> f.VS, f.VMAX)

    nw, nmu, nv, nms = w_mis.copy(), mu.copy(), v.copy(), ms.copy()
    for k in range(K):
        hit = found & (m == k)
        rep = (~found) & (rank[k] == 2)
        nw[k] = np.where(hit, w_hit[k], np.where(rep, w_new, w_mis[k]))
        nmu[k] = np.where(hit, mu_hit[k], np.where(rep, xs, mu[k]))
        nv[k] = np.where(hit, v_hit[k], np.where(rep, v_init, v[k]))
        nms[k] = np.where(hit, ms_hit[k], np.where(rep, 1, ms[k]))
    return fg, np.stack([nw, nmu, nv, nms])


# --------------------------------------------------------------------------
# Morphology (out-of-frame neighbours are ignored)
# --------------------------------------------------------------------------
def _win3(img, pad_val):
    h, w = img.shape
    pp = np.pad(img, 1, constant_values=pad_val)
    return [pp[dy:dy + h, dx:dx + w] for dy in range(3) for dx in range(3)]


def erode3(img):
    out = np.ones_like(img, dtype=bool)
    for v in _win3(img, True):
        out &= v
    return out


def dilate3(img):
    out = np.zeros_like(img, dtype=bool)
    for v in _win3(img, False):
        out |= v
    return out


def opening(img):
    return dilate3(erode3(img))


def morph(img, mode=3):
    """Hardware chain dilate->erode (closing, mode bit 1) then
    erode->dilate (opening, mode bit 0)."""
    out = img.copy()
    if mode & 2:
        out = erode3(dilate3(out))
    if mode & 1:
        out = dilate3(erode3(out))
    return out


# --------------------------------------------------------------------------
# Single-pass CCL - pixel-exact replica of the hardware algorithm
# --------------------------------------------------------------------------
def ccl_single_pass(mask, cfg):
    h, w = mask.shape
    L = cfg.max_lbl
    M = list(range(L))                         # merger table (always flat)
    feat = [None] * L                          # [minx,maxx,miny,maxy,area]
    nxt, overflow = 1, 0
    row = [0] * w
    for y in range(h):
        A = B = D = 0
        Braw = row[0]
        for x in range(w):
            Craw = row[x + 1] if x < w - 1 else 0
            a = M[A] if (y > 0 and x > 0) else 0
            b = M[Braw] if y > 0 else 0
            c = M[Craw] if y > 0 else 0
            d = M[D] if x > 0 else 0
            cur = 0
            if mask[y, x]:
                X = b if b else (a if a else d)
                Y = c
                if X == 0 and Y == 0:
                    if nxt < L:
                        cur = nxt
                        feat[cur] = [x, x, y, y, 1]
                        nxt += 1
                    else:
                        overflow = 1
                else:
                    if X == 0:
                        cur = Y
                    elif Y == 0 or X == Y:
                        cur = X
                    else:
                        s, g = min(X, Y), max(X, Y)
                        for i in range(L):
                            if M[i] == g:
                                M[i] = s
                        fs, fg_ = feat[s], feat[g]
                        feat[s] = [min(fs[0], fg_[0]), max(fs[1], fg_[1]),
                                   min(fs[2], fg_[2]), max(fs[3], fg_[3]),
                                   fs[4] + fg_[4]]
                        cur = s
                    f = feat[cur]
                    feat[cur] = [min(f[0], x), max(f[1], x), min(f[2], y),
                                 max(f[3], y), f[4] + 1]
            # shift registers (A <- B <- C) and row buffer write
            A = Braw
            Braw = Craw
            row[x] = cur
            D = cur
    objs = []
    for i in range(1, nxt):
        if M[i] == i and feat[i][4] >= cfg.min_area and len(objs) < cfg.max_obj:
            objs.append(tuple(feat[i]))
    return objs, overflow


def ccl_reference(mask, min_area=1, max_obj=10 ** 9):
    """Independent BFS labelling, objects ordered by first raster pixel."""
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    objs = []
    for y in range(h):
        for x in range(w):
            if mask[y, x] and not seen[y, x]:
                st = [(y, x)]
                seen[y, x] = True
                xs, ys = [], []
                while st:
                    cy, cx = st.pop()
                    xs.append(cx)
                    ys.append(cy)
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            ny, nx = cy + dy, cx + dx
                            if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] \
                                    and not seen[ny, nx]:
                                seen[ny, nx] = True
                                st.append((ny, nx))
                if len(xs) >= min_area and len(objs) < max_obj:
                    objs.append((min(xs), max(xs), min(ys), max(ys), len(xs)))
    return objs


# --------------------------------------------------------------------------
# Whole pipeline
# --------------------------------------------------------------------------
def run_frame(rgbx, p, cfg, exact_ccl=True):
    gray = rgb2gray(rgbx, cfg.gray_mode)
    fg, newp = gmm_step(gray, p, cfg)
    mask = morph(fg, cfg.morph_mode)
    if exact_ccl:
        objs, ovf = ccl_single_pass(mask, cfg)
    else:
        objs, ovf = ccl_reference(mask, cfg.min_area, cfg.max_obj), 0
    return dict(gray=gray, fg_raw=fg, mask=mask, params=newp,
                objs=objs, overflow=ovf, fg_count=int(mask.sum()))
