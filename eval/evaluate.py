"""
Algorithm-level evaluation used in the report.

  1. Synthetic sequence with exact ground truth (320x240, 400 frames):
     pixel-level precision / recall / F1 and object-level detection rate for
       - hardware model, default word lengths (bit-exact with the RTL)
       - hardware model, word lengths of Table I of Genovese & Napoli
       - floating-point OpenCV GMM (the paper's equations, exact alpha_k)
       - floating-point Stauffer-Grimson GMM
       - OpenCV MOG2
  2. Paper-style "MSE" (fraction of pixels classified differently from the
     floating-point OpenCV GMM) and a word-length sensitivity study.
  3. Post-processing (morphology) and learning-rate studies.
  4. Real surveillance clip (OpenCV sample vtest.avi, 640x480).

usage: python3 evaluate.py <vtest.avi> <outdir>
"""
import json
import os
import sys
import time

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "model"))
sys.path.insert(0, HERE)
import mod_model as mm  # noqa: E402
from scene import Scene  # noqa: E402
from float_gmm import FloatOpenCVGMM, FloatStaufferGrimson  # noqa: E402

plt.rcParams.update({"font.size": 8, "font.family": "serif", "axes.grid": True,
                     "grid.alpha": 0.3, "figure.dpi": 200, "savefig.bbox": "tight"})

A_DEF = 7          # alpha_w = 2^-7


# ------------------------------------------------------------------ metrics
def prf(pred, gt):
    return np.array([np.sum(pred & gt), np.sum(pred & ~gt), np.sum(~pred & gt)])


def summarize(c, n_pix):
    tp, fp, fn = c
    p = tp / max(tp + fp, 1)
    r = tp / max(tp + fn, 1)
    f = 2 * p * r / max(p + r, 1e-12)
    return dict(precision=float(p), recall=float(r), f1=float(f), pwc=float(100.0 * (fp + fn) / n_pix))


def boxes_from(mask, min_area):
    n, _, st, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    return [(x, x + w - 1, y, y + h - 1) for x, y, w, h, a in st[1:] if a >= min_area]


def iou(a, b):
    ix = max(0, min(a[1], b[1]) - max(a[0], b[0]) + 1)
    iy = max(0, min(a[3], b[3]) - max(a[2], b[2]) + 1)
    inter = ix * iy
    ua = (a[1] - a[0] + 1) * (a[3] - a[2] + 1) + (b[1] - b[0] + 1) * (b[3] - b[2] + 1) - inter
    return inter / ua


def merge_overlapping(boxes):
    boxes = list(boxes)
    changed = True
    while changed:
        changed = False
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                a, b = boxes[i], boxes[j]
                if a[0] <= b[1] + 1 and b[0] <= a[1] + 1 and a[2] <= b[3] + 1 and b[2] <= a[3] + 1:
                    boxes[i] = (min(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), max(a[3], b[3]))
                    boxes.pop(j)
                    changed = True
                    break
            if changed:
                break
    return boxes


def object_stats(masks, gtb, t0, min_area):
    det = [0, 0, 0]
    for t in range(t0, len(masks)):
        bx = boxes_from(masks[t], min_area)
        used = set()
        for gb in gtb[t]:
            det[1] += 1
            best = max(((iou(gb, b), k) for k, b in enumerate(bx)), default=(0, -1))
            if best[0] >= 0.5:
                det[0] += 1
                used.add(best[1])
        det[2] += len(bx) - len(used)
    return det[0] / max(det[1], 1), det[2] / (len(masks) - t0)


def run_hw(grays, fmt, a=A_DEF, sched=True):
    """bit-exact hardware model, with the software warm-up schedule"""
    cfg = mm.Cfg(width=grays[0].shape[1], height=grays[0].shape[0], alpha_sh=a, fmt=fmt)
    p = mm.init_params(grays[0], cfg)
    out = []
    for t, g in enumerate(grays):
        cfg.alpha_sh = mm.alpha_schedule(t, a) if sched else a
        fg, p = mm.gmm_step(g, p, cfg)
        out.append(fg)
    return out


def run_float(grays, a=A_DEF, sched=True):
    """double-precision OpenCV GMM (paper equations, exact alpha_k)"""
    fl = FloatOpenCVGMM(grays[0].astype(float), alpha=2.0 ** -a)
    return [fl.apply(g, 2.0 ** -(mm.alpha_schedule(t, a) if sched else a)) for t, g in enumerate(grays)]


# ------------------------------------------------------------------ synthetic
def run_synthetic(outdir, n_frames=400, W=320, H=240, warm=60):
    sc = Scene(W, H, n_obj=3, seed=5, warmup=warm, noise_sigma=4.0)
    frames, gts, gtb = [], [], []
    for _ in range(n_frames):
        f, g = sc.frame()
        frames.append(f)
        gts.append(g)
        gtb.append(merge_overlapping(sc.last_boxes))
    grays = [mm.rgb2gray(f) for f in frames]
    bim = sc.bim
    min_area = 40

    t = time.time()
    raw = {}
    raw["HW model, default widths (126 b)"] = run_hw(grays, mm.Fmt.WIDE)
    raw["HW model, Table I widths (99 b)"] = run_hw(grays, mm.Fmt.PAPER)
    raw["Float OpenCV GMM [paper eqs.]"] = run_float(grays)
    raw["HW model, no warm-up schedule"] = run_hw(grays, mm.Fmt.WIDE, sched=False)
    sg = FloatStaufferGrimson(grays[0].astype(float))
    raw["Float Stauffer-Grimson"] = [sg.apply(g) for g in grays]
    mog = cv2.createBackgroundSubtractorMOG2(history=2 ** A_DEF, varThreshold=6.25, detectShadows=False)
    mog.setNMixtures(3)
    mog.setBackgroundRatio(0.7)
    mog.setVarInit(900)
    mog.setVarMin(16)
    # learningRate=-1: 1/min(t, history), i.e. the same warm-up behaviour
    raw["OpenCV MOG2"] = [mog.apply(g.astype(np.uint8), learningRate=-1) > 0 for g in grays]
    print("synthetic methods: %.1f s" % (time.time() - t))

    float_ref = raw["Float OpenCV GMM [paper eqs.]"]
    res, curves, bim_curves = {}, {}, {}
    for name, fgs in raw.items():
        mo = [mm.morph(m, 3) for m in fgs]
        acc_raw = sum(prf(fgs[t], gts[t]) for t in range(warm, n_frames))
        acc = sum(prf(mo[t], gts[t]) for t in range(warm, n_frames))
        r = summarize(acc, W * H * (n_frames - warm))
        r["f1_raw"] = summarize(acc_raw, 1)["f1"]
        r["obj_recall"], r["false_boxes_per_frame"] = object_stats(mo, gtb, warm, min_area)
        r["mse_vs_float"] = float(np.mean([np.mean(fgs[t] != float_ref[t]) for t in range(n_frames)]))
        res[name] = r
        curves[name] = [summarize(prf(mo[t], gts[t]), 1)["f1"] for t in range(warm, n_frames)]
        bim_curves[name] = [np.mean(mo[t][bim & ~gts[t]]) for t in range(n_frames)]

    # word-length sensitivity (paper Fig. 3 style): vary one word length,
    # keep the others at the default; MSE w.r.t. the float OpenCV GMM
    wl = {}
    base = dict(WB=12, FM=4, VB=14, SB=4)
    sweeps = dict(WB=[5, 6, 7, 8, 9, 10, 12], FM=[0, 1, 2, 3, 4, 6], VB=[8, 9, 10, 11, 12, 14],
                  SB=[2, 3, 4, 5, 6])
    for key, vals in sweeps.items():
        wl[key] = []
        for v in vals:
            d = dict(base)
            d[key] = v
            if key == "WB" and v < A_DEF:
                continue
            fgs = run_hw(grays, mm.Fmt(**d))
            mse = float(np.mean([np.mean(fgs[t] != float_ref[t]) for t in range(n_frames)]))
            wl[key].append((v, mse))
    # paper Table I point
    wl["paper"] = res["HW model, Table I widths (99 b)"]["mse_vs_float"]

    # post-processing study (default model)
    fgs = raw["HW model, default widths (126 b)"]
    morph_tab = []
    for mode, mname in ((0, "none"), (1, "opening"), (2, "closing"), (3, "closing + opening")):
        mo = [mm.morph(m, mode) for m in fgs]
        acc = sum(prf(mo[t], gts[t]) for t in range(warm, n_frames))
        s_ = summarize(acc, W * H * (n_frames - warm))
        s_["mode"] = mname
        s_["obj_recall"], s_["false_boxes_per_frame"] = object_stats(mo, gtb, warm, min_area)
        morph_tab.append(s_)

    # learning-rate study (default model and float reference)
    sweep = []
    for a in (5, 6, 7, 8, 9):
        fh = run_hw(grays, mm.Fmt.WIDE, a)
        ff = run_float(grays, a)
        acc_h = sum(prf(mm.morph(fh[t], 3), gts[t]) for t in range(warm, n_frames))
        acc_f = sum(prf(mm.morph(ff[t], 3), gts[t]) for t in range(warm, n_frames))
        sweep.append(dict(alpha_sh=a, f1_hw=summarize(acc_h, 1)["f1"], f1_float=summarize(acc_f, 1)["f1"]))

    # ---- figures
    short = {"HW model, default widths (126 b)": "HW, default widths",
             "HW model, Table I widths (99 b)": "HW, Table I widths",
             "Float OpenCV GMM [paper eqs.]": "float OpenCV GMM",
             "HW model, no warm-up schedule": "HW, constant $\\alpha_w$",
             "Float Stauffer-Grimson": "float Stauffer-Grimson",
             "OpenCV MOG2": "OpenCV MOG2"}
    styles = ["-", "--", ":", "-", "-.", "-"]
    fig, ax = plt.subplots(figsize=(3.4, 1.9))
    xs = np.arange(warm, n_frames)
    for (name, c), ls in zip(curves.items(), styles):
        ax.plot(xs, c, ls, lw=0.9, label=short[name])
    ax.set_xlabel("frame")
    ax.set_ylabel("F1 (after morphology)")
    ax.set_ylim(0, 1.02)
    ax.legend(fontsize=5.5, loc="lower right", ncol=2)
    fig.savefig(os.path.join(outdir, "fig_f1_synthetic.pdf"))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(3.4, 1.8))
    k = 15
    for (name, c), ls in zip(bim_curves.items(), styles):
        sm = np.convolve(c, np.ones(k) / k, mode="valid")
        ax.plot(np.arange(len(sm)) + k // 2, 100 * sm, ls, lw=0.9, label=short[name])
    ax.set_xlabel("frame")
    ax.set_ylabel("false positives in\nbimodal region (%)")
    ax.set_yscale("symlog", linthresh=1)
    ax.legend(fontsize=5.5, loc="upper right", ncol=2)
    fig.savefig(os.path.join(outdir, "fig_bimodal.pdf"))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(3.4, 1.9))
    lab = {"WB": "weight $w$ (bits)", "FM": "mean $\\mu$ (bits)", "VB": "variance $\\sigma^2$ (bits)",
           "SB": "matchsum (bits)"}
    mk = {"WB": "o", "FM": "s", "VB": "^", "SB": "d"}
    for key in ("WB", "FM", "VB", "SB"):
        v = np.array([p[0] for p in wl[key]], float)
        if key == "FM":
            v = v + 8
        ax.semilogy(v, [max(p[1], 1e-5) for p in wl[key]], marker=mk[key], ms=3, lw=0.9, label=lab[key])
    ax.axhline(0.05, color="k", lw=0.7, ls="--")
    ax.text(2.2, 0.058, "target MSE 0.05 [paper]", fontsize=6)
    ax.set_xlabel("word length of the varied signal (others at default)")
    ax.set_ylabel("MSE vs. float\nOpenCV GMM")
    ax.legend(fontsize=6, loc="upper right")
    fig.savefig(os.path.join(outdir, "fig_wordlength.pdf"))
    plt.close(fig)

    # qualitative frame
    t = warm + 120
    fg = raw["HW model, default widths (126 b)"][t]
    op = mm.morph(fg, 3)
    cfg = mm.Cfg(width=W, height=H, min_area=min_area)
    objs, _ = mm.ccl_single_pass(op, cfg)
    vis = frames[t][..., :3].copy()
    for o in objs:
        cv2.rectangle(vis, (max(o[0] - 1, 0), max(o[2] - 1, 0)), (o[1] + 1, o[3] + 1), (255, 0, 0), 2)
    panels = [frames[t][..., :3], (gts[t] * 255).astype(np.uint8), (fg * 255).astype(np.uint8),
              (op * 255).astype(np.uint8), vis]
    titles = ["(a) input", "(b) ground truth", "(c) GMM output", "(d) after morphology", "(e) CCL boxes"]
    fig, axs = plt.subplots(1, 5, figsize=(7.1, 1.25))
    for a_, im, tt in zip(axs, panels, titles):
        a_.imshow(im, cmap="gray" if im.ndim == 2 else None)
        a_.set_title(tt, fontsize=7)
        a_.axis("off")
    fig.savefig(os.path.join(outdir, "fig_synthetic_qual.pdf"))
    plt.close(fig)
    return res, wl, morph_tab, sweep


# ------------------------------------------------------------------ real video
def run_real(video, outdir, W=640, H=480, n_frames=300):
    cap = cv2.VideoCapture(video)
    frames = []
    while len(frames) < n_frames:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(cv2.resize(fr, (W, H), interpolation=cv2.INTER_AREA))

    def gray_of(fr):
        o = np.zeros((H, W, 4), np.uint8)
        o[..., 0], o[..., 1], o[..., 2] = fr[..., 2], fr[..., 1], fr[..., 0]
        return mm.rgb2gray(o)

    grays = [gray_of(f) for f in frames]
    hw = run_hw(grays, mm.Fmt.WIDE)
    hp = run_hw(grays, mm.Fmt.PAPER)
    ff = run_float(grays)
    mse_w = float(np.mean([np.mean(hw[t] != ff[t]) for t in range(len(frames))]))
    mse_p = float(np.mean([np.mean(hp[t] != ff[t]) for t in range(len(frames))]))
    n_obj = [len(boxes_from(mm.morph(hw[t], 3), 64)) for t in range(30, len(frames))]
    sel = (60, 150, 250)
    fig, axs = plt.subplots(len(sel), 3, figsize=(3.45, 0.84 * len(sel)))
    cfg = mm.Cfg(width=W, height=H, min_area=64)
    for r, t in enumerate(sel):
        op = mm.morph(hw[t], 3)
        objs, _ = mm.ccl_single_pass(op, cfg)
        vis = frames[t].copy()
        for o in objs:
            cv2.rectangle(vis, (o[0], o[2]), (o[1], o[3]), (0, 0, 255), 5)
        for c, (im, tt) in enumerate(((cv2.cvtColor(vis, cv2.COLOR_BGR2RGB), "detections"),
                                      (hw[t], "GMM foreground"), (op, "after morphology"))):
            axs[r, c].imshow(im, cmap="gray" if im.ndim == 2 else None)
            axs[r, c].axis("off")
            if r == 0:
                axs[r, c].set_title(tt, fontsize=7)
    plt.subplots_adjust(wspace=0.03, hspace=0.05)
    fig.savefig(os.path.join(outdir, "fig_real_qual.pdf"))
    plt.close(fig)
    return dict(frames=len(frames), mse_vs_float_default=mse_w, mse_vs_float_tableI=mse_p,
                mean_objects=float(np.mean(n_obj)), max_objects=int(np.max(n_obj)))


def main():
    video, outdir = sys.argv[1], sys.argv[2]
    os.makedirs(outdir, exist_ok=True)
    res, wl, morph_tab, sweep = run_synthetic(outdir)
    real = run_real(video, outdir)
    json.dump(dict(synthetic=res, wordlength=wl, morph=morph_tab, sweep=sweep, real=real),
              open(os.path.join(outdir, "metrics.json"), "w"), indent=2)
    for k, v in res.items():
        print("%-36s P %.3f R %.3f F1 %.3f (raw %.3f) PWC %.2f objR %.3f fb/f %.2f MSEvsFloat %.4f" % (
            k, v["precision"], v["recall"], v["f1"], v["f1_raw"], v["pwc"], v["obj_recall"],
            v["false_boxes_per_frame"], v["mse_vs_float"]))
    for key in ("WB", "FM", "VB", "SB"):
        print("wordlength", key, " ".join("%d:%.4f" % p for p in wl[key]))
    print("Table I MSE", wl["paper"])
    for s in morph_tab:
        print("morph %-18s P %.3f R %.3f F1 %.3f objR %.3f fb/f %.2f" % (
            s["mode"], s["precision"], s["recall"], s["f1"], s["obj_recall"], s["false_boxes_per_frame"]))
    for s in sweep:
        print("alpha 2^-%d : F1 hw %.3f float %.3f" % (s["alpha_sh"], s["f1_hw"], s["f1_float"]))
    print(json.dumps(real, indent=1))


if __name__ == "__main__":
    main()
