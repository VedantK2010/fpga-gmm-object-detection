#!/usr/bin/env python3
"""
On-board demo, verification and benchmark for the PYNQ overlay.

examples (on the board, as root / with sudo):
  python3 run_demo.py --video vtest.avi --frames 300 --out result.avi
  python3 run_demo.py --camera 0 --frames 600
  python3 run_demo.py --video vtest.avi --verify 20     # bit-exact check vs model
  python3 run_demo.py --video vtest.avi --swbench 100   # ARM-only baseline timing

A CSV with per-frame timings is written next to the output video; the
numbers go straight into the results section of the report.
"""
import argparse
import csv
import time

import cv2
import numpy as np

from mod_driver import MovingObjectDetector, draw_objects


def frames_from(args):
    cap = cv2.VideoCapture(args.camera if args.video is None else args.video)
    if args.video is None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    while True:
        ok, fr = cap.read()
        if not ok:
            if args.video is not None and args.loop:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            break
        if fr.shape[1] != args.width or fr.shape[0] != args.height:
            fr = cv2.resize(fr, (args.width, args.height), interpolation=cv2.INTER_AREA)
        yield fr


def verify(det, frames, n):
    """Compare hardware output against the bit-exact software model."""
    import mod_model as mm
    f = det.fmt
    fmt = mm.Fmt(f.WB, f.FM, f.VB, f.SB)
    cfg = mm.Cfg(width=det.w, height=det.h, fmt=fmt,
                 **{k: v for k, v in det.cfg.items()
                    if k in ("alpha_sh", "lambda2", "t_bg", "var_init", "var_min",
                             "var_max", "min_area", "morph_mode")})
    p = None
    bad = 0
    a_target = cfg.alpha_sh
    for i, fr in enumerate(frames):
        if i >= n:
            break
        rgbx = np.zeros((det.h, det.w, 4), np.uint8)
        rgbx[..., 0], rgbx[..., 1], rgbx[..., 2] = fr[..., 2], fr[..., 1], fr[..., 0]
        if p is None:
            p = mm.init_params(mm.rgb2gray(rgbx), cfg)
        mask, objs, st = det.process(fr)
        gray = mm.rgb2gray(rgbx)
        cfg.alpha_sh = mm.alpha_schedule(i, a_target) if det.cfg["warmup"] else a_target
        fg, p = mm.gmm_step(gray, p, cfg)
        m_ref = mm.morph(fg, cfg.morph_mode)
        ok_mask = np.array_equal(mask > 0, m_ref)
        ok_par = np.array_equal(det.params(), mm.pack_params_u64(p, fmt))
        # object list via OpenCV (same raster order of first pixel)
        n_lbl, lbl, stats, _ = cv2.connectedComponentsWithStats(m_ref.astype(np.uint8), connectivity=8)
        ref = []
        for k in range(1, n_lbl):
            x, y, w, h, a = stats[k]
            if a >= cfg.min_area:
                ys, xs = np.nonzero(lbl == k)
                first = (ys * det.w + xs).min()
                ref.append((first, dict(x0=x, x1=x + w - 1, y0=y, y1=y + h - 1, area=a)))
        ref = [o for _, o in sorted(ref, key=lambda t: t[0])][:det.max_obj]
        ok_obj = st["overflow"] or [tuple(o.values()) for o in objs] == \
            [tuple(int(v) for v in o.values()) for o in ref]
        print("frame %3d  mask %s  params %s  objects %s (%d)" %
              (i, "OK" if ok_mask else "DIFF", "OK" if ok_par else "DIFF",
               "OK" if ok_obj else "DIFF", len(objs)))
        bad += (not ok_mask) + (not ok_par) + (not ok_obj)
    print("VERIFY", "PASS" if bad == 0 else "FAIL (%d mismatches)" % bad)


def sw_bench(frames, n, args):
    """Software-only baseline on the ARM: OpenCV MOG2 + morphology + CCL."""
    mog = cv2.createBackgroundSubtractorMOG2(history=128, varThreshold=6.25, detectShadows=False)
    # (MOG2 is OpenCV's current GMM; it is used only as a software speed baseline)
    mog.setNMixtures(3)
    k = np.ones((3, 3), np.uint8)
    # same post-processing as the hardware: closing then opening
    ts = []
    for i, fr in enumerate(frames):
        if i >= n:
            break
        t0 = time.perf_counter()
        m = mog.apply(fr, learningRate=1 / 128)
        m = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_CLOSE, k), cv2.MORPH_OPEN, k)
        cv2.connectedComponentsWithStats(m, connectivity=8)
        ts.append(time.perf_counter() - t0)
    ts = np.array(ts[5:])
    print("ARM software baseline: %.2f ms/frame (%.1f fps)" % (1e3 * ts.mean(), 1 / ts.mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bit", default="mod_overlay.bit")
    ap.add_argument("--video")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--out", default="result.avi")
    ap.add_argument("--verify", type=int, default=0)
    ap.add_argument("--swbench", type=int, default=0)
    ap.add_argument("--min_area", type=int, default=64)
    ap.add_argument("--alpha_sh", type=int, default=7)
    args = ap.parse_args()

    if args.swbench:
        sw_bench(frames_from(args), args.swbench, args)
        return

    det = MovingObjectDetector(args.bit, args.width, args.height, min_area=args.min_area,
                               alpha_sh=args.alpha_sh)
    print("GMM word lengths:", det.fmt)
    if args.verify:
        verify(det, frames_from(args), args.verify)
        return

    vw = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"MJPG"), 25,
                         (2 * args.width, args.height))
    log = open(args.out.rsplit(".", 1)[0] + "_timing.csv", "w", newline="")
    cw = csv.writer(log)
    cw.writerow(["frame", "objects", "fg_pixels", "t_hw_ms", "t_total_ms", "overflow"])
    t_hw, t_tot = [], []
    for i, fr in enumerate(frames_from(args)):
        if i >= args.frames:
            break
        t0 = time.perf_counter()
        mask, objs, st = det.process(fr)
        vis = draw_objects(fr, objs)
        t1 = time.perf_counter()
        vw.write(np.hstack([vis, cv2.cvtColor(np.array(mask), cv2.COLOR_GRAY2BGR)]))
        cw.writerow([i, len(objs), st["fg_pixels"], 1e3 * st["t_hw"], 1e3 * (t1 - t0), st["overflow"]])
        if i >= 5:
            t_hw.append(st["t_hw"])
            t_tot.append(t1 - t0)
        if i % 50 == 0:
            print("frame %d: %d objects, hw %.2f ms" % (i, len(objs), 1e3 * st["t_hw"]))
    vw.release()
    log.close()
    if t_hw:
        print("hardware (DMA + PL) : %.2f ms/frame -> %.1f fps" % (1e3 * np.mean(t_hw), 1 / np.mean(t_hw)))
        print("end-to-end (Python) : %.2f ms/frame -> %.1f fps" % (1e3 * np.mean(t_tot), 1 / np.mean(t_tot)))
    det.close()


if __name__ == "__main__":
    main()
