"""
Generate RTL test vectors and golden outputs for the self-checking
testbench (sim/tb_mod_top.v).

usage: python3 gen_vectors.py OUTDIR [--w 160 --h 120 --frames 24 --seed 1
                                      --fmt wide|paper --alpha_sh 7 ...]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mod_model import (Cfg, Fmt, init_params, pack_params, run_frame,  # noqa: E402
                       ccl_reference, rgb2gray, alpha_schedule)
from scene import Scene  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--w", type=int, default=160)
    ap.add_argument("--h", type=int, default=120)
    ap.add_argument("--frames", type=int, default=24)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--fmt", choices=("wide", "paper"), default="wide")
    ap.add_argument("--alpha_sh", type=int, default=7)
    ap.add_argument("--morph_mode", type=int, default=3)
    ap.add_argument("--max_lbl", type=int, default=128)
    ap.add_argument("--min_area", type=int, default=8)
    ap.add_argument("--sched", type=int, default=1, help="warm-up alpha schedule")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    fmt = Fmt.WIDE if a.fmt == "wide" else Fmt.PAPER
    cfg = Cfg(width=a.w, height=a.h, alpha_sh=a.alpha_sh, morph_mode=a.morph_mode,
              max_lbl=a.max_lbl, min_area=a.min_area, fmt=fmt)
    sc = Scene(a.w, a.h, n_obj=3, seed=a.seed, warmup=4)

    frames = [sc.frame()[0] for _ in range(a.frames)]
    p = init_params(rgb2gray(frames[0]), cfg)

    with open(os.path.join(a.outdir, "par_init.mem"), "w") as f:
        for wd in pack_params(p, fmt):
            f.write("%032x\n" % wd)
    with open(os.path.join(a.outdir, "pix.mem"), "w") as f:
        for fr in frames:
            words = (fr[..., 0].astype(np.uint32) | (fr[..., 1].astype(np.uint32) << 8)
                     | (fr[..., 2].astype(np.uint32) << 16)).ravel()
            f.write("".join("%08x\n" % wd for wd in words))

    mask_f = open(os.path.join(a.outdir, "mask_exp.mem"), "w")
    obj_t = open(os.path.join(a.outdir, "obj_exp.txt"), "w")
    obj_h = open(os.path.join(a.outdir, "obj_exp.mem"), "w")
    n_ovf = 0
    sched_f = open(os.path.join(a.outdir, "alpha_sched.mem"), "w")
    for i, fr in enumerate(frames):
        cfg.alpha_sh = alpha_schedule(i, a.alpha_sh) if a.sched else a.alpha_sh
        sched_f.write("%x\n" % cfg.alpha_sh)
        r = run_frame(fr, p, cfg, exact_ccl=True)
        if not r["overflow"]:
            ref = ccl_reference(r["mask"], cfg.min_area, cfg.max_obj)
            assert r["objs"] == ref, "single-pass CCL model != BFS reference"
        n_ovf += r["overflow"]
        p = r["params"]
        mask_f.write("".join("ff\n" if v else "00\n" for v in r["mask"].ravel()))
        obj_t.write("frame %d n %d fg %d ovf %d\n" % (i, len(r["objs"]), r["fg_count"], r["overflow"]))
        obj_h.write("%08x\n" % ((r["overflow"] << 26) | (len(r["objs"]) << 20) | r["fg_count"]))
        for j in range(cfg.max_obj):
            if j < len(r["objs"]):
                x0, x1, y0, y1, area = r["objs"][j]
                obj_t.write("  %d %d %d %d %d\n" % (x0, x1, y0, y1, area))
                obj_h.write("%015x\n" % (x0 | (x1 << 10) | (y0 << 20) | (y1 << 30) | (area << 40)))
            else:
                obj_h.write("%015x\n" % 0)
    mask_f.close()
    sched_f.close()
    cfg.alpha_sh = a.alpha_sh
    obj_t.close()
    obj_h.close()
    with open(os.path.join(a.outdir, "par_exp.mem"), "w") as f:
        for wd in pack_params(p, fmt):
            f.write("%032x\n" % wd)
    with open(os.path.join(a.outdir, "tb_cfg.vh"), "w") as f:
        for name, val in (("W", cfg.width), ("H", cfg.height), ("NF", a.frames),
                          ("ALPHA_SH", cfg.alpha_sh), ("LAMBDA2", cfg.lambda2),
                          ("T_BG", cfg.t_bg), ("VAR_INIT", cfg.var_init),
                          ("VAR_MIN", cfg.var_min), ("VAR_MAX", cfg.var_max),
                          ("MIN_AREA", cfg.min_area), ("MORPH_MODE", cfg.morph_mode),
                          ("MAX_LBL", cfg.max_lbl), ("WB", fmt.WB), ("FM", fmt.FM),
                          ("VB", fmt.VB), ("SB", fmt.SB)):
            f.write("localparam TB_%s = %d;\n" % (name, val))
    print("vectors written to %s | %s | overflow frames: %d" % (a.outdir, fmt, n_ovf))


if __name__ == "__main__":
    main()
