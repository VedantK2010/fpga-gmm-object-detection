"""Random-image vectors for the stand-alone CCL testbench."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model"))
from mod_model import Cfg, ccl_single_pass, ccl_reference

out = sys.argv[1]; n = int(sys.argv[2]) if len(sys.argv) > 2 else 300
os.makedirs(out, exist_ok=True)
rng = np.random.default_rng(2024)
dims, bits, exp = [], [], []
n_ovf = 0
for t in range(n):
    w = int(rng.choice([5, 6, 17, 37, 64, 100])); h = int(rng.choice([2, 3, 9, 23, 48]))
    kind = t % 4
    if kind == 0:                                  # salt noise
        m = rng.random((h, w)) < rng.uniform(0.05, 0.75)
    elif kind == 1:                                # blocky blobs
        m = np.kron(rng.random((h // 3 + 1, w // 3 + 1)) < 0.45, np.ones((3, 3), bool))[:h, :w]
    elif kind == 2:                                # combs / U-shapes (late merges)
        m = np.zeros((h, w), bool); m[:, ::2] = True; m[h - 1, :] = rng.random() < 0.7
        m &= rng.random((h, w)) < 0.97
    else:                                          # concentric rings / spirals
        yy, xx = np.mgrid[0:h, 0:w]; r = np.hypot(yy - h / 2, xx - w / 2)
        m = (r.astype(int) % 3 == 0) & (rng.random((h, w)) < 0.9)
    cfg = Cfg(width=w, height=h, max_lbl=256, min_area=1, max_obj=32)
    objs, ovf = ccl_single_pass(m, cfg)
    if not ovf:
        assert objs == ccl_reference(m, 1, 32)
    n_ovf += ovf
    dims.append((w, h)); bits.extend(int(v) for v in m.ravel())
    exp.append("img %d n %d ovf %d\n" % (t, len(objs), ovf) + "".join("  %d %d %d %d %d\n" % o for o in objs))
open(os.path.join(out, "dims.hex"), "w").write("".join("%03x%03x\n" % d for d in dims))
open(os.path.join(out, "bits.hex"), "w").write("".join("%x\n" % b for b in bits))
open(os.path.join(out, "ccl_exp.txt"), "w").write("".join(exp))
open(os.path.join(out, "ccl_cfg.vh"), "w").write("localparam N_IMG = %d;\nlocalparam N_BITS = %d;\n" % (n, len(bits)))
print("images:", n, "pixels:", len(bits), "overflow images:", n_ovf)
