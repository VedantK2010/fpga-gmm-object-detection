"""Run the exact single-pass CCL model on every frame of the real clip and
record label usage / overflow (used for the claim in the report)."""
import sys, os, json
import numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model"))
import mod_model as mm

video, n = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 300
W, H = 640, 480
cfg = mm.Cfg(width=W, height=H, min_area=64)
cap = cv2.VideoCapture(video)
p = None; ovf = 0; maxobj = 0; maxlbl = 0; mism = 0
for t in range(n):
    ok, fr = cap.read()
    if not ok: break
    fr = cv2.resize(fr, (W, H), interpolation=cv2.INTER_AREA)
    rx = np.zeros((H, W, 4), np.uint8); rx[..., 0], rx[..., 1], rx[..., 2] = fr[..., 2], fr[..., 1], fr[..., 0]
    g = mm.rgb2gray(rx)
    if p is None: p = mm.init_params(g, cfg)
    cfg.alpha_sh = mm.alpha_schedule(t, 7)
    fg, p = mm.gmm_step(g, p, cfg)
    m = mm.morph(fg, 3)
    # labels used = number of "new label" events; count via a large-L run
    big = mm.Cfg(width=W, height=H, min_area=64, max_lbl=4096)
    objs_big, _ = mm.ccl_single_pass(m, big)
    objs, o = mm.ccl_single_pass(m, cfg)
    ovf += o; maxobj = max(maxobj, len(objs))
    mism += objs != objs_big
print(json.dumps(dict(frames=t + 1, overflow_frames=ovf, max_objects=maxobj, frames_differing_from_unbounded=mism)))
