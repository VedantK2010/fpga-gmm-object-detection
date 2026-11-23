"""Render timing diagrams from the VCD files of the mod_top simulation.
usage: python plot_waves.py WAVE_DIR OUT_DIR
"""
import bisect
import sys
import textwrap
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

wave_dir, out_dir = sys.argv[1], sys.argv[2]
CYC = 10000            # ps per clock (10 ns)
T = "tb_mod_top."


def parse_vcd(path):
    ids, sig, scope = {}, {}, []
    t, in_defs = 0, True
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if in_defs:
                if line.startswith("$scope"):
                    scope.append(line.split()[2])
                elif line.startswith("$upscope"):
                    scope.pop()
                elif line.startswith("$var"):
                    p = line.split()
                    ids.setdefault(p[3], []).append(".".join(scope + [p[4]]))
                elif line.startswith("$enddefinitions"):
                    in_defs = False
                continue
            if line[0] == "#":
                t = int(line[1:])
            elif line[0] in "01xzXZ":
                for n in ids.get(line[1:], []):
                    sig.setdefault(n, []).append((t, line[0]))
            elif line[0] == "b":
                v, i = line[1:].split()
                for n in ids.get(i, []):
                    sig.setdefault(n, []).append((t, v))
    return sig


class Wave:
    def __init__(self, path):
        self.s = parse_vcd(path)
        self.times = {k: [x[0] for x in v] for k, v in self.s.items()}

    def at(self, name, t):
        ts, vs = self.times[name], self.s[name]
        i = bisect.bisect_right(ts, t) - 1
        return vs[i][1] if i >= 0 else "x"

    def edges(self, name, val):
        """times at which the signal changes to val"""
        return [t for t, v in self.s[name] if v == val]

    def num(self, name, t):
        v = self.at(name, t)
        return None if "x" in v or "z" in v else int(v, 2)


def sample_cycles(w, name, t0, n):
    """value of the signal sampled just before each rising clock edge in the window"""
    return [w.num(name, t0 + k * CYC + CYC // 2 - 1) for k in range(n)]


def draw(w, path, title, caption, sigs, t0, n_cyc, shade=None):
    """sigs: list of (label, vcd name, kind, fmt) kind: clk|bit|bus ; t0 is the start in ps, aligned to a rising edge."""
    n = len(sigs)
    fig, ax = plt.subplots(figsize=(11, 0.46 * n + 1.5))
    t1 = t0 + n_cyc * CYC
    h = 0.62
    for k in range(n_cyc + 1):                      # cycle grid
        ax.axvline(k, color="#e6e6e6", lw=0.6, zorder=0)
    if shade:
        a, b = shade
        for k in range(n_cyc):
            tt = t0 + k * CYC + CYC // 2 - 1
            if w.at(T + a, tt) == "1" and w.at(T + b, tt) == "1":
                ax.axvspan(k, k + 1, color="#cfe8cf", zorder=0)
    for row, (label, name, kind, fmt) in enumerate(sigs):
        y = n - 1 - row
        full = name if name.startswith("tb_mod_top") else T + name
        # collect segments inside the window
        ts = [t for t in w.times[full] if t0 <= t <= t1]
        pts = [t0] + ts + [t1]
        if kind in ("clk", "bit"):
            xs, ys = [], []
            for a, b in zip(pts[:-1], pts[1:]):
                v = w.at(full, a + 1 if a != t0 else a)
                lvl = 1 if v == "1" else (0 if v == "0" else 0.5)
                xs += [(a - t0) / CYC, (b - t0) / CYC]
                ys += [y + lvl * h, y + lvl * h]
            ax.plot(xs, ys, color="#1f3b73" if kind == "bit" else "#555555", lw=1.4, solid_joinstyle="miter")
            ax.plot([0, n_cyc], [y, y], color="#dddddd", lw=0.5, zorder=0)
        else:
            for a, b in zip(pts[:-1], pts[1:]):
                v = w.at(full, a)
                x0, x1 = (a - t0) / CYC, (b - t0) / CYC
                if x1 - x0 < 1e-9:
                    continue
                d = min(0.12, (x1 - x0) / 3)
                poly = [(x0, y + h / 2), (x0 + d, y + h), (x1 - d, y + h), (x1, y + h / 2), (x1 - d, y), (x0 + d, y)]
                ax.add_patch(Polygon(poly, closed=True, fill=False, ec="#1f3b73", lw=1.2))
                try:
                    iv = int(v, 2) if v not in ("x", "z") and "x" not in v else None
                except ValueError:
                    iv = None
                txt = "x" if iv is None else (fmt(iv) if fmt else "%X" % iv)
                if x1 - x0 > 0.75:
                    fs = 8 if (x1 - x0) * 0.5 * 11 / n_cyc * 72 > len(txt) * 5.2 else 6
                    ax.text((x0 + x1) / 2, y + h / 2, txt, ha="center", va="center", fontsize=fs)
    ax.set_yticks([n - 1 - i + h / 2 for i in range(n)])
    ax.set_yticklabels([s[0] for s in sigs], fontsize=9)
    ax.set_xlim(0, n_cyc)
    ax.set_ylim(-0.3, n - 0.2)
    ax.set_xlabel("clock cycles (10 ns each)", fontsize=9)
    ax.tick_params(axis="x", labelsize=8)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.tick_params(axis="y", length=0)
    fig.suptitle(title, fontsize=11, fontweight="bold", x=0.02, ha="left")
    cap = "\n".join(textwrap.wrap(caption, 150))
    fig.text(0.02, 0.012, cap, fontsize=8, color="#444444", ha="left")
    fig.tight_layout(rect=(0, 0.07, 1, 0.95))
    fig.savefig(path, dpi=170)
    plt.close(fig)
    print("wrote", path)


def align(t):
    return (t // CYC) * CYC


STATE = {0: "RUN", 1: "DUMP", 2: "FIN", 3: "DRAIN"}
hex8 = lambda v: "%02X" % v

# ------------------------------------------------------------------ no-stall run (frame 5)
w = Wave(wave_dir + "/wave_nostall.vcd")
t_pix = w.edges(T + "pix_v", "1")[0]
t_ff = [t for t in w.edges(T + "mo_d", "11111111") if w.at(T + "mo_v", t + 1) == "1"][0]
t_first_mask = w.edges(T + "mo_v", "1")[0]
t_last = [t for t in w.edges(T + "mo_l", "1") if t > t_first_mask][0]
t_done = [t for t in w.edges(T + "frame_done", "1") if t > t_last][0]
print("first pixel cycle", t_pix // CYC, "| first mask beat", t_first_mask // CYC, "(latency", (t_first_mask - t_pix) // CYC,
      "cycles) | first 0xFF", t_ff // CYC, "| TLAST", t_last // CYC, "| frame_done", t_done // CYC,
      "| labels used", w.num(T + "dut.u_ccl.nxt", t_last))

draw(w, out_dir + "/waveform_1_pipeline_fill.png",
     "mod_top simulation, frame 5 (no stalls): pixels enter and the 5-stage GMM pipeline fills",
     "Inputs: 32-bit RGBX pixel stream and 128-bit model stream. One pixel per clock enters the GMM; the stage valid "
     "signals v1..v5 turn on one clock apart, then the updated model words leave at one per clock.",
     [("aclk", "aclk", "clk", None), ("aresetn", "aresetn", "bit", None),
      ("pix TVALID", "pix_v", "bit", None), ("pix TREADY", "pix_r", "bit", None),
      ("pix TDATA (RGB)", "pix_d", "bus", lambda v: "%06X" % (v & 0xFFFFFF)),
      ("model-in TVALID", "par_v", "bit", None), ("model-in TREADY", "par_r", "bit", None),
      ("GMM stage 1 valid", "dut.u_gmm.v1", "bit", None), ("GMM stage 2 valid", "dut.u_gmm.v2", "bit", None),
      ("GMM stage 3 valid", "dut.u_gmm.v3", "bit", None), ("GMM stage 4 valid", "dut.u_gmm.v4", "bit", None),
      ("GMM stage 5 valid", "dut.u_gmm.v5", "bit", None), ("model-out TVALID", "po_v", "bit", None)],
     align(t_pix) - 3 * CYC, 22)

draw(w, out_dir + "/waveform_2_foreground_mask.png",
     "mod_top simulation, frame 5 (no stalls): foreground pixels in the mask stream",
     "The mask leaves one byte per clock: 0x00 = background, 0xFF = foreground. The first mask byte appears "
     "%d clocks after the first input pixel (the morphology line buffers and the CCL have to fill). "
     "The CCL x and y counters follow the stream." % ((t_first_mask - t_pix) // CYC),
     [("aclk", "aclk", "clk", None), ("mask TVALID", "mo_v", "bit", None), ("mask TREADY", "mo_r", "bit", None),
      ("mask TDATA", "mo_d", "bus", hex8), ("mask TLAST", "mo_l", "bit", None),
      ("CCL x", "dut.u_ccl.x", "bus", lambda v: str(v)), ("CCL y", "dut.u_ccl.y", "bus", lambda v: str(v))],
     align(t_ff) - 10 * CYC, 30)

start = align(t_last) - 6 * CYC
ncyc = int((t_done - start) // CYC) + 8
draw(w, out_dir + "/waveform_3_end_of_frame.png",
     "mod_top simulation, frame 5: end of frame (TLAST, label scan, frame_done)",
     "TLAST marks the last pixel of the 160x120 mask stream (it is only meaningful while TVALID is high). The CCL then "
     "drains one cycle, scans its label table in the DUMP state (one label per clock, %d labels used in this frame), "
     "publishes the object list in FIN and frame_done pulses." % w.num(T + "dut.u_ccl.nxt", t_last),
     [("aclk", "aclk", "clk", None), ("mask TVALID", "mo_v", "bit", None), ("mask TDATA", "mo_d", "bus", hex8),
      ("mask TLAST", "mo_l", "bit", None),
      ("CCL state", "dut.u_ccl.state", "bus", lambda v: STATE.get(v, str(v))),
      ("CCL labels used", "dut.u_ccl.nxt", "bus", lambda v: str(v)), ("frame_done", "frame_done", "bit", None)],
     start, ncyc)

# ------------------------------------------------------------------ run with random stalls
w2 = Wave(wave_dir + "/wave_stall.vcd")
t0 = align(w2.edges(T + "pix_v", "1")[0]) + 3000 * CYC
draw(w2, out_dir + "/waveform_4_random_stalls.png",
     "mod_top simulation, frame 5: random stalls on every stream (VALID/READY handshake)",
     "The testbench inserts random gaps on the inputs and random back-pressure on the outputs (about 30%). A beat "
     "moves only when VALID and READY are both high (green, shown for the pixel input). The results still match "
     "the golden files bit for bit.",
     [("aclk", "aclk", "clk", None),
      ("pix TVALID", "pix_v", "bit", None), ("pix TREADY", "pix_r", "bit", None),
      ("model-in TVALID", "par_v", "bit", None), ("model-in TREADY", "par_r", "bit", None),
      ("model-out TVALID", "po_v", "bit", None), ("model-out TREADY", "po_r", "bit", None),
      ("mask TVALID", "mo_v", "bit", None), ("mask TREADY", "mo_r", "bit", None)],
     t0, 48, shade=("pix_v", "pix_r"))
