# Real-Time Moving Object Detection on Zynq: GMM + Single-Pass CCL

This project builds a moving-object detector in the Zynq-7000 programmable logic. Its GMM stage follows the hardware-oriented formulation of the OpenCV GMM in:

> M. Genovese and E. Napoli, "ASIC and FPGA Implementation of the Gaussian Mixture Model Algorithm for Real-Time Segmentation of High Definition Video," *IEEE TVLSI*, vol. 22, no. 3, pp. 537–547, 2014.

The project extends that paper with:

* morphological filtering,
* single-pass connected component labeling (CCL),
* AXI DMA / AXI-Lite integration for PYNQ and bare-metal Zynq.

```
RGB ─► Y ─► OpenCV GMM (K=3, paper formulation) ─► 3×3 closing/opening ─► single-pass CCL ─► boxes + areas
                ▲                 │
  GMM model (99 or 126 bits/pixel in DDR, read and written back every frame by AXI DMA)
```

**GMM (paper equations).** The GMM stage implements the following equations from the paper:

* match (eq. 15)
* the update of the matched Gaussian and the decay of the other weights (eq. 3, 6)
* α_k = α_w / w_k quantized to a power of two (eq. 4, 12)
* ordering by the inverse fitness IF = σ²·2^(2(n_k−n_w)) (eq. 13)
* no-match replacement with w = 1/msumtot (eq. 7)
* background test (eq. 8)

**Word lengths.** These are synthesis parameters:

* `wide` (default) uses 12/12/14/4 bits for w/μ/σ²/matchsum, 126 bits per pixel.
* `paper` uses Table I of the paper, 8/10/11/4 bits, 99 bits per pixel.

**Throughput.** The pipeline processes 1 pixel per clock. A 640×480 frame takes 309,778 cycles, which is 3.10 ms at 100 MHz.

**Verification.** The RTL is plain Verilog-2001. It matches the Python reference model bit for bit (masks, object tables and GMM model) under random stream stalls, for both word-length sets.

## Folder layout

| Folder | Contents |
|---|---|
| `rtl/` | `mod_top.v` (top), `gmm_core.v`, `morph_chain.v`, `bin_morph3x3.v`, `ccl_single_pass.v`, `mod_axil_regs.v` |
| `sim/` | `tb_mod_top.v` (self-checking), `tb_ccl.v`, `vectors/` (default model), `vectors_paper/` (Table I), `run_xsim.bat`, `run_xsim.sh`, `run_sim.sh`, `run_all_tests.sh` |
| `vivado/` | `create_project.tcl`, which builds the block design, bitstream, `.hwh`, `.xsa` and the simulation set |
| `model/` | `mod_model.py` (bit-exact model), `scene.py` (synthetic test scene), `gen_vectors.py` |
| `sw/pynq/` | `mod_driver.py`, `run_demo.py`, `mod_demo.ipynb`, `mod_model.py` |
| `sw/baremetal/` | Vitis standalone application (`main.c`, `mod_hw.h`) |
| `syn/` | Yosys scripts and reports |
| `eval/` | Accuracy evaluation, word-length study, figures (`results/`) |
| `report/` | IEEE conference-format report (`report.tex`, `report.pdf`, figures) |

## 1. Simulate in Vivado (no Python needed; test vectors are included)

**Option A: Vivado project (GUI).** After step 2 below has created the project, click **Flow ▸ Run Simulation ▸ Run Behavioral Simulation**. The Tcl console ends with `TEST PASSED`.

**Option B: command line.** Run from the `sim/` folder:

* Windows: open the *Vivado Tcl Shell* (or any prompt with Vivado on PATH) and run:
  * `run_xsim.bat` for the default model
  * `run_xsim.bat vectors_paper` for the paper's Table I word lengths
* Linux: `source <Vivado>/settings64.sh`, then run `./run_xsim.sh` or `./run_xsim.sh vectors_paper`.

The testbench prints one line per frame and then `TEST PASSED` (or the first mismatches). A full run takes a few minutes.

**Icarus Verilog plus Python:**

```bash
cd sim
./run_sim.sh                 # regenerates the vectors with Python, then simulates
./run_all_tests.sh           # full regression: both formats, all morphology modes,
                             # label overflow, 640x480, 400 random CCL images
```

## 2. Build the bitstream (Vivado 2020.2 or newer)

```
cd vivado
vivado -mode batch -source create_project.tcl -tclargs pynq-z2 100 1 wide
```

| Argument | Values |
|---|---|
| board | `pynq-z2`, `pynq-z1`, `zedboard`, `zybo-z7-20`, `zc702` |
| PL clock | MHz, for example `100` |
| build | `1` builds the bitstream; `0` only creates the project |
| gmm | `wide` (default) or `paper` (Table I word lengths) |

**From the GUI:** in the Tcl console, run:

```
cd C:/path/to/mod_fpga/vivado
set argv {pynq-z2 100 0 wide}
source create_project.tcl
```

**Board files.** PYNQ-Z1/Z2 and Zybo need their board files installed (*Tools ▸ Vivado Store ▸ Boards*, or copy the board files into `<Vivado>/data/boards/board_files`). ZedBoard and ZC702 board files come with Vivado.

**Outputs** go to `vivado/output/<board>_<gmm>/`:

* `mod_overlay.bit` and `mod_overlay.hwh` for PYNQ
* `mod_overlay.xsa` for Vitis
* `utilization.rpt`, `timing.rpt` and `power.rpt`

**If timing fails** at 100 MHz, rerun with a lower clock, for example `-tclargs pynq-z2 75 1 wide`. At 50 MHz VGA still runs at over 150 fps.

## 3. Run on the PYNQ board

1. Copy these files into one folder on the board:
   * `mod_overlay.bit` and `mod_overlay.hwh`
   * all of `sw/pynq/*`
   * a video, for example OpenCV's `samples/data/vtest.avi`
2. Run the demo with `sudo`:

```
sudo python3 run_demo.py --video vtest.avi --frames 300 --out result.avi   # boxes + mask video, timing CSV
sudo python3 run_demo.py --video vtest.avi --verify 20                     # hardware vs model, bit-exact
sudo python3 run_demo.py --video vtest.avi --swbench 100                   # ARM-only OpenCV baseline
```

You can also use `mod_demo.ipynb` in Jupyter. The driver reads the GMM word lengths from the core, so it works with both builds.

## 4. Run bare metal (any Zynq-7020 board)

1. In Vitis, create a platform from `mod_overlay.xsa`.
2. Create an empty standalone application and add `sw/baremetal/main.c` and `mod_hw.h`.

The application generates a moving scene, processes 200 frames, and prints the boxes and timing on the UART at 115200 baud. If `xparameters.h` has no `XPAR_MOD_TOP_0_BASEADDR`, take the base address from the Address Editor.

## Register map (AXI4-Lite)

| Offset | Name | Description |
|---|---|---|
| 0x000 | CTRL | [0] soft reset; [1] gray input (byte 0); [3:2] morphology: 0 none, 1 open, 2 close, 3 close+open (default) |
| 0x004 | STATUS | [0] label overflow in last frame; [31:16] frame counter |
| 0x008 / 0x00C | WIDTH / HEIGHT | 5 ≤ W ≤ 1024, 2 ≤ H ≤ 1024 |
| 0x010 | SHIFTS | [3:0] a, where α_w = 2^-a (default 7) |
| 0x014 | LAMBDA2 | λ² in Q4.4 (100 = 6.25, λ = 2.5) |
| 0x018 | T_BG | T in Q0.12 (2867 = 0.70) |
| 0x020 / 0x024 / 0x028 | VAR_INIT / VAR_MIN / VAR_MAX | integer σ² (900 / 16 / 16383) |
| 0x02C | MIN_AREA | minimum object area in pixels (64) |
| 0x030 | NUM_OBJ | objects in the last frame (≤ 32) |
| 0x034 | FG_COUNT | foreground pixels in the last frame |
| 0x038 | ID | 0x4D4F4401 |
| 0x03C | CAPS | [7:0] label bits; [15:8] object-table bits; [31:16] maximum width |
| 0x040 | GMM_FMT | [3:0] WB; [7:4] FM; [11:8] VB−8; [15:12] SB |
| 0x200 + 16·i | OBJ[i] | +0 {maxx, minx}; +4 {maxy, miny}; +8 area |

**GMM model word.** Gaussian k sits at bits [GW·k + GW−1 : GW·k] as {matchsum, σ², μ, w}, with w in the LSBs. GW is 42 for the default build and 33 for the paper build. The value encodings are:

* w = code / 2^WB
* μ = code / 2^FM
* σ² = code · 2^(14−VB)

**Pixel word.** 32 bits, {X, B, G, R}, with R in byte 0.

**Per frame, the software does the following:**

1. Write SHIFTS from the warm-up schedule: `a_t = min(a, ceil(log2(t+1)))`. This gives α_w ≈ 1/t during the first 2^a frames.
2. Start both S2MM transfers: the model out (16·W·H bytes) and the mask (W·H bytes).
3. Start both MM2S transfers: the model in and the pixels.
4. Wait until all four DMA transfers finish and the frame counter changes.
5. Read the object table.
6. Swap the two model buffers.

## Limits

* At most 127 provisional labels per frame (STATUS[0] reports overflow) and at most 32 reported objects. Set `LBL_W = 8` on `mod_top` for 255 labels.
* α_w must be ≥ 2^-7 with the 126-bit model. Slower rates lose sub-LSB updates; see the report.
* Change the configuration only between frames. Pulse CTRL[0] after changing the frame size or the morphology mode.
