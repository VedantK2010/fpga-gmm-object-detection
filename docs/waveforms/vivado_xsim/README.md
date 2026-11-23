# Vivado simulator (xsim) run of tb_mod_top

The same self-checking testbench also passes in the Vivado simulator: `TEST PASSED`, with 0 mask, object and model mismatches over all 24 frames (160x120).

* `run.tcl` logs the signals shown in the waveform figures (clock, reset, the pixel / model / mask streams, `frame_done`, the GMM stage valids and the CCL state).
* No stalls: `xsim tb_sim -tclbatch run.tcl -wdb tb_nostall.wdb -testplusarg nostall` (19,858 cycles per frame, 0.967 pixel/clock).
* Random stalls on all streams (the testbench default): `xsim tb_sim -tclbatch run.tcl -wdb tb_stall.wdb`.

Build the simulation snapshot first, from `sim/` (it contains the `.mem` vectors and `tb_cfg.vh`):

```
xvlog --relax -d VEC_FLAT -i . tb_mod_top.v ../rtl/*.v
xelab --relax -debug typical -s tb_sim tb_mod_top
```

Open a result with `File > Open Waveform Database...` in Vivado.

If `xelab` fails at the link step with "cannot find crt1.o / -lm" on Linux, set `LIBRARY_PATH=/usr/lib/x86_64-linux-gnu` before running it.
