# Simulation waveforms (mod_top, 160x120, frame 5 of 24)

Made with Icarus Verilog from the RTL in `../../rtl`. Both runs print `TEST PASSED` (0 mask, 0 object, 0 model mismatches over all 24 frames).

| File | Shows |
|---|---|
| `waveform_1_pipeline_fill.png` | pixels and model enter; GMM stage valids v1..v5 turn on one clock apart |
| `waveform_2_foreground_mask.png` | mask stream (0x00 / 0xFF) and the CCL x, y counters |
| `waveform_3_end_of_frame.png` | TLAST, CCL DRAIN / DUMP / FIN states, frame_done |
| `waveform_4_random_stalls.png` | VALID/READY handshake with random stalls |

`source/tb_wave.v` is the project testbench plus a waveform dump block. `source/plot_waves.py` draws the figures from the dumps (needs matplotlib).

Reproduce, from the repository root:

```
cd sim
iverilog -g2005 -I vectors -DVEC_DIR='"vectors/"' -DWAVE_FILE='"wave_nostall.vcd"' \
         -o tb.vvp ../docs/waveforms/source/tb_wave.v ../rtl/*.v
vvp -n tb.vvp +nostall          # omit +nostall for the random-stall run
```

Then `python3 ../docs/waveforms/source/plot_waves.py <folder with wave_nostall.vcd and wave_stall.vcd> <output folder>`. Open a dump live with `gtkwave wave_nostall.vcd` (signals are under `tb_mod_top`, internal ones under `dut`).

The Vivado simulator run is described in `vivado_xsim/`.
