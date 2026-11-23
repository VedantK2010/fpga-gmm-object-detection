#!/bin/bash
# Full regression (Icarus Verilog + Python). Every case must print TEST PASSED.
cd "$(dirname "$0")"
run() { echo "== $1"; shift; "$@" 2>&1 | grep -E "overflow frames|TEST|frame 2 done"; }
run "wide model, close+open, random stalls"     ./run_sim.sh --fmt wide  --morph_mode 3
run "paper Table I word lengths, close+open"     ./run_sim.sh --fmt paper --morph_mode 3 --seed 4
run "paper word lengths, slow learning a=8"      ./run_sim.sh --fmt paper --morph_mode 1 --alpha_sh 8 --seed 5
run "no morphology"                              ./run_sim.sh --morph_mode 0 --seed 1
run "opening"                                    ./run_sim.sh --morph_mode 1 --seed 2
run "closing"                                    ./run_sim.sh --morph_mode 2 --seed 3
run "fast learning a=2, label overflow"          ./run_sim.sh --morph_mode 0 --seed 7 --alpha_sh 2
run "64-label core, 96x40"                       ./run_sim.sh --morph_mode 0 --seed 3 --max_lbl 64 --w 96 --h 40 --frames 30
PLUS=+nostall run "640x480, no stalls"           ./run_sim.sh --w 640 --h 480 --frames 3
echo "== stand-alone CCL, 400 random images"
python3 gen_ccl_vectors.py ccl_vec 400 && mkdir -p out && \
iverilog -g2005 -I ccl_vec -o out/tb_ccl.vvp tb_ccl.v ../rtl/ccl_single_pass.v && \
vvp -n out/tb_ccl.vvp > out/ccl.log && \
(diff -q ccl_vec/ccl_exp.txt out/ccl_rtl.txt > /dev/null && echo "TEST PASSED" || echo "TEST FAILED")
# restore the default vector set shipped for xsim
python3 ../model/gen_vectors.py vectors > /dev/null
