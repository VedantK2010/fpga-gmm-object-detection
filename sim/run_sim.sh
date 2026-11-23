#!/bin/bash
# Icarus Verilog flow.  usage: ./run_sim.sh [gen_vectors.py args]
# PLUS=+nostall ./run_sim.sh ...   disables random stalls
# NOGEN=1 ./run_sim.sh             reuses the vectors already in ./vectors
set -e
cd "$(dirname "$0")"
[ -z "$NOGEN" ] && python3 ../model/gen_vectors.py vectors "$@"
mkdir -p out
iverilog -g2005 -I ${VDIR:-vectors} -o out/tb.vvp tb_mod_top.v ../rtl/*.v
vvp -n out/tb.vvp $PLUS | tee out/sim.log | grep -E "tb_mod_top|frame (0|23) done|ERROR|mismatches|TEST"
