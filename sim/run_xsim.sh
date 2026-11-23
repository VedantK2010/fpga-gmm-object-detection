#!/bin/bash
# Vivado simulator (xsim), Linux.  Source <Vivado>/settings64.sh first.
#   ./run_xsim.sh                  default model (vectors/)
#   ./run_xsim.sh vectors_paper    Table I word lengths of the paper
# The simulator is compiled and run inside the vector folder so that the
# .mem files and tb_cfg.vh are found without path settings.
set -e
cd "$(dirname "$0")"
VDIR=${1:-vectors}
cd "$VDIR"
xvlog --relax -d VEC_FLAT -i . ../tb_mod_top.v ../../rtl/*.v > xvlog.log
xelab --relax -debug typical -s tb_mod_top_sim tb_mod_top > xelab.log
xsim tb_mod_top_sim -R | tee xsim_run.log | grep -E "tb_mod_top|frame .* done|ERROR|mismatches|TEST"
