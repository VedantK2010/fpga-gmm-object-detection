#!/bin/bash
# Yosys synthesis + checks for the moving-object-detection core
set -e
cd "$(dirname "$0")"; mkdir -p reports
yosys -q -l reports/yosys_top.log synth_xc7.ys
yosys -q -l reports/yosys_modules.log synth_modules.ys
for m in gmm_core ccl_single_pass bin_morph3x3; do
  sed "s/TOP/$m/g" depth_module.ys.in > reports/depth_$m.ys
  yosys -q reports/depth_$m.ys > /dev/null
  echo "$m: $(grep 'Longest topological path' reports/ltp_$m.txt)"
done
grep -A25 "Number of cells" reports/util_top.txt
