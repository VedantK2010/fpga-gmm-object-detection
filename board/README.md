# Prebuilt PYNQ-Z2 overlay

* `mod_overlay.bit`: bitstream for the default `wide` (126-bit model) build at 100 MHz.
* `mod_overlay.hwh`: hardware handoff file. PYNQ needs it next to the `.bit`, with the same base name.

Built with Vivado 2025.2 for the PYNQ-Z2 (XC7Z020). To rebuild, see `../vivado/create_project.tcl` and the guide in `../docs/BUILD_AND_RUN.md`.

Use: copy both files and `../sw/pynq/*` to the board and run `run_demo.py` with `sudo`.
