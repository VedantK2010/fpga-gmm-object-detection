#------------------------------------------------------------------------------
# create_project.tcl
# Builds the moving-object-detection overlay for Zynq-7000 boards.
#
# usage (from this directory):
#   vivado -mode batch -source create_project.tcl -tclargs <board> [fclk_mhz] [build] [gmm]
#     board    : pynq-z2 | pynq-z1 | zedboard | zybo-z7-20 | zc702   (default pynq-z2)
#     fclk_mhz : PL clock in MHz (default 100)
#     build    : 1 = run synthesis/implementation/bitstream (default 1)
#                0 = only create the project / block design (open it in the GUI)
#     gmm      : wide  = 12/12/14/4-bit GMM parameters (default)
#                paper = word lengths of Table I of Genovese & Napoli (8/10/11/4)
#
# GUI alternative: Tools > Run Tcl Script..., then pick this file (defaults
# are used), or in the Tcl console:
#   cd <path>/vivado ; set argv {pynq-z2 100 0 wide} ; source create_project.tcl
#
# Outputs (./output/<board>/):
#   mod_overlay.bit, mod_overlay.hwh  -> copy both to the PYNQ board
#   mod_overlay.xsa                   -> hardware platform for Vitis (bare metal)
#   utilization.rpt, timing.rpt, power.rpt
#------------------------------------------------------------------------------
if {![info exists argv]} { set argv {} }
set board  [expr {[llength $argv] > 0 ? [lindex $argv 0] : "pynq-z2"}]
set fclk   [expr {[llength $argv] > 1 ? [lindex $argv 1] : 100}]
set build  [expr {[llength $argv] > 2 ? [lindex $argv 2] : 1}]
set gmm    [expr {[llength $argv] > 3 ? [lindex $argv 3] : "wide"}]

set here   [file normalize [file dirname [info script]]]
set rtl    [file normalize $here/../rtl]
set proj   mod_${board}_${gmm}
set pdir   $here/build/$proj
set odir   $here/output/${board}_${gmm}
set bdname mod_bd

switch -- $board {
    pynq-z2    { set part xc7z020clg400-1; set bpat "*pynq-z2*" }
    pynq-z1    { set part xc7z020clg400-1; set bpat "*pynq-z1*" }
    zedboard   { set part xc7z020clg484-1; set bpat "*zedboard*" }
    zybo-z7-20 { set part xc7z020clg400-1; set bpat "*zybo-z7-20*" }
    zc702      { set part xc7z020clg484-1; set bpat "*zc702*" }
    default    { error "unknown board '$board'" }
}

create_project $proj $pdir -part $part -force
set bpart [lindex [lsort [get_board_parts -quiet -latest_file_version $bpat]] end]
set preset 1
if {$bpart eq ""} {
    if {[string match "pynq-z*" $board]} {
        # PYNQ: Linux on the board has already configured DDR/MIO, and loading
        # an overlay only reprograms the PL and the PL clocks, so the board
        # preset is not needed. (Not suitable for bare-metal boot.)
        puts "WARNING: board files for '$board' not found - building for part $part"
        puts "WARNING: without the board preset (fine for PYNQ overlays, not for bare metal)."
        set preset 0
    } else {
        error "Board files for '$board' not found. Install them (Vivado: Tools >\
               Vivado Store > Boards, or copy the board files into\
               <Vivado>/data/boards/board_files) and run again."
    }
} else {
    set_property board_part $bpart [current_project]
    puts "INFO: using board part $bpart"
}

add_files -norecurse [glob $rtl/*.v]
update_compile_order -fileset sources_1

# self-checking testbench (Flow > Run Simulation > Run Behavioral Simulation)
set simdir [file normalize $here/../sim]
set vecdir [expr {$gmm eq "paper" ? "$simdir/vectors_paper" : "$simdir/vectors"}]
add_files -fileset sim_1 -norecurse $simdir/tb_mod_top.v
add_files -fileset sim_1 -norecurse [glob $vecdir/*.mem]
set_property include_dirs [list $vecdir] [get_filesets sim_1]
set_property verilog_define {VEC_FLAT} [get_filesets sim_1]
set_property top tb_mod_top [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {-all} -objects [get_filesets sim_1]

#------------------------------------------------------------------------------
# Block design
#------------------------------------------------------------------------------
create_bd_design $bdname

set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7 processing_system7_0]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config [list make_external "FIXED_IO, DDR" apply_board_preset $preset Master "Disable" Slave "Disable"] $ps
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP1 {1} \
    CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {64} \
    CONFIG.PCW_S_AXI_HP1_DATA_WIDTH {64} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ $fclk \
    CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
    CONFIG.PCW_IRQ_F2P_INTR {1} \
] $ps

# DMA for the GMM parameters (128-bit streams, 26-bit length -> up to 64 MB)
set dpar [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma axi_dma_par]
set_property -dict [list \
    CONFIG.c_include_sg {0} \
    CONFIG.c_sg_length_width {26} \
    CONFIG.c_m_axi_mm2s_data_width {128} \
    CONFIG.c_m_axis_mm2s_tdata_width {128} \
    CONFIG.c_m_axi_s2mm_data_width {128} \
    CONFIG.c_s_axis_s2mm_tdata_width {128} \
    CONFIG.c_mm2s_burst_size {32} \
    CONFIG.c_s2mm_burst_size {32} \
] $dpar

# DMA for pixels in (32-bit RGBX) and foreground mask out (8-bit)
set dpix [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma axi_dma_pix]
set_property -dict [list \
    CONFIG.c_include_sg {0} \
    CONFIG.c_sg_length_width {26} \
    CONFIG.c_m_axi_mm2s_data_width {32} \
    CONFIG.c_m_axis_mm2s_tdata_width {32} \
    CONFIG.c_m_axi_s2mm_data_width {32} \
    CONFIG.c_s_axis_s2mm_tdata_width {8} \
    CONFIG.c_mm2s_burst_size {64} \
    CONFIG.c_s2mm_burst_size {64} \
] $dpix

# detector core (RTL module reference)
set core [create_bd_cell -type module -reference mod_top mod_top_0]
if {$gmm eq "paper"} {
    set_property -dict [list CONFIG.GMM_WB {8} CONFIG.GMM_FM {2} CONFIG.GMM_VB {11} CONFIG.GMM_SB {4}] $core
}

# streams
connect_bd_intf_net [get_bd_intf_pins axi_dma_pix/M_AXIS_MM2S] [get_bd_intf_pins mod_top_0/s_axis_pix]
connect_bd_intf_net [get_bd_intf_pins axi_dma_par/M_AXIS_MM2S] [get_bd_intf_pins mod_top_0/s_axis_par]
connect_bd_intf_net [get_bd_intf_pins mod_top_0/m_axis_par]  [get_bd_intf_pins axi_dma_par/S_AXIS_S2MM]
connect_bd_intf_net [get_bd_intf_pins mod_top_0/m_axis_mask] [get_bd_intf_pins axi_dma_pix/S_AXIS_S2MM]

# control path (GP0) - automation adds interconnect, clocks and resets
set clk /processing_system7_0/FCLK_CLK0
foreach slv {axi_dma_par/S_AXI_LITE axi_dma_pix/S_AXI_LITE mod_top_0/s_axi_ctrl} {
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config [list \
        Clk_master $clk Clk_slave $clk Clk_xbar $clk \
        Master /processing_system7_0/M_AXI_GP0 Slave /$slv \
        intc_ip {New AXI Interconnect} master_apm {0}] [get_bd_intf_pins $slv]
}

# data path: parameter DMA on HP0, pixel/mask DMA on HP1
foreach {mst hp} {axi_dma_par/M_AXI_MM2S S_AXI_HP0 axi_dma_par/M_AXI_S2MM S_AXI_HP0 \
                  axi_dma_pix/M_AXI_MM2S S_AXI_HP1 axi_dma_pix/M_AXI_S2MM S_AXI_HP1} {
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config [list \
        Clk_master $clk Clk_slave $clk Clk_xbar $clk \
        Master /$mst Slave /processing_system7_0/$hp \
        intc_ip {Auto} master_apm {0}] [get_bd_intf_pins processing_system7_0/$hp]
}

# core clock / reset
set rstn [get_bd_pins -of_objects [get_bd_cells -filter {VLNV =~ *proc_sys_reset*}] -filter {NAME == peripheral_aresetn}]
# (the AXI automation usually connects these already through ASSOCIATED_BUSIF)
if {[llength [get_bd_nets -quiet -of_objects [get_bd_pins mod_top_0/aclk]]] == 0} {
    connect_bd_net [get_bd_pins $clk] [get_bd_pins mod_top_0/aclk]
}
if {[llength [get_bd_nets -quiet -of_objects [get_bd_pins mod_top_0/aresetn]]] == 0} {
    connect_bd_net [lindex $rstn 0] [get_bd_pins mod_top_0/aresetn]
}

# interrupts (DMA completion + frame done) -> IRQ_F2P
set cc [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat irq_concat]
set_property CONFIG.NUM_PORTS {3} $cc
connect_bd_net [get_bd_pins axi_dma_par/s2mm_introut] [get_bd_pins irq_concat/In0]
connect_bd_net [get_bd_pins axi_dma_pix/s2mm_introut] [get_bd_pins irq_concat/In1]
connect_bd_net [get_bd_pins mod_top_0/frame_done]     [get_bd_pins irq_concat/In2]
connect_bd_net [get_bd_pins irq_concat/dout] [get_bd_pins processing_system7_0/IRQ_F2P]

assign_bd_address
regenerate_bd_layout
validate_bd_design
save_bd_design

set bdfile [get_files $bdname.bd]
generate_target all $bdfile
make_wrapper -files $bdfile -top
set wrapper [glob $pdir/$proj.*/sources_1/bd/$bdname/hdl/${bdname}_wrapper.v]
add_files -norecurse $wrapper
set_property top ${bdname}_wrapper [current_fileset]
update_compile_order -fileset sources_1

puts "INFO: address map"
foreach seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces processing_system7_0/Data]] {
    puts [format "  %-40s 0x%08X  %s" [get_property NAME $seg] [get_property OFFSET $seg] [get_property RANGE $seg]]
}

if {!$build} { return }

#------------------------------------------------------------------------------
# Synthesis / implementation / bitstream
#------------------------------------------------------------------------------
set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
launch_runs synth_1 -jobs 4
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

file mkdir $odir
open_run impl_1
report_utilization        -file $odir/utilization.rpt
report_utilization -hierarchical -file $odir/utilization_hier.rpt
report_timing_summary -max_paths 20 -file $odir/timing.rpt
report_power              -file $odir/power.rpt

# worst path per module of the detector core (timing_<module>.rpt)
foreach m {u_gmm u_skid u_morph u_ccl u_regs} {
    set eps [get_cells -quiet -hierarchical -filter "NAME =~ */mod_top_0/inst/$m/* && IS_SEQUENTIAL"]
    if {[llength $eps] == 0} { continue }
    report_timing -max_paths 5 -to $eps -file $odir/timing_$m.rpt
    set p [get_timing_paths -quiet -max_paths 1 -to $eps]
    if {[llength $p]} { puts [format "INFO: %-8s worst setup slack %s ns" $m [get_property SLACK $p]] }
}

set bit [glob $pdir/$proj.runs/impl_1/*.bit]
file copy -force $bit $odir/mod_overlay.bit
set hwh [glob -nocomplain $pdir/$proj.gen/sources_1/bd/$bdname/hw_handoff/$bdname.hwh \
                          $pdir/$proj.srcs/sources_1/bd/$bdname/hw_handoff/$bdname.hwh]
file copy -force [lindex $hwh 0] $odir/mod_overlay.hwh
write_hw_platform -fixed -include_bit -force -file $odir/mod_overlay.xsa

set wns [get_property STATS.WNS [get_runs impl_1]]
puts "INFO: done. WNS = $wns ns at $fclk MHz. Files in $odir"
if {$wns < 0} {
    puts "WARNING: timing not met - rerun with a lower clock, e.g. -tclargs $board 75 1 $gmm"
}
