# ---------------------------------------------------------------------------
# build_hw.tcl - Vivado project, block design, implementation and XSA export
#
#   vivado -mode batch -source hw/scripts/build_hw.tcl
#   vivado -mode batch -source hw/scripts/build_hw.tcl -tclargs <stage> <board>
#
#     stage : all (default) | synth | bd
#     board : k26 (default) | zcu104
#
# The zcu104 target requires the xczu7ev device family to be present in the
# Vivado installation (Vivado installer -> Zynq UltraScale+ ZU7EV).
# ---------------------------------------------------------------------------

set stage "all"
set board "k26"
if {[llength $argv] > 0} { set stage [lindex $argv 0] }
if {[llength $argv] > 1} { set board [lindex $argv 1] }

set script_dir [file normalize [file dirname [info script]]]
set hw_dir     [file normalize $script_dir/..]
set root_dir   [file normalize $hw_dir/..]
set build_dir  [file normalize $root_dir/build/vivado]

set proj_name  "gimbal"
set bd_name    "gimbal_bd"
set top_module "imu_gimbal_axi"

switch -- $board {
    zcu104 {
        set part       "xczu7ev-ffvc1156-2-e"
        set board_part "xilinx.com:zcu104:part0:1.1"
        set xdc_file   "$hw_dir/xdc/zcu104_gimbal.xdc"
    }
    k26 {
        set part       "xck26-sfvc784-2LV-c"
        set board_part "xilinx.com:kr260_som:part0:2.0"
        set xdc_file   "$hw_dir/xdc/kria_k26_gimbal.xdc"
    }
    default { error "unknown board '$board' (supported: k26, zcu104)" }
}

if {[llength [get_parts $part]] == 0} {
    error "part '$part' is not available in this Vivado installation.\
           Installed Zynq MPSoC parts: [get_parts xczu* xck*]"
}

puts "== board=$board  part=$part  stage=$stage"
file mkdir $build_dir

puts "== creating project in $build_dir"
create_project -force $proj_name $build_dir -part $part
catch { set_property board_part $board_part [current_project] }
set_property target_language Verilog [current_project]

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------
set rtl_files [list \
    $hw_dir/rtl/spi_master.sv \
    $hw_dir/rtl/bno055_txn.sv \
    $hw_dir/rtl/bno055_seq.sv \
    $hw_dir/rtl/imu_preproc.sv \
    $hw_dir/rtl/quat_err.sv \
    $hw_dir/rtl/pid_axis.sv \
    $hw_dir/rtl/stepper_drv.sv \
    $hw_dir/rtl/imu_gimbal_axi.sv ]

add_files -norecurse -fileset sources_1 $rtl_files
set_property file_type SystemVerilog [get_files -of_objects [get_filesets sources_1] *.sv]

add_files -fileset constrs_1 -norecurse $xdc_file

add_files -fileset sim_1 -norecurse [list \
    $hw_dir/sim/bno055_spi_model.sv \
    $hw_dir/sim/tb_imu_gimbal.sv ]
set_property top tb_imu_gimbal [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

# ---------------------------------------------------------------------------
# Block design
# ---------------------------------------------------------------------------
puts "== block design $bd_name"
create_bd_design $bd_name

# --- Zynq UltraScale+ processing system ---
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e zynq_ultra_ps_e_0
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
    -config {apply_board_preset "1"} [get_bd_cells zynq_ultra_ps_e_0]

set_property -dict [list \
    CONFIG.PSU__USE__M_AXI_GP0                   {1} \
    CONFIG.PSU__USE__M_AXI_GP1                   {0} \
    CONFIG.PSU__USE__M_AXI_GP2                   {0} \
    CONFIG.PSU__USE__IRQ0                        {1} \
    CONFIG.PSU__FPGA_PL0_ENABLE                  {1} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ   {100} \
] [get_bd_cells zynq_ultra_ps_e_0]

# --- custom RTL as a module reference (no IP packaging required) ---
create_bd_cell -type module -reference $top_module imu_gimbal_axi_0

# --- AXI connection including clock and reset ---
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config [list \
    Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} \
    Master {/zynq_ultra_ps_e_0/M_AXI_HPM0_FPD} \
    Slave  {/imu_gimbal_axi_0/s_axi} \
    intc_ip {New AXI SmartConnect} master_apm {0} \
] [get_bd_intf_pins imu_gimbal_axi_0/s_axi]

# --- interrupt ---
connect_bd_net [get_bd_pins imu_gimbal_axi_0/irq] \
               [get_bd_pins zynq_ultra_ps_e_0/pl_ps_irq0]

# --- external ports, named to match the XDC ---
proc mk_port {dir width name} {
    if {$width <= 1} {
        create_bd_port -dir $dir $name
    } else {
        create_bd_port -dir $dir -from [expr {$width - 1}] -to 0 $name
    }
    connect_bd_net [get_bd_ports $name] [get_bd_pins imu_gimbal_axi_0/$name]
}

mk_port O 1 imu_sclk
mk_port O 1 imu_mosi
mk_port O 1 imu_csn
mk_port O 1 imu_rstn
mk_port I 1 imu_miso
mk_port I 1 imu_int
mk_port O 3 mot_step
mk_port O 3 mot_dir
mk_port O 3 mot_en_n
mk_port O 3 drv_ms
mk_port O 1 drv_rstn
mk_port O 1 drv_slpn
mk_port O 4 led

assign_bd_address
validate_bd_design
save_bd_design

puts "== address map"
foreach seg [get_bd_addr_segs -of_objects [get_bd_cells imu_gimbal_axi_0]] {
    puts "   $seg : [get_property OFFSET $seg] / [get_property RANGE $seg]"
}

# --- HDL wrapper ---
set bd_file [get_files ${bd_name}.bd]
make_wrapper -files $bd_file -top -import
set_property top ${bd_name}_wrapper [get_filesets sources_1]
update_compile_order -fileset sources_1

if {$stage eq "bd"} {
    puts "== block design only, stopping here"
    exit 0
}

# ---------------------------------------------------------------------------
# Synthesis and implementation
# ---------------------------------------------------------------------------
puts "== synthesis"
launch_runs synth_1 -jobs 8
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    error "synthesis failed, see $build_dir/${proj_name}.runs/synth_1"
}
open_run synth_1 -name synth_1
report_utilization -file $build_dir/utilization_synth.rpt

if {$stage eq "synth"} {
    puts "== synthesis only, stopping here"
    exit 0
}

puts "== implementation and bitstream"
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "implementation failed, see $build_dir/${proj_name}.runs/impl_1"
}

open_run impl_1
report_timing_summary -file $build_dir/timing_impl.rpt
report_utilization    -file $build_dir/utilization_impl.rpt

set wns [get_property SLACK [get_timing_paths -delay_type max]]
puts "== WNS = $wns ns"

# ---------------------------------------------------------------------------
# XSA export for Vitis
# ---------------------------------------------------------------------------
set xsa [file normalize $root_dir/build/gimbal.xsa]
file mkdir [file dirname $xsa]
write_hw_platform -fixed -include_bit -force -file $xsa
puts "== XSA written to $xsa"
puts "== done"
