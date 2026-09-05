# ---------------------------------------------------------------------------
# kria_k26_gimbal.xdc - Pin assignment for the Kria K26 SOM
#                       (xck26-sfvc784-2LV-c)
#
# All application signals sit on SOM240_1 / IO bank 66, an HP bank running at
# 1.8 V on the K26 SOM. On a KR260 carrier these pins appear on the PMOD and
# Raspberry Pi headers; on a KV260 carrier they reach the SOM240 pass-through.
#
#   HARDWARE NOTE
#   The BNO055 (3.3 V) and A4988/DRV8825 drivers require a 1.8 V <-> 3.3 V
#   level shifter. Do not connect bank 66 directly.
#
#   som240_1_* connector pin  ->  package pin
# ---------------------------------------------------------------------------

# ---- BNO055 SPI -----------------------------------------------------------
set_property -dict {PACKAGE_PIN A2 IOSTANDARD LVCMOS18} [get_ports imu_sclk]  ;# som240_1_a3
set_property -dict {PACKAGE_PIN A1 IOSTANDARD LVCMOS18} [get_ports imu_mosi]  ;# som240_1_a4
set_property -dict {PACKAGE_PIN C3 IOSTANDARD LVCMOS18} [get_ports imu_miso]  ;# som240_1_a6
set_property -dict {PACKAGE_PIN C2 IOSTANDARD LVCMOS18} [get_ports imu_csn]   ;# som240_1_a7
set_property -dict {PACKAGE_PIN G6 IOSTANDARD LVCMOS18} [get_ports imu_rstn]  ;# som240_1_a9
set_property -dict {PACKAGE_PIN F6 IOSTANDARD LVCMOS18} [get_ports imu_int]   ;# som240_1_a10

# ---- Motor 0 : roll (X) ---------------------------------------------------
set_property -dict {PACKAGE_PIN G8 IOSTANDARD LVCMOS18} [get_ports {mot_step[0]}] ;# som240_1_a12
set_property -dict {PACKAGE_PIN F7 IOSTANDARD LVCMOS18} [get_ports {mot_dir[0]}]  ;# som240_1_a13
set_property -dict {PACKAGE_PIN C1 IOSTANDARD LVCMOS18} [get_ports {mot_en_n[0]}] ;# som240_1_b1

# ---- Motor 1 : pitch (Y) --------------------------------------------------
set_property -dict {PACKAGE_PIN B1 IOSTANDARD LVCMOS18} [get_ports {mot_step[1]}] ;# som240_1_b2
set_property -dict {PACKAGE_PIN E4 IOSTANDARD LVCMOS18} [get_ports {mot_dir[1]}]  ;# som240_1_b4
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS18} [get_ports {mot_en_n[1]}] ;# som240_1_b5

# ---- Motor 2 : yaw (Z) ----------------------------------------------------
set_property -dict {PACKAGE_PIN B3 IOSTANDARD LVCMOS18} [get_ports {mot_step[2]}] ;# som240_1_b7
set_property -dict {PACKAGE_PIN A3 IOSTANDARD LVCMOS18} [get_ports {mot_dir[2]}]  ;# som240_1_b8
set_property -dict {PACKAGE_PIN E5 IOSTANDARD LVCMOS18} [get_ports {mot_en_n[2]}] ;# som240_1_b10

# ---- Shared driver configuration ------------------------------------------
set_property -dict {PACKAGE_PIN D5 IOSTANDARD LVCMOS18} [get_ports {drv_ms[0]}] ;# som240_1_b11
set_property -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS18} [get_ports {drv_ms[1]}] ;# som240_1_c3
set_property -dict {PACKAGE_PIN F1 IOSTANDARD LVCMOS18} [get_ports {drv_ms[2]}] ;# som240_1_c4
set_property -dict {PACKAGE_PIN G3 IOSTANDARD LVCMOS18} [get_ports drv_rstn]    ;# som240_1_c6
set_property -dict {PACKAGE_PIN F3 IOSTANDARD LVCMOS18} [get_ports drv_slpn]    ;# som240_1_c7

# ---- Status outputs -------------------------------------------------------
set_property -dict {PACKAGE_PIN B4 IOSTANDARD LVCMOS18} [get_ports {led[0]}]    ;# som240_1_c9
set_property -dict {PACKAGE_PIN A4 IOSTANDARD LVCMOS18} [get_ports {led[1]}]    ;# som240_1_c10
set_property -dict {PACKAGE_PIN D7 IOSTANDARD LVCMOS18} [get_ports {led[2]}]    ;# som240_1_c12
set_property -dict {PACKAGE_PIN D6 IOSTANDARD LVCMOS18} [get_ports {led[3]}]    ;# som240_1_c13

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
# SCLK/MOSI/CSN run at most at 12.5 MHz and the motor signals below 30 kHz.
# All of them are evaluated asynchronously, so no I/O timing budget is needed.
set_false_path -to   [get_ports {imu_sclk imu_mosi imu_csn imu_rstn}]
set_false_path -to   [get_ports {mot_step[*] mot_dir[*] mot_en_n[*]}]
set_false_path -to   [get_ports {drv_ms[*] drv_rstn drv_slpn led[*]}]
set_false_path -from [get_ports {imu_miso imu_int}]
