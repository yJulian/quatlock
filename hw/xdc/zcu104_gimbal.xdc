# ---------------------------------------------------------------------------
# zcu104_gimbal.xdc - Pin assignment for the ZCU104
#                     (xczu7ev-ffvc1156-2-e)
#
# The ZCU104 does not expose enough free 3.3 V PL pins, so all application
# signals are routed through the FMC LPC connector (J5, LA00..LA09). That bank
# is supplied from VADJ and is constrained to LVCMOS18 here.
#
#   HARDWARE NOTE
#   The BNO055 (3.3 V) and A4988/DRV8825 drivers require a 1.8 V <-> 3.3 V
#   level shifter on the FMC breakout. VADJ defaults to 1.8 V on the ZCU104
#   (see UG1267); do not connect directly.
#
# The LA pairs are used single-ended as LVCMOS18, which is allowed on HR/HP
# bank pins as long as no clock-capable input is involved.
# ---------------------------------------------------------------------------

# ---- BNO055 SPI -----------------------------------------------------------
set_property -dict {PACKAGE_PIN F17 IOSTANDARD LVCMOS18} [get_ports imu_sclk]  ;# LA00_CC_P
set_property -dict {PACKAGE_PIN F16 IOSTANDARD LVCMOS18} [get_ports imu_mosi]  ;# LA00_CC_N
set_property -dict {PACKAGE_PIN H18 IOSTANDARD LVCMOS18} [get_ports imu_miso]  ;# LA01_CC_P
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS18} [get_ports imu_csn]   ;# LA01_CC_N
set_property -dict {PACKAGE_PIN L20 IOSTANDARD LVCMOS18} [get_ports imu_rstn]  ;# LA02_P
set_property -dict {PACKAGE_PIN K20 IOSTANDARD LVCMOS18} [get_ports imu_int]   ;# LA02_N

# ---- Motor 0 : roll (X) ---------------------------------------------------
set_property -dict {PACKAGE_PIN K19 IOSTANDARD LVCMOS18} [get_ports {mot_step[0]}] ;# LA03_P
set_property -dict {PACKAGE_PIN K18 IOSTANDARD LVCMOS18} [get_ports {mot_dir[0]}]  ;# LA03_N
set_property -dict {PACKAGE_PIN L17 IOSTANDARD LVCMOS18} [get_ports {mot_en_n[0]}] ;# LA04_P

# ---- Motor 1 : pitch (Y) --------------------------------------------------
set_property -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS18} [get_ports {mot_step[1]}] ;# LA04_N
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS18} [get_ports {mot_dir[1]}]  ;# LA05_P
set_property -dict {PACKAGE_PIN J17 IOSTANDARD LVCMOS18} [get_ports {mot_en_n[1]}] ;# LA05_N

# ---- Motor 2 : yaw (Z) ----------------------------------------------------
set_property -dict {PACKAGE_PIN H19 IOSTANDARD LVCMOS18} [get_ports {mot_step[2]}] ;# LA06_P
set_property -dict {PACKAGE_PIN G19 IOSTANDARD LVCMOS18} [get_ports {mot_dir[2]}]  ;# LA06_N
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS18} [get_ports {mot_en_n[2]}] ;# LA07_P

# ---- Shared driver configuration ------------------------------------------
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS18} [get_ports {drv_ms[0]}]   ;# LA07_N
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS18} [get_ports {drv_ms[1]}]   ;# LA08_P
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS18} [get_ports {drv_ms[2]}]   ;# LA08_N
set_property -dict {PACKAGE_PIN H16 IOSTANDARD LVCMOS18} [get_ports drv_rstn]      ;# LA09_P
set_property -dict {PACKAGE_PIN G16 IOSTANDARD LVCMOS18} [get_ports drv_slpn]      ;# LA09_N

# ---- Status LEDs ----------------------------------------------------------
set_property -dict {PACKAGE_PIN D5 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN D6 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN A5 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN B5 IOSTANDARD LVCMOS33} [get_ports {led[3]}]

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
# SCLK/MOSI/CSN run at most at 12.5 MHz and the motor signals below 30 kHz.
# All of them are evaluated asynchronously, so no I/O timing budget is needed.
set_false_path -to   [get_ports {imu_sclk imu_mosi imu_csn imu_rstn}]
set_false_path -to   [get_ports {mot_step[*] mot_dir[*] mot_en_n[*]}]
set_false_path -to   [get_ports {drv_ms[*] drv_rstn drv_slpn led[*]}]
set_false_path -from [get_ports {imu_miso imu_int}]
