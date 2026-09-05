# ---------------------------------------------------------------------------
# run_sim.ps1 - RTL simulation without a Vivado project (xvlog/xelab/xsim)
#
#   powershell -ExecutionPolicy Bypass -File hw\scripts\run_sim.ps1
#   powershell -ExecutionPolicy Bypass -File hw\scripts\run_sim.ps1 -Wave
#
# Builds into build\xsim and runs the self-checking testbench.
# Exit code 0 means all checks passed.
# ---------------------------------------------------------------------------
param(
    [string]$VivadoRoot = "C:\AMDDesignTools\2026.1\Vivado",
    [switch]$Wave
)

$ErrorActionPreference = "Stop"

$hwDir   = Split-Path -Parent (Split-Path -Parent $PSCommandPath)
$rootDir = Split-Path -Parent $hwDir
$simDir  = Join-Path $rootDir "build\xsim"

New-Item -ItemType Directory -Force $simDir | Out-Null
Push-Location $simDir
$env:PATH = "$VivadoRoot\bin;$env:PATH"

$files = @(
    "$hwDir\rtl\spi_master.sv",
    "$hwDir\rtl\bno055_txn.sv",
    "$hwDir\rtl\bno055_seq.sv",
    "$hwDir\rtl\imu_preproc.sv",
    "$hwDir\rtl\quat_err.sv",
    "$hwDir\rtl\pid_axis.sv",
    "$hwDir\rtl\stepper_drv.sv",
    "$hwDir\rtl\imu_gimbal_axi.sv",
    "$hwDir\sim\bno055_spi_model.sv",
    "$hwDir\sim\tb_imu_gimbal.sv"
) -join " "

Write-Host "== xvlog"
cmd /c "xvlog.bat -sv $files 2>&1" | Out-Host
if ($LASTEXITCODE -ne 0) { Pop-Location; exit 1 }

Write-Host "== xelab"
$dbg = if ($Wave) { "-debug typical" } else { "-debug off -O2" }
cmd /c "xelab.bat $dbg -top tb_imu_gimbal -snapshot tbsnap 2>&1" | Out-Host
if ($LASTEXITCODE -ne 0) { Pop-Location; exit 1 }

Write-Host "== xsim"
if ($Wave) {
    # -runall does not log any signals; a batch script is needed to record the
    # full hierarchy into the waveform database before the run starts.
    @'
log_wave -recursive /
run all
exit
'@ | Out-File -Encoding ascii wave.tcl
    $log = cmd /c "xsim tbsnap -tclbatch wave.tcl 2>&1"
} else {
    $log = cmd /c "xsim tbsnap -runall 2>&1"
}
$log | Out-Host

if ($Wave) {
    $wdb = Join-Path $simDir "tbsnap.wdb"
    if (Test-Path $wdb) {
        Write-Host ""
        Write-Host "== waveform: $wdb"
        Write-Host "   open with:  xsim --gui `"$wdb`""
    } else {
        Write-Host "== WARNING: no waveform database produced"
    }
}

Pop-Location

if ($log -match "ALL TESTS PASSED") { exit 0 } else { exit 1 }
