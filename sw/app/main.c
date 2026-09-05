/* -------------------------------------------------------------------------
 * main.c - Bare-metal application for the IMU gimbal (Cortex-A53, standalone)
 *
 * The control loop runs entirely in the PL. This application
 *   - initializes the driver and verifies the IP identification,
 *   - applies sensible startup parameters (limits, filters, gains),
 *   - starts the sensor, which adopts the startup attitude as the setpoint,
 *   - hands over to the command line.
 * ------------------------------------------------------------------------- */

#include <stdint.h>

#include "gimbal_ctrl.h"
#include "shell.h"

#if defined(__has_include)
#  if __has_include("xparameters.h")
#    define APP_HAVE_XIL 1
#  endif
#endif

#ifdef APP_HAVE_XIL
#  include "xparameters.h"
#  include "xil_printf.h"
#  include "xil_cache.h"
#  define PR xil_printf
#else
#  include <stdio.h>
#  define PR printf
#endif

/* Base address of the IP. Vivado places the module reference at 0xA000_0000
 * on HPM0 FPD by default; the BSP definition takes precedence when present. */
#ifndef GIMBAL_BASEADDR
#  if defined(XPAR_IMU_GIMBAL_AXI_0_BASEADDR)
#    define GIMBAL_BASEADDR  XPAR_IMU_GIMBAL_AXI_0_BASEADDR
#  elif defined(XPAR_IMU_GIMBAL_AXI_0_S_AXI_BASEADDR)
#    define GIMBAL_BASEADDR  XPAR_IMU_GIMBAL_AXI_0_S_AXI_BASEADDR
#  else
#    define GIMBAL_BASEADDR  0xA0000000UL
#  endif
#endif

static gimbal_t g;

static void banner(void)
{
    PR("\r\n");
    PR("=========================================================\r\n");
    PR(" BNO055 attitude control - 3-axis gimbal\r\n");
    PR(" Control loop in the PL, operated from this console\r\n");
    PR("=========================================================\r\n");
}

int main(void)
{
#ifdef APP_HAVE_XIL
    Xil_DCacheEnable();
    Xil_ICacheEnable();
#endif

    banner();

    if (gimbal_init(&g, (uintptr_t)GIMBAL_BASEADDR) != GIMBAL_OK) {
        PR("ERROR: no IMU gimbal IP found at 0x%08x (ID=0x%08x).\r\n",
           (unsigned)GIMBAL_BASEADDR,
           (unsigned)gimbal_reg_read(&g, GR_ID));
        PR("Check the block design address map or GIMBAL_BASEADDR.\r\n");
        for (;;) { }
    }

    PR("IP found at 0x%08x, PL clock %u Hz, version 0x%08x\r\n",
       (unsigned)GIMBAL_BASEADDR, (unsigned)g.clk_hz,
       (unsigned)gimbal_reg_read(&g, GR_VERSION));

    /* ---- Startup parameters ----------------------------------------- */
    gimbal_set_spi_hz(&g, 1000000u);      /* 1 MHz SPI to the BNO055     */
    gimbal_set_rate_hz(&g, 100u);         /* NDOF fusion outputs 100 Hz  */
    gimbal_set_filter(&g, 3u, 0u);        /* smooth gyro, quaternion raw */
    gimbal_set_microstep(&g, 7u);         /* A4988: 1/16 stepping        */
    gimbal_set_limits(&g, 4000.0f, 40000.0f);   /* steps/s, steps/s^2    */

    /* Conservative start: proportional only, derivative from the gyro. */
    gimbal_set_d_from_gyro(&g, 1);
    for (int ax = 0; ax < GIMBAL_NAXIS; ax++) {
        gimbal_set_pid(&g, ax, 150.0f, 0.0f, 8.0f);
        gimbal_set_ilimit(&g, ax, 500.0f);
    }

    /* ---- Start the sensor -------------------------------------------- */
    PR("Initializing the BNO055 ...\r\n");
    if (gimbal_start(&g, 3000u) == GIMBAL_OK) {
        PR("Sensor ready. The startup attitude is the default setpoint.\r\n");
        PR("Use 'ctrl 1' and 'motors 1' to close the loop,\r\n");
        PR("and 'rpy <roll> <pitch> <yaw>' to command a new attitude.\r\n");
    } else {
        PR("WARNING: sensor did not respond (STATUS=0x%08x).\r\n",
           (unsigned)gimbal_get_status(&g));
        PR("The console still works ('status', 'reg', 'start').\r\n");
    }

    shell_run(&g);
    return 0;
}
