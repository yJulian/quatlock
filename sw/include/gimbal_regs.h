/* -------------------------------------------------------------------------
 * gimbal_regs.h - Register map of the AXI4-Lite IP "imu_gimbal_axi"
 *
 * This header is the single source of truth for offsets and bit positions.
 * It mirrors hw/rtl/imu_gimbal_axi.sv.
 *
 * Fixed-point formats:
 *   quaternion / attitude error : Q1.14   (16384 = 1.0)
 *   euler angles                : 1/16 degree
 *   angular rate                : 1/16 dps
 *   PID gains                   : Q16.16  (65536 = 1.0)
 *   motor command and limits    : DDS phase increment,
 *                                 f_step = f_clk * inc / 2^32
 * ------------------------------------------------------------------------- */
#ifndef GIMBAL_REGS_H
#define GIMBAL_REGS_H

/* ---- Identification and general ---------------------------------------- */
#define GR_ID              0x000u  /* RO  0x424E4F35 = "BNO5"               */
#define GR_VERSION         0x004u  /* RO  0x00010000 = v1.0                 */
#define GR_CLK_HZ          0x008u  /* RO  PL clock in Hz                    */
#define GR_SCRATCH         0x00Cu  /* RW  unused, useful for a bus self-test*/

#define GIMBAL_MAGIC       0x424E4F35u

/* ---- Control and status ------------------------------------------------ */
#define GR_CTRL            0x010u  /* RW                                    */
#define GR_STATUS          0x014u  /* RO                                    */
#define GR_SAMPLE_CNT      0x018u  /* RO  valid sensor samples              */
#define GR_ERR_CNT         0x01Cu  /* RO  SPI protocol errors               */
#define GR_IRQ_STATUS      0x020u  /* RW1C                                  */

#define CTRL_IMU_EN        (1u << 0)  /* start the sensor state machine     */
#define CTRL_CTRL_EN       (1u << 1)  /* enable the PID controllers         */
#define CTRL_MOT_EN        (1u << 2)  /* enable the driver stages           */
#define CTRL_CAPTURE       (1u << 3)  /* pulse: latch measured as setpoint  */
#define CTRL_ICLEAR        (1u << 4)  /* pulse: clear the integrators       */
#define CTRL_D_FROM_GYRO   (1u << 5)  /* derivative on gyro instead of de/dt*/
#define CTRL_IRQ_EN        (1u << 6)  /* enable the interrupt output        */
#define CTRL_ZERO_POS      (1u << 7)  /* pulse: reset the step counters     */
#define CTRL_DIR_INV_SHIFT 8u
#define CTRL_DIR_INV_MASK  (7u << 8)  /* invert direction, one bit per motor*/

#define STAT_IMU_OK        (1u << 0)
#define STAT_INIT_DONE     (1u << 1)
#define STAT_BUS_ERR       (1u << 2)
#define STAT_IMU_INT       (1u << 3)  /* level on the BNO055 INT pin        */
#define STAT_LASTERR_SHIFT 8u         /* [15:8]  last error code            */
#define STAT_CALIB_SHIFT   16u        /* [23:16] BNO055 CALIB_STAT          */
#define STAT_FSM_SHIFT     24u        /* [31:24] state machine, debug only  */

#define IRQ_NEW_SAMPLE     (1u << 0)

/* Error codes reported in the LASTERR field */
#define GERR_TIMEOUT       0xF0u      /* no response on the SPI bus         */
#define GERR_CHIPID        0xF1u      /* CHIP_ID did not read 0xA0          */

/* CALIB_STAT: two bits per field, 3 means fully calibrated */
#define CALIB_SYS(c)       (((c) >> 6) & 3u)
#define CALIB_GYR(c)       (((c) >> 4) & 3u)
#define CALIB_ACC(c)       (((c) >> 2) & 3u)
#define CALIB_MAG(c)       (((c) >> 0) & 3u)

/* ---- Attitude setpoint (Q1.14, w x y z) -------------------------------- */
#define GR_SP_QW           0x030u
#define GR_SP_QX           0x034u
#define GR_SP_QY           0x038u
#define GR_SP_QZ           0x03Cu

/* ---- Measured attitude (Q1.14) ----------------------------------------- */
#define GR_MEAS_QW         0x040u
#define GR_MEAS_QX         0x044u
#define GR_MEAS_QY         0x048u
#define GR_MEAS_QZ         0x04Cu

/* ---- Attitude error (Q1.14, rotation vector in body frame) ------------- */
#define GR_ERR_X           0x050u
#define GR_ERR_Y           0x054u
#define GR_ERR_Z           0x058u
#define GR_ERR_QW          0x05Cu    /* scalar part of q_err               */

/* ---- Euler angles and angular rate ------------------------------------- */
#define GR_EUL_YAW         0x060u    /* 1/16 degree */
#define GR_EUL_ROLL        0x064u
#define GR_EUL_PITCH       0x068u
#define GR_GYR_X           0x06Cu    /* 1/16 dps    */
#define GR_GYR_Y           0x070u
#define GR_GYR_Z           0x074u

/* ---- Controller gains, four registers per axis ------------------------- */
#define GR_PID_BASE        0x080u
#define GR_PID_STRIDE      0x010u
#define GR_PID_KP(ax)      (GR_PID_BASE + (ax) * GR_PID_STRIDE + 0x0u)
#define GR_PID_KI(ax)      (GR_PID_BASE + (ax) * GR_PID_STRIDE + 0x4u)
#define GR_PID_KD(ax)      (GR_PID_BASE + (ax) * GR_PID_STRIDE + 0x8u)
#define GR_PID_ILIM(ax)    (GR_PID_BASE + (ax) * GR_PID_STRIDE + 0xCu)

/* ---- Motor stage ------------------------------------------------------- */
#define GR_OUT_LIM         0x0B0u    /* max |phase increment|              */
#define GR_ACC_LIM         0x0B4u    /* max change per control tick        */
#define GR_STEP_WIDTH      0x0B8u    /* STEP pulse width in PL clocks      */
#define GR_MS_CFG          0x0BCu    /* [2:0] MS1..3, [3] nRESET, [4] nSLP */

#define GR_MOT_POS(m)      (0x0C0u + (m) * 4u)   /* RO, signed step count  */
#define GR_MOT_VEL(m)      (0x0CCu + (m) * 4u)   /* RO, signed increment   */

/* ---- Timing and filtering ---------------------------------------------- */
#define GR_SAMPLE_DIV      0x0E0u    /* control period in PL clocks        */
#define GR_SPI_DIV         0x0E4u    /* f_sclk = f_clk/(2*(div+1)), div>=2  */
#define GR_INIT_DLY        0x0E8u    /* power-on reset delay in PL clocks   */
#define GR_POLL_MAX        0x0ECu    /* max dummy bytes awaiting a response */
#define GR_FILT_CFG        0x0F0u    /* [3:0] gyro EMA k, [7:4] quat EMA k  */
#define GR_LED_CFG         0x0F4u    /* [3:0] pattern, [4] override enable  */

/* ---- Attitude captured at startup (Q1.14) ------------------------------ */
#define GR_HOME_QW         0x100u
#define GR_HOME_QX         0x104u
#define GR_HOME_QY         0x108u
#define GR_HOME_QZ         0x10Cu

/* ---- Scaling constants ------------------------------------------------- */
#define GIMBAL_Q14_ONE     16384      /* 1.0 in Q1.14                      */
#define GIMBAL_Q16_ONE     65536L     /* 1.0 in Q16.16                     */
#define GIMBAL_EUL_LSB     16         /* LSBs per degree                   */
#define GIMBAL_GYR_LSB     16         /* LSBs per dps                      */

#endif /* GIMBAL_REGS_H */
