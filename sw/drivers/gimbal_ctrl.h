/* -------------------------------------------------------------------------
 * gimbal_ctrl.h - Driver for the IMU gimbal IP
 *
 * Hides the AXI4-Lite register map completely: the application works in
 * degrees, steps per second and controller gains, while the driver handles
 * fixed-point conversion, quaternion algebra and register access.
 *
 * The control loop itself runs in the PL. This driver only parameterizes and
 * observes it.
 * ------------------------------------------------------------------------- */
#ifndef GIMBAL_CTRL_H
#define GIMBAL_CTRL_H

#include <stdint.h>
#include <stddef.h>
#include "gimbal_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Return codes ------------------------------------------------------ */
typedef enum {
    GIMBAL_OK          =  0,
    GIMBAL_ERR_MAGIC   = -1,   /* no matching IP at the base address        */
    GIMBAL_ERR_TIMEOUT = -2,   /* sensor did not come up in time            */
    GIMBAL_ERR_ARG     = -3
} gimbal_status_t;

/* ---- Axis indices ------------------------------------------------------ */
#define GIMBAL_AXIS_ROLL   0   /* X - motor 0 */
#define GIMBAL_AXIS_PITCH  1   /* Y - motor 1 */
#define GIMBAL_AXIS_YAW    2   /* Z - motor 2 */
#define GIMBAL_NAXIS       3

/* ---- Handle ------------------------------------------------------------ */
typedef struct {
    uintptr_t base;
    uint32_t  clk_hz;
    float     home[4];      /* attitude captured at startup, w x y z */
} gimbal_t;

/* ---- Controller snapshot ----------------------------------------------- */
typedef struct {
    float    q[4];          /* measured attitude   w x y z               */
    float    sp[4];         /* setpoint attitude   w x y z               */
    float    err[3];        /* attitude error in radians (x y z)         */
    float    eul[3];        /* measured euler angles: roll, pitch, yaw   */
    float    gyr[3];        /* angular rate in dps                       */
    int32_t  mot_pos[3];    /* step counters                             */
    float    mot_rate[3];   /* current step rate in steps per second     */
    uint32_t status;
    uint32_t sample_cnt;
    uint32_t err_cnt;
    uint8_t  calib;         /* BNO055 CALIB_STAT                         */
    uint8_t  last_err;
} gimbal_state_t;

/* =========================== Lifecycle ================================= */

/* Initialize the handle and verify the IP identification register. */
gimbal_status_t gimbal_init(gimbal_t *g, uintptr_t base);

/* Start the sensor and wait for IMU_OK (pass 0 to return immediately).
 * The hardware automatically adopts the first valid attitude as both the
 * setpoint and the home attitude. */
gimbal_status_t gimbal_start(gimbal_t *g, uint32_t timeout_ms);

/* Stop the sensor state machine; controller and motors are disabled too. */
void gimbal_stop(gimbal_t *g);

/* =========================== Enables =================================== */
void gimbal_enable_control(gimbal_t *g, int on);
void gimbal_enable_motors (gimbal_t *g, int on);
void gimbal_clear_integrators(gimbal_t *g);
void gimbal_zero_positions(gimbal_t *g);
void gimbal_set_dir_invert(gimbal_t *g, uint32_t mask3);
void gimbal_set_d_from_gyro(gimbal_t *g, int on);

/* =========================== Attitude setpoint ========================= */

/* Absolute setpoint as a quaternion (w x y z); normalized before use. */
void gimbal_set_setpoint_quat(gimbal_t *g, const float q[4]);

/* Absolute setpoint from euler angles in degrees (Z-Y-X intrinsic: yaw,
 * pitch, roll - the convention the BNO055 reports). */
void gimbal_set_setpoint_euler(gimbal_t *g,
                               float roll_deg, float pitch_deg, float yaw_deg);

/* Setpoint relative to the attitude captured at startup. */
void gimbal_set_setpoint_rel_euler(gimbal_t *g,
                                   float roll_deg, float pitch_deg, float yaw_deg);

/* Adopt the current measured attitude as the new setpoint. */
void gimbal_capture_setpoint(gimbal_t *g);

/* Return to the attitude captured at startup. */
void gimbal_home(gimbal_t *g);

void gimbal_get_setpoint(gimbal_t *g, float q[4]);

/* =========================== Controller gains ========================== */
void gimbal_set_pid(gimbal_t *g, int axis, float kp, float ki, float kd);
void gimbal_get_pid(gimbal_t *g, int axis, float *kp, float *ki, float *kd);
void gimbal_set_ilimit(gimbal_t *g, int axis, float steps_per_s);

/* Limits expressed in physical units. */
void gimbal_set_limits(gimbal_t *g, float max_rate_sps, float max_acc_sps2);
void gimbal_get_limits(gimbal_t *g, float *max_rate_sps, float *max_acc_sps2);

/* Microstep configuration pins of the driver stage (0..7, e.g. 7 selects
 * 1/16 stepping on an A4988). */
void gimbal_set_microstep(gimbal_t *g, uint32_t ms);

/* EMA filter strength: k = 0 disables, k = 8 is very slow. */
void gimbal_set_filter(gimbal_t *g, uint32_t gyro_k, uint32_t quat_k);

/* Control loop rate in Hz (the sensor provides at most 100 Hz). */
void gimbal_set_rate_hz(gimbal_t *g, uint32_t hz);
uint32_t gimbal_get_rate_hz(gimbal_t *g);

/* SPI clock towards the sensor; the divider is clamped to a minimum of 2. */
void gimbal_set_spi_hz(gimbal_t *g, uint32_t sclk_hz);
uint32_t gimbal_get_spi_hz(gimbal_t *g);

/* =========================== Telemetry ================================= */
void gimbal_get_state(gimbal_t *g, gimbal_state_t *st);
uint32_t gimbal_get_status(gimbal_t *g);
int  gimbal_is_ok(gimbal_t *g);

/* Returns 1 if a new sample arrived since the last call. */
int  gimbal_poll_new_sample(gimbal_t *g);
void gimbal_enable_irq(gimbal_t *g, int on);

/* =========================== Helpers =================================== */

/* Conversion between step rate and DDS phase increment. */
uint32_t gimbal_rate_to_inc(const gimbal_t *g, float steps_per_s);
float    gimbal_inc_to_rate(const gimbal_t *g, int32_t inc);

/* Quaternion algebra (w x y z). */
void gimbal_quat_mul (const float a[4], const float b[4], float out[4]);
void gimbal_quat_conj(const float a[4], float out[4]);
void gimbal_quat_norm(float q[4]);
void gimbal_euler_to_quat(float roll_deg, float pitch_deg, float yaw_deg,
                          float q[4]);

/* Raw register access for debugging and the shell. */
uint32_t gimbal_reg_read (const gimbal_t *g, uint32_t off);
void     gimbal_reg_write(const gimbal_t *g, uint32_t off, uint32_t val);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CTRL_H */
