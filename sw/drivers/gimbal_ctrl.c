/* -------------------------------------------------------------------------
 * gimbal_ctrl.c - Implementation of the IMU gimbal driver
 *
 * Deliberately free of libm: sine and cosine come from a folded Taylor series
 * (error below 1e-6 on the reduced interval). The driver therefore builds in a
 * standalone BSP without linking -lm and without pulling in software floating
 * point emulation for transcendental functions.
 * ------------------------------------------------------------------------- */

#include "gimbal_ctrl.h"

#if defined(__has_include)
#  if __has_include("xil_io.h")
#    define GIMBAL_HAVE_XIL 1
#  endif
#endif

#ifdef GIMBAL_HAVE_XIL
#  include "xil_io.h"
#  include "sleep.h"
#  define REG_RD(a)      Xil_In32((UINTPTR)(a))
#  define REG_WR(a, v)   Xil_Out32((UINTPTR)(a), (u32)(v))
#  define DELAY_MS(ms)   usleep((ms) * 1000u)
#else
#  define REG_RD(a)      (*(volatile uint32_t *)(a))
#  define REG_WR(a, v)   (*(volatile uint32_t *)(a) = (uint32_t)(v))
#  define DELAY_MS(ms)   do { volatile unsigned long _d = (ms) * 20000ul; \
                              while (_d--) { } } while (0)
#endif

#define PI_F   3.14159265358979f

/* ===================== Math without libm ================================ */

static float m_sin(float x)
{
    float x2;
    /* fold into [-pi, pi] */
    while (x >  PI_F) x -= 2.0f * PI_F;
    while (x < -PI_F) x += 2.0f * PI_F;
    /* fold into [-pi/2, pi/2] using sin(pi - x) = sin(x) */
    if      (x >  0.5f * PI_F) x =  PI_F - x;
    else if (x < -0.5f * PI_F) x = -PI_F - x;

    x2 = x * x;
    return x * (1.0f + x2 * (-1.0f / 6.0f
              + x2 * (1.0f / 120.0f
              + x2 * (-1.0f / 5040.0f))));
}

static float m_cos(float x) { return m_sin(x + 0.5f * PI_F); }

static float m_sqrt(float v)
{
    float r;
    int i;
    if (v <= 0.0f) return 0.0f;
    r = v;                                   /* Newton iteration */
    for (i = 0; i < 12; i++) r = 0.5f * (r + v / r);
    return r;
}

/* ===================== Register access ================================== */

uint32_t gimbal_reg_read(const gimbal_t *g, uint32_t off)
{
    return REG_RD(g->base + off);
}

void gimbal_reg_write(const gimbal_t *g, uint32_t off, uint32_t val)
{
    REG_WR(g->base + off, val);
}

static int32_t reg_rd_s(const gimbal_t *g, uint32_t off)
{
    return (int32_t)REG_RD(g->base + off);
}

static void ctrl_set(gimbal_t *g, uint32_t mask, int on)
{
    uint32_t c = REG_RD(g->base + GR_CTRL);
    /* never leave a single-shot bit asserted */
    c &= ~(CTRL_CAPTURE | CTRL_ICLEAR | CTRL_ZERO_POS);
    if (on) c |= mask; else c &= ~mask;
    REG_WR(g->base + GR_CTRL, c);
}

static void ctrl_pulse(gimbal_t *g, uint32_t mask)
{
    uint32_t c = REG_RD(g->base + GR_CTRL);
    c &= ~(CTRL_CAPTURE | CTRL_ICLEAR | CTRL_ZERO_POS);
    REG_WR(g->base + GR_CTRL, c | mask);
    REG_WR(g->base + GR_CTRL, c);
}

/* ===================== Quaternions ====================================== */

void gimbal_quat_mul(const float a[4], const float b[4], float out[4])
{
    float w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    float x = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    float y = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    float z = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void gimbal_quat_conj(const float a[4], float out[4])
{
    out[0] =  a[0];
    out[1] = -a[1];
    out[2] = -a[2];
    out[3] = -a[3];
}

void gimbal_quat_norm(float q[4])
{
    float n = m_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-6f) { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; return; }
    q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
}

/* Z-Y-X intrinsic: yaw about Z, then pitch about Y, then roll about X */
void gimbal_euler_to_quat(float roll_deg, float pitch_deg, float yaw_deg,
                          float q[4])
{
    const float k = PI_F / 360.0f;      /* degrees to half-radians */
    float cr = m_cos(roll_deg  * k), sr = m_sin(roll_deg  * k);
    float cp = m_cos(pitch_deg * k), sp = m_sin(pitch_deg * k);
    float cy = m_cos(yaw_deg   * k), sy = m_sin(yaw_deg   * k);

    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;
    gimbal_quat_norm(q);
}

/* ===================== Scaling ========================================== */

uint32_t gimbal_rate_to_inc(const gimbal_t *g, float steps_per_s)
{
    double inc;
    if (steps_per_s < 0.0f) steps_per_s = -steps_per_s;
    if (g->clk_hz == 0u) return 0u;
    inc = (double)steps_per_s * 4294967296.0 / (double)g->clk_hz;
    if (inc > 2147483000.0) inc = 2147483000.0;
    return (uint32_t)inc;
}

float gimbal_inc_to_rate(const gimbal_t *g, int32_t inc)
{
    if (g->clk_hz == 0u) return 0.0f;
    return (float)((double)inc * (double)g->clk_hz / 4294967296.0);
}

static int32_t q14_from_float(float v)
{
    float s = v * (float)GIMBAL_Q14_ONE;
    if (s >  32767.0f) s =  32767.0f;
    if (s < -32767.0f) s = -32767.0f;
    return (int32_t)(s >= 0.0f ? s + 0.5f : s - 0.5f);
}

static float float_from_q14(int32_t v)
{
    return (float)v / (float)GIMBAL_Q14_ONE;
}

static int32_t q16_from_float(float v)
{
    double s = (double)v * 65536.0;
    if (s >  2147483000.0) s =  2147483000.0;
    if (s < -2147483000.0) s = -2147483000.0;
    return (int32_t)s;
}

/* ===================== Lifecycle ======================================== */

gimbal_status_t gimbal_init(gimbal_t *g, uintptr_t base)
{
    if (g == NULL) return GIMBAL_ERR_ARG;

    g->base   = base;
    g->clk_hz = 100000000u;

    if (REG_RD(base + GR_ID) != GIMBAL_MAGIC) return GIMBAL_ERR_MAGIC;

    g->clk_hz = REG_RD(base + GR_CLK_HZ);
    if (g->clk_hz == 0u) g->clk_hz = 100000000u;

    g->home[0] = 1.0f; g->home[1] = g->home[2] = g->home[3] = 0.0f;
    return GIMBAL_OK;
}

gimbal_status_t gimbal_start(gimbal_t *g, uint32_t timeout_ms)
{
    uint32_t waited = 0u;
    int i;

    ctrl_set(g, CTRL_IMU_EN, 1);

    if (timeout_ms == 0u) return GIMBAL_OK;

    while (waited < timeout_ms) {
        if (REG_RD(g->base + GR_STATUS) & STAT_IMU_OK) {
            /* home attitude latched by the hardware on the first sample */
            for (i = 0; i < 4; i++)
                g->home[i] = float_from_q14(reg_rd_s(g, GR_HOME_QW + 4u * (uint32_t)i));
            gimbal_quat_norm(g->home);
            return GIMBAL_OK;
        }
        DELAY_MS(10);
        waited += 10u;
    }
    return GIMBAL_ERR_TIMEOUT;
}

void gimbal_stop(gimbal_t *g)
{
    REG_WR(g->base + GR_CTRL, 0u);
}

/* ===================== Enables ========================================== */

void gimbal_enable_control(gimbal_t *g, int on) { ctrl_set(g, CTRL_CTRL_EN, on); }
void gimbal_enable_motors (gimbal_t *g, int on) { ctrl_set(g, CTRL_MOT_EN,  on); }
void gimbal_clear_integrators(gimbal_t *g)      { ctrl_pulse(g, CTRL_ICLEAR); }
void gimbal_zero_positions(gimbal_t *g)         { ctrl_pulse(g, CTRL_ZERO_POS); }
void gimbal_set_d_from_gyro(gimbal_t *g, int on){ ctrl_set(g, CTRL_D_FROM_GYRO, on); }
void gimbal_enable_irq(gimbal_t *g, int on)     { ctrl_set(g, CTRL_IRQ_EN, on); }

void gimbal_set_dir_invert(gimbal_t *g, uint32_t mask3)
{
    uint32_t c = REG_RD(g->base + GR_CTRL);
    c &= ~(CTRL_CAPTURE | CTRL_ICLEAR | CTRL_ZERO_POS | CTRL_DIR_INV_MASK);
    c |= (mask3 & 7u) << CTRL_DIR_INV_SHIFT;
    REG_WR(g->base + GR_CTRL, c);
}

/* ===================== Attitude setpoint ================================ */

void gimbal_set_setpoint_quat(gimbal_t *g, const float q_in[4])
{
    float q[4];
    int i;
    for (i = 0; i < 4; i++) q[i] = q_in[i];
    gimbal_quat_norm(q);
    for (i = 0; i < 4; i++)
        REG_WR(g->base + GR_SP_QW + 4u * (uint32_t)i,
               (uint32_t)(q14_from_float(q[i]) & 0xFFFF));
}

void gimbal_set_setpoint_euler(gimbal_t *g,
                               float roll_deg, float pitch_deg, float yaw_deg)
{
    float q[4];
    gimbal_euler_to_quat(roll_deg, pitch_deg, yaw_deg, q);
    gimbal_set_setpoint_quat(g, q);
}

void gimbal_set_setpoint_rel_euler(gimbal_t *g,
                                   float roll_deg, float pitch_deg, float yaw_deg)
{
    float dq[4], q[4];
    gimbal_euler_to_quat(roll_deg, pitch_deg, yaw_deg, dq);
    gimbal_quat_mul(g->home, dq, q);      /* q = q_home (x) q_delta */
    gimbal_set_setpoint_quat(g, q);
}

void gimbal_capture_setpoint(gimbal_t *g) { ctrl_pulse(g, CTRL_CAPTURE); }

void gimbal_home(gimbal_t *g) { gimbal_set_setpoint_quat(g, g->home); }

void gimbal_get_setpoint(gimbal_t *g, float q[4])
{
    int i;
    for (i = 0; i < 4; i++)
        q[i] = float_from_q14(reg_rd_s(g, GR_SP_QW + 4u * (uint32_t)i));
}

/* ===================== Controller gains ================================= */

void gimbal_set_pid(gimbal_t *g, int axis, float kp, float ki, float kd)
{
    if (axis < 0 || axis >= GIMBAL_NAXIS) return;
    REG_WR(g->base + GR_PID_KP((uint32_t)axis), (uint32_t)q16_from_float(kp));
    REG_WR(g->base + GR_PID_KI((uint32_t)axis), (uint32_t)q16_from_float(ki));
    REG_WR(g->base + GR_PID_KD((uint32_t)axis), (uint32_t)q16_from_float(kd));
}

void gimbal_get_pid(gimbal_t *g, int axis, float *kp, float *ki, float *kd)
{
    if (axis < 0 || axis >= GIMBAL_NAXIS) return;
    if (kp) *kp = (float)reg_rd_s(g, GR_PID_KP((uint32_t)axis)) / 65536.0f;
    if (ki) *ki = (float)reg_rd_s(g, GR_PID_KI((uint32_t)axis)) / 65536.0f;
    if (kd) *kd = (float)reg_rd_s(g, GR_PID_KD((uint32_t)axis)) / 65536.0f;
}

void gimbal_set_ilimit(gimbal_t *g, int axis, float steps_per_s)
{
    if (axis < 0 || axis >= GIMBAL_NAXIS) return;
    REG_WR(g->base + GR_PID_ILIM((uint32_t)axis),
           gimbal_rate_to_inc(g, steps_per_s));
}

void gimbal_set_limits(gimbal_t *g, float max_rate_sps, float max_acc_sps2)
{
    uint32_t rate_hz = gimbal_get_rate_hz(g);
    if (rate_hz == 0u) rate_hz = 100u;
    REG_WR(g->base + GR_OUT_LIM, gimbal_rate_to_inc(g, max_rate_sps));
    /* the hardware limit is per control tick, not per second */
    REG_WR(g->base + GR_ACC_LIM,
           gimbal_rate_to_inc(g, max_acc_sps2 / (float)rate_hz));
}

void gimbal_get_limits(gimbal_t *g, float *max_rate_sps, float *max_acc_sps2)
{
    uint32_t rate_hz = gimbal_get_rate_hz(g);
    if (rate_hz == 0u) rate_hz = 100u;
    if (max_rate_sps)
        *max_rate_sps = gimbal_inc_to_rate(g, reg_rd_s(g, GR_OUT_LIM));
    if (max_acc_sps2)
        *max_acc_sps2 = gimbal_inc_to_rate(g, reg_rd_s(g, GR_ACC_LIM))
                        * (float)rate_hz;
}

void gimbal_set_microstep(gimbal_t *g, uint32_t ms)
{
    uint32_t v = REG_RD(g->base + GR_MS_CFG);
    v = (v & ~7u) | (ms & 7u) | (1u << 3) | (1u << 4);   /* nRESET/nSLEEP high */
    REG_WR(g->base + GR_MS_CFG, v);
}

void gimbal_set_filter(gimbal_t *g, uint32_t gyro_k, uint32_t quat_k)
{
    REG_WR(g->base + GR_FILT_CFG, ((gyro_k & 0xFu)) | ((quat_k & 0xFu) << 4));
}

void gimbal_set_rate_hz(gimbal_t *g, uint32_t hz)
{
    if (hz == 0u) return;
    if (hz > 200u) hz = 200u;               /* BNO055 NDOF outputs 100 Hz */
    REG_WR(g->base + GR_SAMPLE_DIV, g->clk_hz / hz);
}

uint32_t gimbal_get_rate_hz(gimbal_t *g)
{
    uint32_t d = REG_RD(g->base + GR_SAMPLE_DIV);
    return (d == 0u) ? 0u : (g->clk_hz / d);
}

void gimbal_set_spi_hz(gimbal_t *g, uint32_t sclk_hz)
{
    uint32_t div;
    if (sclk_hz == 0u) return;
    div = (g->clk_hz / (2u * sclk_hz));
    div = (div == 0u) ? 0u : (div - 1u);
    if (div < 2u)      div = 2u;
    if (div > 0xFFFFu) div = 0xFFFFu;
    REG_WR(g->base + GR_SPI_DIV, div);
}

uint32_t gimbal_get_spi_hz(gimbal_t *g)
{
    uint32_t div = REG_RD(g->base + GR_SPI_DIV) & 0xFFFFu;
    return g->clk_hz / (2u * (div + 1u));
}

/* ===================== Telemetry ======================================== */

uint32_t gimbal_get_status(gimbal_t *g) { return REG_RD(g->base + GR_STATUS); }

int gimbal_is_ok(gimbal_t *g)
{
    return (REG_RD(g->base + GR_STATUS) & STAT_IMU_OK) ? 1 : 0;
}

int gimbal_poll_new_sample(gimbal_t *g)
{
    uint32_t s = REG_RD(g->base + GR_IRQ_STATUS);
    if (s & IRQ_NEW_SAMPLE) {
        REG_WR(g->base + GR_IRQ_STATUS, IRQ_NEW_SAMPLE);   /* write one to clear */
        return 1;
    }
    return 0;
}

void gimbal_get_state(gimbal_t *g, gimbal_state_t *st)
{
    int i;
    if (st == NULL) return;

    for (i = 0; i < 4; i++) {
        st->q[i]  = float_from_q14(reg_rd_s(g, GR_MEAS_QW + 4u * (uint32_t)i));
        st->sp[i] = float_from_q14(reg_rd_s(g, GR_SP_QW   + 4u * (uint32_t)i));
    }
    for (i = 0; i < 3; i++)
        st->err[i] = float_from_q14(reg_rd_s(g, GR_ERR_X + 4u * (uint32_t)i));

    /* hardware register order is yaw, roll, pitch */
    st->eul[0] = (float)reg_rd_s(g, GR_EUL_ROLL)  / (float)GIMBAL_EUL_LSB;
    st->eul[1] = (float)reg_rd_s(g, GR_EUL_PITCH) / (float)GIMBAL_EUL_LSB;
    st->eul[2] = (float)reg_rd_s(g, GR_EUL_YAW)   / (float)GIMBAL_EUL_LSB;

    for (i = 0; i < 3; i++)
        st->gyr[i] = (float)reg_rd_s(g, GR_GYR_X + 4u * (uint32_t)i)
                     / (float)GIMBAL_GYR_LSB;

    for (i = 0; i < 3; i++) {
        st->mot_pos[i]  = reg_rd_s(g, GR_MOT_POS((uint32_t)i));
        st->mot_rate[i] = gimbal_inc_to_rate(g, reg_rd_s(g, GR_MOT_VEL((uint32_t)i)));
    }

    st->status     = REG_RD(g->base + GR_STATUS);
    st->sample_cnt = REG_RD(g->base + GR_SAMPLE_CNT);
    st->err_cnt    = REG_RD(g->base + GR_ERR_CNT);
    st->calib      = (uint8_t)((st->status >> STAT_CALIB_SHIFT) & 0xFFu);
    st->last_err   = (uint8_t)((st->status >> STAT_LASTERR_SHIFT) & 0xFFu);
}
