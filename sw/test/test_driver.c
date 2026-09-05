/* -------------------------------------------------------------------------
 * test_driver.c - Host test for gimbal_ctrl.c and the shell parser
 *
 * The driver reaches the register map through volatile pointers, so pointing
 * the base address at a plain array is enough to exercise fixed-point
 * conversion, quaternion algebra and command parsing without hardware.
 *
 *   gcc -std=c99 -I sw/include -I sw/drivers -I sw/app \
 *       sw/drivers/gimbal_ctrl.c sw/app/shell.c sw/test/test_driver.c -lm -o t
 * ------------------------------------------------------------------------- */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "gimbal_ctrl.h"
#include "shell.h"

static uint32_t regs[1024];
static gimbal_t g;
static int      fails;

#define REG(off) regs[(off) / 4u]

static void ok(const char *name, int cond)
{
    printf("  [%s] %s\n", cond ? " OK " : "FAIL", name);
    if (!cond) fails++;
}

static void near(const char *name, double got, double exp, double tol)
{
    int good = fabs(got - exp) <= tol;
    printf("  [%s] %s (got=%.5f exp=%.5f)\n", good ? " OK " : "FAIL",
           name, got, exp);
    if (!good) fails++;
}

static int16_t sp_reg(int i)
{
    return (int16_t)(uint16_t)REG(GR_SP_QW + 4u * (unsigned)i);
}

int main(void)
{
    float q[4], r[4], c[4];
    char  line[128];

    memset(regs, 0, sizeof(regs));
    REG(GR_ID)      = GIMBAL_MAGIC;
    REG(GR_VERSION) = 0x00010000u;
    REG(GR_CLK_HZ)  = 100000000u;

    printf("=========================================================\n");
    printf(" test_driver - gimbal_ctrl and shell\n");
    printf("=========================================================\n");

    /* ---- 1. Initialization ------------------------------------------- */
    printf("\n[1] Initialization\n");
    ok("gimbal_init accepts a valid identification",
       gimbal_init(&g, (uintptr_t)regs) == GIMBAL_OK);
    ok("PL clock adopted from the register", g.clk_hz == 100000000u);

    REG(GR_ID) = 0xDEADBEEFu;
    {
        gimbal_t bad;
        ok("wrong identification is rejected",
           gimbal_init(&bad, (uintptr_t)regs) == GIMBAL_ERR_MAGIC);
    }
    REG(GR_ID) = GIMBAL_MAGIC;

    /* ---- 2. Quaternion algebra ---------------------------------------- */
    printf("\n[2] Quaternion algebra\n");
    gimbal_euler_to_quat(0.0f, 0.0f, 0.0f, q);
    near("identity w", q[0], 1.0, 1e-4);
    near("identity x", q[1], 0.0, 1e-4);

    gimbal_euler_to_quat(90.0f, 0.0f, 0.0f, q);   /* roll +90 about X */
    near("roll 90: w", q[0], 0.70711, 1e-4);
    near("roll 90: x", q[1], 0.70711, 1e-4);
    near("roll 90: y", q[2], 0.0,     1e-4);
    near("roll 90: z", q[3], 0.0,     1e-4);

    gimbal_euler_to_quat(0.0f, 0.0f, 90.0f, q);   /* yaw +90 about Z */
    near("yaw 90: w", q[0], 0.70711, 1e-4);
    near("yaw 90: z", q[3], 0.70711, 1e-4);

    gimbal_euler_to_quat(30.0f, -20.0f, 75.0f, q);
    gimbal_quat_conj(q, c);
    gimbal_quat_mul(q, c, r);
    near("q (x) conj(q) = 1 (w)", r[0], 1.0, 1e-4);
    near("q (x) conj(q) = 1 (x)", r[1], 0.0, 1e-4);
    near("q (x) conj(q) = 1 (y)", r[2], 0.0, 1e-4);
    near("q (x) conj(q) = 1 (z)", r[3], 0.0, 1e-4);
    near("norm stays at unity",
         sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]), 1.0, 1e-4);

    /* ---- 3. Setpoint registers in Q1.14 -------------------------------- */
    printf("\n[3] Setpoint registers (Q1.14)\n");
    gimbal_set_setpoint_euler(&g, 0.0f, 0.0f, 0.0f);
    ok("identity maps to 16384/0/0/0",
       sp_reg(0) == 16384 && sp_reg(1) == 0 &&
       sp_reg(2) == 0     && sp_reg(3) == 0);

    gimbal_euler_to_quat(30.0f, 0.0f, 0.0f, q);   /* 30 degrees about X */
    gimbal_set_setpoint_quat(&g, q);
    near("30 deg about X: SP_QW", sp_reg(0), 15826, 2);
    near("30 deg about X: SP_QX", sp_reg(1),  4240, 2);

    /* Cross-check against the hardware convention:
     * e = -2*sign(w)*vec(conj(q_sp) (x) q_meas). With q_sp = identity and
     * q_meas rotated 30 degrees about X this is -2*sin(15 degrees). */
    near("expected hardware error value in Q1.14",
         -2.0 * (double)sp_reg(1), -8480.0, 4.0);

    /* ---- 4. Home attitude and relative setpoints ----------------------- */
    printf("\n[4] Relative setpoints\n");
    gimbal_euler_to_quat(0.0f, 0.0f, 90.0f, g.home);   /* home: yaw 90 */
    gimbal_set_setpoint_rel_euler(&g, 0.0f, 0.0f, 30.0f);
    gimbal_euler_to_quat(0.0f, 0.0f, 120.0f, q);       /* 90 + 30 */
    near("yaw 90 + 30 -> SP_QW", sp_reg(0), q[0] * 16384.0, 3);
    near("yaw 90 + 30 -> SP_QZ", sp_reg(3), q[3] * 16384.0, 3);

    gimbal_home(&g);
    gimbal_euler_to_quat(0.0f, 0.0f, 90.0f, q);
    near("home restores the startup attitude", sp_reg(3), q[3] * 16384.0, 3);

    /* ---- 5. Step rate and phase increment ------------------------------ */
    printf("\n[5] Step rate conversion\n");
    {
        uint32_t inc = gimbal_rate_to_inc(&g, 20000.0f);
        near("20 kHz to increment", inc, 858993.0, 2.0);
        near("round trip", gimbal_inc_to_rate(&g, (int32_t)inc),
             20000.0, 0.1);
        near("zero rate maps to zero", gimbal_rate_to_inc(&g, 0.0f), 0.0, 0.0);
    }

    gimbal_set_rate_hz(&g, 100u);
    ok("SAMPLE_DIV is 1e6 at 100 Hz", REG(GR_SAMPLE_DIV) == 1000000u);
    ok("loop rate reads back", gimbal_get_rate_hz(&g) == 100u);

    gimbal_set_limits(&g, 4000.0f, 40000.0f);
    near("OUT_LIM", REG(GR_OUT_LIM), 171798.0, 2.0);
    near("ACC_LIM (per control tick)", REG(GR_ACC_LIM), 17179.0, 2.0);
    {
        float mr, ma;
        gimbal_get_limits(&g, &mr, &ma);
        near("limits read back: rate",         mr,  4000.0, 1.0);
        near("limits read back: acceleration", ma, 40000.0, 20.0);
    }

    /* ---- 6. Controller gains ------------------------------------------- */
    printf("\n[6] Controller gains\n");
    gimbal_set_pid(&g, GIMBAL_AXIS_ROLL, 150.0f, 2.5f, 8.0f);
    ok("Kp 150.0 maps to 9830400", REG(GR_PID_KP(0)) == 9830400u);
    ok("Ki 2.5   maps to 163840",  REG(GR_PID_KI(0)) == 163840u);
    ok("Kd 8.0   maps to 524288",  REG(GR_PID_KD(0)) == 524288u);
    {
        float kp, ki, kd;
        gimbal_get_pid(&g, GIMBAL_AXIS_ROLL, &kp, &ki, &kd);
        near("Kp reads back", kp, 150.0, 1e-3);
        near("Ki reads back", ki,   2.5, 1e-3);
        near("Kd reads back", kd,   8.0, 1e-3);
    }

    /* ---- 7. Control register ------------------------------------------- */
    printf("\n[7] Control register\n");
    REG(GR_CTRL) = 0;
    gimbal_enable_control(&g, 1);
    gimbal_enable_motors(&g, 1);
    ok("CTRL_EN and MOT_EN set",
       (REG(GR_CTRL) & (CTRL_CTRL_EN | CTRL_MOT_EN))
           == (CTRL_CTRL_EN | CTRL_MOT_EN));
    gimbal_capture_setpoint(&g);
    ok("CAPTURE is a pulse and does not stay asserted",
       (REG(GR_CTRL) & CTRL_CAPTURE) == 0u);
    gimbal_set_dir_invert(&g, 5u);
    ok("direction inversion is 5",
       ((REG(GR_CTRL) & CTRL_DIR_INV_MASK) >> CTRL_DIR_INV_SHIFT) == 5u);
    ok("enables survive unrelated writes",
       (REG(GR_CTRL) & (CTRL_CTRL_EN | CTRL_MOT_EN))
           == (CTRL_CTRL_EN | CTRL_MOT_EN));
    gimbal_stop(&g);
    ok("stop clears every enable", REG(GR_CTRL) == 0u);

    /* ---- 8. SPI clock ---------------------------------------------------- */
    printf("\n[8] SPI clock\n");
    gimbal_set_spi_hz(&g, 1000000u);
    ok("1 MHz maps to divider 49", REG(GR_SPI_DIV) == 49u);
    ok("1 MHz reads back",         gimbal_get_spi_hz(&g) == 1000000u);
    gimbal_set_spi_hz(&g, 90000000u);
    ok("excessive request clamps to divider 2", REG(GR_SPI_DIV) == 2u);

    /* ---- 9. Shell parser -------------------------------------------------- */
    printf("\n[9] Shell parser\n");
    strcpy(line, "rpy 0 0 0");
    ok("known command accepted", shell_exec(&g, line) == 0);
    strcpy(line, "nonsense 1 2");
    ok("unknown command reports an error", shell_exec(&g, line) == -1);
    strcpy(line, "   ");
    ok("blank line is harmless", shell_exec(&g, line) == 0);

    strcpy(line, "reg 0x0c 0x1234abcd");
    shell_exec(&g, line);
    ok("reg write reaches the register", REG(GR_SCRATCH) == 0x1234abcdu);

    strcpy(line, "pid y -12.5 0.25 3");
    shell_exec(&g, line);
    ok("negative gains are accepted",
       (int32_t)REG(GR_PID_KP(1)) == -819200);

    strcpy(line, "invert 3");
    shell_exec(&g, line);
    ok("invert applied through the shell",
       ((REG(GR_CTRL) & CTRL_DIR_INV_MASK) >> CTRL_DIR_INV_SHIFT) == 3u);

    strcpy(line, "pid q 1 2 3");
    ok("invalid axis is rejected cleanly", shell_exec(&g, line) == 0);

    /* ---- Result ------------------------------------------------------------ */
    printf("\n=========================================================\n");
    if (fails == 0) printf(" RESULT: ALL TESTS PASSED\n");
    else            printf(" RESULT: %d TEST(S) FAILED\n", fails);
    printf("=========================================================\n");
    return fails ? 1 : 0;
}
