/* -------------------------------------------------------------------------
 * shell.c - Command line for the IMU gimbal
 *
 * Floating point output goes through f2s() because xil_printf does not
 * implement %f. Numeric input uses a local parser so no sscanf or strtod is
 * required in the standalone BSP.
 * ------------------------------------------------------------------------- */

#include "shell.h"

#include <string.h>
#include <stdint.h>

#if defined(__has_include)
#  if __has_include("xil_printf.h")
#    define SHELL_HAVE_XIL 1
#  endif
#endif

#ifdef SHELL_HAVE_XIL
#  include "xil_printf.h"
#  include "sleep.h"
#  define PR              xil_printf
#  define SLEEP_MS(ms)    usleep((ms) * 1000u)
static int sh_getc(void) { return (int)(unsigned char)inbyte(); }
static void sh_putc(char c) { outbyte(c); }
#else
#  include <stdio.h>
#  define PR              printf
#  define SLEEP_MS(ms)    do { volatile unsigned long _d = (ms) * 20000ul; \
                               while (_d--) { } } while (0)
static int sh_getc(void) { return getchar(); }
static void sh_putc(char c) { putchar(c); }
#endif

#define LINE_MAX   128
#define ARG_MAX    12

/* ===================== Float to text ==================================== */

static char f2s_buf[4][24];
static int  f2s_idx;

static const char *f2s(float v, int dec)
{
    char *out = f2s_buf[f2s_idx & 3];
    char tmp[16];
    long scale = 1, ip, fp;
    int  neg = 0, n = 0, i, p = 0;

    f2s_idx++;
    for (i = 0; i < dec; i++) scale *= 10;

    if (v < 0.0f) { neg = 1; v = -v; }
    ip = (long)v;
    fp = (long)((v - (float)ip) * (float)scale + 0.5f);
    if (fp >= scale) { fp -= scale; ip += 1; }

    if (neg) out[p++] = '-';
    if (ip == 0) { out[p++] = '0'; }
    else {
        while (ip > 0 && n < 15) { tmp[n++] = (char)('0' + (ip % 10)); ip /= 10; }
        while (n > 0) out[p++] = tmp[--n];
    }
    if (dec > 0) {
        out[p++] = '.';
        for (i = dec - 1; i >= 0; i--) {
            long d = fp;
            int  j;
            for (j = 0; j < i; j++) d /= 10;
            out[p++] = (char)('0' + (d % 10));
        }
    }
    out[p] = '\0';
    return out;
}

/* ===================== Text to number =================================== */

static int parse_float(const char *s, float *out)
{
    int   neg = 0, seen = 0;
    float v = 0.0f, frac = 0.1f;

    if (s == NULL || *s == '\0') return -1;
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') { s++; }

    while (*s >= '0' && *s <= '9') { v = v * 10.0f + (float)(*s - '0'); s++; seen = 1; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            v += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
            seen = 1;
        }
    }
    if (!seen || *s != '\0') return -1;
    *out = neg ? -v : v;
    return 0;
}

static int parse_u32(const char *s, uint32_t *out)
{
    uint32_t v = 0;
    int base = 10, seen = 0;

    if (s == NULL || *s == '\0') return -1;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) { base = 16; s += 2; }

    while (*s) {
        int d;
        if      (*s >= '0' && *s <= '9') d = *s - '0';
        else if (base == 16 && *s >= 'a' && *s <= 'f') d = *s - 'a' + 10;
        else if (base == 16 && *s >= 'A' && *s <= 'F') d = *s - 'A' + 10;
        else return -1;
        v = v * (uint32_t)base + (uint32_t)d;
        s++;
        seen = 1;
    }
    if (!seen) return -1;
    *out = v;
    return 0;
}

static int parse_axis(const char *s)
{
    if (!strcmp(s, "x") || !strcmp(s, "roll")  || !strcmp(s, "0")) return 0;
    if (!strcmp(s, "y") || !strcmp(s, "pitch") || !strcmp(s, "1")) return 1;
    if (!strcmp(s, "z") || !strcmp(s, "yaw")   || !strcmp(s, "2")) return 2;
    return -1;
}

static const char *axis_name(int a)
{
    return (a == 0) ? "roll/X" : (a == 1) ? "pitch/Y" : "yaw/Z";
}

/* ===================== Output =========================================== */

static void print_help(void)
{
    PR("\r\nCommands:\r\n");
    PR("  help                       this list\r\n");
    PR("  status                     sensor, controller and motor state\r\n");
    PR("  show                       attitude, setpoint and error\r\n");
    PR("  mon <n>                    stream n measurements\r\n");
    PR("\r\n");
    PR("  start                      initialize the BNO055 (home = setpoint)\r\n");
    PR("  stop                       disable sensor, controller and motors\r\n");
    PR("  ctrl <0|1>                 enable the controller\r\n");
    PR("  motors <0|1>               enable the driver stages\r\n");
    PR("\r\n");
    PR("  rpy <r> <p> <y>            setpoint in degrees, relative to home\r\n");
    PR("  arpy <r> <p> <y>           setpoint in degrees, absolute\r\n");
    PR("  quat <w> <x> <y> <z>       setpoint as a quaternion\r\n");
    PR("  capture                    adopt the current attitude as setpoint\r\n");
    PR("  home                       return to the startup attitude\r\n");
    PR("\r\n");
    PR("  pid <axis> <kp> <ki> <kd>  controller gains (axis: x|y|z)\r\n");
    PR("  ilim <axis> <steps/s>      integrator limit\r\n");
    PR("  limit <steps/s> <steps/s2> maximum rate and acceleration\r\n");
    PR("  dgyro <0|1>                derivative on gyro instead of de/dt\r\n");
    PR("  filt <gyro_k> <quat_k>     EMA filter strength (0 = off)\r\n");
    PR("  ms <0..7>                  microstep pins of the driver stage\r\n");
    PR("  invert <mask>              direction per motor (bits 0..2)\r\n");
    PR("  rate <hz>                  control loop rate\r\n");
    PR("  spi <hz>                   SPI clock to the BNO055\r\n");
    PR("  zero                       reset the step counters\r\n");
    PR("  clr                        clear the integrators\r\n");
    PR("  reg <offset> [value]       raw register read/write (hex allowed)\r\n");
}

static void print_status(gimbal_t *g)
{
    gimbal_state_t st;
    float rate, acc, kp, ki, kd;
    uint32_t ctrl = gimbal_reg_read(g, GR_CTRL);
    int i;

    gimbal_get_state(g, &st);
    gimbal_get_limits(g, &rate, &acc);

    PR("\r\n--- Sensor ---------------------------------------------\r\n");
    PR("  IMU_OK        : %s\r\n", (st.status & STAT_IMU_OK)    ? "yes" : "no");
    PR("  INIT_DONE     : %s\r\n", (st.status & STAT_INIT_DONE) ? "yes" : "no");
    PR("  samples       : %u\r\n", (unsigned)st.sample_cnt);
    PR("  SPI errors    : %u (last code 0x%02x)\r\n",
       (unsigned)st.err_cnt, st.last_err);
    PR("  calibration   : sys=%d gyr=%d acc=%d mag=%d  (3 = full)\r\n",
       CALIB_SYS(st.calib), CALIB_GYR(st.calib),
       CALIB_ACC(st.calib), CALIB_MAG(st.calib));
    PR("  SPI clock     : %u Hz, loop rate %u Hz\r\n",
       (unsigned)gimbal_get_spi_hz(g), (unsigned)gimbal_get_rate_hz(g));

    PR("--- Enables --------------------------------------------\r\n");
    PR("  IMU=%d  controller=%d  motors=%d  d-on-gyro=%d  dir-invert=%d\r\n",
       (ctrl & CTRL_IMU_EN) ? 1 : 0,
       (ctrl & CTRL_CTRL_EN) ? 1 : 0,
       (ctrl & CTRL_MOT_EN) ? 1 : 0,
       (ctrl & CTRL_D_FROM_GYRO) ? 1 : 0,
       (int)((ctrl & CTRL_DIR_INV_MASK) >> CTRL_DIR_INV_SHIFT));

    PR("--- Controller -----------------------------------------\r\n");
    PR("  limits: %s steps/s, %s steps/s^2\r\n", f2s(rate, 1), f2s(acc, 1));
    for (i = 0; i < GIMBAL_NAXIS; i++) {
        gimbal_get_pid(g, i, &kp, &ki, &kd);
        PR("  %-8s Kp=%-10s Ki=%-10s Kd=%-10s\r\n",
           axis_name(i), f2s(kp, 3), f2s(ki, 3), f2s(kd, 3));
    }

    PR("--- Motors ---------------------------------------------\r\n");
    for (i = 0; i < GIMBAL_NAXIS; i++)
        PR("  M%d (%-8s) position %-10d  rate %s steps/s\r\n",
           i, axis_name(i), (int)st.mot_pos[i], f2s(st.mot_rate[i], 1));
}

static void print_show(gimbal_t *g)
{
    gimbal_state_t st;
    gimbal_get_state(g, &st);

    PR("  measured q = [%s %s %s %s]\r\n",
       f2s(st.q[0], 4), f2s(st.q[1], 4), f2s(st.q[2], 4), f2s(st.q[3], 4));
    PR("  setpoint q = [%s %s %s %s]\r\n",
       f2s(st.sp[0], 4), f2s(st.sp[1], 4), f2s(st.sp[2], 4), f2s(st.sp[3], 4));
    PR("  euler      : roll %s  pitch %s  yaw %s  [deg]\r\n",
       f2s(st.eul[0], 2), f2s(st.eul[1], 2), f2s(st.eul[2], 2));
    PR("  gyro       : %s %s %s [dps]\r\n",
       f2s(st.gyr[0], 2), f2s(st.gyr[1], 2), f2s(st.gyr[2], 2));
    PR("  error      : %s %s %s [deg]\r\n",
       f2s(st.err[0] * 57.2958f, 2),
       f2s(st.err[1] * 57.2958f, 2),
       f2s(st.err[2] * 57.2958f, 2));
    PR("  motors     : %s %s %s [steps/s]\r\n",
       f2s(st.mot_rate[0], 0), f2s(st.mot_rate[1], 0), f2s(st.mot_rate[2], 0));
}

/* ===================== Command dispatch ================================= */

int shell_exec(gimbal_t *g, char *line)
{
    char    *argv[ARG_MAX];
    int      argc = 0;
    char    *p = line;
    float    a, b, c;
    uint32_t u, u2;
    int      ax;

    /* tokenize in place */
    while (*p && argc < ARG_MAX) {
        while (*p == ' ' || *p == '\t') *p++ = '\0';
        if (*p == '\0') break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
    }
    if (argc == 0) return 0;

    /* ---------------------------------------------------------------- */
    if (!strcmp(argv[0], "help") || !strcmp(argv[0], "?")) {
        print_help();
    }
    else if (!strcmp(argv[0], "status")) {
        print_status(g);
    }
    else if (!strcmp(argv[0], "show")) {
        print_show(g);
    }
    else if (!strcmp(argv[0], "mon")) {
        uint32_t n = 20;
        if (argc > 1 && parse_u32(argv[1], &n) != 0) { PR("  expected a number\r\n"); return 0; }
        while (n--) {
            gimbal_state_t st;
            gimbal_get_state(g, &st);
            PR("  rpy %8s %8s %8s | err %7s %7s %7s | mot %6s %6s %6s\r\n",
               f2s(st.eul[0], 1), f2s(st.eul[1], 1), f2s(st.eul[2], 1),
               f2s(st.err[0] * 57.2958f, 1),
               f2s(st.err[1] * 57.2958f, 1),
               f2s(st.err[2] * 57.2958f, 1),
               f2s(st.mot_rate[0], 0), f2s(st.mot_rate[1], 0),
               f2s(st.mot_rate[2], 0));
            SLEEP_MS(100);
        }
    }
    /* ---------------------------------------------------------------- */
    else if (!strcmp(argv[0], "start")) {
        PR("  initializing the BNO055 (up to 2 s) ...\r\n");
        if (gimbal_start(g, 3000) == GIMBAL_OK) {
            PR("  ready. startup attitude adopted as setpoint:\r\n");
            PR("  home q = [%s %s %s %s]\r\n",
               f2s(g->home[0], 4), f2s(g->home[1], 4),
               f2s(g->home[2], 4), f2s(g->home[3], 4));
        } else {
            PR("  ERROR: no response from the sensor (STATUS=0x%08x).\r\n",
               (unsigned)gimbal_get_status(g));
            PR("  Check wiring, PS1/PS0 strapped for SPI, and level shifters.\r\n");
        }
    }
    else if (!strcmp(argv[0], "stop")) {
        gimbal_stop(g);
        PR("  everything disabled\r\n");
    }
    else if (!strcmp(argv[0], "ctrl")) {
        if (argc < 2) { PR("  ctrl <0|1>\r\n"); return 0; }
        gimbal_enable_control(g, argv[1][0] != '0');
        PR("  controller %s\r\n", (argv[1][0] != '0') ? "enabled" : "disabled");
    }
    else if (!strcmp(argv[0], "motors")) {
        if (argc < 2) { PR("  motors <0|1>\r\n"); return 0; }
        gimbal_enable_motors(g, argv[1][0] != '0');
        PR("  driver stages %s\r\n", (argv[1][0] != '0') ? "enabled" : "disabled");
    }
    /* ---------------------------------------------------------------- */
    else if (!strcmp(argv[0], "rpy") || !strcmp(argv[0], "arpy")) {
        if (argc < 4 ||
            parse_float(argv[1], &a) || parse_float(argv[2], &b) ||
            parse_float(argv[3], &c)) {
            PR("  %s <roll> <pitch> <yaw>   (degrees)\r\n", argv[0]);
            return 0;
        }
        if (argv[0][0] == 'a') gimbal_set_setpoint_euler(g, a, b, c);
        else                   gimbal_set_setpoint_rel_euler(g, a, b, c);
        PR("  new setpoint: roll %s  pitch %s  yaw %s  (%s)\r\n",
           f2s(a, 2), f2s(b, 2), f2s(c, 2),
           (argv[0][0] == 'a') ? "absolute" : "relative to home");
    }
    else if (!strcmp(argv[0], "quat")) {
        float q[4];
        if (argc < 5 ||
            parse_float(argv[1], &q[0]) || parse_float(argv[2], &q[1]) ||
            parse_float(argv[3], &q[2]) || parse_float(argv[4], &q[3])) {
            PR("  quat <w> <x> <y> <z>\r\n");
            return 0;
        }
        gimbal_set_setpoint_quat(g, q);
        PR("  setpoint quaternion applied\r\n");
    }
    else if (!strcmp(argv[0], "capture")) {
        gimbal_capture_setpoint(g);
        PR("  current attitude adopted as setpoint\r\n");
    }
    else if (!strcmp(argv[0], "home")) {
        gimbal_home(g);
        PR("  setpoint reset to the startup attitude\r\n");
    }
    /* ---------------------------------------------------------------- */
    else if (!strcmp(argv[0], "pid")) {
        if (argc < 5 || (ax = parse_axis(argv[1])) < 0 ||
            parse_float(argv[2], &a) || parse_float(argv[3], &b) ||
            parse_float(argv[4], &c)) {
            PR("  pid <x|y|z> <kp> <ki> <kd>\r\n");
            return 0;
        }
        gimbal_set_pid(g, ax, a, b, c);
        PR("  %s: Kp=%s Ki=%s Kd=%s\r\n",
           axis_name(ax), f2s(a, 3), f2s(b, 3), f2s(c, 3));
    }
    else if (!strcmp(argv[0], "ilim")) {
        if (argc < 3 || (ax = parse_axis(argv[1])) < 0 ||
            parse_float(argv[2], &a)) {
            PR("  ilim <x|y|z> <steps/s>\r\n");
            return 0;
        }
        gimbal_set_ilimit(g, ax, a);
        PR("  integrator limit %s = %s steps/s\r\n", axis_name(ax), f2s(a, 1));
    }
    else if (!strcmp(argv[0], "limit")) {
        if (argc < 3 || parse_float(argv[1], &a) || parse_float(argv[2], &b)) {
            PR("  limit <max_steps/s> <max_steps/s^2>\r\n");
            return 0;
        }
        gimbal_set_limits(g, a, b);
        PR("  limits applied: %s steps/s, %s steps/s^2\r\n",
           f2s(a, 1), f2s(b, 1));
    }
    else if (!strcmp(argv[0], "dgyro")) {
        if (argc < 2) { PR("  dgyro <0|1>\r\n"); return 0; }
        gimbal_set_d_from_gyro(g, argv[1][0] != '0');
        PR("  derivative taken from %s\r\n",
           (argv[1][0] != '0') ? "the gyro" : "de/dt");
    }
    else if (!strcmp(argv[0], "filt")) {
        if (argc < 3 || parse_u32(argv[1], &u) || parse_u32(argv[2], &u2)) {
            PR("  filt <gyro_k> <quat_k>   (0 = off, useful range 0..6)\r\n");
            return 0;
        }
        gimbal_set_filter(g, u, u2);
        PR("  filter: gyro k=%u, quaternion k=%u\r\n", (unsigned)u, (unsigned)u2);
    }
    else if (!strcmp(argv[0], "ms")) {
        if (argc < 2 || parse_u32(argv[1], &u)) { PR("  ms <0..7>\r\n"); return 0; }
        gimbal_set_microstep(g, u);
        PR("  microstep pins = %u\r\n", (unsigned)(u & 7u));
    }
    else if (!strcmp(argv[0], "invert")) {
        if (argc < 2 || parse_u32(argv[1], &u)) { PR("  invert <0..7>\r\n"); return 0; }
        gimbal_set_dir_invert(g, u);
        PR("  direction inversion = %u\r\n", (unsigned)(u & 7u));
    }
    else if (!strcmp(argv[0], "rate")) {
        if (argc < 2 || parse_u32(argv[1], &u)) { PR("  rate <hz>\r\n"); return 0; }
        gimbal_set_rate_hz(g, u);
        PR("  loop rate = %u Hz\r\n", (unsigned)gimbal_get_rate_hz(g));
    }
    else if (!strcmp(argv[0], "spi")) {
        if (argc < 2 || parse_u32(argv[1], &u)) { PR("  spi <hz>\r\n"); return 0; }
        gimbal_set_spi_hz(g, u);
        PR("  SPI clock = %u Hz\r\n", (unsigned)gimbal_get_spi_hz(g));
    }
    else if (!strcmp(argv[0], "zero")) {
        gimbal_zero_positions(g);
        PR("  step counters cleared\r\n");
    }
    else if (!strcmp(argv[0], "clr")) {
        gimbal_clear_integrators(g);
        PR("  integrators cleared\r\n");
    }
    /* ---------------------------------------------------------------- */
    else if (!strcmp(argv[0], "reg")) {
        if (argc < 2 || parse_u32(argv[1], &u)) {
            PR("  reg <offset> [value]\r\n");
            return 0;
        }
        if (argc >= 3) {
            if (parse_u32(argv[2], &u2)) { PR("  invalid value\r\n"); return 0; }
            gimbal_reg_write(g, u, u2);
            PR("  [0x%03x] <- 0x%08x\r\n", (unsigned)u, (unsigned)u2);
        } else {
            PR("  [0x%03x] = 0x%08x (%d)\r\n", (unsigned)u,
               (unsigned)gimbal_reg_read(g, u),
               (int)gimbal_reg_read(g, u));
        }
    }
    else {
        PR("  unknown command '%s' - type 'help'\r\n", argv[0]);
        return -1;
    }
    return 0;
}

/* ===================== Main loop ======================================== */

void shell_run(gimbal_t *g)
{
    char line[LINE_MAX];
    int  n = 0;

    PR("\r\nType 'help' for the command list.\r\n");
    PR("\r\ngimbal> ");

    for (;;) {
        int ch = sh_getc();

        if (ch == '\r' || ch == '\n') {
            PR("\r\n");
            line[n] = '\0';
            if (n > 0) shell_exec(g, line);
            n = 0;
            PR("gimbal> ");
        } else if (ch == 0x08 || ch == 0x7F) {          /* backspace */
            if (n > 0) { n--; PR("\b \b"); }
        } else if (ch >= 0x20 && ch < 0x7F) {
            if (n < LINE_MAX - 1) { line[n++] = (char)ch; sh_putc((char)ch); }
        }
    }
}
