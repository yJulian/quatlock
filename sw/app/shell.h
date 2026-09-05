/* -------------------------------------------------------------------------
 * shell.h - Minimal command line on the serial console
 * ------------------------------------------------------------------------- */
#ifndef SHELL_H
#define SHELL_H

#include "gimbal_ctrl.h"

/* Blocks forever, reading and dispatching commands from the console. */
void shell_run(gimbal_t *g);

/* Execute a single command, e.g. from a startup script.
 * Returns 0 if the command was recognized. */
int  shell_exec(gimbal_t *g, char *line);

#endif /* SHELL_H */
