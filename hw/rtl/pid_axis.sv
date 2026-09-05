// ---------------------------------------------------------------------------
// pid_axis.sv - Fixed-point PID controller for one axis
//
//   u = Kp*e + Ki*integral(e) + Kd*d
//
//   e   : attitude error from quat_err (Q1.14, 16384 ~ 1 rad)
//   d   : either -gyro (derivative on measurement, no setpoint kick)
//         or e[n] - e[n-1]
//   K*  : Q16.16 (register value 65536 equals a gain of 1.0)
//   u   : phase increment for the stepper DDS (signed)
//
//   Anti-windup: the integrator is frozen while the output saturates in the
//   direction the error would keep pushing it.
//
//   The computation takes three cycles after 'tick'. At a 100 Hz sample rate
//   that is free and it avoids pipelining the multipliers.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none

module pid_axis (
    input  wire                clk,
    input  wire                rst_n,

    input  wire                tick,          // sample strobe (one cycle)
    input  wire                enable,
    input  wire                iclear,
    input  wire                d_from_gyro,

    input  wire signed [17:0]  err,
    input  wire signed [15:0]  gyr,           // 1/16 dps

    input  wire signed [31:0]  kp, ki, kd,    // Q16.16
    input  wire        [31:0]  i_lim,
    input  wire        [31:0]  o_lim,

    output logic signed [31:0] u
);

    logic [1:0]         st;
    logic signed [17:0] err_r, err_prev;
    logic signed [49:0] mp, mi;
    logic signed [51:0] md;
    logic signed [19:0] dsel;
    logic signed [63:0] i_acc, i_cand;

    wire signed [63:0] i_max = signed'({32'd0, i_lim}) <<< 16;
    wire signed [63:0] o_max = signed'({32'd0, o_lim});

    function automatic logic signed [63:0] clamp64(input logic signed [63:0] v,
                                                   input logic signed [63:0] lim);
        if (v > lim)       clamp64 =  lim;
        else if (v < -lim) clamp64 = -lim;
        else               clamp64 =  v;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin : pid_proc
        automatic logic signed [63:0] u_raw = '0;

        if (!rst_n) begin
            st       <= 2'd0;
            err_r    <= '0;
            err_prev <= '0;
            mp <= '0; mi <= '0; md <= '0;
            dsel   <= '0;
            i_acc  <= '0;
            i_cand <= '0;
            u      <= '0;
        end else if (!enable) begin
            st    <= 2'd0;
            i_acc <= '0;
            u     <= '0;
        end else begin
            if (tick) begin
                st    <= 2'd1;
                err_r <= err;
            end else if (st != 2'd0) begin
                st <= st + 2'd1;      // 1 -> 2 -> 3 -> 0
            end

            unique case (st)
                2'd1: begin
                    mp       <= kp * err_r;
                    mi       <= ki * err_r;
                    dsel     <= d_from_gyro ? -20'(gyr) : 20'(err_r - err_prev);
                    err_prev <= err_r;
                end

                2'd2: begin
                    md     <= kd * dsel;
                    i_cand <= clamp64(i_acc + 64'(mi), i_max);
                end

                2'd3: begin
                    u_raw = 64'(mp >>> 16) + 64'(i_cand >>> 16) + 64'(md >>> 16);

                    // Conditional integration: only commit when the output is
                    // not saturating in the direction of the error.
                    if (!iclear &&
                        !((u_raw >  o_max && err_r > 0) ||
                          (u_raw < -o_max && err_r < 0)))
                        i_acc <= i_cand;
                    else if (iclear)
                        i_acc <= '0;

                    if (u_raw >  o_max)      u <=  32'(o_max);
                    else if (u_raw < -o_max) u <= -32'(o_max);
                    else                     u <=  32'(u_raw);
                end

                default: begin
                    if (iclear) i_acc <= '0;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
