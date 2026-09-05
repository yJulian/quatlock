// ---------------------------------------------------------------------------
// stepper_drv.sv - STEP/DIR generation for a stepper driver stage
//                  (A4988 / DRV8825 / TMC compatible)
//
// The controller output is interpreted as the signed phase increment of a
// 32-bit DDS:
//
//      f_step = f_clk * |inc| / 2^32
//
// A slew rate limiter (acc_lim per control tick) prevents step loss on abrupt
// setpoint changes. The step position is tracked so software knows the joint
// angle.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none

module stepper_drv (
    input  wire                clk,
    input  wire                rst_n,

    input  wire                tick,        // control tick (drives the ramp)
    input  wire signed [31:0]  vel_cmd,     // target increment
    input  wire        [31:0]  acc_lim,     // max change per control tick
    input  wire        [31:0]  max_rate,    // magnitude limit
    input  wire        [15:0]  step_width,  // pulse width in clock cycles
    input  wire                enable,
    input  wire                invert_dir,
    input  wire                zero_pos,

    output logic               step,
    output logic               dir,
    output logic               en_n,
    output logic signed [31:0] cur_vel,
    output logic signed [31:0] position
);

    logic [31:0] phase;
    logic [15:0] pulse_cnt;

    wire signed [32:0] lim_p =  signed'({1'b0, max_rate});
    wire signed [32:0] lim_n = -signed'({1'b0, max_rate});

    // Clamp the target to max_rate
    wire signed [32:0] tgt_raw = 33'(vel_cmd);
    wire signed [32:0] tgt = (tgt_raw > lim_p) ? lim_p :
                             (tgt_raw < lim_n) ? lim_n : tgt_raw;

    wire signed [33:0] diff = 34'(tgt) - 34'(cur_vel);
    wire signed [33:0] acc  = 34'(signed'({1'b0, acc_lim}));

    wire [31:0] mag        = cur_vel[31] ? (~cur_vel + 32'd1) : cur_vel;
    wire        dir_i      = cur_vel[31] ^ invert_dir;
    wire [32:0] phase_next = {1'b0, phase} + {1'b0, mag};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_vel   <= '0;
            phase     <= '0;
            pulse_cnt <= '0;
            position  <= '0;
            step      <= 1'b0;
            dir       <= 1'b0;
            en_n      <= 1'b1;
        end else begin
            en_n <= ~enable;
            dir  <= dir_i;

            // ---- Slew rate limiter (control tick only) -------------------
            if (tick) begin
                if (!enable)            cur_vel <= '0;
                else if (diff >  acc)   cur_vel <= cur_vel + 32'(acc);
                else if (diff < -acc)   cur_vel <= cur_vel - 32'(acc);
                else                    cur_vel <= 32'(tgt);
            end

            // ---- DDS and pulse shaper -----------------------------------
            if (pulse_cnt != 0) begin
                pulse_cnt <= pulse_cnt - 1'b1;
                if (pulse_cnt == 16'd1) step <= 1'b0;
            end

            if (enable) begin
                phase <= phase_next[31:0];
                // Accumulator overflow means one step
                if (phase_next[32] && pulse_cnt == 0) begin
                    step      <= 1'b1;
                    pulse_cnt <= (step_width == 0) ? 16'd1 : step_width;
                    position  <= dir_i ? (position - 32'sd1) : (position + 32'sd1);
                end
            end else begin
                phase <= '0;
                step  <= 1'b0;
            end

            if (zero_pos) position <= '0;
        end
    end

endmodule

`default_nettype wire
