// ---------------------------------------------------------------------------
// quat_err.sv - Attitude error from setpoint and measured quaternion
//
//   q_err = conj(q_sp) (x) q_meas        rotation from setpoint to measurement
//   e     = 2 * vec( conj(q_err) ) * sign(w)
//         = -2 * sign(w_err) * vec(q_err)
//
// e is the rotation vector (in body coordinates) still required to move from
// the measured attitude to the setpoint. For small angles e ~= theta * axis in
// radians, represented as Q1.14 (16384 = 1.0, range +-2.0).
//
// The sign(w) factor always selects the shorter of the two equivalent rotation
// paths (q and -q describe the same attitude). Without it the controller would
// turn the wrong way for errors beyond 180 degrees.
//
// Two pipeline stages: stage 1 performs 16 multiplications, stage 2 the sums,
// scaling, sign selection and saturation. out_valid trails in_valid.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none

module quat_err (
    input  wire                clk,
    input  wire                rst_n,

    input  wire signed [15:0]  sp_w, sp_x, sp_y, sp_z,
    input  wire signed [15:0]  q_w,  q_x,  q_y,  q_z,
    input  wire                in_valid,

    output logic signed [17:0] err_x,
    output logic signed [17:0] err_y,
    output logic signed [17:0] err_z,
    output logic signed [15:0] eq_w,     // scalar part of q_err (Q1.14)
    output logic               out_valid
);

    // ---- Stage 1: products ----------------------------------------------
    logic signed [31:0] p_ww, p_xx, p_yy, p_zz;   // for w
    logic signed [31:0] p_wx, p_xw, p_yz, p_zy;   // for x
    logic signed [31:0] p_wy, p_xz, p_yw, p_zx;   // for y
    logic signed [31:0] p_wz, p_xy, p_yx, p_zw;   // for z
    logic               v1, v2;

    always_ff @(posedge clk) begin
        p_ww <= sp_w * q_w;  p_xx <= sp_x * q_x;  p_yy <= sp_y * q_y;  p_zz <= sp_z * q_z;
        p_wx <= sp_w * q_x;  p_xw <= sp_x * q_w;  p_yz <= sp_y * q_z;  p_zy <= sp_z * q_y;
        p_wy <= sp_w * q_y;  p_xz <= sp_x * q_z;  p_yw <= sp_y * q_w;  p_zx <= sp_z * q_x;
        p_wz <= sp_w * q_z;  p_xy <= sp_x * q_y;  p_yx <= sp_y * q_x;  p_zw <= sp_z * q_w;
    end

    // ---- Stage 2: sums, scaling, sign, saturation -----------------------
    // With conj(q_sp) = (w, -x, -y, -z) the product expands to:
    //   w_err =  spw*qw + spx*qx + spy*qy + spz*qz
    //   x_err =  spw*qx - spx*qw - spy*qz + spz*qy
    //   y_err =  spw*qy + spx*qz - spy*qw - spz*qx
    //   z_err =  spw*qz - spx*qy + spy*qx - spz*qw
    logic signed [33:0] sw, sx, sy, sz;
    always_comb begin
        sw = 34'(p_ww) + 34'(p_xx) + 34'(p_yy) + 34'(p_zz);
        sx = 34'(p_wx) - 34'(p_xw) - 34'(p_yz) + 34'(p_zy);
        sy = 34'(p_wy) + 34'(p_xz) - 34'(p_yw) - 34'(p_zx);
        sz = 34'(p_wz) - 34'(p_xy) + 34'(p_yx) - 34'(p_zw);
    end

    function automatic logic signed [17:0] sat18(input logic signed [33:0] v);
        if (v >  34'sd32767) sat18 = 18'sd32767;
        else if (v < -34'sd32767) sat18 = -18'sd32767;
        else sat18 = 18'(v);
    endfunction

    // -2 * sign(w) * vec  ->  negate when w >= 0, keep otherwise
    function automatic logic signed [33:0] scale_err(input logic signed [33:0] v,
                                                     input logic               w_neg);
        logic signed [33:0] t;
        begin
            t = (v >>> 14) <<< 1;         // back to Q1.14, then times two
            scale_err = w_neg ? t : -t;
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            err_x <= '0; err_y <= '0; err_z <= '0;
            eq_w  <= '0;
            v1 <= 1'b0; v2 <= 1'b0;
            out_valid <= 1'b0;
        end else begin
            v1        <= in_valid;
            v2        <= v1;
            out_valid <= v2;
            if (v1) begin
                eq_w  <= 16'(sw >>> 14);
                err_x <= sat18(scale_err(sx, sw[33]));
                err_y <= sat18(scale_err(sy, sw[33]));
                err_z <= sat18(scale_err(sz, sw[33]));
            end
        end
    end

endmodule

`default_nettype wire
