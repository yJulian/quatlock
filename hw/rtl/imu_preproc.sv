// ---------------------------------------------------------------------------
// imu_preproc.sv - Hardware preprocessing of the raw BNO055 sample
//
//  * Splits the 20-byte burst into signed 16-bit values (the BNO055 sends
//    LSB first).
//  * Exponential moving average without a multiplier:
//        y[n] = y[n-1] + (x[n] - y[n-1]) >> k
//    The state carries 8 extra fractional bits. k = 0 bypasses the filter.
//
//  Gyro and (optionally) the quaternion are filtered. Euler angles are
//  deliberately left unfiltered: heading wraps from 360 to 0 degrees and an
//  EMA would produce garbage across that discontinuity. The control path uses
//  the quaternion anyway.
//
//  Units (UNIT_SEL = 0x00):
//    gyro       1 LSB = 1/16 dps
//    euler      1 LSB = 1/16 degree
//    quaternion 1 LSB = 2^-14   (Q1.14)
//
//  Latency: valid pulses for one cycle two clocks after sample_valid, which is
//  when all outputs have been updated consistently.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none

module imu_preproc (
    input  wire                clk,
    input  wire                rst_n,

    input  wire [159:0]        sample_data,
    input  wire                sample_valid,

    input  wire [3:0]          gyr_shift,   // gyro EMA shift  (0 = off)
    input  wire [3:0]          quat_shift,  // quaternion EMA shift (0 = off)

    output logic signed [15:0] gyr_x,
    output logic signed [15:0] gyr_y,
    output logic signed [15:0] gyr_z,
    output logic signed [15:0] eul_yaw,
    output logic signed [15:0] eul_roll,
    output logic signed [15:0] eul_pitch,
    output logic signed [15:0] q_w,
    output logic signed [15:0] q_x,
    output logic signed [15:0] q_y,
    output logic signed [15:0] q_z,
    output logic               valid
);

    // ---- Raw values from the byte buffer (LSB first) --------------------
    function automatic logic signed [15:0] word_at(input logic [159:0] d,
                                                   input int unsigned i);
        word_at = signed'({d[8*(i+1) +: 8], d[8*i +: 8]});
    endfunction

    wire signed [15:0] raw_gyr_x = word_at(sample_data,  0);
    wire signed [15:0] raw_gyr_y = word_at(sample_data,  2);
    wire signed [15:0] raw_gyr_z = word_at(sample_data,  4);
    wire signed [15:0] raw_eul_h = word_at(sample_data,  6);
    wire signed [15:0] raw_eul_r = word_at(sample_data,  8);
    wire signed [15:0] raw_eul_p = word_at(sample_data, 10);
    wire signed [15:0] raw_q_w   = word_at(sample_data, 12);
    wire signed [15:0] raw_q_x   = word_at(sample_data, 14);
    wire signed [15:0] raw_q_y   = word_at(sample_data, 16);
    wire signed [15:0] raw_q_z   = word_at(sample_data, 18);

    // ---- Exponential moving average -------------------------------------
    function automatic logic signed [23:0] ema(input logic signed [23:0] st,
                                               input logic signed [15:0] x,
                                               input logic [3:0]         k);
        logic signed [24:0] xs;
        logic signed [24:0] st_e;
        logic signed [24:0] d;
        begin
            xs   = signed'({x, 8'h00});   // x * 256, sign preserved
            st_e = 25'(st);
            d    = xs - st_e;
            ema  = 24'(st_e + (d >>> k));
        end
    endfunction

    logic signed [23:0] s_gx, s_gy, s_gz;
    logic signed [23:0] s_qw, s_qx, s_qy, s_qz;
    logic               vd1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_gx <= '0; s_gy <= '0; s_gz <= '0;
            s_qw <= '0; s_qx <= '0; s_qy <= '0; s_qz <= '0;
            gyr_x <= '0; gyr_y <= '0; gyr_z <= '0;
            eul_yaw <= '0; eul_roll <= '0; eul_pitch <= '0;
            q_w <= '0; q_x <= '0; q_y <= '0; q_z <= '0;
            vd1   <= 1'b0;
            valid <= 1'b0;
        end else begin
            vd1   <= sample_valid;
            valid <= vd1;

            if (sample_valid) begin
                s_gx <= ema(s_gx, raw_gyr_x, gyr_shift);
                s_gy <= ema(s_gy, raw_gyr_y, gyr_shift);
                s_gz <= ema(s_gz, raw_gyr_z, gyr_shift);
                s_qw <= ema(s_qw, raw_q_w,   quat_shift);
                s_qx <= ema(s_qx, raw_q_x,   quat_shift);
                s_qy <= ema(s_qy, raw_q_y,   quat_shift);
                s_qz <= ema(s_qz, raw_q_z,   quat_shift);

                eul_yaw   <= raw_eul_h;
                eul_roll  <= raw_eul_r;
                eul_pitch <= raw_eul_p;
            end

            gyr_x <= s_gx[23:8];
            gyr_y <= s_gy[23:8];
            gyr_z <= s_gz[23:8];
            q_w   <= s_qw[23:8];
            q_x   <= s_qx[23:8];
            q_y   <= s_qy[23:8];
            q_z   <= s_qz[23:8];
        end
    end

endmodule

`default_nettype wire
