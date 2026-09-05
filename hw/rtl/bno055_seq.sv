// ---------------------------------------------------------------------------
// bno055_seq.sv - Startup and sampling sequencer for the BNO055
//
// Sequence after imu_en goes high:
//   1. Assert nRESET (hardware reset of the sensor)
//   2. Wait for power-on reset (datasheet: ~650 ms until the chip responds)
//   3. Read CHIP_ID (0x00) and verify it reads 0xA0
//   4. PAGE_ID=0, OPR_MODE=CONFIG, SYS_TRIGGER=0, PWR_MODE=normal,
//      UNIT_SEL=0 (dps / degrees / m/s^2 / Windows), OPR_MODE=NDOF (0x0C)
//   5. RUN: every sample_div cycles, one 20-byte burst read from 0x14
//        0x14..0x19  GYR_X/Y/Z    (LSB first, 1/16 dps)
//        0x1A..0x1F  EUL_H/R/P    (LSB first, 1/16 degree)
//        0x20..0x27  QUA_W/X/Y/Z  (LSB first, Q1.14)
//      Every 32nd cycle additionally reads CALIB_STAT (0x35).
//
// All delays scale with init_dly so that simulation can walk through the whole
// sequence in microseconds by writing a small value.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none

module bno055_seq (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         imu_en,
    input  wire [15:0]  spi_div,
    input  wire [15:0]  poll_max,
    input  wire [31:0]  init_dly,     // power-on reset delay in clock cycles
    input  wire [31:0]  sample_div,   // sample period in clock cycles

    output logic        imu_ok,
    output logic        init_done,
    output logic [7:0]  last_err,
    output logic [7:0]  calib_stat,
    output logic [31:0] sample_cnt,
    output logic [31:0] err_cnt,

    output logic [159:0] sample_data, // byte i = register 0x14 + i
    output logic         sample_valid,

    output logic [7:0]  dbg_state,

    // SPI pins to the BNO055
    output wire         imu_sclk,
    output wire         imu_mosi,
    output wire         imu_csn,
    output logic        imu_rstn,
    input  wire         imu_miso
);

    localparam int NBYTES   = 20;
    localparam int CFG_LAST = 6;   // index of the last configuration step

    // ---------------- States ---------------------------------------------
    localparam logic [3:0] S_OFF     = 4'd0;
    localparam logic [3:0] S_RSTLOW  = 4'd1;
    localparam logic [3:0] S_POR     = 4'd2;
    localparam logic [3:0] S_CFGREQ  = 4'd3;
    localparam logic [3:0] S_CFGWAIT = 4'd4;
    localparam logic [3:0] S_CFGDLY  = 4'd5;
    localparam logic [3:0] S_RUNIDLE = 4'd6;
    localparam logic [3:0] S_RDREQ   = 4'd7;
    localparam logic [3:0] S_RDWAIT  = 4'd8;
    localparam logic [3:0] S_CALREQ  = 4'd9;
    localparam logic [3:0] S_CALWAIT = 4'd10;
    localparam logic [3:0] S_FAIL    = 4'd11;

    logic [3:0]  state;
    logic [2:0]  step;
    logic [31:0] dly_cnt;
    logic [31:0] samp_cnt;
    logic [4:0]  cal_div;
    logic [3:0]  fail_cnt;

    // ---------------- Transaction engine ---------------------------------
    logic        req_valid, req_ready, req_write;
    logic [7:0]  req_reg, req_len, req_wdata;
    logic        resp_valid, resp_ok;
    logic [7:0]  resp_code;
    logic        rd_stb;
    logic [7:0]  rd_idx, rd_data;
    logic [3:0]  txn_state;

    logic [7:0]  rbuf [0:31];

    bno055_txn #(.CS_CYCLES(32)) u_txn (
        .clk        (clk),
        .rst_n      (rst_n),
        .spi_div    (spi_div),
        .poll_max   (poll_max),
        .req_valid  (req_valid),
        .req_ready  (req_ready),
        .req_write  (req_write),
        .req_reg    (req_reg),
        .req_len    (req_len),
        .req_wdata  (req_wdata),
        .resp_valid (resp_valid),
        .resp_ok    (resp_ok),
        .resp_code  (resp_code),
        .rd_stb     (rd_stb),
        .rd_idx     (rd_idx),
        .rd_data    (rd_data),
        .sclk       (imu_sclk),
        .mosi       (imu_mosi),
        .csn        (imu_csn),
        .miso       (imu_miso),
        .dbg_state  (txn_state)
    );

    assign dbg_state = {txn_state, state};

    // ---------------- Configuration ROM ----------------------------------
    // cfg_wr = 1 -> write access, otherwise read access compared to cfg_dat
    logic       cfg_wr;
    logic [7:0] cfg_reg, cfg_dat;
    logic       cfg_long;   // 1 -> long settling delay after this step

    always_comb begin
        unique case (step)
            3'd0: begin cfg_wr = 1'b0; cfg_reg = 8'h00; cfg_dat = 8'hA0; cfg_long = 1'b0; end // CHIP_ID
            3'd1: begin cfg_wr = 1'b1; cfg_reg = 8'h07; cfg_dat = 8'h00; cfg_long = 1'b0; end // PAGE_ID = 0
            3'd2: begin cfg_wr = 1'b1; cfg_reg = 8'h3D; cfg_dat = 8'h00; cfg_long = 1'b1; end // OPR_MODE = CONFIG
            3'd3: begin cfg_wr = 1'b1; cfg_reg = 8'h3F; cfg_dat = 8'h00; cfg_long = 1'b0; end // SYS_TRIGGER
            3'd4: begin cfg_wr = 1'b1; cfg_reg = 8'h3E; cfg_dat = 8'h00; cfg_long = 1'b0; end // PWR_MODE = normal
            3'd5: begin cfg_wr = 1'b1; cfg_reg = 8'h3B; cfg_dat = 8'h00; cfg_long = 1'b0; end // UNIT_SEL
            default: begin cfg_wr = 1'b1; cfg_reg = 8'h3D; cfg_dat = 8'h0C; cfg_long = 1'b1; end // OPR_MODE = NDOF
        endcase
    end

    wire [31:0] dly_short = (init_dly >> 8) | 32'd4;
    wire [31:0] dly_long  = (init_dly >> 4) | 32'd8;

    // ---------------- Read buffer ----------------------------------------
    always_ff @(posedge clk) begin
        if (rd_stb && rd_idx < 8'd32) rbuf[rd_idx] <= rd_data;
    end

    // ---------------- Main sequencer -------------------------------------
    integer i;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_OFF;
            step         <= 3'd0;
            dly_cnt      <= '0;
            samp_cnt     <= '0;
            cal_div      <= '0;
            fail_cnt     <= '0;
            req_valid    <= 1'b0;
            req_write    <= 1'b0;
            req_reg      <= '0;
            req_len      <= 8'd1;
            req_wdata    <= '0;
            imu_ok       <= 1'b0;
            init_done    <= 1'b0;
            last_err     <= '0;
            calib_stat   <= '0;
            sample_cnt   <= '0;
            err_cnt      <= '0;
            sample_data  <= '0;
            sample_valid <= 1'b0;
            imu_rstn     <= 1'b0;
        end else begin
            req_valid    <= 1'b0;
            sample_valid <= 1'b0;

            if (!imu_en) begin
                state     <= S_OFF;
                imu_rstn  <= 1'b0;
                imu_ok    <= 1'b0;
                init_done <= 1'b0;
            end else begin
                unique case (state)
                    // ---------------------------------------------------
                    S_OFF: begin
                        imu_rstn  <= 1'b0;
                        init_done <= 1'b0;
                        imu_ok    <= 1'b0;
                        step      <= 3'd0;
                        dly_cnt   <= dly_long;
                        state     <= S_RSTLOW;
                    end

                    S_RSTLOW: begin
                        imu_rstn <= 1'b0;
                        if (dly_cnt == 0) begin
                            imu_rstn <= 1'b1;
                            dly_cnt  <= init_dly;
                            state    <= S_POR;
                        end else begin
                            dly_cnt <= dly_cnt - 1'b1;
                        end
                    end

                    S_POR: begin
                        if (dly_cnt == 0) begin
                            step  <= 3'd0;
                            state <= S_CFGREQ;
                        end else begin
                            dly_cnt <= dly_cnt - 1'b1;
                        end
                    end

                    // ---------------------------------------------------
                    S_CFGREQ: begin
                        if (req_ready) begin
                            req_valid <= 1'b1;
                            req_write <= cfg_wr;
                            req_reg   <= cfg_reg;
                            req_len   <= 8'd1;
                            req_wdata <= cfg_dat;
                            state     <= S_CFGWAIT;
                        end
                    end

                    S_CFGWAIT: begin
                        if (resp_valid) begin
                            if (!resp_ok) begin
                                last_err <= resp_code;
                                err_cnt  <= err_cnt + 1'b1;
                                dly_cnt  <= dly_long;
                                state    <= S_FAIL;
                            end else if (step == 3'd0 && rbuf[0] != 8'hA0) begin
                                // CHIP_ID mismatch: no BNO055 on the bus
                                last_err <= 8'hF1;
                                err_cnt  <= err_cnt + 1'b1;
                                dly_cnt  <= dly_long;
                                state    <= S_FAIL;
                            end else begin
                                dly_cnt <= cfg_long ? dly_long : dly_short;
                                state   <= S_CFGDLY;
                            end
                        end
                    end

                    S_CFGDLY: begin
                        if (dly_cnt == 0) begin
                            if (step == CFG_LAST[2:0]) begin
                                imu_ok    <= 1'b1;
                                init_done <= 1'b1;
                                fail_cnt  <= '0;
                                samp_cnt  <= '0;
                                cal_div   <= '0;
                                state     <= S_RUNIDLE;
                            end else begin
                                step  <= step + 1'b1;
                                state <= S_CFGREQ;
                            end
                        end else begin
                            dly_cnt <= dly_cnt - 1'b1;
                        end
                    end

                    // ---------------------------------------------------
                    S_RUNIDLE: begin
                        if (samp_cnt >= sample_div) begin
                            samp_cnt <= '0;
                            state    <= S_RDREQ;
                        end else begin
                            samp_cnt <= samp_cnt + 1'b1;
                        end
                    end

                    S_RDREQ: begin
                        if (req_ready) begin
                            req_valid <= 1'b1;
                            req_write <= 1'b0;
                            req_reg   <= 8'h14;
                            req_len   <= NBYTES[7:0];
                            state     <= S_RDWAIT;
                        end
                    end

                    S_RDWAIT: begin
                        if (resp_valid) begin
                            if (resp_ok) begin
                                for (i = 0; i < NBYTES; i = i + 1)
                                    sample_data[8*i +: 8] <= rbuf[i];
                                sample_valid <= 1'b1;
                                sample_cnt   <= sample_cnt + 1'b1;
                                fail_cnt     <= '0;
                                if (cal_div == 5'd31) begin
                                    cal_div <= '0;
                                    state   <= S_CALREQ;
                                end else begin
                                    cal_div <= cal_div + 1'b1;
                                    state   <= S_RUNIDLE;
                                end
                            end else begin
                                last_err <= resp_code;
                                err_cnt  <= err_cnt + 1'b1;
                                if (fail_cnt >= 4'd7) begin
                                    imu_ok <= 1'b0;
                                    dly_cnt<= dly_long;
                                    state  <= S_FAIL;
                                end else begin
                                    fail_cnt <= fail_cnt + 1'b1;
                                    state    <= S_RUNIDLE;
                                end
                            end
                        end
                    end

                    S_CALREQ: begin
                        if (req_ready) begin
                            req_valid <= 1'b1;
                            req_write <= 1'b0;
                            req_reg   <= 8'h35;      // CALIB_STAT
                            req_len   <= 8'd1;
                            state     <= S_CALWAIT;
                        end
                    end

                    S_CALWAIT: begin
                        if (resp_valid) begin
                            if (resp_ok) calib_stat <= rbuf[0];
                            state <= S_RUNIDLE;
                        end
                    end

                    // ---------------------------------------------------
                    S_FAIL: begin
                        imu_ok    <= 1'b0;
                        init_done <= 1'b0;
                        if (dly_cnt == 0) state <= S_OFF;
                        else              dly_cnt <= dly_cnt - 1'b1;
                    end

                    default: state <= S_OFF;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
