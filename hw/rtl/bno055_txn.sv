// ---------------------------------------------------------------------------
// bno055_txn.sv - Transaction engine for the BNO055 SPI/UART packet protocol
//
// On SPI the BNO055 uses the same packet protocol as on UART:
//
//   Write request  : 0xAA 0x00 <reg> <len> <data0..N>
//   Write response : 0xEE <status>          (status == 0x01 -> WRITE_SUCCESS)
//
//   Read request   : 0xAA 0x01 <reg> <len>
//   Read response  : 0xBB <len> <data0..N>
//                    or 0xEE <error_code>
//
// The response is not available immediately: the master has to keep clocking
// and discard bytes until the 0xBB or 0xEE header appears. That is what the
// POLL phase does, bounded by poll_max.
//
// CS stays asserted for the complete transaction (request and response).
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none

module bno055_txn #(
    parameter int CS_CYCLES = 32   // CS setup/hold and inter-frame gap
) (
    input  wire         clk,
    input  wire         rst_n,

    input  wire [15:0]  spi_div,
    input  wire [15:0]  poll_max,

    // ---- Request ---------------------------------------------------------
    input  wire         req_valid,
    output logic        req_ready,
    input  wire         req_write,     // 1 = write, 0 = read
    input  wire [7:0]   req_reg,
    input  wire [7:0]   req_len,       // payload byte count (>= 1)
    input  wire [7:0]   req_wdata,     // single-byte writes only

    // ---- Response --------------------------------------------------------
    output logic        resp_valid,    // one-cycle pulse
    output logic        resp_ok,
    output logic [7:0]  resp_code,     // BNO055 status/error code, 0xF0 = timeout

    // ---- Read data stream ------------------------------------------------
    output logic        rd_stb,
    output logic [7:0]  rd_idx,
    output logic [7:0]  rd_data,

    // ---- SPI pins --------------------------------------------------------
    output wire         sclk,
    output wire         mosi,
    output wire         csn,
    input  wire         miso,

    output logic [3:0]  dbg_state
);

    localparam logic [7:0] START_BYTE = 8'hAA;
    localparam logic [7:0] RESP_READ  = 8'hBB;
    localparam logic [7:0] RESP_STAT  = 8'hEE;
    localparam logic [7:0] ERR_TMO    = 8'hF0;

    // Transaction state machine
    localparam logic [3:0] T_IDLE = 4'd0;
    localparam logic [3:0] T_CSLO = 4'd1;
    localparam logic [3:0] T_LOAD = 4'd2;
    localparam logic [3:0] T_XFER = 4'd3;
    localparam logic [3:0] T_EVAL = 4'd4;
    localparam logic [3:0] T_CSHI = 4'd5;
    localparam logic [3:0] T_GAP  = 4'd6;

    // Protocol phase within one transaction
    localparam logic [2:0] P_HDR   = 3'd0;
    localparam logic [2:0] P_WDATA = 3'd1;
    localparam logic [2:0] P_POLL  = 3'd2;
    localparam logic [2:0] P_LEN   = 3'd3;
    localparam logic [2:0] P_DATA  = 3'd4;
    localparam logic [2:0] P_CODE  = 3'd5;

    logic [3:0]  state;
    logic [2:0]  phase;
    logic [7:0]  idx;
    logic [15:0] poll_cnt;
    logic [15:0] gap_cnt;

    logic        c_write;
    logic [7:0]  c_reg, c_len, c_wdata;

    logic        cs_assert;
    logic        spi_start;
    logic [7:0]  spi_tx;
    logic [7:0]  spi_rx;
    logic        spi_done;

    assign dbg_state = state;
    assign req_ready = (state == T_IDLE);

    spi_master #(.DIV_W(16)) u_spi (
        .clk       (clk),
        .rst_n     (rst_n),
        .clk_div   (spi_div),
        .cs_assert (cs_assert),
        .start     (spi_start),
        .tx_byte   (spi_tx),
        .rx_byte   (spi_rx),
        .done      (spi_done),
        .busy      (),
        .sclk      (sclk),
        .mosi      (mosi),
        .csn       (csn),
        .miso      (miso)
    );

    // Next byte to transmit, derived from phase and index
    logic [7:0] next_tx;
    always_comb begin
        unique case (phase)
            P_HDR: begin
                unique case (idx[1:0])
                    2'd0: next_tx = START_BYTE;
                    2'd1: next_tx = c_write ? 8'h00 : 8'h01;
                    2'd2: next_tx = c_reg;
                    default: next_tx = c_len;
                endcase
            end
            P_WDATA: next_tx = c_wdata;
            default: next_tx = 8'hFF;   // dummy byte during the response phase
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= T_IDLE;
            phase      <= P_HDR;
            idx        <= '0;
            poll_cnt   <= '0;
            gap_cnt    <= '0;
            cs_assert  <= 1'b0;
            spi_start  <= 1'b0;
            spi_tx     <= '0;
            resp_valid <= 1'b0;
            resp_ok    <= 1'b0;
            resp_code  <= '0;
            rd_stb     <= 1'b0;
            rd_idx     <= '0;
            rd_data    <= '0;
            c_write    <= 1'b0;
            c_reg      <= '0;
            c_len      <= 8'd1;
            c_wdata    <= '0;
        end else begin
            spi_start  <= 1'b0;
            resp_valid <= 1'b0;
            rd_stb     <= 1'b0;

            unique case (state)
                // -------------------------------------------------------
                T_IDLE: begin
                    cs_assert <= 1'b0;
                    if (req_valid) begin
                        c_write  <= req_write;
                        c_reg    <= req_reg;
                        c_len    <= (req_len == 8'd0) ? 8'd1 : req_len;
                        c_wdata  <= req_wdata;
                        phase    <= P_HDR;
                        idx      <= '0;
                        poll_cnt <= '0;
                        gap_cnt  <= CS_CYCLES[15:0];
                        cs_assert<= 1'b1;
                        state    <= T_CSLO;
                    end
                end

                // CS setup time
                T_CSLO: begin
                    if (gap_cnt == 0) state <= T_LOAD;
                    else              gap_cnt <= gap_cnt - 1'b1;
                end

                // Hand the next byte to the SPI master
                T_LOAD: begin
                    spi_tx    <= next_tx;
                    spi_start <= 1'b1;
                    state     <= T_XFER;
                end

                T_XFER: begin
                    if (spi_done) state <= T_EVAL;
                end

                // -------------------------------------------------------
                T_EVAL: begin
                    state <= T_LOAD;   // default: continue with next byte
                    unique case (phase)
                        P_HDR: begin
                            if (idx < 8'd3) begin
                                idx <= idx + 1'b1;
                            end else if (c_write) begin
                                phase <= P_WDATA;
                                idx   <= '0;
                            end else begin
                                phase    <= P_POLL;
                                poll_cnt <= '0;
                            end
                        end

                        P_WDATA: begin
                            if (idx + 8'd1 >= c_len) begin
                                phase    <= P_POLL;
                                poll_cnt <= '0;
                            end else begin
                                idx <= idx + 1'b1;
                            end
                        end

                        P_POLL: begin
                            if (!c_write && spi_rx == RESP_READ) begin
                                phase <= P_LEN;
                            end else if (spi_rx == RESP_STAT) begin
                                phase <= P_CODE;
                            end else if (poll_cnt >= poll_max) begin
                                resp_ok   <= 1'b0;
                                resp_code <= ERR_TMO;
                                gap_cnt   <= CS_CYCLES[15:0];
                                state     <= T_CSHI;
                            end else begin
                                poll_cnt <= poll_cnt + 1'b1;
                            end
                        end

                        // Length byte of the read response
                        P_LEN: begin
                            phase <= P_DATA;
                            idx   <= '0;
                        end

                        P_DATA: begin
                            rd_stb  <= 1'b1;
                            rd_idx  <= idx;
                            rd_data <= spi_rx;
                            if (idx + 8'd1 >= c_len) begin
                                resp_ok   <= 1'b1;
                                resp_code <= 8'h00;
                                gap_cnt   <= CS_CYCLES[15:0];
                                state     <= T_CSHI;
                            end else begin
                                idx <= idx + 1'b1;
                            end
                        end

                        // 0xEE <status>: 0x01 means success for a write,
                        // any 0xEE response to a read is an error.
                        P_CODE: begin
                            resp_code <= spi_rx;
                            resp_ok   <= c_write && (spi_rx == 8'h01);
                            gap_cnt   <= CS_CYCLES[15:0];
                            state     <= T_CSHI;
                        end

                        default: state <= T_CSHI;
                    endcase
                end

                // -------------------------------------------------------
                T_CSHI: begin
                    if (gap_cnt == 0) begin
                        cs_assert <= 1'b0;
                        gap_cnt   <= CS_CYCLES[15:0];
                        state     <= T_GAP;
                    end else begin
                        gap_cnt <= gap_cnt - 1'b1;
                    end
                end

                T_GAP: begin
                    if (gap_cnt == 0) begin
                        resp_valid <= 1'b1;
                        state      <= T_IDLE;
                    end else begin
                        gap_cnt <= gap_cnt - 1'b1;
                    end
                end

                default: state <= T_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
