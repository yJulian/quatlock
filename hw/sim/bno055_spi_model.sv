// ---------------------------------------------------------------------------
// bno055_spi_model.sv - Behavioral model of the BNO055 in SPI mode (sim only)
//
// Implements the packet protocol of the sensor:
//   Request  : 0xAA <rw> <reg> <len> [data...]
//   Response : 0xBB <len> <data...>        (read OK)
//              0xEE <status>               (write ack / error)
//
// The response is preceded by RESP_DELAY dummy bytes so that the master's poll
// loop is exercised. A rising chip select flushes the response queue, which
// starts a fresh transaction.
//
// Measurement values are supplied through ports so the testbench can change
// the sensor attitude at run time.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps

module bno055_spi_model #(
    parameter int RESP_DELAY = 2
) (
    input  wire        csn,
    input  wire        sclk,
    input  wire        mosi,
    output wire        miso,

    input  wire signed [15:0] set_gx, set_gy, set_gz,
    input  wire signed [15:0] set_eh, set_er, set_ep,
    input  wire signed [15:0] set_qw, set_qx, set_qy, set_qz,
    input  wire        [7:0]  set_calib,
    input  wire               inject_err,   // 1 -> answer reads with 0xEE 0x02

    output logic [31:0]       n_reads,
    output logic [31:0]       n_writes
);

    logic [7:0] wregs [0:255];   // registers written by the master
    logic [7:0] shin, shout;
    logic       miso_r;
    int         bitcnt;

    byte unsigned q [$];

    // Protocol parser
    int         pstate;
    logic       is_wr;
    logic [7:0] radd, rlen;
    int         dcnt;

    assign miso = miso_r;

    function automatic logic [7:0] reg_rd(input logic [7:0] a);
        case (a)
            8'h00:   reg_rd = 8'hA0;              // CHIP_ID
            8'h01:   reg_rd = 8'hFB;              // ACC_ID
            8'h14:   reg_rd = set_gx[7:0];   8'h15: reg_rd = set_gx[15:8];
            8'h16:   reg_rd = set_gy[7:0];   8'h17: reg_rd = set_gy[15:8];
            8'h18:   reg_rd = set_gz[7:0];   8'h19: reg_rd = set_gz[15:8];
            8'h1A:   reg_rd = set_eh[7:0];   8'h1B: reg_rd = set_eh[15:8];
            8'h1C:   reg_rd = set_er[7:0];   8'h1D: reg_rd = set_er[15:8];
            8'h1E:   reg_rd = set_ep[7:0];   8'h1F: reg_rd = set_ep[15:8];
            8'h20:   reg_rd = set_qw[7:0];   8'h21: reg_rd = set_qw[15:8];
            8'h22:   reg_rd = set_qx[7:0];   8'h23: reg_rd = set_qx[15:8];
            8'h24:   reg_rd = set_qy[7:0];   8'h25: reg_rd = set_qy[15:8];
            8'h26:   reg_rd = set_qz[7:0];   8'h27: reg_rd = set_qz[15:8];
            8'h35:   reg_rd = set_calib;
            default: reg_rd = wregs[a];
        endcase
    endfunction

    initial begin
        for (int i = 0; i < 256; i++) wregs[i] = 8'h00;
        miso_r   = 1'b0;
        shin     = 8'h00;
        shout    = 8'h00;
        bitcnt   = 0;
        pstate   = 0;
        dcnt     = 0;
        n_reads  = 0;
        n_writes = 0;
        q.delete();
    end

    // ---- Chip select falling: reset the transaction ---------------------
    always @(negedge csn) begin
        bitcnt = 0;
        pstate = 0;
        dcnt   = 0;
        q.delete();
    end

    // ---- Sample MOSI on the rising edge (mode 3) ------------------------
    always @(posedge sclk) begin
        if (!csn) begin
            shin   = {shin[6:0], mosi};
            bitcnt = bitcnt + 1;
            if (bitcnt == 8) begin
                bitcnt = 0;
                handle_byte(shin);
            end
        end
    end

    task automatic handle_byte(input logic [7:0] b);
        case (pstate)
            0: if (b == 8'hAA) pstate = 1;
            1: begin is_wr = (b == 8'h00); pstate = 2; end
            2: begin radd = b; pstate = 3; end
            3: begin
                   rlen = (b == 8'h00) ? 8'd1 : b;
                   if (is_wr) begin
                       dcnt   = 0;
                       pstate = 4;
                   end else begin
                       for (int d = 0; d < RESP_DELAY; d++) q.push_back(8'h00);
                       if (inject_err) begin
                           q.push_back(8'hEE);
                           q.push_back(8'h02);
                       end else begin
                           q.push_back(8'hBB);
                           q.push_back(rlen);
                           for (int i = 0; i < rlen; i++)
                               q.push_back(reg_rd(radd + 8'(i)));
                           n_reads = n_reads + 1;
                       end
                       pstate = 0;
                   end
               end
            4: begin
                   wregs[radd + 8'(dcnt)] = b;
                   dcnt = dcnt + 1;
                   if (dcnt >= int'(rlen)) begin
                       for (int d = 0; d < RESP_DELAY; d++) q.push_back(8'h00);
                       q.push_back(8'hEE);
                       q.push_back(8'h01);       // WRITE_SUCCESS
                       n_writes = n_writes + 1;
                       pstate   = 0;
                   end
               end
            default: pstate = 0;
        endcase
    endtask

    // ---- Drive MISO on the falling edge (mode 3) ------------------------
    always @(negedge sclk or posedge csn) begin
        if (csn) begin
            miso_r <= 1'b0;
            shout  <= 8'h00;
        end else begin
            if (bitcnt == 0) begin
                logic [7:0] nb;
                nb     = (q.size() > 0) ? q.pop_front() : 8'h00;
                miso_r <= nb[7];
                shout  <= {nb[6:0], 1'b0};
            end else begin
                miso_r <= shout[7];
                shout  <= {shout[6:0], 1'b0};
            end
        end
    end

endmodule
