// ---------------------------------------------------------------------------
// spi_master.sv - Byte-oriented SPI master, mode 3 (CPOL = 1, CPHA = 1)
//
// The BNO055 in SPI mode keeps SCK idle high, drives MOSI on the falling edge
// and expects MISO to be sampled on the rising edge.
//
//   f_sclk = f_clk / (2 * (clk_div + 1))
//
// clk_div must be >= 2 so that the two-stage MISO synchronizer settles well
// inside the high phase. The register file clamps the value accordingly.
//
// Chip select is not driven here: the transaction FSM keeps CS asserted for a
// whole request/response sequence, as required by the BNO055 protocol.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none

module spi_master #(
    parameter int DIV_W = 16
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire [DIV_W-1:0]  clk_div,    // half period = clk_div + 1 cycles
    input  wire              cs_assert,  // level: 1 => csn = 0
    input  wire              start,      // pulse: transfer one byte
    input  wire [7:0]        tx_byte,
    output logic [7:0]       rx_byte,
    output logic             done,       // one-cycle pulse at end of byte
    output logic             busy,

    output logic             sclk,
    output logic             mosi,
    output wire              csn,
    input  wire              miso
);

    localparam logic [1:0] S_IDLE = 2'd0;
    localparam logic [1:0] S_LOW  = 2'd1;
    localparam logic [1:0] S_HIGH = 2'd2;

    logic [1:0]       state;
    logic [DIV_W-1:0] tcnt;
    logic [3:0]       bcnt;
    logic [7:0]       sh;
    logic             miso_meta, miso_s;

    // MISO is asynchronous to the PL clock
    always_ff @(posedge clk) begin
        miso_meta <= miso;
        miso_s    <= miso_meta;
    end

    wire tick = (tcnt >= clk_div);

    assign busy = (state != S_IDLE);
    assign csn  = ~cs_assert;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            tcnt    <= '0;
            bcnt    <= '0;
            sh      <= '0;
            sclk    <= 1'b1;   // idle high (CPOL = 1)
            mosi    <= 1'b1;
            done    <= 1'b0;
            rx_byte <= '0;
        end else begin
            done <= 1'b0;
            unique case (state)
                S_IDLE: begin
                    sclk <= 1'b1;
                    tcnt <= '0;
                    bcnt <= '0;
                    if (start) begin
                        sh    <= tx_byte;
                        mosi  <= tx_byte[7];   // setup before first falling edge
                        state <= S_LOW;
                    end
                end

                // Falling edge: drive data
                S_LOW: begin
                    sclk <= 1'b0;
                    mosi <= sh[7];
                    if (tick) begin
                        tcnt  <= '0;
                        state <= S_HIGH;
                    end else begin
                        tcnt <= tcnt + 1'b1;
                    end
                end

                // Rising edge: capture data. Sampling at the end of the high
                // phase guarantees the synchronizer output is settled.
                S_HIGH: begin
                    sclk <= 1'b1;
                    if (tick) begin
                        tcnt <= '0;
                        sh   <= {sh[6:0], miso_s};
                        if (bcnt == 4'd7) begin
                            rx_byte <= {sh[6:0], miso_s};
                            done    <= 1'b1;
                            state   <= S_IDLE;
                        end else begin
                            bcnt  <= bcnt + 1'b1;
                            state <= S_LOW;
                        end
                    end else begin
                        tcnt <= tcnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
