// ---------------------------------------------------------------------------
// tb_imu_gimbal.sv - Self-checking testbench for imu_gimbal_axi
//
// Coverage:
//   1. AXI4-Lite read/write (identification and configuration registers)
//   2. BNO055 startup sequence against the SPI protocol model
//   3. Burst read of measurement data and preprocessing
//   4. Automatic capture of the startup attitude as the setpoint
//   5. Quaternion error computation with the IMU rotated
//   6. PID and stepper output (pulse count and direction)
//   7. New setpoint over AXI: error returns to zero, motors stop
//   8. SPI protocol error path (0xEE response)
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_imu_gimbal;

    // ---- Register map ----------------------------------------------------
    localparam logic [11:0] R_ID         = 12'h000;
    localparam logic [11:0] R_VERSION    = 12'h004;
    localparam logic [11:0] R_CLK_HZ     = 12'h008;
    localparam logic [11:0] R_SCRATCH    = 12'h00C;
    localparam logic [11:0] R_CTRL       = 12'h010;
    localparam logic [11:0] R_STATUS     = 12'h014;
    localparam logic [11:0] R_SAMPLE_CNT = 12'h018;
    localparam logic [11:0] R_ERR_CNT    = 12'h01C;
    localparam logic [11:0] R_IRQ        = 12'h020;
    localparam logic [11:0] R_SP_QW      = 12'h030;
    localparam logic [11:0] R_MEAS_QW    = 12'h040;
    localparam logic [11:0] R_ERR_X      = 12'h050;
    localparam logic [11:0] R_EUL_YAW    = 12'h060;
    localparam logic [11:0] R_GYR_X      = 12'h06C;
    localparam logic [11:0] R_KP_X       = 12'h080;
    localparam logic [11:0] R_OUT_LIM    = 12'h0B0;
    localparam logic [11:0] R_ACC_LIM    = 12'h0B4;
    localparam logic [11:0] R_MOT0_POS   = 12'h0C0;
    localparam logic [11:0] R_MOT0_VEL   = 12'h0CC;
    localparam logic [11:0] R_SAMPLE_DIV = 12'h0E0;
    localparam logic [11:0] R_SPI_DIV    = 12'h0E4;
    localparam logic [11:0] R_INIT_DLY   = 12'h0E8;
    localparam logic [11:0] R_POLL_MAX   = 12'h0EC;
    localparam logic [11:0] R_FILT_CFG   = 12'h0F0;
    localparam logic [11:0] R_HOME_QW    = 12'h100;

    // ---- Clock and reset -------------------------------------------------
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk;              // 100 MHz

    // ---- AXI4-Lite -------------------------------------------------------
    logic [11:0] awaddr = '0;
    logic        awvalid = 1'b0;
    wire         awready;
    logic [31:0] wdata = '0;
    logic [3:0]  wstrb = 4'hF;
    logic        wvalid = 1'b0;
    wire         wready;
    wire  [1:0]  bresp;
    wire         bvalid;
    logic        bready = 1'b0;
    logic [11:0] araddr = '0;
    logic        arvalid = 1'b0;
    wire         arready;
    wire  [31:0] rdata;
    wire  [1:0]  rresp;
    wire         rvalid;
    logic        rready = 1'b0;

    // ---- Device pins -----------------------------------------------------
    wire imu_sclk, imu_mosi, imu_csn, imu_rstn;
    wire imu_miso;
    wire [2:0] mot_step, mot_dir, mot_en_n, drv_ms;
    wire drv_rstn, drv_slpn;
    wire [3:0] led;
    wire irq;

    // ---- Sensor stimuli --------------------------------------------------
    logic signed [15:0] m_gx = 0, m_gy = 0, m_gz = 0;
    logic signed [15:0] m_eh = 0, m_er = 0, m_ep = 0;
    logic signed [15:0] m_qw = 16'sd16384, m_qx = 0, m_qy = 0, m_qz = 0;
    logic [7:0]         m_cal = 8'hFF;
    logic               m_err = 1'b0;
    wire [31:0]         n_reads, n_writes;

    int errors = 0;

    // =====================================================================
    imu_gimbal_axi #(
        .CLK_HZ (100_000_000)
    ) dut (
        .s_axi_aclk    (clk),
        .s_axi_aresetn (rst_n),
        .s_axi_awaddr  (awaddr),
        .s_axi_awprot  (3'b000),
        .s_axi_awvalid (awvalid),
        .s_axi_awready (awready),
        .s_axi_wdata   (wdata),
        .s_axi_wstrb   (wstrb),
        .s_axi_wvalid  (wvalid),
        .s_axi_wready  (wready),
        .s_axi_bresp   (bresp),
        .s_axi_bvalid  (bvalid),
        .s_axi_bready  (bready),
        .s_axi_araddr  (araddr),
        .s_axi_arprot  (3'b000),
        .s_axi_arvalid (arvalid),
        .s_axi_arready (arready),
        .s_axi_rdata   (rdata),
        .s_axi_rresp   (rresp),
        .s_axi_rvalid  (rvalid),
        .s_axi_rready  (rready),

        .imu_sclk (imu_sclk),
        .imu_mosi (imu_mosi),
        .imu_csn  (imu_csn),
        .imu_rstn (imu_rstn),
        .imu_miso (imu_miso),
        .imu_int  (1'b0),

        .mot_step (mot_step),
        .mot_dir  (mot_dir),
        .mot_en_n (mot_en_n),
        .drv_ms   (drv_ms),
        .drv_rstn (drv_rstn),
        .drv_slpn (drv_slpn),
        .led      (led),
        .irq      (irq)
    );

    bno055_spi_model #(.RESP_DELAY(2)) sensor (
        .csn        (imu_csn),
        .sclk       (imu_sclk),
        .mosi       (imu_mosi),
        .miso       (imu_miso),
        .set_gx     (m_gx), .set_gy (m_gy), .set_gz (m_gz),
        .set_eh     (m_eh), .set_er (m_er), .set_ep (m_ep),
        .set_qw     (m_qw), .set_qx (m_qx), .set_qy (m_qy), .set_qz (m_qz),
        .set_calib  (m_cal),
        .inject_err (m_err),
        .n_reads    (n_reads),
        .n_writes   (n_writes)
    );

    // ---- Step counters ---------------------------------------------------
    int step_cnt [0:2];
    initial for (int i = 0; i < 3; i++) step_cnt[i] = 0;
    always @(posedge mot_step[0]) step_cnt[0]++;
    always @(posedge mot_step[1]) step_cnt[1]++;
    always @(posedge mot_step[2]) step_cnt[2]++;

    // =====================================================================
    // AXI4-Lite bus functional model
    // =====================================================================
    task automatic axi_wr(input logic [11:0] a, input logic [31:0] d);
        begin
            @(negedge clk);
            awaddr  = a;  wdata = d; wstrb = 4'hF;
            awvalid = 1'b1; wvalid = 1'b1; bready = 1'b1;
            do @(negedge clk); while (!(awready && wready));
            @(negedge clk);
            awvalid = 1'b0; wvalid = 1'b0;
            while (!bvalid) @(negedge clk);
            @(negedge clk);
            bready = 1'b0;
        end
    endtask

    task automatic axi_rd(input logic [11:0] a, output logic [31:0] d);
        begin
            @(negedge clk);
            araddr = a; arvalid = 1'b1; rready = 1'b1;
            while (!rvalid) @(negedge clk);
            d = rdata;
            @(negedge clk);
            arvalid = 1'b0; rready = 1'b0;
        end
    endtask

    // ---- Check helpers ---------------------------------------------------
    task automatic chk(input string name, input logic cond);
        begin
            if (cond) $display("  [ OK ] %s", name);
            else begin $display("  [FAIL] %s", name); errors++; end
        end
    endtask

    task automatic chk_near(input string name, input int got, input int exp,
                            input int tol);
        int diff;
        begin
            diff = (got > exp) ? (got - exp) : (exp - got);
            if (diff <= tol) $display("  [ OK ] %s (got=%0d exp=%0d)", name, got, exp);
            else begin
                $display("  [FAIL] %s (got=%0d exp=%0d tol=%0d)", name, got, exp, tol);
                errors++;
            end
        end
    endtask

    task automatic wait_samples(input int n);
        logic [31:0] c0, c1;
        begin
            axi_rd(R_SAMPLE_CNT, c0);
            c1 = c0;
            while (c1 < c0 + n) begin
                repeat (200) @(negedge clk);
                axi_rd(R_SAMPLE_CNT, c1);
            end
        end
    endtask

    // =====================================================================
    // Test sequence
    // =====================================================================
    logic [31:0] v, v2;
    int          t0;

    initial begin
        $display("=========================================================");
        $display(" tb_imu_gimbal - BNO055 attitude control, 3-axis gimbal");
        $display("=========================================================");

        repeat (20) @(negedge clk);
        rst_n = 1'b1;
        repeat (20) @(negedge clk);

        // ---- 1. Identification -------------------------------------------
        $display("\n[1] AXI4-Lite basics");
        axi_rd(R_ID, v);       chk("ID reads 'BNO5'",  v == 32'h424E4F35);
        axi_rd(R_VERSION, v);  chk("VERSION is 1.0",   v == 32'h00010000);
        axi_rd(R_CLK_HZ, v);   chk("CLK_HZ is 100 MHz", v == 32'd100_000_000);
        axi_wr(R_SCRATCH, 32'hA5A5_1234);
        axi_rd(R_SCRATCH, v);  chk("SCRATCH read-back", v == 32'hA5A5_1234);

        // ---- 2. Shorten timing for simulation ----------------------------
        $display("\n[2] Configure simulation timing");
        axi_wr(R_INIT_DLY,   32'd2000);      // 20 us instead of 700 ms
        axi_wr(R_SAMPLE_DIV, 32'd4000);      // 40 us sample period
        axi_wr(R_SPI_DIV,    32'd3);         // 12.5 MHz SCLK
        axi_wr(R_POLL_MAX,   32'd64);
        axi_wr(R_ACC_LIM,    32'd2_000_000); // effectively unlimited ramp
        axi_rd(R_SAMPLE_DIV, v); chk("SAMPLE_DIV applied", v == 32'd4000);

        // ---- 3. Sensor startup --------------------------------------------
        $display("\n[3] BNO055 initialization");
        m_qw = 16'sd16384; m_qx = 0; m_qy = 0; m_qz = 0;   // home attitude
        axi_wr(R_CTRL, 32'h0000_0001);                     // IMU_EN

        t0 = 0;
        axi_rd(R_STATUS, v);
        while (!v[0] && t0 < 4000) begin
            repeat (100) @(negedge clk);
            axi_rd(R_STATUS, v);
            t0++;
        end
        chk("IMU_OK asserted",    v[0]);
        chk("INIT_DONE asserted", v[1]);
        $display("       model writes: %0d, reads: %0d", n_writes, n_reads);
        chk("OPR_MODE set to NDOF (0x0C)", sensor.wregs[8'h3D] == 8'h0C);
        chk("UNIT_SEL set to 0x00",        sensor.wregs[8'h3B] == 8'h00);
        chk("PAGE_ID set to 0x00",         sensor.wregs[8'h07] == 8'h00);

        // ---- 4. Measurement data ------------------------------------------
        $display("\n[4] Measurement data and preprocessing");
        m_eh = 16'sd1600;   // 100.0 degrees
        m_er = -16'sd320;   // -20.0 degrees
        m_ep = 16'sd160;    // 10.0 degrees
        m_gx = 16'sd0; m_gy = 16'sd0; m_gz = 16'sd0;
        wait_samples(4);

        axi_rd(R_MEAS_QW,        v); chk_near("MEAS_QW", $signed(v), 16384, 4);
        axi_rd(R_MEAS_QW + 12'd4,v); chk_near("MEAS_QX", $signed(v), 0, 4);
        axi_rd(R_EUL_YAW,        v); chk_near("EUL_YAW",   $signed(v),  1600, 1);
        axi_rd(R_EUL_YAW + 12'd4,v); chk_near("EUL_ROLL",  $signed(v),  -320, 1);
        axi_rd(R_EUL_YAW + 12'd8,v); chk_near("EUL_PITCH", $signed(v),   160, 1);

        // The startup attitude must have been captured automatically
        axi_rd(R_SP_QW,          v); chk_near("SP_QW equals home", $signed(v), 16384, 4);
        axi_rd(R_HOME_QW,        v); chk_near("HOME_QW",           $signed(v), 16384, 4);
        axi_rd(R_ERR_X,          v); chk_near("ERR_X near zero", $signed(v), 0, 8);

        // ---- 5. Gyro filter -----------------------------------------------
        $display("\n[5] Gyro EMA filter (k = 3)");
        m_gx = 16'sd1600;                 // 100 dps
        wait_samples(40);
        axi_rd(R_GYR_X, v);
        chk("GYR_X tracks the input once settled",
            $signed(v) > 1500 && $signed(v) <= 1600);

        // ---- 6. Quaternion error -------------------------------------------
        $display("\n[6] Attitude error for a 30 degree rotation about X");
        m_gx = 0;
        m_qw = 16'sd15826;  m_qx = 16'sd4240;  m_qy = 0;  m_qz = 0;
        wait_samples(4);
        axi_rd(R_ERR_X, v);  chk_near("ERR_X equals -2*qx", $signed(v), -8480, 40);
        axi_rd(R_ERR_X + 12'd4, v); chk_near("ERR_Y near zero", $signed(v), 0, 40);
        axi_rd(R_ERR_X + 12'd8, v); chk_near("ERR_Z near zero", $signed(v), 0, 40);

        // ---- 7. Controller and motors --------------------------------------
        $display("\n[7] PID and stepper output");
        axi_wr(R_KP_X, 32'd13107200);                 // Kp = 200.0
        axi_wr(R_CTRL, 32'h0000_0007);                // IMU_EN | CTRL_EN | MOT_EN
        wait_samples(4);

        axi_rd(R_MOT0_VEL, v);
        chk("motor 0 commanded backwards (u < 0)", $signed(v) < 0);
        chk("DIR 0 is high", mot_dir[0] === 1'b1);
        chk("EN_n 0 is low (driver enabled)", mot_en_n[0] === 1'b0);

        step_cnt[0] = 0;
        repeat (100_000) @(negedge clk);              // 1 ms
        $display("       motor 0 steps in 1 ms: %0d (expected ~20)", step_cnt[0]);
        chk("motor 0 generates steps", step_cnt[0] > 5);
        chk("motor 1 stays idle",      step_cnt[1] < 3);
        chk("motor 2 stays idle",      step_cnt[2] < 3);

        axi_rd(R_MOT0_POS, v);
        chk("motor 0 position is negative", $signed(v) < 0);

        // ---- 8. New setpoint over AXI ---------------------------------------
        $display("\n[8] New setpoint drives the error to zero");
        axi_wr(R_SP_QW + 12'd0, 32'd15826);
        axi_wr(R_SP_QW + 12'd4, 32'd4240);
        axi_wr(R_SP_QW + 12'd8, 32'd0);
        axi_wr(R_SP_QW + 12'd12,32'd0);
        wait_samples(6);

        axi_rd(R_ERR_X, v);      chk_near("ERR_X near zero after setpoint change",
                                          $signed(v), 0, 40);
        axi_rd(R_MOT0_VEL, v);   chk_near("motor command near zero", $signed(v), 0, 40000);
        step_cnt[0] = 0;
        repeat (50_000) @(negedge clk);
        chk("motor 0 stopped", step_cnt[0] < 3);

        // ---- 9. Interrupt ----------------------------------------------------
        $display("\n[9] Interrupt");
        axi_rd(R_IRQ, v);  chk("IRQ_STATUS.NEW_SAMPLE set", v[0]);
        axi_wr(R_IRQ, 32'h1);
        axi_rd(R_IRQ, v);  chk("IRQ_STATUS cleared by write-one-to-clear", v[0] == 1'b0);

        // ---- 10. Error path ---------------------------------------------------
        $display("\n[10] SPI protocol error (0xEE response)");
        axi_rd(R_ERR_CNT, v);
        m_err = 1'b1;
        repeat (200_000) @(negedge clk);
        axi_rd(R_ERR_CNT, v2);
        chk("ERR_CNT increases on sensor errors", v2 > v);
        axi_rd(R_STATUS, v);
        chk("IMU_OK drops after repeated errors", v[0] == 1'b0);
        m_err = 1'b0;

        // ---- Result ------------------------------------------------------------
        $display("\n=========================================================");
        if (errors == 0) $display(" RESULT: ALL TESTS PASSED");
        else             $display(" RESULT: %0d TEST(S) FAILED", errors);
        $display("=========================================================");
        $finish;
    end

    // ---- Watchdog -----------------------------------------------------------
    initial begin
        #20ms;
        $display("\n[FAIL] testbench timeout");
        $finish;
    end

endmodule
