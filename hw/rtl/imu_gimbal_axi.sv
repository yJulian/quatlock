// ---------------------------------------------------------------------------
// imu_gimbal_axi.sv - AXI4-Lite IP: BNO055 preprocessing + 3-axis attitude controller
//
//  PL-Seite (vollstaendig in Hardware, ohne CPU-Beteiligung im Regelkreis):
//     BNO055 (SPI) -> preprocessing -> quaternion error -> 3x PID
//                  -> 3x STEP/DIR-Generator
//
//  PS-Seite (Software):
//     AXI4-Lite register file: attitude setpoint, controller gains,
//     telemetry, enables. The control loop keeps running even when the
//     software does nothing.
//
//  Register map: see README.md and sw/include/gimbal_regs.h
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none

module imu_gimbal_axi #(
    parameter int CLK_HZ             = 100_000_000,
    parameter int C_S_AXI_DATA_WIDTH = 32,
    parameter int C_S_AXI_ADDR_WIDTH = 12
) (
    // ---- AXI4-Lite Slave -------------------------------------------------
    input  wire                                s_axi_aclk,
    input  wire                                s_axi_aresetn,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       s_axi_awaddr,
    input  wire [2:0]                          s_axi_awprot,
    input  wire                                s_axi_awvalid,
    output wire                                s_axi_awready,

    input  wire [C_S_AXI_DATA_WIDTH-1:0]       s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]   s_axi_wstrb,
    input  wire                                s_axi_wvalid,
    output wire                                s_axi_wready,

    output wire [1:0]                          s_axi_bresp,
    output wire                                s_axi_bvalid,
    input  wire                                s_axi_bready,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       s_axi_araddr,
    input  wire [2:0]                          s_axi_arprot,
    input  wire                                s_axi_arvalid,
    output wire                                s_axi_arready,

    output wire [C_S_AXI_DATA_WIDTH-1:0]       s_axi_rdata,
    output wire [1:0]                          s_axi_rresp,
    output wire                                s_axi_rvalid,
    input  wire                                s_axi_rready,

    // ---- BNO055 (SPI) ----------------------------------------------------
    output wire                                imu_sclk,
    output wire                                imu_mosi,
    output wire                                imu_csn,
    output wire                                imu_rstn,
    input  wire                                imu_miso,
    input  wire                                imu_int,

    // ---- Stepper motors --------------------------------------------------
    output wire [2:0]                          mot_step,
    output wire [2:0]                          mot_dir,
    output wire [2:0]                          mot_en_n,
    output wire [2:0]                          drv_ms,
    output wire                                drv_rstn,
    output wire                                drv_slpn,

    // ---- Status ----------------------------------------------------------
    output wire [3:0]                          led,
    output wire                                irq
);

    localparam logic [31:0] MAGIC_ID = 32'h424E_4F35;   // "BNO5"
    localparam logic [31:0] VERSION  = 32'h0001_0000;   // 1.0

    wire clk   = s_axi_aclk;
    wire rst_n = s_axi_aresetn;

    // =====================================================================
    // Registers
    // =====================================================================
    logic [31:0] scratch_r;
    logic [13:0] ctrl_r;
    logic [31:0] irq_stat_r;

    logic signed [15:0] sp_q  [0:3];   // setpoint quaternion w,x,y,z (Q1.14)
    logic signed [15:0] home_q[0:3];   // attitude captured at startup

    logic signed [31:0] kp_r  [0:2];
    logic signed [31:0] ki_r  [0:2];
    logic signed [31:0] kd_r  [0:2];
    logic        [31:0] ilim_r[0:2];

    logic [31:0] out_lim_r;
    logic [31:0] acc_lim_r;
    logic [15:0] step_width_r;
    logic [4:0]  ms_cfg_r;

    logic [31:0] sample_div_r;
    logic [15:0] spi_div_r;
    logic [31:0] init_dly_r;
    logic [15:0] poll_max_r;
    logic [7:0]  filt_cfg_r;
    logic [4:0]  led_cfg_r;

    // Single-shot pulses decoded from CTRL
    logic capture_req;
    logic iclear_pulse;
    logic zero_pos_pulse;

    wire imu_en_w      = ctrl_r[0];
    wire ctrl_en_w     = ctrl_r[1];
    wire mot_en_w      = ctrl_r[2];
    wire d_from_gyro_w = ctrl_r[5];
    wire irq_en_w      = ctrl_r[6];
    wire [2:0] dir_inv_w = ctrl_r[10:8];

    // ---- AXI handshake (declared early: several processes reference it)
    logic axi_awready, axi_wready, axi_bvalid, axi_arready, axi_rvalid;
    logic [C_S_AXI_ADDR_WIDTH-1:0] axi_araddr;
    logic [31:0] axi_rdata;

    wire        wr_en    = axi_awready & s_axi_awvalid & axi_wready & s_axi_wvalid;
    wire [9:0]  wr_idx   = s_axi_awaddr[11:2];
    wire [9:0]  rd_idx_w = axi_araddr[11:2];

    wire ctrl_wr = wr_en && (wr_idx == 10'd4);
    wire irq_w1c = wr_en && (wr_idx == 10'd8);

    // Setpoint write access (collision with capture_req resolved below)
    wire        axi_sp_we  = wr_en && (wr_idx >= 10'd12) && (wr_idx <= 10'd15);
    wire [1:0]  axi_sp_idx = wr_idx[1:0];
    wire signed [15:0] axi_sp_val = s_axi_wdata[15:0];

    // =====================================================================
    // IMU sequencer
    // =====================================================================
    wire         imu_ok, init_done;
    wire [7:0]   last_err, calib_stat, seq_dbg;
    wire [31:0]  sample_cnt, err_cnt;
    wire [159:0] sample_data;
    wire         sample_valid;

    wire [15:0] spi_div_eff = (spi_div_r < 16'd2) ? 16'd2 : spi_div_r;

    bno055_seq u_seq (
        .clk          (clk),
        .rst_n        (rst_n),
        .imu_en       (imu_en_w),
        .spi_div      (spi_div_eff),
        .poll_max     (poll_max_r),
        .init_dly     (init_dly_r),
        .sample_div   (sample_div_r),
        .imu_ok       (imu_ok),
        .init_done    (init_done),
        .last_err     (last_err),
        .calib_stat   (calib_stat),
        .sample_cnt   (sample_cnt),
        .err_cnt      (err_cnt),
        .sample_data  (sample_data),
        .sample_valid (sample_valid),
        .dbg_state    (seq_dbg),
        .imu_sclk     (imu_sclk),
        .imu_mosi     (imu_mosi),
        .imu_csn      (imu_csn),
        .imu_rstn     (imu_rstn),
        .imu_miso     (imu_miso)
    );

    // =====================================================================
    // Preprocessing
    // =====================================================================
    wire signed [15:0] gyr_x, gyr_y, gyr_z;
    wire signed [15:0] eul_yaw, eul_roll, eul_pitch;
    wire signed [15:0] q_w, q_x, q_y, q_z;
    wire               pre_valid;

    imu_preproc u_pre (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_data  (sample_data),
        .sample_valid (sample_valid),
        .gyr_shift    (filt_cfg_r[3:0]),
        .quat_shift   (filt_cfg_r[7:4]),
        .gyr_x        (gyr_x),
        .gyr_y        (gyr_y),
        .gyr_z        (gyr_z),
        .eul_yaw      (eul_yaw),
        .eul_roll     (eul_roll),
        .eul_pitch    (eul_pitch),
        .q_w          (q_w),
        .q_x          (q_x),
        .q_y          (q_y),
        .q_z          (q_z),
        .valid        (pre_valid)
    );

    // =====================================================================
    // Quaternion error
    // =====================================================================
    wire signed [17:0] err_x, err_y, err_z;
    wire signed [15:0] eq_w;
    wire               err_valid;

    quat_err u_qerr (
        .clk       (clk),
        .rst_n     (rst_n),
        .sp_w      (sp_q[0]), .sp_x (sp_q[1]), .sp_y (sp_q[2]), .sp_z (sp_q[3]),
        .q_w       (q_w),     .q_x  (q_x),     .q_y  (q_y),     .q_z  (q_z),
        .in_valid  (pre_valid),
        .err_x     (err_x),
        .err_y     (err_y),
        .err_z     (err_z),
        .eq_w      (eq_w),
        .out_valid (err_valid)
    );

    // =====================================================================
    // Controllers and motors
    // =====================================================================
    wire signed [17:0] err_v [0:2];
    wire signed [15:0] gyr_v [0:2];
    assign err_v[0] = err_x;  assign err_v[1] = err_y;  assign err_v[2] = err_z;
    assign gyr_v[0] = gyr_x;  assign gyr_v[1] = gyr_y;  assign gyr_v[2] = gyr_z;

    wire signed [31:0] u_v      [0:2];
    wire signed [31:0] mot_pos  [0:2];
    wire signed [31:0] mot_vel  [0:2];

    genvar gi;
    generate
        for (gi = 0; gi < 3; gi = gi + 1) begin : g_axis
            pid_axis u_pid (
                .clk         (clk),
                .rst_n       (rst_n),
                .tick        (err_valid),
                .enable      (ctrl_en_w && imu_ok),
                .iclear      (iclear_pulse),
                .d_from_gyro (d_from_gyro_w),
                .err         (err_v[gi]),
                .gyr         (gyr_v[gi]),
                .kp          (kp_r[gi]),
                .ki          (ki_r[gi]),
                .kd          (kd_r[gi]),
                .i_lim       (ilim_r[gi]),
                .o_lim       (out_lim_r),
                .u           (u_v[gi])
            );

            stepper_drv u_mot (
                .clk        (clk),
                .rst_n      (rst_n),
                .tick       (err_valid),
                .vel_cmd    (u_v[gi]),
                .acc_lim    (acc_lim_r),
                .max_rate   (out_lim_r),
                .step_width (step_width_r),
                .enable     (mot_en_w),
                .invert_dir (dir_inv_w[gi]),
                .zero_pos   (zero_pos_pulse),
                .step       (mot_step[gi]),
                .dir        (mot_dir[gi]),
                .en_n       (mot_en_n[gi]),
                .cur_vel    (mot_vel[gi]),
                .position   (mot_pos[gi])
            );
        end
    endgenerate

    assign drv_ms   = ms_cfg_r[2:0];
    assign drv_rstn = ms_cfg_r[3];
    assign drv_slpn = ms_cfg_r[4];

    // =====================================================================
    // Setpoint capture: when the IMU is enabled, the first valid attitude
    // becomes the home attitude and therefore the default setpoint.
    // =====================================================================
    logic imu_en_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imu_en_d    <= 1'b0;
            capture_req <= 1'b1;      // first sample after reset defines home
            sp_q[0] <= 16'sd16384; sp_q[1] <= '0; sp_q[2] <= '0; sp_q[3] <= '0;
            home_q[0] <= 16'sd16384; home_q[1] <= '0; home_q[2] <= '0; home_q[3] <= '0;
        end else begin
            imu_en_d <= imu_en_w;
            if (imu_en_w && !imu_en_d) capture_req <= 1'b1;

            if (capture_req && pre_valid) begin
                capture_req <= 1'b0;
                sp_q[0]   <= q_w;  sp_q[1]   <= q_x;  sp_q[2]   <= q_y;  sp_q[3]   <= q_z;
                home_q[0] <= q_w;  home_q[1] <= q_x;  home_q[2] <= q_y;  home_q[3] <= q_z;
            end else if (axi_sp_we) begin
                sp_q[axi_sp_idx] <= axi_sp_val;
            end

            if (ctrl_wr && s_axi_wdata[3]) capture_req <= 1'b1;
        end
    end

    // =====================================================================
    // BNO055 INT pin: synchronized and exposed in STATUS only. NDOF provides
    // no data-ready interrupt, so the pin stays available for user-configured
    // sensor interrupts.
    // =====================================================================
    logic imu_int_meta, imu_int_s;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imu_int_meta <= 1'b0;
            imu_int_s    <= 1'b0;
        end else begin
            imu_int_meta <= imu_int;
            imu_int_s    <= imu_int_meta;
        end
    end

    // =====================================================================
    // Interrupt
    // =====================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            irq_stat_r <= '0;
        end else begin
            if (pre_valid) irq_stat_r[0] <= 1'b1;
            if (irq_w1c)   irq_stat_r    <= irq_stat_r & ~s_axi_wdata;
        end
    end
    assign irq = irq_en_w & (|irq_stat_r);

    // =====================================================================
    // LEDs
    // =====================================================================
    logic [24:0] hb;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) hb <= '0;
        else        hb <= hb + 1'b1;
    end
    assign led = led_cfg_r[4] ? led_cfg_r[3:0]
                              : {hb[24], ctrl_en_w, init_done, imu_ok};

    // =====================================================================
    // AXI4-Lite slave
    // =====================================================================
    assign s_axi_awready = axi_awready;
    assign s_axi_wready  = axi_wready;
    assign s_axi_bvalid  = axi_bvalid;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_arready = axi_arready;
    assign s_axi_rvalid  = axi_rvalid;
    assign s_axi_rresp   = 2'b00;
    assign s_axi_rdata   = axi_rdata;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_awready <= 1'b0;
            axi_wready  <= 1'b0;
            axi_bvalid  <= 1'b0;

            scratch_r    <= '0;
            ctrl_r       <= '0;
            iclear_pulse <= 1'b0;
            zero_pos_pulse <= 1'b0;

            kp_r[0] <= 32'sd13107200; ki_r[0] <= '0; kd_r[0] <= '0; ilim_r[0] <= 32'd200000;
            kp_r[1] <= 32'sd13107200; ki_r[1] <= '0; kd_r[1] <= '0; ilim_r[1] <= 32'd200000;
            kp_r[2] <= 32'sd13107200; ki_r[2] <= '0; kd_r[2] <= '0; ilim_r[2] <= 32'd200000;

            out_lim_r    <= 32'd859000;      // ~20 kHz step rate at 100 MHz
            acc_lim_r    <= 32'd21500;       // ~500 steps/s per control tick
            step_width_r <= 16'd200;         // 2 us
            ms_cfg_r     <= 5'b11111;        // MS = 1/16, RESET/SLEEP inactive

            sample_div_r <= 32'd1_000_000;   // 100 Hz at 100 MHz
            spi_div_r    <= 16'd49;          // 1 MHz SCLK at 100 MHz
            init_dly_r   <= 32'd70_000_000;  // 700 ms power-on reset
            poll_max_r   <= 16'd64;
            filt_cfg_r   <= 8'h03;           // gyro EMA k=3, quaternion EMA off
            led_cfg_r    <= '0;
        end else begin
            iclear_pulse   <= 1'b0;
            zero_pos_pulse <= 1'b0;

            // Write-Handshake
            if (!axi_awready && s_axi_awvalid && s_axi_wvalid && !axi_bvalid) begin
                axi_awready <= 1'b1;
                axi_wready  <= 1'b1;
            end else begin
                axi_awready <= 1'b0;
                axi_wready  <= 1'b0;
            end

            if (wr_en)                       axi_bvalid <= 1'b1;
            else if (axi_bvalid && s_axi_bready) axi_bvalid <= 1'b0;

            if (wr_en) begin
                case (wr_idx)
                    10'd3:  scratch_r <= s_axi_wdata;
                    10'd4:  begin
                                ctrl_r         <= s_axi_wdata[13:0];
                                iclear_pulse   <= s_axi_wdata[4];
                                zero_pos_pulse <= s_axi_wdata[7];
                            end
                    // 12..15 setpoint quaternion: handled in its own process
                    10'd32: kp_r[0]   <= s_axi_wdata;
                    10'd33: ki_r[0]   <= s_axi_wdata;
                    10'd34: kd_r[0]   <= s_axi_wdata;
                    10'd35: ilim_r[0] <= s_axi_wdata;
                    10'd36: kp_r[1]   <= s_axi_wdata;
                    10'd37: ki_r[1]   <= s_axi_wdata;
                    10'd38: kd_r[1]   <= s_axi_wdata;
                    10'd39: ilim_r[1] <= s_axi_wdata;
                    10'd40: kp_r[2]   <= s_axi_wdata;
                    10'd41: ki_r[2]   <= s_axi_wdata;
                    10'd42: kd_r[2]   <= s_axi_wdata;
                    10'd43: ilim_r[2] <= s_axi_wdata;
                    10'd44: out_lim_r    <= s_axi_wdata;
                    10'd45: acc_lim_r    <= s_axi_wdata;
                    10'd46: step_width_r <= s_axi_wdata[15:0];
                    10'd47: ms_cfg_r     <= s_axi_wdata[4:0];
                    10'd56: sample_div_r <= s_axi_wdata;
                    10'd57: spi_div_r    <= s_axi_wdata[15:0];
                    10'd58: init_dly_r   <= s_axi_wdata;
                    10'd59: poll_max_r   <= s_axi_wdata[15:0];
                    10'd60: filt_cfg_r   <= s_axi_wdata[7:0];
                    10'd61: led_cfg_r    <= s_axi_wdata[4:0];
                    default: ;
                endcase
            end
        end
    end

    // ---- Read multiplexer ------------------------------------------------
    function automatic logic [31:0] sext16(input logic signed [15:0] v);
        sext16 = 32'(v);
    endfunction

    logic [31:0] rd_mux;
    always_comb begin
        case (rd_idx_w)
            10'd0:  rd_mux = MAGIC_ID;
            10'd1:  rd_mux = VERSION;
            10'd2:  rd_mux = CLK_HZ;
            10'd3:  rd_mux = scratch_r;
            10'd4:  rd_mux = {18'd0, ctrl_r};
            10'd5:  rd_mux = {seq_dbg, calib_stat, last_err,
                              4'd0, imu_int_s, |err_cnt, init_done, imu_ok};
            10'd6:  rd_mux = sample_cnt;
            10'd7:  rd_mux = err_cnt;
            10'd8:  rd_mux = irq_stat_r;

            10'd12: rd_mux = sext16(sp_q[0]);
            10'd13: rd_mux = sext16(sp_q[1]);
            10'd14: rd_mux = sext16(sp_q[2]);
            10'd15: rd_mux = sext16(sp_q[3]);

            10'd16: rd_mux = sext16(q_w);
            10'd17: rd_mux = sext16(q_x);
            10'd18: rd_mux = sext16(q_y);
            10'd19: rd_mux = sext16(q_z);

            10'd20: rd_mux = 32'(err_x);
            10'd21: rd_mux = 32'(err_y);
            10'd22: rd_mux = 32'(err_z);
            10'd23: rd_mux = sext16(eq_w);

            10'd24: rd_mux = sext16(eul_yaw);
            10'd25: rd_mux = sext16(eul_roll);
            10'd26: rd_mux = sext16(eul_pitch);
            10'd27: rd_mux = sext16(gyr_x);
            10'd28: rd_mux = sext16(gyr_y);
            10'd29: rd_mux = sext16(gyr_z);

            10'd32: rd_mux = kp_r[0];
            10'd33: rd_mux = ki_r[0];
            10'd34: rd_mux = kd_r[0];
            10'd35: rd_mux = ilim_r[0];
            10'd36: rd_mux = kp_r[1];
            10'd37: rd_mux = ki_r[1];
            10'd38: rd_mux = kd_r[1];
            10'd39: rd_mux = ilim_r[1];
            10'd40: rd_mux = kp_r[2];
            10'd41: rd_mux = ki_r[2];
            10'd42: rd_mux = kd_r[2];
            10'd43: rd_mux = ilim_r[2];

            10'd44: rd_mux = out_lim_r;
            10'd45: rd_mux = acc_lim_r;
            10'd46: rd_mux = {16'd0, step_width_r};
            10'd47: rd_mux = {27'd0, ms_cfg_r};

            10'd48: rd_mux = mot_pos[0];
            10'd49: rd_mux = mot_pos[1];
            10'd50: rd_mux = mot_pos[2];
            10'd51: rd_mux = mot_vel[0];
            10'd52: rd_mux = mot_vel[1];
            10'd53: rd_mux = mot_vel[2];

            10'd56: rd_mux = sample_div_r;
            10'd57: rd_mux = {16'd0, spi_div_r};
            10'd58: rd_mux = init_dly_r;
            10'd59: rd_mux = {16'd0, poll_max_r};
            10'd60: rd_mux = {24'd0, filt_cfg_r};
            10'd61: rd_mux = {27'd0, led_cfg_r};

            10'd64: rd_mux = sext16(home_q[0]);
            10'd65: rd_mux = sext16(home_q[1]);
            10'd66: rd_mux = sext16(home_q[2]);
            10'd67: rd_mux = sext16(home_q[3]);

            default: rd_mux = 32'hDEAD_BEEF;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_arready <= 1'b0;
            axi_rvalid  <= 1'b0;
            axi_araddr  <= '0;
            axi_rdata   <= '0;
        end else begin
            if (!axi_arready && s_axi_arvalid) begin
                axi_arready <= 1'b1;
                axi_araddr  <= s_axi_araddr;
            end else begin
                axi_arready <= 1'b0;
            end

            if (axi_arready && s_axi_arvalid && !axi_rvalid) begin
                axi_rvalid <= 1'b1;
                axi_rdata  <= rd_mux;
            end else if (axi_rvalid && s_axi_rready) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

    // AXI4-Lite: awprot/arprot are ignored, and wstrb is always 0xF for
    // 32-bit registers.
    wire unused_ok = &{1'b0, s_axi_awprot, s_axi_arprot, s_axi_wstrb, 1'b0};

endmodule

`default_nettype wire
