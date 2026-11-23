//-----------------------------------------------------------------------------
// mod_axil_regs.v
// AXI4-Lite control / status register file for the moving object detector.
//
//  0x000 CTRL      rw  [0] soft reset  [1] gray input
//                      [3:2] morphology: 0 none, 1 open, 2 close, 3 close+open
//  0x004 STATUS    ro  [0] label overflow (last frame)  [31:16] frame counter
//  0x008 WIDTH     rw  [10:0]
//  0x00C HEIGHT    rw  [10:0]
//  0x010 SHIFTS    rw  [3:0] a, alpha_w = 2^-a (alpha_k = alpha_w / w_k)
//  0x014 LAMBDA2   rw  [7:0]  Q4.4
//  0x018 T_BG      rw  [11:0] Q0.12
//  0x01C reserved
//  0x020 VAR_INIT  rw  [13:0] integer sigma^2 (new Gaussians)
//  0x024 VAR_MIN   rw  [13:0] integer sigma^2
//  0x028 VAR_MAX   rw  [13:0] integer sigma^2
//  0x02C MIN_AREA  rw  [19:0]
//  0x030 NUM_OBJ   ro
//  0x034 FG_COUNT  ro
//  0x038 ID        ro  0x4D4F4401
//  0x03C CAPS      ro  [7:0] LBL_W  [15:8] OBJ_AW  [31:16] MAX_W
//  0x040 GMM_FMT   ro  [3:0] WB  [7:4] FM  [11:8] VB-8  [15:12] SB  (word lengths)
//  0x200 + 16*i    ro  object i: +0 {maxx,minx} +4 {maxy,miny} +8 area
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none
module mod_axil_regs #(
    parameter MAX_W  = 1024,
    parameter LBL_W  = 7,
    parameter OBJ_AW = 5,
    parameter GMM_WB = 12,
    parameter GMM_FM = 4,
    parameter GMM_VB = 14,
    parameter GMM_SB = 4
) (
    input  wire        clk,
    input  wire        rst,
    // AXI4-Lite slave
    input  wire [9:0]  s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [9:0]  s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,
    // configuration outputs
    output reg         soft_rst,
    output reg         gray_mode,
    output reg  [1:0]  morph_mode,
    output reg  [10:0] width,
    output reg  [10:0] height,
    output reg  [3:0]  alpha_sh,
    output reg  [7:0]  lambda2,
    output reg  [11:0] t_bg,
    output reg  [13:0] var_init,
    output reg  [13:0] var_min,
    output reg  [13:0] var_max,
    output reg  [19:0] min_area,
    // status inputs
    input  wire        overflow,
    input  wire [15:0] frame_cnt,
    input  wire [OBJ_AW:0] num_obj,
    input  wire [19:0] fg_count,
    output wire [OBJ_AW-1:0] obj_idx,
    input  wire [59:0] obj_data
);

    localparam [15:0] P_MAXW  = MAX_W;
    localparam [7:0]  P_OBJAW = OBJ_AW;
    localparam [7:0]  P_LBLW  = LBL_W;
    localparam [3:0]  F_WB = GMM_WB, F_FM = GMM_FM, F_VB = GMM_VB - 8, F_SB = GMM_SB;
    localparam [15:0] P_FMT   = {F_SB, F_VB, F_FM, F_WB};

    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    // ------------------------------------------------------------ write
    // accept AW and W together; the register is written in the handshake
    // cycle and BRESP is issued in the following cycle
    wire wr_go = s_axi_awvalid & s_axi_wvalid & ~s_axi_bvalid
                 & ~s_axi_awready;
    wire wr_hs = s_axi_awready;
    wire [7:0] wa = s_axi_awaddr[9:2];

    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready <= 1'b0; s_axi_wready <= 1'b0; s_axi_bvalid <= 1'b0;
            soft_rst <= 1'b0; gray_mode <= 1'b0; morph_mode <= 2'd3;
            width    <= 11'd640;  height  <= 11'd480;
            alpha_sh <= 4'd7;
            lambda2  <= 8'd100;   t_bg    <= 12'd2867;
            var_init <= 14'd900;  var_min <= 14'd16;    var_max <= 14'd16383;
            min_area <= 20'd64;
        end else begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            if (s_axi_bvalid & s_axi_bready) s_axi_bvalid <= 1'b0;
            if (wr_go) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
            end
            if (wr_hs) begin
                s_axi_bvalid  <= 1'b1;
                case (wa)
                8'h00: {morph_mode, gray_mode, soft_rst} <= s_axi_wdata[3:0];
                8'h02: width    <= s_axi_wdata[10:0];
                8'h03: height   <= s_axi_wdata[10:0];
                8'h04: alpha_sh <= (s_axi_wdata[3:0] > GMM_WB) ? GMM_WB : s_axi_wdata[3:0];
                8'h05: lambda2  <= s_axi_wdata[7:0];
                8'h06: t_bg     <= s_axi_wdata[11:0];
                8'h08: var_init <= s_axi_wdata[13:0];
                8'h09: var_min  <= s_axi_wdata[13:0];
                8'h0A: var_max  <= s_axi_wdata[13:0];
                8'h0B: min_area <= s_axi_wdata[19:0];
                default: ;
                endcase
            end
        end
    end

    // ------------------------------------------------------------ read
    reg [9:0] ra;
    assign obj_idx = ra[OBJ_AW+3:4];

    reg [31:0] rmux;
    always @(*) begin
        rmux = 32'd0;
        if (ra[9]) begin
            case (ra[3:2])
            2'd0: rmux = {6'd0, obj_data[19:10], 6'd0, obj_data[9:0]};
            2'd1: rmux = {6'd0, obj_data[39:30], 6'd0, obj_data[29:20]};
            2'd2: rmux = {12'd0, obj_data[59:40]};
            default: rmux = 32'd0;
            endcase
            if (ra[8:4] >= (1 << OBJ_AW)) rmux = 32'd0;
        end else begin
            case (ra[9:2])
            8'h00: rmux = {28'd0, morph_mode, gray_mode, soft_rst};
            8'h01: rmux = {frame_cnt, 15'd0, overflow};
            8'h02: rmux = {21'd0, width};
            8'h03: rmux = {21'd0, height};
            8'h04: rmux = {28'd0, alpha_sh};
            8'h05: rmux = {24'd0, lambda2};
            8'h06: rmux = {20'd0, t_bg};
            8'h08: rmux = {18'd0, var_init};
            8'h09: rmux = {18'd0, var_min};
            8'h0A: rmux = {18'd0, var_max};
            8'h0B: rmux = {12'd0, min_area};
            8'h0C: rmux = num_obj;
            8'h0D: rmux = {12'd0, fg_count};
            8'h0E: rmux = 32'h4D4F4401;
            8'h0F: rmux = {P_MAXW, P_OBJAW, P_LBLW};
            8'h10: rmux = {16'd0, P_FMT};
            default: rmux = 32'd0;
            endcase
        end
    end

    reg rd_pend;
    always @(posedge clk) begin
        if (rst) begin
            s_axi_arready <= 1'b0; s_axi_rvalid <= 1'b0; s_axi_rdata <= 0;
            ra <= 0; rd_pend <= 1'b0;
        end else begin
            s_axi_arready <= 1'b0;
            if (s_axi_rvalid & s_axi_rready) s_axi_rvalid <= 1'b0;
            if (s_axi_arvalid & ~s_axi_arready & ~s_axi_rvalid & ~rd_pend) begin
                s_axi_arready <= 1'b1;
                ra      <= s_axi_araddr;
                rd_pend <= 1'b1;               // data mux settles next cycle
            end else if (rd_pend) begin
                s_axi_rdata  <= rmux;
                s_axi_rvalid <= 1'b1;
                rd_pend      <= 1'b0;
            end
        end
    end

endmodule
`default_nettype wire
