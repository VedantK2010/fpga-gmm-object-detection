//-----------------------------------------------------------------------------
// mod_top.v
// Real-time moving object detector for Zynq-7000:
//   GMM background identification (Genovese & Napoli formulation of the
//   OpenCV GMM) -> 3x3 morphology -> single-pass CCL
//
// Interfaces (all synchronous to aclk, active-low aresetn):
//   s_axi_ctrl   AXI4-Lite   control / status / object table
//   s_axis_pix   AXI4-Stream 32-bit RGBX pixels       (from DMA MM2S)
//   s_axis_par   AXI4-Stream 128-bit GMM parameters   (from DMA MM2S)
//   m_axis_par   AXI4-Stream 128-bit updated params   (to DMA S2MM)
//   m_axis_mask  AXI4-Stream 8-bit foreground mask    (to DMA S2MM)
// Both output streams assert TLAST on the last pixel of a frame.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none
module mod_top #(
    parameter MAX_W  = 1024,
    parameter LBL_W  = 7,
    parameter OBJ_AW = 5,
    // GMM word lengths (default: 126-bit model word). Paper Table I
    // (Genovese & Napoli): GMM_WB=8, GMM_FM=2, GMM_VB=11, GMM_SB=4
    parameter GMM_WB = 12,
    parameter GMM_FM = 4,
    parameter GMM_VB = 14,
    parameter GMM_SB = 4
) (
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi_ctrl:s_axis_pix:s_axis_par:m_axis_par:m_axis_mask, ASSOCIATED_RESET aresetn" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    input  wire         aclk,
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    input  wire         aresetn,

    // AXI4-Lite
    input  wire [9:0]   s_axi_ctrl_awaddr,
    input  wire [2:0]   s_axi_ctrl_awprot,
    input  wire         s_axi_ctrl_awvalid,
    output wire         s_axi_ctrl_awready,
    input  wire [31:0]  s_axi_ctrl_wdata,
    input  wire [3:0]   s_axi_ctrl_wstrb,
    input  wire         s_axi_ctrl_wvalid,
    output wire         s_axi_ctrl_wready,
    output wire [1:0]   s_axi_ctrl_bresp,
    output wire         s_axi_ctrl_bvalid,
    input  wire         s_axi_ctrl_bready,
    input  wire [9:0]   s_axi_ctrl_araddr,
    input  wire [2:0]   s_axi_ctrl_arprot,
    input  wire         s_axi_ctrl_arvalid,
    output wire         s_axi_ctrl_arready,
    output wire [31:0]  s_axi_ctrl_rdata,
    output wire [1:0]   s_axi_ctrl_rresp,
    output wire         s_axi_ctrl_rvalid,
    input  wire         s_axi_ctrl_rready,

    // pixel input
    input  wire [31:0]  s_axis_pix_tdata,
    input  wire         s_axis_pix_tvalid,
    output wire         s_axis_pix_tready,
    input  wire         s_axis_pix_tlast,

    // parameter input
    input  wire [127:0] s_axis_par_tdata,
    input  wire         s_axis_par_tvalid,
    output wire         s_axis_par_tready,
    input  wire         s_axis_par_tlast,

    // parameter output
    output wire [127:0] m_axis_par_tdata,
    output wire [15:0]  m_axis_par_tkeep,
    output wire         m_axis_par_tvalid,
    input  wire         m_axis_par_tready,
    output wire         m_axis_par_tlast,

    // mask output
    output wire [7:0]   m_axis_mask_tdata,
    output wire [0:0]   m_axis_mask_tkeep,
    output wire         m_axis_mask_tvalid,
    input  wire         m_axis_mask_tready,
    output wire         m_axis_mask_tlast,

    // level interrupt-style pulse: one clock per processed frame
    output reg          frame_done
);

    // ------------------------------------------------------------ registers
    wire        soft_rst, gray_mode;
    wire [1:0]  morph_mode;
    wire [10:0] width, height;
    wire [3:0]  alpha_sh;
    wire [7:0]  lambda2;
    wire [11:0] t_bg;
    wire [13:0] var_init, var_min, var_max;
    wire [19:0] min_area;
    wire        overflow;
    wire [15:0] frame_cnt;
    wire [OBJ_AW:0] num_obj;
    wire [19:0] fg_count;
    wire [OBJ_AW-1:0] obj_idx;
    wire [59:0] obj_data;

    reg  rst_r;
    always @(posedge aclk) rst_r <= ~aresetn;
    wire rst_bus  = rst_r;
    wire rst_core = rst_r | soft_rst;

    mod_axil_regs #(.MAX_W(MAX_W), .LBL_W(LBL_W), .OBJ_AW(OBJ_AW),
                    .GMM_WB(GMM_WB), .GMM_FM(GMM_FM), .GMM_VB(GMM_VB), .GMM_SB(GMM_SB)) u_regs (
        .clk(aclk), .rst(rst_bus),
        .s_axi_awaddr(s_axi_ctrl_awaddr), .s_axi_awprot(s_axi_ctrl_awprot),
        .s_axi_awvalid(s_axi_ctrl_awvalid), .s_axi_awready(s_axi_ctrl_awready),
        .s_axi_wdata(s_axi_ctrl_wdata), .s_axi_wstrb(s_axi_ctrl_wstrb),
        .s_axi_wvalid(s_axi_ctrl_wvalid), .s_axi_wready(s_axi_ctrl_wready),
        .s_axi_bresp(s_axi_ctrl_bresp), .s_axi_bvalid(s_axi_ctrl_bvalid),
        .s_axi_bready(s_axi_ctrl_bready),
        .s_axi_araddr(s_axi_ctrl_araddr), .s_axi_arprot(s_axi_ctrl_arprot),
        .s_axi_arvalid(s_axi_ctrl_arvalid), .s_axi_arready(s_axi_ctrl_arready),
        .s_axi_rdata(s_axi_ctrl_rdata), .s_axi_rresp(s_axi_ctrl_rresp),
        .s_axi_rvalid(s_axi_ctrl_rvalid), .s_axi_rready(s_axi_ctrl_rready),
        .soft_rst(soft_rst), .gray_mode(gray_mode), .morph_mode(morph_mode),
        .width(width), .height(height), .alpha_sh(alpha_sh),
        .lambda2(lambda2), .t_bg(t_bg), .var_init(var_init),
        .var_min(var_min), .var_max(var_max), .min_area(min_area),
        .overflow(overflow), .frame_cnt(frame_cnt), .num_obj(num_obj),
        .fg_count(fg_count), .obj_idx(obj_idx), .obj_data(obj_data)
    );

    // ------------------------------------------------------------ join
    wire g_in_ready;
    wire g_in_valid = s_axis_pix_tvalid & s_axis_par_tvalid;
    assign s_axis_pix_tready = g_in_ready & s_axis_par_tvalid;
    assign s_axis_par_tready = g_in_ready & s_axis_pix_tvalid;

    // ------------------------------------------------------------ GMM
    wire         g_out_valid, g_out_ready, g_out_fg;
    wire [127:0] g_out_par;

    gmm_core #(.WB(GMM_WB), .FM(GMM_FM), .VB(GMM_VB), .SB(GMM_SB)) u_gmm (
        .clk(aclk), .rst(rst_core),
        .alpha_sh(alpha_sh), .lambda2(lambda2),
        .t_bg(t_bg), .var_init(var_init),
        .var_min(var_min), .var_max(var_max), .gray_mode(gray_mode),
        .in_valid(g_in_valid), .in_ready(g_in_ready),
        .in_pix(s_axis_pix_tdata), .in_par(s_axis_par_tdata),
        .out_valid(g_out_valid), .out_ready(g_out_ready),
        .out_par(g_out_par), .out_fg(g_out_fg)
    );

    // ------------------------------------------------------------ fork
    reg         pv, fv, pl, fd;
    reg [127:0] pd;
    wire        f_ready;
    wire        p_free = ~pv | m_axis_par_tready;
    wire        f_free = ~fv | f_ready;
    assign g_out_ready = p_free & f_free;
    wire        g_hs = g_out_valid & g_out_ready;

    reg [10:0] gx, gy;
    wire g_last = (gx == width - 11'd1) & (gy == height - 11'd1);

    always @(posedge aclk) begin
        if (rst_core) begin
            pv <= 1'b0; fv <= 1'b0; pl <= 1'b0; fd <= 1'b0; pd <= 0;
            gx <= 0; gy <= 0;
        end else begin
            if (m_axis_par_tready) pv <= 1'b0;
            if (f_ready)           fv <= 1'b0;
            if (g_hs) begin
                pv <= 1'b1; pd <= g_out_par; pl <= g_last;
                fv <= 1'b1; fd <= g_out_fg;
                if (gx == width - 11'd1) begin
                    gx <= 0;
                    gy <= (gy == height - 11'd1) ? 11'd0 : gy + 11'd1;
                end else
                    gx <= gx + 11'd1;
            end
        end
    end

    assign m_axis_par_tdata  = pd;
    assign m_axis_par_tkeep  = 16'hFFFF;
    assign m_axis_par_tvalid = pv;
    assign m_axis_par_tlast  = pl;

    // ------------------------------------------------------------ morphology
    wire cc_in_ready, mf_valid, mf_bit;

    // register slice: cuts the ready path DMA -> CCL -> morphology -> GMM
    wire sk_valid, sk_ready, sk_bit;
    axis_skid #(.W(1)) u_skid (
        .clk(aclk), .rst(rst_core),
        .s_valid(fv), .s_ready(f_ready), .s_data(fd),
        .m_valid(sk_valid), .m_ready(sk_ready), .m_data(sk_bit)
    );

    morph_chain #(.MAX_W(MAX_W)) u_morph (
        .clk(aclk), .rst(rst_core), .mode(morph_mode),
        .width(width), .height(height),
        .in_valid(sk_valid), .in_ready(sk_ready), .in_bit(sk_bit),
        .out_valid(mf_valid), .out_ready(cc_in_ready), .out_bit(mf_bit)
    );

    // ------------------------------------------------------------ CCL
    wire cc_in_valid = mf_valid;
    wire cc_in_bit   = mf_bit;
    wire [7:0] mk_data;
    wire       mk_valid, mk_last;

    ccl_single_pass #(.MAX_W(MAX_W), .LBL_W(LBL_W), .OBJ_AW(OBJ_AW)) u_ccl (
        .clk(aclk), .rst(rst_core), .width(width), .height(height),
        .min_area(min_area),
        .in_valid(cc_in_valid), .in_ready(cc_in_ready), .in_bit(cc_in_bit),
        .out_valid(mk_valid), .out_ready(m_axis_mask_tready),
        .out_data(mk_data), .out_last(mk_last),
        .obj_rd_idx(obj_idx), .obj_rd_data(obj_data),
        .num_obj(num_obj), .fg_count(fg_count), .overflow(overflow),
        .frame_cnt(frame_cnt)
    );

    assign m_axis_mask_tdata  = mk_data;
    assign m_axis_mask_tkeep  = 1'b1;
    assign m_axis_mask_tvalid = mk_valid;
    assign m_axis_mask_tlast  = mk_last;

    reg [15:0] fc_d;
    always @(posedge aclk) begin
        fc_d       <= frame_cnt;
        frame_done <= (fc_d != frame_cnt);
    end

endmodule
`default_nettype wire
