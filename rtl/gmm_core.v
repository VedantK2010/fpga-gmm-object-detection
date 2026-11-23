//-----------------------------------------------------------------------------
// gmm_core.v
// Pixel-parallel GMM background identification (K = 3) following the
// hardware-oriented formulation of the OpenCV GMM by M. Genovese and
// E. Napoli, "ASIC and FPGA Implementation of the Gaussian Mixture Model
// Algorithm for Real-Time Segmentation of High Definition Video",
// IEEE TVLSI 22(3), 2014.
//
//   * Gaussian parameters: weight w, mean mu, variance s2, matchsum
//   * alpha_w = 2^-a, alpha_k = alpha_w / w_k rounded to a power of two
//     (log2 look-up on w_k)                                 (paper eq. 4,12)
//   * order parameter IF_k = s2_k * 2^(2(a - s_k)) = s2_k / w_k^2 (eq. 13)
//   * match (x - mu)^2 < lambda^2 s2                          (eq. 15)
//   * update of the matched Gaussian / decay of the others    (eq. 3, 6)
//   * no match: Gaussian with largest IF <- (1/msumtot, x, v_init, 1) (eq. 7)
//   * background: matched Gaussian within the first B in fitness order (eq. 8)
//
// One pixel per clock, 5 register stages, global stall (ce) on back-pressure.
//   S1 : RGB -> gray, unpack
//   S2 : x - mu, (x - mu)^2, lambda^2 s2           (multipliers),
//        L = round(log2 w) by constant comparisons
//   S3 : match, learning-rate shift s_k, inverse fitness IF_k,
//        (x - mu)^2 - s2
//   S4 : ordering, matched / replaced Gaussian, Bg test, candidate updates
//   S5 : variance add + clamp, output selection, pack
//
// Word lengths (parameters; Gaussian k at bits [GW*k +: GW] as
// {matchsum, s2, mu, w}):
//   WB weight bits  (w = code / 2^WB)        paper Table I: 8
//   FM mean fraction bits (mu = code / 2^FM)  paper Table I: 2  (MB = 8+FM)
//   VB variance bits (s2 = code * 2^(14-VB))  paper Table I: 11
//   SB matchsum bits                          paper Table I: 4
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none
module gmm_core #(
    parameter WB = 12,
    parameter FM = 4,
    parameter VB = 14,
    parameter SB = 4
) (
    input  wire         clk,
    input  wire         rst,
    // configuration (static during a frame)
    input  wire [3:0]   alpha_sh,     // a, alpha_w = 2^-a  (a <= WB)
    input  wire [7:0]   lambda2,      // Q4.4
    input  wire [11:0]  t_bg,         // Q0.12
    input  wire [13:0]  var_init,     // integer sigma^2
    input  wire [13:0]  var_min,
    input  wire [13:0]  var_max,
    input  wire         gray_mode,    // 1: use byte0 as luminance
    // input (pixel + model)
    input  wire         in_valid,
    output wire         in_ready,
    input  wire [31:0]  in_pix,       // {X, B, G, R}
    input  wire [127:0] in_par,
    // output (updated model + foreground flag)
    output wire         out_valid,
    input  wire         out_ready,
    output wire [127:0] out_par,
    output wire         out_fg
);

    localparam MB   = 8 + FM;
    localparam VS   = 14 - VB;                 // s2 = code << VS
    localparam E    = VS + 2 * FM;             // (x-mu)^2 code >> E = s2 code
    localparam GW   = WB + MB + VB + SB;
    localparam PW   = 3 * GW;
    localparam IFW  = VB + 24;                 // a <= 12
    localparam CW   = ((2*MB + 4) > (8 + VB + E)) ? (2*MB + 4) : (8 + VB + E);
    localparam [WB-1:0] WMAX = {WB{1'b1}};
    localparam [SB-1:0] SMAX = {SB{1'b1}};

    // 1/n in weight units, rounded (paper eq. 7: w = 1/msumtot)
    function [WB-1:0] recip;
        input [SB:0] n;
        integer i, t;
        begin
            recip = WMAX;
            for (i = 1; i < (1 << (SB + 1)); i = i + 1)
                if (n == i) begin
                    t = ((1 << WB) + i / 2) / i;
                    recip = (t > (1 << WB) - 1) ? WMAX : t;
                end
        end
    endfunction

    reg v1, v2, v3, v4, v5;
    wire ce = ~v5 | out_ready;
    assign in_ready  = ce;
    assign out_valid = v5;

    // ======================================================== S1
    wire [16:0] y_sum = 17'd77  * in_pix[7:0]
                      + 17'd150 * in_pix[15:8]
                      + 17'd29  * in_pix[23:16];
    wire [7:0]  gray  = gray_mode ? in_pix[7:0] : y_sum[15:8];

    reg  [7:0]    s1_x;
    reg  [PW-1:0] s1_g;
    always @(posedge clk) begin
        if (rst) v1 <= 1'b0;
        else if (ce) v1 <= in_valid;
        if (ce) begin
            s1_x <= gray;
            s1_g <= in_par[PW-1:0];
        end
    end

    // ======================================================== S2
    reg  [7:0]      s2_x;
    reg  [PW-1:0]   s2_g;
    reg  [MB:0]     s2_d   [0:2];
    reg  [2*MB-1:0] s2_d2  [0:2];
    reg  [8+VB-1:0] s2_thr [0:2];
    reg  [3:0]      s2_L   [0:2];

    // round(log2 c) = number of r with c >= ceil(2^(r+0.5))  (c >= 1)
    function [3:0] log2r;
        input [11:0] c;
        begin
            log2r = (c >= 12'd2)    + (c >= 12'd3)    + (c >= 12'd6)    + (c >= 12'd12)  +
                    (c >= 12'd23)   + (c >= 12'd46)   + (c >= 12'd91)   + (c >= 12'd182) +
                    (c >= 12'd363)  + (c >= 12'd725)  + (c >= 12'd1449) + (c >= 12'd2897);
        end
    endfunction

    genvar k;
    generate
        for (k = 0; k < 3; k = k + 1) begin : g_s2
            wire        [WB-1:0]   w_k  = s1_g[GW*k +: WB];
            wire        [MB-1:0]   mu_k = s1_g[GW*k + WB +: MB];
            wire        [VB-1:0]   v_k  = s1_g[GW*k + WB + MB +: VB];
            wire        [MB-1:0]   x_e  = s1_x;
            wire        [MB-1:0]   xs   = x_e << FM;
            wire signed [MB:0]     dx   = $signed({1'b0, xs}) - $signed({1'b0, mu_k});
            wire signed [2*MB+1:0] dsq  = dx * dx;
            always @(posedge clk) if (ce) begin
                s2_d[k]   <= dx;
                s2_d2[k]  <= dsq[2*MB-1:0];
                s2_thr[k] <= v_k * lambda2;
                s2_L[k]   <= log2r(w_k);
            end
        end
    endgenerate

    always @(posedge clk) begin
        if (rst) v2 <= 1'b0;
        else if (ce) v2 <= v1;
        if (ce) begin
            s2_x <= s1_x;
            s2_g <= s1_g;
        end
    end

    // ======================================================== S3
    reg  [7:0]      s3_x;
    reg  [PW-1:0]   s3_g;
    reg  [MB:0]     s3_d   [0:2];
    reg  [3:0]      s3_s   [0:2];
    reg  [IFW-1:0]  s3_if  [0:2];
    reg  [24:0]     s3_dv  [0:2];      // (x-mu)^2 - s2 in s2 units, signed
    reg  [2:0]      s3_m;
    wire [2:0]      mt_w;

    generate
        for (k = 0; k < 3; k = k + 1) begin : g_s3
            wire [WB-1:0] w_k = s2_g[GW*k +: WB];
            wire [VB-1:0] v_k = s2_g[GW*k + WB + MB +: VB];
            // match (eq. 15): 16 (x-mu)^2 < lambda2 * s2   (common scale)
            wire [CW-1:0] lhs = {s2_d2[k], 4'b0000};
            wire [CW-1:0] thr_e = s2_thr[k];
            wire [CW-1:0] rhs = thr_e << E;
            assign mt_w[k] = lhs < rhs;
            // learning-rate shift: s = max(a - WB + round(log2 w), 0)
            wire [4:0] L = s2_L[k];
            wire signed [6:0] t  = $signed({3'b000, alpha_sh}) + $signed({2'b00, L}) - WB;
            wire        [3:0] sh = (w_k == 0) ? 4'd0 : (t[6] ? 4'd0 : t[3:0]);
            // inverse fitness IF = s2 * 2^(2(a - s))
            wire [4:0]     ifs = {(alpha_sh - sh), 1'b0};
            wire [IFW-1:0] v_e = v_k;
            wire [IFW-1:0] ifv = (w_k == 0) ? {IFW{1'b1}} : (v_e << ifs);
            // (x - mu)^2 - s2, used by the variance update in S4
            wire        [31:0] d2q = s2_d2[k] >> E;
            wire signed [31:0] dvf = $signed(d2q) - $signed({1'b0, v_k});
            always @(posedge clk) if (ce) begin
                s3_d[k]  <= s2_d[k];
                s3_dv[k] <= dvf[24:0];
                s3_s[k]  <= sh;
                s3_if[k] <= ifv;
            end
        end
    endgenerate

    always @(posedge clk) begin
        if (rst) v3 <= 1'b0;
        else if (ce) v3 <= v2;
        if (ce) begin
            s3_x <= s2_x;
            s3_g <= s2_g;
            s3_m <= mt_w;
        end
    end

    // ======================================================== S4
    // stable ascending order of IF -> rank of every Gaussian
    wire b01 = s3_if[0] <= s3_if[1];
    wire b02 = s3_if[0] <= s3_if[2];
    wire b12 = s3_if[1] <= s3_if[2];
    wire [1:0] rk0 = {1'b0, ~b01} + {1'b0, ~b02};
    wire [1:0] rk1 = {1'b0,  b01} + {1'b0, ~b12};
    wire [1:0] rk2 = {1'b0,  b02} + {1'b0,  b12};
    wire [1:0] rk [0:2];
    assign rk[0] = rk0; assign rk[1] = rk1; assign rk[2] = rk2;

    wire [WB-1:0] w0 = s3_g[0      +: WB];
    wire [WB-1:0] w1 = s3_g[GW     +: WB];
    wire [WB-1:0] w2 = s3_g[2*GW   +: WB];
    wire [SB-1:0] m0 = s3_g[0    + WB + MB + VB +: SB];
    wire [SB-1:0] m1 = s3_g[GW   + WB + MB + VB +: SB];
    wire [SB-1:0] m2 = s3_g[2*GW + WB + MB + VB +: SB];

    // matched Gaussian = matched one with the lowest rank
    wire hit0 = s3_m[0] & ~(s3_m[1] & (rk1 < rk0)) & ~(s3_m[2] & (rk2 < rk0));
    wire hit1 = s3_m[1] & ~(s3_m[0] & (rk0 < rk1)) & ~(s3_m[2] & (rk2 < rk1));
    wire hit2 = s3_m[2] & ~(s3_m[0] & (rk0 < rk2)) & ~(s3_m[1] & (rk1 < rk2));
    wire found = |s3_m;

    // background test (eq. 8): weight ranked before the matched one <= T
    wire [WB+1:0] sb0 = ((rk1 < rk0) ? w1 : 0) + ((rk2 < rk0) ? w2 : 0);
    wire [WB+1:0] sb1 = ((rk0 < rk1) ? w0 : 0) + ((rk2 < rk1) ? w2 : 0);
    wire [WB+1:0] sb2 = ((rk0 < rk2) ? w0 : 0) + ((rk1 < rk2) ? w1 : 0);
    wire [13:0]   sbx = hit0 ? sb0 : (hit1 ? sb1 : sb2);
    wire [13:0]   sbm = sbx << (12 - WB);                       // to Q0.12
    wire fg = ~(found & (sbm <= {2'b00, t_bg}));

    // no-match replacement value of the weight (eq. 7)
    wire [SB:0] msumtot = ((rk0 < 2) ? m0 : 0) + ((rk1 < 2) ? m1 : 0) + ((rk2 < 2) ? m2 : 0);
    wire [WB-1:0] w_new = recip(msumtot);

    reg  [7:0]      s4_x;
    reg  [PW-1:0]   s4_g;
    reg             s4_fg;
    reg  [2:0]      s4_hit, s4_rep;
    reg  [WB-1:0]   s4_wnew;
    reg  [WB-1:0]   s4_whit [0:2], s4_wmis [0:2];
    reg  [MB-1:0]   s4_mu   [0:2];
    reg  [24:0]     s4_dvs  [0:2];      // round(alpha_k ((x-mu)^2 - s2)), signed
    reg  [SB-1:0]   s4_ms   [0:2];

    wire [WB:0] inc = {1'b1, {WB{1'b0}}} >> alpha_sh;          // alpha_w in weight LSBs
    wire [15:0] half_a = (16'd1 << alpha_sh) >> 1;

    generate
        for (k = 0; k < 3; k = k + 1) begin : g_s4
            wire [WB-1:0] w_k  = s3_g[GW*k +: WB];
            wire [MB-1:0] mu_k = s3_g[GW*k + WB +: MB];
            wire [VB-1:0] v_k  = s3_g[GW*k + WB + MB +: VB];
            wire [SB-1:0] ms_k = s3_g[GW*k + WB + MB + VB +: SB];
            wire [3:0]    sk   = s3_s[k];
            wire [15:0]   half_s = (16'd1 << sk) >> 1;
            // weight: w - round(w * alpha_w) (+ alpha_w if matched)
            wire [WB:0]   dec   = ({1'b0, w_k} + half_a[WB:0]) >> alpha_sh;
            wire [WB:0]   wmis  = {1'b0, w_k} - dec;
            wire [WB+1:0] whit0 = {1'b0, wmis} + {1'b0, inc};
            wire [WB-1:0] whit  = (whit0 > WMAX) ? WMAX : whit0[WB-1:0];
            // mean: mu + round(alpha_k (x - mu))       (32-bit signed math,
            // trimmed by synthesis)
            wire signed [31:0] d_s  = $signed(s3_d[k]);
            wire signed [31:0] hs   = $signed({16'd0, half_s});
            wire signed [31:0] mu_e = $signed({1'b0, mu_k});
            wire signed [31:0] mus  = mu_e + ((d_s + hs) >>> sk);
            // variance: round(alpha_k ((x - mu)^2 - s2)); the add and the
            // clamp are done in S5
            wire signed [31:0] dv   = $signed(s3_dv[k]);
            wire signed [31:0] dvs  = (dv + hs) >>> sk;
            always @(posedge clk) if (ce) begin
                s4_wmis[k] <= wmis[WB-1:0];
                s4_whit[k] <= whit;
                s4_mu[k]   <= mus[MB-1:0];
                s4_dvs[k]  <= dvs[24:0];
                s4_ms[k]   <= (ms_k == SMAX) ? SMAX : ms_k + 1'b1;
            end
        end
    endgenerate

    always @(posedge clk) begin
        if (rst) v4 <= 1'b0;
        else if (ce) v4 <= v3;
        if (ce) begin
            s4_x    <= s3_x;
            s4_g    <= s3_g;
            s4_fg   <= fg;
            s4_hit  <= {hit2, hit1, hit0};
            s4_rep  <= found ? 3'b000 : {rk2 == 2'd2, rk1 == 2'd2, rk0 == 2'd2};
            s4_wnew <= w_new;
        end
    end

    // ======================================================== S5
    wire [VB-1:0] vmin_c  = var_min[13:VS];
    wire [VB-1:0] vmax_c  = var_max[13:VS];
    wire [VB-1:0] vinit_c = var_init[13:VS];

    reg  [127:0] s5_par;
    reg          s5_fg;
    wire [PW-1:0] go;
    wire [127:0]  go_e = go;                                    // zero-extend
    generate
        for (k = 0; k < 3; k = k + 1) begin : g_s5
            wire [MB-1:0] mu_k = s4_g[GW*k + WB +: MB];
            wire [VB-1:0] v_k  = s4_g[GW*k + WB + MB +: VB];
            wire [SB-1:0] ms_k = s4_g[GW*k + WB + MB + VB +: SB];
            wire signed [31:0] vs = $signed({1'b0, v_k}) + $signed(s4_dvs[k]);
            wire [VB-1:0] vcl  = (vs < $signed({1'b0, vmin_c})) ? vmin_c :
                                 (vs > $signed({1'b0, vmax_c})) ? vmax_c : vs[VB-1:0];
            wire h = s4_hit[k];
            wire r = s4_rep[k];
            wire [WB-1:0] w_o  = h ? s4_whit[k] : (r ? s4_wnew : s4_wmis[k]);
            wire [MB-1:0] x_e  = s4_x;
            wire [MB-1:0] xs   = x_e << FM;
            wire [MB-1:0] mu_o = h ? s4_mu[k]   : (r ? xs : mu_k);
            wire [VB-1:0] v_o  = h ? vcl        : (r ? vinit_c : v_k);
            wire [SB-1:0] one  = 1;
            wire [SB-1:0] ms_o = h ? s4_ms[k]   : (r ? one : ms_k);
            assign go[GW*k +: GW] = {ms_o, v_o, mu_o, w_o};
        end
    endgenerate

    always @(posedge clk) begin
        if (rst) v5 <= 1'b0;
        else if (ce) v5 <= v4;
        if (ce) begin
            s5_fg  <= s4_fg;
            s5_par <= go_e;
        end
    end

    assign out_par = s5_par;
    assign out_fg  = s5_fg;

endmodule
`default_nettype wire
