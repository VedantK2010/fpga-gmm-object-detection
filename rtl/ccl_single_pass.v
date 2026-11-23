//-----------------------------------------------------------------------------
// ccl_single_pass.v
// Single-pass connected component labelling with on-the-fly feature
// extraction (8-connectivity). No labelled image is stored: only one row
// of provisional labels, a merger table and a feature table.
//
//   neighbourhood   A B C        A,B,C come from the row buffer
//                   D x          D is the label given to the previous pixel
//
// The merger table M is kept fully resolved (every entry points to its
// root) by updating all entries in parallel when two labels merge, so a
// single table look-up gives the root of any provisional label.
// Features per label: bounding box (minx,maxx,miny,maxy) and area.
// At the end of the frame the roots are scanned (DUMP state, one label
// per clock) and objects with area >= min_area are written to the object
// table, which is read by software through AXI-Lite.
//
// Timing structure (one pixel per clock, two stages):
//   stage A (labelling): the neighbour registers A, B, C, D always hold
//     ROOT labels. They are corrected in the same cycle when a merge
//     absorbs their label, and the row-buffer value for C is read from
//     block RAM three pixels ahead and resolved through the merger table
//     one pixel ahead, so the labelling decision needs no table look-up.
//   stage B (features): the feature-table read-modify-write of a pixel is
//     done in the cycle after its labelling. A following pixel's stage B
//     reads the table one cycle later, after the write, so no forwarding
//     is needed. Merging (min/max/sum) is symmetric and uses 3-input
//     min/max with parallel comparators.
// Minimum frame width: 5 pixels.
//
// The input stream is passed through unchanged (0x00 / 0xFF bytes) so the
// foreground mask can be written to memory by a DMA.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none
module ccl_single_pass #(
    parameter MAX_W   = 1024,
    parameter LBL_W   = 7,                 // MAX_LBL = 2^LBL_W labels
    parameter OBJ_AW  = 5                  // MAX_OBJ = 2^OBJ_AW objects
) (
    input  wire              clk,
    input  wire              rst,
    input  wire [10:0]       width,
    input  wire [10:0]       height,
    input  wire [19:0]       min_area,
    // binary pixel input
    input  wire              in_valid,
    output wire              in_ready,
    input  wire              in_bit,
    // mask pass-through output
    output reg               out_valid,
    input  wire              out_ready,
    output reg  [7:0]        out_data,
    output reg               out_last,
    // results
    input  wire [OBJ_AW-1:0] obj_rd_idx,
    output wire [59:0]       obj_rd_data,  // {area, maxy, miny, maxx, minx}
    output reg  [OBJ_AW:0]   num_obj,
    output reg  [19:0]       fg_count,
    output reg               overflow,
    output reg  [15:0]       frame_cnt
);

    localparam MAX_LBL = 1 << LBL_W;
    localparam MAX_OBJ = 1 << OBJ_AW;
    localparam AW      = $clog2(MAX_W);
    localparam S_RUN = 2'd0, S_DUMP = 2'd1, S_FIN = 2'd2, S_DRAIN = 2'd3;

    reg [1:0] state;

    // ------------------------------------------------------------ storage
    reg [LBL_W-1:0] rowbuf [0:MAX_W-1];        // labels of previous row
    reg [LBL_W-1:0] mtab   [0:MAX_LBL-1];      // merger table (resolved)
    reg [59:0]      feat   [0:MAX_LBL-1];      // feature table
    reg [59:0]      objt   [0:MAX_OBJ-1];      // object table

    // ------------------------------------------------------------ position
    reg [10:0] x, y;
    reg [LBL_W-1:0] rA, rB, rC, rD;            // neighbour labels (roots)
    reg [LBL_W:0]   nxt;                       // next free label
    reg [19:0]      fg_acc;
    reg             ovf_acc;

    wire out_free = ~out_valid | out_ready;
    wire adv      = (state == S_RUN) & in_valid & out_free;
    assign in_ready = (state == S_RUN) & out_free;

    // position flags are registered to keep the width/height comparators
    // out of the labelling path (width, height >= 3)
    reg  x_last, y_last, x0, y0;
    wire x_last_n = ~x_last & (x == width - 11'd2);   // x_last of next step
    wire y_last_n = ~y_last & (y == height - 11'd2);

    // row buffer: read address runs four pixels ahead, (x + 4) mod width.
    // The block RAM is read with its output register (rq -> rq2), the
    // value is resolved through the merger table in the following step
    // and becomes C three pixels after the read.
    reg  [AW-1:0]    ra;
    reg  [LBL_W-1:0] rq, rq2;
    always @(posedge clk) if (adv) begin
        rq  <= rowbuf[ra];
        rq2 <= rq;
    end

    // masked neighbour roots
    wire [LBL_W-1:0] la = (y0 | x0)     ? {LBL_W{1'b0}} : rA;
    wire [LBL_W-1:0] lb =  y0           ? {LBL_W{1'b0}} : rB;
    wire [LBL_W-1:0] lc = (y0 | x_last) ? {LBL_W{1'b0}} : rC;
    wire [LBL_W-1:0] ld =  x0           ? {LBL_W{1'b0}} : rD;

    wire [LBL_W-1:0] lx = (lb != 0) ? lb : ((la != 0) ? la : ld);
    wire [LBL_W-1:0] ly = lc;
    wire nz_x = (lx != 0);
    wire nz_y = (ly != 0);

    wire is_new   = in_bit & ~nz_x & ~nz_y;
    wire can_new  = (nxt < MAX_LBL);
    wire is_merge = in_bit & nz_x & nz_y & (lx != ly);
    wire [LBL_W-1:0] ls = (lx < ly) ? lx : ly;  // survivor
    wire [LBL_W-1:0] lg = (lx < ly) ? ly : lx;  // absorbed

    wire [LBL_W-1:0] cur =
        ~in_bit           ? {LBL_W{1'b0}} :
        is_new            ? (can_new ? nxt[LBL_W-1:0] : {LBL_W{1'b0}}) :
        is_merge          ? ls :
        nz_x              ? lx : ly;

    // keep the neighbour registers resolved: replace the absorbed root
    wire mrg = adv & is_merge;
    function [LBL_W-1:0] fix;
        input [LBL_W-1:0] v;
        begin
            fix = (mrg && v == lg) ? ls : v;
        end
    endfunction
    wire [LBL_W-1:0] cq = mtab[rq2];            // resolve the prefetched label

    // ------------------------------------------------------------ features
    // stage B: registered request from stage A, executed the next cycle
    reg             b_we, b_new, b_mrg;
    reg [LBL_W-1:0] b_s, b_g;
    reg [9:0]       b_x, b_y;
    always @(posedge clk) begin
        if (rst) b_we <= 1'b0;
        else     b_we <= adv & in_bit & (cur != 0);
        if (adv) begin
            b_new <= is_new;
            b_mrg <= is_merge;
            b_s   <= cur;
            b_g   <= lg;
            b_x   <= x[9:0];
            b_y   <= y[9:0];
        end
    end

    function [9:0] min3;
        input [9:0] a, b, c;
        reg ab, ac, bc;
        begin
            ab = a < b; ac = a < c; bc = b < c;
            min3 = (ab & ac) ? a : (bc ? b : c);
        end
    endfunction
    function [9:0] max3;
        input [9:0] a, b, c;
        reg ab, ac, bc;
        begin
            ab = a > b; ac = a > c; bc = b > c;
            max3 = (ab & ac) ? a : (bc ? b : c);
        end
    endfunction

    wire [59:0] f_s   = feat[b_s];
    wire [59:0] f_g   = feat[b_g];
    wire [59:0] f_o   = b_mrg ? f_g : f_s;      // min/max are idempotent
    wire [19:0] a_g   = b_mrg ? f_g[59:40] : 20'd0;
    wire [59:0] f_pix = {20'd1, b_y, b_y, b_x, b_x};
    wire [59:0] f_upd = b_new ? f_pix :
                        { f_s[59:40] + a_g + 20'd1,
                          max3(f_s[39:30], f_o[39:30], b_y),
                          min3(f_s[29:20], f_o[29:20], b_y),
                          max3(f_s[19:10], f_o[19:10], b_x),
                          min3(f_s[9:0],   f_o[9:0],   b_x) };

    always @(posedge clk) begin
        if (b_we) feat[b_s] <= f_upd;
    end

    // ------------------------------------------------------------ row buffer
    always @(posedge clk) begin
        if (adv) rowbuf[x[AW-1:0]] <= cur;
    end

    // ------------------------------------------------------------ merger table
    integer i;
    always @(posedge clk) begin
        if (rst || state == S_FIN) begin
            for (i = 0; i < MAX_LBL; i = i + 1) mtab[i] <= i;
        end else if (adv & is_merge) begin
            for (i = 0; i < MAX_LBL; i = i + 1)
                if (mtab[i] == lg) mtab[i] <= ls;
        end
    end

    // ------------------------------------------------------------ dump scan
    reg  [LBL_W:0]  di;
    reg  [OBJ_AW:0] ocnt;
    wire [LBL_W-1:0] di_l   = di[LBL_W-1:0];
    wire [59:0]      f_di   = feat[di_l];
    wire             di_ok  = (mtab[di_l] == di_l) & (f_di[59:40] >= min_area)
                              & (ocnt < MAX_OBJ);

    always @(posedge clk) begin
        if (state == S_DUMP && di < nxt && di_ok)
            objt[ocnt[OBJ_AW-1:0]] <= f_di;
    end

    assign obj_rd_data = objt[obj_rd_idx];

    // ------------------------------------------------------------ control
    always @(posedge clk) begin
        if (rst) begin
            state <= S_RUN;
            x <= 0; y <= 0;
            x0 <= 1'b1; y0 <= 1'b1; x_last <= 1'b0; y_last <= 1'b0;
            rA <= 0; rB <= 0; rC <= 0; rD <= 0;
            ra <= 4;
            nxt <= 1;
            fg_acc <= 0; ovf_acc <= 1'b0;
            di <= 0; ocnt <= 0;
            out_valid <= 1'b0; out_data <= 8'd0; out_last <= 1'b0;
            num_obj <= 0; fg_count <= 0; overflow <= 1'b0; frame_cnt <= 0;
        end else begin
            if (out_ready) out_valid <= 1'b0;
            case (state)
            S_RUN: if (adv) begin
                out_valid <= 1'b1;
                out_data  <= {8{in_bit}};
                out_last  <= x_last & y_last;
                rA <= fix(rB);
                rB <= fix(rC);                 // C of this pixel, or B of the next row
                rC <= fix(cq);                 // C of the next pixel
                rD <= cur;
                ra <= (ra == width[AW-1:0] - 1'b1) ? {AW{1'b0}} : ra + 1'b1;
                if (in_bit & is_new) begin
                    if (can_new) nxt <= nxt + 1'b1;
                    else         ovf_acc <= 1'b1;
                end
                if (in_bit) fg_acc <= fg_acc + 1'b1;
                x_last <= x_last_n;
                if (x_last) begin
                    x <= 0; x0 <= 1'b1;
                    y_last <= y_last_n;
                    if (y_last) begin
                        y <= 0; y0 <= 1'b1;
                        state <= S_DRAIN;      // let stage B finish the last pixel
                        di <= 1; ocnt <= 0;
                    end else begin
                        y <= y + 11'd1; y0 <= 1'b0;
                    end
                end else begin
                    x <= x + 11'd1; x0 <= 1'b0;
                end
            end
            S_DRAIN: state <= S_DUMP;
            S_DUMP: begin
                if (di < nxt) begin
                    if (di_ok) ocnt <= ocnt + 1'b1;
                    di <= di + 1'b1;
                end else
                    state <= S_FIN;
            end
            default: begin            // S_FIN: publish results, reset tables
                num_obj   <= ocnt;
                fg_count  <= fg_acc;
                overflow  <= ovf_acc;
                frame_cnt <= frame_cnt + 1'b1;
                fg_acc    <= 0;
                ovf_acc   <= 1'b0;
                nxt       <= 1;
                state     <= S_RUN;
            end
            endcase
        end
    end

endmodule
`default_nettype wire
