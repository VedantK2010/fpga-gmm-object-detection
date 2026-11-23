//-----------------------------------------------------------------------------
// bin_morph3x3.v
// Streaming 3x3 binary erosion (OP=0) or dilation (OP=1).
//
// Two line buffers (distributed RAM, 1 bit x MAX_W) form a 3x3 window.
// Neighbours that fall outside the frame are ignored (neutral element).
// The output stream has exactly width*height pixels; after the last input
// pixel the module generates width+1 internal flush steps so the last
// row is emitted without needing the next frame.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none
module bin_morph3x3 #(
    parameter OP    = 0,          // 0 = erode, 1 = dilate
    parameter MAX_W = 1024
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [10:0] width,
    input  wire [10:0] height,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire        in_bit,
    output reg         out_valid,
    input  wire        out_ready,
    output reg         out_bit,
    output reg         out_last
);

    reg lb1 [0:MAX_W-1];
    reg lb2 [0:MAX_W-1];

    reg [10:0] ix, iy;            // current input step position
    reg [10:0] cx, cy;            // centre of next output pixel
    localparam AW = $clog2(MAX_W);
    // window columns, each {row y-2, row y-1, row y}
    reg [2:0]  wc, wr;            // columns of steps n-2 and n-1

    wire out_free = ~out_valid | out_ready;
    wire virt     = (iy >= height);                 // flush step
    wire adv      = out_free & (virt | in_valid);
    assign in_ready = out_free & ~virt;

    wire       p   = virt ? 1'b0 : in_bit;
    wire       r1  = lb1[ix[AW-1:0]];
    wire       r2  = lb2[ix[AW-1:0]];
    wire [2:0] col = {r2, r1, p};                   // rows y-2, y-1, y

    wire emit = (iy >= 11'd2) | ((iy == 11'd1) & (ix != 11'd0));

    // validity of neighbours around the centre (cx, cy)
    wire vl = (cx != 11'd0);
    wire vr = (cx != width - 11'd1);
    wire vt = (cy != 11'd0);
    wire vb = (cy != height - 11'd1);
    wire [2:0] rv = {vt, 1'b1, vb};
    wire [2:0] cl = vl ? rv : 3'b000;
    wire [2:0] cc = rv;
    wire [2:0] cr = vr ? rv : 3'b000;
    // window after this step: left = wc, centre = wr, right = col;
    // the centre pixel is (cx, cy)
    wire [2:0] nl = wc;
    wire [2:0] nc = wr;
    wire [2:0] nr = col;
    wire res = (OP == 0) ?
        &{(nl | ~cl), (nc | ~cc), (nr | ~cr)} :
        |{(nl &  cl), (nc &  cc), (nr &  cr)};

    wire last_step = (iy == height + 11'd1) & (ix == 11'd0);

    always @(posedge clk) begin
        if (adv) begin
            lb1[ix[AW-1:0]] <= p;
            lb2[ix[AW-1:0]] <= r1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            ix <= 0; iy <= 0; cx <= 0; cy <= 0;
            wc <= 0; wr <= 0;
            out_valid <= 1'b0; out_bit <= 1'b0; out_last <= 1'b0;
        end else begin
            if (out_ready) out_valid <= 1'b0;
            if (adv) begin
                wc <= wr; wr <= col;
                if (last_step) begin
                    ix <= 0; iy <= 0;
                end else if (ix == width - 11'd1) begin
                    ix <= 0; iy <= iy + 11'd1;
                end else begin
                    ix <= ix + 11'd1;
                end
                if (emit) begin
                    out_valid <= 1'b1;
                    out_bit   <= res;
                    out_last  <= (cx == width - 11'd1) & (cy == height - 11'd1);
                    if (cx == width - 11'd1) begin
                        cx <= 0;
                        cy <= (cy == height - 11'd1) ? 11'd0 : cy + 11'd1;
                    end else begin
                        cx <= cx + 11'd1;
                    end
                end
            end
        end
    end

endmodule
`default_nettype wire
