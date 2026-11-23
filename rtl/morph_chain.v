//-----------------------------------------------------------------------------
// morph_chain.v
// Configurable binary morphological filter built from four 3x3 stages:
//     dilate -> erode -> erode -> dilate
//   mode 0 : none          (all stages bypassed)
//   mode 1 : opening       (stages 3,4)
//   mode 2 : closing       (stages 1,2)
//   mode 3 : closing + opening (all stages)
// A bypassed stage is a wire. Change the mode only between frames.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none
module morph_chain #(
    parameter MAX_W = 1024
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [1:0]  mode,
    input  wire [10:0] width,
    input  wire [10:0] height,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire        in_bit,
    output wire        out_valid,
    input  wire        out_ready,
    output wire        out_bit
);

    wire [3:0] en = { mode[0], mode[0], mode[1], mode[1] };   // stage 4..1

    wire [4:0] v, b, r;            // stage boundaries 0..4
    assign v[0] = in_valid;
    assign b[0] = in_bit;
    assign in_ready = r[0];
    assign out_valid = v[4];
    assign out_bit   = b[4];
    assign r[4] = out_ready;

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_st
            wire iv, ir, ob, ov, ol;
            bin_morph3x3 #(.OP((i == 1 || i == 2) ? 0 : 1), .MAX_W(MAX_W)) u_st (
                .clk(clk), .rst(rst), .width(width), .height(height),
                .in_valid(v[i] & en[i]), .in_ready(ir), .in_bit(b[i]),
                .out_valid(ov), .out_ready(r[i+1] & en[i]),
                .out_bit(ob), .out_last(ol)
            );
            assign v[i+1] = en[i] ? ov : v[i];
            assign b[i+1] = en[i] ? ob : b[i];
            assign r[i]   = en[i] ? ir : r[i+1];
        end
    endgenerate

endmodule
`default_nettype wire
