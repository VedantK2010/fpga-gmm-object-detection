// Stand-alone testbench: random binary images through ccl_single_pass
`timescale 1ns/1ps
module tb_ccl;
`include "ccl_cfg.vh"
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;
    reg [10:0] width, height;
    reg in_valid = 0, in_bit = 0, out_ready = 0;
    wire in_ready, out_valid, out_last;
    wire [7:0] out_data;
    reg  [4:0] idx = 0;
    wire [59:0] od;
    wire [5:0] num_obj; wire [19:0] fgc; wire ovf; wire [15:0] fcnt;
    ccl_single_pass #(.MAX_W(1024), .LBL_W(8), .OBJ_AW(5)) dut (
        .clk(clk), .rst(rst), .width(width), .height(height), .min_area(20'd1),
        .in_valid(in_valid), .in_ready(in_ready), .in_bit(in_bit),
        .out_valid(out_valid), .out_ready(out_ready), .out_data(out_data), .out_last(out_last),
        .obj_rd_idx(idx), .obj_rd_data(od), .num_obj(num_obj), .fg_count(fgc),
        .overflow(ovf), .frame_cnt(fcnt));
    reg [23:0] dims [0:N_IMG-1];
    reg        bits [0:N_BITS-1];
    integer t, p, base = 0, i, fo, seed = 77, nout;
    reg [15:0] fc0;
    always @(posedge clk) begin
        out_ready <= ($random(seed) & 3) != 0;
        if (out_valid && out_ready) nout = nout + 1;
    end
    initial begin
        $readmemh("ccl_vec/dims.hex", dims);
        $readmemh("ccl_vec/bits.hex", bits);
        fo = $fopen("out/ccl_rtl.txt", "w");
        repeat (3) @(posedge clk); rst <= 0;
        for (t = 0; t < N_IMG; t = t + 1) begin
            width = dims[t][23:12]; height = dims[t][11:0];
            fc0 = fcnt; nout = 0;
            for (p = 0; p < width * height; p = p + 1) begin
                @(posedge clk);
                while (($random(seed) & 7) == 0) begin in_valid <= 0; @(posedge clk); end
                in_valid <= 1; in_bit <= bits[base + p];
                @(posedge clk);
                while (!in_ready) @(posedge clk);
                in_valid <= 0;
                // re-align: the handshake happened at this edge
            end
            base = base + width * height;
            while (fcnt == fc0) @(posedge clk);
            $fwrite(fo, "img %0d n %0d ovf %0d\n", t, num_obj, ovf);
            for (i = 0; i < num_obj; i = i + 1) begin
                idx = i; #1;
                $fwrite(fo, "  %0d %0d %0d %0d %0d\n", od[9:0], od[19:10], od[29:20], od[39:30], od[59:40]);
            end
            while (nout < width * height) @(posedge clk);
        end
        $fclose(fo);
        $display("CCL_TB_DONE");
        $finish;
    end
endmodule
