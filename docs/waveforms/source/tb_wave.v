//-----------------------------------------------------------------------------
// tb_mod_top.v - self-checking, closed-loop frame-level testbench for mod_top
//
// * configures the core over AXI4-Lite
// * streams RGBX frames + GMM parameters with random valid gaps
// * applies random back-pressure on both output streams
// * feeds the returned parameters back as the next frame's model
//   (exactly what the ARM software does with its ping-pong buffers)
// * compares mask, object table and final model with the golden files
//   produced by model/gen_vectors.py and prints TEST PASSED / TEST FAILED
//
// Expected files are read from `VEC_DIR (default "vectors/").
// Works with Vivado xsim (run_xsim.sh / run_xsim.bat or the Vivado GUI)
// and with Icarus Verilog (run_sim.sh).
//-----------------------------------------------------------------------------
`timescale 1ns/1ps
// In a Vivado project the .mem files are copied next to the simulator
// (define VEC_FLAT); stand-alone runs read them from ./vectors/.
`ifdef VEC_FLAT
 `define VFILE(n) n
`else
 `ifndef VEC_DIR
  `define VEC_DIR "vectors/"
 `endif
 `define VFILE(n) {`VEC_DIR, n}
`endif
module tb_mod_top;
`include "tb_cfg.vh"

    localparam NPIX   = TB_W * TB_H;
    localparam MAXOBJ = 32;
    localparam LBLW   = (TB_MAX_LBL == 64) ? 6 : (TB_MAX_LBL == 256 ? 8 : 7);
    localparam [15:0] EXP_FMT = TB_SB * 4096 + (TB_VB - 8) * 256 + TB_FM * 16 + TB_WB;

    reg aclk = 0, aresetn = 0;
    always #5 aclk = ~aclk;          // 100 MHz

    integer seed = 12345;
    integer stall_pct = 38;           // out of 128: ~30 % gaps / back-pressure

    // AXI-Lite
    reg  [9:0]  awaddr = 0, araddr = 0;
    reg         awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    reg  [31:0] wdata = 0;
    wire        awready, wready, bvalid, arready, rvalid;
    wire [1:0]  bresp, rresp;
    wire [31:0] rdata;

    // streams
    reg  [31:0]  pix_d = 0;  reg pix_v = 0; wire pix_r;
    reg  [127:0] par_d = 0;  reg par_v = 0; wire par_r;
    wire [127:0] po_d;   wire po_v, po_l; reg po_r = 0; wire [15:0] po_k;
    wire [7:0]   mo_d;   wire mo_v, mo_l; reg mo_r = 0; wire [0:0] mo_k;
    wire         frame_done;

    mod_top #(.LBL_W(LBLW), .GMM_WB(TB_WB), .GMM_FM(TB_FM), .GMM_VB(TB_VB), .GMM_SB(TB_SB)) dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_ctrl_awaddr(awaddr), .s_axi_ctrl_awprot(3'b0),
        .s_axi_ctrl_awvalid(awvalid), .s_axi_ctrl_awready(awready),
        .s_axi_ctrl_wdata(wdata), .s_axi_ctrl_wstrb(4'hF),
        .s_axi_ctrl_wvalid(wvalid), .s_axi_ctrl_wready(wready),
        .s_axi_ctrl_bresp(bresp), .s_axi_ctrl_bvalid(bvalid), .s_axi_ctrl_bready(bready),
        .s_axi_ctrl_araddr(araddr), .s_axi_ctrl_arprot(3'b0),
        .s_axi_ctrl_arvalid(arvalid), .s_axi_ctrl_arready(arready),
        .s_axi_ctrl_rdata(rdata), .s_axi_ctrl_rresp(rresp),
        .s_axi_ctrl_rvalid(rvalid), .s_axi_ctrl_rready(rready),
        .s_axis_pix_tdata(pix_d), .s_axis_pix_tvalid(pix_v), .s_axis_pix_tready(pix_r),
        .s_axis_pix_tlast(1'b0),
        .s_axis_par_tdata(par_d), .s_axis_par_tvalid(par_v), .s_axis_par_tready(par_r),
        .s_axis_par_tlast(1'b0),
        .m_axis_par_tdata(po_d), .m_axis_par_tkeep(po_k), .m_axis_par_tvalid(po_v),
        .m_axis_par_tready(po_r), .m_axis_par_tlast(po_l),
        .m_axis_mask_tdata(mo_d), .m_axis_mask_tkeep(mo_k), .m_axis_mask_tvalid(mo_v),
        .m_axis_mask_tready(mo_r), .m_axis_mask_tlast(mo_l),
        .frame_done(frame_done)
    );

    reg [31:0]  pix_mem  [0:NPIX*TB_NF-1];
    reg [127:0] par_mem  [0:NPIX-1];
    reg [127:0] par_nxt  [0:NPIX-1];
    reg [127:0] par_exp  [0:NPIX-1];
    reg [7:0]   mask_exp [0:NPIX*TB_NF-1];
    reg [59:0]  obj_exp  [0:(MAXOBJ+1)*TB_NF-1];
    reg [3:0]   alpha_t  [0:TB_NF-1];          // per-frame SHIFTS value

    integer errors = 0, mask_err = 0, par_err = 0, obj_err = 0;
    integer cycles_total = 0;
    always @(posedge aclk) cycles_total <= cycles_total + 1;

    // ---------------------------------------------------------- AXI-Lite
    task axil_write(input [9:0] a, input [31:0] d);
        begin
            @(posedge aclk);
            awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1; bready <= 1;
            @(posedge aclk);
            while (!(awready && wready)) @(posedge aclk);
            awvalid <= 0; wvalid <= 0;
            while (!bvalid) @(posedge aclk);
            @(posedge aclk);
            bready <= 0;
        end
    endtask

    task axil_read(input [9:0] a, output [31:0] d);
        begin
            @(posedge aclk);
            araddr <= a; arvalid <= 1; rready <= 1;
            @(posedge aclk);
            while (!arready) @(posedge aclk);
            arvalid <= 0;
            while (!rvalid) @(posedge aclk);
            d = rdata;
            @(posedge aclk);
            rready <= 0;
        end
    endtask

    // ---------------------------------------------------------- streams
    integer f, n_pix_in, n_par_in, n_po, n_mo, t0, t1, i;
    reg [31:0] r, r2, r3, fc_before, ra, rb, rc;
    reg [59:0] eo;
    reg run = 0;

    always @(posedge aclk) begin
        if (run) begin
            if (pix_v && pix_r) n_pix_in = n_pix_in + 1;
            if (n_pix_in < NPIX && (!pix_v || pix_r)) begin
                if (($random(seed) & 127) >= stall_pct) begin
                    pix_v <= 1; pix_d <= pix_mem[f*NPIX + n_pix_in];
                end else pix_v <= 0;
            end else if (pix_r) pix_v <= 0;

            if (par_v && par_r) n_par_in = n_par_in + 1;
            if (n_par_in < NPIX && (!par_v || par_r)) begin
                if (($random(seed) & 127) >= stall_pct) begin
                    par_v <= 1; par_d <= par_mem[n_par_in];
                end else par_v <= 0;
            end else if (par_r) par_v <= 0;

            if (po_v && po_r) begin
                par_nxt[n_po] = po_d;
                if (po_l != (n_po == NPIX-1)) begin
                    $display("ERROR: params TLAST at beat %0d", n_po); errors = errors + 1;
                end
                n_po = n_po + 1;
            end
            if (mo_v && mo_r) begin
                if (mo_d !== mask_exp[f*NPIX + n_mo]) begin
                    if (mask_err < 10)
                        $display("ERROR: frame %0d mask pixel %0d (x=%0d y=%0d): got %h expected %h",
                                 f, n_mo, n_mo % TB_W, n_mo / TB_W, mo_d, mask_exp[f*NPIX + n_mo]);
                    mask_err = mask_err + 1;
                end
                if (mo_l != (n_mo == NPIX-1)) begin
                    $display("ERROR: mask TLAST at beat %0d", n_mo); errors = errors + 1;
                end
                n_mo = n_mo + 1;
            end
            po_r <= (($random(seed) & 127) >= stall_pct);
            mo_r <= (($random(seed) & 127) >= stall_pct);
        end else begin
            pix_v <= 0; par_v <= 0; po_r <= 0; mo_r <= 0;
        end
    end

    // ---------------------------------------------------------- main
    initial begin
        $readmemh(`VFILE("pix.mem"),      pix_mem);
        $readmemh(`VFILE("par_init.mem"), par_mem);
        $readmemh(`VFILE("par_exp.mem"),  par_exp);
        $readmemh(`VFILE("mask_exp.mem"), mask_exp);
        $readmemh(`VFILE("obj_exp.mem"),  obj_exp);
        $readmemh(`VFILE("alpha_sched.mem"), alpha_t);
        if ($test$plusargs("nostall")) stall_pct = 0;
        $display("tb_mod_top: %0dx%0d, %0d frames, GMM word lengths w%0d mu%0d s2%0d ms%0d",
                 TB_W, TB_H, TB_NF, TB_WB, 8 + TB_FM, TB_VB, TB_SB);

        repeat (5) @(posedge aclk);
        aresetn <= 1;
        repeat (5) @(posedge aclk);

        axil_read(10'h038, r);
        if (r !== 32'h4D4F4401) begin
            $display("ERROR: bad ID %h", r); errors = errors + 1;
        end
        axil_read(10'h040, r);
        if (r[15:0] !== EXP_FMT) begin
            $display("ERROR: GMM_FMT register %h", r); errors = errors + 1;
        end
        axil_write(10'h000, 1);                         // soft reset while configuring
        axil_write(10'h008, TB_W);
        axil_write(10'h00C, TB_H);
        axil_write(10'h010, TB_ALPHA_SH);
        axil_write(10'h014, TB_LAMBDA2);
        axil_write(10'h018, TB_T_BG);
        axil_write(10'h020, TB_VAR_INIT);
        axil_write(10'h024, TB_VAR_MIN);
        axil_write(10'h028, TB_VAR_MAX);
        axil_write(10'h02C, TB_MIN_AREA);
        axil_write(10'h000, TB_MORPH_MODE << 2);        // release reset
        axil_read(10'h008, r);
        if (r != TB_W) begin $display("ERROR: WIDTH readback %0d", r); errors = errors + 1; end

        for (f = 0; f < TB_NF; f = f + 1) begin
            axil_write(10'h010, alpha_t[f]);            // warm-up learning-rate schedule
            axil_read(10'h004, fc_before);
            n_pix_in = 0; n_par_in = 0; n_po = 0; n_mo = 0;
            t0 = cycles_total;
            @(posedge aclk); run <= 1;
            while (n_po < NPIX || n_mo < NPIX) @(posedge aclk);
            t1 = cycles_total;
            @(posedge aclk); run <= 0;
            // wait for the object table of this frame
            r = fc_before;
            while (r[31:16] == fc_before[31:16]) axil_read(10'h004, r);
            axil_read(10'h030, r2);
            axil_read(10'h034, r3);
            eo = obj_exp[f*(MAXOBJ+1)];
            if (r2 != eo[25:20] || r3 != eo[19:0] || r[0] != eo[26]) begin
                $display("ERROR: frame %0d: objects %0d fg %0d ovf %0d, expected %0d %0d %0d",
                         f, r2, r3, r[0], eo[25:20], eo[19:0], eo[26]);
                obj_err = obj_err + 1;
            end
            for (i = 0; i < r2 && i < MAXOBJ; i = i + 1) begin
                axil_read(10'h200 + i*16 + 0, ra);
                axil_read(10'h200 + i*16 + 4, rb);
                axil_read(10'h200 + i*16 + 8, rc);
                eo = obj_exp[f*(MAXOBJ+1) + 1 + i];
                if (ra[9:0] != eo[9:0] || ra[25:16] != eo[19:10] || rb[9:0] != eo[29:20] ||
                    rb[25:16] != eo[39:30] || rc[19:0] != eo[59:40]) begin
                    $display("ERROR: frame %0d object %0d: x %0d..%0d y %0d..%0d a %0d, expected x %0d..%0d y %0d..%0d a %0d",
                             f, i, ra[9:0], ra[25:16], rb[9:0], rb[25:16], rc[19:0],
                             eo[9:0], eo[19:10], eo[29:20], eo[39:30], eo[59:40]);
                    obj_err = obj_err + 1;
                end
            end
            for (i = 0; i < NPIX; i = i + 1) par_mem[i] = par_nxt[i];
            $display("frame %0d done: %0d objects, %0d cycles (%.3f px/clk)",
                     f, r2, t1 - t0, NPIX * 1.0 / (t1 - t0));
        end

        for (i = 0; i < NPIX; i = i + 1)
            if (par_mem[i] !== par_exp[i]) begin
                if (par_err < 5) $display("ERROR: model word %0d: got %h expected %h", i, par_mem[i], par_exp[i]);
                par_err = par_err + 1;
            end

        errors = errors + mask_err + par_err + obj_err;
        $display("mask mismatches: %0d, object mismatches: %0d, model mismatches: %0d",
                 mask_err, obj_err, par_err);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED (%0d errors)", errors);
        $finish;
    end

    initial begin
        #(64'd4000000000);
        $display("TEST FAILED (timeout)");
        $finish;
    end


    // ---- waveform capture (added for the report; frame 5 only) ----
    reg wave_on = 0;
    initial begin
        $dumpfile(`WAVE_FILE);
        $dumpvars(0, tb_mod_top.aclk, tb_mod_top.aresetn,
                  tb_mod_top.pix_v, tb_mod_top.pix_r, tb_mod_top.pix_d,
                  tb_mod_top.par_v, tb_mod_top.par_r,
                  tb_mod_top.po_v,  tb_mod_top.po_r,  tb_mod_top.po_l,
                  tb_mod_top.mo_v,  tb_mod_top.mo_r,  tb_mod_top.mo_d, tb_mod_top.mo_l,
                  tb_mod_top.frame_done,
                  tb_mod_top.dut.u_gmm.v1, tb_mod_top.dut.u_gmm.v2, tb_mod_top.dut.u_gmm.v3,
                  tb_mod_top.dut.u_gmm.v4, tb_mod_top.dut.u_gmm.v5,
                  tb_mod_top.dut.u_ccl.state, tb_mod_top.dut.u_ccl.x, tb_mod_top.dut.u_ccl.y,
                  tb_mod_top.dut.u_ccl.nxt);
        $dumpoff;
    end
    always @(posedge aclk) begin
        if (f == 5 && !wave_on) begin wave_on = 1; $dumpon; end
        if (f == 6) $dumpoff;
    end
endmodule
