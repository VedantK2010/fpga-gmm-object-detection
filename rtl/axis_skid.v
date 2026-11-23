//-----------------------------------------------------------------------------
// axis_skid.v
// Two-entry AXI4-Stream register slice (skid buffer). Registers both the
// data/valid path and the ready path, so a long combinational ready chain
// is cut in two without losing throughput (one transfer per clock).
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none
module axis_skid #(
    parameter W = 1
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         s_valid,
    output wire         s_ready,
    input  wire [W-1:0] s_data,
    output wire         m_valid,
    input  wire         m_ready,
    output wire [W-1:0] m_data
);

    reg         m_valid_r, skid_valid;
    reg [W-1:0] m_data_r, skid_data;

    assign s_ready = ~skid_valid;
    assign m_valid = m_valid_r;
    assign m_data  = m_data_r;

    wire s_hs = s_valid & ~skid_valid;
    wire m_hs = m_valid_r & m_ready;

    always @(posedge clk) begin
        if (rst) begin
            m_valid_r  <= 1'b0;
            skid_valid <= 1'b0;
        end else begin
            if (s_hs) begin
                if (m_valid_r && !m_ready) begin
                    skid_valid <= 1'b1;          // output stalled: park the beat
                    skid_data  <= s_data;
                end else begin
                    m_valid_r  <= 1'b1;
                    m_data_r   <= s_data;
                end
            end
            if (m_hs) begin
                if (skid_valid) begin
                    m_data_r   <= skid_data;     // refill from the skid register
                    skid_valid <= 1'b0;
                    m_valid_r  <= 1'b1;
                end else if (!s_hs) begin
                    m_valid_r  <= 1'b0;
                end
            end
        end
    end

endmodule
`default_nettype wire
