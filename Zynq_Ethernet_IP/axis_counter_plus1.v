// =============================================================================
// Module      : axis_counter_plus1
// Description : AXI4-Stream data processing engine. Sits between an AXI DMA
//               MM2S channel (feeding s_axis) and an AXI DMA S2MM channel
//               (draining m_axis). Adds 1 to EVERY BYTE of each beat
//               independently (matches the software "out[i] = in[i] + 1"
//               counter logic from the Ethernet project, done in hardware).
//
//               Combinational pass-through: no internal buffering, no extra
//               latency cycle. s_axis_tready is driven directly from
//               m_axis_tready, so back-pressure from the downstream DMA
//               (S2MM) propagates immediately to the upstream DMA (MM2S).
//
// Ports (Vivado auto-detects the AXI4-Stream interfaces from the
// s_axis_/m_axis_ naming convention when you package this as an IP):
//   aclk, aresetn        - single clock domain shared by both streams
//   s_axis_tvalid/tready  - slave (input) stream handshake
//   s_axis_tdata          - input data beat
//   s_axis_tkeep          - input byte-qualifier (which bytes are valid)
//   s_axis_tlast          - input end-of-packet marker
//   m_axis_tvalid/tready  - master (output) stream handshake
//   m_axis_tdata          - output data beat (= s_axis_tdata + 1 per byte)
//   m_axis_tkeep          - passed through unchanged
//   m_axis_tlast          - passed through unchanged
// =============================================================================

module axis_counter_plus1 #(
    parameter DATA_WIDTH = 32   // must be a multiple of 8; 32 matches AXI DMA default
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    // AXI4-Stream slave (data IN, from DMA MM2S)
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    input  wire [DATA_WIDTH-1:0]        s_axis_tdata,
    input  wire [(DATA_WIDTH/8)-1:0]    s_axis_tkeep,
    input  wire                         s_axis_tlast,

    // AXI4-Stream master (data OUT, to DMA S2MM)
    output wire                         m_axis_tvalid,
    input  wire                         m_axis_tready,
    output wire [DATA_WIDTH-1:0]        m_axis_tdata,
    output wire [(DATA_WIDTH/8)-1:0]    m_axis_tkeep,
    output wire                         m_axis_tlast
);

    localparam NUM_BYTES = DATA_WIDTH / 8;

    genvar i;
    generate
        for (i = 0; i < NUM_BYTES; i = i + 1) begin : gen_byte_add
            // each byte lane incremented independently, standard 8-bit
            // unsigned wrap-around on overflow (0xFF + 1 -> 0x00)
            assign m_axis_tdata[(i*8)+7 : (i*8)] = s_axis_tdata[(i*8)+7 : (i*8)] + 8'd1;
        end
    endgenerate

    // straight pass-through of handshake and side-channel signals --
    // this block adds zero extra latency and zero extra back-pressure
    assign m_axis_tvalid = s_axis_tvalid;
    assign s_axis_tready = m_axis_tready;
    assign m_axis_tkeep  = s_axis_tkeep;
    assign m_axis_tlast  = s_axis_tlast;

    // aresetn is intentionally unused: this module is pure combinational
    // logic with no state, so there is nothing to reset. It is still an
    // input port because Vivado's AXI4-Stream interface inference expects
    // an associated reset signal for the interface.

endmodule
