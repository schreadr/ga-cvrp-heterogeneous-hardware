/**
 * This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
 *
 * MIT Licence
 * Copyright (c) 2025, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/////////////////////////////////////////////////////////
//               CONTROL REGISTERS                    //
///////////////////////////////////////////////////////
// Parses the kernel configuration, set by the user from the software via setCSR(...)
// Register 0 : Number of individuals on stream 0
// Register 1 : Number of individuals on stream 1
// Register 2 : Number of beats per individual; writing this register (the software
//              should write it last) latches the configuration and forwards it
//              to the HLS kernel over the m_axis_cfg stream
// For more details, see hdl/ga_cvrp_axi_ctrl_parser.sv and the user software
logic [95:0] cfg_tdata;
logic [11:0] cfg_tkeep;
logic        cfg_tlast;
logic        cfg_tvalid;
logic        cfg_tready;

ga_cvrp_axi_ctrl_parser inst_ga_cvrp_axi_ctrl_parser (
    .aclk(aclk),
    .aresetn(aresetn),

    .axi_ctrl(axi_ctrl),

    .m_axis_cfg_tdata(cfg_tdata),
    .m_axis_cfg_tkeep(cfg_tkeep),
    .m_axis_cfg_tlast(cfg_tlast),
    .m_axis_cfg_tvalid(cfg_tvalid),
    .m_axis_cfg_tready(cfg_tready)
);

/////////////////////////////////////////////////////////
//               PROCESSING NODE (HLS)                //
///////////////////////////////////////////////////////
// Free-running kernel; no AP control interface, hence no need to "start" it
// from the software. The configuration is received from the parser above over
// the cfg AXI4-Stream interface
processing_node_hls_ip inst_pn(
    .ap_clk        (aclk),
    .ap_rst_n      (aresetn),

    // AXI4-Stream configuration input
    .cfg_TDATA     (cfg_tdata),
    .cfg_TKEEP     (cfg_tkeep),
    .cfg_TLAST     (cfg_tlast),
    .cfg_TVALID    (cfg_tvalid),
    .cfg_TREADY    (cfg_tready),

    // AXI-Stream input 0
    .in0_TDATA     (axis_host_recv[0].tdata),
    .in0_TKEEP     (axis_host_recv[0].tkeep),
    .in0_TLAST     (axis_host_recv[0].tlast),
    .in0_TVALID    (axis_host_recv[0].tvalid),
    .in0_TREADY    (axis_host_recv[0].tready),

    // AXI-Stream input 1
    .in1_TDATA     (axis_host_recv[1].tdata),
    .in1_TKEEP     (axis_host_recv[1].tkeep),
    .in1_TLAST     (axis_host_recv[1].tlast),
    .in1_TVALID    (axis_host_recv[1].tvalid),
    .in1_TREADY    (axis_host_recv[1].tready),

    // AXI-Stream output 0
    .out0_TDATA    (axis_host_send[0].tdata),
    .out0_TKEEP    (axis_host_send[0].tkeep),
    .out0_TLAST    (axis_host_send[0].tlast),
    .out0_TVALID   (axis_host_send[0].tvalid),
    .out0_TREADY   (axis_host_send[0].tready),

    // AXI-Stream output 1
    .out1_TDATA    (axis_host_send[1].tdata),
    .out1_TKEEP    (axis_host_send[1].tkeep),
    .out1_TLAST    (axis_host_send[1].tlast),
    .out1_TVALID   (axis_host_send[1].tvalid),
    .out1_TREADY   (axis_host_send[1].tready)
);

// Tie-off unused signals to avoid synthesis problems
always_comb notify.tie_off_m();
always_comb sq_rd.tie_off_m();
always_comb sq_wr.tie_off_m();
always_comb cq_rd.tie_off_s();
always_comb cq_wr.tie_off_s();

ila_processing_node inst_ila_processing_node (
    .clk(aclk),
    .probe0(axis_host_recv[0].tvalid),  // 1 bit
    .probe1(axis_host_recv[0].tready),  // 1 bit
    .probe2(axis_host_recv[0].tlast),   // 1 bit
    .probe3(axis_host_recv[0].tdata),   // 512 bits
    .probe4(axis_host_send[0].tvalid),  // 1 bit
    .probe5(axis_host_send[0].tready),  // 1 bit
    .probe6(axis_host_send[0].tlast),   // 1 bit
    .probe7(axis_host_send[0].tdata),   // 512 bits
    .probe8(axis_host_recv[1].tvalid),  // 1 bit
    .probe9(axis_host_recv[1].tready),  // 1 bit
    .probe10(axis_host_recv[1].tlast),   // 1 bit
    .probe11(axis_host_recv[1].tdata),   // 512 bits
    .probe12(axis_host_send[1].tvalid),  // 1 bit
    .probe13(axis_host_send[1].tready),  // 1 bit
    .probe14(axis_host_send[1].tlast),   // 1 bit
    .probe15(axis_host_send[1].tdata)    // 512 bits
);
