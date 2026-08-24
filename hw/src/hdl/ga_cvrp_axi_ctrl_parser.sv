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

import lynxTypes::*;

/**
 * ga_cvrp_axi_ctrl_parser
 * @brief Reads the kernel configuration, set by the user from the software via setCSR(...)
 * and forwards it to the HLS processing node kernel as a single AXI4-Stream word
 *
 * Writing the BEATS_PER_INDIVIDUAL_REG (which the software should always write last)
 * latches the whole configuration and offers a new configuration word on the m_axis_cfg
 * stream, which the free-running HLS kernel consumes before (re-)processing the data
 * streams
 *
 * @param[in] aclk Clock signal
 * @param[in] aresetn Active low reset signal
 *
 * @param[in/out] axi_ctrl AXI Lite Control signal, from/to the host via PCIe and XDMA (UltraScale+) / QDMA (Versal)
 *
 * @param[out] m_axis_cfg_tdata Configuration word; bits [31:0] num_individuals0, [63:32] num_individuals1, [95:64] beats_per_individual
 * @param[out] m_axis_cfg_tkeep Byte enables of the configuration word
 * @param[out] m_axis_cfg_tlast Last beat indication of the configuration word
 * @param[out] m_axis_cfg_tvalid Valid indication of the configuration word
 * @param[in] m_axis_cfg_tready Ready indication of the configuration word
 */
module ga_cvrp_axi_ctrl_parser #(
  parameter integer CFG_BITS = 96
) (
  input  logic                    aclk,
  input  logic                    aresetn,

  AXI4L.s                       axi_ctrl,

  output logic [CFG_BITS-1:0]   m_axis_cfg_tdata,
  output logic [CFG_BITS/8-1:0]   m_axis_cfg_tkeep,
  output logic                    m_axis_cfg_tlast,
  output logic                    m_axis_cfg_tvalid,
  input  logic                    m_axis_cfg_tready
);

///////////////////////////////////
//          CONSTANTS             //
/////////////////////////////////
localparam integer N_REGS = 3;
localparam integer ADDR_MSB = $clog2(N_REGS);
localparam integer ADDR_LSB = $clog2(AXIL_DATA_BITS/8);
localparam integer AXI_ADDR_BITS = ADDR_LSB + ADDR_MSB;

///////////////////////////////////
//          REGISTERS             //
/////////////////////////////////
// Internal AXI registers
logic [AXI_ADDR_BITS-1:0] axi_awaddr;
logic axi_awready;
logic [AXI_ADDR_BITS-1:0] axi_araddr;
logic axi_arready;
logic [1:0] axi_bresp;
logic axi_bvalid;
logic axi_wready;
logic [AXIL_DATA_BITS-1:0] axi_rdata;
logic [1:0] axi_rresp;
logic axi_rvalid;
logic aw_en;

// Registers for holding the values read from/to be written to the AXI Lite interface
// These are synchronous but the outputs are combinatorial
logic [N_REGS-1:0][AXIL_DATA_BITS-1:0] ctrl_reg;
logic ctrl_reg_rden;
logic ctrl_reg_wren;

/////////////////////////////////////
//         REGISTER MAP           //
///////////////////////////////////
// 0 (RW)   : Number of individuals on stream 0
localparam integer NUM_INDIVIDUALS_0_REG = 0;

// 1 (RW)   : Number of individuals on stream 1
localparam integer NUM_INDIVIDUALS_1_REG = 1;

// 2 (RW)   : Number of beats per individual; writing this register (the software
//            should write it last) latches the configuration and forwards it
//            to the kernel over the m_axis_cfg stream
localparam integer BEATS_PER_INDIVIDUAL_REG = 2;

///////////////////////////////////
//         WRITE PROCESS          //
///////////////////////////////////
// Data coming in from host to the vFPGA via PCIe and XDMA/QDMA
assign ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;

always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    ctrl_reg <= 0;
  end
  else begin
    if(ctrl_reg_wren) begin
      case (axi_awaddr[ADDR_LSB+:ADDR_MSB])
        NUM_INDIVIDUALS_0_REG:
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[NUM_INDIVIDUALS_0_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        NUM_INDIVIDUALS_1_REG:
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[NUM_INDIVIDUALS_1_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        BEATS_PER_INDIVIDUAL_REG:
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BEATS_PER_INDIVIDUAL_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        default: ;
      endcase
    end
  end
end

///////////////////////////////////
//         READ PROCESS           //
///////////////////////////////////
// Data going to the host from the vFPGA via XDMA/QDMA and PCIe
assign ctrl_reg_rden = axi_arready & axi_ctrl.arvalid & ~axi_rvalid;

always_ff @(posedge aclk) begin
  if(aresetn == 1'b0) begin
    axi_rdata <= 0;
  end
  else begin
    axi_rdata <= 0;
    if(ctrl_reg_rden) begin
      case (axi_araddr[ADDR_LSB+:ADDR_MSB])
        NUM_INDIVIDUALS_0_REG:
          axi_rdata <= ctrl_reg[NUM_INDIVIDUALS_0_REG];
        NUM_INDIVIDUALS_1_REG:
          axi_rdata <= ctrl_reg[NUM_INDIVIDUALS_1_REG];
        BEATS_PER_INDIVIDUAL_REG:
          axi_rdata <= ctrl_reg[BEATS_PER_INDIVIDUAL_REG];
        default: ;
      endcase
    end
  end
end

///////////////////////////////////
//     CONFIGURATION STREAM     //
///////////////////////////////////
// Latch the configuration and offer it to the kernel as one AXI4-Stream word
logic cfg_valid;

always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    cfg_valid <= 1'b0;
  end
  else begin
    // Configuration word was accepted by the kernel
    if (cfg_valid && m_axis_cfg_tready)
      cfg_valid <= 1'b0;

    // Writing the beats per individual register latches the configuration
    // and offers a (possibly updated) configuration word to the kernel
    if (ctrl_reg_wren && (axi_awaddr[ADDR_LSB+:ADDR_MSB] == BEATS_PER_INDIVIDUAL_REG))
      cfg_valid <= 1'b1;
  end
end

assign m_axis_cfg_tvalid = cfg_valid;
assign m_axis_cfg_tdata  = {ctrl_reg[BEATS_PER_INDIVIDUAL_REG][31:0],
                            ctrl_reg[NUM_INDIVIDUALS_1_REG][31:0],
                            ctrl_reg[NUM_INDIVIDUALS_0_REG][31:0]};
assign m_axis_cfg_tkeep  = '1;
assign m_axis_cfg_tlast  = 1'b1;

///////////////////////////////////
//     STANDARD AXI CONTROL       //
///////////////////////////////////
// NOT TO BE EDITED

// I/O
assign axi_ctrl.awready = axi_awready;
assign axi_ctrl.arready = axi_arready;
assign axi_ctrl.bresp = axi_bresp;
assign axi_ctrl.bvalid = axi_bvalid;
assign axi_ctrl.wready = axi_wready;
assign axi_ctrl.rdata = axi_rdata;
assign axi_ctrl.rresp = axi_rresp;
assign axi_ctrl.rvalid = axi_rvalid;

// awready and awaddr
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_awready <= 1'b0;
      axi_awaddr <= 0;
      aw_en <= 1'b1;
    end
  else
    begin
      if (~axi_awready && axi_ctrl.awvalid && axi_ctrl.wvalid && aw_en)
        begin
          axi_awready <= 1'b1;
          aw_en <= 1'b0;
          axi_awaddr <= axi_ctrl.awaddr;
        end
      else if (axi_ctrl.bready && axi_bvalid)
        begin
          aw_en <= 1'b1;
          axi_awready <= 1'b0;
        end
      else
        begin
          axi_awready <= 1'b0;
        end
    end
end

// arready and araddr
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_arready <= 1'b0;
      axi_araddr  <= 0;
    end
  else
    begin
      if (~axi_arready && axi_ctrl.arvalid)
        begin
          axi_arready <= 1'b1;
          axi_araddr  <= axi_ctrl.araddr;
        end
      else
        begin
          axi_arready <= 1'b0;
        end
    end
end

// bvalid and bresp
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_bvalid  <= 0;
      axi_bresp   <= 2'b0;
    end
  else
    begin
      if (axi_awready && axi_ctrl.awvalid && ~axi_bvalid && axi_wready && axi_ctrl.wvalid)
        begin
          axi_bvalid <= 1'b1;
          axi_bresp  <= 2'b0;
        end
      else
        begin
          if (axi_ctrl.bready && axi_bvalid)
            begin
              axi_bvalid <= 1'b0;
            end
        end
    end
end

// wready
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_wready <= 1'b0;
    end
  else
    begin
      if (~axi_wready && axi_ctrl.wvalid && axi_ctrl.awvalid && aw_en )
        begin
          axi_wready <= 1'b1;
        end
      else
        begin
          axi_wready <= 1'b0;
        end
    end
end

// rvalid and rresp
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_rvalid <= 0;
      axi_rresp  <= 0;
    end
  else
    begin
      if (axi_arready && axi_ctrl.arvalid && ~axi_rvalid)
        begin
          axi_rvalid <= 1'b1;
          axi_rresp  <= 2'b0;
        end
      else if (axi_rvalid && axi_ctrl.rready)
        begin
          axi_rvalid <= 1'b0;
        end
    end
end

endmodule
