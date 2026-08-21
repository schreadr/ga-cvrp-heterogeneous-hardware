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

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

`timescale 1ns / 1ps

import lynxTypes::*;

`include "axi_macros.svh"

/**
 * perf_local first working version
 * @brief Captures two parent packets from AXI streams and runs processing_node.
 *
 * The wrapper is intentionally simple: it buffers one parent packet from each
 * stream, feeds the exact example `processing_node` configuration, and forwards
 * the generated child packets to two AXI outputs.
 */
module perf_local (
    AXI4SR.s        axis_parent_1_in,
    AXI4SR.s        axis_parent_2_in,
    AXI4SR.m        axis_child_1_out,
    AXI4SR.m        axis_child_2_out,

    input  logic    aclk,
    input  logic    aresetn
);

localparam int lpNumCities = 16;
localparam int lpCapBitWidth = 3;
localparam int lpIDBitWidth = 12;
localparam int lpCoordBitWidth = 6;
localparam int lpDistMaxBitWidth = 13;
localparam int lpVehicleCapacity = 10;
localparam int lpRandomSubtourOffset = 5;
localparam int lpMutationDistance = 6;
localparam int lpInitMutationProcesses = 25;
localparam int lpDepotXCoord = 16;
localparam int lpDepotYCoord = 16;
localparam int lpCityNumBitWidth = 5;
localparam int lpMemLineEntryCount = 72 / (lpIDBitWidth + lpCapBitWidth + 1);
localparam int lpParentWordCount = 5;

typedef enum logic [2:0] {
    ST_WAIT_PARENTS = 3'b000,
    ST_LOAD_PARENT_1 = 3'b001,
    ST_FINISH_PARENT_1 = 3'b010,
    ST_LOAD_PARENT_2 = 3'b011,
    ST_FINISH_PARENT_2 = 3'b100,
    ST_CAPTURE_CHILDREN = 3'b101
} wrapper_state_t;

AXI4SR axis_parent_1_int (.*);
AXI4SR axis_parent_2_int (.*);
AXI4SR axis_child_1_int (.*);
AXI4SR axis_child_2_int (.*);

axisr_reg inst_reg_parent_1 (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_parent_1_in), .m_axis(axis_parent_1_int));
axisr_reg inst_reg_parent_2 (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_parent_2_in), .m_axis(axis_parent_2_int));
axisr_reg inst_reg_child_1  (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_child_1_int), .m_axis(axis_child_1_out));
axisr_reg inst_reg_child_2  (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_child_2_int), .m_axis(axis_child_2_out));

wrapper_state_t state_q;
logic [$clog2(lpParentWordCount)-1:0] parent_word_idx_q;
logic [71:0] parent_1_word_q [0:lpParentWordCount-1];
logic [71:0] parent_2_word_q [0:lpParentWordCount-1];
logic [71:0] pn_pop_data_i_q;
logic [71:0] pn_pop_data_o_q;
logic pn_start_parent_1_q;
logic pn_start_parent_2_q;
logic pn_finished_parent_1_q;
logic pn_finished_parent_2_q;
logic pn_input_reading_done_q;
logic pn_transmission_ind_1_q;
logic pn_transmission_ind_2_q;
logic pn_unchanged_q;
logic pn_better_fitness_slot_q;
logic [15:0] pn_better_fitness_value_q;
logic child_1_valid_q;
logic child_2_valid_q;
logic child_1_captured_q;
logic child_2_captured_q;
logic [$clog2(lpParentWordCount)-1:0] child_1_word_idx_q;
logic [$clog2(lpParentWordCount)-1:0] child_2_word_idx_q;
logic [71:0] child_1_word_q [0:lpParentWordCount-1];
logic [71:0] child_2_word_q [0:lpParentWordCount-1];

processing_node #(
    .pNumCities(lpNumCities),
    .pCapBitWidth(lpCapBitWidth),
    .pIDBitWidth(lpIDBitWidth),
    .pCoordBitWidth(lpCoordBitWidth),
    .pDistMaxBitWidth(lpDistMaxBitWidth),
    .pVehicleCapacity(lpVehicleCapacity),
    .pRandomSubtourOffset(lpRandomSubtourOffset),
    .pMutationDistance(lpMutationDistance),
    .pInitMutationProcesses(lpInitMutationProcesses),
    .pDepotXCoord(lpDepotXCoord),
    .pDepotYCoord(lpDepotYCoord),
    .pCityNumBitWidth(lpCityNumBitWidth)
) pn_inst (
    .clk_i(aclk),
    .rst_i(~aresetn),
    .init_mode_i(1'b0),
    .start_parent_1_i(pn_start_parent_1_q),
    .start_parent_2_i(pn_start_parent_2_q),
    .finished_parent_1_i(pn_finished_parent_1_q),
    .finished_parent_2_i(pn_finished_parent_2_q),
    .pop_data_i(pn_pop_data_i_q),
    .pop_data_o(pn_pop_data_o_q),
    .input_reading_done(pn_input_reading_done_q),
    .transmission_ind_1_o(pn_transmission_ind_1_q),
    .transmission_ind_2_o(pn_transmission_ind_2_q),
    .unchanged_ind_o(pn_unchanged_q),
    .random_city_number_mutation_1_i('0),
    .random_city_number_mutation_2_i('0),
    .better_fitness_slot_o(pn_better_fitness_slot_q),
    .better_fitness_value_o(pn_better_fitness_value_q),
    .do_mutation_i(1'b0),
    .do_wild_child_i(1'b1),
    .crossover_1_length_i(1),
    .crossover_2_length_i(1),
    .mutation_length_1_i(1),
    .mutation_length_2_i(1)
);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state_q <= ST_WAIT_PARENTS;
        parent_word_idx_q <= '0;
        pn_pop_data_i_q <= '0;
        pn_start_parent_1_q <= 1'b0;
        pn_start_parent_2_q <= 1'b0;
        pn_finished_parent_1_q <= 1'b0;
        pn_finished_parent_2_q <= 1'b0;
        child_1_valid_q <= 1'b0;
        child_2_valid_q <= 1'b0;
        child_1_captured_q <= 1'b0;
        child_2_captured_q <= 1'b0;
        child_1_word_idx_q <= '0;
        child_2_word_idx_q <= '0;
        for (int idx = 0; idx < lpParentWordCount; idx++) begin
            parent_1_word_q[idx] <= '0;
            parent_2_word_q[idx] <= '0;
            child_1_word_q[idx] <= '0;
            child_2_word_q[idx] <= '0;
        end
    end else begin
        pn_start_parent_1_q <= 1'b0;
        pn_start_parent_2_q <= 1'b0;
        pn_finished_parent_1_q <= 1'b0;
        pn_finished_parent_2_q <= 1'b0;

        if (child_1_valid_q && axis_child_1_int.tready) begin
            child_1_valid_q <= 1'b0;
        end
        if (child_2_valid_q && axis_child_2_int.tready) begin
            child_2_valid_q <= 1'b0;
        end

        case (state_q)
            ST_WAIT_PARENTS: begin
                child_1_captured_q <= 1'b0;
                child_2_captured_q <= 1'b0;
                if (axis_parent_1_int.tvalid && axis_parent_2_int.tvalid) begin
                    for (int idx = 0; idx < lpParentWordCount; idx++) begin
                        parent_1_word_q[idx] <= axis_parent_1_int.tdata[idx*72 +: 72];
                        parent_2_word_q[idx] <= axis_parent_2_int.tdata[idx*72 +: 72];
                    end
                    parent_word_idx_q <= 0;
                    state_q <= ST_LOAD_PARENT_1;
                end
            end

            ST_LOAD_PARENT_1: begin
                pn_pop_data_i_q <= parent_1_word_q[parent_word_idx_q];
                pn_start_parent_1_q <= 1'b1;
                if (pn_input_reading_done_q) begin
                    if (parent_word_idx_q == lpParentWordCount - 1) begin
                        parent_word_idx_q <= 0;
                        state_q <= ST_FINISH_PARENT_1;
                    end else begin
                        parent_word_idx_q <= parent_word_idx_q + 1;
                    end
                end
            end

            ST_FINISH_PARENT_1: begin
                pn_finished_parent_1_q <= 1'b1;
                pn_pop_data_i_q <= parent_1_word_q[lpParentWordCount-1];
                state_q <= ST_LOAD_PARENT_2;
                parent_word_idx_q <= 0;
            end

            ST_LOAD_PARENT_2: begin
                pn_pop_data_i_q <= parent_2_word_q[parent_word_idx_q];
                pn_start_parent_2_q <= 1'b1;
                if (pn_input_reading_done_q) begin
                    if (parent_word_idx_q == lpParentWordCount - 1) begin
                        parent_word_idx_q <= 0;
                        state_q <= ST_FINISH_PARENT_2;
                    end else begin
                        parent_word_idx_q <= parent_word_idx_q + 1;
                    end
                end
            end

            ST_FINISH_PARENT_2: begin
                pn_finished_parent_2_q <= 1'b1;
                pn_pop_data_i_q <= parent_2_word_q[lpParentWordCount-1];
                child_1_word_idx_q <= 0;
                child_2_word_idx_q <= 0;
                state_q <= ST_CAPTURE_CHILDREN;
            end

            ST_CAPTURE_CHILDREN: begin
                if (pn_transmission_ind_1_q && !child_1_captured_q) begin
                    child_1_word_q[child_1_word_idx_q] <= pn_pop_data_o_q;
                    if (child_1_word_idx_q == lpParentWordCount - 1) begin
                        child_1_valid_q <= 1'b1;
                        child_1_captured_q <= 1'b1;
                    end else begin
                        child_1_word_idx_q <= child_1_word_idx_q + 1;
                    end
                end

                if (pn_transmission_ind_2_q && !child_2_captured_q) begin
                    child_2_word_q[child_2_word_idx_q] <= pn_pop_data_o_q;
                    if (child_2_word_idx_q == lpParentWordCount - 1) begin
                        child_2_valid_q <= 1'b1;
                        child_2_captured_q <= 1'b1;
                    end else begin
                        child_2_word_idx_q <= child_2_word_idx_q + 1;
                    end
                end

                if (child_1_captured_q && child_2_captured_q && !child_1_valid_q && !child_2_valid_q) begin
                    state_q <= ST_WAIT_PARENTS;
                end
            end

            default: begin
                state_q <= ST_WAIT_PARENTS;
            end
        endcase
    end
end

always_comb begin
    axis_parent_1_int.tready = (state_q == ST_WAIT_PARENTS);
    axis_parent_2_int.tready = (state_q == ST_WAIT_PARENTS);

    axis_child_1_int.tvalid = child_1_valid_q;
    axis_child_2_int.tvalid = child_2_valid_q;
    axis_child_1_int.tdata  = {
        {(512-360){1'b0}},
        child_1_word_q[4],
        child_1_word_q[3],
        child_1_word_q[2],
        child_1_word_q[1],
        child_1_word_q[0]
    };
    axis_child_2_int.tdata  = {
        {(512-360){1'b0}},
        child_2_word_q[4],
        child_2_word_q[3],
        child_2_word_q[2],
        child_2_word_q[1],
        child_2_word_q[0]
    };
    axis_child_1_int.tkeep  = '1;
    axis_child_2_int.tkeep  = '1;
    axis_child_1_int.tid    = '0;
    axis_child_2_int.tid    = '0;
    axis_child_1_int.tlast  = child_1_valid_q;
    axis_child_2_int.tlast  = child_2_valid_q;
end

endmodule
