/*
 * Copyright (C) 2020 ETH Zurich and University of Bologna
 *
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the "License"); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */

/*
 * Authors:  Francesco Conti <f.conti@unibo.it>
 */

import hwpe_stream_package::*;
import hci_package::*;

module datamover_streamer #(
  parameter int unsigned TCDM_FIFO_DEPTH = 2,
  parameter int unsigned BW = 32
) (
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // local enable & clear
  input  logic                   enable_i,
  input  logic                   clear_i,
  // input data stream + handshake
  hwpe_stream_intf_stream.source data_in,
  // output data stream + handshake
  hwpe_stream_intf_stream.sink   data_out,
  // TCDM ports
  hci_core_intf.master           tcdm,
  // control channel
  input  ctrl_streamer_t         ctrl_i,
  output flags_streamer_t        flags_o
);

  // we "sacrifice" 1 word of memory interface bandwidth in order to support realignment at a byte boundary
  localparam BW_ALIGNED = BW-32;
  flags_fifo_t tcdm_fifo_flags;

  hci_core_intf #(
    .DW ( BW )
  ) virt_tcdm [1:0] (
    .clk ( clk_i )
  );

  hci_core_intf #(
    .DW ( BW )
  ) tcdm_prefifo [0:0] (
    .clk ( clk_i )
  );

  hci_core_intf #(
    .DW ( BW )
  ) tcdm_prefilter [0:0] (
    .clk ( clk_i )
  );

  hci_core_source #(
    .DATA_WIDTH ( BW )
  ) i_source (
    .clk_i       ( clk_i                         ),
    .rst_ni      ( rst_ni                        ),
    .test_mode_i ( test_mode_i                   ),
    .clear_i     ( clear_i                       ),
    .enable_i    ( 1'b1                          ),
    .tcdm        ( virt_tcdm [0]                 ),
    .stream      ( data_in                       ),
    .ctrl_i      ( ctrl_i.data_in_ctrl           ),
    .flags_o     ( flags_o.data_in_flags         )
  );

  hci_core_sink #(
    .DATA_WIDTH ( BW )
  ) i_sink (
    .clk_i       ( clk_i                       ),
    .rst_ni      ( rst_ni                      ),
    .test_mode_i ( test_mode_i                 ),
    .clear_i     ( clear_i                     ),
    .enable_i    ( 1'b1                        ),
    .tcdm        ( virt_tcdm [1]               ),
    .stream      ( data_out                    ),
    .ctrl_i      ( ctrl_i.data_out_ctrl        ),
    .flags_o     ( flags_o.data_out_flags      )
  );

  generate
    if(TCDM_FIFO_DEPTH > 0) begin : use_fifo_gen
      hci_core_mux_static #(
        .NB_IN_CHAN  ( 2  ),
        .NB_OUT_CHAN ( 1  ),
        .DW          ( BW )
      ) i_ld_st_mux_static (
        .clk_i   ( clk_i                ),
        .rst_ni  ( rst_ni               ),
        .clear_i ( clear_i              ),
        .in      ( virt_tcdm            ),
        .out     ( tcdm_prefifo         )
      );

      hci_core_fifo #(
        .FIFO_DEPTH ( TCDM_FIFO_DEPTH ),
        .DW         ( BW              ),
        .AW         ( 32              ),
        .OW         (  1              )
      ) i_tcdm_fifo (
        .clk_i       ( clk_i                       ),
        .rst_ni      ( rst_ni                      ),
        .clear_i     ( clear_i                     ),
        .flags_o     ( tcdm_fifo_flags             ),
        .tcdm_slave  ( tcdm_prefifo[0]             ),
        .tcdm_master ( tcdm_prefilter[0]           )
      );
    end
    else begin : dont_use_fifo_gen
      hci_core_mux_dynamic #(
        .NB_IN_CHAN  ( 2  ),
        .NB_OUT_CHAN ( 1  ),
        .DW          ( BW )
      ) i_ld_st_mux_static (
        .clk_i   ( clk_i                ),
        .rst_ni  ( rst_ni               ),
        .clear_i ( clear_i              ),
        .in      ( virt_tcdm            ),
        .out     ( tcdm_prefilter       )
      );
      assign tcdm_fifo_flags.empty = 1'b1;
    end
  endgenerate

  // filter out r_valid strobes generated when the TCDM access is a write
  hci_core_r_valid_filter i_tcdm_filter (
    .clk_i       ( clk_i                ),
    .rst_ni      ( rst_ni               ),
    .clear_i     ( clear_i              ),
    .enable_i    ( 1'b1                 ),
    .tcdm_slave  ( tcdm_prefilter[0]    ),
    .tcdm_master ( tcdm                 )
  );
  assign flags_o.tcdm_fifo_empty = tcdm_fifo_flags.empty;

endmodule // datamover_streamer
