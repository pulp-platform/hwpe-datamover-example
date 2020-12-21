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

import hwpe_ctrl_package::*;
import hci_package::*;

module datamover_top #(
  parameter int unsigned ID        = 10,
  parameter int unsigned BW        = 288,
  parameter int unsigned N_CORES   = 8,
  parameter int unsigned N_CONTEXT = 2
) (
  // global signals
  input  logic                    clk_i,
  input  logic                    rst_ni,
  input  logic                    test_mode_i,
  // events
  output logic [N_CORES-1:0][1:0] evt_o,
  // tcdm master ports
  hci_core_intf.master            tcdm,
  // periph slave port
  hwpe_ctrl_intf_periph.slave     periph
);

  // we "sacrifice" 1 word of memory interface bandwidth in order to support realignment at a byte boundary
  localparam BW_ALIGNED = BW-32;
  
  typedef enum { DM_IDLE, DM_STARTING, DM_WORKING, DM_FINISHED } dm_state;
  dm_state cs, ns;

  // signals
  logic clear;

  typedef struct packed {
    hci_package::hci_streamer_ctrl_t data_in_source_ctrl;
    hci_package::hci_streamer_ctrl_t data_out_sink_ctrl;
  } ctrl_streamer_t;
  ctrl_streamer_t  streamer_ctrl, streamer_ctrl_cfg;
  
  typedef struct packed {
    hci_package::hci_streamer_flags_t data_in_source_flags;
    hci_package::hci_streamer_flags_t data_out_sink_flags;
    logic tcdm_fifo_empty;
  } flags_streamer_t;
  flags_streamer_t streamer_flags;

  hwpe_stream_intf_stream #(
    .DATA_WIDTH(BW_ALIGNED)
  ) data_in  (
    .clk(clk_i)
  );
  hwpe_stream_intf_stream #(
    .DATA_WIDTH(BW_ALIGNED)
  ) data_out (
    .clk(clk_i)
  );

  datamover_streamer #(
    .BW    ( BW                 )
  ) i_streamer (
    .clk_i      ( clk_i          ),
    .rst_ni     ( rst_ni         ),
    .test_mode_i( test_mode_i    ),
    .enable_i   ( 1'b1           ),
    .clear_i    ( clear          ),
    .data_in    ( data_in        ),
    .data_out   ( data_out       ),
    .tcdm       ( tcdm           ),
    .ctrl_i     ( streamer_ctrl  ),
    .flags_o    ( streamer_flags )
  );

  datamover_engine #(
    .FIFO_DEPTH ( 4          ),
    .BW_ALIGNED ( BW_ALIGNED )
  ) (
    .clk_i      ( clk_i          ),
    .rst_ni     ( rst_ni         ),
    .test_mode_i( test_mode_i    ),
    .enable_i   ( 1'b1           ),
    .clear_i    ( clear          ),
    .data_in    ( data_in        ),
    .data_out   ( data_out       )
  );

  ctrl_slave_t slave_ctrl;
  flags_slave_t slave_flags;
  ctrl_regfile_t reg_file;
  
  hwpe_ctrl_slave #(
    .N_CORES        ( N_CORES       ),
    .N_CONTEXT      ( N_CONTEXT     ),
    .N_IO_REGS      ( 13            ),
    .N_GENERIC_REGS ( 0             ),
    .ID_WIDTH       ( ID            )
  ) i_slave (
    .clk_i   ( clk_i       ),
    .rst_ni  ( rst_ni      ),
    .clear_o ( clear       ),
    .cfg     ( periph      ),
    .ctrl_i  ( slave_ctrl  ),
    .flags_o ( slave_flags ),
    .reg_file( reg_file    )
  );

  // minimal FSM for minimal control!
  always_ff @(posedge clk_i or negedge rst_ni)
  begin : fsm_seq
    if(~rst_ni)
      cs <= DM_IDLE;
    else if(clear_i)
      cs <= DM_IDLE;
    else
      cs <= ns;
  end

  always_comb
  begin : fsm_ns_comb
    ns = cs;
    if(cs == DM_IDLE) begin
      if(slave_flags.start)
        ns = DM_STARTING;
    end
    else if(cs == DM_STARTING) begin
      ns = DM_WORKING;
    end
    else if(cs == DM_WORKING) begin
      if ((streamer_flags.data_out_flags.done | streamer_flags.data_out_flags.ready_start) & (streamer_flags.data_in_flags.done | streamer_flags.data_in_flags.ready_start) & streamer_flags.tcdm_fifo_empty)
        ns = DM_FINISHED;
    end
    else begin
      ns = DM_IDLE;
    end
  end

  always_comb
  begin : fsm_out_comb
    slave_ctrl = '0;
    streamer_ctrl = streamer_ctrl_cfg;
    if(cs == DM_STARTING) begin
      streamer_ctrl.data_in_source_ctrl.req_start;
      streamer_ctrl.data_out_sink_ctrl.req_start;
    end
    else begin
      slave_ctrl.done = 1'b1;
    end
  end

  // streamer configuration binding
  always_comb
  begin
    streamer_ctrl_cfg = '0;
    streamer_ctrl_cfg.data_in_source_ctrl.dim_enable_1h = '1;
    streamer_ctrl_cfg.data_out_sink_ctrl.dim_enable_1h  = '1;
    streamer_ctrl_cfg.data_in_source_ctrl.addressgen_ctrl.base_addr = reg_file.hwpe_params[0];
    streamer_ctrl_cfg.data_out_sink_ctrl.addressgen_ctrl.base_addr  = reg_file.hwpe_params[1];
    streamer_ctrl_cfg.data_in_source_ctrl.addressgen_ctrl.tot_len   = reg_file.hwpe_params[2];
    streamer_ctrl_cfg.data_out_sink_ctrl.addressgen_ctrl.tot_len    = reg_file.hwpe_params[2];
    streamer_ctrl_cfg.data_in_source_ctrl.addressgen_ctrl.d0_len    = reg_file.hwpe_params[3];
    streamer_ctrl_cfg.data_in_source_ctrl.addressgen_ctrl.d0_stride = reg_file.hwpe_params[4];
    streamer_ctrl_cfg.data_in_source_ctrl.addressgen_ctrl.d1_len    = reg_file.hwpe_params[5];
    streamer_ctrl_cfg.data_in_source_ctrl.addressgen_ctrl.d1_stride = reg_file.hwpe_params[6];
    streamer_ctrl_cfg.data_in_source_ctrl.addressgen_ctrl.d2_len    = reg_file.hwpe_params[7];
    streamer_ctrl_cfg.data_out_sink_ctrl.addressgen_ctrl.d0_len     = reg_file.hwpe_params[8];
    streamer_ctrl_cfg.data_out_sink_ctrl.addressgen_ctrl.d0_stride  = reg_file.hwpe_params[9];
    streamer_ctrl_cfg.data_out_sink_ctrl.addressgen_ctrl.d1_len     = reg_file.hwpe_params[10];
    streamer_ctrl_cfg.data_out_sink_ctrl.addressgen_ctrl.d1_stride  = reg_file.hwpe_params[11];
    streamer_ctrl_cfg.data_out_sink_ctrl.addressgen_ctrl.d2_len     = reg_file.hwpe_params[12];
  end

endmodule // datamover_top
