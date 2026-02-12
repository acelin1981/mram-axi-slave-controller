/*
 * Copyright (c) 2026 acelin1981
 * All rights reserved.
 *
 * Academic/Research Use Permission:
 *   You may use, copy, and modify this code for non-commercial academic or research purposes,
 *   provided that you retain this copyright notice and clearly cite "acelin1981" as the source.
 *
 * Restrictions:
 *   - Commercial use, redistribution for profit, or inclusion in proprietary products is NOT permitted
 *     without prior written permission from the copyright holder (acelin1981).
 *
 * Disclaimer:
 *   This software is provided "AS IS", without warranty of any kind, express or implied.
 */
`include "design_preamble.sv"




// ================================================================
// npu_trace_axi_master_no_reuse.sv
// Trace-driven NPU AXI master (NO REUSE mode)
// ================================================================

module npu_trace_axi_master_no_reuse #(
  parameter int unsigned AXI_ADDR_WIDTH = 32,
  parameter int unsigned AXI_DATA_WIDTH = 64,
  parameter int unsigned AXI_ID_WIDTH   = 4,
  parameter int unsigned NUM_LAYERS     = 8,
  parameter logic [AXI_ADDR_WIDTH-1:0] WEIGHT_BASE = 32'h0000_0000,
  parameter logic [AXI_ADDR_WIDTH-1:0] INPUT_BASE  = 32'h1000_0000,
  parameter logic [AXI_ADDR_WIDTH-1:0] OUTPUT_BASE = 32'h2000_0000
)(
  input  logic                         clk,
  input  logic                         rst_n,
  input  logic                         start,
  output logic                         done,

  output logic [AXI_ID_WIDTH-1:0]       m_awid,
  output logic [AXI_ADDR_WIDTH-1:0]     m_awaddr,
  output logic [7:0]                    m_awlen,
  output logic [2:0]                    m_awsize,
  output logic [1:0]                    m_awburst,
  output logic                          m_awvalid,
  input  logic                          m_awready,

  output logic [AXI_DATA_WIDTH-1:0]     m_wdata,
  output logic [AXI_DATA_WIDTH/8-1:0]   m_wstrb,
  output logic                          m_wlast,
  output logic                          m_wvalid,
  input  logic                          m_wready,

  input  logic [AXI_ID_WIDTH-1:0]       m_bid,
  input  logic [1:0]                    m_bresp,
  input  logic                          m_bvalid,
  output logic                          m_bready,

  output logic [AXI_ID_WIDTH-1:0]       m_arid,
  output logic [AXI_ADDR_WIDTH-1:0]     m_araddr,
  output logic [7:0]                    m_arlen,
  output logic [2:0]                    m_arsize,
  output logic [1:0]                    m_arburst,
  output logic                          m_arvalid,
  input  logic                          m_arready,

  input  logic [AXI_ID_WIDTH-1:0]       m_rid,
  input  logic [AXI_DATA_WIDTH-1:0]     m_rdata,
  input  logic [1:0]                    m_rresp,
  input  logic                          m_rlast,
  input  logic                          m_rvalid,
  output logic                          m_rready
);

  // one-cycle pulse when the trace workload completes
  logic done_lvl;

  localparam int unsigned BYTES_PER_BEAT  = AXI_DATA_WIDTH/8;
  localparam int unsigned MAX_BURST_BEATS = 16;

  integer weight_bytes [0:NUM_LAYERS-1];
  integer input_bytes  [0:NUM_LAYERS-1];
  integer output_bytes [0:NUM_LAYERS-1];

  integer i;
  initial begin
    for (i = 0; i < NUM_LAYERS; i = i + 1) begin
      weight_bytes[i] = (1024 + i*256);
      input_bytes[i]  = (2048 + i*128);
      output_bytes[i] = (2048 + i*128);
    end
  end

  typedef enum logic [2:0] {ST_IDLE, ST_RD_AR, ST_RD_R, ST_WR_AW, ST_WR_W, ST_WR_B, ST_NEXT} state_t;
  typedef enum logic [1:0] {PH_WEIGHT, PH_INPUT, PH_OUTPUT} phase_t;

  state_t state;
  phase_t phase;

  integer layer_idx;
  integer beats_total, beats_rem, burst_beats;
  integer burst_cnt;

  // NOTE:
  // burst_beats is updated in always_ff (nonblocking). If we drive ARLEN/AWLEN
  // directly from burst_beats while asserting ARVALID/AWVALID in the same cycle
  // we *compute* burst_beats, ARLEN/AWLEN can reflect the previous value for
  // one cycle. To avoid this, compute the intended burst length combinationally
  // and use that for ARLEN/AWLEN when issuing AR/AW.
  integer burst_beats_calc;

  logic [AXI_ADDR_WIDTH-1:0] addr_cur;

  logic start_d;

  function automatic integer bytes_for_phase(input integer lid, input phase_t ph);
    begin
      case (ph)
        PH_WEIGHT: bytes_for_phase = weight_bytes[lid];
        PH_INPUT : bytes_for_phase = input_bytes[lid];
        default  : bytes_for_phase = output_bytes[lid];
      endcase
    end
  endfunction

  function automatic logic [AXI_ADDR_WIDTH-1:0] base_for_phase(input phase_t ph);
    begin
      case (ph)
        PH_WEIGHT: base_for_phase = WEIGHT_BASE;
        PH_INPUT : base_for_phase = INPUT_BASE;
        default  : base_for_phase = OUTPUT_BASE;
      endcase
    end
  endfunction

  task automatic load_transaction;
    integer bytes;
    begin
      bytes       = bytes_for_phase(layer_idx, phase);
      beats_total = (bytes + BYTES_PER_BEAT - 1) / BYTES_PER_BEAT;
      beats_rem   = beats_total;
      addr_cur    = base_for_phase(phase) + (layer_idx * 32'h0001_0000);
    end
  endtask

  // combinational drive
  always_comb begin
    burst_beats_calc = (beats_rem > MAX_BURST_BEATS) ? MAX_BURST_BEATS : beats_rem;

    m_awid    = '0;
    m_awaddr  = addr_cur;
    // Use computed burst length when issuing AW, so AWLEN matches the beat count.
    m_awlen   = ((state==ST_WR_AW) && (burst_beats_calc>0)) ? (burst_beats_calc-1) :
               ((burst_beats>0) ? (burst_beats-1) : 0);
    m_awsize  = $clog2(BYTES_PER_BEAT);
    m_awburst = 2'b01;
    m_awvalid = 1'b0;

    m_wdata   = 64'h0;
    m_wstrb   = {AXI_DATA_WIDTH/8{1'b1}};
    m_wlast   = 1'b0;
    m_wvalid  = 1'b0;

    m_bready  = 1'b0;

    m_arid    = '0;
    m_araddr  = addr_cur;
    // Use computed burst length when issuing AR, so ARLEN matches the beat count.
    m_arlen   = ((state==ST_RD_AR) && (burst_beats_calc>0)) ? (burst_beats_calc-1) :
               ((burst_beats>0) ? (burst_beats-1) : 0);
    m_arsize  = $clog2(BYTES_PER_BEAT);
    m_arburst = 2'b01;
    m_arvalid = 1'b0;

    m_rready  = 1'b0;

    done      = done_lvl;

    case (state)
      ST_RD_AR: m_arvalid = 1'b1;
      ST_RD_R : m_rready  = 1'b1;
      ST_WR_AW: m_awvalid = 1'b1;
      ST_WR_W : begin
        m_wvalid = 1'b1;
        m_wdata  = {32'h4E50555F, 16'(layer_idx), 8'(phase), 8'(beats_rem)};
        m_wlast  = (burst_beats==1);
      end
      ST_WR_B : m_bready  = 1'b1;
      default: ;
    endcase
  end

  // sequential FSM
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done_lvl <= 1'b0;
      start_d <= start;
      state       <= ST_IDLE;
      phase       <= PH_WEIGHT;
      layer_idx   <= 0;
      beats_total <= 0;
      beats_rem   <= 0;
      burst_beats <= 0;
      addr_cur    <= '0;
      start_d    <= 1'b0;
    end else begin
      done_lvl <= 1'b0;
      case (state)
        ST_IDLE: begin
          if (start && !start_d) begin
            layer_idx <= 0;
            phase     <= PH_WEIGHT;
            load_transaction();
            state     <= ST_RD_AR;
          end
        end

        ST_RD_AR: begin
          burst_beats <= burst_beats_calc;
          if (m_arvalid && m_arready) begin
            burst_cnt <= 0;
            state <= ST_RD_R;
          end
        end

        ST_RD_R: begin
          if (m_rvalid && m_rready) begin
            // Count beats locally so we don't depend on RLAST always being asserted correctly
            burst_cnt <= burst_cnt + 1;
            beats_rem <= beats_rem - 1;
            addr_cur  <= addr_cur + BYTES_PER_BEAT;

            // Treat end-of-burst when either RLAST is seen, or we've received burst_beats beats
            if (m_rlast || (burst_cnt == (burst_beats-1))) begin
              burst_cnt <= 0;
              if ((beats_rem - 1) == 0) begin
                state <= ST_NEXT;
              end else begin
                state <= ST_RD_AR;
              end
            end
          end
        end

        ST_WR_AW: begin
          burst_beats <= burst_beats_calc;
          if (m_awvalid && m_awready) state <= ST_WR_W;
        end

        ST_WR_W: begin
          if (m_wvalid && m_wready) begin
            beats_rem   <= beats_rem - 1;
            addr_cur    <= addr_cur + BYTES_PER_BEAT;
            burst_beats <= burst_beats - 1;
            if (burst_beats == 1) state <= ST_WR_B;
          end
        end

        ST_WR_B: begin
          if (m_bvalid && m_bready) begin
            if (beats_rem == 0) state <= ST_NEXT;
            else                state <= ST_WR_AW;
          end
        end

        ST_NEXT: begin
          if (phase == PH_WEIGHT) begin
            phase <= PH_INPUT;
            load_transaction();
            state <= ST_RD_AR;
          end else if (phase == PH_INPUT) begin
            phase <= PH_OUTPUT;
            load_transaction();
            state <= ST_WR_AW;
          end else begin
            if (layer_idx == NUM_LAYERS-1) begin
              done_lvl <= 1'b1;
              state <= ST_IDLE;
            end else begin
              layer_idx <= layer_idx + 1;
              phase     <= PH_WEIGHT;
              load_transaction();
              state     <= ST_RD_AR;
            end
          end
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

endmodule
