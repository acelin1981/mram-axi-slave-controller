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



// soc_mem_power_monitor.sv
// ------------------------------------------------------------
// Power model for SoC-level simulation using activity counters.
// - eMRAM: uses cs/rd_en/we_en behavior (write=100 cycles, read=2 cycles).
// - DDR4 : AXI-level approximation using IDD-like active/background currents.
// NOTE: This is a *model* for architecture comparison, not signoff.
// ------------------------------------------------------------

module soc_mem_power_monitor #(
  parameter int  ADDR_W = 32,
  parameter int  DATA_W = 64,
  parameter int  ID_W   = 4,

  // Clock
  parameter real FCLK_HZ = 200_000_000.0,   // 200MHz
  parameter real TCLK_S  = 1.0 / FCLK_HZ,

  // Supplies
  parameter real VDD  = 1.0,
  parameter real VDDQ = 1.0,

  // ------------------------
  // Leakage / Static currents
  // These are ALWAYS consumed when mem_pwr_on=1, even during RD/WR/BG.
  // ------------------------
  parameter real I_LEAK_VDD_A  = 0.0,
  parameter real I_LEAK_VDDQ_A = 0.0,

  // ------------------------
  // Dynamic/Incremental currents (AXI-level approximation)
  // NOTE: These are "activity deltas" on top of leakage.
  // ------------------------
  parameter real I_BG_VDD_A  = 0.020,
  parameter real I_BG_VDDQ_A = 0.005,
  parameter real I_RD_VDD_A  = 0.080,
  parameter real I_RD_VDDQ_A = 0.030,
  parameter real I_WR_VDD_A  = 0.090,
  parameter real I_WR_VDDQ_A = 0.035,

  // ------------------------
  // eMRAM currents (pin/activity-based, also treated as deltas)
  // ------------------------
  parameter real MRAM_I_LEAK_A = 0.0,     // ALWAYS when mem_pwr_on=1
  parameter real MRAM_I_RD_A   = 0.010,   // delta during charged rd cycles
  parameter real MRAM_I_WR_A   = 0.020,   // delta during charged wr cycles

  // charged cycles per op (pin-level abstraction)
  parameter int  MRAM_RD_CYCLES = 2,
  parameter int  MRAM_WR_CYCLES = 100,

  // optional refresh busy injection (counts cycles; does NOT stall AXI)
  parameter bit REF_EN = 0,
  parameter int REF_PERIOD_CYC = 1560, // ~7.8us @200MHz
  parameter int REF_BUSY_CYC   = 32,
  // ------------------------
  // Weight-range tracking (for weight-only energy accounting)
  // If WEIGHT_BYTES == 0, tracking is disabled.
  // ------------------------
  parameter logic [ADDR_W-1:0] WEIGHT_BASE  = '0,
  parameter int unsigned       WEIGHT_BYTES = 0
)(
  input  logic clk,
  input  logic rst_n,

  // If 0: leakage/static is removed (e.g., power-gated domain).
  input  logic mem_pwr_on,

  // AXI slave observation
  input  logic [ID_W-1:0]   s_awid,
  input  logic [ADDR_W-1:0] s_awaddr,
  input  logic [7:0]        s_awlen,
  input  logic              s_awvalid,
  input  logic              s_awready,

  input  logic [DATA_W-1:0] s_wdata,
  input  logic [(DATA_W/8)-1:0] s_wstrb,
  input  logic              s_wlast,
  input  logic              s_wvalid,
  input  logic              s_wready,

  input  logic [ID_W-1:0]   s_bid,
  input  logic [1:0]        s_bresp,
  input  logic              s_bvalid,
  input  logic              s_bready,

  input  logic [ID_W-1:0]   s_arid,
  input  logic [ADDR_W-1:0] s_araddr,
  input  logic [7:0]        s_arlen,
  input  logic              s_arvalid,
  input  logic              s_arready,

  input  logic [ID_W-1:0]   s_rid,
  input  logic [DATA_W-1:0] s_rdata,
  input  logic [1:0]        s_rresp,
  input  logic              s_rlast,
  input  logic              s_rvalid,
  input  logic              s_rready,

  // eMRAM pin/activity observation (tie off if not used)
  input  logic m_cs,
  input  logic m_we_en,
  input  logic m_rd_en,

  // control pulses (tie off if not used)
  input  logic clear_pulse,
  input  logic report_pulse,

  // Last reported values (updated on report_pulse)
  output real o_t_total_s,
  output real o_e_ddr_j,
  output real o_e_mram_j,
  output real o_e_total_j,
  output real o_p_ddr_w,
  output real o_p_mram_w,
  output real o_p_total_w
);

  // ------------------------
  // Handshake detects
  // ------------------------
  wire wr_xfer = s_wvalid && s_wready;
  wire rd_xfer = s_rvalid && s_rready;

  // ------------------------
// Weight-range decode
// ------------------------
function automatic bit in_weight_range(input logic [ADDR_W-1:0] a);
  if (WEIGHT_BYTES == 0) return 1'b0;
  return (a >= WEIGHT_BASE) && (a < (WEIGHT_BASE + WEIGHT_BYTES));
endfunction

// ------------------------
// Lightweight address FIFOs to associate R/W beats with AW/AR addresses
// (needed because AXI R/W channels do not carry address)
// ------------------------
localparam int unsigned _QDEPTH = 8;

// NOTE: Icarus Verilog has limited support for arrays-of-struct assignments.
// Use parallel arrays for a tiny beat queue instead.
logic [ADDR_W-1:0] rdq_addr      [_QDEPTH];
logic [8:0]        rdq_beats_left[_QDEPTH];
logic              rdq_is_wgt    [_QDEPTH];
logic [ADDR_W-1:0] wrq_addr      [_QDEPTH];
logic [8:0]        wrq_beats_left[_QDEPTH];
logic              wrq_is_wgt    [_QDEPTH];
logic [$clog2(_QDEPTH):0] rdq_count, wrq_count;
logic [$clog2(_QDEPTH)-1:0] rdq_head, rdq_tail;
logic [$clog2(_QDEPTH)-1:0] wrq_head, wrq_tail;

// Weight-only counters (beats/cycles)
logic [63:0] wgt_rd_beats;
logic [63:0] wgt_wr_beats;
logic [63:0] wgt_rd_cycles;
logic [63:0] wgt_wr_cycles;

// ------------------------
  // Counters (use 64b to avoid overflow)
  // ------------------------
  logic [63:0] cycles_total;
  logic [63:0] cycles_on;
  logic [63:0] cycles_bg;
  logic [63:0] cycles_rd;
  logic [63:0] cycles_wr;

  logic [63:0] rd_beats;
  logic [63:0] wr_beats;
  logic [63:0] aw_cnt;
  logic [63:0] ar_cnt;

  logic [63:0] mram_wr_ops;
  logic [63:0] mram_rd_ops;
  logic [63:0] mram_wr_cycles;
  logic [63:0] mram_rd_cycles;

  logic [63:0] ref_timer;
  logic [63:0] ref_busy_left;
  logic [63:0] ref_cycles;

  // MRAM edge detect to avoid multi-cycle enable double charging
  logic prev_mram_wr_cond;
  logic prev_mram_rd_cond;
  wire  mram_wr_cond = (m_cs && m_we_en);
  wire  mram_rd_cond = (m_cs && m_rd_en);
  wire  mram_wr_fire = mem_pwr_on && mram_wr_cond && !prev_mram_wr_cond;
  wire  mram_rd_fire = mem_pwr_on && mram_rd_cond && !prev_mram_rd_cond;

  // ------------------------
  // Accumulation temporaries (computed on report_pulse)
  // ------------------------
  real t_total_s;
  real e_vdd_j, e_vddq_j, e_ddr_j;
  real e_wgt_vdd_j, e_wgt_vddq_j, e_wgt_access_j;
  real e_mram_j;
  real e_total_j;
  real p_avg_w, p_ddr_w, p_mram_w;
  integer wgt_mram_rd_cycles_i;
  integer wgt_mram_wr_cycles_i;
  reg     use_mram_wgt;

  // ------------------------
  // Main counters (report timing independent)
  // ------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // outputs
      o_t_total_s <= 0.0;
      o_e_ddr_j   <= 0.0;
      o_e_mram_j  <= 0.0;
      o_e_total_j <= 0.0;
      o_p_ddr_w   <= 0.0;
      o_p_mram_w  <= 0.0;
      o_p_total_w <= 0.0;

      // counters
      cycles_total <= 0;
      cycles_on    <= 0;
      cycles_bg    <= 0;
      cycles_rd    <= 0;
      cycles_wr    <= 0;

      rd_beats <= 0;
      wr_beats <= 0;
      aw_cnt   <= 0;
      ar_cnt   <= 0;

      mram_wr_ops    <= 0;
      mram_rd_ops    <= 0;
      mram_wr_cycles <= 0;
      mram_rd_cycles <= 0;

// weight tracking
wgt_rd_beats  <= 0;
wgt_wr_beats  <= 0;
wgt_rd_cycles <= 0;
wgt_wr_cycles <= 0;

rdq_count <= 0;
wrq_count <= 0;
rdq_head  <= '0;
rdq_tail  <= '0;
wrq_head  <= '0;
wrq_tail  <= '0;

      ref_timer     <= 0;
      ref_busy_left <= 0;
      ref_cycles    <= 0;

      prev_mram_wr_cond <= 1'b0;
      prev_mram_rd_cond <= 1'b0;

    end else begin
      // clear
      if (clear_pulse) begin
        o_t_total_s <= 0.0;
        o_e_ddr_j   <= 0.0;
        o_e_mram_j  <= 0.0;
        o_e_total_j <= 0.0;
        o_p_ddr_w   <= 0.0;
        o_p_mram_w  <= 0.0;
        o_p_total_w <= 0.0;

        cycles_total <= 0;
        cycles_on    <= 0;
        cycles_bg    <= 0;
        cycles_rd    <= 0;
        cycles_wr    <= 0;

        rd_beats <= 0;
        wr_beats <= 0;
        aw_cnt   <= 0;
        ar_cnt   <= 0;

        mram_wr_ops    <= 0;
        mram_rd_ops    <= 0;
        mram_wr_cycles <= 0;
        mram_rd_cycles <= 0;

// weight tracking
wgt_rd_beats  <= 0;
wgt_wr_beats  <= 0;
wgt_rd_cycles <= 0;
wgt_wr_cycles <= 0;

rdq_count <= 0;
wrq_count <= 0;
rdq_head  <= '0;
rdq_tail  <= '0;
wrq_head  <= '0;
wrq_tail  <= '0;

        ref_timer     <= 0;
        ref_busy_left <= 0;
        ref_cycles    <= 0;

        prev_mram_wr_cond <= 1'b0;
        prev_mram_rd_cond <= 1'b0;
      end

      // total wall time always advances
      cycles_total <= cycles_total + 1;

      // edge detect history
      prev_mram_wr_cond <= mram_wr_cond;
      prev_mram_rd_cond <= mram_rd_cond;

      // When power is OFF: do not accumulate any energy-related buckets
      if (mem_pwr_on) begin
        cycles_on <= cycles_on + 1;

        // refresh model (only when on)
        if (REF_EN) begin
          if (ref_busy_left != 0) begin
            ref_busy_left <= ref_busy_left - 1;
            ref_cycles    <= ref_cycles + 1;
          end else begin
            if (ref_timer >= (REF_PERIOD_CYC-1)) begin
              ref_timer     <= 0;
              ref_busy_left <= REF_BUSY_CYC;
            end else begin
              ref_timer <= ref_timer + 1;
            end
          end
        end

        // AXI activity buckets (RD/WR independent)
        if (rd_xfer) begin
          cycles_rd <= cycles_rd + 1;
          rd_beats  <= rd_beats  + 1;
        end

        if (wr_xfer) begin
          cycles_wr <= cycles_wr + 1;
          wr_beats  <= wr_beats  + 1;
        end

        if (!rd_xfer && !wr_xfer) begin
          cycles_bg <= cycles_bg + 1;
        end

        if (s_awvalid && s_awready) aw_cnt <= aw_cnt + 1;
        if (s_arvalid && s_arready) ar_cnt <= ar_cnt + 1;

// ------------------------
// Track address context for weight-only accounting
// ------------------------
if (s_arvalid && s_arready) begin
  if (rdq_count < _QDEPTH) begin
    rdq_addr[rdq_tail]       <= s_araddr;
    rdq_beats_left[rdq_tail] <= {1'b0, s_arlen} + 9'd1;
    rdq_is_wgt[rdq_tail]     <= in_weight_range(s_araddr);
    rdq_tail  <= rdq_tail + 1'b1;
    rdq_count <= rdq_count + 1'b1;
  end
end

if (s_awvalid && s_awready) begin
  if (wrq_count < _QDEPTH) begin
    wrq_addr[wrq_tail]       <= s_awaddr;
    wrq_beats_left[wrq_tail] <= {1'b0, s_awlen} + 9'd1;
    wrq_is_wgt[wrq_tail]     <= 1'b0; // disable weight-write tracking
    wrq_tail  <= wrq_tail + 1'b1;
    wrq_count <= wrq_count + 1'b1;
  end
end

// Consume one beat on each successful R/W transfer
if (rd_xfer && (rdq_count != 0)) begin
  if (rdq_is_wgt[rdq_head]) begin
    wgt_rd_beats  <= wgt_rd_beats + 1;
    wgt_rd_cycles <= wgt_rd_cycles + 1;
  end
  // advance beat
  if (rdq_beats_left[rdq_head] <= 9'd1) begin
    rdq_head  <= rdq_head + 1'b1;
    rdq_count <= rdq_count - 1'b1;
  end else begin
    rdq_beats_left[rdq_head] <= rdq_beats_left[rdq_head] - 9'd1;
    rdq_addr[rdq_head]       <= rdq_addr[rdq_head] + (DATA_W/8);
  end
end

if (wr_xfer && (wrq_count != 0)) begin
  // We do NOT count weight writes for this workload (weights are fetched via reads).
  // Keeping this disabled avoids accidental overlap with activation/frame buffers.
  if (wrq_beats_left[wrq_head] <= 9'd1) begin
    wrq_head  <= wrq_head + 1'b1;
    wrq_count <= wrq_count - 1'b1;
  end else begin
    wrq_beats_left[wrq_head] <= wrq_beats_left[wrq_head] - 9'd1;
    wrq_addr[wrq_head]       <= wrq_addr[wrq_head] + (DATA_W/8);
  end
end

        // eMRAM pin-based charging (edge-based)
        if (mram_wr_fire) begin
          mram_wr_ops    <= mram_wr_ops + 1;
          mram_wr_cycles <= mram_wr_cycles + MRAM_WR_CYCLES;
        end

        if (mram_rd_fire) begin
          mram_rd_ops    <= mram_rd_ops + 1;
          mram_rd_cycles <= mram_rd_cycles + MRAM_RD_CYCLES;
        end

      end else begin
        // power OFF: no refresh should accrue
        ref_timer     <= 0;
        ref_busy_left <= 0;
        // ref_cycles is historical; keep as-is
      end

      // ------------------------
      // report (purely computes from accumulated counters)
      // ------------------------
      if (report_pulse) begin
        t_total_s = cycles_total * TCLK_S;

        // DDR-like energy model:
        // leakage uses cycles_on (NOT cycles_total) -> report timing independent.
        e_vdd_j  = VDD  * ( (cycles_on * I_LEAK_VDD_A)
                          + (cycles_bg * I_BG_VDD_A)
                          + (cycles_rd * I_RD_VDD_A)
                          + (cycles_wr * I_WR_VDD_A) ) * TCLK_S;

        e_vddq_j = VDDQ * ( (cycles_on * I_LEAK_VDDQ_A)
                          + (cycles_bg * I_BG_VDDQ_A)
                          + (cycles_rd * I_RD_VDDQ_A)
                          + (cycles_wr * I_WR_VDDQ_A) ) * TCLK_S;

        e_ddr_j  = e_vdd_j + e_vddq_j;

        // eMRAM energy model:
        // leakage uses cycles_on; rd/wr uses charged delta cycles
        e_mram_j = VDD * ( (cycles_on * MRAM_I_LEAK_A)
                         + (mram_rd_cycles * MRAM_I_RD_A)
                         + (mram_wr_cycles * MRAM_I_WR_A) ) * TCLK_S;

        e_total_j = e_ddr_j + e_mram_j;

        p_avg_w  = (t_total_s > 0.0) ? (e_total_j / t_total_s) : 0.0;
        p_ddr_w  = (t_total_s > 0.0) ? (e_ddr_j   / t_total_s) : 0.0;
        p_mram_w = (t_total_s > 0.0) ? (e_mram_j  / t_total_s) : 0.0;

        // latch last report to outputs
        o_t_total_s <= t_total_s;
        o_e_ddr_j   <= e_ddr_j;
        o_e_mram_j  <= e_mram_j;
        o_e_total_j <= e_total_j;
        o_p_total_w <= p_avg_w;
        o_p_ddr_w   <= p_ddr_w;
        o_p_mram_w  <= p_mram_w;

        $display("------------------------------------------------------------");
        $display("[PWR] cycles_total=%0d  cycles_on=%0d  t_total=%.6e s (F=%.3f MHz)",
                 cycles_total, cycles_on, t_total_s, FCLK_HZ/1e6);
        $display("[PWR] AXI: aw_cnt=%0d ar_cnt=%0d wr_beats=%0d rd_beats=%0d", aw_cnt, ar_cnt, wr_beats, rd_beats);
// Weight-only dynamic access energy within [WEIGHT_BASE, WEIGHT_BASE+WEIGHT_BYTES)
// NOTE: For DDR-like monitors, use VDD/VDDQ + DDR delta currents.
//       For MRAM monitor (VDDQ=0, DDR currents=0, MRAM currents non-zero), use MRAM pin-current model.

use_mram_wgt = (WEIGHT_BYTES != 0) &&
               (MRAM_I_RD_A != 0.0 || MRAM_I_WR_A != 0.0) &&
               (I_RD_VDD_A  == 0.0 && I_WR_VDD_A  == 0.0 && VDDQ == 0.0);

wgt_mram_rd_cycles_i = wgt_rd_beats * MRAM_RD_CYCLES;
wgt_mram_wr_cycles_i = wgt_wr_beats * MRAM_WR_CYCLES;

e_wgt_vdd_j  = VDD  * ( (wgt_rd_cycles * I_RD_VDD_A)  + (wgt_wr_cycles * I_WR_VDD_A) ) * TCLK_S;
e_wgt_vddq_j = VDDQ * ( (wgt_rd_cycles * I_RD_VDDQ_A) + (wgt_wr_cycles * I_WR_VDDQ_A) ) * TCLK_S;

if (use_mram_wgt) begin
  e_wgt_access_j = VDD * ( (wgt_mram_rd_cycles_i * MRAM_I_RD_A) + (wgt_mram_wr_cycles_i * MRAM_I_WR_A) ) * TCLK_S;
end else begin
  e_wgt_access_j = e_wgt_vdd_j + e_wgt_vddq_j;
end

$display("[PWR][wgt] base=0x%08x bytes=%0d  wgt_rd_beats=%0d wgt_wr_beats=%0d  E_access=%.6e J",
         WEIGHT_BASE, WEIGHT_BYTES, wgt_rd_beats, wgt_wr_beats, e_wgt_access_j);
        $display("[PWR] AXI cycles(on): BG=%0d RD=%0d WR=%0d REF=%0d", cycles_bg, cycles_rd, cycles_wr, ref_cycles);
        $display("[PWR] MRAM ops: rd_ops=%0d wr_ops=%0d  charged_cycles: rd=%0d wr=%0d",
                 mram_rd_ops, mram_wr_ops, mram_rd_cycles, mram_wr_cycles);
        $display("[PWR] Energy(J): DDR_like=%.6e  eMRAM=%.6e  TOTAL=%.6e", e_ddr_j, e_mram_j, e_total_j);
        $display("[PWR] AvgPow(W): TOTAL=%.6e  DDR_like=%.6e  eMRAM=%.6e", p_avg_w, p_ddr_w, p_mram_w);
        // NOTE: Print *effective* leakage currents that are actually applied in the energy integration.
        // When mem_pwr_on==0, cycles_on does not advance and leakage energy is not accumulated.
        // Showing the raw parameter here is misleading, so we gate the printed value.
        $display("[PWR] Leakage(A): I_LEAK_VDD=%.6e I_LEAK_VDDQ=%.6e MRAM_I_LEAK=%.6e (mem_pwr_on_now=%0d)",
                 I_LEAK_VDD_A, I_LEAK_VDDQ_A,
                 (mem_pwr_on ? MRAM_I_LEAK_A : 0.0),
                 mem_pwr_on);
        $display("------------------------------------------------------------");
      end
    end
  end

endmodule
