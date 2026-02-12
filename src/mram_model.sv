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





// ============================================================
// MRAM model (sync write, fixed latency read)
// ============================================================
module mram_model #(
  parameter int ADDR_WIDTH   = 32,
  parameter int DATA_WIDTH   = 64,
  parameter int DEPTH_WORDS  = 32768,
  parameter int READ_LAT     = 2,
  parameter int WRITE_CYCLES = 100
)(
  input  logic clk,
  input  logic rst_n,

  input  logic [ADDR_WIDTH-1:0] mram_addr,
  input  logic [DATA_WIDTH-1:0] mram_wdata,
  input  logic                  mram_write_en,
  input  logic                  mram_read_en,
  input  logic                  mram_cs,

  output logic [DATA_WIDTH-1:0] mram_rdata,
  output logic                  mram_ready,

  input  logic                  mram_pwr_on
);

  // memory array
  logic [DATA_WIDTH-1:0] mem [0:DEPTH_WORDS-1];

  // word index
  wire [31:0] word_index = (mram_addr >> $clog2(DATA_WIDTH/8));

  // ------------------------------------------------------------
  // Busy model: single-port MRAM
  // - write_en starts a 100-cycle commit window (busy)
  // - while busy: mram_ready=0, no new read/write accepted
  // ------------------------------------------------------------
  logic                      busy;
  int unsigned               wr_cnt;
  logic [31:0]               wr_addr_q;
  logic [DATA_WIDTH-1:0]     wr_data_q;

  // Read pipeline (2-cycle latency after accepted read)
  logic [31:0] rd_q [0:READ_LAT-1];

  integer i, k;

  // ✅ ready only when powered + not busy
  always @* begin
    mram_ready = mram_pwr_on && !busy;
  end

  // helper: treat cs as valid only when op happens
  wire do_wr = mram_pwr_on && mram_cs && mram_write_en && mram_ready;
  wire do_rd = mram_pwr_on && mram_cs && mram_read_en  && mram_ready;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy      <= 1'b0;
      wr_cnt    <= 0;
      wr_addr_q <= 0;
      wr_data_q <= '0;

      mram_rdata <= '0;

      for (i=0; i<DEPTH_WORDS; i=i+1) mem[i] <= '0;
      for (k=0; k<READ_LAT; k=k+1) rd_q[k] <= 32'hFFFF_FFFF;
    end else begin
      // -----------------------------
      // WRITE: latch request then commit after WRITE_CYCLES clocks
      // -----------------------------
      if (!busy) begin
        if (do_wr) begin
          busy      <= 1'b1;
          wr_cnt    <= WRITE_CYCLES;  // countdown to commit
          wr_addr_q <= word_index;
          wr_data_q <= mram_wdata;
        end
      end else begin
        if (wr_cnt != 0) wr_cnt <= wr_cnt - 1;

        // commit when counter reaches 1 -> next cycle would be 0
        if (wr_cnt == 1) begin
          if (wr_addr_q < DEPTH_WORDS) begin
            mem[wr_addr_q] <= wr_data_q;
          end
          busy <= 1'b0;
        end
      end

      // -----------------------------
      // READ: accept only when not busy
      // data appears after READ_LAT cycles
      // -----------------------------
      if (do_rd) begin
        rd_q[0] <= word_index;
      end else begin
        // mark "no read" bubble to keep deterministic behavior
        rd_q[0] <= 32'hFFFF_FFFF;
      end

      for (k=1; k<READ_LAT; k=k+1) begin
        rd_q[k] <= rd_q[k-1];
      end

      if (rd_q[READ_LAT-1] != 32'hFFFF_FFFF) begin
        if (rd_q[READ_LAT-1] < DEPTH_WORDS)
          mram_rdata <= mem[rd_q[READ_LAT-1]];
        else
          mram_rdata <= '0;
      end
    end
  end

endmodule
