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
// AXI MRAM SLAVE CONTROLLER (FIXED READ FSM)
// ============================================================

module axi_mram_slave_ctrl #(
  parameter int AXI_ID_WIDTH    = 4,
  parameter int AXI_ADDR_WIDTH  = 32,
  parameter int AXI_DATA_WIDTH  = 64,
  parameter int BURST_LEN       = 16,
  parameter int MRAM_READ_LAT   = 2,
  // Read-data FIFO to absorb AXI backpressure (default 32-deep)
  parameter int RD_DATA_FIFO_DEPTH = 32
)(
  input  logic clk,
  input  logic rst_n,
  input  logic [13:0] write_delay_config,

  // AXI SLAVE
  input  logic [AXI_ID_WIDTH-1:0]   awid,
  input  logic [AXI_ADDR_WIDTH-1:0] awaddr,
  input  logic [7:0]                awlen,
  input  logic                      awvalid,
  output logic                      awready,

  input  logic [AXI_DATA_WIDTH-1:0] wdata,
  input  logic                      wlast,
  input  logic                      wvalid,
  output logic                      wready,

  output logic [AXI_ID_WIDTH-1:0]   bid,
  output logic [1:0]                bresp,
  output logic                      bvalid,
  input  logic                      bready,

  input  logic [AXI_ID_WIDTH-1:0]   arid,
  input  logic [AXI_ADDR_WIDTH-1:0] araddr,
  input  logic [7:0]                arlen,
  input  logic                      arvalid,
  output logic                      arready,

  output logic [AXI_ID_WIDTH-1:0]   rid,
  output logic [AXI_DATA_WIDTH-1:0] rdata,
  output logic [1:0]                rresp,
  output logic                      rvalid,
  output logic                      rlast,
  input  logic                      rready,

  // MRAM port
  output logic [AXI_ADDR_WIDTH-1:0] mram_addr,
  output logic [AXI_DATA_WIDTH-1:0] mram_wdata,
  output logic                      mram_write_en,
  output logic                      mram_read_en,
  output logic                      mram_cs,
  input  logic [AXI_DATA_WIDTH-1:0] mram_rdata,
  input  logic                      mram_ready,
  input  logic                      mram_pwr_on
);

  // ============================================================
  // Address request queues (simple in-order)
  // ============================================================
  localparam int WRQ_DEPTH = 8;
  localparam int RDQ_DEPTH = 8;

  logic [AXI_ID_WIDTH-1:0]     wrq_id   [WRQ_DEPTH];
  logic [AXI_ADDR_WIDTH-1:0]   wrq_addr [WRQ_DEPTH];
  logic [7:0]                  wrq_len  [WRQ_DEPTH];
  logic [$clog2(WRQ_DEPTH):0]  wr_q_cnt;
  logic [$clog2(WRQ_DEPTH)-1:0] wrq_wr_ptr, wrq_rd_ptr;

  logic [AXI_ID_WIDTH-1:0]     rdq_id   [RDQ_DEPTH];
  logic [AXI_ADDR_WIDTH-1:0]   rdq_addr [RDQ_DEPTH];
  logic [7:0]                  rdq_len  [RDQ_DEPTH];
  logic [$clog2(RDQ_DEPTH):0]  rd_q_cnt;
  logic [$clog2(RDQ_DEPTH)-1:0] rdq_wr_ptr, rdq_rd_ptr;

  assign awready = (wr_q_cnt < WRQ_DEPTH);
  assign arready = (rd_q_cnt < RDQ_DEPTH);

  wire push_aw = awvalid && awready;
  wire push_ar = arvalid && arready;

  // ============================================================
  // WRITE FSM (keep behavior as-is; do not change)
  // ============================================================
  typedef enum logic [1:0] {WR_IDLE, WR_DATA, WR_RESP} wr_state_t;
  wr_state_t wr_state;

  logic [AXI_ID_WIDTH-1:0]   wr_id;
  logic [AXI_ADDR_WIDTH-1:0] wr_addr;
  logic [7:0]                wr_rem;

  // ============================================================
  // READ PIPELINED ENGINE (clean split: request issue vs AXI R send)
  // - MRAM read data appears MRAM_READ_LAT cycles after accepted read
  // - Next MRAM address can be issued without waiting for previous data
  // - Read-data FIFO absorbs AXI backpressure
  // ============================================================
  logic                       rd_active;
  logic [AXI_ID_WIDTH-1:0]    rd_id;
  logic [AXI_ADDR_WIDTH-1:0]  rd_addr;
  logic [7:0]                 rd_beats_left; // beats remaining in current burst (1..N)

  // Issue-side pipeline metadata (aligned to mram_rdata)
  logic [MRAM_READ_LAT-1:0]                 rd_vld_sh;
  logic [MRAM_READ_LAT-1:0][AXI_ID_WIDTH-1:0] rd_id_sh;
  logic [MRAM_READ_LAT-1:0]                 rd_last_sh;

  // Read-data FIFO (stores {id,last,data})
  localparam int RD_FIFO_W = AXI_ID_WIDTH + 1 + AXI_DATA_WIDTH;
  logic [RD_FIFO_W-1:0] rd_fifo_mem [RD_DATA_FIFO_DEPTH];
  localparam int RD_FIFO_PTR_W = (RD_DATA_FIFO_DEPTH <= 2) ? 1 : $clog2(RD_DATA_FIFO_DEPTH);
  logic [RD_FIFO_PTR_W-1:0] rd_fifo_wptr, rd_fifo_rptr;
  logic [$clog2(RD_DATA_FIFO_DEPTH+1)-1:0] rd_fifo_cnt;

  wire rd_fifo_empty = (rd_fifo_cnt == 0);
  wire rd_fifo_full  = (rd_fifo_cnt == RD_DATA_FIFO_DEPTH);

  // conservative "free slots" check so that in-flight pipeline returns can't overflow
  int unsigned rd_fifo_free;
  always @* rd_fifo_free = RD_DATA_FIFO_DEPTH - rd_fifo_cnt;

  wire can_issue_read = rd_active &&
                        mram_pwr_on &&
                        mram_ready &&
                        (rd_fifo_free > MRAM_READ_LAT); // keep > LAT free slots

  wire do_issue = can_issue_read; // 1-beat per cycle issue when allowed

  // FIFO push when MRAM data returns (metadata delayed MRAM_READ_LAT)
  wire pipe_ret_vld = rd_vld_sh[MRAM_READ_LAT-1];
  wire pipe_ret_last= rd_last_sh[MRAM_READ_LAT-1];
  wire [AXI_ID_WIDTH-1:0] pipe_ret_id = rd_id_sh[MRAM_READ_LAT-1];

  // ------------------------------------------------------------
  // Return-data alignment
  // ------------------------------------------------------------
  // The mram_model provided in this file updates mram_rdata on the same
  // rising clock edge where the READ_LAT pipeline address reaches the last
  // stage. If we push into the FIFO on that same edge, we would capture the
  // *previous* cycle's mram_rdata due to nonblocking assignment ordering.
  // Therefore, delay the "return valid"/metadata by 1 cycle and push using
  // the stable mram_rdata value observed in the following cycle.
  logic                     ret_vld_d;
  logic [AXI_ID_WIDTH-1:0]   ret_id_d;
  logic                     ret_last_d;

  wire fifo_push = ret_vld_d && !rd_fifo_full;
  wire fifo_pop  = !rd_fifo_empty && rready; // when presenting rvalid

  // ============================================================
  // MRAM signals
  // ============================================================
  logic [AXI_ADDR_WIDTH-1:0] mram_addr_r;
  logic [AXI_DATA_WIDTH-1:0] mram_wdata_r;
  logic                      mram_we_r;
  logic                      mram_re_r;

  assign mram_addr     = mram_addr_r;
  assign mram_wdata    = mram_wdata_r;
  assign mram_write_en = mram_we_r;
  assign mram_read_en  = mram_re_r;
  assign mram_cs       = mram_pwr_on && (mram_we_r || mram_re_r);

  // ============================================================
  // Combinational outputs
  // ============================================================
  always @* begin
    // defaults
    wready = 1'b0;
    bvalid = 1'b0;
    bid    = wr_id;
    bresp  = 2'b00;

    // R comes from FIFO
    rvalid = !rd_fifo_empty;
    rresp  = 2'b00;
    {rid, rlast, rdata} = rd_fifo_empty ? '0 : rd_fifo_mem[rd_fifo_rptr];

    // default MRAM controls
    mram_addr_r  = '0;
    mram_wdata_r = '0;
    mram_we_r    = 1'b0;
    mram_re_r    = 1'b0;

    // WRITE (unchanged behavior)
    if ((wr_state == WR_DATA) && wvalid && mram_pwr_on && mram_ready) begin
      wready        = 1'b1;
      mram_we_r     = 1'b1;
      mram_addr_r   = wr_addr;
      mram_wdata_r  = wdata;
    end
    if (wr_state == WR_RESP) begin
      bvalid = 1'b1;
    end

    // READ issue: pulse read enable; address can advance every cycle (pipelined)
    if (do_issue) begin
      mram_re_r    = 1'b1;
      mram_addr_r  = rd_addr;
    end
  end

  // ============================================================
  // Sequential logic
  // ============================================================
  integer k;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // queues
      wr_q_cnt    <= '0;
      rd_q_cnt    <= '0;
      wrq_wr_ptr  <= '0;
      wrq_rd_ptr  <= '0;
      rdq_wr_ptr  <= '0;
      rdq_rd_ptr  <= '0;

      // write
      wr_state <= WR_IDLE;
      wr_id    <= '0;
      wr_addr  <= '0;
      wr_rem   <= '0;

      // read engine
      rd_active     <= 1'b0;
      rd_id         <= '0;
      rd_addr       <= '0;
      rd_beats_left <= '0;

      // return-data align regs
      ret_vld_d  <= 1'b0;
      ret_id_d   <= '0;
      ret_last_d <= 1'b0;

      rd_vld_sh  <= '0;
      rd_last_sh <= '0;
      for (k=0; k<MRAM_READ_LAT; k=k+1) begin
        rd_id_sh[k] <= '0;
      end

      // rd fifo
      rd_fifo_wptr <= '0;
      rd_fifo_rptr <= '0;
      rd_fifo_cnt  <= '0;
    end else begin
      // ----------------------------
      // Queue push
      // ----------------------------
      if (push_aw) begin
        wrq_id  [wrq_wr_ptr] <= awid;
        wrq_addr[wrq_wr_ptr] <= awaddr;
        wrq_len [wrq_wr_ptr] <= awlen;
        wrq_wr_ptr <= wrq_wr_ptr + 1'b1;
        wr_q_cnt   <= wr_q_cnt   + 1'b1;
      end

      if (push_ar) begin
        rdq_id  [rdq_wr_ptr] <= arid;
        rdq_addr[rdq_wr_ptr] <= araddr;
        rdq_len [rdq_wr_ptr] <= arlen;
        rdq_wr_ptr <= rdq_wr_ptr + 1'b1;
        rd_q_cnt   <= rd_q_cnt   + 1'b1;
      end

      // ----------------------------
      // WRITE FSM (unchanged)
      // ----------------------------
      case (wr_state)
        WR_IDLE: if (wr_q_cnt != 0) begin
          wr_id   <= wrq_id  [wrq_rd_ptr];
          wr_addr <= wrq_addr[wrq_rd_ptr];
          wr_rem  <= wrq_len [wrq_rd_ptr];
          wrq_rd_ptr <= wrq_rd_ptr + 1'b1;
          wr_q_cnt   <= wr_q_cnt   - 1'b1;
          wr_state   <= WR_DATA;
        end

        WR_DATA: if (wvalid && wready) begin
          wr_addr <= wr_addr + (AXI_DATA_WIDTH/8);
          if (wr_rem == 0) wr_state <= WR_RESP;
          else wr_rem <= wr_rem - 1'b1;
        end

        WR_RESP: if (bvalid && bready) begin
          wr_state <= WR_IDLE;
        end
      endcase

      // ----------------------------
      // READ: load a new burst when idle
      // ----------------------------
      if (!rd_active && (rd_q_cnt != 0)) begin
        rd_id         <= rdq_id  [rdq_rd_ptr];
        rd_addr       <= rdq_addr[rdq_rd_ptr];
        rd_beats_left <= rdq_len [rdq_rd_ptr] + 8'd1; // beats = arlen+1
        rdq_rd_ptr    <= rdq_rd_ptr + 1'b1;
        rd_q_cnt      <= rd_q_cnt   - 1'b1;
        rd_active     <= 1'b1;
      end

      // ----------------------------
      // READ: issue side (pipelined addresses)
      // ----------------------------
      if (do_issue) begin
        // advance for next beat
        rd_addr <= rd_addr + (AXI_DATA_WIDTH/8);
        if (rd_beats_left == 8'd1) begin
          rd_beats_left <= '0;
          rd_active <= 1'b0; // done issuing this burst
        end else begin
          rd_beats_left <= rd_beats_left - 1'b1;
        end
      end

      // metadata shift registers (align with mram_rdata)
      // stage 0
      rd_vld_sh[0]  <= do_issue;
      rd_id_sh[0]   <= rd_id;
      rd_last_sh[0] <= do_issue && (rd_beats_left == 8'd1);

      // stages 1..LAT-1
      for (k=1; k<MRAM_READ_LAT; k=k+1) begin
        rd_vld_sh[k]  <= rd_vld_sh[k-1];
        rd_id_sh[k]   <= rd_id_sh[k-1];
        rd_last_sh[k] <= rd_last_sh[k-1];
      end

      // delay return-valid/metadata by 1 cycle to align with registered mram_rdata
      ret_vld_d  <= pipe_ret_vld;
      ret_id_d   <= pipe_ret_id;
      ret_last_d <= pipe_ret_last;

      // ----------------------------
      // RD FIFO push/pop
      // ----------------------------
      // push returned data
      if (fifo_push) begin
        rd_fifo_mem[rd_fifo_wptr] <= {ret_id_d, ret_last_d, mram_rdata};
        if (rd_fifo_wptr == RD_DATA_FIFO_DEPTH-1) rd_fifo_wptr <= '0;
        else rd_fifo_wptr <= rd_fifo_wptr + 1'b1;
      end
      // pop on AXI accept
      if (rvalid && rready) begin
        if (rd_fifo_rptr == RD_DATA_FIFO_DEPTH-1) rd_fifo_rptr <= '0;
        else rd_fifo_rptr <= rd_fifo_rptr + 1'b1;
      end

      // update count (supports simultaneous push+pop)
      case ({fifo_push, (rvalid && rready)})
        2'b10: rd_fifo_cnt <= rd_fifo_cnt + 1'b1;
        2'b01: rd_fifo_cnt <= rd_fifo_cnt - 1'b1;
        default: rd_fifo_cnt <= rd_fifo_cnt;
      endcase
    end
  end

endmodule
