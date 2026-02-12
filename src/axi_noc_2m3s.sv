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


module axi_noc_2m3s #(
  parameter int AXI_ID_WIDTH   = 4,
  parameter int AXI_ADDR_WIDTH = 32,
  parameter int AXI_DATA_WIDTH = 64
)(
  input  logic                      clk,
  input  logic                      rst_n,

  // -------------------------
  // M0 AXI (no WSTRB in this simplified fabric)
  // -------------------------
  input  logic [AXI_ID_WIDTH-1:0]    m0_awid,
  input  logic [AXI_ADDR_WIDTH-1:0]  m0_awaddr,
  input  logic [7:0]                 m0_awlen,
  input  logic                       m0_awvalid,
  output logic                       m0_awready,

  input  logic [AXI_DATA_WIDTH-1:0]  m0_wdata,
  input  logic                       m0_wlast,
  input  logic                       m0_wvalid,
  output logic                       m0_wready,

  output logic [AXI_ID_WIDTH-1:0]    m0_bid,
  output logic [1:0]                 m0_bresp,
  output logic                       m0_bvalid,
  input  logic                       m0_bready,

  input  logic [AXI_ID_WIDTH-1:0]    m0_arid,
  input  logic [AXI_ADDR_WIDTH-1:0]  m0_araddr,
  input  logic [7:0]                 m0_arlen,
  input  logic                       m0_arvalid,
  output logic                       m0_arready,

  output logic [AXI_ID_WIDTH-1:0]    m0_rid,
  output logic [AXI_DATA_WIDTH-1:0]  m0_rdata,
  output logic [1:0]                 m0_rresp,
  output logic                       m0_rvalid,
  output logic                       m0_rlast,
  input  logic                       m0_rready,

  // -------------------------
  // M1 AXI
  // -------------------------
  input  logic [AXI_ID_WIDTH-1:0]    m1_awid,
  input  logic [AXI_ADDR_WIDTH-1:0]  m1_awaddr,
  input  logic [7:0]                 m1_awlen,
  input  logic                       m1_awvalid,
  output logic                       m1_awready,

  input  logic [AXI_DATA_WIDTH-1:0]  m1_wdata,
  input  logic                       m1_wlast,
  input  logic                       m1_wvalid,
  output logic                       m1_wready,

  output logic [AXI_ID_WIDTH-1:0]    m1_bid,
  output logic [1:0]                 m1_bresp,
  output logic                       m1_bvalid,
  input  logic                       m1_bready,

  input  logic [AXI_ID_WIDTH-1:0]    m1_arid,
  input  logic [AXI_ADDR_WIDTH-1:0]  m1_araddr,
  input  logic [7:0]                 m1_arlen,
  input  logic                       m1_arvalid,
  output logic                       m1_arready,

  output logic [AXI_ID_WIDTH-1:0]    m1_rid,
  output logic [AXI_DATA_WIDTH-1:0]  m1_rdata,
  output logic [1:0]                 m1_rresp,
  output logic                       m1_rvalid,
  output logic                       m1_rlast,
  input  logic                       m1_rready,

  // -------------------------
  // S0 AXI
  // -------------------------
  output logic [AXI_ID_WIDTH-1:0]    s0_awid,
  output logic [AXI_ADDR_WIDTH-1:0]  s0_awaddr,
  output logic [7:0]                 s0_awlen,
  output logic                       s0_awvalid,
  input  logic                       s0_awready,

  output logic [AXI_DATA_WIDTH-1:0]  s0_wdata,
  output logic                       s0_wlast,
  output logic                       s0_wvalid,
  input  logic                       s0_wready,

  input  logic [AXI_ID_WIDTH-1:0]    s0_bid,
  input  logic [1:0]                 s0_bresp,
  input  logic                       s0_bvalid,
  output logic                       s0_bready,

  output logic [AXI_ID_WIDTH-1:0]    s0_arid,
  output logic [AXI_ADDR_WIDTH-1:0]  s0_araddr,
  output logic [7:0]                 s0_arlen,
  output logic                       s0_arvalid,
  input  logic                       s0_arready,

  input  logic [AXI_ID_WIDTH-1:0]    s0_rid,
  input  logic [AXI_DATA_WIDTH-1:0]  s0_rdata,
  input  logic [1:0]                 s0_rresp,
  input  logic                       s0_rvalid,
  input  logic                       s0_rlast,
  output logic                       s0_rready,

  // -------------------------
  // S1 AXI
  // -------------------------
  output logic [AXI_ID_WIDTH-1:0]    s1_awid,
  output logic [AXI_ADDR_WIDTH-1:0]  s1_awaddr,
  output logic [7:0]                 s1_awlen,
  output logic                       s1_awvalid,
  input  logic                       s1_awready,

  output logic [AXI_DATA_WIDTH-1:0]  s1_wdata,
  output logic                       s1_wlast,
  output logic                       s1_wvalid,
  input  logic                       s1_wready,

  input  logic [AXI_ID_WIDTH-1:0]    s1_bid,
  input  logic [1:0]                 s1_bresp,
  input  logic                       s1_bvalid,
  output logic                       s1_bready,

  output logic [AXI_ID_WIDTH-1:0]    s1_arid,
  output logic [AXI_ADDR_WIDTH-1:0]  s1_araddr,
  output logic [7:0]                 s1_arlen,
  output logic                       s1_arvalid,
  input  logic                       s1_arready,

  input  logic [AXI_ID_WIDTH-1:0]    s1_rid,
  input  logic [AXI_DATA_WIDTH-1:0]  s1_rdata,
  input  logic [1:0]                 s1_rresp,
  input  logic                       s1_rvalid,
  input  logic                       s1_rlast,
  output logic                       s1_rready,

  // -------------------------
  // S2 AXI
  // -------------------------
  output logic [AXI_ID_WIDTH-1:0]    s2_awid,
  output logic [AXI_ADDR_WIDTH-1:0]  s2_awaddr,
  output logic [7:0]                 s2_awlen,
  output logic                       s2_awvalid,
  input  logic                       s2_awready,

  output logic [AXI_DATA_WIDTH-1:0]  s2_wdata,
  output logic                       s2_wlast,
  output logic                       s2_wvalid,
  input  logic                       s2_wready,

  input  logic [AXI_ID_WIDTH-1:0]    s2_bid,
  input  logic [1:0]                 s2_bresp,
  input  logic                       s2_bvalid,
  output logic                       s2_bready,

  output logic [AXI_ID_WIDTH-1:0]    s2_arid,
  output logic [AXI_ADDR_WIDTH-1:0]  s2_araddr,
  output logic [7:0]                 s2_arlen,
  output logic                       s2_arvalid,
  input  logic                       s2_arready,

  input  logic [AXI_ID_WIDTH-1:0]    s2_rid,
  input  logic [AXI_DATA_WIDTH-1:0]  s2_rdata,
  input  logic [1:0]                 s2_rresp,
  input  logic                       s2_rvalid,
  input  logic                       s2_rlast,
  output logic                       s2_rready
);

  // ============================================================
  // Address decode
  //  0x0xxx_xxxx -> S0
  //  0x1xxx_xxxx -> S1
  //  0x2xxx_xxxx -> S2
  // default      -> S0
  // ============================================================
  function automatic logic [1:0] dec_slave(input logic [AXI_ADDR_WIDTH-1:0] a);
    case (a[31:28])
      4'h0: dec_slave = 2'd0;
      4'h1: dec_slave = 2'd1;
      4'h2: dec_slave = 2'd2;
      default: dec_slave = 2'd0;
    endcase
  endfunction

  // ============================================================
  // WRITE path: single outstanding (fixed7)
  //   - wr_active ends ONLY on WLAST handshake (never on B)
  //   - fabric always-ready for B to avoid deadlock
  // ============================================================
  logic        wr_active;       // W channel in progress (data beats)
  logic        wr_resp_wait;
  // ------------------------------------------------------------
  // fixed17: AW latch (slave-side one-shot)
  // ------------------------------------------------------------
  logic aw_hold;
  logic [AXI_ID_WIDTH-1:0]   aw_id_r;
  logic [AXI_ADDR_WIDTH-1:0] aw_addr_r;
  logic [7:0]                aw_len_r;
  // ------------------------------------------------------------
  // fixed20: B channel owner latch
  // ------------------------------------------------------------
  logic b_owner; // 0 = M0, 1 = M1

    // waiting for B from selected slave
  logic        wr_owner;        // 0=M0, 1=M1
  logic [1:0]  wr_dst;          // 0/1/2
// ============================================================
// FIX: Write-channel acceptance must match the destination slave.
// - For MRAM (S0): only acknowledge W beats when S0 is ready (prevents dropped beats -> missing B).
// - For other slaves (S1/S2): keep fabric always-ready to avoid deadlock with simple models.
// ============================================================
wire wready_dst;
assign wready_dst = (wr_dst == 2'd0) ? s0_wready : 1'b1;

  logic [1:0]  b_dst;          // latched B response destination

  // FIX: Latch B response destination ONLY when the selected slave accepts AW.
  // This guarantees B routing remains stable even if wr_dst changes for later transactions.
  wire aw_hs_s0 = s0_awvalid && s0_awready;
  wire aw_hs_s1 = s1_awvalid && s1_awready;
  wire aw_hs_s2 = s2_awvalid && s2_awready;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      b_dst <= 2'd0;
    end else begin
      if (aw_hs_s0)      b_dst <= 2'd0;
      else if (aw_hs_s1) b_dst <= 2'd1;
      else if (aw_hs_s2) b_dst <= 2'd2;
    end
  end

  // FIX: latched B response destination (for B routing only)
  logic [7:0]  wr_left;         // beats left (awlen+1 .. 0)
  logic [AXI_ID_WIDTH-1:0] wr_id;
  logic        rr_aw_turn;      // 0: prefer M0 when both valid, 1: prefer M1

  wire         wr_busy = wr_active || wr_resp_wait;
  // ------------------------------------------------------------
  // fixed16: write inflight lock
  // Only allow ONE AW per write transaction
  // ------------------------------------------------------------
  wire write_inflight;
  assign write_inflight = wr_active || wr_resp_wait;

  // ------------------------------------------------------------
  // Global NoC busy lock (fixed12)
  // Serialize R/W to avoid ownership overlap
  // ------------------------------------------------------------
  logic noc_busy;
  // rd_active / rd_resp_wait may have different names; assume rd_active exists
  assign noc_busy = wr_active || wr_resp_wait || rd_active;


  // Destination BVALID (fabric completion only)
  wire bvalid_sel_pre =
      (b_dst==2'd0) ? s0_bvalid :
      (b_dst==2'd1) ? s1_bvalid :
                       s2_bvalid;

  // Accept rule (Round-Robin between M0/M1 when both request):
  wire grant_aw_m0 = (!write_inflight) && m0_awvalid && ( !m1_awvalid || (rr_aw_turn==1'b0) );
  wire grant_aw_m1 = (!write_inflight) && m1_awvalid && ( !m0_awvalid || (rr_aw_turn==1'b1) );

  // NOTE: registered-ready is handled elsewhere for AW; keep interface stable
  // (fixed9) m0_awready/m1_awready are driven by registered versions below
  // Which master is selected for AW? (handshake)
  wire sel_aw_m0 = m0_awvalid && m0_awready;
  wire sel_aw_m1 = m1_awvalid && m1_awready;
  // Registered AWREADY to avoid half-cycle pulse (fixed14)
  // Also prevents duplicate AW handshakes if master holds AWVALID longer than one cycle.
  logic m0_awready_r, m1_awready_r;
  logic aw0_consumed, aw1_consumed;

  
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      aw_hold <= 1'b0;
      m0_awready_r <= 1'b0;
      m1_awready_r <= 1'b0;
      aw0_consumed <= 1'b0;
      aw1_consumed <= 1'b0;
    end else begin
      // Track whether we've already accepted the current AWVALID assertion
      if (!m0_awvalid) aw0_consumed <= 1'b0;
      else if (m0_awvalid && m0_awready) aw0_consumed <= 1'b1;

      if (!m1_awvalid) aw1_consumed <= 1'b0;
      else if (m1_awvalid && m1_awready) aw1_consumed <= 1'b1;

      // Ready is asserted for at most one handshake per AWVALID assertion
      m0_awready_r <= (grant_aw_m0 && !aw0_consumed);
      m1_awready_r <= (grant_aw_m1 && !aw1_consumed);
    end
  end

  assign m0_awready = m0_awready_r;
  assign m1_awready = m1_awready_r;


  // Per-slave AW valid (clean mux based on granted master)
  wire m0_aw_s0 = grant_aw_m0 && (dec_slave(m0_awaddr)==2'd0);
  wire m0_aw_s1 = grant_aw_m0 && (dec_slave(m0_awaddr)==2'd1);
  wire m0_aw_s2 = grant_aw_m0 && (dec_slave(m0_awaddr)==2'd2);

  wire m1_aw_s0 = grant_aw_m1 && (dec_slave(m1_awaddr)==2'd0);
  wire m1_aw_s1 = grant_aw_m1 && (dec_slave(m1_awaddr)==2'd1);
  wire m1_aw_s2 = grant_aw_m1 && (dec_slave(m1_awaddr)==2'd2);

  assign s0_awvalid = aw_hold && (wr_dst==2'd0);
  assign s1_awvalid = aw_hold && (wr_dst==2'd1);
  assign s2_awvalid = aw_hold && (wr_dst==2'd2);
assign s0_awaddr = aw_addr_r;
  assign s1_awaddr = aw_addr_r;
  assign s2_awaddr = aw_addr_r;
assign s0_awlen = aw_len_r;
  assign s1_awlen = aw_len_r;
  assign s2_awlen = aw_len_r;
assign s0_awid = aw_id_r;
  assign s1_awid = aw_id_r;
  assign s2_awid = aw_id_r;
// Forward declarations for W mux/ready (used inside the write-context FF block)
  wire [AXI_DATA_WIDTH-1:0] wdata_mux;
  wire                      wlast_mux;
  wire                      wvalid_mux;
  wire                      wready_sel;

  // WLAST handshake (唯一 write 結束條件)
  wire wlast_hs;
  assign wlast_hs = wvalid_mux && wready_sel && wlast_mux;

  // Latch write context on AW handshake with selected master (fabric-side)
// FIX: Write path is independent of rd_active (read/write can run in parallel).
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    wr_active    <= 1'b0;
    wr_resp_wait <= 1'b0;
    wr_owner     <= 1'b0;
    wr_dst       <= 2'd0;
        b_dst      <= 2'd0;
wr_left      <= 8'd0;
    wr_id        <= '0;
    rr_aw_turn   <= 1'b0;
    aw_hold      <= 1'b0;
    aw_id_r      <= '0;
    aw_addr_r    <= '0;
    aw_len_r     <= 8'd0;
    b_owner      <= 1'b0;
  end else begin
    // Keep aw_hold asserted until S0 accepts the AW.
    if (aw_hold) begin
      if ((b_dst==2'd0 && s0_awready) || (b_dst==2'd1 && s1_awready) || (b_dst==2'd2 && s2_awready))
        aw_hold <= 1'b0;
    end

    // Start a new write on AW handshake (only blocked by write-side busy)
    if (!wr_busy) begin
      if (sel_aw_m0) begin
        wr_active    <= 1'b1;
        wr_resp_wait <= 1'b1;
        rr_aw_turn   <= 1'b1; // next time prefer M1 when both request
        wr_owner     <= 1'b0;
        wr_dst       <= dec_slave(m0_awaddr);
                    b_dst <= dec_slave(m0_awaddr);
b_dst  <= wr_dst; // will be overwritten below; keep for safety
wr_left      <= m0_awlen + 8'd1;
        wr_id        <= m0_awid;
        b_owner      <= 1'b0;

        begin
          aw_hold   <= 1'b1;
          aw_id_r   <= m0_awid;
          aw_addr_r <= m0_awaddr;
          aw_len_r  <= m0_awlen;
        end
      end else if (sel_aw_m1) begin
        wr_active    <= 1'b1;
        wr_resp_wait <= 1'b1;
        rr_aw_turn   <= 1'b0; // next time prefer M0 when both request
        wr_owner     <= 1'b1;
        wr_dst       <= dec_slave(m1_awaddr);
              b_dst <= dec_slave(m1_awaddr);
wr_left      <= m1_awlen + 8'd1;
        wr_id        <= m1_awid;
        b_owner      <= 1'b1;

        begin
          aw_hold   <= 1'b1;
          aw_id_r   <= m1_awid;
          aw_addr_r <= m1_awaddr;
          aw_len_r  <= m1_awlen;
        end
      end
    end

    // Track W beats while active (independent of rd_active)
    if (wr_active) begin
      if (wvalid_mux && wready_sel) begin
        if (wr_left != 0) wr_left <= wr_left - 8'd1;

        // End of write data: ONLY WLAST handshake can close wr_active
        if (wlast_mux) begin
          wr_active <= 1'b0;
        end
      end
    end

    // Consume B from slave (fabric boundary always-ready)
    if (wr_resp_wait && (bvalid_sel && b_m_ready)) begin
      wr_resp_wait <= 1'b0;
    end
  end
end

  // W routing
  wire use_m0_w = (wr_active && (wr_owner==1'b0));
  wire use_m1_w = (wr_active && (wr_owner==1'b1));

  assign wdata_mux  = use_m0_w ? m0_wdata  : m1_wdata;
  assign wlast_mux  = use_m0_w ? m0_wlast  : m1_wlast;
  assign wvalid_mux = use_m0_w ? m0_wvalid : m1_wvalid;

  assign s0_wvalid = wr_active && (wr_dst==2'd0) && wvalid_mux;
  assign s1_wvalid = wr_active && (wr_dst==2'd1) && wvalid_mux;
  assign s2_wvalid = wr_active && (wr_dst==2'd2) && wvalid_mux;

  assign s0_wdata  = wdata_mux;
  assign s1_wdata  = wdata_mux;
  assign s2_wdata  = wdata_mux;

  assign s0_wlast  = wlast_mux;
  assign s1_wlast  = wlast_mux;
  assign s2_wlast  = wlast_mux;

  // Backpressure WREADY to selected master from selected slave
  // FIX: Once AW accepted and wr_active asserted, fabric must always accept W (do not gate by slave wready)
  assign wready_sel = wr_active && wready_dst;
  assign m0_wready = wr_active && (wr_owner==1'b0) && wready_sel;
  assign m1_wready = wr_active && (wr_owner==1'b1) && wready_sel;

  // B routing: only from destination, back to owner
  wire bvalid_sel =
      (b_dst==2'd0) ? s0_bvalid :
      (b_dst==2'd1) ? s1_bvalid :
                       s2_bvalid;

  wire [AXI_ID_WIDTH-1:0] bid_sel =
      (b_dst==2'd0) ? s0_bid :
      (b_dst==2'd1) ? s1_bid :
                       s2_bid;

  wire [1:0] bresp_sel =
      (b_dst==2'd0) ? s0_bresp :
      (b_dst==2'd1) ? s1_bresp :
                       s2_bresp;

  // Fabric BREADY only for selected destination; others deasserted
  // Note: one outstanding write at a time (wr_dst + b_owner identify the response path)
  wire b_m_ready = (b_owner==1'b0) ? m0_bready : m1_bready;

  assign s0_bready = (b_dst==2'd0) ? b_m_ready : 1'b0;
  assign s1_bready = (b_dst==2'd1) ? b_m_ready : 1'b0;
  assign s2_bready = (b_dst==2'd2) ? b_m_ready : 1'b0;

  assign m0_bvalid = bvalid_sel && (b_owner==1'b0);
  assign m1_bvalid = bvalid_sel && (b_owner==1'b1);
  assign m0_bid    = bid_sel;
  assign m1_bid    = bid_sel;
  assign m0_bresp  = bresp_sel;
  assign m1_bresp  = bresp_sel;



  // ============================================================
// READ path: single outstanding with latched AR payload
//   - Fixes ARLEN glitch/mismatch by registering AR fields before
//     presenting them to the selected slave.
//   - Does NOT trust slave RLAST; generates master RLAST from
//     internal beats_left counter.
// ============================================================

  // ============================================================
  // Registered ARREADY to avoid half-cycle pulse (fixed14)
  // Also prevents duplicate AR handshakes if master holds ARVALID longer than one cycle.
  // ============================================================
  logic m0_arready_r, m1_arready_r;
  logic ar0_consumed, ar1_consumed;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m0_arready_r <= 1'b0;
      m1_arready_r <= 1'b0;
      ar0_consumed <= 1'b0;
      ar1_consumed <= 1'b0;
    end else begin
      if (!m0_arvalid) ar0_consumed <= 1'b0;
      else if (m0_arvalid && m0_arready) ar0_consumed <= 1'b1;

      if (!m1_arvalid) ar1_consumed <= 1'b0;
      else if (m1_arvalid && m1_arready) ar1_consumed <= 1'b1;

      m0_arready_r <= (grant_ar_m0 && !ar0_consumed);
      m1_arready_r <= (grant_ar_m1 && !ar1_consumed);
    end
  end

  assign m0_arready = m0_arready_r;
  assign m1_arready = m1_arready_r;
logic        rd_active;
logic        rd_owner;        // 0=M0, 1=M1
logic [1:0]  rd_dst;

logic [AXI_ID_WIDTH-1:0]   rd_arid_lat;
logic [AXI_ADDR_WIDTH-1:0] rd_araddr_lat;
logic [7:0]                rd_arlen_lat;   // AXI LEN (beats-1)

logic [7:0]  rd_beats_left;  // beats remaining (1..N)
logic        rd_ar_pending;  // AR payload waiting to be accepted by slave
logic        rr_ar_turn;      // 0: prefer M0 when both valid, 1: prefer M1

// Round-Robin AR arbitration (single outstanding read):
wire grant_ar_m0 = (!rd_active) && m0_arvalid && ( !m1_arvalid || (rr_ar_turn==1'b0) );
wire grant_ar_m1 = (!rd_active) && m1_arvalid && ( !m0_arvalid || (rr_ar_turn==1'b1) );
  // assign m0_arready = grant_ar_m0; // fixed11 removed
  // assign m1_arready = grant_ar_m1; // fixed11 removed
wire sel_ar_m0 = m0_arvalid && m0_arready;
wire sel_ar_m1 = m1_arvalid && m1_arready;

// Present *latched* AR payload to the selected slave
assign s0_arvalid = rd_ar_pending && (rd_dst==2'd0);
assign s1_arvalid = rd_ar_pending && (rd_dst==2'd1);
assign s2_arvalid = rd_ar_pending && (rd_dst==2'd2);

assign s0_araddr  = rd_araddr_lat;
assign s1_araddr  = rd_araddr_lat;
assign s2_araddr  = rd_araddr_lat;

assign s0_arlen   = rd_arlen_lat;
assign s1_arlen   = rd_arlen_lat;
assign s2_arlen   = rd_arlen_lat;

assign s0_arid    = rd_arid_lat;
assign s1_arid    = rd_arid_lat;
assign s2_arid    = rd_arid_lat;

wire ar_hs_s0 = s0_arvalid && s0_arready;
wire ar_hs_s1 = s1_arvalid && s1_arready;
wire ar_hs_s2 = s2_arvalid && s2_arready;
wire ar_hs_any = ar_hs_s0 || ar_hs_s1 || ar_hs_s2;

// Latch read context
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    rd_active      <= 1'b0;
    rd_owner       <= 1'b0;
    rd_dst         <= 2'd0;

    rd_arid_lat    <= '0;
    rd_araddr_lat  <= '0;
    rd_arlen_lat   <= 8'd0;

    rd_beats_left  <= 8'd0;
    rd_ar_pending  <= 1'b0;
    rr_ar_turn     <= 1'b0;
  end else begin
    // issue new read (accept from master, then drive to slave next cycle)
    if (!rd_active) begin
      if (sel_ar_m0) begin
        rd_active     <= 1'b1;
        rd_owner      <= 1'b0;
        rr_ar_turn    <= 1'b1; // next time prefer M1 when both request
        rd_dst        <= dec_slave(m0_araddr);

        rd_arid_lat   <= m0_arid;
        rd_araddr_lat <= m0_araddr;
        rd_arlen_lat  <= m0_arlen;

        rd_beats_left <= m0_arlen + 8'd1;
        rd_ar_pending <= 1'b1;
      end else if (sel_ar_m1) begin
        rd_active     <= 1'b1;
        rd_owner      <= 1'b1;
        rr_ar_turn    <= 1'b0; // next time prefer M0 when both request
        rd_dst        <= dec_slave(m1_araddr);

        rd_arid_lat   <= m1_arid;
        rd_araddr_lat <= m1_araddr;
        rd_arlen_lat  <= m1_arlen;

        rd_beats_left <= m1_arlen + 8'd1;
        rd_ar_pending <= 1'b1;
      end
    end else begin
      // Clear AR pending once the selected slave accepts the AR
      if (rd_ar_pending && ar_hs_any) begin
        rd_ar_pending <= 1'b0;
      end

      // track beats and clear when last beat accepted
      if (!rd_ar_pending) begin
        if (r_fire) begin
          if (rd_beats_left != 0) rd_beats_left <= rd_beats_left - 8'd1;
          if (rlast_fire) begin
            rd_active <= 1'b0;
          end
        end
      end
    end
  end
end

// Mux slave R channel
wire sel_s0_r = (rd_dst==2'd0);
wire sel_s1_r = (rd_dst==2'd1);
wire sel_s2_r = (rd_dst==2'd2);

wire rvalid_mux = (sel_s0_r && s0_rvalid) || (sel_s1_r && s1_rvalid) || (sel_s2_r && s2_rvalid);
wire rready_mux = (rd_owner==1'b0) ? m0_rready : m1_rready;
wire r_fire      = rd_active && !rd_ar_pending && rvalid_mux && rready_mux;
wire rlast_fire  = r_fire && rlast_mux;

wire [AXI_DATA_WIDTH-1:0] rdata_mux =
    sel_s0_r ? s0_rdata :
    sel_s1_r ? s1_rdata :
               s2_rdata;

wire [AXI_ID_WIDTH-1:0] rid_mux =
    sel_s0_r ? s0_rid :
    sel_s1_r ? s1_rid :
               s2_rid;

wire [1:0] rresp_mux =
    sel_s0_r ? s0_rresp :
    sel_s1_r ? s1_rresp :
               s2_rresp;

// Generate RLAST to master on the final expected beat (only when we're actually delivering R beats)
wire rlast_mux = (rd_active && !rd_ar_pending && (rd_beats_left == 8'd1) && rvalid_mux);

assign m0_rvalid = rd_active && !rd_ar_pending && (rd_owner==1'b0) && rvalid_mux;
assign m1_rvalid = rd_active && !rd_ar_pending && (rd_owner==1'b1) && rvalid_mux;

assign m0_rdata  = rdata_mux;
assign m1_rdata  = rdata_mux;

assign m0_rid    = rid_mux;
assign m1_rid    = rid_mux;

assign m0_rresp  = rresp_mux;
assign m1_rresp  = rresp_mux;

assign m0_rlast  = rlast_mux;
assign m1_rlast  = rlast_mux;

// rready to selected slave from owner
assign s0_rready = rd_active && !rd_ar_pending && sel_s0_r && (rd_owner ? m1_rready : m0_rready);
assign s1_rready = rd_active && !rd_ar_pending && sel_s1_r && (rd_owner ? m1_rready : m0_rready);
assign s2_rready = rd_active && !rd_ar_pending && sel_s2_r && (rd_owner ? m1_rready : m0_rready);

// ============================================================
  // Debug hooks used by your TB
  // ============================================================
  // "active" means current outstanding read targets that slave
  // (Your TB peeks: u_noc.s0_rd_active, etc.)
  logic s0_rd_active, s1_rd_active, s2_rd_active;
  always_comb begin
    s0_rd_active = rd_active && (rd_dst==2'd0);
    s1_rd_active = rd_active && (rd_dst==2'd1);
    s2_rd_active = rd_active && (rd_dst==2'd2);
  end

endmodule
