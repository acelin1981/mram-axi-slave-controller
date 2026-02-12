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
// AXI SRAM SLAVE MODEL (FIXED: no mem[] in always_comb)
// ============================================================
module axi_sram_slave_model #(
  parameter int AXI_ID_WIDTH   = 4,
  parameter int AXI_ADDR_WIDTH = 32,
  parameter int AXI_DATA_WIDTH = 64,
  parameter int DEPTH_WORDS    = 2048,
  parameter logic [AXI_ADDR_WIDTH-1:0] BASE_ADDR = 32'h0000_4000
)(
  input  logic clk,
  input  logic rst_n,

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
  input  logic                      rready
);

  logic [AXI_DATA_WIDTH-1:0] mem [0:DEPTH_WORDS-1];

  typedef enum logic [1:0] {WIDLE, WDATA, WRESP} wst_t;
  typedef enum logic [1:0] {RIDLE, RSEND} rst_t;

  wst_t wst;
  rst_t rstt;

  logic [AXI_ID_WIDTH-1:0] wid, rid_q;
  logic [AXI_ADDR_WIDTH-1:0] waddr_q, raddr_q;
  logic [7:0] wrem, rrem;

  logic [AXI_DATA_WIDTH-1:0] rdata_q;
  logic [AXI_ADDR_WIDTH-1:0] next_addr;

  // address relative
  logic [AXI_ADDR_WIDTH-1:0] raddr_rel;
  wire [31:0] ridx = (raddr_rel >> $clog2(AXI_DATA_WIDTH/8));

  always_comb begin
    raddr_rel = (raddr_q >= BASE_ADDR) ? (raddr_q - BASE_ADDR) : '0;

    awready = (wst==WIDLE);
    wready  = (wst==WDATA);
    bvalid  = (wst==WRESP);
    bresp   = 2'b00;
    bid     = wid;

    arready = (rstt==RIDLE);
    rvalid  = (rstt==RSEND);
    rresp   = 2'b00;
    rid     = rid_q;
    rlast   = (rstt==RSEND) && (rrem==0);

    rdata   = rdata_q;
  end

  integer i;

  always_ff @(posedge clk or negedge rst_n) begin
    // fixed15: gate B until WLAST handshake
    logic wlast_seen;

    if (!rst_n) begin
      wlast_seen <= 1'b0;
      wst <= WIDLE;
      rstt<= RIDLE;
      wid <= '0;
      rid_q <= '0;
      waddr_q <= '0;
      raddr_q <= '0;
      wrem <= '0;
      rrem <= '0;
      rdata_q <= '0;
      for (i=0;i<DEPTH_WORDS;i=i+1) mem[i]<='0;
    end else begin
      // WRITE FSM
      case (wst)
        WIDLE: if (awvalid && awready) begin
          wid <= awid;
          waddr_q <= awaddr;
          wrem <= awlen;
          wst <= WDATA;
        end
        WDATA: if (wvalid && wready) begin
        if (wlast) wlast_seen <= 1'b1;
          mem[(waddr_q-BASE_ADDR)>>3] <= wdata;
          waddr_q <= waddr_q + (AXI_DATA_WIDTH/8);
          if (wrem==0) wst <= WRESP;
          else wrem <= wrem - 1;
        end
        WRESP: if (bvalid && bready) wst <= WIDLE;
      endcase

      // READ FSM
      case (rstt)
        RIDLE: if (arvalid && arready) begin
          rid_q   <= arid;
          raddr_q <= araddr;
          rrem    <= arlen;

          // FIX: first beat must use araddr directly (raddr_q updates after this clock edge)
          if (((araddr-BASE_ADDR)>>3) < DEPTH_WORDS)
            rdata_q <= mem[(araddr-BASE_ADDR)>>3];
          else
            rdata_q <= '0;

          rstt <= RSEND;
        end

        RSEND: if (rvalid && rready) begin
          next_addr = raddr_q + (AXI_DATA_WIDTH/8);
          raddr_q   <= next_addr;

          if (rrem==0) begin
            rstt <= RIDLE;
          end else begin
            rrem <= rrem - 1;

            // Prefetch next beat
            if (((next_addr-BASE_ADDR)>>3) < DEPTH_WORDS)
              rdata_q <= mem[(next_addr-BASE_ADDR)>>3];
            else
              rdata_q <= '0;
          end
        end
      endcase
    end
  end
endmodule
