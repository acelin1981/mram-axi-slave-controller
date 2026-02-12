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
// DDR4-like AXI slave model (simple, in-order, single outstanding)
// - Write: accept W beats, enqueue commit after DDR_WR_LAT cycles
// - Read : AR accepted -> wait DDR_RD_LAT cycles -> stream R beats
// - Optional "ddr_cs" asserted only when active (rd/wr in progress)
// ============================================================


module axi_ddr4_slave_model #(
  parameter int AXI_ID_WIDTH   = 4,
  parameter int AXI_ADDR_WIDTH = 32,
  parameter int AXI_DATA_WIDTH = 64,

  parameter int DEPTH_WORDS    = 8192,
  parameter int DDR_RD_LAT     = 2,
  parameter int DDR_WR_LAT     = 100,

  // *** ADD THIS ***
  parameter logic [AXI_ADDR_WIDTH-1:0] BASE_ADDR = '0
)(
  input  logic                      clk,
  input  logic                      rst_n,

  // AW
  input  logic [AXI_ID_WIDTH-1:0]    awid,
  input  logic [AXI_ADDR_WIDTH-1:0]  awaddr,
  input  logic [7:0]                 awlen,
  input  logic                       awvalid,
  output logic                       awready,

  // W
  input  logic [AXI_DATA_WIDTH-1:0]  wdata,
  input  logic                       wlast,
  input  logic                       wvalid,
  output logic                       wready,

  // B
  output logic [AXI_ID_WIDTH-1:0]    bid,
  output logic [1:0]                 bresp,
  output logic                       bvalid,
  input  logic                       bready,

  // AR
  input  logic [AXI_ID_WIDTH-1:0]    arid,
  input  logic [AXI_ADDR_WIDTH-1:0]  araddr,
  input  logic [7:0]                 arlen,
  input  logic                       arvalid,
  output logic                       arready,

  // R
  output logic [AXI_ID_WIDTH-1:0]    rid,
  output logic [AXI_DATA_WIDTH-1:0]  rdata,
  output logic [1:0]                 rresp,
  output logic                       rvalid,
  output logic                       rlast,
  input  logic                       rready
);

  // ------------------------------------------------------------
  // Simple memory (word addressed, 64-bit per word)
  // ------------------------------------------------------------
  logic [AXI_DATA_WIDTH-1:0] mem [0:DEPTH_WORDS-1];

  integer i;
  initial begin
    // avoid X if read before write
    for (i = 0; i < DEPTH_WORDS; i++) mem[i] = '0;
  end

 
// return word index (64-bit word addressed => >>3)
function integer addr_to_index;
  input [AXI_ADDR_WIDTH-1:0] a;
  reg   [AXI_ADDR_WIDTH-1:0] tmp_addr;
  begin
    tmp_addr = a - BASE_ADDR;
    addr_to_index = tmp_addr[AXI_ADDR_WIDTH-1:3];
  end
endfunction

function in_range;
  input [AXI_ADDR_WIDTH-1:0] a;
  integer idx;
  begin
    idx = addr_to_index(a);
    if ((idx >= 0) && (idx < DEPTH_WORDS))
      in_range = 1'b1;
    else
      in_range = 1'b0;
  end
endfunction



  // ------------------------------------------------------------
  // Write state
  // ------------------------------------------------------------
  typedef enum logic [1:0] {W_IDLE, W_DATA, W_RESP} wstate_t;
  wstate_t wst;

  logic [AXI_ID_WIDTH-1:0]   w_id;
  logic [AXI_ADDR_WIDTH-1:0] w_addr;
  logic [7:0]                w_beats_left;
  int unsigned               w_lat_cnt;

  // Ready signals
  assign awready = (wst == W_IDLE);
  assign wready  = (wst == W_DATA);

  // B channel
  assign bid   = w_id;
  assign bresp = 2'b00;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wst          <= W_IDLE;
      w_id         <= '0;
      w_addr       <= '0;
      w_beats_left <= 8'd0;
      w_lat_cnt    <= 0;
      bvalid       <= 1'b0;
    end else begin
      case (wst)
        W_IDLE: begin
          bvalid <= 1'b0;
          if (awvalid && awready) begin
            w_id         <= awid;
            w_addr       <= awaddr;
            w_beats_left <= awlen + 8'd1;
            wst          <= W_DATA;
          end
        end

        W_DATA: begin
          if (wvalid && wready) begin
            // write current beat
            if (in_range(w_addr)) begin
              mem[addr_to_index(w_addr)] <= wdata;
            end
            // next word address (64-bit)
            w_addr <= w_addr + (AXI_DATA_WIDTH/8);

            if (w_beats_left != 0) w_beats_left <= w_beats_left - 8'd1;

            if (wlast) begin
              w_lat_cnt <= DDR_WR_LAT;
              wst       <= W_RESP;
            end
          end
        end

        W_RESP: begin
          if (w_lat_cnt > 0) begin
            w_lat_cnt <= w_lat_cnt - 1;
          end else begin
            bvalid <= 1'b1;
            if (bvalid && bready) begin
              bvalid <= 1'b0;
              wst    <= W_IDLE;
            end
          end
        end
      endcase
    end
  end

  // ------------------------------------------------------------
  // Read state
  // ------------------------------------------------------------
  typedef enum logic [1:0] {R_IDLE, R_WAIT, R_SEND} rstate_t;
  rstate_t rst;

  logic [AXI_ID_WIDTH-1:0]   r_id;
  logic [AXI_ADDR_WIDTH-1:0] r_addr;
  logic [7:0]                r_beats_left;
  int unsigned               r_lat_cnt;

  assign arready = (rst == R_IDLE);

  assign rid   = r_id;
  assign rresp = 2'b00;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rst          <= R_IDLE;
      r_id         <= '0;
      r_addr       <= '0;
      r_beats_left <= 8'd0;
      r_lat_cnt    <= 0;
      rvalid       <= 1'b0;
      rlast        <= 1'b0;
      rdata        <= '0;
    end else begin
      case (rst)
        R_IDLE: begin
          rvalid <= 1'b0;
          rlast  <= 1'b0;
          if (arvalid && arready) begin
            r_id         <= arid;
            r_addr       <= araddr;
            r_beats_left <= arlen + 8'd1;
            r_lat_cnt    <= DDR_RD_LAT;
            rst          <= R_WAIT;
          end
        end

        R_WAIT: begin
          if (r_lat_cnt > 0) begin
            r_lat_cnt <= r_lat_cnt - 1;
          end else begin
            // prepare first data before asserting rvalid
            if (in_range(r_addr)) rdata <= mem[addr_to_index(r_addr)];
            else                  rdata <= '0; // out-of-range -> return 0 instead of X

            rvalid <= 1'b1;
            rlast  <= (r_beats_left == 8'd1);
            rst    <= R_SEND;
          end
        end

        R_SEND: begin
          if (rvalid && rready) begin
            // advance
            r_addr <= r_addr + (AXI_DATA_WIDTH/8);
            if (r_beats_left != 0) r_beats_left <= r_beats_left - 8'd1;

            if (r_beats_left == 8'd1) begin
              // last beat just accepted
              rvalid <= 1'b0;
              rlast  <= 1'b0;
              rst    <= R_IDLE;
            end else begin
              // next beat data (no extra latency inside burst)
              if (in_range(r_addr + (AXI_DATA_WIDTH/8)))
                rdata <= mem[addr_to_index(r_addr + (AXI_DATA_WIDTH/8))];
              else
                rdata <= '0;

              rlast <= (r_beats_left == 8'd2);
            end
          end
        end
      endcase
    end
  end

endmodule
