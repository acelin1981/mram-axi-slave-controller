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
// Code your testbench here
`timescale 1ns/1ps

module tb_soc_top_2m3s_noc_mram;

  int unsigned frame_scale;
  int unsigned frame_beats_sim;
  int unsigned frame_bytes_sim;
  int unsigned weight_bytes_sim;
  int unsigned weight_beats_sim;
logic pwr_report_pulse;

  // Power monitor reported values
  real ddr_t_s;
  real ddr_e_ddr_j, ddr_e_mram_j, ddr_e_total_j;
  real ddr_p_ddr_w, ddr_p_mram_w, ddr_p_total_w;

  real mram_t_s;
  real mram_e_ddr_j, mram_e_mram_j, mram_e_total_j;
  real mram_p_ddr_w, mram_p_mram_w, mram_p_total_w;

  real pwr_e_ddr_j, pwr_e_mram_j, pwr_e_total_j;
  real pwr_p_ddr_w, pwr_p_mram_w, pwr_p_total_w;

// ============================================================
  // FAST-SIM memory sizing
  // NOTE:
  // - Large RTL arrays + VCD can make vvp appear "stuck" at time 0.
  // - For paper-level 4K workloads, we model *traffic* and power via
  //   AXI beats/ops; full 4K frame storage can be represented via tiling.
  // ============================================================
  localparam int unsigned MRAM_SIZE_BYTES = 8*1024*1024;   // 8MB (fits ~5.5MB INT8 weights + headroom)
  localparam int unsigned DDR4_SIZE_BYTES = 8*1024*1024;   // 8MB (tile buffer / streaming)
  localparam int unsigned SRAM_SIZE_BYTES = 4*1024*1024;   // 4MB (output tile / scratch)

  localparam int unsigned MRAM_DEPTH_WORDS = MRAM_SIZE_BYTES / 8; // 64-bit word
  localparam int unsigned DDR4_DEPTH_WORDS = DDR4_SIZE_BYTES / 8;
  localparam int unsigned SRAM_DEPTH_WORDS = SRAM_SIZE_BYTES / 8;

  // Waveform control: set to 0 for fastest runtime
  localparam bit DUMP_WAVES = 1'b1;

  // ============================================================
  // 4K STREAMING MODEL (paper: "4K input")
  // We do NOT allocate a full 4K frame in the RTL memory array (too slow in vvp).
  // Instead, we generate the *same traffic volume* as a 4K frame and wrap the
  // address into a smaller DDR4/SRAM tile buffer using modulo addressing.
  // ============================================================
  localparam int unsigned FRAME_W = 3840;
  localparam int unsigned FRAME_H = 2160;
  localparam int unsigned BYTES_PER_PIXEL = 3; // RGB888
  localparam int unsigned FRAME_BYTES = FRAME_W * FRAME_H * BYTES_PER_PIXEL; // 24,883,200 bytes

  localparam int unsigned AXI_BEAT_BYTES = 8; // 64-bit
  localparam int unsigned FRAME_BEATS = (FRAME_BYTES + AXI_BEAT_BYTES - 1) / AXI_BEAT_BYTES; // ceil

  // ------------------------------------------------------------
  // Simulation controls
  // - VERBOSE_BEAT_LOG: per-beat prints are EXTREMELY slow for 4K (millions of beats)
  // - FRAME_SCALE: simulate 1/FRAME_SCALE of the 4K beats and scale reported bytes/energy linearly
  //   (valid here because our power model is linear in counted beats/cycles).
  // ------------------------------------------------------------
  localparam bit VERBOSE_BEAT_LOG = 1'b0;
  localparam bit VERBOSE_MON_LOG  = 1'b0;
  localparam int unsigned FRAME_SCALE = 1024; // 1024x faster; effective traffic still 4K via scaling

  localparam int unsigned FRAME_BEATS_SIM = (FRAME_BEATS + FRAME_SCALE - 1) / FRAME_SCALE;
  localparam int unsigned FRAME_BYTES_SIM = FRAME_BEATS_SIM * AXI_BEAT_BYTES;

  // Approximate MobileNetV3-Large INT8 weight footprint (bytes).
  // Used to size the one-time weight preload into MRAM (S0). We scale it down similarly to frames.
  localparam int unsigned MOBILENETV3_WEIGHT_BYTES_REAL = 5_500_000; // ~5.5 MB

  // Weight placement mode for NPU weight fetch simulation:
  //   A = 100% weights in MRAM (S0)
  //   B = 100% weights in DRAM (S1)
  //   C = split weights between MRAM/DRAM using WEIGHT_MRAM_PCT
  integer weight_mode_sel = 2;          // 0=A, 1=B, 2=C
  integer weight_mram_pct = 50;         // used when mode=C (0~100)
  logic [AXI_ADDR_WIDTH-1:0] weight_base_cfg;  // runtime: MRAM_WEIGHT_BASE or DRAM_WEIGHT_BASE
  reg [8*8-1:0] weight_mode_str;
  localparam int unsigned MIN_WEIGHT_BYTES_SIM = 512*1024; // >= max NPU weight address span (covers up to ~0x00031000)

  
  // -------------------------
  // Select scale at runtime:
  //   +SCALE=1024  (default)
  //   +SCALE=256
  //   +SCALE=128
  //   +SCALE=1     (true 4K, slow)
  // -------------------------
  // ================================
  // Power clear helper
  // ================================
  task automatic pwr_clear_now();
    begin
      clear_pulse <= 1'b1;
      @(posedge clk);
      clear_pulse <= 1'b0;
      repeat (2) @(posedge clk);
    end
  endtask

  // ================================
  // Simple AXI read burst (no data check)
  // - mid: 0=M0 (not driven in this TB), 1=M1 (DMA traffic generator)
  // - beats: number of 64-bit beats to read
  // ================================
  task automatic axi_read_burst_no_check(input int mid,
                                        input [31:0] addr,
                                        input int beats);
	    int b;
	    int burst_beats;
	    int arlen;
	    reg [31:0] a;
	    int unsigned tmo;
    begin : AR_NC
      a = addr;
      b = 0;
      while (b < beats) begin
        burst_beats = (beats - b);
        if (burst_beats > 16) burst_beats = 16;
        arlen = burst_beats - 1;

	        // Drive AR/R (robust handshake + timeout)
	        if (mid == 1) begin
	          // NOTE: use blocking assignments to guarantee the signals are held
	          // stable across at least one rising edge.
	          m1_arid    = 0;
	          m1_araddr  = a;
	          m1_arlen   = arlen[7:0];
	          m1_arvalid = 1'b1;
	
	          // Wait for AR handshake (at a clock edge)
	          tmo = 0;
	          while (!m1_arready) begin
	            @(posedge clk);
	            tmo++;
	            if (tmo > 200000) begin
	              $display("[TB][HANG] axi_read_burst_no_check: AR timeout mid=%0d addr=0x%08x len=%0d", mid, a, arlen);
	              m1_arvalid = 1'b0;
	              disable AR_NC;
	            end
	          end
	          // Consume the handshake on this posedge
	          @(posedge clk);
	          m1_arvalid = 1'b0;
	
	          // Consume R beats (wait for each beat to handshake)
	          m1_rready = 1'b1;
	          tmo = 0;
	          repeat (burst_beats) begin
	            while (!m1_rvalid) begin
	              @(posedge clk);
	              tmo++;
	              if (tmo > 400000) begin
	                $display("[TB][HANG] axi_read_burst_no_check: R timeout mid=%0d addr=0x%08x", mid, a);
	                m1_rready = 1'b0;
	                disable AR_NC;
	              end
	            end
	            // handshake happens since rready=1
	            @(posedge clk);
	          end
	          m1_rready = 1'b0;
        end else begin
          // M0 is driven by the built-in NPU trace model in this TB; do not drive it here.
          // Keep this branch for completeness.
        end

        a = a + (burst_beats * 8);
        b = b + burst_beats;
      end
    end
  endtask

  // ================================
  // MobileNetV3-Large INT8 weight fetch simulation
  // - Generates DMA (M1) read traffic from MRAM/DRAM based on WEIGHT_MODE.
  // - This is used to make "weight placement" comparisons fair/repeatable.
  // ================================
  task automatic npu_weight_fetch_dma(input int frame_idx);
    // Address bases for "weights" region (must be within each slave's decode window)

    int bytes_eff;
    int bytes_sim;
    int beats_total;
    int bursts_total;
    int bursts_mram;
    int bursts_dram;
    int i;
    int mram_beats, dram_beats;
    reg [31:0] a_mram, a_dram;

    begin
      bytes_eff  = MOBILENETV3_WEIGHT_BYTES_REAL;
      // allow override
      if ($value$plusargs("WEIGHT_BYTES_EFF=%d", bytes_eff)) begin
        if (bytes_eff < 0) bytes_eff = MOBILENETV3_WEIGHT_BYTES_REAL;
      end

      // scale down to simulation size (same scaling concept as frame traffic)
      bytes_sim  = (bytes_eff + frame_scale - 1) / frame_scale;
      if (bytes_sim < 8) bytes_sim = 8;

      beats_total  = (bytes_sim + 7) / 8;           // 64-bit beats
      bursts_total = (beats_total + 15) / 16;       // 16 beats per burst

      // decide split
      if (weight_mode_sel == 0) begin
        bursts_mram = bursts_total;
      end else if (weight_mode_sel == 1) begin
        bursts_mram = 0;
      end else begin
        bursts_mram = (bursts_total * weight_mram_pct + 99) / 100; // round up
      end
      if (bursts_mram > bursts_total) bursts_mram = bursts_total;
      bursts_dram = bursts_total - bursts_mram;

      a_mram = MRAM_WEIGHT_BASE;
      a_dram = DRAM_WEIGHT_BASE;

      $display("[TB] [NPU][frame %0d] weight_fetch: eff=%0dB sim=%0dB beats=%0d bursts=%0d (MRAM=%0d, DRAM=%0d)",
               frame_idx, bytes_eff, bytes_sim, beats_total, bursts_total, bursts_mram, bursts_dram);

      // MRAM part
      mram_beats = 0;
      for (i = 0; i < bursts_mram; i = i + 1) begin
        axi_read_burst_no_check(1, a_mram, 16);
        a_mram = a_mram + 16*8;
        mram_beats = mram_beats + 16;
      end

      // DRAM part
      dram_beats = 0;
      for (i = 0; i < bursts_dram; i = i + 1) begin
        axi_read_burst_no_check(1, a_dram, 16);
        a_dram = a_dram + 16*8;
        dram_beats = dram_beats + 16;
      end

      $display("[TB] [NPU][frame %0d] weight_fetch DONE: MRAM=%0dB DRAM=%0dB (sim bytes)",
               frame_idx, mram_beats*8, dram_beats*8);
    end
  endtask


  initial begin
    frame_scale = 1024;
    if ($value$plusargs("SCALE=%d", frame_scale)) begin end
    if (!((frame_scale==1)||(frame_scale==128)||(frame_scale==256)||(frame_scale==1024))) begin
      $display("[TB][WARN] Unsupported SCALE=%0d, forcing 1024", frame_scale);
      frame_scale = 1024;
    end
    frame_beats_sim = (FRAME_BEATS + frame_scale - 1) / frame_scale;
    frame_bytes_sim = frame_beats_sim * AXI_BEAT_BYTES;
    weight_bytes_sim = (MOBILENETV3_WEIGHT_BYTES_REAL + frame_scale - 1) / frame_scale;
    if (weight_bytes_sim < MIN_WEIGHT_BYTES_SIM) weight_bytes_sim = MIN_WEIGHT_BYTES_SIM;
    weight_beats_sim = (weight_bytes_sim + AXI_BEAT_BYTES - 1) / AXI_BEAT_BYTES;
    $display("[TB] SCALE=%0d  FRAME_BEATS=%0d  FRAME_BEATS_SIM=%0d  effective_bytes=%0d  simulated_bytes=%0d",
             frame_scale, FRAME_BEATS, frame_beats_sim, FRAME_BYTES, frame_bytes_sim);

    // Weight placement mode (A/B/C) for NPU weight fetch simulation
    weight_mode_str = "C";
    if ($value$plusargs("WEIGHT_MODE=%s", weight_mode_str)) begin
      if ((weight_mode_str[7:0] == "A") || (weight_mode_str[7:0] == "a")) weight_mode_sel = 0;
      else if ((weight_mode_str[7:0] == "B") || (weight_mode_str[7:0] == "b")) weight_mode_sel = 1;
      else weight_mode_sel = 2; // C or anything else
    end
    if ($value$plusargs("WEIGHT_MRAM_PCT=%d", weight_mram_pct)) begin
      if (weight_mram_pct < 0)   weight_mram_pct = 0;
      if (weight_mram_pct > 100) weight_mram_pct = 100;
    end
    // Set NPU runtime weight base to avoid MRAM reads when WEIGHT_MRAM_PCT=0
    weight_base_cfg = (weight_mram_pct > 0) ? MRAM_WEIGHT_BASE : DRAM_WEIGHT_BASE;
    // If WEIGHT_MODE=C and WEIGHT_MRAM_PCT=0, we intentionally disable MRAM energy accounting
    // in the power model so that MRAM_total becomes exactly 0 even if there is incidental S0 traffic.
    if ((weight_mode_sel == 2) && (weight_mram_pct == 0)) begin
      $display("[TB] MRAM power accounting disabled (WEIGHT_MODE=C, WEIGHT_MRAM_PCT=0)");
    end
    $display("[TB] WEIGHT_MODE=%s  WEIGHT_MRAM_PCT=%0d  (MobileNetV3-L INT8 weights ~%0d bytes effective)",
             weight_mode_str, weight_mram_pct, MOBILENETV3_WEIGHT_BYTES_REAL);
  end

function automatic [31:0] wrap_addr(input [31:0] base, input int unsigned offset_bytes, input int unsigned buf_bytes);
    // Keep access within allocated model buffer while preserving traffic volume.
    wrap_addr = base + (offset_bytes % buf_bytes);
  endfunction

// ------------------------------------------------------------
// fixed_tb: robust timeouts so simulation never "hangs forever"
// ------------------------------------------------------------
localparam int unsigned TB_TIMEOUT_CYCLES = 25_000_000;

task automatic tb_wait_cycles(input int unsigned n);
  repeat (n) @(posedge clk);
endtask

task automatic tb_wait_until_or_fatal(
  input string         what,
  input logic          cond
);
  int unsigned wd;
  wd = 0;
  while (!cond) begin
    @(posedge clk);
    wd++;
    if (wd >= TB_TIMEOUT_CYCLES) begin
      $display("[%0t] [TB][HANG] timeout waiting for %s", $time, what);
      $fatal(1);
    end
  end
endtask


  // -------------------------
  // clock/reset
  // -------------------------
  logic clk, rst_n;
  initial begin
    clk = 1'b0;
	//forever #5 clk = ~clk;  // 100MHz  (T=10ns)
    forever #2.5 clk = ~clk;  // 200MHz  (T=5ns)
	//forever #1.25 clk = ~clk;  // 400MHz  (T=2.5ns)
  end

  logic [13:0] write_delay_config;

  // -------------------------
  // AXI params
  // -------------------------
  localparam int AXI_ID_WIDTH   = 4;
  localparam int AXI_ADDR_WIDTH = 32;
  localparam int AXI_DATA_WIDTH = 64;

  // -------------------------
  // AXI Master0 signals
  // -------------------------
  // Master0 is driven by NPU (module outputs). Use WIRE/net types for Icarus compatibility.
  wire [AXI_ID_WIDTH-1:0]     m0_awid,  m0_bid,  m0_arid,  m0_rid;
  wire [AXI_ADDR_WIDTH-1:0]   m0_awaddr, m0_araddr;
  wire [7:0]                  m0_awlen,  m0_arlen;
  wire [2:0]                  m0_awsize;
  wire [1:0]                  m0_awburst;
  wire                        m0_awvalid, m0_awready;

  wire [AXI_DATA_WIDTH-1:0]   m0_wdata,  m0_rdata;
  wire [AXI_DATA_WIDTH/8-1:0] m0_wstrb;
  wire                        m0_wlast,  m0_wvalid, m0_wready;

  wire [1:0]                  m0_bresp,  m0_rresp;
  wire                        m0_bvalid, m0_bready;

  wire [2:0]                  m0_arsize;
  wire [1:0]                  m0_arburst;
  wire                        m0_arvalid, m0_arready;

  wire                        m0_rvalid, m0_rlast, m0_rready;

  // -------------------------
  // AXI Master1 signals
  // -------------------------
  logic [AXI_ID_WIDTH-1:0]   m1_awid,  m1_bid,  m1_arid,  m1_rid;
  logic [AXI_ADDR_WIDTH-1:0] m1_awaddr, m1_araddr;
  logic [7:0]                m1_awlen,  m1_arlen;
  logic                      m1_awvalid, m1_awready;
  logic [AXI_DATA_WIDTH-1:0] m1_wdata,  m1_rdata;
  logic                      m1_wlast,  m1_wvalid, m1_wready;
  logic [1:0]                m1_bresp,  m1_rresp;
  logic                      m1_bvalid, m1_bready;
  logic                      m1_arvalid, m1_arready;
  logic                      m1_rvalid, m1_rlast, m1_rready;


  // NPU master control
  logic npu_start;
  logic npu_done;
  // -------------------------
  // Slave0 (MRAM) AXI wires
  // -------------------------
  logic [AXI_ID_WIDTH-1:0]   s0_awid,  s0_bid,  s0_arid,  s0_rid;
  logic [AXI_ADDR_WIDTH-1:0] s0_awaddr, s0_araddr;
  logic [7:0]                s0_awlen,  s0_arlen;
  logic                      s0_awvalid, s0_awready;
  logic [AXI_DATA_WIDTH-1:0] s0_wdata,  s0_rdata;
  logic                      s0_wlast,  s0_wvalid, s0_wready;
  logic [1:0]                s0_bresp,  s0_rresp;
  logic                      s0_bvalid, s0_bready;
  logic                      s0_arvalid, s0_arready;
  logic                      s0_rvalid, s0_rlast, s0_rready;

  // -------------------------
  // Slave1 (DDR) AXI wires
  // -------------------------
  logic [AXI_ID_WIDTH-1:0]   s1_awid,  s1_bid,  s1_arid,  s1_rid;
  logic [AXI_ADDR_WIDTH-1:0] s1_awaddr, s1_araddr;
  logic [7:0]                s1_awlen,  s1_arlen;
  logic                      s1_awvalid, s1_awready;
  logic [AXI_DATA_WIDTH-1:0] s1_wdata,  s1_rdata;
  logic                      s1_wlast,  s1_wvalid, s1_wready;
  logic [1:0]                s1_bresp,  s1_rresp;
  logic                      s1_bvalid, s1_bready;
  logic                      s1_arvalid, s1_arready;
  logic                      s1_rvalid, s1_rlast, s1_rready;

  // -------------------------
  // Slave2 (SRAM) AXI wires
  // -------------------------
  logic [AXI_ID_WIDTH-1:0]   s2_awid,  s2_bid,  s2_arid,  s2_rid;
  logic [AXI_ADDR_WIDTH-1:0] s2_awaddr, s2_araddr;
  logic [7:0]                s2_awlen,  s2_arlen;
  logic                      s2_awvalid, s2_awready;
  logic [AXI_DATA_WIDTH-1:0] s2_wdata,  s2_rdata;
  logic                      s2_wlast,  s2_wvalid, s2_wready;
  logic [1:0]                s2_bresp,  s2_rresp;
  logic                      s2_bvalid, s2_bready;
  logic                      s2_arvalid, s2_arready;
  logic                      s2_rvalid, s2_rlast, s2_rready;

  // For each slave channel, wstrb width = DATA_W/8 = 8
  // NOTE: Your NoC/slaves don't carry WSTRB, so tie to full-write (all bytes valid)
  logic [7:0] s0_wstrb;
  logic [7:0] s1_wstrb;
  logic [7:0] s2_wstrb;
  assign s0_wstrb = 8'hFF;
  assign s1_wstrb = 8'hFF;
  assign s2_wstrb = 8'hFF;

  // MRAM model pins (for power monitor)
  logic mram_we_en;
  logic mram_rd_en;

  // -------------------------
  // Instantiate NOC (2 masters -> 3 slaves)
  // IMPORTANT:
  // Your updated NoC should use ADDRESS DECODE:
  //   S0: 0x0000_0000 ~ 0x0FFF_FFFF
  //   S1: 0x1000_0000 ~ 0x1FFF_FFFF
  //   S2: 0x2000_0000 ~ 0x2FFF_FFFF
  // -------------------------
  axi_noc_2m3s #(
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
  ) u_noc (
    .clk(clk), .rst_n(rst_n),

    // M0
    .m0_awid(m0_awid), .m0_awaddr(m0_awaddr), .m0_awlen(m0_awlen), .m0_awvalid(m0_awvalid), .m0_awready(m0_awready),
    .m0_wdata(m0_wdata), .m0_wlast(m0_wlast), .m0_wvalid(m0_wvalid), .m0_wready(m0_wready),
    .m0_bid(m0_bid), .m0_bresp(m0_bresp), .m0_bvalid(m0_bvalid), .m0_bready(m0_bready),
    .m0_arid(m0_arid), .m0_araddr(m0_araddr), .m0_arlen(m0_arlen), .m0_arvalid(m0_arvalid), .m0_arready(m0_arready),
    .m0_rid(m0_rid), .m0_rdata(m0_rdata), .m0_rresp(m0_rresp), .m0_rvalid(m0_rvalid), .m0_rlast(m0_rlast), .m0_rready(m0_rready),

    // M1
    .m1_awid(m1_awid), .m1_awaddr(m1_awaddr), .m1_awlen(m1_awlen), .m1_awvalid(m1_awvalid), .m1_awready(m1_awready),
    .m1_wdata(m1_wdata), .m1_wlast(m1_wlast), .m1_wvalid(m1_wvalid), .m1_wready(m1_wready),
    .m1_bid(m1_bid), .m1_bresp(m1_bresp), .m1_bvalid(m1_bvalid), .m1_bready(m1_bready),
    .m1_arid(m1_arid), .m1_araddr(m1_araddr), .m1_arlen(m1_arlen), .m1_arvalid(m1_arvalid), .m1_arready(m1_arready),
    .m1_rid(m1_rid), .m1_rdata(m1_rdata), .m1_rresp(m1_rresp), .m1_rvalid(m1_rvalid), .m1_rlast(m1_rlast), .m1_rready(m1_rready),

    // S0
    .s0_awid(s0_awid), .s0_awaddr(s0_awaddr), .s0_awlen(s0_awlen), .s0_awvalid(s0_awvalid), .s0_awready(s0_awready),
    .s0_wdata(s0_wdata), .s0_wlast(s0_wlast), .s0_wvalid(s0_wvalid), .s0_wready(s0_wready),
    .s0_bid(s0_bid), .s0_bresp(s0_bresp), .s0_bvalid(s0_bvalid), .s0_bready(s0_bready),
    .s0_arid(s0_arid), .s0_araddr(s0_araddr), .s0_arlen(s0_arlen), .s0_arvalid(s0_arvalid), .s0_arready(s0_arready),
    .s0_rid(s0_rid), .s0_rdata(s0_rdata), .s0_rresp(s0_rresp), .s0_rvalid(s0_rvalid), .s0_rlast(s0_rlast), .s0_rready(s0_rready),

    // S1
    .s1_awid(s1_awid), .s1_awaddr(s1_awaddr), .s1_awlen(s1_awlen), .s1_awvalid(s1_awvalid), .s1_awready(s1_awready),
    .s1_wdata(s1_wdata), .s1_wlast(s1_wlast), .s1_wvalid(s1_wvalid), .s1_wready(s1_wready),
    .s1_bid(s1_bid), .s1_bresp(s1_bresp), .s1_bvalid(s1_bvalid), .s1_bready(s1_bready),
    .s1_arid(s1_arid), .s1_araddr(s1_araddr), .s1_arlen(s1_arlen), .s1_arvalid(s1_arvalid), .s1_arready(s1_arready),
    .s1_rid(s1_rid), .s1_rdata(s1_rdata), .s1_rresp(s1_rresp), .s1_rvalid(s1_rvalid), .s1_rlast(s1_rlast), .s1_rready(s1_rready),

    // S2
    .s2_awid(s2_awid), .s2_awaddr(s2_awaddr), .s2_awlen(s2_awlen), .s2_awvalid(s2_awvalid), .s2_awready(s2_awready),
    .s2_wdata(s2_wdata), .s2_wlast(s2_wlast), .s2_wvalid(s2_wvalid), .s2_wready(s2_wready),
    .s2_bid(s2_bid), .s2_bresp(s2_bresp), .s2_bvalid(s2_bvalid), .s2_bready(s2_bready),
    .s2_arid(s2_arid), .s2_araddr(s2_araddr), .s2_arlen(s2_arlen), .s2_arvalid(s2_arvalid), .s2_arready(s2_arready),
    .s2_rid(s2_rid), .s2_rdata(s2_rdata), .s2_rresp(s2_rresp), .s2_rvalid(s2_rvalid), .s2_rlast(s2_rlast), .s2_rready(s2_rready)
  );

// ------------------------------------------------------------
// NPU trace-driven AXI master (replaces CPU on M0)
// ------------------------------------------------------------
// ------------------------------------------------------------
// NPU AXI master(s)
//   NOTE: DDR model is simplified and only supports 1 active read burst at a time.
//   To avoid accidental MRAM address reads when WEIGHT_MRAM_PCT=0, we instantiate
//   two NPU trace masters with different WEIGHT_BASE and mux their AXI outputs.
// ------------------------------------------------------------
localparam [31:0] WEIGHT_BASE_MRAM = 32'h0001_1000;
localparam [31:0] WEIGHT_BASE_DDR  = 32'h1001_1000;

// Aliases for older naming used in some experiments
// Weight base addresses (named to avoid collisions with existing parameter names)
localparam [31:0] MRAM_WEIGHT_BASE = WEIGHT_BASE_MRAM;
localparam [31:0] DDR_W_BASE       = WEIGHT_BASE_DDR; // kept for backward compatibility in log prints
localparam [31:0] DRAM_WEIGHT_BASE = WEIGHT_BASE_DDR;

logic use_mram_npu;
always @(*) use_mram_npu = (weight_mram_pct != 0);

// Start is asserted only to the selected instance.
wire npu_start_mram = npu_start &  use_mram_npu;
wire npu_start_ddr  = npu_start & ~use_mram_npu;

// Per-instance done
wire npu_done_mram, npu_done_ddr;

// Per-instance AXI outputs
wire [AXI_ID_WIDTH-1:0]   m0_awid_mram,   m0_awid_ddr;
wire [AXI_ADDR_WIDTH-1:0] m0_awaddr_mram, m0_awaddr_ddr;
wire [7:0]                m0_awlen_mram,  m0_awlen_ddr;
wire [2:0]                m0_awsize_mram, m0_awsize_ddr;
wire [1:0]                m0_awburst_mram,m0_awburst_ddr;
wire                      m0_awvalid_mram,m0_awvalid_ddr;

wire [AXI_DATA_WIDTH-1:0] m0_wdata_mram,  m0_wdata_ddr;
wire [AXI_DATA_WIDTH/8-1:0] m0_wstrb_mram, m0_wstrb_ddr;
wire                      m0_wlast_mram,  m0_wlast_ddr;
wire                      m0_wvalid_mram, m0_wvalid_ddr;
wire                      m0_bready_mram, m0_bready_ddr;

wire [AXI_ID_WIDTH-1:0]   m0_arid_mram,   m0_arid_ddr;
wire [AXI_ADDR_WIDTH-1:0] m0_araddr_mram, m0_araddr_ddr;
wire [7:0]                m0_arlen_mram,  m0_arlen_ddr;
wire [2:0]                m0_arsize_mram, m0_arsize_ddr;
wire [1:0]                m0_arburst_mram,m0_arburst_ddr;
wire                      m0_arvalid_mram,m0_arvalid_ddr;
wire                      m0_rready_mram, m0_rready_ddr;

// Mux to DUT (m0_* are the DUT-facing wires already declared above)
assign m0_awid    = use_mram_npu ? m0_awid_mram    : m0_awid_ddr;
assign m0_awaddr  = use_mram_npu ? m0_awaddr_mram  : m0_awaddr_ddr;
assign m0_awlen   = use_mram_npu ? m0_awlen_mram   : m0_awlen_ddr;
assign m0_awsize  = use_mram_npu ? m0_awsize_mram  : m0_awsize_ddr;
assign m0_awburst = use_mram_npu ? m0_awburst_mram : m0_awburst_ddr;
assign m0_awvalid = use_mram_npu ? m0_awvalid_mram : m0_awvalid_ddr;

assign m0_wdata   = use_mram_npu ? m0_wdata_mram   : m0_wdata_ddr;
assign m0_wstrb   = use_mram_npu ? m0_wstrb_mram   : m0_wstrb_ddr;
assign m0_wlast   = use_mram_npu ? m0_wlast_mram   : m0_wlast_ddr;
assign m0_wvalid  = use_mram_npu ? m0_wvalid_mram  : m0_wvalid_ddr;
assign m0_bready  = use_mram_npu ? m0_bready_mram  : m0_bready_ddr;

assign m0_arid    = use_mram_npu ? m0_arid_mram    : m0_arid_ddr;
assign m0_araddr  = use_mram_npu ? m0_araddr_mram  : m0_araddr_ddr;
assign m0_arlen   = use_mram_npu ? m0_arlen_mram   : m0_arlen_ddr;
assign m0_arsize  = use_mram_npu ? m0_arsize_mram  : m0_arsize_ddr;
assign m0_arburst = use_mram_npu ? m0_arburst_mram : m0_arburst_ddr;
assign m0_arvalid = use_mram_npu ? m0_arvalid_mram : m0_arvalid_ddr;
assign m0_rready  = use_mram_npu ? m0_rready_mram  : m0_rready_ddr;

// Select done
always @(*) npu_done = use_mram_npu ? npu_done_mram : npu_done_ddr;

// MRAM-weight NPU master
npu_trace_axi_master_no_reuse #(
  .AXI_ID_WIDTH   (AXI_ID_WIDTH),
  .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
  .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
  .WEIGHT_BASE    (WEIGHT_BASE_MRAM),
  .INPUT_BASE     (32'h1000_2000),
  .OUTPUT_BASE    (32'h2000_3000)
) u_npu_mram (
  .clk      (clk),
  .rst_n    (rst_n),
  .start    (npu_start_mram),
  .done     (npu_done_mram),

  .m_awid    (m0_awid_mram),
  .m_awaddr  (m0_awaddr_mram),
  .m_awlen   (m0_awlen_mram),
  .m_awsize  (m0_awsize_mram),
  .m_awburst (m0_awburst_mram),
  .m_awvalid (m0_awvalid_mram),
  .m_awready (m0_awready),

  .m_wdata   (m0_wdata_mram),
  .m_wstrb   (m0_wstrb_mram),
  .m_wlast   (m0_wlast_mram),
  .m_wvalid  (m0_wvalid_mram),
  .m_wready  (m0_wready),

  .m_bid     (m0_bid),
  .m_bresp   (m0_bresp),
  .m_bvalid  (m0_bvalid),
  .m_bready  (m0_bready_mram),

  .m_arid    (m0_arid_mram),
  .m_araddr  (m0_araddr_mram),
  .m_arlen   (m0_arlen_mram),
  .m_arsize  (m0_arsize_mram),
  .m_arburst (m0_arburst_mram),
  .m_arvalid (m0_arvalid_mram),
  .m_arready (m0_arready),

  .m_rid     (m0_rid),
  .m_rdata   (m0_rdata),
  .m_rresp   (m0_rresp),
  .m_rlast   (m0_rlast),
  .m_rvalid  (m0_rvalid),
  .m_rready  (m0_rready_mram)
);

// DDR-weight NPU master
npu_trace_axi_master_no_reuse #(
  .AXI_ID_WIDTH   (AXI_ID_WIDTH),
  .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
  .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
  .WEIGHT_BASE    (WEIGHT_BASE_DDR),
  .INPUT_BASE     (32'h1000_2000),
	  .OUTPUT_BASE    (32'h2000_3000)
) u_npu_ddr (
  .clk      (clk),
  .rst_n    (rst_n),
  .start    (npu_start_ddr),
  .done     (npu_done_ddr),

  .m_awid    (m0_awid_ddr),
  .m_awaddr  (m0_awaddr_ddr),
  .m_awlen   (m0_awlen_ddr),
  .m_awsize  (m0_awsize_ddr),
  .m_awburst (m0_awburst_ddr),
  .m_awvalid (m0_awvalid_ddr),
  .m_awready (m0_awready),

  .m_wdata   (m0_wdata_ddr),
  .m_wstrb   (m0_wstrb_ddr),
  .m_wlast   (m0_wlast_ddr),
  .m_wvalid  (m0_wvalid_ddr),
  .m_wready  (m0_wready),

  .m_bid     (m0_bid),
  .m_bresp   (m0_bresp),
  .m_bvalid  (m0_bvalid),
  .m_bready  (m0_bready_ddr),

  .m_arid    (m0_arid_ddr),
  .m_araddr  (m0_araddr_ddr),
  .m_arlen   (m0_arlen_ddr),
  .m_arsize  (m0_arsize_ddr),
  .m_arburst (m0_arburst_ddr),
  .m_arvalid (m0_arvalid_ddr),
  .m_arready (m0_arready),

  .m_rid     (m0_rid),
  .m_rdata   (m0_rdata),
  .m_rresp   (m0_rresp),
  .m_rlast   (m0_rlast),
  .m_rvalid  (m0_rvalid),
  .m_rready  (m0_rready_ddr)
);
// -------------------------
  // Slave0: MRAM AXI slave + MRAM model
  // -------------------------
  logic [AXI_ADDR_WIDTH-1:0] mram_addr;
  logic [AXI_DATA_WIDTH-1:0] mram_wdata;
  logic                      mram_write_en, mram_read_en, mram_cs;
  logic [AXI_DATA_WIDTH-1:0] mram_rdata;
  logic                      mram_ready;

  // IMPORTANT: connect pins for power monitor
  assign mram_we_en = mram_write_en;
  assign mram_rd_en = mram_read_en;

  axi_mram_slave_ctrl #(
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .BURST_LEN(16),
    .MRAM_READ_LAT(2)
  ) u_axi_mram (
    .clk(clk), .rst_n(rst_n),
    .write_delay_config(write_delay_config),

    .awid(s0_awid), .awaddr(s0_awaddr), .awlen(s0_awlen), .awvalid(s0_awvalid), .awready(s0_awready),
    .wdata(s0_wdata), .wlast(s0_wlast), .wvalid(s0_wvalid), .wready(s0_wready),
    .bid(s0_bid), .bresp(s0_bresp), .bvalid(s0_bvalid), .bready(s0_bready),
    .arid(s0_arid), .araddr(s0_araddr), .arlen(s0_arlen), .arvalid(s0_arvalid), .arready(s0_arready),
    .rid(s0_rid), .rdata(s0_rdata), .rresp(s0_rresp), .rvalid(s0_rvalid), .rlast(s0_rlast), .rready(s0_rready),

    .mram_addr(mram_addr),
    .mram_wdata(mram_wdata),
    .mram_write_en(mram_write_en),
    .mram_read_en(mram_read_en),
    .mram_cs(mram_cs),
    .mram_rdata(mram_rdata),
    .mram_ready(mram_ready),
    .mram_pwr_on(mram_pwr_on)
  );

  mram_model #(
    .ADDR_WIDTH(AXI_ADDR_WIDTH),
    .DATA_WIDTH(AXI_DATA_WIDTH),
    .DEPTH_WORDS(MRAM_DEPTH_WORDS),
    .READ_LAT(2)
  ) u_mram (
    .clk(clk), .rst_n(rst_n),
    .mram_addr(mram_addr),
    .mram_wdata(mram_wdata),
    .mram_write_en(mram_write_en),
    .mram_read_en(mram_read_en),
    .mram_cs(mram_cs),
    .mram_rdata(mram_rdata),
    .mram_ready(mram_ready),
    .mram_pwr_on(mram_pwr_on)
  );

  // -------------------------
  // Slave1: DDR4 model
  // -------------------------
  axi_ddr4_slave_model #(
    .AXI_ID_WIDTH (AXI_ID_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .DEPTH_WORDS  (8192),
    .DDR_RD_LAT   (2),
    .DDR_WR_LAT   (100),
	.BASE_ADDR    (32'h1000_0000)   // <<< 加這個
  ) u_ddr4 (
    .clk(clk), .rst_n(rst_n),

    .awid(s1_awid), .awaddr(s1_awaddr), .awlen(s1_awlen), .awvalid(s1_awvalid), .awready(s1_awready),
    .wdata(s1_wdata), .wlast(s1_wlast), .wvalid(s1_wvalid), .wready(s1_wready),
    .bid(s1_bid), .bresp(s1_bresp), .bvalid(s1_bvalid), .bready(s1_bready),
    .arid(s1_arid), .araddr(s1_araddr), .arlen(s1_arlen), .arvalid(s1_arvalid), .arready(s1_arready),
    .rid(s1_rid), .rdata(s1_rdata), .rresp(s1_rresp), .rvalid(s1_rvalid), .rlast(s1_rlast), .rready(s1_rready)
  );

  // -------------------------
  // Slave2: SRAM model
  // IMPORTANT:
  // Since SRAM is mapped to 0x2000_0000 ~ 0x2FFF_FFFF,
  // set BASE_ADDR = 0x2000_0000 so index calc is BASE-relative.
  // -------------------------
  axi_sram_slave_model #(
    .AXI_ID_WIDTH (AXI_ID_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .DEPTH_WORDS  (8192),
    .BASE_ADDR    (32'h2000_0000)
  ) u_sram (
    .clk(clk), .rst_n(rst_n),

    .awid(s2_awid), .awaddr(s2_awaddr), .awlen(s2_awlen), .awvalid(s2_awvalid), .awready(s2_awready),
    .wdata(s2_wdata), .wlast(s2_wlast), .wvalid(s2_wvalid), .wready(s2_wready),
    .bid(s2_bid), .bresp(s2_bresp), .bvalid(s2_bvalid), .bready(s2_bready),

    .arid(s2_arid), .araddr(s2_araddr), .arlen(s2_arlen), .arvalid(s2_arvalid), .arready(s2_arready),
    .rid(s2_rid), .rdata(s2_rdata), .rresp(s2_rresp), .rvalid(s2_rvalid), .rlast(s2_rlast), .rready(s2_rready)
  );

  // ============================================================
  // POWER REPORT CONTROL (pulse)
  // ============================================================
  logic report_pulse, clear_pulse;

  // ============================================================
  // MRAM POWER-GATING (Level A): affects power model only
  // ============================================================
  // 之前 TB 在「frame 計算期間」把 mram_pwr_on 強制為 1，
  // 所以你在 GTWave 會看到中間沒有 access MRAM 但仍然是 pwr_on。
  // 這版改成：AUTO 模式下只要真的有 S0(MRAM) 的 in-flight 存取才會開。
  typedef enum logic [1:0] {
    MRAM_PWR_AUTO = 2'd0,
    MRAM_PWR_ON   = 2'd1,
    MRAM_PWR_OFF  = 2'd2
  } mram_pwr_mode_e;

  mram_pwr_mode_e mram_pwr_mode;
  logic           mram_pwr_on;

  // ------------------------------------------------------------------
  // Robust MRAM address-range detection
  // (Used to decide when MRAM really needs to be powered ON)
  // Keep consistent with u_mram: BASE_ADDR=0x0001_0000, DEPTH_WORDS=524288
  // ------------------------------------------------------------------
  function automatic bit is_mram_addr(input logic [31:0] a);
    logic [31:0] lo;
    logic [31:0] hi;
    begin
      lo = 32'h0001_0000;
      hi = lo + (32'd524288 * 32'd4) - 32'd1;
      is_mram_addr = (a >= lo) && (a <= hi);
    end
  endfunction

  // Track S0(MRAM) in-flight transactions to avoid turning power off too early.
  logic s0_rd_inflight, s0_wr_inflight;
  wire  s0_ar_hs    = s0_arvalid && s0_arready;
  wire  s0_aw_hs    = s0_awvalid && s0_awready;
  wire  s0_r_last_hs= s0_rvalid  && s0_rready && s0_rlast;
  wire  s0_b_hs     = s0_bvalid  && s0_bready;

  // Some runs showed a mismatch where MRAM reads are happening (seen at master side)
  // but the MRAM power signal was mostly treated as OFF due to short/hidden activity
  // on the slave side (e.g., buffering/skid effects).
  // To make the power-on decision consistent with *actual* MRAM traffic, also look at
  // master-side requests that target the MRAM address range.
  wire  m0_mram_req = (m0_arvalid && is_mram_addr(m0_araddr)) || (m0_awvalid && is_mram_addr(m0_awaddr));
  wire  m1_mram_req = (m1_arvalid && is_mram_addr(m1_araddr)) || (m1_awvalid && is_mram_addr(m1_awaddr));

  // "Need MRAM ON" if any MRAM transaction is pending or trying to start.
  // Include both slave-side and master-side indicators.
  wire  mram_need_on = s0_rd_inflight || s0_wr_inflight ||
                       s0_arvalid || s0_awvalid || s0_rvalid || s0_wvalid ||
                       m0_mram_req || m1_mram_req;
  integer vblank_cnt;
  integer mram_wake_cnt;
  localparam int unsigned NUM_FRAMES  = 10;
  localparam int unsigned VBLANK_CYC  = 100000; // vblank duration in cycles (tune)


  initial begin
    report_pulse = 0;
    clear_pulse  = 0;
    mram_pwr_mode = MRAM_PWR_ON; // reset/bring-up: keep ON
    mram_pwr_on   = 1'b1;
    vblank_cnt   = 0;
    mram_wake_cnt = 0;

    // clear after reset
    wait (rst_n === 1'b1);
    repeat (5) @(posedge clk);
    clear_pulse <= 1;
    @(posedge clk);
    clear_pulse <= 0;
  end

  // Track MRAM (S0) inflight status
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_rd_inflight <= 1'b0;
      s0_wr_inflight <= 1'b0;
    end else begin
      if (s0_ar_hs)     s0_rd_inflight <= 1'b1;
      if (s0_r_last_hs) s0_rd_inflight <= 1'b0;

      if (s0_aw_hs)  s0_wr_inflight <= 1'b1;
      if (s0_b_hs)   s0_wr_inflight <= 1'b0;
    end
  end

  // Final MRAM power state
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mram_pwr_on <= 1'b1;
    end else begin
      unique case (mram_pwr_mode)
        MRAM_PWR_ON:   mram_pwr_on <= 1'b1;
        MRAM_PWR_OFF:  mram_pwr_on <= 1'b0;
        default:       mram_pwr_on <= mram_need_on; // MRAM_PWR_AUTO
      endcase
    end
  end

  task automatic report_power();
    report_pulse <= 1;
    @(posedge clk);
    report_pulse <= 0;
  endtask

  // ---- Monitor for MRAM (S0) ----
  soc_mem_power_monitor #(
  //.FCLK_HZ(100_000_000.0),
  .FCLK_HZ(200_000_000.0),
  //.FCLK_HZ(400_000_000.0),
  .VDD(0.8),  // MRAM domain example
  .VDDQ(0.0), // no I/O rail in this simplified MRAM
  .REF_EN(0),

   // used for DDR4
  .I_LEAK_VDD_A(0.0),
  .I_LEAK_VDDQ_A(0.0),
  .I_BG_VDD_A(0.0),  
  .I_BG_VDDQ_A(0.0),
  .I_RD_VDD_A(0.0),  
  .I_RD_VDDQ_A(0.0),
  .I_WR_VDD_A(0.0),  
  .I_WR_VDDQ_A(0.0),

  // MRAM: low leakage, moderate read, HIGH write (write > DRAM write)
  .MRAM_I_LEAK_A (0.0001),  // 0.1 mA @0.8V (retention/leakage-ish)
  .MRAM_I_RD_A   (0.03),    // 30 mA  during charged RD cycles
  .MRAM_I_WR_A   (0.055),   // 55 mA during charged WR cycles  (<< make WR expensive)
  .MRAM_RD_CYCLES(2),
  .MRAM_WR_CYCLES(20),      // keep long-ish program time

  // Weight-range tracking (use a fixed superset span)
  .WEIGHT_BASE (WEIGHT_BASE_MRAM),
  .WEIGHT_BYTES(MIN_WEIGHT_BYTES_SIM)

  ) u_pwr_mram (
    .clk(clk),
    .rst_n(rst_n),



    .mem_pwr_on(mram_pwr_on),

    .s_awid   (s0_awid),
    .s_awaddr (s0_awaddr),
    .s_awlen  (s0_awlen),
    .s_awvalid(s0_awvalid),
    .s_awready(s0_awready),

    .s_wdata  (s0_wdata),
    .s_wstrb  (s0_wstrb),
    .s_wlast  (s0_wlast),
    .s_wvalid (s0_wvalid),
    .s_wready (s0_wready),

    .s_bid    (s0_bid),
    .s_bresp  (s0_bresp),
    .s_bvalid (s0_bvalid),
    .s_bready (s0_bready),

    .s_arid   (s0_arid),
    .s_araddr (s0_araddr),
    .s_arlen  (s0_arlen),
    .s_arvalid(s0_arvalid),
    .s_arready(s0_arready),

    .s_rid    (s0_rid),
    .s_rdata  (s0_rdata),
    .s_rresp  (s0_rresp),
    .s_rlast  (s0_rlast),
    .s_rvalid (s0_rvalid),
    .s_rready (s0_rready),

    .m_cs     (mram_cs),
    .m_we_en  (mram_we_en),
    .m_rd_en  (mram_rd_en),

    .report_pulse(pwr_report_pulse),
    .clear_pulse (clear_pulse),
  
    .o_t_total_s(mram_t_s),
    .o_e_ddr_j(mram_e_ddr_j),
    .o_e_mram_j(mram_e_mram_j),
    .o_e_total_j(mram_e_total_j),
    .o_p_ddr_w(mram_p_ddr_w),
    .o_p_mram_w(mram_p_mram_w),
    .o_p_total_w(mram_p_total_w)
  );
// ---- Monitor for DDR4 (S1) ----
  soc_mem_power_monitor #(
  //.FCLK_HZ(100_000_000.0),
  .FCLK_HZ(200_000_000.0),
  //.FCLK_HZ(400_000_000.0),
  .VDD(1.2),  // DDR4 core
  .VDDQ(1.2), // DDR4 I/O 

  // Refresh model (approx. DDR background penalty)
  .REF_EN(1),
  .REF_PERIOD_CYC(1560), // 7.8us @200MHz
  .REF_BUSY_CYC(32),

 // Standby leakage (representative IDD2N-inspired)
  .I_LEAK_VDD_A (0.020),  // 20 mA @1.2V
  .I_LEAK_VDDQ_A(0.010),  // 10 mA @1.2V

  // Background delta (IDD3N-IDD2N inspired)
  .I_BG_VDD_A   (0.012),  // 12 mA
  .I_BG_VDDQ_A  (0.006),  // 6 mA

  // Read delta (IDD4R-IDD3N inspired)
  .I_RD_VDD_A   (0.028),  // 28 mA
  .I_RD_VDDQ_A  (0.012),  // 12 mA

  // Write delta (IDD4W-IDD3N inspired)  (DRAM write < MRAM write in our chosen set)
  .I_WR_VDD_A   (0.032),  // 32 mA
  .I_WR_VDDQ_A  (0.014),  // 14 mA

  // Weight-range tracking (use a fixed superset span)
  .WEIGHT_BASE (WEIGHT_BASE_DDR),
  .WEIGHT_BYTES(MIN_WEIGHT_BYTES_SIM)

  ) u_pwr_ddr (
    .clk(clk),
    .rst_n(rst_n),

    // DDR monitor does not use MRAM pin model; keep it disabled.


    .mem_pwr_on(1'b1),

    .s_awid   (s1_awid),
    .s_awaddr (s1_awaddr),
    .s_awlen  (s1_awlen),
    .s_awvalid(s1_awvalid),
    .s_awready(s1_awready),

    .s_wdata  (s1_wdata),
    .s_wstrb  (s1_wstrb),
    .s_wlast  (s1_wlast),
    .s_wvalid (s1_wvalid),
    .s_wready (s1_wready),

    .s_bid    (s1_bid),
    .s_bresp  (s1_bresp),
    .s_bvalid (s1_bvalid),
    .s_bready (s1_bready),

    .s_arid   (s1_arid),
    .s_araddr (s1_araddr),
    .s_arlen  (s1_arlen),
    .s_arvalid(s1_arvalid),
    .s_arready(s1_arready),

    .s_rid    (s1_rid),
    .s_rdata  (s1_rdata),
    .s_rresp  (s1_rresp),
    .s_rlast  (s1_rlast),
    .s_rvalid (s1_rvalid),
    .s_rready (s1_rready),

    .m_cs     (1'b0),
    .m_we_en  (1'b0),
    .m_rd_en  (1'b0),

    .report_pulse(pwr_report_pulse),
    .clear_pulse (clear_pulse)
  ,
    .o_t_total_s(ddr_t_s),
    .o_e_ddr_j(ddr_e_ddr_j),
    .o_e_mram_j(ddr_e_mram_j),
    .o_e_total_j(ddr_e_total_j),
    .o_p_ddr_w(ddr_p_ddr_w),
    .o_p_mram_w(ddr_p_mram_w),
    .o_p_total_w(ddr_p_total_w)
  );
// -------------------------
  // Reset + defaults
  // -------------------------
    task automatic init_master0();
    // M0 is driven by NPU, do not drive from TB.
  endtask


  task automatic init_master1();
    m1_awid='0; m1_awaddr='0; m1_awlen='0; m1_awvalid=0;
    m1_wdata='0; m1_wlast=0; m1_wvalid=0;
    m1_bready=0;
    m1_arid='0; m1_araddr='0; m1_arlen='0; m1_arvalid=0;
    m1_rready=0;
  endtask

  // -------------------------
  // AXI helper: data pattern
  // -------------------------
  function automatic [63:0] make_data(input int unsigned tag, input int unsigned beat);
    make_data = {32'h2000_0000 + tag[31:0] + beat[31:0],
                 32'hA5A5_0000 + tag[31:0] + beat[31:0]};
  endfunction

  // ============================================================
  // DRAM (S1 / axi_ddr4_slave_model) TEST PATTERN helpers
  // - Similar to MRAM test, but with classic memory patterns
  // - Uses M1 AXI helper style (AW/W/B then AR/R compare)
  // ============================================================

  function automatic [63:0] dram_make_data(input int unsigned pat_sel, input int unsigned beat);
    // 4 classic patterns, add beat to make each beat unique (still deterministic)
    logic [63:0] base;
    case (pat_sel[1:0])
      2'd0: base = 64'h0000_0000_0000_0000;
      2'd1: base = 64'hFFFF_FFFF_FFFF_FFFF;
      2'd2: base = 64'hA5A5_A5A5_A5A5_A5A5;
      default: base = 64'h5A5A_5A5A_5A5A_5A5A;
    endcase
    // Mix in beat index on low bits to avoid identical beats
    dram_make_data = base ^ {48'h0, beat[15:0]};
  endfunction

  task automatic axi_write_burst_dram_pattern(
    input int unsigned mid,
    input logic [AXI_ID_WIDTH-1:0] id,
    input logic [AXI_ADDR_WIDTH-1:0] addr,
    input int unsigned nbeats,
    input int unsigned pat_sel
  );
    int unsigned b;
    int unsigned timeout;

    if (mid != 1) begin
      $display("[%0t] [TB][FATAL] axi_write_burst_dram_pattern: M0 is reserved for NPU. Use mid=1.", $time);
      $fatal;
    end

    // Ensure no outstanding READ overlaps this WRITE (single-outstanding NoC)
    wait_all_reads_done();

    // AW
    @(negedge clk);
    m1_awid    = id;
    m1_awaddr  = addr;
    m1_awlen   = nbeats-1;
    m1_awvalid = 1'b1;

    timeout = 0;
    while (!m1_awready) begin
      @(negedge clk);
      timeout++;
      if (timeout > 2000) begin
        $display("[%0t] [TB][HANG] M1_AW timeout (DRAM) awvalid=%0d awready=%0d", $time, m1_awvalid, m1_awready);
        $fatal;
      end
    end
    @(negedge clk);
    m1_awvalid = 1'b0;

    // W beats
    for (b = 0; b < nbeats; b++) begin
      @(negedge clk);
      m1_wdata  = dram_make_data(pat_sel,b);
      m1_wlast  = (b == nbeats-1);
      m1_wvalid = 1'b1;

      timeout = 0;
      while (!m1_wready) begin
        @(negedge clk);
        timeout++;
        if (timeout > 2000) begin
          $display("[%0t] [TB][HANG] M1_W timeout (DRAM) wvalid=%0d wready=%0d", $time, m1_wvalid, m1_wready);
          $fatal;
        end
      end

      if (VERBOSE_BEAT_LOG) if (VERBOSE_BEAT_LOG) $display("[%0t] [TB] DRAM(W) beat=%0d data=0x%016x last=%0d pat=%0d", $time, b, m1_wdata, m1_wlast, pat_sel);
      @(negedge clk);
      m1_wvalid = 1'b0;
      m1_wlast  = 1'b0;
    end

    // B
    @(negedge clk);
    m1_bready = 1'b1;

    timeout = 0;
    while (!m1_bvalid) begin
      @(negedge clk);
      timeout++;
      if (timeout > 4000) begin
        $display("[%0t] [TB][HANG] M1_B timeout (DRAM) bvalid=%0d bready=%0d", $time, m1_bvalid, m1_bready);
        $fatal;
      end
    end
//    $display("[%0t] [TB] DRAM(B) resp=0x%0x bid=0x%0x pat=%0d", $time, m1_bresp, m1_bid, pat_sel);

    @(negedge clk);
    m1_bready = 1'b0;
  endtask

// Write a large DRAM region by splitting into AXI bursts (AWLEN max 256 beats).
task automatic axi_write_large_dram_pattern(
  input int unsigned mid,
  input logic [AXI_ID_WIDTH-1:0] id,
  input logic [AXI_ADDR_WIDTH-1:0] base_addr,
  input int unsigned total_beats,
  input int unsigned pat_sel
);
  int unsigned off;
  int unsigned chunk;
  for (off = 0; off < total_beats; off += 256) begin
    chunk = (total_beats - off > 256) ? 256 : (total_beats - off);
    axi_write_burst_dram_pattern(mid, id, base_addr + (off * AXI_BEAT_BYTES), chunk, pat_sel);
  end
endtask

// Write a large MRAM region by splitting into AXI bursts (AWLEN max 256 beats).
task automatic axi_write_large_mram(
  input int unsigned mid,
  input logic [AXI_ID_WIDTH-1:0] id,
  input logic [AXI_ADDR_WIDTH-1:0] base_addr,
  input int unsigned total_beats,
  input int unsigned tag
);
  int unsigned off;
  int unsigned chunk;
  for (off = 0; off < total_beats; off += 256) begin
    chunk = (total_beats - off > 256) ? 256 : (total_beats - off);
    axi_write_burst(mid, id, base_addr + (off * AXI_BEAT_BYTES), chunk, tag);
  end
endtask

  task automatic axi_read_burst_and_check_dram_pattern(
    input int unsigned mid,
    input logic [AXI_ID_WIDTH-1:0] id,
    input logic [AXI_ADDR_WIDTH-1:0] addr,
    input int unsigned nbeats,
    input int unsigned pat_sel
  );
    int unsigned b;
    int unsigned timeout;
    logic [AXI_DATA_WIDTH-1:0] exp;

    if (mid != 1) begin
      $display("[%0t] [TB][FATAL] axi_read_burst_and_check_dram_pattern: M0 is reserved for NPU. Use mid=1.", $time);
      $fatal;
    end

    // AR
    @(negedge clk);
    m1_arid    = id;
    m1_araddr  = addr;
    m1_arlen   = nbeats-1;
    m1_arvalid = 1'b1;

    timeout = 0;
    while (!m1_arready) begin
      @(negedge clk);
      timeout++;
      if (timeout > 2000) begin
        $display("[%0t] [TB][HANG] M1_AR timeout (DRAM) arvalid=%0d arready=%0d", $time, m1_arvalid, m1_arready);
        $fatal;
      end
    end

    @(negedge clk);
    m1_arvalid = 1'b0;
    m1_rready  = 1'b1;

    // R beats
    for (b = 0; b < nbeats; b++) begin
      timeout = 0;
      while (!m1_rvalid) begin
        @(negedge clk);
        timeout++;
        if (timeout > 4000) begin
          $display("[%0t] [TB][HANG] M1_R timeout (DRAM) rvalid=%0d rready=%0d", $time, m1_rvalid, m1_rready);
          $fatal;
        end
      end

      exp = dram_make_data(pat_sel,b);
      if (m1_rdata !== exp) begin
        $display("[%0t] [TB][FAIL] DRAM(R) mismatch beat=%0d exp=0x%016x got=0x%016x pat=%0d addr=0x%08x",
                 $time, b, exp, m1_rdata, pat_sel, addr);
        $fatal;
      end
 //     $display("[%0t] [TB] DRAM(R) beat=%0d data=0x%016x last=%0d pat=%0d", $time, b, m1_rdata, m1_rlast, pat_sel);

      if ((b == nbeats-1) && (m1_rlast !== 1'b1)) begin
        $display("[%0t] [TB][FAIL] DRAM RLAST not asserted on last beat (pat=%0d)", $time, pat_sel);
        $fatal;
      end

      @(negedge clk);
    end

    @(negedge clk);
    m1_rready = 1'b0;
  endtask


  // ============================================================
  // AXI helper tasks
  // ============================================================

  // ------------------------------------------------------------
  // Wait until all outstanding READs are finished
  // (for single-outstanding NoC: do NOT overlap READ and WRITE)
  // ------------------------------------------------------------
  task automatic wait_all_reads_done;
    int unsigned timeout;
    begin
      timeout = 0;
      // Wait until no R-channel activity is observed on fabric/master/slave sides.
      while (m0_rvalid || m1_rvalid || s0_rvalid || s1_rvalid || s2_rvalid) begin
        @(negedge clk);
        timeout++;
        if (timeout > 5000) begin
          $display("[%0t] [TB][HANG] wait_all_reads_done timeout", $time);
          $fatal;
        end
      end
    end
  endtask

  // ============================================================
  // SRAM (S2) TEST PATTERN helpers
  // - Uses same classic patterns as DRAM
  // - Assumes S2 is SRAM slave (1-cycle or small fixed latency)
  // ============================================================

  function automatic [63:0] sram_make_data(input int unsigned pat_sel, input int unsigned beat);
    logic [63:0] base;
    case (pat_sel[1:0])
      2'd0: base = 64'h0000_0000_0000_0000;
      2'd1: base = 64'hFFFF_FFFF_FFFF_FFFF;
      2'd2: base = 64'hA5A5_A5A5_A5A5_A5A5;
      default: base = 64'h5A5A_5A5A_5A5A_5A5A;
    endcase
    sram_make_data = base ^ {48'h0, beat[15:0]};
  endfunction

  task automatic axi_write_burst_sram_pattern(
    input int unsigned mid,
    input logic [AXI_ID_WIDTH-1:0] id,
    input logic [AXI_ADDR_WIDTH-1:0] addr,
    input int unsigned nbeats,
    input int unsigned pat_sel
  );
    int unsigned b;
    int unsigned timeout;

    if (mid != 1) begin
      $display("[%0t] [TB][FATAL] axi_write_burst_sram_pattern: use mid=1.", $time);
      $fatal;
    end

    wait_all_reads_done();

    // AW
    @(negedge clk);
    m1_awid    = id;
    m1_awaddr  = addr;
    m1_awlen   = nbeats-1;
    m1_awvalid = 1'b1;

    timeout = 0;
    while (!m1_awready) begin
      @(negedge clk);
      if (++timeout > 2000) begin
        $display("[%0t] [TB][HANG] M1_AW timeout (SRAM)", $time);
        $fatal;
      end
    end
    @(negedge clk);
    m1_awvalid = 1'b0;

    // W
    for (b = 0; b < nbeats; b++) begin
      @(negedge clk);
      m1_wdata  = sram_make_data(pat_sel,b);
      m1_wlast  = (b == nbeats-1);
      m1_wvalid = 1'b1;

      timeout = 0;
      while (!m1_wready) begin
        @(negedge clk);
        if (++timeout > 2000) begin
          $display("[%0t] [TB][HANG] M1_W timeout (SRAM)", $time);
          $fatal;
        end
      end
      @(negedge clk);
      m1_wvalid = 1'b0;
      m1_wlast  = 1'b0;
    end

    // B
    @(negedge clk);
    m1_bready = 1'b1;
    timeout = 0;
    while (!m1_bvalid) begin
      @(negedge clk);
      if (++timeout > 2000) begin
        $display("[%0t] [TB][HANG] M1_B timeout (SRAM)", $time);
        $fatal;
      end
    end
    @(negedge clk);
    m1_bready = 1'b0;
  endtask

  task automatic axi_read_burst_and_check_sram_pattern(
    input int unsigned mid,
    input logic [AXI_ID_WIDTH-1:0] id,
    input logic [AXI_ADDR_WIDTH-1:0] addr,
    input int unsigned nbeats,
    input int unsigned pat_sel
  );
    int unsigned b;
    int unsigned timeout;
    logic [63:0] exp;

    if (mid != 1) begin
      $display("[%0t] [TB][FATAL] axi_read_burst_and_check_sram_pattern: use mid=1.", $time);
      $fatal;
    end

    // AR
    @(negedge clk);
    m1_arid    = id;
    m1_araddr  = addr;
    m1_arlen   = nbeats-1;
    m1_arvalid = 1'b1;

    timeout = 0;
    while (!m1_arready) begin
      @(negedge clk);
      if (++timeout > 2000) begin
        $display("[%0t] [TB][HANG] M1_AR timeout (SRAM)", $time);
        $fatal;
      end
    end
    @(negedge clk);
    m1_arvalid = 1'b0;
    m1_rready  = 1'b1;

    for (b = 0; b < nbeats; b++) begin
      timeout = 0;
      while (!m1_rvalid) begin
        @(negedge clk);
        if (++timeout > 2000) begin
          $display("[%0t] [TB][HANG] M1_R timeout (SRAM)", $time);
          $fatal;
        end
      end

      exp = sram_make_data(pat_sel,b);
      if (m1_rdata !== exp) begin
        $display("[%0t] [TB][FAIL] SRAM(R) beat=%0d exp=0x%016x got=0x%016x",
                 $time, b, exp, m1_rdata);
        $fatal;
      end
      if ((b == nbeats-1) && (m1_rlast !== 1'b1)) begin
        $display("[%0t] [TB][FAIL] SRAM RLAST missing", $time);
        $fatal;
      end
      @(negedge clk);
    end

    @(negedge clk);
    m1_rready = 1'b0;
  endtask


  // ============================================================
  // Generate 4K traffic on DDR4(S1) and SRAM(S2) using DMA(M1)
  // - Writes DDR4 tile buffer with a simple pattern
  // - Reads back a subset to validate functionality (optional)
  // - Writes SRAM as "output" traffic of same volume (configurable ratio)
  // ============================================================
  task automatic run_4k_streaming_traffic(input int unsigned frame_idx);
    int unsigned beat;
    int unsigned burst_beats;
    int unsigned offset_bytes;
    logic [31:0] ddr_addr;
    logic [31:0] sram_addr;
    int unsigned remaining_beats;

    remaining_beats = frame_beats_sim;
    offset_bytes = 0;

    // --- DDR4 write stream (simulate "4K input loaded to DDR4") ---
    while (remaining_beats != 0) begin
      burst_beats = (remaining_beats > 16) ? 16 : remaining_beats;
      ddr_addr = wrap_addr(32'h1000_0000, offset_bytes, DDR4_SIZE_BYTES);

      // Use pat_sel = frame_idx[1:0] to vary pattern per frame
      axi_write_burst_dram_pattern(1, 4'h1, ddr_addr, burst_beats, frame_idx[1:0]);

      remaining_beats -= burst_beats;
      offset_bytes    += burst_beats * AXI_BEAT_BYTES;
    end

    // Optional quick sanity check: read back first 16 beats of this frame
    axi_read_burst_and_check_dram_pattern(1, 4'h1, 32'h1000_0000, 16, frame_idx[1:0]);

    // --- SRAM write stream (simulate "4K output written to SRAM") ---
    // Many pipelines write less than full RGB; keep it 1:1 for worst-case.
    remaining_beats = frame_beats_sim;
    offset_bytes = 0;
    while (remaining_beats != 0) begin
      burst_beats = (remaining_beats > 16) ? 16 : remaining_beats;
      sram_addr = wrap_addr(32'h2000_0000, offset_bytes, SRAM_SIZE_BYTES);

      axi_write_burst_sram_pattern(1, 4'h3, sram_addr, burst_beats, frame_idx[1:0]);

      remaining_beats -= burst_beats;
      offset_bytes    += burst_beats * AXI_BEAT_BYTES;
    end

    // Optional quick sanity check: read back first 16 beats of SRAM output
    axi_read_burst_and_check_sram_pattern(1, 4'h3, 32'h2000_0000, 16, frame_idx[1:0]);

    $display("[%0t] [TB] 4K traffic (scaled) DONE for frame %0d (effective_bytes=%0d, simulated_bytes=%0d, scale=%0d, DDR4_buf=%0dB, SRAM_buf=%0dB)",
             $time, frame_idx, FRAME_BYTES, FRAME_BYTES_SIM, FRAME_SCALE, DDR4_SIZE_BYTES, SRAM_SIZE_BYTES);
  endtask



    task automatic axi_write_burst(
    input int unsigned mid,
    input logic [AXI_ID_WIDTH-1:0] id,
    input logic [AXI_ADDR_WIDTH-1:0] addr,
    input int unsigned nbeats,
    input int unsigned tag
  );
    int unsigned b;
    int unsigned timeout;

    if (mid != 1) begin
    int unsigned __wd;
    __wd = 0;

      $display("[%0t] [TB][FATAL] axi_write_burst: M0 is reserved for NPU. Use mid=1.", $time);
      $fatal;
    end

    // Ensure no outstanding READ overlaps this WRITE (single-outstanding NoC)
    wait_all_reads_done();


    // --------------------
    // AW
    // --------------------
    @(negedge clk);
    m1_awid   = id;
    m1_awaddr = addr;
    m1_awlen  = nbeats-1;
    m1_awvalid= 1'b1;

    timeout = 0;
    while (!m1_awready) begin
      @(negedge clk);
      timeout++;
      if (timeout > 2000) begin
        $display("[%0t] [TB][HANG] M1_AW timeout awvalid=%0d awready=%0d", $time, m1_awvalid, m1_awready);
        $fatal;
      end
    end
    @(negedge clk);
    m1_awvalid = 1'b0;

    // --------------------
    // W beats
    // --------------------
    for (b = 0; b < nbeats; b++) begin
      @(negedge clk);
      m1_wdata  = make_data(tag,b);
      m1_wlast  = (b == nbeats-1);
      m1_wvalid = 1'b1;

      timeout = 0;
      while (!m1_wready) begin
        @(negedge clk);
        timeout++;
        if (timeout > 2000) begin
          $display("[%0t] [TB][HANG] M1_W timeout wvalid=%0d wready=%0d", $time, m1_wvalid, m1_wready);
          $fatal;
        end
      end

//      $display("[%0t] [TB] M1 W beat=%0d data=0x%016x last=%0d", $time, b, m1_wdata, m1_wlast);
      @(negedge clk);
      m1_wvalid = 1'b0;
      m1_wlast  = 1'b0;
    end

    // --------------------
    // B
    // --------------------
    @(negedge clk);
    m1_bready = 1'b1;

    timeout = 0;
    while (!m1_bvalid) begin
      @(negedge clk);
      timeout++;
      if (timeout > 4000) begin
        $display("[%0t] [TB][HANG] M1_B timeout bvalid=%0d bready=%0d", $time, m1_bvalid, m1_bready);
        $fatal;
      end
    end
//    $display("[%0t] [TB] M1 B resp=0x%0x bid=0x%0x", $time, m1_bresp, m1_bid);

    @(negedge clk);
    m1_bready = 1'b0;
  endtask


  // ============================================================
  // READ + CHECK  (加入 AR watchdog debug)
  // ============================================================
    task automatic axi_read_burst_and_check(
    input int unsigned mid,
    input logic [AXI_ID_WIDTH-1:0] id,
    input logic [AXI_ADDR_WIDTH-1:0] addr,
    input int unsigned nbeats,
    input int unsigned tag
  );
    int unsigned b;
    int unsigned timeout;
    logic [AXI_DATA_WIDTH-1:0] exp;

    if (mid != 1) begin
      $display("[%0t] [TB][FATAL] axi_read_burst_and_check: M0 is reserved for NPU. Use mid=1.", $time);
      $fatal;
    end

    // --------------------
    // AR
    // --------------------
    @(negedge clk);
    m1_arid    = id;
    m1_araddr  = addr;
    m1_arlen   = nbeats-1;
    m1_arvalid = 1'b1;

    timeout = 0;
    while (!m1_arready) begin
      @(negedge clk);
      timeout++;
      if (timeout > 2000) begin
        $display("[%0t] [TB][HANG] M1_AR timeout arvalid=%0d arready=%0d", $time, m1_arvalid, m1_arready);
        $fatal;
      end
    end

    @(negedge clk);
    m1_arvalid = 1'b0;
    m1_rready  = 1'b1;

    // --------------------
    // R beats
    // --------------------
    for (b = 0; b < nbeats; b++) begin
      timeout = 0;
      while (!m1_rvalid) begin
        @(negedge clk);
        timeout++;
        if (timeout > 4000) begin
          $display("[%0t] [TB][HANG] M1_R timeout rvalid=%0d rready=%0d", $time, m1_rvalid, m1_rready);
          $fatal;
        end
      end

      exp = make_data(tag,b);
      if (m1_rdata !== exp) begin
        $display("[%0t] [TB][FAIL] M1 RDATA mismatch beat=%0d exp=0x%016x got=0x%016x", $time, b, exp, m1_rdata);
        $fatal;
      end
//      $display("[%0t] [TB] M1 R beat=%0d data=0x%016x last=%0d", $time, b, m1_rdata, m1_rlast);

      if ((b == nbeats-1) && (m1_rlast !== 1'b1)) begin
        $display("[%0t] [TB][FAIL] M1 RLAST not asserted on last beat", $time);
        $fatal;
      end

      @(negedge clk);
    end

    @(negedge clk);
    m1_rready = 1'b0;
  endtask


  // ============================================================
  // MONITOR (simulation-only)
  // ============================================================
 // always @(posedge clk) begin
 //   if (rst_n) begin
      // S0
 //     if (s0_awvalid && s0_awready) $display("[%0t] [MON] S0 AW id=0x%0x addr=0x%08x len=%0d", $time, s0_awid, s0_awaddr, s0_awlen);
 //    if (s0_wvalid  && s0_wready ) if (VERBOSE_MON_LOG) if (VERBOSE_MON_LOG) $display("[%0t] [MON] S0 W  data=0x%016x last=%0d", $time, s0_wdata, s0_wlast);
 //     if (s0_bvalid  && s0_bready ) $display("[%0t] [MON] S0 B  bid=0x%0x resp=0x%0x", $time, s0_bid, s0_bresp);
 //   if (s0_arvalid && s0_arready) $display("[%0t] [MON] S0 AR id=0x%0x addr=0x%08x len=%0d", $time, s0_arid, s0_araddr, s0_arlen);
 //    if (s0_rvalid  && s0_rready ) $display("[%0t] [MON] S0 R  rid=0x%0x data=0x%016x last=%0d", $time, s0_rid, s0_rdata, s0_rlast);

      // S1
 //     if (s1_awvalid && s1_awready) $display("[%0t] [MON] S1 AW id=0x%0x addr=0x%08x len=%0d", $time, s1_awid, s1_awaddr, s1_awlen);
 //     if (s1_wvalid  && s1_wready ) if (VERBOSE_MON_LOG) if (VERBOSE_MON_LOG) $display("[%0t] [MON] S1 W  data=0x%016x last=%0d", $time, s1_wdata, s1_wlast);
 //     if (s1_bvalid  && s1_bready ) $display("[%0t] [MON] S1 B  bid=0x%0x resp=0x%0x", $time, s1_bid, s1_bresp);
 //     if (s1_arvalid && s1_arready) $display("[%0t] [MON] S1 AR id=0x%0x addr=0x%08x len=%0d", $time, s1_arid, s1_araddr, s1_arlen);
 //     if (s1_rvalid  && s1_rready ) $display("[%0t] [MON] S1 R  rid=0x%0x data=0x%016x last=%0d", $time, s1_rid, s1_rdata, s1_rlast);

      // S2
 //     if (s2_awvalid && s2_awready) $display("[%0t] [MON] S2 AW id=0x%0x addr=0x%08x len=%0d", $time, s2_awid, s2_awaddr, s2_awlen);
 //     if (s2_wvalid  && s2_wready ) if (VERBOSE_MON_LOG) if (VERBOSE_MON_LOG) $display("[%0t] [MON] S2 W  data=0x%016x last=%0d", $time, s2_wdata, s2_wlast);
 //     if (s2_bvalid  && s2_bready ) $display("[%0t] [MON] S2 B  bid=0x%0x resp=0x%0x", $time, s2_bid, s2_bresp);
 //     if (s2_arvalid && s2_arready) $display("[%0t] [MON] S2 AR id=0x%0x addr=0x%08x len=%0d", $time, s2_arid, s2_araddr, s2_arlen);
 //     if (s2_rvalid  && s2_rready ) $display("[%0t] [MON] S2 R  rid=0x%0x data=0x%016x last=%0d", $time, s2_rid, s2_rdata, s2_rlast);
 //   end
 // end


// fixed_tb: if npu_done signal is missing/glitchy, derive a robust completion
// condition from M0 write-response handshake observed AFTER npu_start is issued.
logic npu_arm;
logic npu_done_seen;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    npu_arm       <= 1'b0;
    npu_done_seen <= 1'b0;
  end else begin
    // On each start pulse, (re)arm and clear previous done
    if (npu_start) begin
      npu_arm       <= 1'b1;
      npu_done_seen <= 1'b0;
    end

    // Prefer real done if it asserts; else fall back to observed M0 B handshake
    if (npu_arm) begin
      if (npu_done === 1'b1) begin
        npu_done_seen <= 1'b1;
        npu_arm       <= 1'b0;
      end else if (m0_bvalid && m0_bready) begin
        npu_done_seen <= 1'b1;
        npu_arm       <= 1'b0;
      end
    end
  end
end

// Use a "done" alias: prefer real npu_done if it asserts, else fall back to observed completion
wire npu_done_effective = (npu_done === 1'b1) ? 1'b1 : npu_done_seen;

  // MRAM power-gating protection (Level-A):
  // When MRAM is OFF and any master tries to access S0 range, we auto-wake MRAM instead of killing the sim.
  // Rationale: after "NPU done" there can still be late/outstanding address phases; a real system would either
  // (a) delay power-off until the bus is idle, or (b) wake MRAM on demand.
  // Here we model (b) to keep long multi-frame runs robust while still letting you account for OFF time.
  always @(posedge clk) begin
    if (rst_n) begin
      // If MRAM is explicitly forced OFF (e.g., VBLANK) but a master touches MRAM range,
      // switch to AUTO so MRAM can wake only for the real S0 transaction.
      if (mram_pwr_mode == MRAM_PWR_OFF) begin
        // Wake as soon as a master *requests* an MRAM transaction.
        // Reason: if the MRAM model backpressures when OFF, waiting for *ready* can create a deadlock
        // and can also cause power accounting to classify the burst as mostly OFF.
        if (m0_arvalid && is_mram_addr(m0_araddr)) begin
          mram_pwr_mode <= MRAM_PWR_AUTO;
          mram_wake_cnt <= mram_wake_cnt + 1;
          $display("[%0t] [TB][WARN] MRAM_OFF but NPU(M0) AR addr=0x%08x => auto-wake MRAM (wake_cnt=%0d)",
                   $time, m0_araddr, mram_wake_cnt + 1);
        end
        if (m0_awvalid && is_mram_addr(m0_awaddr)) begin
          mram_pwr_mode <= MRAM_PWR_AUTO;
          mram_wake_cnt <= mram_wake_cnt + 1;
          $display("[%0t] [TB][WARN] MRAM_OFF but NPU(M0) AW addr=0x%08x => auto-wake MRAM (wake_cnt=%0d)",
                   $time, m0_awaddr, mram_wake_cnt + 1);
        end
        if (m1_arvalid && is_mram_addr(m1_araddr)) begin
          mram_pwr_mode <= MRAM_PWR_AUTO;
          mram_wake_cnt <= mram_wake_cnt + 1;
          $display("[%0t] [TB][WARN] MRAM_OFF but DMA(M1) AR addr=0x%08x => auto-wake MRAM (wake_cnt=%0d)",
                   $time, m1_araddr, mram_wake_cnt + 1);
        end
        if (m1_awvalid && is_mram_addr(m1_awaddr)) begin
          mram_pwr_mode <= MRAM_PWR_AUTO;
          mram_wake_cnt <= mram_wake_cnt + 1;
          $display("[%0t] [TB][WARN] MRAM_OFF but DMA(M1) AW addr=0x%08x => auto-wake MRAM (wake_cnt=%0d)",
                   $time, m1_awaddr, mram_wake_cnt + 1);
        end
      end
    end
  end


int unsigned sb_frame_idx;

// Main test
// -------------------------
initial begin
  // Waveform
  //$dumpfile("soc_2m3s_noc_addrdecode.vcd");
  //$dumpvars(0, tb_soc_top_2m2s_noc_mram);

  // Init
  rst_n = 0;
  npu_start = 0;
  init_master0();
  init_master1();

  // Reset
  repeat (10) @(posedge clk);
  rst_n = 1;
  repeat (5) @(posedge clk);

  // ------------------------------------------------------------
  // MRAM PRELOAD: DMA(M1) writes MobileNetV3-like INT8 weights (scaled) into MRAM (S0)
  // ------------------------------------------------------------
  $display("[%0t] [TB] Preload MRAM weights: bytes_sim=%0d beats_sim=%0d (scale=%0d)",
           $time, weight_bytes_sim, weight_beats_sim, frame_scale);
  // NOTE: NPU trace reads weights from 0x0000_1000; preload must match that address.
  axi_write_large_mram(1, 4'h1, 32'h0000_1000, weight_beats_sim, 4'hA);
  // (Optional quick spot-check) read first 8 beats only, to avoid long sim time.
  axi_read_burst_and_check(1, 4'h1, 32'h0000_1000, 8, 4'hA);
  $display("[%0t] [TB][PASS] MRAM preload spot-check PASSED at addr=0x%08x (8 beats).", $time, 32'h0000_1000);

  // After MRAM preload, switch to AUTO mode:
  // - MRAM will be ON only when S0 actually has in-flight transactions.
  // - So DRAM/SRAM-only phases won't burn MRAM leakage.
  mram_pwr_mode = MRAM_PWR_AUTO;
  @(posedge clk);

  

  // ------------------------------------------------------------
  // DRAM (S1 / axi_ddr4_slave_model): TEST PATTERN
  //   - write 8 beats then read 8 beats and check
  //   - patterns: 0x00..00, 0xFF..FF, 0xA5..A5, 0x5A..5A
  //   - addresses are in S1 range (0x1xxx_xxxx)
  // ------------------------------------------------------------
  begin : dram_test_pattern
    int unsigned p;
    int unsigned dram_addr;

    // ------------------------------------------------------------
    // DRAM PRELOAD: DMA(M1) writes one scaled 4K input frame into DRAM (S1)
    // ------------------------------------------------------------
    dram_addr = 32'h1000_2000;
    $display("[%0t] [TB] Preload DRAM input frame: beats_sim=%0d (scale=%0d) addr=0x%08x",
             $time, frame_beats_sim, frame_scale, dram_addr);
    // Use a single stable pattern (A5) to keep sim time predictable.
    axi_write_large_dram_pattern(1, 4'h2, dram_addr, frame_beats_sim, 2);
    // Spot-check first 8 beats only.
    axi_read_burst_and_check_dram_pattern(1, 4'h2, dram_addr, 8, 2);
    $display("[%0t] [TB][PASS] DRAM preload spot-check PASSED at addr=0x%08x (8 beats).", $time, dram_addr);

  begin : sram_test_pattern
    int unsigned p;
    logic [31:0] sram_addr;
    for (p = 0; p < 4; p++) begin
      sram_addr = 32'h2000_2000 + (p * 32'h0000_0100); // S2 range
      axi_write_burst_sram_pattern(1, 4'h3, sram_addr, 8, p);
      axi_read_burst_and_check_sram_pattern(1, 4'h3, sram_addr, 8, p);
      $display("[%0t] [TB][PASS] SRAM pattern %0d WR8/RD8 compare PASSED at addr=0x%08x.",
               $time, p, sram_addr);
    end
    $display("[%0t] [TB] SRAM test-pattern DONE.", $time);

  // ------------------------------------------------------------
  // 4K streaming traffic + MRAM power gating (Level A: power-model only)
  // ------------------------------------------------------------
  begin : run_4k_frames
    int unsigned f;
    for (f = 0; f < NUM_FRAMES; f++) begin
	      // Frame compute: use AUTO so MRAM leakage is counted only when S0(MRAM) is actually accessed.
	      mram_pwr_mode <= MRAM_PWR_AUTO;
      @(posedge clk);

      $display("[%0t] [TB] Frame %0d START: MRAM power ON", $time, f);

      // Run one NPU frame workload (existing behavior) if enabled by your TB
      // If your TB already runs NPU internally, this call might be a no-op.
      // Otherwise, replace with your start pulse/task.
      // start_npu_frame();

      // Generate 4K traffic to DDR4/SRAM (tile-buffer wrapped)
      run_4k_streaming_traffic(f);


      // PER-FRAME power report pulse
      pwr_report_pulse <= 1'b1;
      @(posedge clk);
      pwr_report_pulse <= 1'b0;
      repeat (2) @(posedge clk);

      
      // Consolidated, unambiguous per-frame summary:
      // - DDR monitor reports its own total energy/power (o_e_total_j / o_p_total_w)
      // - MRAM monitor reports its own total energy/power (o_e_total_j / o_p_total_w)
      // - We print both plus a summed system total.
      pwr_e_ddr_j   = ddr_e_total_j;
      pwr_e_mram_j  = mram_e_total_j;
      pwr_e_total_j = pwr_e_ddr_j + pwr_e_mram_j;

      // Average power uses the same wall time base (both monitors share cycles_total).
      pwr_p_ddr_w   = ddr_p_total_w;
      pwr_p_mram_w  = mram_p_total_w;
      pwr_p_total_w = (ddr_t_s > 0.0) ? (pwr_e_total_j / ddr_t_s) : 0.0;

      $display("[PWR][frame %0d] DDR_total : E=%.6e J  P=%.6e W  t=%.6e s", f, ddr_e_total_j, ddr_p_total_w, ddr_t_s);
      $display("[PWR][frame %0d] MRAM_total: E=%.6e J  P=%.6e W  t=%.6e s", f, mram_e_total_j, mram_p_total_w, mram_t_s);
      $display("[PWR][frame %0d] SYS_total : E=%.6e J  P=%.6e W  t=%.6e s", f, pwr_e_total_j, pwr_p_total_w, ddr_t_s);

      // Optional scaling to 4K-equivalent (tile-buffer traffic scaling)
      $display("[PWR][scaled x%0d][frame %0d] DDR_total : E=%.6e J", frame_scale, f, ddr_e_total_j * frame_scale);
      $display("[PWR][scaled x%0d][frame %0d] MRAM_total: E=%.6e J", frame_scale, f, mram_e_total_j * frame_scale);
      $display("[PWR][scaled x%0d][frame %0d] SYS_total : E=%.6e J", frame_scale, f, pwr_e_total_j  * frame_scale);
      // Report power at end of frame
      pwr_report_pulse <= 1'b1;
      @(posedge clk);
      pwr_report_pulse <= 1'b0;




	      // Enter VBLANK: force MRAM OFF (power model only)
	      mram_pwr_mode <= MRAM_PWR_OFF;
	      $display("[%0t] [TB] Frame %0d VBLANK: MRAM power OFF (power-model)", $time, f);

      repeat (VBLANK_CYC) @(posedge clk);

      $display("[%0t] [TB] Frame %0d END", $time, f);
    end
  end
  end

  end
$display("[%0t] [TB] Memory functional tests DONE. Clearing power counters before NPU frames...", $time);
    pwr_clear_now();
    $display("[%0t] [TB] Starting NPU frames...", $time);
  // Optional: report baseline right before starting NPU frames
  report_power();

  // ------------------------------------------------------------
  // NPU FRAMES + VBLANK MRAM POWER-GATING (Level A)
  // - Each NPU run is treated as one "frame"
  // - After each frame completes, MRAM leakage is gated off for VBLANK_CYC cycles
  // - Next frame starts after MRAM is powered on again
  // ------------------------------------------------------------
	
  begin : npu_frames
    int unsigned f;
    for (f = 0; f < NUM_FRAMES; f++) begin
	      // NPU frame: use AUTO so只有真的打到 S0(MRAM) 時才會開 (可看到 leakage 省下來)
	      mram_pwr_mode = MRAM_PWR_AUTO;
	      $display("[%0t] [TB] Frame %0d: MRAM power AUTO, starting NPU", $time, f);
      sb_frame_idx = f;

      // Kick NPU (1-cycle pulse)
      @(posedge clk);
      npu_start = 1'b1;
      @(posedge clk);
      npu_start = 1'b0;


      // Generate additional "MobileNetV3-Large INT8" weight fetch traffic (via DMA=M1)
      // so we can compare MRAM vs DRAM placement fairly.
      npu_weight_fetch_dma(f);

      // Wait for completion
      wait (npu_done_effective === 1'b1);
      @(posedge clk); // align
      $display("[%0t] [TB] Frame %0d: NPU DONE", $time, f);

      // Report power at end of frame
      report_power();

      // Enter vblank: gate MRAM leakage/static
      $display("[%0t] [TB] Frame %0d: Enter VBLANK (%0d cycles), MRAM power OFF (power-model)", $time, f, VBLANK_CYC);
	      mram_pwr_mode = MRAM_PWR_OFF;
      repeat (VBLANK_CYC) @(posedge clk);
	      $display("[%0t] [TB] Frame %0d: Exit VBLANK, MRAM power AUTO", $time, f);
	      mram_pwr_mode = MRAM_PWR_AUTO;

      // Small gap between frames
      repeat (10) @(posedge clk);
    end
  end

  // Final report
  report_power();

  $display("[%0t] [TB] ALL DONE (memory tests + NPU frames).", $time);
  #100;
  
  // ============================================================
  // Guaranteed power report (even if frame loop doesn't pulse it)
  // ============================================================
    pwr_report_pulse = 1'b0;
    wait (rst_n == 1'b1);
    repeat (50) @(posedge clk);

    $display("[%0t] [TB] FORCE power report now", $time);
    pwr_report_pulse = 1'b1;
    @(posedge clk);
    pwr_report_pulse = 1'b0;

    repeat (5) @(posedge clk);

    // Final summary using last-reported outputs (monitor already printed detailed breakdown)
    $display("[PWR][final] DDR_total : E=%.6e J  P=%.6e W  t=%.6e s", ddr_e_total_j, ddr_p_total_w, ddr_t_s);
    $display("[PWR][final] MRAM_total: E=%.6e J  P=%.6e W  t=%.6e s", mram_e_total_j, mram_p_total_w, mram_t_s);
    $display("[PWR][final] SYS_total : E=%.6e J  P=%.6e W  t=%.6e s", (ddr_e_total_j+mram_e_total_j), (ddr_p_total_w+mram_p_total_w), ddr_t_s);
    $display("[PWR][scaled x%0d][final] SYS_total : E=%.6e J", frame_scale, (ddr_e_total_j+mram_e_total_j)*frame_scale);

$finish;
end

// -------------------------
// Global watchdog (prevents infinite run if design stalls)
// -------------------------
initial begin : global_watchdog
  int unsigned wd;
  wd = 0;
  while (wd < 500000000) begin  // 500M cycles @200MHz sim time
    @(posedge clk);
    wd++;
  end
  $display("[%0t] [TB][TIMEOUT] Global watchdog hit -> $finish", $time);
  $finish;
end


endmodule