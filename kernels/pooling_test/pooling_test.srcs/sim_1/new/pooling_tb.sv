`timescale 1ns / 1ps
// ============================================================================
// PoolingKernel HLS Testbench (mirrors conv_tb.sv structure)
//
// Components:
//   pool_item  - transaction object (pooling geometry, fill value, expected output)
//   driver     - loads DDR arrays, programs AXI-Lite registers, asserts ap_start
//   monitor    - waits for interrupt, services it
//   scoreboard - backdoor-reads y and verifies every element
//   env        - connects all components; runs one transaction
//   test       - builds test matrix, iterates, prints summary
//
// DUT: design_pooling block design (no external ports).
//   Zynq UltraScale+ PS VIP: AXI-Lite master HPM0_FPD + DDR slave HPC0_FPD
//   Kernel: PoolingKernel (ap_fixed<16,8>; gmem0/x read, gmem1/y write —
//           both 128-bit hls::burst_maxi ports, y with byte-strobed run
//           edges; the scoreboard also checks the poisoned y tail lanes)
//
// ap_fixed<16,8> encoding:
//   real_value × 256 = raw int16
//   1.0=0x0100  2.0=0x0200  4.0=0x0400  0.5=0x0080  16.0=0x1000
//  -1.0=0xFF00  100.0=0x6400  -100.0=0x9C00
//   max=0x7FFF (+127.996)  min=0x8000 (-128.0)
//
// pool_type codes:
//   0 = kPoolMax   — maximum over pool window
//   1 = kPoolAvg   — average over pool window (uses float reciprocal)
//   2 = kPoolLp    — Lp-norm (p=lp_order=1: Σ|x|; p=2: sqrt(Σx²))
//
// Test fixtures live in the directory passed via +DATA_DIR=<dir>:
//   manifest.txt        — one row per test (18 ints + label, see test::run)
//   test_<NN>_x.hex     — input x array as 16-bit raw values, one per line
//   test_<NN>_y.hex     — reference y array as 16-bit raw values, one per line
// The testbench compares the kernel's y output element-wise against y_ref,
// so test cases with padding/dilation/etc. are verified position-by-position
// rather than against a single closed-form expected value.
//
// Kernel AXI-Lite register layout (base 0xA000_0000).
//   Source: pooling_test.gen/.../PoolingKernel_v1_0/src/xpoolingkernel_hw.h
//
//   0x00 ap_ctrl         0x04 gie        0x08 ier        0x0C isr
//   0x10 x_lo            0x14 x_hi       (0x18 reserved)
//   0x1C y_lo            0x20 y_hi       (0x24 reserved)
//   0x28 batch           (0x2C reserved)
//   0x30 channels        (0x34 reserved)
//   0x38 in_h            (0x3C reserved)
//   0x40 in_w            (0x44 reserved)
//   0x48 out_h           (0x4C reserved)
//   0x50 out_w           (0x54 reserved)
//   0x58 pool_h          (0x5C reserved)
//   0x60 pool_w          (0x64 reserved)
//   0x68 stride_h        (0x6C reserved)
//   0x70 stride_w        (0x74 reserved)
//   0x78 pad_top         (0x7C reserved)
//   0x80 pad_left        (0x84 reserved)
//   0x88 dil_h           (0x8C reserved)
//   0x90 dil_w           (0x94 reserved)
//   0x98 pool_type       (0x9C reserved)
//   0xA0 lp_order        (0xA4 reserved)
//   0xA8 count_include_pad (0xAC reserved)
//
// Test matrix: every case of TestPoolingSim.cpp's table, dumped by
// `make gen_pool_test_data` (43 cases as of POOL §2.14: the 31 original
// geometries plus 12 word-tail / alignment cases).
// ============================================================================

module pooling_tb;
    // -----------------------------------------------------------------------
    // DUT instantiation.
    // -----------------------------------------------------------------------
    design_pooling dut();

    // PS VIP access path.
    `define PS dut.zynq_ultra_ps_e_0.inst

    // -----------------------------------------------------------------------
    // Kernel AXI-Lite register map  (base 0xA000_0000)
    // -----------------------------------------------------------------------
    localparam [39:0] CTRL_BASE              = 40'hA000_0000;
    localparam [39:0] REG_AP_CTRL            = CTRL_BASE + 40'h00;
    localparam [39:0] REG_GIE                = CTRL_BASE + 40'h04;
    localparam [39:0] REG_IER                = CTRL_BASE + 40'h08;
    localparam [39:0] REG_ISR                = CTRL_BASE + 40'h0C;
    localparam [39:0] REG_X_LO              = CTRL_BASE + 40'h10;
    localparam [39:0] REG_X_HI              = CTRL_BASE + 40'h14;
    localparam [39:0] REG_Y_LO              = CTRL_BASE + 40'h1C;
    localparam [39:0] REG_Y_HI              = CTRL_BASE + 40'h20;
    localparam [39:0] REG_BATCH             = CTRL_BASE + 40'h28;
    localparam [39:0] REG_CHANNELS          = CTRL_BASE + 40'h30;
    localparam [39:0] REG_IN_H              = CTRL_BASE + 40'h38;
    localparam [39:0] REG_IN_W              = CTRL_BASE + 40'h40;
    localparam [39:0] REG_OUT_H             = CTRL_BASE + 40'h48;
    localparam [39:0] REG_OUT_W             = CTRL_BASE + 40'h50;
    localparam [39:0] REG_POOL_H            = CTRL_BASE + 40'h58;
    localparam [39:0] REG_POOL_W            = CTRL_BASE + 40'h60;
    localparam [39:0] REG_STRIDE_H          = CTRL_BASE + 40'h68;
    localparam [39:0] REG_STRIDE_W          = CTRL_BASE + 40'h70;
    localparam [39:0] REG_PAD_TOP           = CTRL_BASE + 40'h78;
    localparam [39:0] REG_PAD_LEFT          = CTRL_BASE + 40'h80;
    localparam [39:0] REG_DIL_H             = CTRL_BASE + 40'h88;
    localparam [39:0] REG_DIL_W             = CTRL_BASE + 40'h90;
    localparam [39:0] REG_POOL_TYPE         = CTRL_BASE + 40'h98;
    localparam [39:0] REG_LP_ORDER          = CTRL_BASE + 40'hA0;
    localparam [39:0] REG_COUNT_INCLUDE_PAD = CTRL_BASE + 40'hA8;

    // -----------------------------------------------------------------------
    // Testbench parameters
    // -----------------------------------------------------------------------
    localparam int unsigned ELEM_BYTES = 2;    // sizeof(ap_fixed<16,8>)
    localparam integer      CHUNK_SIZE = 1024; // PS VIP transfer chunk (bytes)
    localparam integer      CHUNK_BITS = CHUNK_SIZE * 8;
    localparam int unsigned MEM_GAP    = 64 * 1024; // guard gap between arrays (bytes)

    localparam logic [15:0] Y_POISON = 16'hDEAD; // sentinel for unwritten y elements

    // Shadow of every byte the bench or the kernel has written (byte address
    // -> value).  The PS VIP's DDRC write path races (see ddrc_wr_fix below)
    // and can deposit STALE data in the bytes a partial-strobe beat leaves
    // unstrobed (observed with the 128-bit conv y port, CONV §2.38).  The
    // pool y port is 128-bit with byte-strobed run edges too (POOL §2.14);
    // ddrc_wr_fix restores unstrobed bytes from this shadow, so the y tail
    // pad check in the scoreboard is a genuine test of the kernel's WSTRB.
    logic [7:0] shadow_mem [longint unsigned];

    function automatic void shadow_store(input [39:0] base, input int unsigned nbytes,
                                         input logic [CHUNK_BITS-1:0] buf_mem);
        for (int unsigned b = 0; b < nbytes; b++)
            shadow_mem[longint'(base) + b] = buf_mem[b*8 +: 8];
    endfunction

    // Kernel interrupt wire.
    wire kernel_irq = dut.PoolingKernel_0_interrupt;

    // -----------------------------------------------------------------------
    // Helper: round n up to the next multiple of align.
    // -----------------------------------------------------------------------
    function automatic int unsigned align_up(int unsigned n, int unsigned align);
        return (n + align - 1) & ~(align - 1);
    endfunction

    // =========================================================================
    // Transaction - one complete PoolingKernel call
    // =========================================================================
    class pool_item;
        // Manifest-supplied identity (used for log lines + JSON report)
        int unsigned index;
        string       label;

        // Geometry — every dimension comes from the manifest, not recomputed,
        // so the upstream pipeline that generated the manifest stays the
        // single source of truth (matches conv_tb's conv_item).
        int unsigned batch;
        int unsigned channels;
        int unsigned in_h,  in_w;
        int unsigned out_h, out_w;
        int unsigned pool_h, pool_w;
        int unsigned stride_h, stride_w;
        int unsigned pad_top, pad_left;
        int unsigned dil_h, dil_w;
        int unsigned pool_type;        // 0=Max  1=Avg  2=Lp
        int unsigned lp_order;         // 1 or 2  (LpPool only)
        int unsigned count_include_pad;

        // Element counts (derived).
        int unsigned x_count;
        int unsigned y_count;

        // DDR base addresses
        logic [39:0] addr_x;
        logic [39:0] addr_y;

        // Fixture data (loaded from <data_dir>/test_<NN>_{x,y}.hex).
        logic [15:0] x_data[];
        logic [15:0] y_ref [];

        function new(
            int unsigned   index_,
            string         lbl,
            int unsigned   batch_,
            int unsigned   channels_,
            int unsigned   in_h_,         int unsigned in_w_,
            int unsigned   out_h_,        int unsigned out_w_,
            int unsigned   pool_h_,       int unsigned pool_w_,
            int unsigned   stride_h_,     int unsigned stride_w_,
            int unsigned   pad_top_,      int unsigned pad_left_,
            int unsigned   dil_h_,        int unsigned dil_w_,
            int unsigned   pool_type_,
            int unsigned   lp_order_,
            int unsigned   count_include_pad_);

            int unsigned x_bytes;

            this.index             = index_;
            this.label             = lbl;
            this.batch             = batch_;
            this.channels          = channels_;
            this.in_h              = in_h_;
            this.in_w              = in_w_;
            this.out_h             = out_h_;
            this.out_w             = out_w_;
            this.pool_h            = pool_h_;
            this.pool_w            = pool_w_;
            this.stride_h          = stride_h_;
            this.stride_w          = stride_w_;
            this.pad_top           = pad_top_;
            this.pad_left          = pad_left_;
            this.dil_h             = dil_h_;
            this.dil_w             = dil_w_;
            this.pool_type         = pool_type_;
            this.lp_order          = lp_order_;
            this.count_include_pad = count_include_pad_;

            this.x_count = batch_ * channels_ * in_h_  * in_w_;
            this.y_count = batch_ * channels_ * out_h_ * out_w_;

            // DDR layout: x | gap | y
            x_bytes = align_up(this.x_count * ELEM_BYTES, 16);
            this.addr_x = 40'h1000_0000;
            this.addr_y = this.addr_x + 40'(x_bytes) + 40'(MEM_GAP);
        endfunction

        // Load x / y_ref from <dir>/test_<NN>_{x,y}.hex.  Each file has one
        // 16-bit raw value per line — the upstream pooling reference dump
        // produces them, exactly like the conv test stand.
        function void load_fixture(string dir);
            string idx_str;
            x_data = new[x_count];
            y_ref  = new[y_count];
            idx_str = $sformatf("%02d", index);
            $readmemh($sformatf("%s/test_%s_x.hex", dir, idx_str), x_data);
            $readmemh($sformatf("%s/test_%s_y.hex", dir, idx_str), y_ref);
        endfunction

        function string to_string();
            return $sformatf(
                "%-36s  N=%0d C=%0d IH=%0d IW=%0d  OH=%0d OW=%0d  k=%0dx%0d s=%0dx%0d d=%0dx%0d p=%0d,%0d  type=%0d lp=%0d cip=%0d  |x|=%0d |y|=%0d",
                label,
                batch, channels, in_h, in_w, out_h, out_w,
                pool_h, pool_w, stride_h, stride_w, dil_h, dil_w,
                pad_top, pad_left, pool_type, lp_order, count_include_pad,
                x_count, y_count);
        endfunction
    endclass

    // =========================================================================
    // Per-test result record — captured by scoreboard, dumped to JSON at end.
    // Mirrors the layout in conv_tb.sv so the test stand's run_tb.sh wrapper
    // can parse either kernel's report with the same logic.
    // =========================================================================
    class test_result;
        // Identity
        int unsigned index;
        string       label;
        int unsigned errors;
        int unsigned total_elements;

        // Geometry mirror (kept here so the JSON report is self-contained)
        int unsigned batch, channels;
        int unsigned in_h, in_w, out_h, out_w;
        int unsigned pool_h, pool_w;
        int unsigned stride_h, stride_w;
        int unsigned pad_top, pad_left;
        int unsigned dil_h, dil_w;
        int unsigned pool_type, lp_order, count_include_pad;

        // Per-test simulation timing — stamped by env::run_one.  Units are
        // nanoseconds (file-level `timescale 1ns/1ps).  start_ns is sampled
        // before the driver programs registers; end_ns after the scoreboard
        // finishes verifying.
        longint unsigned start_ns;
        longint unsigned end_ns;
        longint unsigned duration_ns;

        // First N mismatches (capped to MAX_MM)
        int unsigned mm_idx[$];
        logic [15:0] mm_got[$];
        logic [15:0] mm_exp[$];

        function string status();
            return (errors == 0) ? "PASS" : "FAIL";
        endfunction
    endclass

    // Cap for per-test mismatch records emitted to JSON.  Matches conv_tb.
    localparam int unsigned MAX_MM = 16;

    // =========================================================================
    // Driver - fills DDR arrays, programs kernel registers, asserts ap_start
    // =========================================================================
    class driver;

        local task axil_write(input [39:0] addr, input [31:0] data);
            logic [1:0] rsp;
            `PS.write_data(addr, 4, {{(2048-32){1'b0}}, data}, rsp);
            if (rsp !== 2'b00)
                $error("[%0t][DRV] AXI-Lite write FAILED  addr=0x%010h  rsp=%0b",
                       $time, addr, rsp);
        endtask

        // Fill nbytes bytes starting at base with a constant 16-bit pattern
        // (used to poison y[] before each run).
        local task fill_const_ddr(input [39:0]      base,
                                  input int unsigned nbytes,
                                  input [15:0]       val);
            logic [CHUNK_BITS-1:0] buf_mem;
            int unsigned           i, n_chunks, rem;
            for (i = 0; i < CHUNK_SIZE / 2; i++)
                buf_mem[i*16 +: 16] = val;
            n_chunks = nbytes / CHUNK_SIZE;
            rem      = nbytes % CHUNK_SIZE;
            for (i = 0; i < n_chunks; i++) begin
                `PS.write_mem(buf_mem, base + 40'(i * CHUNK_SIZE), CHUNK_SIZE);
                shadow_store(base + 40'(i * CHUNK_SIZE), CHUNK_SIZE, buf_mem);
            end
            if (rem > 0) begin
                `PS.write_mem(buf_mem, base + 40'(n_chunks * CHUNK_SIZE), rem);
                shadow_store(base + 40'(n_chunks * CHUNK_SIZE), rem, buf_mem);
            end
        endtask

        // Write a 16-bit data array to DDR starting at base, in CHUNK_SIZE
        // bursts.  Tail beyond n_elements is zero-padded so the AXI write
        // is still 16-byte aligned, and the kernel never reads that tail.
        local task write_data_ddr(input [39:0] base,
                                  ref logic [15:0] data[],
                                  input int unsigned n_elements);
            logic [CHUNK_BITS-1:0] buf_mem;
            int unsigned items_per_chunk, n_bytes, aligned_bytes;
            int unsigned n_chunks, rem, c, i, idx;
            items_per_chunk = CHUNK_SIZE / 2;
            n_bytes         = n_elements * 2;
            aligned_bytes   = align_up(n_bytes, 16);
            n_chunks        = aligned_bytes / CHUNK_SIZE;
            rem             = aligned_bytes % CHUNK_SIZE;
            for (c = 0; c < n_chunks; c++) begin
                buf_mem = '0;
                for (i = 0; i < items_per_chunk; i++) begin
                    idx = c * items_per_chunk + i;
                    if (idx < n_elements)
                        buf_mem[i*16 +: 16] = data[idx];
                end
                `PS.write_mem(buf_mem, base + 40'(c * CHUNK_SIZE), CHUNK_SIZE);
                shadow_store(base + 40'(c * CHUNK_SIZE), CHUNK_SIZE, buf_mem);
            end
            if (rem > 0) begin
                buf_mem = '0;
                for (i = 0; i < rem / 2; i++) begin
                    idx = n_chunks * items_per_chunk + i;
                    if (idx < n_elements)
                        buf_mem[i*16 +: 16] = data[idx];
                end
                `PS.write_mem(buf_mem, base + 40'(n_chunks * CHUNK_SIZE), rem);
                shadow_store(base + 40'(n_chunks * CHUNK_SIZE), rem, buf_mem);
            end
        endtask

        task run(pool_item item);
            int unsigned y_bytes;

            $display("[%0t][DRV] %s", $time, item.to_string());

            y_bytes = item.y_count * ELEM_BYTES;

            $display("[%0t][DRV] Loading x   (%0d elements) ...",
                     $time, item.x_count);
            write_data_ddr(item.addr_x, item.x_data, item.x_count);

            // Poison y so any missed element is detected by the scoreboard.
            $display("[%0t][DRV] Pre-filling y (%0d B) with 0x%04h ...",
                     $time, align_up(y_bytes, 16), Y_POISON);
            fill_const_ddr(item.addr_y, align_up(y_bytes, 16), Y_POISON);

            // Program kernel AXI-Lite registers.
            $display("[%0t][DRV] Programming registers ...", $time);
            axil_write(REG_X_LO,              item.addr_x[31:0]);
            axil_write(REG_X_HI,              {24'b0, item.addr_x[39:32]});
            axil_write(REG_Y_LO,              item.addr_y[31:0]);
            axil_write(REG_Y_HI,              {24'b0, item.addr_y[39:32]});
            axil_write(REG_BATCH,             32'(item.batch));
            axil_write(REG_CHANNELS,          32'(item.channels));
            axil_write(REG_IN_H,              32'(item.in_h));
            axil_write(REG_IN_W,              32'(item.in_w));
            axil_write(REG_OUT_H,             32'(item.out_h));
            axil_write(REG_OUT_W,             32'(item.out_w));
            axil_write(REG_POOL_H,            32'(item.pool_h));
            axil_write(REG_POOL_W,            32'(item.pool_w));
            axil_write(REG_STRIDE_H,          32'(item.stride_h));
            axil_write(REG_STRIDE_W,          32'(item.stride_w));
            axil_write(REG_PAD_TOP,           32'(item.pad_top));
            axil_write(REG_PAD_LEFT,          32'(item.pad_left));
            axil_write(REG_DIL_H,             32'(item.dil_h));
            axil_write(REG_DIL_W,             32'(item.dil_w));
            axil_write(REG_POOL_TYPE,         32'(item.pool_type));
            axil_write(REG_LP_ORDER,          32'(item.lp_order));
            axil_write(REG_COUNT_INCLUDE_PAD, 32'(item.count_include_pad));

            // Enable ap_done interrupt and assert ap_start.
            axil_write(REG_GIE,     32'h1);
            axil_write(REG_IER,     32'h1);
            $display("[%0t][DRV] Asserting ap_start ...", $time);
            axil_write(REG_AP_CTRL, 32'h1);
        endtask
    endclass

    // =========================================================================
    // Monitor - waits for kernel interrupt, services it
    // =========================================================================
    class monitor;

        local task axil_read(input [39:0] addr, output [31:0] data);
            logic [127:0] rd_raw;
            logic [1:0]   rsp;
            `PS.read_data(addr, 4, rd_raw, rsp);
            data = rd_raw[31:0];
            if (rsp !== 2'b00)
                $error("[%0t][MON] AXI-Lite read FAILED  addr=0x%010h  rsp=%0b",
                       $time, addr, rsp);
        endtask

        local task axil_write(input [39:0] addr, input [31:0] data);
            logic [1:0] rsp;
            `PS.write_data(addr, 4, {{(2048-32){1'b0}}, data}, rsp);
            if (rsp !== 2'b00)
                $error("[%0t][MON] AXI-Lite write FAILED  addr=0x%010h  rsp=%0b",
                       $time, addr, rsp);
        endtask

        task run(pool_item item);
            logic [15:0] irq_status;
            logic [31:0] isr_val, ap_ctrl_val;

            $display("[%0t][MON] Waiting for interrupt ...", $time);
            irq_status = 16'h0;
            fork
                begin : irq_wait
                    `PS.wait_interrupt(4'd0, irq_status);
                end
                begin : irq_timeout
                    // Allow up to 2 s sim-time per test.
                    #2_000_000_000;
                end
            join_any
            disable fork;

            if (!irq_status[0]) begin
                $error("[%0t][MON] TIMEOUT: no interrupt after 2 s sim-time  test=%s",
                       $time, item.label);
                $finish;
            end
            $display("[%0t][MON] Interrupt received (irq_status=0x%04h)", $time, irq_status);

            // Confirm ap_done
            axil_read(REG_AP_CTRL, ap_ctrl_val);
            $display("[%0t][MON] ap_ctrl=0x%08h  done=%0b  idle=%0b  ready=%0b",
                     $time, ap_ctrl_val, ap_ctrl_val[1], ap_ctrl_val[2], ap_ctrl_val[3]);

            // Service interrupt: read ISR, toggle-on-write clear, disable GIE
            axil_read(REG_ISR, isr_val);
            $display("[%0t][MON] ISR=0x%08h  ap_done=%0b  ap_ready=%0b",
                     $time, isr_val, isr_val[0], isr_val[1]);
            axil_write(REG_ISR, isr_val);   // TOW: write set bits to clear
            axil_write(REG_GIE, 32'h0);

            // Wait for interrupt line to physically deassert before returning so
            // the next test's wait_interrupt does not fire on the stale level.
            if (kernel_irq) begin
                $display("[%0t][MON] Waiting for interrupt line to deassert ...", $time);
                @(negedge kernel_irq);
            end
            $display("[%0t][MON] Interrupt line low - ready for next test.", $time);
        endtask
    endclass

    // =========================================================================
    // Scoreboard - backdoor-reads y and checks every element
    // =========================================================================
    class scoreboard;
        int unsigned total_tests = 0;
        int unsigned pass_cnt    = 0;
        int unsigned fail_cnt    = 0;
        test_result  results[$];

        task run(pool_item item, longint unsigned t_start_ns = 0);
            logic [CHUNK_BITS-1:0] chunk_buf;
            int unsigned n_bytes, n_chunks, rem, errors, eidx, w, total_elems;
            logic [15:0] elem, exp;
            test_result  tr;

            total_elems = item.y_count;
            n_bytes     = total_elems * ELEM_BYTES;
            n_chunks    = n_bytes / CHUNK_SIZE;
            rem         = n_bytes % CHUNK_SIZE;
            errors      = 0;

            tr                     = new();
            tr.index               = item.index;
            tr.label               = item.label;
            tr.batch               = item.batch;
            tr.channels            = item.channels;
            tr.in_h                = item.in_h;
            tr.in_w                = item.in_w;
            tr.out_h               = item.out_h;
            tr.out_w               = item.out_w;
            tr.pool_h              = item.pool_h;
            tr.pool_w              = item.pool_w;
            tr.stride_h            = item.stride_h;
            tr.stride_w            = item.stride_w;
            tr.pad_top             = item.pad_top;
            tr.pad_left            = item.pad_left;
            tr.dil_h               = item.dil_h;
            tr.dil_w               = item.dil_w;
            tr.pool_type           = item.pool_type;
            tr.lp_order            = item.lp_order;
            tr.count_include_pad   = item.count_include_pad;
            tr.total_elements      = total_elems;

            $display("[%0t][SCB] Verifying y[0..%0d] (%0d elem × %0d B = %0d B) against y_ref ...",
                     $time, total_elems - 1, total_elems,
                     ELEM_BYTES, n_bytes);

            for (int i = 0; i < int'(n_chunks); i++) begin
                `PS.read_mem(item.addr_y + 40'(i * CHUNK_SIZE), CHUNK_SIZE, chunk_buf);
                for (w = 0; w < CHUNK_SIZE / 2; w++) begin
                    eidx = i * (CHUNK_SIZE / 2) + w;
                    if (eidx >= total_elems) break;
                    elem = chunk_buf[w*16 +: 16];
                    exp  = item.y_ref[eidx];
                    if (elem !== exp) begin
                        if (errors < 5)
                            $display("[%0t][SCB] MISMATCH y[%0d]: got=0x%04h  exp=0x%04h",
                                     $time, eidx, elem, exp);
                        if (errors < MAX_MM) begin
                            tr.mm_idx.push_back(eidx);
                            tr.mm_got.push_back(elem);
                            tr.mm_exp.push_back(exp);
                        end
                        errors++;
                    end
                end
            end
            if (rem > 0) begin
                `PS.read_mem(item.addr_y + 40'(n_chunks * CHUNK_SIZE), rem, chunk_buf);
                for (w = 0; w < rem / 2; w++) begin
                    eidx = n_chunks * (CHUNK_SIZE / 2) + w;
                    if (eidx >= total_elems) break;
                    elem = chunk_buf[w*16 +: 16];
                    exp  = item.y_ref[eidx];
                    if (elem !== exp) begin
                        if (errors < 5)
                            $display("[%0t][SCB] MISMATCH y[%0d]: got=0x%04h  exp=0x%04h",
                                     $time, eidx, elem, exp);
                        if (errors < MAX_MM) begin
                            tr.mm_idx.push_back(eidx);
                            tr.mm_got.push_back(elem);
                            tr.mm_exp.push_back(exp);
                        end
                        errors++;
                    end
                end
            end

            // POOL §2.14: y is written through a 128-bit port with byte
            // strobes at every run's first / last word.  The lanes between
            // y_count and the 16-byte-aligned end of the y region were
            // poisoned before the run and are never part of the tensor; if
            // the kernel's tail strobe were wrong they would be overwritten.
            begin
                int unsigned pad_base, pad_len;
                logic [CHUNK_BITS-1:0] pad_buf;
                // read_mem must start on a 16-byte beat (an 8-byte-aligned
                // start silently returned the beat below): read the whole
                // last beat and skip its in-tensor elements.
                pad_base = n_bytes & ~32'hF;
                pad_len  = align_up(n_bytes, 16) - pad_base;
                if (pad_len > 0) begin
                    `PS.read_mem(item.addr_y + 40'(pad_base), pad_len, pad_buf);
                    for (w = 0; w < pad_len / 2; w++) begin
                        eidx = pad_base / 2 + w;
                        if (eidx < total_elems) continue;
                        elem = pad_buf[w*16 +: 16];
                        if (elem !== Y_POISON) begin
                            $display("[%0t][SCB] TAIL PAD CLOBBERED y[%0d] (past y_count): got=0x%04h  exp=0x%04h",
                                     $time, eidx, elem, Y_POISON);
                            if (errors < MAX_MM) begin
                                tr.mm_idx.push_back(eidx);
                                tr.mm_got.push_back(elem);
                                tr.mm_exp.push_back(Y_POISON);
                            end
                            errors++;
                        end
                    end
                end
            end

            tr.errors      = errors;
            tr.start_ns    = t_start_ns;
            tr.end_ns      = $time;
            tr.duration_ns = (tr.end_ns >= tr.start_ns)
                             ? (tr.end_ns - tr.start_ns)
                             : 64'd0;
            results.push_back(tr);
            total_tests++;
            if (errors == 0) begin
                pass_cnt++;
                $display("[%0t][SCB] PASS  %-36s  N=%0d C=%0d OH=%0d OW=%0d",
                         $time, item.label,
                         item.batch, item.channels, item.out_h, item.out_w);
            end else begin
                fail_cnt++;
                $display("[%0t][SCB] FAIL  %-36s  %0d/%0d mismatches",
                         $time, item.label, errors, total_elems);
            end
        endtask

        task print_summary();
            $display("==========================================================");
            $display(" PoolingKernel Test Summary: %0d / %0d passed",
                     pass_cnt, total_tests);
            if (fail_cnt == 0)
                $display(" ALL TESTS PASSED");
            else
                $display(" %0d TEST(S) FAILED", fail_cnt);
            $display("==========================================================");
        endtask

        // Emit a JSON report describing every test, its parameters, and the
        // first MAX_MM mismatches for any failing test.  Shape matches the
        // conv testbench so run_tb.sh's parser handles either kernel.
        task write_json_report(string path);
            int          fd;
            test_result  tr;
            fd = $fopen(path, "w");
            if (fd == 0) begin
                $error("[SCB] Failed to open JSON report file: %s", path);
                return;
            end

            $fdisplay(fd, "{");
            $fdisplay(fd, "  \"kernel\": \"PoolingKernel\",");
            $fdisplay(fd, "  \"data_type\": \"ap_fixed<16,8>\",");
            $fdisplay(fd, "  \"sim_time_ns\": %0d,", $time);
            $fdisplay(fd, "  \"summary\": {");
            $fdisplay(fd, "    \"total\":      %0d,", total_tests);
            $fdisplay(fd, "    \"passed\":     %0d,", pass_cnt);
            $fdisplay(fd, "    \"failed\":     %0d,", fail_cnt);
            $fdisplay(fd, "    \"all_passed\": %s",   (fail_cnt == 0) ? "true" : "false");
            $fdisplay(fd, "  },");
            $fdisplay(fd, "  \"tests\": [");

            foreach (results[i]) begin
                tr = results[i];
                $fwrite(fd, "    {\n");
                $fwrite(fd, "      \"index\":  %0d,\n",     tr.index);
                $fwrite(fd, "      \"label\":  \"%s\",\n",  tr.label);
                $fwrite(fd, "      \"status\": \"%s\",\n",  tr.status());
                $fwrite(fd, "      \"geometry\": {\n");
                $fwrite(fd, "        \"batch\": %0d, \"channels\": %0d, \"in_h\": %0d, \"in_w\": %0d,\n",
                            tr.batch, tr.channels, tr.in_h, tr.in_w);
                $fwrite(fd, "        \"out_h\": %0d, \"out_w\": %0d,\n",
                            tr.out_h, tr.out_w);
                $fwrite(fd, "        \"pool_h\": %0d, \"pool_w\": %0d, \"stride_h\": %0d, \"stride_w\": %0d,\n",
                            tr.pool_h, tr.pool_w, tr.stride_h, tr.stride_w);
                $fwrite(fd, "        \"pad_top\": %0d, \"pad_left\": %0d,\n",
                            tr.pad_top, tr.pad_left);
                $fwrite(fd, "        \"dil_h\": %0d, \"dil_w\": %0d,\n",
                            tr.dil_h, tr.dil_w);
                $fwrite(fd, "        \"pool_type\": %0d, \"lp_order\": %0d, \"count_include_pad\": %0d\n",
                            tr.pool_type, tr.lp_order, tr.count_include_pad);
                $fwrite(fd, "      },\n");
                $fwrite(fd, "      \"total_elements\":     %0d,\n",        tr.total_elements);
                $fwrite(fd, "      \"errors\":             %0d,\n",        tr.errors);
                $fwrite(fd, "      \"start_ns\":           %0d,\n",        tr.start_ns);
                $fwrite(fd, "      \"end_ns\":             %0d,\n",        tr.end_ns);
                $fwrite(fd, "      \"duration_ns\":        %0d,\n",        tr.duration_ns);
                $fwrite(fd, "      \"mismatches_reported\": %0d,\n",       tr.mm_idx.size());
                if (tr.mm_idx.size() == 0) begin
                    $fwrite(fd, "      \"mismatches\": []\n");
                end else begin
                    $fwrite(fd, "      \"mismatches\": [\n");
                    foreach (tr.mm_idx[j]) begin
                        $fwrite(fd, "        { \"index\": %0d, \"got\": \"0x%04h\", \"expected\": \"0x%04h\" }%s\n",
                                    tr.mm_idx[j], tr.mm_got[j], tr.mm_exp[j],
                                    (j == tr.mm_idx.size() - 1) ? "" : ",");
                    end
                    $fwrite(fd, "      ]\n");
                end
                $fwrite(fd, "    }%s\n", (i == results.size() - 1) ? "" : ",");
            end

            $fdisplay(fd, "  ]");
            $fdisplay(fd, "}");
            $fclose(fd);
            $display("[SCB] JSON report written: %s", path);
        endtask
    endclass

    // =========================================================================
    // Environment - connects driver, monitor, scoreboard; runs one transaction
    // =========================================================================
    class env;
        driver     drv;
        monitor    mon;
        scoreboard scb;

        function new();
            drv = new();
            mon = new();
            scb = new();
        endfunction

        task run_one(pool_item item);
            longint unsigned t_start_ns;

            t_start_ns = $time;
            drv.run(item);
            mon.run(item);

            // Diagnostic DDR probe: sample first and last y element directly
            // from the DDRC memory model before the scoreboard reads.
            begin : ddr_probe
                logic [31:0] w_first, w_last;
                int unsigned widx_first, widx_last, total_elems;
                logic [15:0] exp_first, exp_last;
                total_elems = item.y_count;
                widx_first  = item.addr_y[31:2];
                widx_last   = item.addr_y[31:2] +
                              ((total_elems - 1) * ELEM_BYTES) / 4;
                w_first = dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem0[widx_first];
                w_last  = dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem0[widx_last];
                exp_first = item.y_ref[0];
                exp_last  = item.y_ref[total_elems - 1];
                $display("[%0t][PROBE] y[0]   word ddr_mem0[0x%08h]=0x%08h  (exp lo-halfword=0x%04h)",
                         $time, widx_first, w_first, exp_first);
                $display("[%0t][PROBE] y[%0d] word ddr_mem0[0x%08h]=0x%08h  (exp=0x????%04h)",
                         $time, total_elems-1, widx_last, w_last, exp_last);
            end

            scb.run(item, t_start_ns);
        endtask
    endclass

    // =========================================================================
    // Test - defines test matrix, runs all cases, prints summary
    // =========================================================================
    class test;
        env       e;
        pool_item tests[$];

        // Parse one manifest line — 18 ints + 1 trailing label token —
        // and create + load a pool_item.  Returns null if the line is
        // blank, a comment, or unparseable.  Manifest column layout
        // (matches the upstream pooling reference dump):
        //
        //   idx batch channels in_h in_w out_h out_w pool_h pool_w
        //   stride_h stride_w pad_top pad_left dil_h dil_w
        //   pool_type lp_order count_include_pad   label
        //
        // Per-test x and y_ref are loaded from <data_dir>/test_<NN>_x.hex
        // and <data_dir>/test_<NN>_y.hex via $readmemh.
        function automatic pool_item parse_manifest_line(string line,
                                                          string data_dir);
            int unsigned idx;
            int unsigned batch, channels, in_h, in_w;
            int unsigned out_h, out_w;
            int unsigned pool_h, pool_w;
            int unsigned stride_h, stride_w;
            int unsigned pad_top, pad_left;
            int unsigned dil_h, dil_w;
            int unsigned pool_type, lp_order, count_include_pad;
            string       label;
            int          rc;
            int          first;
            pool_item    it;

            // Skip leading whitespace; ignore blank or comment lines.
            first = 0;
            while (first < line.len() &&
                   (line.getc(first) == " "  || line.getc(first) == "\t" ||
                    line.getc(first) == "\n" || line.getc(first) == "\r"))
                first++;
            if (first == line.len()) return null;
            if (line.getc(first) == "#") return null;

            rc = $sscanf(line,
                "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %s",
                idx, batch, channels, in_h, in_w,
                out_h, out_w, pool_h, pool_w,
                stride_h, stride_w, pad_top, pad_left, dil_h, dil_w,
                pool_type, lp_order, count_include_pad, label);
            if (rc < 18) begin
                $display("[%0t][TEST] WARN: skipping unparseable manifest line: %s",
                         $time, line);
                return null;
            end
            if (rc < 19) label = "(unlabelled)";

            it = new(idx, label,
                     batch, channels,
                     in_h, in_w, out_h, out_w,
                     pool_h, pool_w, stride_h, stride_w,
                     pad_top, pad_left, dil_h, dil_w,
                     pool_type, lp_order, count_include_pad);
            it.load_fixture(data_dir);
            return it;
        endfunction

        task run();
            int          fd;
            int          rc;
            string       data_dir;
            string       manifest_path;
            string       line;
            int unsigned n;
            pool_item    it;

            if (!$value$plusargs("DATA_DIR=%s", data_dir))
                data_dir = "pooling_test_data";

            e = new();

            manifest_path = $sformatf("%s/manifest.txt", data_dir);
            fd = $fopen(manifest_path, "r");
            if (fd == 0) begin
                $error("[%0t][TEST] Cannot open manifest %s — generate the pooling fixtures before running the testbench",
                       $time, manifest_path);
                $finish;
            end

            $display("==========================================================");
            $display(" PoolingKernel Testbench   data dir = %s", data_dir);
            $display("==========================================================");

            // Pre-build all test items so we know the count up front.
            while (!$feof(fd)) begin
                line = "";
                rc   = $fgets(line, fd);
                if (rc == 0) break;
                it = parse_manifest_line(line, data_dir);
                if (it != null) tests.push_back(it);
            end
            $fclose(fd);

            n = tests.size();
            if (n == 0) begin
                $error("[%0t][TEST] No tests parsed from %s", $time, manifest_path);
                $finish;
            end
            $display(" Loaded %0d test cases", n);
            $display("==========================================================");

            foreach (tests[i]) begin
                $display("----------------------------------------------------------");
                $display(" Test %0d / %0d : %s", i+1, n, tests[i].to_string());
                $display("----------------------------------------------------------");
                e.run_one(tests[i]);
            end

            e.scb.print_summary();

            // JSON report — override path with +REPORT=<path>.
            begin : json_dump
                string report_path;
                if (!$value$plusargs("REPORT=%s", report_path))
                    report_path = "pooling_test_report.json";
                e.scb.write_json_report(report_path);
            end
        endtask
    endclass

    // =========================================================================
    // DDRC write-request monitor - logs every write the PS VIP issues to DDR.
    // =========================================================================
    initial begin : ddrc_wr_monitor
        int unsigned wr_cnt;
        wr_cnt = 0;
        forever begin
            @(posedge dut.zynq_ultra_ps_e_0.inst.ddrc.wr_req);
            wr_cnt++;
            $display("[%0t][DDRC_WR#%0d] addr=0x%010h bytes=%0d strb[3:0]=0x%01h data[31:0]=0x%08h",
                     $time, wr_cnt,
                     dut.zynq_ultra_ps_e_0.inst.ddrc.wr_addr,
                     dut.zynq_ultra_ps_e_0.inst.ddrc.wr_bytes,
                     dut.zynq_ultra_ps_e_0.inst.ddrc.wr_strb[3:0],
                     dut.zynq_ultra_ps_e_0.inst.ddrc.wr_data[31:0]);
        end
    end

    // =========================================================================
    // Workaround for arb_wr_6 VIP race condition (same fix as conv_tb / matmul_tb).
    //
    // prt_req fires in the active event region before prt_data/prt_strb settle
    // in the inactive region.  Re-apply every byte-enabled write 1 ns later
    // when all signals have settled, using direct memory model access.
    // =========================================================================
    initial begin : ddrc_wr_fix
        int unsigned nb, boff;
        logic [39:0] ba;
        logic [31:0] wa;
        logic [ 7:0] bd;
        logic [31:0] tmp_word;
        forever begin
            @(posedge dut.zynq_ultra_ps_e_0.inst.ddrc.wr_req);
            #1;
            nb = int'(dut.zynq_ultra_ps_e_0.inst.ddrc.wr_bytes);
            for (int b = 0; b < nb; b++) begin
                ba = dut.zynq_ultra_ps_e_0.inst.ddrc.wr_addr + 40'(b);
                if (dut.zynq_ultra_ps_e_0.inst.ddrc.wr_strb[b]) begin
                    bd = dut.zynq_ultra_ps_e_0.inst.ddrc.wr_data[b*8 +: 8];
                    shadow_mem[longint'(ba)] = bd;
                end else if (shadow_mem.exists(longint'(ba))) begin
                    // Unstrobed byte: undo whatever the VIP's racing write
                    // deposited by restoring the last value this bench or a
                    // strobed beat wrote there.
                    bd = shadow_mem[longint'(ba)];
                end else begin
                    continue;
                end
                begin
                    wa   = ba[33:2];
                    boff = int'(ba[1:0]);
                    if (wa[28] == 1'b0) begin
                        tmp_word = dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem0[wa[27:0]];
                        tmp_word[boff*8 +: 8] = bd;
                        dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem0[wa[27:0]] = tmp_word;
                    end else begin
                        tmp_word = dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem1[wa[27:0]];
                        tmp_word[boff*8 +: 8] = bd;
                        dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem1[wa[27:0]] = tmp_word;
                    end
                end
            end
        end
    end

    // =========================================================================
    // Logic probes — PoolingKernel FSM state + AXI bus handshake logging
    //
    // Hierarchy:
    //   dut.PoolingKernel_0.inst.*                   - HLS kernel internals
    //   dut.PoolingKernel_0_m_axi_gmem0_*            - AXI bus wires (x reads)
    //   dut.PoolingKernel_0_m_axi_gmem1_*            - AXI bus wires (y writes)
    //
    // ap_CS_fsm is 57-bit one-hot.  Key sub-pipeline instances:
    //   LOOP_123_4_LOOP_124_5  - valid_count accumulation (oh × ow)
    //   LOOP_153_7_LOOP_156_8_LOOP_162_9  - window load into win_buf
    //   LOOP_209_10            - II=1 reduce loop (kTileC × pool_h × pool_w)
    //   LOOP_253_12            - finalize and scatter-write to y
    // =========================================================================

    `define PK dut.PoolingKernel_0.inst

    function automatic int unsigned fsm_decode(input [56:0] v);
        for (int i = 0; i < 57; i++) if (v[i]) return i + 1;
        return 0;
    endfunction

    // ---- Main FSM state-change logger (uncomment to enable) -----------------
    initial begin : pk_fsm_probe
        // int prev_s, cur_s;
        // prev_s = 0;
        // wait (`PK.ap_rst_n === 1'b1);
        // @(posedge dut.zynq_ultra_ps_e_0_pl_clk0);
        // forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
        //     #1;
        //     cur_s = fsm_decode(`PK.ap_CS_fsm);
        //     if (cur_s !== prev_s) begin
        //         $display("[%0t][PK_FSM] %0d->%0d  idle=%b done=%b",
        //                  $time, prev_s, cur_s, `PK.ap_idle, `PK.ap_done);
        //         prev_s = cur_s;
        //     end
        // end
    end

    // ---- gmem0 AR-channel (x reads) -----------------------------------------
    initial begin : pk_gmem0_ar_probe
        forever begin
            @(posedge dut.PoolingKernel_0_m_axi_gmem0_ARVALID or
              posedge dut.PoolingKernel_0_m_axi_gmem0_ARREADY);
            #1;
            if (dut.PoolingKernel_0_m_axi_gmem0_ARVALID | dut.PoolingKernel_0_m_axi_gmem0_ARREADY)
                $display("[%0t][PK_AXI] gmem0 AR: ARVALID=%b ARREADY=%b  ADDR=%016h  LEN=%0d  SIZE=%0d",
                    $time,
                    dut.PoolingKernel_0_m_axi_gmem0_ARVALID,
                    dut.PoolingKernel_0_m_axi_gmem0_ARREADY,
                    dut.PoolingKernel_0_m_axi_gmem0_ARADDR,
                    dut.PoolingKernel_0_m_axi_gmem0_ARLEN,
                    dut.PoolingKernel_0_m_axi_gmem0_ARSIZE);
        end
    end

    // ---- gmem0 R-channel (x read data beats) --------------------------------
    initial begin : pk_gmem0_r_probe
        int r_beat;
        r_beat = 0;
        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
            #1;
            if (dut.PoolingKernel_0_m_axi_gmem0_RVALID & dut.PoolingKernel_0_m_axi_gmem0_RREADY) begin
                $display("[%0t][PK_AXI] gmem0 R:  beat#%0d  RDATA=%08h  RLAST=%b  RRESP=%b",
                    $time, r_beat,
                    dut.PoolingKernel_0_m_axi_gmem0_RDATA,
                    dut.PoolingKernel_0_m_axi_gmem0_RLAST,
                    dut.PoolingKernel_0_m_axi_gmem0_RRESP);
                r_beat++;
            end
        end
    end

    // ---- gmem1 AW-channel (y write addresses) --------------------------------
    initial begin : pk_gmem1_aw_probe
        forever begin
            @(posedge dut.PoolingKernel_0_m_axi_gmem1_AWVALID or
              posedge dut.PoolingKernel_0_m_axi_gmem1_AWREADY);
            #1;
            if (dut.PoolingKernel_0_m_axi_gmem1_AWVALID | dut.PoolingKernel_0_m_axi_gmem1_AWREADY)
                $display("[%0t][PK_AXI] gmem1 AW: AWVALID=%b AWREADY=%b  ADDR=%016h  LEN=%0d",
                    $time,
                    dut.PoolingKernel_0_m_axi_gmem1_AWVALID,
                    dut.PoolingKernel_0_m_axi_gmem1_AWREADY,
                    dut.PoolingKernel_0_m_axi_gmem1_AWADDR,
                    dut.PoolingKernel_0_m_axi_gmem1_AWLEN);
        end
    end

    // ---- gmem1 B-channel (y write response) ---------------------------------
    initial begin : pk_gmem1_b_probe
        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
            #1;
            if (dut.PoolingKernel_0_m_axi_gmem1_BVALID & dut.PoolingKernel_0_m_axi_gmem1_BREADY)
                $display("[%0t][PK_AXI] gmem1 B:  BRESP=%b  (write response ack)",
                    $time, dut.PoolingKernel_0_m_axi_gmem1_BRESP);
        end
    end

    // =========================================================================
    // Testbench top - PS VIP reset sequence, then run all tests
    // =========================================================================
    test t;

    initial begin
        `PS.set_stop_on_error(1);
        `PS.set_debug_level_info(1);

        // POR + system reset, then PL fabric reset.
        `PS.por_srstb_reset(1'b0);   // assert  → DDR model enters reset
        `PS.fpga_soft_reset(32'hF);  // assert PL resets
        #500;
        `PS.por_srstb_reset(1'b1);   // deassert → DDR model comes up cleanly
        #800;
        `PS.fpga_soft_reset(32'h0);  // deassert PL resets → interconnect starts
        #900;

        // BEST_CASE (fixed 21-cycle) write-response latency on HPC0_FPD.
        `PS.set_slave_profile("S_AXI_HPC0_FPD", 0);

        t = new();
        t.run();

        $finish;
    end
endmodule
