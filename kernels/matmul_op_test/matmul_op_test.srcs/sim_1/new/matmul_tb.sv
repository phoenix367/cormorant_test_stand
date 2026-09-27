`timescale 1ns / 1ps
// ============================================================================
// MatmulKernel HLS Testbench - OOP structure (mirrors vectorop_tb.sv)
//
// Components:
//   mm_item    - transaction object (matrix dims, fill values, expected output)
//   driver     - loads DDR, programs AXI-Lite registers, asserts ap_start
//   monitor    - waits for interrupt, services it
//   scoreboard - backdoor-reads C and verifies every element
//   env        - connects all components; runs one transaction
//   test       - builds test matrix, iterates, prints summary
//
// DUT: design_matmul block design (no external ports).
//   Zynq UltraScale+ PS VIP: AXI-Lite master HPM0_FPD + DDR slave HPC0_FPD
//   Kernel: MatmulKernel (ap_fixed<16,8>; gmem0/A, gmem1/B, gmem2/C)
//
// ap_fixed<16,8> encoding:
//   real_value × 256 = raw int16
//   1.0=0x0100  2.0=0x0200  4.0=0x0400  8.0=0x0800
//  -1.0=0xFF00  100.0=0x6400  -100.0=0x9C00
//   max=0x7FFF (+127.996)  min=0x8000 (-128.0)
//
// Test fixtures live in the directory passed via +DATA_DIR=<dir>:
//   manifest.txt        — one row per test (8 ints + label, see test::run)
//   test_<NN>_a.hex     — input A matrix as 16-bit raw values, one per line
//   test_<NN>_b.hex     — input B matrix as 16-bit raw values, one per line
//   test_<NN>_c.hex     — reference C output as 16-bit raw values, one per line
// The testbench compares the kernel's C output element-wise against c_ref,
// so any operand pattern (random, saturating, broadcast, etc.) is verified
// position-by-position rather than against a single closed-form expected.
//
// Kernel AXI-Lite register layout (base 0xA000_0000).
//   IMPORTANT: verify these offsets from the HLS synthesis report:
//     solution1/impl/ip/drivers/MatmulKernel_v1_0/src/xmatmulkernel_hw.h
//   Each 64-bit pointer uses 2×4-byte regs + 4-byte reserved = 12 bytes.
//   Each 32-bit scalar uses 1×4-byte reg + 4-byte reserved = 8 bytes.
//
//   0x00 ap_ctrl   0x04 gie    0x08 ier    0x0C isr
//   0x10 a_lo      0x14 a_hi  (0x18 reserved)
//   0x1C b_lo      0x20 b_hi  (0x24 reserved)
//   0x28 c_lo      0x2C c_hi  (0x30 reserved)
//   0x34 n         (0x38 reserved)
//   0x3C k         (0x40 reserved)
//   0x44 m         (0x48 reserved)
//   0x4C batch     (0x50 reserved)
//   0x54 a_batch_stride  (0x58 reserved)
//   0x5C b_batch_stride  (0x60 reserved)
//   0x64 c_batch_stride  (0x68 reserved)
//
// Test matrix:
//   1×1×1               - degenerate minimum
//   4×4×16 (=TileN×K×TileM)  - exact tile fill, A=B=1.0  → C=4.0
//   5×4×16              - partial last N-tile
//   4×4×17              - partial last M-tile
//   4×3×16              - K not a multiple of TileN
//   sat+: 2×2×2 A=100 B=1   → C=200 → AP_MAX=0x7FFF
//   sat-: 2×2×2 A=-100 B=1  → C=-200 → AP_MIN=0x8000
//   batch=2 no broadcast
//   batch=2 A broadcasts (a_batch_stride=0)
// ============================================================================

module matmul_tb;
    // -----------------------------------------------------------------------
    // DUT instantiation.
    // Update the module name to match your block design wrapper name.
    // -----------------------------------------------------------------------
    design_matmul dut();

    // PS VIP access path (update if the Zynq instance has a different name).
    `define PS dut.zynq_ultra_ps_e_0.inst

    // -----------------------------------------------------------------------
    // Kernel AXI-Lite register map  (base 0xA000_0000)
    // -----------------------------------------------------------------------
    localparam [39:0] CTRL_BASE          = 40'hA000_0000;
    localparam [39:0] REG_AP_CTRL        = CTRL_BASE + 40'h00;
    localparam [39:0] REG_GIE            = CTRL_BASE + 40'h04;
    localparam [39:0] REG_IER            = CTRL_BASE + 40'h08;
    localparam [39:0] REG_ISR            = CTRL_BASE + 40'h0C;
    localparam [39:0] REG_A_LO           = CTRL_BASE + 40'h10;
    localparam [39:0] REG_A_HI           = CTRL_BASE + 40'h14;
    localparam [39:0] REG_B_LO           = CTRL_BASE + 40'h1C;
    localparam [39:0] REG_B_HI           = CTRL_BASE + 40'h20;
    localparam [39:0] REG_C_LO           = CTRL_BASE + 40'h28;
    localparam [39:0] REG_C_HI           = CTRL_BASE + 40'h2C;
    localparam [39:0] REG_N              = CTRL_BASE + 40'h34;
    localparam [39:0] REG_K              = CTRL_BASE + 40'h3C;
    localparam [39:0] REG_M              = CTRL_BASE + 40'h44;
    localparam [39:0] REG_BATCH          = CTRL_BASE + 40'h4C;
    localparam [39:0] REG_A_BATCH_STRIDE = CTRL_BASE + 40'h54;
    localparam [39:0] REG_B_BATCH_STRIDE = CTRL_BASE + 40'h5C;
    localparam [39:0] REG_C_BATCH_STRIDE = CTRL_BASE + 40'h64;
    localparam [39:0] REG_B_PACKED       = CTRL_BASE + 40'h6C;   // 0: row-major B, 1: tile-major packed B
    localparam [39:0] REG_GEMV_KW        = CTRL_BASE + 40'h74;   // 0: tiled path, 1/2/4/8: GEMV streaming
    localparam [39:0] REG_A_TO_B_LO      = CTRL_BASE + 40'h7C;   // GEMV: address of b minus address of a
    localparam [39:0] REG_A_TO_B_HI      = CTRL_BASE + 40'h80;

    // -----------------------------------------------------------------------
    // Testbench parameters
    // -----------------------------------------------------------------------
    localparam int unsigned ELEM_BYTES = 2;    // sizeof(ap_fixed<16,8>)
    localparam integer      CHUNK_SIZE = 1024; // PS VIP transfer chunk (bytes)
    localparam integer      CHUNK_BITS = CHUNK_SIZE * 8;
    localparam int unsigned MEM_GAP    = 64 * 1024; // guard gap between arrays (bytes)
    // kTileM of the synthesised kernel (platforms/kv260.json kernels.matmul.tile_m):
    // the packed-B image pads m to a multiple of it (MatmulKernel.h).
    localparam int unsigned TILE_M     = 32;

    localparam logic [15:0] C_POISON = 16'hDEAD; // sentinel for un-written C elements

    // Interrupt output from the HLS kernel.
    // Update the hierarchical path to match your block design instance names.
    wire kernel_irq = dut.MatmulKernel_0_interrupt;

    // -----------------------------------------------------------------------
    // Helper: round n up to the next multiple of align (power of two).
    // -----------------------------------------------------------------------
    function automatic int unsigned align_up(int unsigned n, int unsigned align);
        return (n + align - 1) & ~(align - 1);
    endfunction

    // =========================================================================
    // Transaction - one complete matmul call
    // =========================================================================
    class mm_item;
        // Manifest-supplied identity (used for log lines + JSON report)
        int unsigned index;
        string       label;

        // Matrix dimensions / stride control (every field comes from the
        // manifest; the upstream pipeline that generated the manifest stays
        // the single source of truth, matching conv_tb / pooling_tb).
        int unsigned n;
        int unsigned k;
        int unsigned m;
        int unsigned batch;
        int unsigned a_batch_stride; // 0 = broadcast A across batches
        int unsigned b_batch_stride; // 0 = broadcast B across batches
        int unsigned b_packed;       // 0: row-major, 1: tile-major packed (manifest col 8)
        int unsigned gemv_kw;        // 0: tiled path, 1/2/4/8: GEMV image kernel width (col 9)
        int unsigned c_batch_stride; // always = n × m  (derived, not in manifest)

        // Element counts (derived from dims + stride mode).
        int unsigned a_count;
        int unsigned b_count;
        int unsigned c_count;

        // DDR base addresses (computed from dims + gaps)
        logic [39:0] addr_a;
        logic [39:0] addr_b;
        logic [39:0] addr_c;

        // Fixture data (loaded from <data_dir>/test_<NN>_{a,b,c}.hex).
        logic [15:0] a_data[];
        logic [15:0] b_data[];
        logic [15:0] c_ref [];

        function new(int unsigned   index_,
                     string         lbl,
                     int unsigned   n_,
                     int unsigned   k_,
                     int unsigned   m_,
                     int unsigned   batch_,
                     int unsigned   a_batch_stride_,
                     int unsigned   b_batch_stride_,
                     int unsigned   b_packed_ = 0,
                     int unsigned   gemv_kw_  = 0);
            int unsigned a_region_bytes, b_region_bytes;

            this.index          = index_;
            this.label          = lbl;
            this.n              = n_;
            this.k              = k_;
            this.m              = m_;
            this.batch          = batch_;
            this.a_batch_stride = a_batch_stride_;
            this.b_batch_stride = b_batch_stride_;
            this.b_packed       = b_packed_;
            this.gemv_kw        = gemv_kw_;   // GEMV image: k x m per slice, like row-major B
            this.c_batch_stride = n_ * m_;

            // Element counts.
            //   a_batch_stride == 0   → A is broadcast (single matrix in DDR)
            //   a_batch_stride != 0   → batch matrices, advancing by stride
            //   For batch == 1 the manifest emits a_batch_stride = n*k so the
            //   kernel pointer arithmetic is well-defined; the kernel ignores
            //   the stride at bi=0 either way.
            this.a_count = (a_batch_stride_ == 0) ? (n_ * k_)
                                                  : (batch_ * a_batch_stride_);
            // Packed B (MatmulKernel.h "Packed (tile-major) B layout"): each
            // batch slice is k x roundup(m, TILE_M) elements; the manifest's
            // b_stride is already in packed elements.
            this.b_count = (b_batch_stride_ == 0)
                         ? (b_packed_ ? (k_ * align_up(m_, TILE_M)) : (k_ * m_))
                         : (batch_ * b_batch_stride_);
            this.c_count = batch_ * n_ * m_;

            // DDR layout: A | gap | B | gap | C
            a_region_bytes = align_up(this.a_count * ELEM_BYTES, 16);
            b_region_bytes = align_up(this.b_count * ELEM_BYTES, 16);

            // Alternate the DDR base between consecutive tests so a test's
            // inputs never occupy addresses the previous test's kernel
            // wrote: the PS DDR model commits the kernel's last
            // (partial-strobe) C write late enough to land on top of the
            // next test's backdoor-loaded B (39-case sequence, 2026-09-25:
            // B[49922] of test 33 read back as C[98] of test 32 on every
            // kernel revision, never when the case ran alone).
            this.addr_a = 40'h1000_0000 + 40'(index_ % 2) * 40'h0400_0000;
            this.addr_b = this.addr_a + 40'(a_region_bytes) + 40'(MEM_GAP);
            this.addr_c = this.addr_b + 40'(b_region_bytes) + 40'(MEM_GAP);
        endfunction

        // Load A / B / C_ref from <dir>/test_<NN>_{a,b,c}.hex.  Each file
        // holds one 16-bit raw value per line — same format as the conv and
        // pooling test stands.
        function void load_fixture(string dir);
            string idx_str;
            a_data = new[a_count];
            b_data = new[b_count];
            c_ref  = new[c_count];
            idx_str = $sformatf("%02d", index);
            $readmemh($sformatf("%s/test_%s_a.hex", dir, idx_str), a_data);
            $readmemh($sformatf("%s/test_%s_b.hex", dir, idx_str), b_data);
            $readmemh($sformatf("%s/test_%s_c.hex", dir, idx_str), c_ref);
        endfunction

        function string to_string();
            return $sformatf(
                "%-32s  N=%0d K=%0d M=%0d  batch=%0d  a_stride=%0d b_stride=%0d  gemv_kw=%0d  |A|=%0d |B|=%0d |C|=%0d",
                label, n, k, m, batch,
                a_batch_stride, b_batch_stride, gemv_kw,
                a_count, b_count, c_count);
        endfunction
    endclass

    // =========================================================================
    // Per-test result record — captured by scoreboard, dumped to JSON at end.
    // Mirrors conv_tb / pooling_tb so the test stand's run_tb.sh wrapper can
    // parse any kernel's report with the same logic.
    // =========================================================================
    class test_result;
        // Identity
        int unsigned index;
        string       label;
        int unsigned errors;
        int unsigned total_elements;

        // Geometry mirror (kept here so the JSON report is self-contained)
        int unsigned n, k, m, batch;
        int unsigned a_batch_stride, b_batch_stride, c_batch_stride;

        // Per-test simulation timing (file-level `timescale 1ns/1ps).
        // start_ns is sampled before the driver programs registers; end_ns
        // after the scoreboard finishes verifying.
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
        // (used to poison C[] before each run).
        local task fill_const_ddr(input [39:0]      base,
                                  input int unsigned nbytes,
                                  input [15:0]       val);
            logic [CHUNK_BITS-1:0] buf_mem;
            int unsigned           i, n_chunks, rem;
            for (i = 0; i < CHUNK_SIZE / 2; i++)
                buf_mem[i*16 +: 16] = val;
            n_chunks = nbytes / CHUNK_SIZE;
            rem      = nbytes % CHUNK_SIZE;
            for (i = 0; i < n_chunks; i++)
                `PS.write_mem(buf_mem, base + 40'(i * CHUNK_SIZE), CHUNK_SIZE);
            if (rem > 0)
                `PS.write_mem(buf_mem, base + 40'(n_chunks * CHUNK_SIZE), rem);
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
            end
            if (rem > 0) begin
                buf_mem = '0;
                for (i = 0; i < rem / 2; i++) begin
                    idx = n_chunks * items_per_chunk + i;
                    if (idx < n_elements)
                        buf_mem[i*16 +: 16] = data[idx];
                end
                `PS.write_mem(buf_mem, base + 40'(n_chunks * CHUNK_SIZE), rem);
            end
        endtask

        task run(mm_item item);
            int unsigned c_total_bytes;
            logic [63:0] a_to_b;
            $display("[%0t][DRV] %s", $time, item.to_string());

            c_total_bytes = align_up(item.c_count * ELEM_BYTES, 16);

            $display("[%0t][DRV] Loading A   (%0d elements) ...",
                     $time, item.a_count);
            write_data_ddr(item.addr_a, item.a_data, item.a_count);

            $display("[%0t][DRV] Loading B   (%0d elements) ...",
                     $time, item.b_count);
            write_data_ddr(item.addr_b, item.b_data, item.b_count);

            // Poison C so any missed element is detected by the scoreboard.
            $display("[%0t][DRV] Pre-filling C (%0d B) with 0x%04h ...",
                     $time, c_total_bytes, C_POISON);
            fill_const_ddr(item.addr_c, c_total_bytes, C_POISON);

            // Program kernel AXI-Lite registers.
            $display("[%0t][DRV] Programming registers ...", $time);
            axil_write(REG_A_LO,           item.addr_a[31:0]);
            axil_write(REG_A_HI,           {24'b0, item.addr_a[39:32]});
            axil_write(REG_B_LO,           item.addr_b[31:0]);
            axil_write(REG_B_HI,           {24'b0, item.addr_b[39:32]});
            axil_write(REG_C_LO,           item.addr_c[31:0]);
            axil_write(REG_C_HI,           {24'b0, item.addr_c[39:32]});
            axil_write(REG_N,              32'(item.n));
            axil_write(REG_K,              32'(item.k));
            axil_write(REG_M,              32'(item.m));
            axil_write(REG_BATCH,          32'(item.batch));
            axil_write(REG_A_BATCH_STRIDE, 32'(item.a_batch_stride));
            axil_write(REG_B_BATCH_STRIDE, 32'(item.b_batch_stride));
            axil_write(REG_C_BATCH_STRIDE, 32'(item.c_batch_stride));
            axil_write(REG_B_PACKED,       32'(item.b_packed));
            // GEMV (MatmulKernel.h "GEMV streaming mode"): port a reaches its
            // half of B at a_to_b bytes.  Written for every case — the
            // registers keep their value across calls.
            a_to_b = 64'(item.addr_b) - 64'(item.addr_a);
            axil_write(REG_GEMV_KW,        32'(item.gemv_kw));
            axil_write(REG_A_TO_B_LO,      a_to_b[31:0]);
            axil_write(REG_A_TO_B_HI,      a_to_b[63:32]);

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

        task run(mm_item item);
            logic [15:0] irq_status;
            logic [31:0] isr_val, ap_ctrl_val;

            $display("[%0t][MON] Waiting for interrupt ...", $time);
            irq_status = 16'h0;
            fork
                begin : irq_wait
                    `PS.wait_interrupt(4'd0, irq_status);
                end
                begin : irq_timeout
                    // Matmul is slower than VectorOP; allow up to 500 ms sim-time.
                    // Increase if you synthesise large N/K/M values.
                    #500_000_000;
                end
            join_any
            disable fork;

            if (!irq_status[0]) begin
                $error("[%0t][MON] TIMEOUT: no interrupt after 500 ms sim-time  test=%s",
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
    // Scoreboard - backdoor-reads C and checks every element
    // =========================================================================
    class scoreboard;
        int unsigned total_tests = 0;
        int unsigned pass_cnt    = 0;
        int unsigned fail_cnt    = 0;
        test_result  results[$];

        task run(mm_item item, longint unsigned t_start_ns = 0);
            logic [CHUNK_BITS-1:0] chunk_buf;
            int unsigned n_bytes, n_chunks, rem, errors, eidx, w, total_elems;
            logic [15:0] elem, exp;
            test_result  tr;

            total_elems = item.c_count;
            n_bytes     = total_elems * ELEM_BYTES;
            n_chunks    = n_bytes / CHUNK_SIZE;
            rem         = n_bytes % CHUNK_SIZE;
            errors      = 0;

            tr                     = new();
            tr.index               = item.index;
            tr.label               = item.label;
            tr.n                   = item.n;
            tr.k                   = item.k;
            tr.m                   = item.m;
            tr.batch               = item.batch;
            tr.a_batch_stride      = item.a_batch_stride;
            tr.b_batch_stride      = item.b_batch_stride;
            tr.c_batch_stride      = item.c_batch_stride;
            tr.total_elements      = total_elems;

            $display("[%0t][SCB] Verifying C[0..%0d] (%0d elem × %0d B = %0d B) against c_ref ...",
                     $time, total_elems - 1, total_elems,
                     ELEM_BYTES, n_bytes);

            for (int i = 0; i < int'(n_chunks); i++) begin
                `PS.read_mem(item.addr_c + 40'(i * CHUNK_SIZE), CHUNK_SIZE, chunk_buf);
                for (w = 0; w < CHUNK_SIZE / 2; w++) begin
                    eidx = i * (CHUNK_SIZE / 2) + w;
                    if (eidx >= total_elems) break;
                    elem = chunk_buf[w*16 +: 16];
                    exp  = item.c_ref[eidx];
                    if (elem !== exp) begin
                        if (errors < 5)
                            $display("[%0t][SCB] MISMATCH C[%0d]: got=0x%04h  exp=0x%04h",
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
                `PS.read_mem(item.addr_c + 40'(n_chunks * CHUNK_SIZE), rem, chunk_buf);
                for (w = 0; w < rem / 2; w++) begin
                    eidx = n_chunks * (CHUNK_SIZE / 2) + w;
                    if (eidx >= total_elems) break;
                    elem = chunk_buf[w*16 +: 16];
                    exp  = item.c_ref[eidx];
                    if (elem !== exp) begin
                        if (errors < 5)
                            $display("[%0t][SCB] MISMATCH C[%0d]: got=0x%04h  exp=0x%04h",
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
                $display("[%0t][SCB] PASS  %-32s  %0d×%0d×%0d batch=%0d",
                         $time, item.label, item.n, item.k, item.m, item.batch);
            end else begin
                fail_cnt++;
                $display("[%0t][SCB] FAIL  %-32s  %0d/%0d mismatches",
                         $time, item.label, errors, total_elems);
            end
        endtask

        task print_summary();
            $display("==========================================================");
            $display(" MatmulKernel Test Summary: %0d / %0d passed",
                     pass_cnt, total_tests);
            if (fail_cnt == 0)
                $display(" ALL TESTS PASSED");
            else
                $display(" %0d TEST(S) FAILED", fail_cnt);
            $display("==========================================================");
        endtask

        // Emit a JSON report describing every test, its parameters, and the
        // first MAX_MM mismatches for any failing test.  Shape matches the
        // conv / pooling testbenches so run_tb.sh's parser handles any kernel.
        task write_json_report(string path);
            int          fd;
            test_result  tr;
            fd = $fopen(path, "w");
            if (fd == 0) begin
                $error("[SCB] Failed to open JSON report file: %s", path);
                return;
            end

            $fdisplay(fd, "{");
            $fdisplay(fd, "  \"kernel\": \"MatmulKernel\",");
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
                $fwrite(fd, "        \"n\": %0d, \"k\": %0d, \"m\": %0d, \"batch\": %0d,\n",
                            tr.n, tr.k, tr.m, tr.batch);
                $fwrite(fd, "        \"a_batch_stride\": %0d, \"b_batch_stride\": %0d, \"c_batch_stride\": %0d\n",
                            tr.a_batch_stride, tr.b_batch_stride, tr.c_batch_stride);
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

        task run_one(mm_item item);
            longint unsigned t_start_ns;

            t_start_ns = $time;
            drv.run(item);
            mon.run(item);

            // Diagnostic DDR probe: sample first and last C element directly
            // from the DDRC memory model before the scoreboard reads.
            begin : ddr_probe
                logic [31:0] w_first, w_last;
                int unsigned widx_first, widx_last, total_elems;
                logic [15:0] exp_first, exp_last;
                total_elems  = item.c_count;
                widx_first   = item.addr_c[31:2];
                widx_last    = item.addr_c[31:2] +
                               ((total_elems - 1) * ELEM_BYTES) / 4;
                w_first = dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem0[widx_first];
                w_last  = dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem0[widx_last];
                exp_first = item.c_ref[0];
                exp_last  = item.c_ref[total_elems - 1];
                $display("[%0t][PROBE] C[0]   word ddr_mem0[0x%08h]=0x%08h  (exp lo-halfword=0x%04h)",
                         $time, widx_first, w_first, exp_first);
                $display("[%0t][PROBE] C[%0d] word ddr_mem0[0x%08h]=0x%08h  (exp=0x????%04h)",
                         $time, total_elems-1, widx_last, w_last, exp_last);
            end

            scb.run(item, t_start_ns);
        endtask
    endclass

    // =========================================================================
    // Test - defines test matrix, runs all cases, prints summary
    // =========================================================================
    class test;
        env     e;
        mm_item tests[$];

        // Parse one manifest line — 8 or 9 ints + 1 trailing label token —
        // and create + load an mm_item.  Returns null if the line is
        // blank, a comment, or unparseable.  Manifest column layout
        // (matches the upstream matmul reference dump):
        //
        //   idx n k m batch a_stride b_stride b_packed gemv_kw   label
        //
        // gemv_kw is present when the header comment names it (dumps since
        // the GEMV streaming mode); older manifests have 8 ints.
        //
        // a_stride / b_stride are per-batch element counts.  A value of
        // 0 means "broadcast that operand across batches" (single matrix
        // in DDR).  Per-test a/b/c arrays are loaded from
        // <data_dir>/test_<NN>_{a,b,c}.hex via $readmemh.
        // Does `s` contain `sub`?
        function automatic bit str_contains(string s, string sub);
            for (int i = 0; i + sub.len() <= s.len(); i++)
                if (s.substr(i, i + sub.len() - 1) == sub) return 1;
            return 0;
        endfunction

        function automatic mm_item parse_manifest_line(string line,
                                                        string data_dir,
                                                        bit    has_gemv);
            int unsigned idx;
            int unsigned n_, k_, m_, batch_;
            int unsigned a_stride, b_stride, b_packed, gemv_kw;
            string       label;
            int          rc;
            int          first;
            mm_item      it;

            // Skip leading whitespace; ignore blank or comment lines.
            first = 0;
            while (first < line.len() &&
                   (line.getc(first) == " "  || line.getc(first) == "\t" ||
                    line.getc(first) == "\n" || line.getc(first) == "\r"))
                first++;
            if (first == line.len()) return null;
            if (line.getc(first) == "#") return null;

            gemv_kw = 0;
            if (has_gemv) begin
                rc = $sscanf(line, "%d %d %d %d %d %d %d %d %d %s",
                    idx, n_, k_, m_, batch_, a_stride, b_stride, b_packed, gemv_kw, label);
                rc = rc - 1;                 // count as the 8-int layout below
            end else
                rc = $sscanf(line, "%d %d %d %d %d %d %d %d %s",
                    idx, n_, k_, m_, batch_, a_stride, b_stride, b_packed, label);
            if (rc < 8) begin
                $display("[%0t][TEST] WARN: skipping unparseable manifest line: %s",
                         $time, line);
                return null;
            end
            if (rc < 9) label = "(unlabelled)";

            it = new(idx, label, n_, k_, m_, batch_, a_stride, b_stride, b_packed, gemv_kw);
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
            mm_item      it;
            bit          has_gemv;

            if (!$value$plusargs("DATA_DIR=%s", data_dir))
                data_dir = "matmul_test_data";

            e = new();

            manifest_path = $sformatf("%s/manifest.txt", data_dir);
            fd = $fopen(manifest_path, "r");
            if (fd == 0) begin
                $error("[%0t][TEST] Cannot open manifest %s — generate the matmul fixtures before running the testbench",
                       $time, manifest_path);
                $finish;
            end

            $display("==========================================================");
            $display(" MatmulKernel Testbench   data dir = %s", data_dir);
            $display("==========================================================");

            // Pre-build all test items so we know the count up front.  The
            // header comment of a manifest with the gemv_kw column names it.
            has_gemv = 0;
            while (!$feof(fd)) begin
                line = "";
                rc   = $fgets(line, fd);
                if (rc == 0) break;
                if (line.len() > 0 && line.getc(0) == "#" && str_contains(line, "gemv_kw"))
                    has_gemv = 1;
                it = parse_manifest_line(line, data_dir, has_gemv);
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
                #(20us);   // settle (see the DDR-base note in mm_item::new)
            end

            e.scb.print_summary();

            // JSON report — override path with +REPORT=<path>.
            begin : json_dump
                string report_path;
                if (!$value$plusargs("REPORT=%s", report_path))
                    report_path = "matmul_test_report.json";
                e.scb.write_json_report(report_path);
            end
        endtask
    endclass

    // =========================================================================
    // DDRC write-request monitor - logs every write the PS VIP issues to DDR.
    // Same as vectorop_tb: retained for debugging AXI write behaviour.
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
    // Workaround for arb_wr_6 VIP race condition (same fix as vectorop_tb).
    //
    // prt_req fires in the active event region before prt_data/prt_strb settle
    // in the inactive region.  We re-apply every byte-enabled write 1 ns later
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
                if (dut.zynq_ultra_ps_e_0.inst.ddrc.wr_strb[b]) begin
                    ba   = dut.zynq_ultra_ps_e_0.inst.ddrc.wr_addr + 40'(b);
                    wa   = ba[33:2];
                    boff = int'(ba[1:0]);
                    bd   = dut.zynq_ultra_ps_e_0.inst.ddrc.wr_data[b*8 +: 8];
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
    // Logic probes - MatmulKernel FSM state + AXI handshake logging
    //
    // Hierarchy:
    //   dut.MatmulKernel_0.inst.*        - HLS kernel internals (MatmulKernel module)
    //   dut.MatmulKernel_0_m_axi_gmem*_* - AXI data bus wires inside design_matmul
    //
    // The kernel-internals FSM probe below is DISABLED — see the note on the
    // mm_fsm_probe block.  Only the AXI handshake probes, which reference
    // stable design-level wire names, remain active.
    // =========================================================================

    // Shorthand for the kernel internals path.
    `define MM dut.MatmulKernel_0.inst

    // Decode 19-bit one-hot FSM to a plain integer 1..19 (0 = all-zero / reset).
    function automatic int unsigned fsm_decode(input [18:0] v);
        for (int i = 0; i < 19; i++) if (v[i]) return i + 1;
        return 0;
    endfunction

    // ---- Main FSM state-change logger -----------------------------------
    //
    // DISABLED: the loop below probes MatmulKernel's internal FSM and
    // pipeline sub-instances by hierarchical name.  Since the kernel was
    // re-architected as an HLS DATAFLOW design, those names no longer exist:
    // there is no single one-hot `ap_CS_fsm` / `ap_ST_fsm_state*_blk`, and the
    // `grp_*_Pipeline_VITIS_LOOP_<line>_fu_<N>` sub-blocks are renamed on every
    // re-synthesis (source line numbers and the `fu_<N>` suffixes are not
    // stable).  Referencing them aborts xelab elaboration.  Kept commented for
    // waveform debugging — mirrors conv_tb.sv's ck_fsm_probe.
    initial begin : mm_fsm_probe
        int prev_s, cur_s;
        prev_s = 0;
        wait (`MM.ap_rst_n === 1'b1);
        @(posedge dut.zynq_ultra_ps_e_0_pl_clk0);

//        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
//            #1; // sample a tick after the clock edge to avoid races
//            cur_s = fsm_decode(`MM.ap_CS_fsm);
//            if (cur_s !== prev_s) begin
//                $display("[%0t][MM_FSM] %0d->%0d idle=%b done=%b",
//                    $time, prev_s, cur_s, `MM.ap_idle, `MM.ap_done);
//                prev_s = cur_s;
//            end
//        end
    end

    // ---- gmem0 AR-channel (A matrix reads) ------------------------------
    // AXI bus wires are in design_vectorop scope, named MatmulKernel_0_m_axi_gmem0_*.
    initial begin : mm_gmem0_ar_probe
        forever begin
            @(posedge dut.MatmulKernel_0_m_axi_gmem0_ARVALID or
              posedge dut.MatmulKernel_0_m_axi_gmem0_ARREADY);
            #1;
            if (dut.MatmulKernel_0_m_axi_gmem0_ARVALID | dut.MatmulKernel_0_m_axi_gmem0_ARREADY)
                $display("[%0t][MM_AXI] gmem0 AR: ARVALID=%b ARREADY=%b  ADDR=%016h  LEN=%0d  SIZE=%0d",
                    $time,
                    dut.MatmulKernel_0_m_axi_gmem0_ARVALID,
                    dut.MatmulKernel_0_m_axi_gmem0_ARREADY,
                    dut.MatmulKernel_0_m_axi_gmem0_ARADDR,
                    dut.MatmulKernel_0_m_axi_gmem0_ARLEN,
                    dut.MatmulKernel_0_m_axi_gmem0_ARSIZE);
        end
    end

    // ---- gmem0 R-channel (read data beats) ------------------------------
    initial begin : mm_gmem0_r_probe
        int r_beat;
        r_beat = 0;
        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
            #1;
            if (dut.MatmulKernel_0_m_axi_gmem0_RVALID & dut.MatmulKernel_0_m_axi_gmem0_RREADY) begin
                $display("[%0t][MM_AXI] gmem0 R:  beat#%0d  RDATA=%08h  RLAST=%b  RRESP=%b",
                    $time, r_beat,
                    dut.MatmulKernel_0_m_axi_gmem0_RDATA,
                    dut.MatmulKernel_0_m_axi_gmem0_RLAST,
                    dut.MatmulKernel_0_m_axi_gmem0_RRESP);
                r_beat++;
            end
        end
    end

    // ---- gmem2 AW-channel (C matrix write addresses) --------------------
    initial begin : mm_gmem2_aw_probe
        forever begin
            @(posedge dut.MatmulKernel_0_m_axi_gmem2_AWVALID or
              posedge dut.MatmulKernel_0_m_axi_gmem2_AWREADY);
            #1;
            if (dut.MatmulKernel_0_m_axi_gmem2_AWVALID | dut.MatmulKernel_0_m_axi_gmem2_AWREADY)
                $display("[%0t][MM_AXI] gmem2 AW: AWVALID=%b AWREADY=%b  ADDR=%016h  LEN=%0d",
                    $time,
                    dut.MatmulKernel_0_m_axi_gmem2_AWVALID,
                    dut.MatmulKernel_0_m_axi_gmem2_AWREADY,
                    dut.MatmulKernel_0_m_axi_gmem2_AWADDR,
                    dut.MatmulKernel_0_m_axi_gmem2_AWLEN);
        end
    end

    // ---- gmem2 B-channel (write response) -------------------------------
    initial begin : mm_gmem2_b_probe
        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
            #1;
            if (dut.MatmulKernel_0_m_axi_gmem2_BVALID & dut.MatmulKernel_0_m_axi_gmem2_BREADY)
                $display("[%0t][MM_AXI] gmem2 B:  BRESP=%b  (write response ack)",
                    $time, dut.MatmulKernel_0_m_axi_gmem2_BRESP);
        end
    end

    // =========================================================================
    // Testbench top - PS VIP reset sequence, then run all tests
    // =========================================================================
    test t;

    initial begin
        `PS.set_stop_on_error(1);
        `PS.set_debug_level_info(1);

        // POR + system reset, then PL fabric reset (same sequence as vectorop_tb).
        `PS.por_srstb_reset(1'b0);   // assert  → DDR model enters reset
        `PS.fpga_soft_reset(32'hF);  // assert PL resets
        #500;
        `PS.por_srstb_reset(1'b1);   // deassert → DDR model comes up cleanly
        #800;
        `PS.fpga_soft_reset(32'h0);  // deassert PL resets → interconnect starts
        #900;

        // BEST_CASE (fixed 21-cycle) write-response latency on HPC0_FPD to
        // keep the AXI write pipeline flowing without stalls between bursts.
        `PS.set_slave_profile("S_AXI_HPC0_FPD", 0);

        t = new();
        t.run();

        $finish;
    end
endmodule
