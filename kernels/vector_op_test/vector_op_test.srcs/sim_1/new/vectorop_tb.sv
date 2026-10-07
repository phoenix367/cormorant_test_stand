`timescale 1ns / 1ps
// ============================================================================
// VectorOP HLS Kernel Testbench — OOP structure
//
// Components:
//   vop_item   — transaction object (op code, fill values, expected output)
//   driver     — loads DDR, programs AXI-Lite registers, asserts ap_start
//   monitor    — waits for interrupt, services it
//   scoreboard — backdoor-reads C and verifies every element
//   env        — connects all components; runs one transaction
//   test       — builds test matrix, iterates, prints summary
//
// DUT: design_vectorop block design (no external ports)
//   Zynq UltraScale+ PS VIP: AXI-Lite master HPM0_FPD + DDR slave HPC0_FPD
//   Kernel: VectorOPKernel (ap_fixed<16,8>; gmem0/A, gmem1/B, gmem2/C)
//
// Test fixtures live in the directory passed via +DATA_DIR=<dir>:
//   manifest.txt        — one row per test (6 ints + label, see test::run)
//   test_<NN>_a.hex     — input A vector as 16-bit raw values, one per line
//   test_<NN>_b.hex     — input B vector as 16-bit raw values, one per line
//   test_<NN>_c.hex     — reference C output as 16-bit raw values, one per line
// The testbench compares the kernel's C output element-wise against c_ref,
// so any operand pattern (constant, ramp, random, saturating) is verified
// position-by-position rather than against a single closed-form expected.
//
// ap_fixed<16,8> encoding (value × 256 = raw int16):
//   1.0=0x0100  2.0=0x0200  3.0=0x0300  4.0=0x0400  5.0=0x0500  6.0=0x0600
//   100.0=0x6400  -1.0=0xFF00  -2.0=0xFE00  -100.0=0x9C00
//   max=0x7FFF (+127.996)  min=0x8000 (-128.0)
//
// Kernel AXI-Lite register layout (declaration order: a, b, c, size, op,
// outer, a_inc, b_inc):
//   Each 64-bit pointer occupies 2 × 4-byte regs + 4-byte reserved gap.
//   Each 32-bit scalar occupies 1 × 4-byte reg   + 4-byte reserved gap.
//   Source: drivers/VectorOPKernel_v1_0/src/xvectoropkernel_hw.h
//   0x00 ap_ctrl  0x04 gie  0x08 ier  0x0C isr
//   0x10 a_lo     0x14 a_hi (0x18 reserved)
//   0x1C b_lo     0x20 b_hi (0x24 reserved)
//   0x28 c_lo     0x2C c_hi (0x30 reserved)
//   0x34 size               (0x38 reserved)
//   0x3C op                 (0x40 reserved)
//   0x44 outer              (0x48 reserved)
//   0x4C a_inc              (0x50 reserved)
//   0x54 b_inc              (0x58 reserved)
//   0x5C act                (0x60 reserved)   fused activation, 0 = none
//   0x64 alpha              (0x68 reserved)   LeakyReLU slope, alpha[15:0] / 65536
// outer / a_inc / b_inc / act reset to 0; outer=0 makes the kernel emit no
// writes, so the testbench MUST program these every transaction (the manifest
// may carry outer=1 a_inc=0 b_inc=0 act=0 — that still has to be written).
//
// Geometry (VectorOP.h): run o covers a[o*a_inc .. +size), b[o*b_inc .. +size)
// and c[o*(a_inc+b_inc) .. +size); every run start is 16-byte aligned (a_inc /
// b_inc are 0 or a multiple of 8 elements).  The kernel's 128-bit ports read
// whole words (lanes past `size` are masked) and write the last word of every
// run whole, so c positions in [size, ceil8(size)) of a run hold 0 and the
// positions up to the next run are untouched — the scoreboard compares only
// the valid positions.
// ============================================================================

module vectorop_tb;
    // -----------------------------------------------------------------------
    // DUT instantiation (block design wrapper has no external ports)
    // -----------------------------------------------------------------------
    design_vectorop dut();

    // PS VIP access path
    `define PS dut.zynq_ultra_ps_e_0.inst

    // -----------------------------------------------------------------------
    // Kernel AXI-Lite register map  (base 0xA000_0000)
    // -----------------------------------------------------------------------
    localparam [39:0] CTRL_BASE   = 40'hA000_0000;
    localparam [39:0] REG_AP_CTRL = CTRL_BASE + 40'h00;
    localparam [39:0] REG_GIE     = CTRL_BASE + 40'h04;
    localparam [39:0] REG_IER     = CTRL_BASE + 40'h08;
    localparam [39:0] REG_ISR     = CTRL_BASE + 40'h0C;
    localparam [39:0] REG_A_LO    = CTRL_BASE + 40'h10;  // a base addr [31:0]
    localparam [39:0] REG_A_HI    = CTRL_BASE + 40'h14;  // a base addr [63:32]
    localparam [39:0] REG_B_LO    = CTRL_BASE + 40'h1C;  // b base addr [31:0]
    localparam [39:0] REG_B_HI    = CTRL_BASE + 40'h20;  // b base addr [63:32]
    localparam [39:0] REG_C_LO    = CTRL_BASE + 40'h28;  // c base addr [31:0]
    localparam [39:0] REG_C_HI    = CTRL_BASE + 40'h2C;  // c base addr [63:32]
    localparam [39:0] REG_SIZE    = CTRL_BASE + 40'h34;  // vector length (elements)
    localparam [39:0] REG_OP      = CTRL_BASE + 40'h3C;  // operation selector (Op enum)
    localparam [39:0] REG_OUTER   = CTRL_BASE + 40'h44;  // outer-loop iteration count
    localparam [39:0] REG_A_INC   = CTRL_BASE + 40'h4C;  // A advance per outer iteration (elements)
    localparam [39:0] REG_B_INC   = CTRL_BASE + 40'h54;  // B advance per outer iteration (elements)
    localparam [39:0] REG_ACT     = CTRL_BASE + 40'h5C;  // fused activation (Act enum, 0 none)
    localparam [39:0] REG_ALPHA   = CTRL_BASE + 40'h64;  // LeakyReLU slope, alpha[15:0] / 65536

    // Op enum (must match include/VectorOP.h)
    localparam int unsigned OP_ADD   = 0;
    localparam int unsigned OP_SUB   = 1;
    localparam int unsigned OP_MUL   = 2;
    localparam int unsigned OP_DIV   = 3;
    localparam int unsigned OP_RELU  = 4;
    localparam int unsigned OP_RELU6 = 5;
    localparam int unsigned OP_LEAKY_RELU = 6;   // the activation ops (unary)
    localparam int unsigned OP_SILU       = 7;
    localparam int unsigned OP_GELU       = 8;
    localparam int unsigned OP_GELU_TANH  = 9;

    // -----------------------------------------------------------------------
    // Test parameters
    // -----------------------------------------------------------------------
    localparam int unsigned ELEM_BYTES = 2;    // sizeof(ap_fixed<16,8>)
    localparam integer      CHUNK_SIZE = 1024; // PS VIP transfer chunk (bytes)
    localparam integer      CHUNK_BITS = CHUNK_SIZE * 8;
    localparam int unsigned MEM_GAP   = 64 * 1024;  // guard gap between arrays

    localparam logic [15:0] C_POISON = 16'hDEAD;  // sentinel for un-written C elements

    // Interrupt output from the HLS kernel, monitored to detect deassert between tests.
    // wait_interrupt is level-sensitive: if this line is still high when the next
    // wait_interrupt is called it returns immediately with stale data.
    wire kernel_irq = dut.VectorOPKernel_0_interrupt;

    // Round n up to the next multiple of align (must be a power of two).
    //   align=4  — satisfies PS VIP write_mem base-address alignment requirement
    //   align=16 — covers 128-bit AXI beat so last burst read never hits X bytes
    function automatic int unsigned align_up(int unsigned n, int unsigned align);
        return (n + align - 1) & ~(align - 1);
    endfunction

    // Map an Op enum value to the human-readable label in the manifest.
    // Used when the manifest line is missing the trailing label token, so log
    // output stays informative even on partially-formed manifests.
    function automatic string op_to_name(int unsigned op);
        case (op)
            OP_ADD:   return "ADD";
            OP_SUB:   return "SUB";
            OP_MUL:   return "MUL";
            OP_DIV:   return "DIV";
            OP_RELU:  return "RELU";
            OP_RELU6: return "RELU6";
            OP_LEAKY_RELU: return "LEAKY_RELU";
            OP_SILU:       return "SILU";
            OP_GELU:       return "GELU";
            OP_GELU_TANH:  return "GELU_TANH";
            default:  return $sformatf("OP%0d", op);
        endcase
    endfunction

    // =========================================================================
    // Transaction — geometry from manifest, fixture data from .hex files
    // =========================================================================
    class vop_item;
        // Manifest-supplied identity (used for log lines + JSON report)
        int unsigned index;
        string       label;

        // Manifest geometry / op control — all programmed via AXI-Lite.
        int unsigned size;          // elements per run (outer iteration)
        int unsigned op;            // Op enum value
        int unsigned outer;         // outer-loop count
        int unsigned a_inc;         // per-outer-iter A advance (elements, 0 = repeat)
        int unsigned b_inc;         // per-outer-iter B advance (elements, 0 = repeat)
        int unsigned act;           // fused activation (Act enum, 0 none)
        int unsigned alpha;         // LeakyReLU slope (alpha[15:0] / 65536)
        bit          is_unary;      // 1 for op >= RELU — kernel skips B reads

        // Element extents (derived from the geometry, see the header): the
        // upstream pipeline emits .hex files of exactly these sizes.
        int unsigned a_count;
        int unsigned b_count;
        int unsigned c_count;
        int unsigned c_inc;

        // DDR base addresses.  16-byte alignment matches the 128-bit AXI bus
        // width so every base sits on byte lane 0 — without that the kernel's
        // narrow writes can land on the wrong lane and silently drop the
        // tail element.
        logic [39:0] addr_a;
        logic [39:0] addr_b;
        logic [39:0] addr_c;

        // Fixture data (loaded from <data_dir>/test_<NN>_{a,b,c}.hex).
        logic [15:0] a_data[];
        logic [15:0] b_data[];
        logic [15:0] c_ref [];

        function new(int unsigned   index_,
                     string         lbl,
                     int unsigned   size_,
                     int unsigned   op_,
                     int unsigned   outer_,
                     int unsigned   a_inc_,
                     int unsigned   b_inc_,
                     int unsigned   act_,
                     int unsigned   alpha_);
            int unsigned a_bytes, b_bytes, n_outer;

            this.index    = index_;
            this.label    = lbl;
            this.size     = size_;
            this.op       = op_;
            this.outer    = outer_;
            this.a_inc    = a_inc_;
            this.b_inc    = b_inc_;
            this.act      = act_;
            this.alpha    = alpha_;
            this.is_unary = (op_ >= OP_RELU);

            // Extents: (outer-1) * stride + size for each array.
            n_outer       = outer_ ? outer_ : 1;
            this.c_inc    = a_inc_ + b_inc_;
            this.a_count  = (n_outer - 1) * a_inc_ + size_;
            this.b_count  = (n_outer - 1) * b_inc_ + size_;
            this.c_count  = (n_outer - 1) * this.c_inc + size_;

            a_bytes = align_up(this.a_count * ELEM_BYTES, 16);
            b_bytes = align_up(this.b_count * ELEM_BYTES, 16);
            this.addr_a = 40'h1000_0000;
            this.addr_b = this.addr_a + 40'(a_bytes) + 40'(MEM_GAP);
            this.addr_c = this.addr_b + 40'(b_bytes) + 40'(MEM_GAP);
        endfunction

        // 1 when c position e is produced by the kernel (inside a run's
        // `size` elements); 0 for the alignment tail / stride gap.
        function bit c_valid(int unsigned e);
            if (outer <= 1 || c_inc == 0) return (e < size);
            return ((e % c_inc) < size);
        endfunction

        // Load A / B / C_ref from <dir>/test_<NN>_{a,b,c}.hex.  The upstream
        // pipeline emits b.hex even for unary ops (zero-filled), so always
        // read all three regardless of is_unary — the driver still skips
        // pushing B over AXI for unary ops to match what the kernel reads.
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
                "%-16s  op=%0d(%s)  size=%0d outer=%0d  a_inc=%0d b_inc=%0d act=%0d alpha=%0d  unary=%0d  |A|=%0d |B|=%0d |C|=%0d",
                label, op, op_to_name(op), size, outer,
                a_inc, b_inc, act, alpha, is_unary, a_count, b_count, c_count);
        endfunction
    endclass

    // =========================================================================
    // Per-test result record — captured by scoreboard, dumped to JSON at end.
    // Mirrors conv_tb / pooling_tb / matmul_tb so the test stand's run_tb.sh
    // wrapper can parse any kernel's report with the same logic.
    // =========================================================================
    class test_result;
        // Identity
        int unsigned index;
        string       label;
        int unsigned errors;
        int unsigned total_elements;

        // Geometry mirror (kept here so the JSON report is self-contained)
        int unsigned size;
        int unsigned op;
        int unsigned outer;
        int unsigned a_inc;
        int unsigned b_inc;
        int unsigned act;

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
    // Driver — fills DDR arrays, programs kernel registers, asserts ap_start
    // =========================================================================
    class driver;
        // Write a 32-bit value to an AXI-Lite register via HPM0_FPD.
        local task axil_write(input [39:0] addr, input [31:0] data);
            logic [1:0] rsp;
            `PS.write_data(addr, 4, {{(2048-32){1'b0}}, data}, rsp);
            if (rsp !== 2'b00)
                $error("[%0t][DRV] AXI-Lite write FAILED  addr=0x%010h  rsp=%0b",
                       $time, addr, rsp);
        endtask

        // Backdoor-fill a DDR region with a constant 16-bit value.
        // Chunk-based: handles arbitrarily large regions.
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

        task run(vop_item item);
            int unsigned c_bytes;
            $display("[%0t][DRV] %s", $time, item.to_string());

            // C poison region rounded up to 16-byte (128-bit AXI beat) alignment.
            c_bytes = align_up(item.c_count * ELEM_BYTES, 16);

            $display("[%0t][DRV] Loading A   (%0d elements) ...",
                     $time, item.a_count);
            write_data_ddr(item.addr_a, item.a_data, item.a_count);

            // Kernel issues no gmem1 AXI transactions for the unary ops (>= OP_RELU)
            // (op is loop-invariant in HLS), so skip the B push for unary ops.
            if (!item.is_unary) begin
                $display("[%0t][DRV] Loading B   (%0d elements) ...",
                         $time, item.b_count);
                write_data_ddr(item.addr_b, item.b_data, item.b_count);
            end else begin
                $display("[%0t][DRV] Skipping B push (unary op)", $time);
            end

            // Pre-poison C so any element the kernel skips fails the scoreboard
            $display("[%0t][DRV] Pre-filling C (%0d B) with 0x%04h ...",
                     $time, c_bytes, C_POISON);
            fill_const_ddr(item.addr_c, c_bytes, C_POISON);

            // Program kernel AXI-Lite registers.  outer / a_inc / b_inc
            // were added in the HLS revision that exposes outer-loop control;
            // they reset to 0 in the kernel, and outer=0 means the kernel
            // emits no writes — so they must be programmed every test even
            // when the manifest sets them all to defaults.
            $display("[%0t][DRV] Programming registers (op=%0d outer=%0d a_inc=%0d b_inc=%0d act=%0d alpha=%0d) ...",
                     $time, item.op, item.outer, item.a_inc, item.b_inc, item.act, item.alpha);
            axil_write(REG_A_LO,  item.addr_a[31:0]);
            axil_write(REG_A_HI,  {24'b0, item.addr_a[39:32]});
            axil_write(REG_B_LO,  item.addr_b[31:0]);
            axil_write(REG_B_HI,  {24'b0, item.addr_b[39:32]});
            axil_write(REG_C_LO,  item.addr_c[31:0]);
            axil_write(REG_C_HI,  {24'b0, item.addr_c[39:32]});
            axil_write(REG_SIZE,  32'(item.size));
            axil_write(REG_OP,    32'(item.op));
            axil_write(REG_OUTER, 32'(item.outer));
            axil_write(REG_A_INC, 32'(item.a_inc));
            axil_write(REG_B_INC, 32'(item.b_inc));
            axil_write(REG_ACT,   32'(item.act));
            axil_write(REG_ALPHA, 32'(item.alpha));

            // Enable ap_done interrupt and assert ap_start
            axil_write(REG_GIE,     32'h1);
            axil_write(REG_IER,     32'h1);
            $display("[%0t][DRV] Asserting ap_start ...", $time);
            axil_write(REG_AP_CTRL, 32'h1);
        endtask
    endclass

    // =========================================================================
    // Monitor — waits for kernel interrupt, services it
    // =========================================================================
    class monitor;
        local task axil_read(input [39:0] addr, output [31:0] data);
            logic [127:0] rd_raw;
            logic [1:0]   rsp;
            `PS.read_data(addr, 4, rd_raw, rsp);
            // The PS VIP read_data task returns data right-justified at rd_raw[31:0]
            // regardless of address offset; the byte-lane formula (addr[3:0]*8) shifts
            // past the valid data for registers at non-zero offsets (e.g. ISR at 0x0C
            // would read rd_raw[127:96] which is zero).
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

        task run(vop_item item);
            logic [15:0] irq_status;
            logic [31:0] isr_val, ap_ctrl_val;

            $display("[%0t][MON] Waiting for interrupt ...", $time);
            irq_status = 16'h0;
            fork
                begin : irq_wait
                    `PS.wait_interrupt(4'd0, irq_status);
                end
                begin : irq_timeout
                    #200_000_000;
                end
            join_any
            disable fork;

            if (!irq_status[0]) begin
                $error("[%0t][MON] TIMEOUT: no interrupt after 200 ms sim-time  test=%s",
                       $time, item.label);
                $finish;
            end
            $display("[%0t][MON] Interrupt received (irq_status=0x%04h)",
                     $time, irq_status);

            // Read back ap_ctrl to confirm ap_done
            axil_read(REG_AP_CTRL, ap_ctrl_val);
            $display("[%0t][MON] ap_ctrl=0x%08h  done=%0b  idle=%0b  ready=%0b",
                     $time, ap_ctrl_val, ap_ctrl_val[1], ap_ctrl_val[2], ap_ctrl_val[3]);

            // Service interrupt: read ISR, TOW-clear, disable GIE
            axil_read(REG_ISR, isr_val);
            $display("[%0t][MON] ISR=0x%08h  ap_done=%0b  ap_ready=%0b",
                     $time, isr_val, isr_val[0], isr_val[1]);
            axil_write(REG_ISR, isr_val);  // TOW: write set bits to clear
            axil_write(REG_GIE, 32'h0);

            // Wait for the interrupt line to physically deassert before returning.
            // wait_interrupt is level-sensitive: returning while kernel_irq is still
            // high would cause the next test's wait_interrupt to fire immediately on
            // the stale signal rather than waiting for the new kernel run to finish.
            if (kernel_irq) begin
                $display("[%0t][MON] Waiting for interrupt line to deassert ...", $time);
                @(negedge kernel_irq);
            end
            $display("[%0t][MON] Interrupt line low — ready for next test.", $time);
        endtask
    endclass

    // =========================================================================
    // Scoreboard — backdoor-reads C and checks every 16-bit element
    // =========================================================================
    class scoreboard;
        int unsigned total_tests = 0;
        int unsigned pass_cnt    = 0;
        int unsigned fail_cnt    = 0;
        test_result  results[$];

        task run(vop_item item, longint unsigned t_start_ns = 0);
            logic [CHUNK_BITS-1:0] chunk_buf;
            int unsigned           n_bytes, n_chunks, rem, errors, i, w, eidx, total_elems;
            logic [15:0]           elem, exp_elem;
            test_result            tr;

            total_elems = item.c_count;
            n_bytes     = total_elems * ELEM_BYTES;
            n_chunks    = n_bytes / CHUNK_SIZE;
            rem         = n_bytes % CHUNK_SIZE;
            errors      = 0;

            tr                = new();
            tr.index          = item.index;
            tr.label          = item.label;
            tr.size           = item.size;
            tr.op             = item.op;
            tr.outer          = item.outer;
            tr.a_inc          = item.a_inc;
            tr.b_inc          = item.b_inc;
            tr.act            = item.act;
            tr.total_elements = total_elems;

            $display("[%0t][SCB] Verifying C[0..%0d] (%0d elem × %0d B = %0d B) against c_ref ...",
                     $time, total_elems - 1, total_elems, ELEM_BYTES, n_bytes);

            for (i = 0; i < n_chunks; i++) begin
                `PS.read_mem(item.addr_c + 40'(i * CHUNK_SIZE), CHUNK_SIZE, chunk_buf);
                for (w = 0; w < CHUNK_SIZE / 2; w++) begin
                    eidx = i * (CHUNK_SIZE / 2) + w;
                    if (eidx >= total_elems) break;
                    if (!item.c_valid(eidx)) continue;   // alignment tail / stride gap
                    elem     = chunk_buf[w*16 +: 16];
                    exp_elem = item.c_ref[eidx];
                    if (elem !== exp_elem) begin
                        if (errors < 5)
                            $display("[%0t][SCB] MISMATCH C[%0d]: got=0x%04h  exp=0x%04h",
                                     $time, eidx, elem, exp_elem);
                        if (errors < MAX_MM) begin
                            tr.mm_idx.push_back(eidx);
                            tr.mm_got.push_back(elem);
                            tr.mm_exp.push_back(exp_elem);
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
                    if (!item.c_valid(eidx)) continue;   // alignment tail / stride gap
                    elem     = chunk_buf[w*16 +: 16];
                    exp_elem = item.c_ref[eidx];
                    if (elem !== exp_elem) begin
                        if (errors < 5)
                            $display("[%0t][SCB] MISMATCH C[%0d]: got=0x%04h  exp=0x%04h",
                                     $time, eidx, elem, exp_elem);
                        if (errors < MAX_MM) begin
                            tr.mm_idx.push_back(eidx);
                            tr.mm_got.push_back(elem);
                            tr.mm_exp.push_back(exp_elem);
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
                $display("[%0t][SCB] PASS  %-16s  op=%s size=%0d",
                         $time, item.label, op_to_name(item.op), item.size);
            end else begin
                fail_cnt++;
                $display("[%0t][SCB] FAIL  %-16s  %0d/%0d mismatches",
                         $time, item.label, errors, total_elems);
            end
        endtask

        task print_summary();
            $display("==========================================================");
            $display(" VectorOP Test Summary: %0d / %0d passed", pass_cnt, total_tests);
            if (fail_cnt == 0)
                $display(" ALL TESTS PASSED");
            else
                $display(" %0d TEST(S) FAILED", fail_cnt);
            $display("==========================================================");
        endtask

        // Emit a JSON report describing every test, its parameters, and the
        // first MAX_MM mismatches for any failing test.  Shape matches the
        // conv / pooling / matmul testbenches so run_tb.sh's parser handles
        // any kernel.
        task write_json_report(string path);
            int          fd;
            test_result  tr;
            fd = $fopen(path, "w");
            if (fd == 0) begin
                $error("[SCB] Failed to open JSON report file: %s", path);
                return;
            end

            $fdisplay(fd, "{");
            $fdisplay(fd, "  \"kernel\": \"VectorOPKernel\",");
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
                $fwrite(fd, "        \"size\": %0d, \"op\": %0d, \"op_name\": \"%s\",\n",
                            tr.size, tr.op, op_to_name(tr.op));
                $fwrite(fd, "        \"outer\": %0d, \"a_inc\": %0d, \"b_inc\": %0d, \"act\": %0d\n",
                            tr.outer, tr.a_inc, tr.b_inc, tr.act);
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
    // Environment — holds driver, monitor, scoreboard; runs one transaction
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

        // Run one test transaction end-to-end, strictly sequentially:
        //   1. driver  — fills DDR, programs registers, asserts ap_start
        //   2. monitor — waits for interrupt, services it, waits for line-low
        //   3. scb     — reads C from DDR and verifies every element
        // fork/join was replaced because monitor caught stale level-sensitive
        // interrupts from the previous test before the new kernel had started,
        // causing scb to read C while it still held the poison value.
        task run_one(vop_item item);
            longint unsigned t_start_ns;

            t_start_ns = $time;
            drv.run(item);
            mon.run(item);
            // --- diagnostic: direct DDR probe before scoreboard ---
            begin : ddr_probe
                logic [31:0] w0, w1, w_last;
                int unsigned widx, last_widx, total_elems;
                logic [15:0] exp_last;
                // word index = byte_addr >> 2; addr[28] selects ddr_mem0 vs ddr_mem1;
                // for our addresses (< 0x1000_0000 << 2 = 0x4000_0000) addr[28]=0 → ddr_mem0
                total_elems = item.c_count;
                widx        = item.addr_c[31:2];   // word of C[0]
                last_widx   = widx + ((total_elems - 1) * ELEM_BYTES) / 4;
                w0       = dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem0[widx];
                w1       = dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem0[widx+1];
                w_last   = dut.zynq_ultra_ps_e_0.inst.ddrc.ddr.ddr_mem0[last_widx];
                exp_last = item.c_ref[total_elems - 1];
                $display("[%0t][PROBE] ddr_mem0[0x%08h]=0x%08h  [0x%08h]=0x%08h",
                         $time, widx, w0, widx+1, w1);
                $display("[%0t][PROBE] C[%0d] word: ddr_mem0[0x%08h]=0x%08h  (expect 0x????%04h)",
                         $time, total_elems-1, last_widx, w_last, exp_last);
                // probe DDRC wires — hold last-driven values from arb_wr_6
                $display("[%0t][PROBE] DDRC last: wr_req=%0b wr_addr=0x%010h bytes=%0d strb[3:0]=0x%01h data[31:0]=0x%08h",
                         $time,
                         dut.zynq_ultra_ps_e_0.inst.ddrc.wr_req,
                         dut.zynq_ultra_ps_e_0.inst.ddrc.wr_addr,
                         dut.zynq_ultra_ps_e_0.inst.ddrc.wr_bytes,
                         dut.zynq_ultra_ps_e_0.inst.ddrc.wr_strb[3:0],
                         dut.zynq_ultra_ps_e_0.inst.ddrc.wr_data[31:0]);
            end
            scb.run(item, t_start_ns);
        endtask
    endclass

    // =========================================================================
    // Test — builds test matrix, runs all cases, prints summary
    // =========================================================================
    class test;
        env      e;
        vop_item tests[$];

        // Parse one manifest line — 8 ints + 1 trailing label token —
        // and create + load a vop_item.  Returns null if the line is
        // blank, a comment, or unparseable.  Manifest column layout
        // (matches the upstream vectorop reference dump):
        //
        //   idx size op outer a_inc b_inc act alpha  label
        //
        // Older manifests are accepted: 7 ints (no alpha column: alpha = 0)
        // and 6 ints (no act column either: act = 0).
        //
        // Per-test a / b / c arrays are loaded from
        // <data_dir>/test_<NN>_{a,b,c}.hex via $readmemh.
        function automatic bit is_number(string t);
            return t.len() > 0 && t.getc(0) >= "0" && t.getc(0) <= "9";
        endfunction

        function automatic vop_item parse_manifest_line(string line,
                                                         string data_dir);
            int unsigned idx;
            int unsigned size_, op_, outer_, a_inc_, b_inc_, act_, alpha_;
            string       label, tok7, tok8;
            int          rc;
            int          first;
            vop_item     it;

            // Skip leading whitespace; ignore blank or comment lines.
            first = 0;
            while (first < line.len() &&
                   (line.getc(first) == " "  || line.getc(first) == "\t" ||
                    line.getc(first) == "\n" || line.getc(first) == "\r"))
                first++;
            if (first == line.len()) return null;
            if (line.getc(first) == "#") return null;

            rc = $sscanf(line, "%d %d %d %d %d %d %s %s %s",
                idx, size_, op_, outer_, a_inc_, b_inc_, tok7, tok8, label);
            if (rc < 6) begin
                $display("[%0t][TEST] WARN: skipping unparseable manifest line: %s",
                         $time, line);
                return null;
            end
            // Tokens 7 and 8: the act and alpha columns when numeric, else
            // the label of an older manifest.
            act_   = 0;
            alpha_ = 0;
            if (rc >= 7 && !is_number(tok7)) begin
                label = tok7;
            end else if (rc >= 8 && !is_number(tok8)) begin
                act_  = tok7.atoi();
                label = tok8;
            end else begin
                if (rc >= 7) act_   = tok7.atoi();
                if (rc >= 8) alpha_ = tok8.atoi();
                if (rc < 9)  label  = op_to_name(op_);
            end

            it = new(idx, label, size_, op_, outer_, a_inc_, b_inc_, act_, alpha_);
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
            vop_item     it;

            if (!$value$plusargs("DATA_DIR=%s", data_dir))
                data_dir = "vecop_test_data";

            e = new();

            manifest_path = $sformatf("%s/manifest.txt", data_dir);
            fd = $fopen(manifest_path, "r");
            if (fd == 0) begin
                $error("[%0t][TEST] Cannot open manifest %s — generate the vectorop fixtures before running the testbench",
                       $time, manifest_path);
                $finish;
            end

            $display("==========================================================");
            $display(" VectorOPKernel Testbench   data dir = %s", data_dir);
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
                    report_path = "vectorop_test_report.json";
                e.scb.write_json_report(report_path);
            end
        endtask
    endclass

    // =========================================================================
    // DDRC write-request monitor — fires on every posedge of wr_req and logs
    // the exact parameters passed to ddr.write_mem.  This captures what DDRC
    // actually receives regardless of #0-delayed #0 prt_data/#0 prt_strb
    // assignments in arb_wr_6 (those settle in the inactive event region, after
    // the active clock edge where DDRC samples wr_req).
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
    // Workaround for arb_wr_6 VIP race condition
    //
    // Root cause: prt_req=1 fires in the active event region before the
    // #0-delayed prt_data/prt_strb and the sequential prt_addr/prt_bytes
    // assignments settle in the inactive event region.  DDRC fires at the
    // same posedge sw_clk and calls write_mem with stale burst parameters
    // from the previous burst, silently dropping narrow writes (WSTRB≠0xF).
    //
    // Fix: on every wr_req rising edge, delay 1 ns (all #0 inactive-region
    // events have drained by then), read the now-correct wr_addr/wr_data/
    // wr_bytes/wr_strb, and re-apply the write directly to ddr_mem0/ddr_mem1
    // with correct byte-lane masking.
    //
    // Indexing matches the VIP's set_data/get_data tasks:
    //   word_addr = byte_addr >> 2
    //   word_addr[28]==0 → ddr_mem0[word_addr[27:0]]
    //   word_addr[28]==1 → ddr_mem1[word_addr[27:0]]
    // =========================================================================
    initial begin : ddrc_wr_fix
        int unsigned nb, boff;
        logic [39:0] ba;
        logic [31:0] wa;
        logic [ 7:0] bd;
        logic [31:0] tmp_word;
        forever begin
            @(posedge dut.zynq_ultra_ps_e_0.inst.ddrc.wr_req);
            #1; // past all #0 inactive-region events; signals now settled
            nb = int'(dut.zynq_ultra_ps_e_0.inst.ddrc.wr_bytes);
            for (int b = 0; b < nb; b++) begin
                if (dut.zynq_ultra_ps_e_0.inst.ddrc.wr_strb[b]) begin
                    ba   = dut.zynq_ultra_ps_e_0.inst.ddrc.wr_addr + 40'(b);
                    wa   = ba[33:2];  // word_addr = byte_addr >> 2 (32-bit)
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
    // Testbench top — PS VIP reset sequence, then run all tests
    // =========================================================================
    test t;

    initial begin
        // PG338: drive reset before the first VIP clock edge.
        `PS.set_stop_on_error(1);
        `PS.set_debug_level_info(1);

        // Assert POR + sys reset first so the DDR model starts in a clean state.
        // por_srstb_reset drives both por_rst_n and sys_rst_n, which feed
        // rst_out_n — the master reset for all PS VIP internal logic including
        // the DDR arbiters.  fpga_soft_reset only controls PL fabric resets
        // (PL_RESETN0-3) and has NO effect on the DDR model.
        `PS.por_srstb_reset(1'b0);   // assert POR+sys reset → DDR model held in reset
        `PS.fpga_soft_reset(32'hF);  // assert PL fabric resets
        #500;
        `PS.por_srstb_reset(1'b1);   // deassert → DDR model comes out of reset cleanly
        #800;
        `PS.fpga_soft_reset(32'h0);  // deassert PL fabric resets → PL interconnect starts
        #900;

        // Use BEST_CASE (fixed 21-cycle) write-response latency on HPC0_FPD.
        // Default RANDOM_CASE can stall WREADY between bursts; BEST_CASE keeps
        // the write pipeline flowing without stalls.
        `PS.set_slave_profile("S_AXI_HPC0_FPD", 0);

        t = new();
        t.run();

        $finish;
    end
endmodule
