`timescale 1ns / 1ps
// ============================================================================
// ConvKernel HLS Testbench - OOP structure (mirrors matmul_tb.sv)
//
// Components:
//   conv_item  - transaction object (conv geometry, fill values, expected output)
//   driver     - loads DDR arrays, programs AXI-Lite registers, asserts ap_start
//   monitor    - waits for interrupt, services it
//   scoreboard - backdoor-reads y and verifies every element
//   env        - connects all components; runs one transaction
//   test       - builds test matrix, iterates, prints summary
//
// DUT: design_conv block design (no external ports).
//   Zynq UltraScale+ PS VIP: AXI-Lite master HPM0_FPD + DDR slave HPC0_FPD
//   Kernel: ConvKernel (ap_fixed<16,8>; gmem0/x, gmem1/weight, gmem2/bias, gmem3/y)
//
// ap_fixed<16,8> encoding:
//   real_value × 256 = raw int16
//   1.0=0x0100  2.0=0x0200  0.5=0x0080  4.0=0x0400  9.0=0x0900
//  -1.0=0xFF00  100.0=0x6400  -100.0=0x9C00
//   max=0x7FFF (+127.996)  min=0x8000 (-128.0)
//
// Reference arithmetic (constant-fill, no padding):
//
//   For each output element y[n,m,oh,ow]:
//     acc_raw32  = n_active × x_raw16 × w_raw16   (int64 accumulate)
//     if has_bias: acc_raw32 += b_raw16 × 256     (AccData_t conversion)
//     y_raw16    = clip(acc_raw32 >>> 8, -32768, 32767)   (AP_TRN, AP_SAT)
//
//   Standard conv:  n_active = in_ch × kh × kw
//   Depthwise conv: n_active = kh × kw  (each output channel processes
//                                         only its corresponding input channel)
//
//   Derivation: x and weight are ap_fixed<16,8>; products accumulate in
//   AccData_t = ap_fixed<32,16>.  Promoting ap_fixed<16,8> to ap_fixed<32,16>
//   scales raw by 256 (binary-point alignment).  The a×b product in
//   ap_fixed<32,16> units equals a_raw16 × b_raw16 (same derivation as matmul).
//   saturate_cast<ap_fixed<16,8>>(acc) = clip(acc_raw32 >>> 8, ±32768).
//
//   All test cases use pad_top=pad_left=0 so every output position sees exactly
//   n_active non-zero input elements → uniform expected value.
//
// Kernel AXI-Lite register layout (base 0xA000_0000).
//   Source: conv/build/kv260/conv_kv260/solution1/impl/misc/drivers/
//           ConvKernel_v1_0/src/xconvkernel_hw.h
//
//   0x00 ap_ctrl   0x04 gie    0x08 ier    0x0C isr
//   0x10 x_lo      0x14 x_hi   (0x18 reserved)
//   0x1C w_lo      0x20 w_hi   (0x24 reserved)
//   0x28 b_lo      0x2C b_hi   (0x30 reserved)
//   0x34 y_lo      0x38 y_hi   (0x3C reserved)
//   0x40 batch     (0x44 reserved)
//   0x48 in_ch     (0x4C reserved)
//   0x50 in_h      (0x54 reserved)
//   0x58 in_w      (0x5C reserved)
//   0x60 out_ch    (0x64 reserved)
//   0x68 out_h     (0x6C reserved)
//   0x70 out_w     (0x74 reserved)
//   0x78 kh        (0x7C reserved)
//   0x80 kw        (0x84 reserved)
//   0x88 stride_h  (0x8C reserved)
//   0x90 stride_w  (0x94 reserved)
//   0x98 dilation_h (0x9C reserved)
//   0xA0 dilation_w (0xA4 reserved)
//   0xA8 pad_top   (0xAC reserved)
//   0xB0 pad_left  (0xB4 reserved)
//   0xB8 has_bias  (0xBC reserved)
//   0xC0 is_depthwise (0xC4 reserved)
//
// Test matrix (standard conv):
//   1×1×1×1 k=1×1           - degenerate minimum
//   1×1×3×3 k=1×1           - spatial sweep, no reduction
//   1×1×5×5 k=3×3           - 9-tap spatial reduction
//   1×4×5×5 k=3×3           - IC=4 channel reduction (36 products)
//   1×1×5×5 k=3×3 OC=4      - 4 output channels, same expected
//   1×4×5×5 k=3×3 IC=4 OC=8 - IC and OC both > kTileIC/kTileM
//   1×1×7×7 k=3×3 stride=2  - strided convolution
//   1×1×7×7 k=3×3 dil=2     - dilation
//   2×1×5×5 k=3×3           - batch=2
//   sat+: x=100 w=1 IC=2 k=1×1 → acc=51200 → 0x7FFF
//   sat-: x=-100 w=1 IC=2 k=1×1 → acc=-51200 → 0x8000
//   bias: x=1 w=1 b=0.5 IC=1 k=1×1 OC=2 → 1.5=0x0180
//
// Test matrix (depthwise conv, is_depthwise=1):
//   dw 1×4×5×5 k=3×3              - basic depthwise, n_active=9 → 9.0
//   dw 1×4×5×5 k=3×3 bias         - depthwise+bias → 9.5=0x0980
//   dw 1×4×7×7 k=3×3 stride=2     - depthwise strided
//   dw 2×4×5×5 k=3×3 batch=2      - depthwise batched
//   dw 1×16×5×5 k=3×3             - OC=16 > kTileM; exercises m_tile wrapping
// ============================================================================

module conv_tb;
    // -----------------------------------------------------------------------
    // DUT instantiation.
    // -----------------------------------------------------------------------
    design_conv dut();

    // PS VIP access path.
    `define PS dut.zynq_ultra_ps_e_0.inst

    // -----------------------------------------------------------------------
    // Kernel AXI-Lite register map  (base 0xA000_0000)
    // -----------------------------------------------------------------------
    localparam [39:0] CTRL_BASE        = 40'hA000_0000;
    localparam [39:0] REG_AP_CTRL      = CTRL_BASE + 40'h00;
    localparam [39:0] REG_GIE          = CTRL_BASE + 40'h04;
    localparam [39:0] REG_IER          = CTRL_BASE + 40'h08;
    localparam [39:0] REG_ISR          = CTRL_BASE + 40'h0C;
    localparam [39:0] REG_X_LO         = CTRL_BASE + 40'h10;
    localparam [39:0] REG_X_HI         = CTRL_BASE + 40'h14;
    localparam [39:0] REG_W_LO         = CTRL_BASE + 40'h1C;
    localparam [39:0] REG_W_HI         = CTRL_BASE + 40'h20;
    localparam [39:0] REG_B_LO         = CTRL_BASE + 40'h28;
    localparam [39:0] REG_B_HI         = CTRL_BASE + 40'h2C;
    localparam [39:0] REG_Y_LO         = CTRL_BASE + 40'h34;
    localparam [39:0] REG_Y_HI         = CTRL_BASE + 40'h38;
    localparam [39:0] REG_BATCH        = CTRL_BASE + 40'h40;
    localparam [39:0] REG_IN_CH        = CTRL_BASE + 40'h48;
    localparam [39:0] REG_IN_H         = CTRL_BASE + 40'h50;
    localparam [39:0] REG_IN_W         = CTRL_BASE + 40'h58;
    localparam [39:0] REG_OUT_CH       = CTRL_BASE + 40'h60;
    localparam [39:0] REG_OUT_H        = CTRL_BASE + 40'h68;
    localparam [39:0] REG_OUT_W        = CTRL_BASE + 40'h70;
    localparam [39:0] REG_KH           = CTRL_BASE + 40'h78;
    localparam [39:0] REG_KW           = CTRL_BASE + 40'h80;
    localparam [39:0] REG_STRIDE_H     = CTRL_BASE + 40'h88;
    localparam [39:0] REG_STRIDE_W     = CTRL_BASE + 40'h90;
    localparam [39:0] REG_DIL_H        = CTRL_BASE + 40'h98;
    localparam [39:0] REG_DIL_W        = CTRL_BASE + 40'hA0;
    localparam [39:0] REG_PAD_TOP      = CTRL_BASE + 40'hA8;
    localparam [39:0] REG_PAD_LEFT     = CTRL_BASE + 40'hB0;
    localparam [39:0] REG_HAS_BIAS     = CTRL_BASE + 40'hB8;
    localparam [39:0] REG_IS_DEPTHWISE = CTRL_BASE + 40'hC0;

    // -----------------------------------------------------------------------
    // Testbench parameters
    // -----------------------------------------------------------------------
    localparam int unsigned ELEM_BYTES = 2;    // sizeof(ap_fixed<16,8>)
    localparam integer      CHUNK_SIZE = 1024; // PS VIP transfer chunk (bytes)
    localparam integer      CHUNK_BITS = CHUNK_SIZE * 8;
    localparam int unsigned MEM_GAP    = 64 * 1024; // guard gap between arrays (bytes)

    localparam logic [15:0] Y_POISON = 16'hDEAD; // sentinel for un-written y elements

    // Maximum number of mismatches recorded per test in the JSON report.
    localparam int MAX_MM = 16;

    // Kernel interrupt wire.
    wire kernel_irq = dut.ConvKernel_0_interrupt;

    // -----------------------------------------------------------------------
    // Helper: round n up to the next multiple of align.
    // -----------------------------------------------------------------------
    function automatic int unsigned align_up(int unsigned n, int unsigned align);
        return (n + align - 1) & ~(align - 1);
    endfunction

    // =========================================================================
    // Transaction — one ConvKernel run.
    //
    // Geometry comes from manifest.txt; per-tensor fixture data (x, weight,
    // bias, y_ref) is loaded from hex files generated by
    //   make gen_conv_test_data
    // which dumps everything via TestConvRef --dump-data.  No reference
    // formula in SV: the C++ ref_conv()/ref_depthwise_conv() oracle in
    // TestConvSim.cpp is the single source of truth.
    // =========================================================================
    class conv_item;
        // Geometry
        int unsigned index;
        int unsigned batch;
        int unsigned in_ch, in_h, in_w;
        int unsigned out_ch, out_h, out_w;
        int unsigned kh, kw;
        int unsigned stride_h, stride_w;
        int unsigned dilation_h, dilation_w;
        int unsigned pad_top, pad_left;
        int unsigned has_bias;
        int unsigned is_depthwise;

        // Element counts (derived).
        int unsigned x_count, w_count, b_count, y_count;

        // DDR base addresses.
        logic [39:0] addr_x;
        logic [39:0] addr_w;
        logic [39:0] addr_b;
        logic [39:0] addr_y;

        // Fixture data (loaded from hex files).
        logic [15:0] x_data[];
        logic [15:0] w_data[];
        logic [15:0] b_data[];
        logic [15:0] y_ref [];

        // Human-readable label.
        string label;

        function new(int unsigned    index_,
                     string          lbl,
                     int unsigned    batch_,
                     int unsigned    in_ch_,    int unsigned in_h_,  int unsigned in_w_,
                     int unsigned    out_ch_,   int unsigned out_h_, int unsigned out_w_,
                     int unsigned    kh_,       int unsigned kw_,
                     int unsigned    stride_h_,    int unsigned stride_w_,
                     int unsigned    dilation_h_,  int unsigned dilation_w_,
                     int unsigned    pad_top_,     int unsigned pad_left_,
                     int unsigned    has_bias_,
                     int unsigned    is_depthwise_);
            int unsigned x_bytes, w_bytes, b_bytes;

            this.index        = index_;
            this.label        = lbl;
            this.batch        = batch_;
            this.in_ch        = in_ch_;
            this.in_h         = in_h_;
            this.in_w         = in_w_;
            this.out_ch       = out_ch_;
            this.out_h        = out_h_;
            this.out_w        = out_w_;
            this.kh           = kh_;
            this.kw           = kw_;
            this.stride_h     = stride_h_;
            this.stride_w     = stride_w_;
            this.dilation_h   = dilation_h_;
            this.dilation_w   = dilation_w_;
            this.pad_top      = pad_top_;
            this.pad_left     = pad_left_;
            this.has_bias     = has_bias_;
            this.is_depthwise = is_depthwise_;

            // Element counts.  Weights and bias use ConvKernel's packed DDR
            // layout (ConvKernel.h "Weight / bias port width and DDR layout";
            // TestConvRef --dump-data writes the fixtures in it):
            //   standard : out_ch * ceil(in_ch/16) * kh * kw * 16   (16 ic lanes
            //              per kernel position, zero-padded last tile)
            //   depthwise: out_ch * roundup(kh*kw, 8)
            //   bias     : roundup(out_ch, 8)
            // The 16 / 8 are kTileIC and the 128-bit port's lanes per beat.
            this.x_count = batch_  * in_ch_  * in_h_  * in_w_;
            this.w_count = is_depthwise_
                         ? out_ch_ * (((kh_ * kw_) + 7) / 8) * 8
                         : out_ch_ * ((in_ch_ + 15) / 16) * kh_ * kw_ * 16;
            this.b_count = ((out_ch_ + 7) / 8) * 8;
            this.y_count = batch_  * out_ch_ * out_h_ * out_w_;

            // DDR layout: x | gap | weight | gap | bias | gap | y
            x_bytes = align_up(this.x_count * ELEM_BYTES, 16);
            w_bytes = align_up(this.w_count * ELEM_BYTES, 16);
            b_bytes = align_up(this.b_count * ELEM_BYTES, 16);

            this.addr_x = 40'h1000_0000;
            this.addr_w = this.addr_x + 40'(x_bytes) + 40'(MEM_GAP);
            this.addr_b = this.addr_w + 40'(w_bytes) + 40'(MEM_GAP);
            this.addr_y = this.addr_b + 40'(b_bytes) + 40'(MEM_GAP);
        endfunction

        // Load x/w/b/y_ref from <dir>/test_<NN>_*.hex.  Each file holds one
        // 16-bit raw value per line (output of TestConvRef --dump-data).
        function void load_fixture(string dir);
            string idx_str;
            x_data = new[x_count];
            w_data = new[w_count];
            b_data = new[b_count];
            y_ref  = new[y_count];
            idx_str = $sformatf("%02d", index);
            $readmemh($sformatf("%s/test_%s_x.hex", dir, idx_str), x_data);
            $readmemh($sformatf("%s/test_%s_w.hex", dir, idx_str), w_data);
            $readmemh($sformatf("%s/test_%s_b.hex", dir, idx_str), b_data);
            $readmemh($sformatf("%s/test_%s_y.hex", dir, idx_str), y_ref);
        endfunction

        function string to_string();
            return $sformatf(
                "%-46s  N=%0d IC=%0d IH=%0d IW=%0d  OC=%0d OH=%0d OW=%0d  k=%0dx%0d s=%0dx%0d d=%0dx%0d p=%0d,%0d  bias=%0d dw=%0d  |x|=%0d |w|=%0d |y|=%0d",
                label,
                batch, in_ch, in_h, in_w, out_ch, out_h, out_w,
                kh, kw, stride_h, stride_w, dilation_h, dilation_w,
                pad_top, pad_left, has_bias, is_depthwise,
                x_count, w_count, y_count);
        endfunction
    endclass

    // =========================================================================
    // Driver - pushes fixture data to DDR, programs registers, asserts ap_start
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
        local task fill_const_ddr(input [39:0]       base,
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

        task run(conv_item item);
            int unsigned y_bytes;

            $display("[%0t][DRV] %s", $time, item.to_string());

            y_bytes = item.y_count * ELEM_BYTES;

            $display("[%0t][DRV] Loading x   (%0d elements) ...",
                     $time, item.x_count);
            write_data_ddr(item.addr_x, item.x_data, item.x_count);

            $display("[%0t][DRV] Loading w   (%0d elements) ...",
                     $time, item.w_count);
            write_data_ddr(item.addr_w, item.w_data, item.w_count);

            $display("[%0t][DRV] Loading b   (%0d elements, has_bias=%0d) ...",
                     $time, item.b_count, item.has_bias);
            write_data_ddr(item.addr_b, item.b_data, item.b_count);

            // Poison y so any missed element is detected by the scoreboard.
            $display("[%0t][DRV] Pre-filling y (%0d B) with 0x%04h ...",
                     $time, align_up(y_bytes, 16), Y_POISON);
            fill_const_ddr(item.addr_y, align_up(y_bytes, 16), Y_POISON);

            // Program kernel AXI-Lite registers.
            $display("[%0t][DRV] Programming registers ...", $time);
            axil_write(REG_X_LO,          item.addr_x[31:0]);
            axil_write(REG_X_HI,          {24'b0, item.addr_x[39:32]});
            axil_write(REG_W_LO,          item.addr_w[31:0]);
            axil_write(REG_W_HI,          {24'b0, item.addr_w[39:32]});
            axil_write(REG_B_LO,          item.addr_b[31:0]);
            axil_write(REG_B_HI,          {24'b0, item.addr_b[39:32]});
            axil_write(REG_Y_LO,          item.addr_y[31:0]);
            axil_write(REG_Y_HI,          {24'b0, item.addr_y[39:32]});
            axil_write(REG_BATCH,         32'(item.batch));
            axil_write(REG_IN_CH,         32'(item.in_ch));
            axil_write(REG_IN_H,          32'(item.in_h));
            axil_write(REG_IN_W,          32'(item.in_w));
            axil_write(REG_OUT_CH,        32'(item.out_ch));
            axil_write(REG_OUT_H,         32'(item.out_h));
            axil_write(REG_OUT_W,         32'(item.out_w));
            axil_write(REG_KH,            32'(item.kh));
            axil_write(REG_KW,            32'(item.kw));
            axil_write(REG_STRIDE_H,      32'(item.stride_h));
            axil_write(REG_STRIDE_W,      32'(item.stride_w));
            axil_write(REG_DIL_H,         32'(item.dilation_h));
            axil_write(REG_DIL_W,         32'(item.dilation_w));
            axil_write(REG_PAD_TOP,       32'(item.pad_top));
            axil_write(REG_PAD_LEFT,      32'(item.pad_left));
            axil_write(REG_HAS_BIAS,      32'(item.has_bias));
            axil_write(REG_IS_DEPTHWISE,  32'(item.is_depthwise));

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

        task run(conv_item item);
            logic [15:0] irq_status;
            logic [31:0] isr_val, ap_ctrl_val;

            $display("[%0t][MON] Waiting for interrupt ...", $time);
            irq_status = 16'h0;
            fork
                begin : irq_wait
                    `PS.wait_interrupt(4'd0, irq_status);
                end
                begin : irq_timeout
                    // Conv is compute-intensive; allow up to 2 s sim-time.
                    // Increase if simulating large spatial dimensions.
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
    // Per-test result record - captured by scoreboard, dumped to JSON at end
    // =========================================================================
    class test_result;
        int unsigned index;
        string       label;
        int unsigned errors;
        int unsigned total_elements;
        // Geometry mirror (kept here so the JSON report is self-contained)
        int unsigned batch, in_ch, in_h, in_w;
        int unsigned out_ch, out_h, out_w;
        int unsigned kh, kw;
        int unsigned stride_h, stride_w;
        int unsigned dilation_h, dilation_w;
        int unsigned pad_top, pad_left;
        int unsigned has_bias, is_depthwise;
        // Per-test simulation timing — populated by env::run_one.  Units are
        // nanoseconds (matches the file-level `timescale 1ns/1ps).  start_ns
        // is sampled before the driver programs registers; end_ns after the
        // scoreboard finishes verifying.
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

    // =========================================================================
    // Scoreboard - reads y back, compares element-wise against item.y_ref
    // =========================================================================
    class scoreboard;
        int unsigned  total_tests = 0;
        int unsigned  pass_cnt    = 0;
        int unsigned  fail_cnt    = 0;
        test_result   results[$];

        task run(conv_item item, longint unsigned t_start_ns = 0);
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
            tr.in_ch               = item.in_ch;
            tr.in_h                = item.in_h;
            tr.in_w                = item.in_w;
            tr.out_ch              = item.out_ch;
            tr.out_h               = item.out_h;
            tr.out_w               = item.out_w;
            tr.kh                  = item.kh;
            tr.kw                  = item.kw;
            tr.stride_h            = item.stride_h;
            tr.stride_w            = item.stride_w;
            tr.dilation_h          = item.dilation_h;
            tr.dilation_w          = item.dilation_w;
            tr.pad_top             = item.pad_top;
            tr.pad_left            = item.pad_left;
            tr.has_bias            = item.has_bias;
            tr.is_depthwise        = item.is_depthwise;
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
                $display("[%0t][SCB] PASS  %-46s  N=%0d IC=%0d OH=%0d OW=%0d OC=%0d dw=%0d",
                         $time, item.label,
                         item.batch, item.in_ch, item.out_h, item.out_w, item.out_ch,
                         item.is_depthwise);
            end else begin
                fail_cnt++;
                $display("[%0t][SCB] FAIL  %-46s  %0d/%0d mismatches",
                         $time, item.label, errors, total_elems);
            end
        endtask

        task print_summary();
            $display("==========================================================");
            $display(" ConvKernel Test Summary: %0d / %0d passed",
                     pass_cnt, total_tests);
            if (fail_cnt == 0)
                $display(" ALL TESTS PASSED");
            else
                $display(" %0d TEST(S) FAILED", fail_cnt);
            $display("==========================================================");
        endtask

        // Emit a JSON report describing every test, its parameters, and the
        // first MAX_MM mismatches for any failing test.
        task write_json_report(string path);
            int          fd;
            test_result  tr;
            fd = $fopen(path, "w");
            if (fd == 0) begin
                $error("[SCB] Failed to open JSON report file: %s", path);
                return;
            end

            $fdisplay(fd, "{");
            $fdisplay(fd, "  \"kernel\": \"ConvKernel\",");
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
                $fwrite(fd, "        \"batch\": %0d, \"in_ch\": %0d, \"in_h\": %0d, \"in_w\": %0d,\n",
                            tr.batch, tr.in_ch, tr.in_h, tr.in_w);
                $fwrite(fd, "        \"out_ch\": %0d, \"out_h\": %0d, \"out_w\": %0d,\n",
                            tr.out_ch, tr.out_h, tr.out_w);
                $fwrite(fd, "        \"kh\": %0d, \"kw\": %0d, \"stride_h\": %0d, \"stride_w\": %0d,\n",
                            tr.kh, tr.kw, tr.stride_h, tr.stride_w);
                $fwrite(fd, "        \"dilation_h\": %0d, \"dilation_w\": %0d,\n",
                            tr.dilation_h, tr.dilation_w);
                $fwrite(fd, "        \"pad_top\": %0d, \"pad_left\": %0d,\n",
                            tr.pad_top, tr.pad_left);
                $fwrite(fd, "        \"has_bias\": %0d, \"is_depthwise\": %0d\n",
                            tr.has_bias, tr.is_depthwise);
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

        task run_one(conv_item item);
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
                total_elems  = item.y_count;
                widx_first   = item.addr_y[31:2];
                widx_last    = item.addr_y[31:2] +
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
        conv_item tests[$];

        // Parse one manifest line — 18 ints + 1 trailing label token —
        // and create + load a conv_item.  Returns null if the line is
        // blank / a comment / unparseable.
        function automatic conv_item parse_manifest_line(string line,
                                                          string data_dir);
            int unsigned idx;
            int unsigned batch, in_ch, in_h, in_w;
            int unsigned out_ch, out_h, out_w;
            int unsigned kh, kw;
            int unsigned stride_h, stride_w;
            int unsigned dil_h, dil_w;
            int unsigned pt, pl;
            int unsigned has_bias, is_dw;
            string label;
            int          rc;
            int          first;
            conv_item    it;

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
                idx, batch, in_ch, in_h, in_w,
                out_ch, out_h, out_w, kh, kw,
                stride_h, stride_w, dil_h, dil_w,
                pt, pl, has_bias, is_dw, label);
            if (rc < 18) begin
                $display("[%0t][TEST] WARN: skipping unparseable manifest line: %s",
                         $time, line);
                return null;
            end
            if (rc < 19) label = "(unlabelled)";

            it = new(idx, label,
                     batch, in_ch, in_h, in_w,
                     out_ch, out_h, out_w,
                     kh, kw, stride_h, stride_w, dil_h, dil_w,
                     pt, pl, has_bias, is_dw);
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
            conv_item    it;

            if (!$value$plusargs("DATA_DIR=%s", data_dir))
                data_dir = "conv_test_data";

            e = new();

            manifest_path = $sformatf("%s/manifest.txt", data_dir);
            fd = $fopen(manifest_path, "r");
            if (fd == 0) begin
                $error("[%0t][TEST] Cannot open manifest %s — generate it with `make gen_conv_test_data`",
                       $time, manifest_path);
                $finish;
            end

            $display("==========================================================");
            $display(" ConvKernel Testbench   data dir = %s", data_dir);
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
                    report_path = "conv_test_report.json";
                e.scb.write_json_report(report_path);
            end
        endtask
    endclass

    // =========================================================================
    // +VERBOSE — per-transaction tracing.  The DDRC write monitor and the
    // CK_AXI channel probes below print one line per AXI beat / DDR write;
    // on the large fixtures that is >500k lines and dominates wall-clock.
    // They are OFF unless the run passes -testplusarg VERBOSE (the test
    // stand's run_sim.tcl adds it when TS_VERBOSE=1).  PASS/FAIL, per-test
    // [TEST]/[SCB] lines and the JSON report are unaffected.
    // =========================================================================
    bit verbose;
    initial verbose = $test$plusargs("VERBOSE");

    // =========================================================================
    // DDRC write-request monitor - logs every write the PS VIP issues to DDR.
    // =========================================================================
    initial begin : ddrc_wr_monitor
        int unsigned wr_cnt;
        wr_cnt = 0;
        forever begin
            @(posedge dut.zynq_ultra_ps_e_0.inst.ddrc.wr_req);
            wr_cnt++;
            if (verbose)
            $display("[%0t][DDRC_WR#%0d] addr=0x%010h bytes=%0d strb[3:0]=0x%01h data[31:0]=0x%08h",
                     $time, wr_cnt,
                     dut.zynq_ultra_ps_e_0.inst.ddrc.wr_addr,
                     dut.zynq_ultra_ps_e_0.inst.ddrc.wr_bytes,
                     dut.zynq_ultra_ps_e_0.inst.ddrc.wr_strb[3:0],
                     dut.zynq_ultra_ps_e_0.inst.ddrc.wr_data[31:0]);
        end
    end

    // =========================================================================
    // Workaround for arb_wr_6 VIP race condition (same fix as matmul_tb).
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
    // Logic probes - ConvKernel FSM state + AXI handshake logging
    //
    // Hierarchy:
    //   dut.ConvKernel_0.inst.*                   - HLS kernel internals
    //   dut.ConvKernel_0_m_axi_gmem*_*            - AXI bus wires in design_conv
    //
    // ap_CS_fsm is 34-bit one-hot.  Key sub-pipeline instances:
    //   LOOP_143_6       - bias init (m1 loop over out_ch tile)
    //   LOOP_165/166/174 - patch load (ic_l × khi × kwi burst reads)
    //   LOOP_196/202     - weight load (m1 × r burst reads)
    //   LOOP_234_13      - accumulate (ri inner loop, II=1)
    //   LOOP_263_14      - output write (m1 scatter writes to y)
    // =========================================================================

    `define CK dut.ConvKernel_0.inst

    function automatic int unsigned fsm_decode(input [33:0] v);
        for (int i = 0; i < 34; i++) if (v[i]) return i + 1;
        return 0;
    endfunction

    // ---- Main FSM state-change logger -----------------------------------
    initial begin : ck_fsm_probe
        int prev_s, cur_s;
        prev_s = 0;
        wait (`CK.ap_rst_n === 1'b1);
        @(posedge dut.zynq_ultra_ps_e_0_pl_clk0);

//        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
//            #1;
//            cur_s = fsm_decode(`CK.ap_CS_fsm);
//            if (cur_s !== prev_s) begin
//                $display("[%0t][CK_FSM] %0d->%0d idle=%b done=%b | bias=%b/%b patch=%b/%b wload=%b/%b acc=%b/%b wr=%b/%b",
//                    $time, prev_s, cur_s,
//                    `CK.ap_idle, `CK.ap_done,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_143_6_fu_556_ap_start,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_143_6_fu_556_ap_idle,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_165_8_VITIS_LOOP_166_9_VITIS_LOOP_174_10_fu_580_ap_start,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_165_8_VITIS_LOOP_166_9_VITIS_LOOP_174_10_fu_580_ap_idle,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_196_11_VITIS_LOOP_202_12_fu_632_ap_start,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_196_11_VITIS_LOOP_202_12_fu_632_ap_idle,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_234_13_fu_663_ap_start,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_234_13_fu_663_ap_idle,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_263_14_fu_734_ap_start,
//                    `CK.grp_ConvKernel_Pipeline_VITIS_LOOP_263_14_fu_734_ap_idle);
//                prev_s = cur_s;
//            end
//        end
    end

    // ---- gmem0 AR-channel (x reads) -------------------------------------
    initial begin : ck_gmem0_ar_probe
        forever begin
            @(posedge dut.ConvKernel_0_m_axi_gmem0_ARVALID or
              posedge dut.ConvKernel_0_m_axi_gmem0_ARREADY);
            #1;
            if (verbose && (dut.ConvKernel_0_m_axi_gmem0_ARVALID | dut.ConvKernel_0_m_axi_gmem0_ARREADY))
                $display("[%0t][CK_AXI] gmem0 AR: ARVALID=%b ARREADY=%b  ADDR=%016h  LEN=%0d  SIZE=%0d",
                    $time,
                    dut.ConvKernel_0_m_axi_gmem0_ARVALID,
                    dut.ConvKernel_0_m_axi_gmem0_ARREADY,
                    dut.ConvKernel_0_m_axi_gmem0_ARADDR,
                    dut.ConvKernel_0_m_axi_gmem0_ARLEN,
                    dut.ConvKernel_0_m_axi_gmem0_ARSIZE);
        end
    end

    // ---- gmem0 R-channel (x read data beats) ----------------------------
    initial begin : ck_gmem0_r_probe
        int r_beat;
        r_beat = 0;
        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
            #1;
            if (verbose && dut.ConvKernel_0_m_axi_gmem0_RVALID & dut.ConvKernel_0_m_axi_gmem0_RREADY) begin
                $display("[%0t][CK_AXI] gmem0 R:  beat#%0d  RDATA=%08h  RLAST=%b  RRESP=%b",
                    $time, r_beat,
                    dut.ConvKernel_0_m_axi_gmem0_RDATA,
                    dut.ConvKernel_0_m_axi_gmem0_RLAST,
                    dut.ConvKernel_0_m_axi_gmem0_RRESP);
                r_beat++;
            end
        end
    end

    // ---- gmem3 AW-channel (y write addresses) ---------------------------
    initial begin : ck_gmem3_aw_probe
        forever begin
            @(posedge dut.ConvKernel_0_m_axi_gmem3_AWVALID or
              posedge dut.ConvKernel_0_m_axi_gmem3_AWREADY);
            #1;
            if (verbose && (dut.ConvKernel_0_m_axi_gmem3_AWVALID | dut.ConvKernel_0_m_axi_gmem3_AWREADY))
                $display("[%0t][CK_AXI] gmem3 AW: AWVALID=%b AWREADY=%b  ADDR=%016h  LEN=%0d",
                    $time,
                    dut.ConvKernel_0_m_axi_gmem3_AWVALID,
                    dut.ConvKernel_0_m_axi_gmem3_AWREADY,
                    dut.ConvKernel_0_m_axi_gmem3_AWADDR,
                    dut.ConvKernel_0_m_axi_gmem3_AWLEN);
        end
    end

    // ---- gmem3 W-channel (y write data beats) ---------------------------
    initial begin : ck_gmem3_w_probe
        int w_beat;
        w_beat = 0;
        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
            #1;
            if (verbose && dut.ConvKernel_0_m_axi_gmem3_WVALID & dut.ConvKernel_0_m_axi_gmem3_WREADY) begin
                $display("[%0t][CK_AXI] gmem3 W:  beat#%0d  WSTRB=%h  WLAST=%b",
                    $time, w_beat,
                    dut.ConvKernel_0_m_axi_gmem3_WSTRB,
                    dut.ConvKernel_0_m_axi_gmem3_WLAST);
                w_beat++;
            end
        end
    end

    // ---- acc_stream FIFO occupancy (consumer drain vs writer pop rate) ----
    // Internal probe, +VERBOSE only: one summary line per 1024 pushes.
    initial begin : ck_acc_stream_probe
        int pushes, pops;
        longint last_push;
        pushes = 0; pops = 0; last_push = 0;
        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
            #1;
            if (verbose) begin
                if (`CK.process_conv_kernel_tile_U0_acc_stream_write & `CK.acc_stream_full_n) begin
                    pushes++;
                    if ($time - last_push > 1000)
                        $display("[%0t][CK_ACC] push gap %0d ns before push #%0d (pops=%0d occupancy=%0d)",
                                 $time, $time - last_push, pushes, pops, `CK.acc_stream_num_data_valid);
                    last_push = $time;
                    if ((pushes & 1023) == 0)
                        $display("[%0t][CK_ACC] pushes=%0d pops=%0d occupancy=%0d",
                                 $time, pushes, pops, `CK.acc_stream_num_data_valid);
                end
                if (`CK.process_conv_kernel_tile_U0_ap_done)
                    $display("[%0t][CK_ACC] consumer ap_done (pushes=%0d)", $time, pushes);
                if (`CK.write_output_tile_U0_ap_done)
                    $display("[%0t][CK_ACC] writer ap_done (pops=%0d)", $time, pops);
                if (`CK.input_patch_producer_U0_ap_done)
                    $display("[%0t][CK_ACC] patch producer ap_done", $time);
                if (`CK.stream_load_weights_U0_ap_done)
                    $display("[%0t][CK_ACC] weight producer ap_done", $time);
                if (`CK.write_output_tile_U0_acc_stream_read & `CK.acc_stream_empty_n)
                    pops++;
            end
        end
    end

    // ---- gmem3 B-channel (y write response) -----------------------------
    initial begin : ck_gmem3_b_probe
        forever @(posedge dut.zynq_ultra_ps_e_0_pl_clk0) begin
            #1;
            if (verbose && dut.ConvKernel_0_m_axi_gmem3_BVALID & dut.ConvKernel_0_m_axi_gmem3_BREADY)
                $display("[%0t][CK_AXI] gmem3 B:  BRESP=%b  (write response ack)",
                    $time, dut.ConvKernel_0_m_axi_gmem3_BRESP);
        end
    end

    // =========================================================================
    // Testbench top - PS VIP reset sequence, then run all tests
    // =========================================================================
    test t;

    initial begin
        `PS.set_stop_on_error(1);
        `PS.set_debug_level_info(verbose);   // VIP info chatter only under +VERBOSE

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
