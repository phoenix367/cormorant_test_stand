# cormorant_test_stand

Vivado test harnesses for the four kernels of the
[**Cormorant** FPGA neural-network inference accelerator](https://github.com/GradeBuilderSL/cormorant) —
`ConvKernel`, `PoolingKernel`, `MatmulKernel`, and `VectorOPKernel`.
Each kernel sits in its own Vivado project under `kernels/<name>_test/` with
a Zynq-UltraScale+ PS VIP driving the AXI-Lite control port and DDR slave; a
SystemVerilog OOP testbench programs the registers, asserts `ap_start`, and
back-door verifies the output against per-test reference fixtures.

The repo's main job is to make those projects scriptable: build/synth/impl
runs and behavioural simulations are driven by the wrappers under `scripts/`,
and the top-level `Makefile` exposes uniform `make hw-<kernel>` /
`make tb-<kernel>` entry points generated from a single registry file.

Test fixtures (`manifest.txt` + per-test `.hex` files) are produced
**outside** this repo by whatever pipeline owns the kernel under test. You
point the testbench at them per-run with a `DATA_DIR` knob — each kernel can
have its own.

The kernel's HLS IP catalogue (the directory the .xpr's `ip_repo_paths`
references) is also external. Repoint it per-run with `--ip-repo` /
`IP_REPO_<k>=` so the test stand picks up freshly-rebuilt kernel sources
without editing the project file. Whether or not the override is supplied,
both `build_hw.sh` and `run_tb.sh` upgrade any locked IPs and regenerate the
BD wrapper before launching synth or simulation, so the test always reflects
the current kernel revision.  After the upgrade they put every kernel
instance's `C_M_AXI_*_DATA_WIDTH` back to the default of the IP now in the
catalogue (`ts_apply_ip_default_widths`): two IPs share the MatmulKernel VLNV
— the Vitis HLS export (gmem2 32 bits) and the parent repo's SystemVerilog
kernel (`kernels/matmul_rtl`, gmem2 128 bits) — and an upgrade keeps the
instance's old value.

## Related project

The kernel sources, ONNX → C inference scheduler, and reference test-fixture
generators all live in the parent
[**cormorant**](https://github.com/GradeBuilderSL/cormorant) repository.
This test stand consumes two of cormorant's outputs:

- **HLS IP catalogues** — produced by cormorant's `make synthesize_<k>_kv260`
  targets under `cormorant/build/kernels/<k>/kv260/` (MatmulKernel: the
  SystemVerilog IP, `make package_matmul_rtl`, at
  `cormorant/build/rtl_ip/MatmulKernel_ip/`). Pass the
  matching directory with `IP_REPO_<k>=` (see Quick start below).
- **Behavioural test fixtures** (`manifest.txt` + per-test `.hex` files) —
  produced by cormorant's `make gen_<k>_test_data` targets (into its build
  tree) and checked in under `cormorant/hw/test_data/<dir>/`:

  | `<k>`       | Fixture target          | Default fixture directory                      |
  |-------------|-------------------------|------------------------------------------------|
  | `conv`      | `gen_conv_test_data`    | `cormorant/hw/test_data/conv_test_data/`       |
  | `pooling`   | `gen_pool_test_data`    | `cormorant/hw/test_data/pool_test_data/`       |
  | `matmul_op` | `gen_matmul_test_data`  | `cormorant/hw/test_data/matmul_test_data/`     |
  | `vector_op` | `gen_vectorop_test_data`| `cormorant/hw/test_data/vecop_test_data/`      |

  Pass the directory with `DATA_DIR_<k>=`. Different layouts are fine — the
  testbench only needs `manifest.txt` plus the matching `test_NN_*.hex`
  files; nothing in this repo assumes the cormorant tree.

## Registered kernels

| `<k>` | Project                       | BD wrapper                | Testbench top  | Manifest cols              | Fixture files                      |
|-------|-------------------------------|---------------------------|----------------|----------------------------|------------------------------------|
| `conv`      | `kernels/conv_test/`      | `design_conv_wrapper`     | `conv_tb`      | 18 ints + label            | `test_NN_{x,w,b,y}.hex`            |
| `pooling`   | `kernels/pooling_test/`   | `design_pooling_wrapper`  | `pooling_tb`   | 18 ints + label            | `test_NN_{x,y}.hex`                |
| `matmul_op` | `kernels/matmul_op_test/` | `design_matmul_wrapper`   | `matmul_tb`    |  9 ints + label            | `test_NN_{a,b,c}.hex`              |
| `vector_op` | `kernels/vector_op_test/` | `design_vectorop_wrapper` | `vectorop_tb`  |  7 ints + label            | `test_NN_{a,b,c}.hex`              |

`make list` reflects this table at run time. The exact column layout per
manifest is documented in each testbench's `parse_manifest_line` function.

## Layout

```
cormorant_test_stand/
├── Makefile                       — entry points; rules generated from registry
├── kernels/
│   ├── conv_test/                 — Vivado project for ConvKernel
│   │   ├── conv_test.xpr
│   │   └── conv_test.srcs/
│   │       ├── sources_1/bd/design_conv/    — block design (PS VIP + IP)
│   │       └── sim_1/new/conv_tb.sv         — OOP testbench
│   ├── pooling_test/              — PoolingKernel  (mirror layout)
│   ├── matmul_op_test/            — MatmulKernel   (mirror layout)
│   └── vector_op_test/            — VectorOPKernel (mirror layout)
└── scripts/
    ├── registry.sh                — kernel table (single source of truth)
    ├── lib.sh                     — Vivado discovery + registry helpers
    ├── build_hw.sh                — synth + impl + bitstream wrapper
    ├── run_tb.sh                  — xsim wrapper
    ├── clean.sh                   — wipe Vivado scratch dirs
    └── tcl/
        ├── lib.tcl                — shared Tcl: ip-repo override + locked-IP / BD refresh
        ├── build_hw.tcl
        ├── run_sim.tcl            — open .xpr, launch xsim, run -all (batch by default)
        └── run_sim_batch.tcl      — no-wave xsim -tclbatch script used unless TS_WAVES=1
```

## Prerequisites

- **Vivado 2025.2** (or any 2023.2+; the discovery logic walks
  `/mnt/data/xilinx`, `/tools/Xilinx`, `/opt/Xilinx`). Either source
  `settings64.sh` first, set `VIVADO=/path/to/bin/vivado`, or just put
  `vivado` on `PATH` — the scripts handle all three.
- **A fixtures directory per kernel** containing `manifest.txt` and the
  `test_*.hex` files the testbench reads. Generated however the kernel's
  upstream pipeline generates them; not produced here.
- **An IP repository** holding the kernel's exported HLS IP catalogue. The
  path stored in the .xpr can drift between machines; use
  `IP_REPO_<k>=<dir>` to override at run time (see Quick start below).

## Quick start

```bash
# Hardware build (synth + impl + bitstream)
make hw-conv

# Hardware build, no bitstream, 8 parallel jobs
make hw-conv BIT=0 VIVADO_JOBS=8

# Behavioural xsim test — DATA_DIR is mandatory
make tb-conv DATA_DIR_conv=/path/to/conv/fixtures

# Single-kernel shorthand: bare DATA_DIR works too
make tb-conv DATA_DIR=/path/to/conv/fixtures

# Custom JSON report destination
make tb-conv DATA_DIR_conv=/path/to/conv/fix REPORT_conv=/tmp/conv.json

# Repoint the IP repo at a freshly-rebuilt kernel catalogue
make hw-conv IP_REPO_conv=/path/to/kernels/build
make tb-conv DATA_DIR_conv=/path/to/conv/fix IP_REPO_conv=/path/to/kernels/build

# Wipe Vivado scratch dirs (.cache .gen .runs .sim .hw .ip_user_files)
make clean-conv

# Run all registered kernels — each one needs its own DATA_DIR_<k>
make all-tb \
    DATA_DIR_conv=/path/to/conv/fix \
    DATA_DIR_pooling=/path/to/pool/fix \
    DATA_DIR_matmul_op=/path/to/matmul/fix \
    DATA_DIR_vector_op=/path/to/vecop/fix
```

The wrappers can also be invoked directly:

```bash
scripts/build_hw.sh conv --jobs 8 --no-bitstream --ip-repo /path/to/kernels/build
scripts/run_tb.sh   conv --data-dir /path/to/conv/fix --report /tmp/conv.json --ip-repo /path/to/kernels/build
scripts/clean.sh    conv
```

`make help` and `scripts/*.sh --help` print full option lists.

## Reading test results

The testbench writes a JSON report to
`kernels/<kernel>_test/<kernel>_test_report.json` (the project directory the
wrapper `cd`s into before invoking Vivado). Override with `--report` /
`REPORT_<k>=` / `REPORT=`. The fixtures directory is treated as read-only,
so the report deliberately lands next to the project, not next to the
inputs. `run_tb.sh` parses it at the end and prints a one-line summary:

```
[ts] kernel=ConvKernel  total=63  passed=63  failed=0  all_passed=True
```

The wrapper exits non-zero if `all_passed` is false, so it composes cleanly
in CI.

All four testbenches emit the **same JSON shape** so the wrapper's parser is
kernel-agnostic. Top-level keys:

| Key            | Meaning                                                                              |
|----------------|--------------------------------------------------------------------------------------|
| `kernel`       | One of `ConvKernel`, `PoolingKernel`, `MatmulKernel`, `VectorOPKernel`               |
| `data_type`    | Element type — `ap_fixed<16,8>` for the current builds                               |
| `sim_time_ns`  | Total simulation time at $finish, in nanoseconds                                     |
| `summary`      | `{ total, passed, failed, all_passed }`                                              |
| `tests[]`      | Per-test records (see below)                                                         |

Per-test record:

```json
{
  "index":         0,
  "label":         "MaxPool_2x2_stride2",
  "status":        "PASS",
  "geometry":      { ... kernel-specific ... },
  "total_elements": 64,
  "errors":         0,
  "start_ns":       123456,    // sampled before driver programs registers
  "end_ns":         234567,    // after scoreboard finishes verifying
  "duration_ns":    111111,    // end_ns − start_ns
  "mismatches_reported": 0,
  "mismatches": []             // up to MAX_MM = 16 per-element diffs on FAIL
}
```

`duration_ns` covers the full per-test simulation time (register
programming → `ap_start` → kernel work → interrupt → scoreboard back-door
verification), in ns.

## Adding a new kernel

The scripts are deliberately data-driven — adding a kernel is one row of
`scripts/registry.sh` plus a Vivado project under `kernels/<name>_test/`:

1. Create `kernels/<name>_test/<name>_test.xpr` containing a block design
   `design_<name>` with a Zynq UltraScale+ PS VIP and the kernel IP, plus a
   wrapper module `design_<name>_wrapper`.
2. Drop a SystemVerilog testbench at the project's `sim_1` fileset
   (`<name>_tb.sv` is the convention). The testbench must:
   - read its fixtures directory from `+DATA_DIR=<dir>` (a `manifest.txt`
     and per-test `.hex` files live there);
   - write its JSON scoreboard report to `+REPORT=<file>`;
   - emit the JSON shape documented above (per-test `start_ns` /
     `end_ns` / `duration_ns`, etc.).

   The `parse_manifest_line` and `write_json_report` helpers in any of the
   four existing testbenches are good copy-paste starting points.
3. Append one row to `KERNELS_TABLE` in `scripts/registry.sh` (five fields):
   ```bash
   "norm: kernels/norm_test: norm_test.xpr: design_norm_wrapper: norm_tb"
   ```
4. `make hw-norm`, `make tb-norm DATA_DIR_norm=<dir>`, and
   `make clean-norm` are now available — no Makefile or script edits
   needed.

See `scripts/registry.sh` for a description of each field and the
conventions the Tcl drivers assume.

### When the kernel grows new AXI-Lite registers

If a new HLS revision adds control registers (e.g. extra loop-bound
parameters), two things have to be in sync after the IP repo is updated:

1. **Testbench register map** — add the new offsets and an `axil_write` for
   each one in the driver's `run` task. Sourced from the kernel's HLS-
   generated `xkernel_hw.h`.
2. **BD address segment** — the address editor must allocate a range wide
   enough to cover the highest register offset. If the segment is sized for
   the old register layout, writes to the new offsets return OKAY on the
   AXI-Lite bus but never reach the IP, so the registers stay at their
   reset value (commonly 0) and the kernel silently does nothing. After
   `upgrade_ip` resizes the CTRL port, run `assign_bd_address -force` (or
   bump the range manually in the address editor) and re-validate the BD.

## Troubleshooting

**`[ts] ERROR: IP(s) still locked after upgrade_ip`** — the .xpr's stored
`ip_repo_paths` doesn't reach a catalogue that contains the kernel IP at the
version the .xci references. Re-run with `IP_REPO_<k>=/path/to/kernels/build`
(or `--ip-repo` for the bare wrapper). The error message prints the current
`ip_repo_paths` and the locked IP's VLNV to help locate the right directory.

**Kernel completes (`ap_done` fires) but the output buffer keeps the poison
value** — usually a runtime parameter the kernel reads is zero. Check that
the testbench programs every register in the new HLS-generated header (in
particular any new loop-bound register added in the latest revision), and
that the BD address segment for the kernel's CTRL port is wide enough to
cover it (see "When the kernel grows new AXI-Lite registers").

## Environment overrides

| Variable | Meaning |
|---|---|
| `VIVADO` | Pin a specific `vivado` binary (skips PATH/install search) |
| `VIVADO_JOBS` | Default parallelism for synth/impl (overridden by `--jobs`/`VIVADO_JOBS=`) |
| `DATA_DIR_<k>` | Fixtures directory for kernel `<k>` (required for `tb-<k>`) |
| `DATA_DIR` | Single-kernel shorthand for `DATA_DIR_<k>` |
| `REPORT_<k>` | JSON report path for kernel `<k>` (default `kernels/<k>_test/<k>_test_report.json`) |
| `REPORT` | Single-kernel shorthand for `REPORT_<k>` |
| `IP_REPO_<k>` | Override the HLS IP repository path stored in kernel `<k>`'s `.xpr` |
| `IP_REPO` | Single-kernel shorthand for `IP_REPO_<k>` |
| `TS_WAVES` | `1` restores Vivado's default sim tcl (`add_wave /` into a .wdb). Default: batch run with no waveform logging; same results, smaller .wdb, modestly faster on large fixtures (xsim is CPU-bound in the design itself) |
| `TS_VERBOSE` | `1` passes `+VERBOSE` to the testbench so its per-beat AXI / DDR monitors print (conv_tb; default off — they dominate the log) |

---

## Funding

[![dAIEDGE Project](https://img.shields.io/badge/dAIEDGE-Project-6A5ACD?style=for-the-badge)](https://daiedge.eu/)
[![EU Horizon Europe](https://img.shields.io/badge/Funded%20by-EU%20Horizon%20Europe-003399?style=for-the-badge&logo=europeanunion&logoColor=white)](https://research-and-innovation.ec.europa.eu/funding/funding-opportunities/funding-programmes-and-open-calls/horizon-europe_en)

This work was supported by the **[dAIEDGE Open Call Programme](https://daiedge.eu/)**, funded by the **[European Union's Horizon Europe research and innovation programme](https://research-and-innovation.ec.europa.eu/funding/funding-opportunities/funding-programmes-and-open-calls/horizon-europe_en)** under project number **#101120726**.

---

## License

Copyright 2026 GradeBuilder SL. Licensed under the
[Apache License, Version 2.0](LICENSE).
