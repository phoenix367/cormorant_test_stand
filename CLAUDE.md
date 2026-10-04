# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this repo is

`cormorant_test_stand` is a small set of Vivado projects and shell/Tcl drivers
that exercise the kernels. Each kernel under test lives in its own Vivado project —
`kernels/conv_test/`, `pooling_test/`, `matmul_op_test/`, `vector_op_test/`
— and is driven by an OOP SystemVerilog testbench that
programs registers through a Zynq UltraScale+ PS VIP and back-door reads the
output buffer from the simulated DDR slave.

The repo's surface area is intentionally thin: hardware builds and behavioural
simulations both run from one Makefile whose per-kernel rules are *generated*
from a single registry file. There is no per-kernel Makefile or script — when
a new kernel arrives, scaffolding it is a one-line registry edit plus a Vivado
project drop.

Test fixtures (`manifest.txt` + `test_*.hex`) live **outside** this repo and
are passed in per-run via a mandatory `DATA_DIR` knob. Each kernel can have
its own fixtures directory; this repo never auto-locates or generates them.

The kernel's HLS IP catalogue is also external. The `--ip-repo` /
`IP_REPO_<k>` knob overrides the path stored in the .xpr; whether or not
it's set, both `build_hw.sh` and `run_tb.sh` always run the shared
`ts_prepare_bd` proc — it `upgrade_ip`s any locked IPs and regenerates the
BD wrapper, so a kernel that was rebuilt outside the test stand is picked
up automatically on the next run.

## Layout overview

| Path | Purpose |
|---|---|
| `Makefile` | Top-level entry. Reads `KERNELS` from `scripts/registry.sh` and instantiates `hw-<k>`, `tb-<k>`, `clean-<k>` rules per kernel via `$(eval $(call KERNEL_RULES,<k>))`. Resolves fixtures via `DATA_DIR_<k>` (preferred) or bare `DATA_DIR` (single-kernel shorthand). |
| `scripts/registry.sh` | The single source of truth for kernel metadata. One colon-separated row per kernel: `name : project_dir : xpr : wrapper_top : tb_top`. |
| `scripts/lib.sh` | Sourced by every wrapper. Owns Vivado discovery (`ensure_vivado`) and registry lookup (`kernel_field`, `kernel_xpr_path`). |
| `scripts/build_hw.sh` + `scripts/tcl/build_hw.tcl` | Synthesis + implementation + bitstream. The shell wrapper handles flag parsing and Vivado discovery, the Tcl handles `open_project` / `ts_apply_ip_repo` / `ts_prepare_bd` / `reset_run` / `launch_runs`. |
| `scripts/run_tb.sh` + `scripts/tcl/run_sim.tcl` | xsim batch run. Requires `--data-dir`, validates the directory contains `manifest.txt`, applies the optional `--ip-repo` override, refreshes locked IPs + BD wrapper, then passes `+DATA_DIR=<abs>` and `+REPORT=<file>` plusargs to the testbench. Parses the JSON report and prints a PASS/FAIL line. |
| `scripts/tcl/lib.tcl` | Shared Tcl helpers: `ts_apply_ip_repo` (override `ip_repo_paths` + `update_ip_catalog -rebuild`), `ts_apply_ip_default_widths` (every kernel instance's `C_M_AXI_*_DATA_WIDTH` = the default of its IP, read from a temporary instance) and `ts_prepare_bd` (`upgrade_ip` locked IPs, the width reset, `generate_target all`, regenerate BD wrapper). Sourced by both Tcl drivers. |
| `scripts/clean.sh` | Removes `<proj>.{cache,gen,hw,ip_user_files,runs,sim}` plus stray logs. Does not touch fixtures (they're not this repo's concern). |
| `kernels/conv_test/` | Vivado project. `design_conv` block design has the PS VIP + ConvKernel IP; `conv_test.srcs/sim_1/new/conv_tb.sv` is the OOP testbench. |
| `kernels/{pooling,matmul_op,vector_op}_test/` | Same layout for PoolingKernel (`design_pooling`, `pooling_tb`), MatmulKernel (`design_matmul`, `matmul_tb`), VectorOPKernel (`design_vectorop`, `vectorop_tb`). |

## Common workflows

```bash
# Behavioural simulation — DATA_DIR is mandatory
make tb-conv DATA_DIR_conv=/path/to/conv/fixtures

# Synth + impl + bitstream
make hw-conv

# Iterate on RTL/IP
make clean-conv && make hw-conv

# Run every registered kernel (each kernel gets its own dir)
make all-tb DATA_DIR_conv=... DATA_DIR_matmul_op=...
make all-hw
```

`make help` lists the supported knobs (`VIVADO_JOBS`, `BIT`, `DATA_DIR_<k>`,
`REPORT_<k>`, plus the bare `DATA_DIR` / `REPORT` shorthands).

## Conventions to preserve

- **One Vivado project per kernel under `kernels/<name>_test/`.** The block
  design is `design_<name>`, the synthesis top is `design_<name>_wrapper`,
  and the testbench top is `<name>_tb`. The `run_sim.tcl` driver picks the
  sim top from the registry — do not rely on whatever Vivado last saved as
  `top` of `sim_1`.
- **Testbench plusargs.** The shell driver passes `+DATA_DIR=<dir>` and
  `+REPORT=<file>` via `xsim.simulate.xsim.more_options`. New testbenches
  must read both with `$value$plusargs("DATA_DIR=%s", ...)` /
  `("REPORT=%s", ...)`. `manifest.txt` lives in `DATA_DIR`.
- **Fixtures are an external input.** They are not produced, located, or
  cleaned by this repo. `run_tb.sh` requires `--data-dir`; do not add an
  auto-locate fallback or a "generate fixtures" step. If a kernel needs
  fixtures from a particular pipeline, document the pipeline; don't bake
  it in.
- **Per-kernel fixtures dirs.** Two kernels usually have two different
  directories. The Makefile reads `DATA_DIR_<k>` first, then falls back to
  the bare `DATA_DIR` only for single-kernel convenience. New rules should
  follow the same `$(or $(DATA_DIR_$(1)),$(DATA_DIR))` resolution pattern.
  `REPORT_<k>` / `REPORT` and `IP_REPO_<k>` / `IP_REPO` use the same idiom.
- **Always refresh locked IPs.** Kernel sources are rebuilt outside this
  repo, so the .xpr's cached XCI can drift behind the catalogue between
  runs. `ts_prepare_bd` runs unconditionally on every build/sim and calls
  `upgrade_ip` on any IP marked `IS_LOCKED == 1`. Never short-circuit it —
  the next run silently using a stale kernel is exactly the failure mode
  this exists to prevent. If `--ip-repo` is supplied, `ts_apply_ip_repo`
  swaps the catalogue path before the lock-check so you can repoint at a
  fresh artefact directory without editing the project file.
- **Instance widths follow the IP.** After the upgrade `ts_prepare_bd` puts
  every kernel instance's m_axi data widths back to the IP's defaults
  (`ts_apply_ip_default_widths`; a no-op when they agree).  It matters for
  MatmulKernel: the HLS export has a 32-bit gmem2, the parent repo's RTL
  kernel (`kernels/matmul_rtl`, same VLNV) a 128-bit one, and `upgrade_ip`
  keeps the instance's old value.
- **Vivado batch invocations.** `vivado -mode batch -nojournal -nolog
  -source <tcl> -tclargs ...`. Don't add `-notrace`, `-stack`, etc. without
  a reason — the wrappers stay simple on purpose.
- **Per-project working directory.** Both shell wrappers `cd` to
  `$(dirname xpr)` before invoking Vivado so any `$PPRDIR`-based paths inside the
  `.xpr` (notably the IP repo path the project references) resolve the way
  Vivado saved them.

## Adding a new kernel

The scripts are designed so adding a kernel never requires editing them.
The full procedure:

1. Create `kernels/<name>_test/` containing the Vivado project. Block design
   `design_<name>`, wrapper `design_<name>_wrapper`, and testbench
   `<name>_tb` (reading `+DATA_DIR` / `+REPORT` plusargs as above).
2. Append one row to `scripts/registry.sh` `KERNELS_TABLE`. Field order
   (colon-separated): `name : project_dir : xpr : wrapper_top : tb_top`.
   The header comment in `registry.sh` documents each field.
3. `make hw-<name>`, `make tb-<name> DATA_DIR_<name>=<dir>`, and
   `make clean-<name>` are now active. `make list` reflects the new entry;
   `make help` regenerates its kernel list from the registry.

## Things to avoid

- Don't add per-kernel scripts. The wrappers + registry are the only API.
- Don't move fixtures into this repo. They're external inputs by design.
- Don't reintroduce an `axi_demo` (or any other) auto-locate fallback for
  fixtures. `--data-dir` / `DATA_DIR_<k>` is mandatory by design — different
  kernels can pull from entirely different pipelines.
- Don't hard-code Vivado install paths or IP repo paths in new Tcl. Use
  `ensure_vivado` and the `$PPRDIR` substitutions Vivado writes into the
  `.xpr` automatically.
- Don't run `vivado -mode gui` from any wrapper. All targets are batch mode.
- Don't `git rm` a kernel's `<proj>.runs/` or `.cache/` directories under
  the assumption they're scratch — they're already gitignored, but a
  selective `git add` on a kernel project should never sweep them in.

## Reference: registry row anatomy

```
"conv  : kernels/conv_test : conv_test.xpr : design_conv_wrapper : conv_tb"
 ^name   ^project_dir       ^xpr            ^wrapper_top          ^tb_top
```

`kernel_field <name> <field>` in `lib.sh` is the only consumer of these
columns; if you ever need to add a new field, that function and
`registry.sh`'s header comment are the two places to update.
