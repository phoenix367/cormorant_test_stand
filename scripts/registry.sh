# Copyright 2026 GradeBuilder SL
# SPDX-License-Identifier: Apache-2.0

# shellcheck shell=bash
# ---------------------------------------------------------------------------
# Kernel registry — one row per kernel under test.
#
# Adding a new kernel:
#   1. Create the Vivado project at kernels/<name>_test/ with a block design
#      named design_<name> and a wrapper module design_<name>_wrapper.
#   2. Drop a SystemVerilog testbench in the project's sim_1 fileset that
#      reads its fixtures from a directory passed via +DATA_DIR=<path> and
#      writes a JSON scoreboard report to +REPORT=<file>.
#   3. Append one row to KERNELS_TABLE below — five colon-separated fields.
#
# Each kernel brings its own fixtures; this repo does not produce them. Pass
# the per-kernel directory with `--data-dir` (run_tb.sh) or `DATA_DIR_<k>=`
# (Makefile).
#
# Field reference:
#   project_dir   path under kernels/  (relative to repo root)
#   xpr           .xpr filename inside the project_dir
#   wrapper_top   synthesis top-level for the impl run
#   tb_top        testbench top-level for the sim run
# ---------------------------------------------------------------------------

KERNELS_TABLE=(
    # name      : project_dir              : xpr                  : wrapper_top             : tb_top
    "conv       : kernels/conv_test        : conv_test.xpr        : design_conv_wrapper     : conv_tb"
    "pooling    : kernels/pooling_test     : pooling_test.xpr     : design_pooling_wrapper  : pooling_tb"
    "matmul_op  : kernels/matmul_op_test   : matmul_op_test.xpr   : design_matmul_wrapper   : matmul_tb"
    "vector_op  : kernels/vector_op_test   : vector_op_test.xpr   : design_vectorop_wrapper : vectorop_tb"
)

# Whitespace-strip helper used by lib.sh::kernel_field.
_kr_strip() { local s="$1"; s="${s#"${s%%[![:space:]]*}"}"; s="${s%"${s##*[![:space:]]}"}"; printf '%s' "$s"; }
