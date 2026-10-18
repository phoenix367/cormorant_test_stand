# Copyright 2026 GradeBuilder SL
# SPDX-License-Identifier: Apache-2.0

# shellcheck shell=bash
# ---------------------------------------------------------------------------
# lib.sh — shared helpers for the cormorant_test_stand build/sim scripts.
#
# Sourced by:  build_hw.sh, run_tb.sh, clean.sh
# ---------------------------------------------------------------------------

set -eu -o pipefail

# Repo root = parent of the scripts/ directory.
TS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS_DIR="$TS_ROOT/scripts"
TCL_DIR="$SCRIPTS_DIR/tcl"

# shellcheck source=/dev/null
. "$SCRIPTS_DIR/registry.sh"

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log()  { printf '[ts] %s\n' "$*"; }
warn() { printf '[ts] WARN: %s\n' "$*" >&2; }
die()  { printf '[ts] ERROR: %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# Vivado discovery
#
# Honour an explicit VIVADO env-var if set; otherwise pick up `vivado` from
# PATH; finally walk a small list of common Xilinx install roots and source
# the highest-numbered settings64.sh found.
# ---------------------------------------------------------------------------

ensure_vivado() {
    if [ "${_TS_VIVADO_READY:-0}" = "1" ]; then return 0; fi

    if [ -n "${VIVADO:-}" ] && [ -x "$VIVADO" ]; then
        export PATH="$(dirname -- "$VIVADO"):$PATH"
        _TS_VIVADO_READY=1
        return 0
    fi
    if command -v vivado >/dev/null 2>&1; then
        _TS_VIVADO_READY=1
        return 0
    fi

    local root v candidate latest=""
    for root in /mnt/data/xilinx /tools/Xilinx /opt/Xilinx; do
        [ -d "$root" ] || continue
        for v in 2025.2 2025.1 2024.2 2024.1 2023.2; do
            candidate="$root/$v/settings64.sh"
            [ -r "$candidate" ] || continue
            latest="$candidate"; break 2
        done
    done

    [ -n "$latest" ] || die "Vivado not found. Source settings64.sh, set VIVADO=/path/to/bin/vivado, or add vivado to PATH."
    log "Sourcing $latest"
    # shellcheck source=/dev/null
    . "$latest"
    command -v vivado >/dev/null 2>&1 \
        || die "Sourced $latest but vivado is still not on PATH."
    _TS_VIVADO_READY=1
}

# ---------------------------------------------------------------------------
# Kernel registry lookup
#
# kernel_field <name> <field>  — print one of:
#     project_dir | xpr | wrapper_top | tb_top
# Exits non-zero if <name> is not in the registry.
# ---------------------------------------------------------------------------

kernel_field() {
    local want="$1" field="$2"
    local row name pd xp wt tb
    for row in "${KERNELS_TABLE[@]}"; do
        IFS=':' read -r name pd xp wt tb <<<"$row"
        name="$(_kr_strip "$name")"
        if [ "$name" = "$want" ]; then
            case "$field" in
                project_dir) _kr_strip "$pd"; return 0 ;;
                xpr)         _kr_strip "$xp"; return 0 ;;
                wrapper_top) _kr_strip "$wt"; return 0 ;;
                tb_top)      _kr_strip "$tb"; return 0 ;;
                *) die "kernel_field: unknown field '$field'" ;;
            esac
        fi
    done
    die "Unknown kernel '$want'. Known: $(known_kernels | tr '\n' ' ')"
}

known_kernels() {
    local row name _rest
    for row in "${KERNELS_TABLE[@]}"; do
        IFS=':' read -r name _rest <<<"$row"
        _kr_strip "$name"
        printf '\n'
    done
}

# Resolve the absolute path to the .xpr for a kernel.
kernel_xpr_path() {
    local name="$1" pd xp
    pd="$(kernel_field "$name" project_dir)"
    xp="$(kernel_field "$name" xpr)"
    printf '%s/%s/%s' "$TS_ROOT" "$pd" "$xp"
}
