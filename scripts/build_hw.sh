#!/usr/bin/env bash
# Copyright 2026 GradeBuilder SL
# SPDX-License-Identifier: Apache-2.0

# ---------------------------------------------------------------------------
# build_hw.sh — synthesise + implement (+ optionally write bitstream) for
#              a kernel listed in scripts/registry.sh.
#
# Usage:
#   scripts/build_hw.sh <kernel> [--no-bitstream] [--jobs N] [--ip-repo DIR]
#
# Examples:
#   scripts/build_hw.sh conv
#   scripts/build_hw.sh conv --no-bitstream
#   scripts/build_hw.sh conv --jobs 8
#   scripts/build_hw.sh conv --ip-repo /path/to/kernels/build
#
# --ip-repo points the project at the directory holding the kernel's HLS IP
# catalogue.  Whether or not it's supplied, the Tcl driver always upgrades
# any locked IPs (i.e. kernel sources rebuilt since the .xpr was last saved)
# before launching synthesis.
#
# Environment overrides:
#   VIVADO=/path/to/bin/vivado     pin a specific Vivado install
#   VIVADO_JOBS=8                  default parallelism (overridden by --jobs)
# ---------------------------------------------------------------------------

set -eu -o pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
. "$HERE/lib.sh"

usage() {
    cat <<EOF
Usage: $(basename "$0") <kernel> [--no-bitstream] [--jobs N] [--ip-repo DIR]

Known kernels:
$(known_kernels | sed 's/^/  /')
EOF
    exit 2
}

[ $# -ge 1 ] || usage
case "$1" in -h|--help) usage ;; esac
kernel="$1"; shift

bit="1"
jobs="${VIVADO_JOBS:-4}"
ip_repo=""

while [ $# -gt 0 ]; do
    case "$1" in
        --no-bitstream) bit="0"; shift ;;
        --jobs)         jobs="$2"; shift 2 ;;
        --jobs=*)       jobs="${1#--jobs=}"; shift ;;
        --ip-repo)      ip_repo="$2"; shift 2 ;;
        --ip-repo=*)    ip_repo="${1#--ip-repo=}"; shift ;;
        -h|--help)      usage ;;
        *)              die "unknown argument '$1' (run with --help)" ;;
    esac
done

if [ -n "$ip_repo" ]; then
    [ -d "$ip_repo" ] || die "--ip-repo '$ip_repo' is not a directory"
    ip_repo="$(cd -- "$ip_repo" && pwd)"
fi

xpr="$(kernel_xpr_path "$kernel")"
top="$(kernel_field "$kernel" wrapper_top)"
[ -f "$xpr" ] || die "$xpr not found"

ensure_vivado

log "kernel       : $kernel"
log "project      : $xpr"
log "top          : $top"
log "jobs         : $jobs"
log "write_bit    : $bit"
log "ip_repo      : ${ip_repo:-<from .xpr>}"

# Run Vivado in batch from the project's directory so $PPRDIR substitutions
# (used by the IP repo path inside the .xpr) resolve consistently.
cd -- "$(dirname -- "$xpr")"

vivado -mode batch -nojournal -nolog \
    -source "$TCL_DIR/build_hw.tcl" \
    -tclargs "$xpr" "$top" "$jobs" "$bit" "$ip_repo"

log "build_hw OK"
