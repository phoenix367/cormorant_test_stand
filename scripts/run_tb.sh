#!/usr/bin/env bash
# Copyright 2026 GradeBuilder SL
# SPDX-License-Identifier: Apache-2.0

# ---------------------------------------------------------------------------
# run_tb.sh — run the behavioural testbench for a kernel listed in
#             scripts/registry.sh.
#
# Usage:
#   scripts/run_tb.sh <kernel> --data-dir DIR [--report FILE] [--ip-repo DIR]
#
# Examples:
#   scripts/run_tb.sh conv --data-dir /home/me/conv_fixtures
#   scripts/run_tb.sh conv --data-dir /home/me/conv_fixtures --report /tmp/conv.json
#   scripts/run_tb.sh conv --data-dir /home/me/conv_fixtures --ip-repo /path/to/kernels/build
#
# --data-dir is REQUIRED.  The directory must contain manifest.txt plus the
# test_*.hex fixtures the testbench reads — produce them however your kernel
# pipeline produces them; this repo does not generate them.  Each kernel may
# point at a different directory.
#
# --ip-repo is optional.  When supplied it overrides the IP repository path
# stored in the .xpr.  Whether or not it's supplied, the Tcl driver always
# upgrades any locked IPs (i.e. kernel sources rebuilt since the project was
# last saved) before launching simulation.
#
# Environment overrides:
#   VIVADO=/path/to/bin/vivado   pin a specific Vivado install
# ---------------------------------------------------------------------------

set -eu -o pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
. "$HERE/lib.sh"

usage() {
    cat <<EOF
Usage: $(basename "$0") <kernel> --data-dir DIR [--report FILE] [--ip-repo DIR]

Known kernels:
$(known_kernels | sed 's/^/  /')
EOF
    exit 2
}

[ $# -ge 1 ] || usage
case "$1" in -h|--help) usage ;; esac
kernel="$1"; shift

data_dir=""
report=""
ip_repo=""

while [ $# -gt 0 ]; do
    case "$1" in
        --data-dir)   data_dir="$2"; shift 2 ;;
        --data-dir=*) data_dir="${1#--data-dir=}"; shift ;;
        --report)     report="$2"; shift 2 ;;
        --report=*)   report="${1#--report=}"; shift ;;
        --ip-repo)    ip_repo="$2"; shift 2 ;;
        --ip-repo=*)  ip_repo="${1#--ip-repo=}"; shift ;;
        -h|--help)    usage ;;
        *)            die "unknown argument '$1' (run with --help)" ;;
    esac
done

if [ -n "$ip_repo" ]; then
    [ -d "$ip_repo" ] || die "--ip-repo '$ip_repo' is not a directory"
    ip_repo="$(cd -- "$ip_repo" && pwd)"
fi

xpr="$(kernel_xpr_path "$kernel")"
tb_top="$(kernel_field "$kernel" tb_top)"
[ -f "$xpr" ] || die "$xpr not found"

[ -n "$data_dir" ] \
    || die "--data-dir is required (each kernel has its own fixtures directory)"
[ -d "$data_dir" ] \
    || die "--data-dir '$data_dir' is not a directory"

# Canonicalise the path so the testbench gets an absolute, symlink-resolved
# value via $value$plusargs.
data_dir="$(cd -- "$data_dir" && pwd)"

manifest="$data_dir/manifest.txt"
[ -f "$manifest" ] \
    || die "$manifest not found — generate the kernel's fixtures (manifest.txt + test_*.hex) before running the testbench"

# Default report path lives in the kernel's Vivado project directory (the
# wrapper cd's there before invoking Vivado, so this is the testbench's cwd).
# We deliberately do NOT write into $data_dir — that's the read-only fixture
# directory, and stomping on it surprises external pipelines that own it.
if [ -z "$report" ]; then
    report="$(dirname -- "$xpr")/${kernel}_test_report.json"
fi

ensure_vivado

log "kernel    : $kernel"
log "project   : $xpr"
log "tb_top    : $tb_top"
log "data_dir  : $data_dir"
log "report    : $report"
log "ip_repo   : ${ip_repo:-<from .xpr>}"

cd -- "$(dirname -- "$xpr")"

vivado -mode batch -nojournal -nolog \
    -source "$TCL_DIR/run_sim.tcl" \
    -tclargs "$xpr" "$tb_top" "$data_dir" "$report" "$ip_repo"

# Brief summary if the report is parseable.
if [ -s "$report" ] && command -v python3 >/dev/null 2>&1; then
    python3 - "$report" <<'PY' || true
import json, sys
try:
    with open(sys.argv[1]) as f:
        r = json.load(f)
    s = r.get("summary", {})
    total  = s.get("total", "?")
    passed = s.get("passed", "?")
    failed = s.get("failed", "?")
    ap     = s.get("all_passed", False)
    print(f"[ts] kernel={r.get('kernel','?')}  total={total}  passed={passed}  failed={failed}  all_passed={ap}")
    sys.exit(0 if ap else 1)
except Exception as e:
    print(f"[ts] could not parse report: {e}")
    sys.exit(0)
PY
fi
