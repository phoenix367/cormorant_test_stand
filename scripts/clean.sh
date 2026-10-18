#!/usr/bin/env bash
# Copyright 2026 GradeBuilder SL
# SPDX-License-Identifier: Apache-2.0

# ---------------------------------------------------------------------------
# clean.sh — wipe Vivado-generated artefacts for a kernel project so the
#            next build/sim starts from a clean slate.
#
# Usage:
#   scripts/clean.sh <kernel>
#
# Removes the per-project caches that Vivado regenerates from the .xpr
# (runs, sim, gen, cache, ip_user_files, *.log, *.jou, *.str, *.pb, *.wdb,
# .Xil).  The .xpr, sources/, and bd/ directories are never touched.
#
# Test-data fixtures are owned by whatever pipeline produced them (the
# directory you pass via --data-dir / DATA_DIR_<kernel>) and are NOT this
# script's concern.
# ---------------------------------------------------------------------------

set -eu -o pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
. "$HERE/lib.sh"

usage() {
    cat <<EOF
Usage: $(basename "$0") <kernel>

Known kernels:
$(known_kernels | sed 's/^/  /')
EOF
    exit 2
}

[ $# -ge 1 ] || usage
case "$1" in -h|--help) usage ;; esac
kernel="$1"; shift

while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help) usage ;;
        *)         die "unknown argument '$1' (run with --help)" ;;
    esac
done

xpr="$(kernel_xpr_path "$kernel")"
[ -f "$xpr" ] || die "$xpr not found"

proj_dir="$(dirname -- "$xpr")"
proj_name="$(basename -- "$xpr" .xpr)"

log "kernel       : $kernel"
log "project_dir  : $proj_dir"

# Vivado scratch directories follow the pattern <proj>.{cache,gen,hw,ip_user_files,runs,sim}.
# Sources live under <proj>.srcs and must NEVER be removed.
for sub in cache gen hw ip_user_files runs sim; do
    target="$proj_dir/${proj_name}.${sub}"
    if [ -e "$target" ]; then
        log "  rm -rf $target"
        rm -rf -- "$target"
    fi
done

# Stray Vivado log/journal files, plus xsim simulation snapshots that landed
# in the project root.
shopt -s nullglob
for f in "$proj_dir"/*.log "$proj_dir"/*.jou "$proj_dir"/*.str \
         "$proj_dir"/*.pb  "$proj_dir"/*.wdb "$proj_dir"/.Xil; do
    [ -e "$f" ] || continue
    log "  rm -rf $f"
    rm -rf -- "$f"
done
shopt -u nullglob

log "clean OK"
