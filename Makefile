# ---------------------------------------------------------------------------
# cormorant_test_stand — top-level entry points.
#
# Per-kernel rules are generated from KERNELS, which is read directly from
# scripts/registry.sh (the single source of truth). Adding a new kernel only
# requires appending a row there — the Makefile picks it up automatically.
#
# Each kernel keeps its own fixtures directory.  Tell the Makefile where it
# is with a per-kernel DATA_DIR_<k> variable (DATA_DIR is also accepted as a
# shorthand when you're running just one kernel and there is no ambiguity).
# REPORT_<k> / REPORT works the same way for the JSON scoreboard output, and
# IP_REPO_<k> / IP_REPO works the same way for the kernel's HLS IP repo
# directory (used to override the path stored in the .xpr).  Whether or not
# IP_REPO is set, the Tcl driver always upgrades any locked kernel IPs and
# regenerates the BD wrapper before synth/sim.
#
# Usage:
#   make help
#   make hw-conv                                  # synth + impl + bitstream
#   make hw-conv VIVADO_JOBS=8                    # parallelism
#   make hw-conv BIT=0                            # synth + impl only
#   make hw-conv IP_REPO_conv=/path/to/kernels    # repoint IP repo
#   make tb-conv DATA_DIR_conv=/tmp/conv_fix      # per-kernel fixtures (preferred)
#   make tb-conv DATA_DIR=/tmp/conv_fix           # shorthand for single kernel
#   make tb-conv DATA_DIR_conv=... REPORT_conv=/tmp/conv.json IP_REPO_conv=/path/to/kernels
#   make clean-conv                               # wipe Vivado scratch dirs
#   make all-hw                                   # build hardware for every kernel
#   make all-tb DATA_DIR_conv=... DATA_DIR_matmul_op=...
# ---------------------------------------------------------------------------

SHELL := /usr/bin/env bash
.SHELLFLAGS := -eu -o pipefail -c

ROOT       := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
SCRIPTS    := $(ROOT)/scripts
REGISTRY   := $(SCRIPTS)/registry.sh

# Extract the first colon-separated field (kernel name) from each
# uncommented row in KERNELS_TABLE.  The $$ escape passes a literal $ to
# the shell.  Done at parse-time so $(KERNELS) is available for $(foreach).
KERNELS := $(shell awk -F: '/^[[:space:]]*"[^#]/ { gsub(/[" \t]/,"",$$1); print $$1 }' $(REGISTRY))

# Hardware build knobs (apply uniformly across kernels).
VIVADO_JOBS ?=
BIT         ?= 1

# Single-kernel shorthand for fixtures / report / IP repo.  Per-kernel
# DATA_DIR_<k>, REPORT_<k>, IP_REPO_<k> take precedence when both are set.
DATA_DIR ?=
REPORT   ?=
IP_REPO  ?=

ifeq ($(BIT),0)
  _BIT_FLAG := --no-bitstream
else
  _BIT_FLAG :=
endif

ifneq ($(strip $(VIVADO_JOBS)),)
  _JOBS_FLAG := --jobs $(VIVADO_JOBS)
else
  _JOBS_FLAG :=
endif

.PHONY: help all-hw all-tb list

help:
	@echo "cormorant_test_stand — kernel build / sim driver"
	@echo
	@echo "Known kernels:"
	@for k in $(KERNELS); do echo "  $$k"; done
	@echo
	@echo "Per-kernel targets (substitute <k>):"
	@echo "  make hw-<k>                              synth + impl + bitstream"
	@echo "  make tb-<k> DATA_DIR_<k>=<dir>           behavioural xsim test"
	@echo "  make clean-<k>                           wipe Vivado runs/cache/sim"
	@echo
	@echo "Aggregate targets:"
	@echo "  make all-hw                              hardware build for every kernel"
	@echo "  make all-tb DATA_DIR_<k1>=... DATA_DIR_<k2>=...   simulation for every kernel"
	@echo
	@echo "Knobs:"
	@echo "  VIVADO_JOBS=N                            parallelism for synth/impl"
	@echo "  BIT=0                                    skip bitstream (synth + impl only)"
	@echo "  DATA_DIR_<k>=<dir>                       per-kernel fixtures directory (REQUIRED for tb-<k>)"
	@echo "  REPORT_<k>=<file>                        per-kernel JSON scoreboard path (default: kernels/<k>_test/<k>_test_report.json)"
	@echo "  IP_REPO_<k>=<dir>                        override per-kernel HLS IP repo path stored in the .xpr"
	@echo "  DATA_DIR=<dir>, REPORT=<file>, IP_REPO=<dir>   shorthands when running a single kernel"

list:
	@for k in $(KERNELS); do echo "$$k"; done

# ---------------------------------------------------------------------------
# Per-kernel rule template.  Expanded once per kernel in $(KERNELS) via
# $(eval $(call KERNEL_RULES,<name>)).  Each call instantiates three .PHONY
# rules and registers them as dependencies of the all-* aggregates.
#
# DATA_DIR / REPORT resolution: prefer the per-kernel variable (DATA_DIR_<k>),
# fall back to the bare DATA_DIR / REPORT for single-kernel convenience.  The
# tb-<k> rule errors out if no fixture directory is set.
# ---------------------------------------------------------------------------

define KERNEL_RULES
.PHONY: hw-$(1) tb-$(1) clean-$(1)

hw-$(1):
	@ip="$$(or $$(IP_REPO_$(1)),$$(IP_REPO))";                                  \
	ip_flag="";                                                                  \
	if [ -n "$$$$ip" ]; then ip_flag="--ip-repo $$$$ip"; fi;                     \
	$(SCRIPTS)/build_hw.sh $(1) $$(_BIT_FLAG) $$(_JOBS_FLAG) $$$$ip_flag

tb-$(1):
	@dir="$$(or $$(DATA_DIR_$(1)),$$(DATA_DIR))";                               \
	rep="$$(or $$(REPORT_$(1)),$$(REPORT))";                                    \
	ip="$$(or $$(IP_REPO_$(1)),$$(IP_REPO))";                                   \
	if [ -z "$$$$dir" ]; then                                                   \
	    echo "[ts] ERROR: tb-$(1) requires a fixtures directory."           >&2; \
	    echo "[ts]        set DATA_DIR_$(1)=<path> (or DATA_DIR=<path>)."   >&2; \
	    exit 2;                                                                  \
	fi;                                                                          \
	rep_flag="";                                                                 \
	if [ -n "$$$$rep" ]; then rep_flag="--report $$$$rep"; fi;                   \
	ip_flag="";                                                                  \
	if [ -n "$$$$ip" ]; then ip_flag="--ip-repo $$$$ip"; fi;                     \
	$(SCRIPTS)/run_tb.sh $(1) --data-dir "$$$$dir" $$$$rep_flag $$$$ip_flag

clean-$(1):
	@$(SCRIPTS)/clean.sh $(1)

all-hw:    hw-$(1)
all-tb:    tb-$(1)
endef

$(foreach k,$(KERNELS),$(eval $(call KERNEL_RULES,$(k))))
