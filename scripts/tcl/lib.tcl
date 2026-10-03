# ---------------------------------------------------------------------------
# lib.tcl — shared Tcl helpers for the cormorant_test_stand build/sim scripts.
#
# Sourced by:  build_hw.tcl, run_sim.tcl
#
# Both helpers must run AFTER open_project — they operate on [current_project]
# and the source filesets attached to it.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# ts_apply_ip_repo — override the project's IP repository path and rebuild
# the catalog so newly-rebuilt kernel IPs are picked up.
#
# Pass an empty string to leave the path stored in the .xpr untouched.  The
# Vivado open_project still re-stats the existing path and locks IPs whose
# version differs, but this proc forces a full rescan when the caller wants
# to swap repos at run time (e.g. CI pointing at a different artefact dir).
# ---------------------------------------------------------------------------
proc ts_apply_ip_repo {ip_repo} {
    if {$ip_repo eq ""} {
        return
    }
    set ip_repo [file normalize $ip_repo]
    if {![file isdirectory $ip_repo]} {
        error "ts_apply_ip_repo: -ip-repo directory not found: $ip_repo"
    }
    puts "\[ts\] Setting IP repository: $ip_repo"
    set_property ip_repo_paths [list $ip_repo] [current_project]
    update_ip_catalog -rebuild
    puts "\[ts\] IP catalog updated"
}

# ---------------------------------------------------------------------------
# ts_apply_ip_default_widths — every C_M_AXI_*_DATA_WIDTH of every
# xilinx.com:hls:* cell back to the default of the IP now in the catalog.
#
# A kernel instance's m_axi widths must equal its IP's defaults, and two IPs
# share the MatmulKernel VLNV: the Vitis HLS export has a 32-bit gmem2, the
# RTL kernel (axi_demo kernels/matmul_rtl) a 128-bit one.  An upgrade from
# one to the other keeps the instance's old value; this puts it back to the
# new IP's default (a no-op when they agree).  Vivado has no reset to
# default (reset_property refuses CONFIG.*, VALUE_SRC DEFAULT keeps the
# value), so the defaults are read from a temporary instance of each IP.
# ---------------------------------------------------------------------------
proc ts_ip_default_widths {vlnv} {
    set probe [create_bd_cell -type ip -vlnv $vlnv ip_default_probe]
    set widths {}
    foreach p [list_property $probe -regexp {^CONFIG\.C_M_AXI_\w+_DATA_WIDTH$}] {
        dict set widths $p [get_property $p $probe]
    }
    delete_bd_objs $probe
    return $widths
}

proc ts_apply_ip_default_widths {bd_file} {
    open_bd_design $bd_file
    set defaults {}
    set changed 0
    foreach cell [get_bd_cells -quiet -filter {VLNV =~ "xilinx.com:hls:*"}] {
        set vlnv [get_property VLNV $cell]
        if {![dict exists $defaults $vlnv]} {
            dict set defaults $vlnv [ts_ip_default_widths $vlnv]
        }
        dict for {p want} [dict get $defaults $vlnv] {
            set have [get_property $p $cell]
            if {$have eq $want} {
                continue
            }
            set_property $p $want $cell
            set have [get_property $p $cell]
            if {$have ne $want} {
                error "ts_apply_ip_default_widths: $cell $p stays $have, the IP default is $want"
            }
            puts "\[ts\] $cell: [string range $p 7 end] -> $want (the IP default)"
            incr changed
        }
    }
    puts "\[ts\] m_axi widths: [dict size $defaults] kernel IP(s) checked, $changed instance parameter(s) reset"
    if {$changed > 0} {
        validate_bd_design
    }
    save_bd_design
}

# ---------------------------------------------------------------------------
# ts_prepare_bd — make sure the block design's HDL targets and wrapper are
# in sync with the IP catalog before synth/sim runs.  This is the place
# where stale kernel IPs are detected and upgraded.
#
#   1. Any IP whose stored XCI is older than the catalog is reported as
#      LOCKED — running upgrade_ip rebuilds the XCI from the new source;
#      then the kernel instances' m_axi widths are put back to the IPs'
#      defaults (ts_apply_ip_default_widths).
#   2. generate_target rebuilds the BD's HDL output (.gen/<bd>/...).  Safe
#      to run on an already-up-to-date tree (it's a no-op then).
#   3. make_wrapper rewrites design_<k>_wrapper.v at the project's current
#      path.  Required on a clean checkout where *.gen/ is absent and the
#      fileset's stored wrapper path is stale.
#
# The caller passes the wrapper-top module name from the registry so we can
# also (re)set [current_fileset] top — keeps `set_property top` consistent
# whatever the .xpr last saved.
# ---------------------------------------------------------------------------
proc ts_prepare_bd {wrapper_top} {
    set locked [get_ips -quiet -filter {IS_LOCKED == 1}]
    if {[llength $locked] > 0} {
        set names {}
        foreach ip $locked { lappend names [get_property NAME $ip] }
        puts "\[ts\] Upgrading [llength $locked] locked IP(s): [join $names {, }]"
        upgrade_ip $locked

        # Re-query: IPs whose VLNV cannot be resolved in any visible
        # ip_repo_paths entry stay locked even after upgrade_ip — Vivado
        # has nothing to upgrade *to*.  generate_target / make_wrapper
        # would then fail with a useless "BD is locked" message; bail with
        # an actionable error pointing at --ip-repo instead.
        set still_locked [get_ips -quiet -filter {IS_LOCKED == 1}]
        if {[llength $still_locked] > 0} {
            puts "\[ts\] ERROR: IP(s) still locked after upgrade_ip:"
            foreach ip $still_locked {
                set nm   [get_property NAME  $ip]
                set vlnv [get_property IPDEF $ip]
                # LOCK_STATUS is the canonical "why locked" property in
                # 2024.x+, but fall back gracefully on older releases.
                set reason ""
                if {[catch {get_property LOCK_STATUS $ip} val] == 0} {
                    set reason $val
                } elseif {[catch {get_property LOCK_REASON $ip} val] == 0} {
                    set reason $val
                } else {
                    set reason "(no LOCK_STATUS / LOCK_REASON property)"
                }
                puts "\[ts\]   - $nm    vlnv=$vlnv    reason=$reason"
            }
            set repos [get_property ip_repo_paths [current_project]]
            if {[llength $repos] == 0} {
                puts "\[ts\] Project ip_repo_paths is EMPTY."
            } else {
                puts "\[ts\] Current ip_repo_paths:"
                foreach r $repos { puts "\[ts\]   - $r" }
            }
            puts "\[ts\]"
            puts "\[ts\] This usually means the kernel's HLS IP catalogue is missing from the"
            puts "\[ts\] paths above (or the version stored in the .xci is unreachable).  Re-run"
            puts "\[ts\] with --ip-repo / IP_REPO_<kernel>=<dir> pointing at the directory that"
            puts "\[ts\] contains the kernel's exported IP, e.g. for the pooling kernel:"
            puts "\[ts\]"
            puts "\[ts\]   make tb-pooling DATA_DIR_pooling=<...> \\"
            puts "\[ts\]                   IP_REPO_pooling=/path/to/axi_demo/build/kernels/pooling"
            puts "\[ts\]"
            error "ts_prepare_bd: locked IPs prevent BD wrapper generation (see above)"
        }
        puts "\[ts\] IP upgrade complete"
    } else {
        puts "\[ts\] No locked IPs"
    }

    set bd_files [get_files -of_objects [get_filesets sources_1] \
                      -filter {FILE_TYPE == "Block Designs"}]
    if {[llength $bd_files] == 0} {
        puts "\[ts\] No block design in sources_1 — skipping BD wrapper regen"
        return
    }
    set bd_file [lindex $bd_files 0]
    ts_apply_ip_default_widths $bd_file
    puts "\[ts\] Generating BD targets: [file tail $bd_file]"
    generate_target all $bd_file

    puts "\[ts\] Creating BD wrapper"
    set wrapper [make_wrapper -files $bd_file -top]
    if {[llength [get_files -quiet $wrapper]] == 0} {
        add_files -norecurse $wrapper
        puts "\[ts\] Wrapper added: [file tail $wrapper]"
    } else {
        puts "\[ts\] Wrapper already registered at current path"
    }

    if {$wrapper_top ne ""} {
        set_property top $wrapper_top [current_fileset]
    }
    update_compile_order -fileset sources_1
    puts "\[ts\] BD preparation complete"
}
