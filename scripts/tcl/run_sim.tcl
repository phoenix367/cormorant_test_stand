# ---------------------------------------------------------------------------
# run_sim.tcl — open a kernel test-stand Vivado project and run its
# behavioural testbench in xsim batch mode.  The simulator runs to
# completion (the testbench calls $finish at the end of the test matrix);
# results are written to <data_dir>/<kernel>_test_report.json by the
# testbench's scoreboard.
#
# Invoked by scripts/run_tb.sh as:
#   vivado -mode batch -source run_sim.tcl \
#       -tclargs <xpr> <tb> <data_dir> <report> <ip_repo>
#
# Environment switches (all optional, read via $::env):
#   TS_WAVES=1     keep Vivado's default simulation tcl (add_wave / on every
#                  top-level signal → .wdb of hundreds of MB).  Default OFF: a
#                  minimal custom tcl with no wave logging — the "batch" run.
#                  Results and the JSON report are identical; only wall-clock
#                  and disk use differ.
#   TS_VERBOSE=1   pass -testplusarg VERBOSE so the testbench's per-beat AXI /
#                  DDR monitors print (default off — they dominate the log on
#                  large fixtures).
#
# tclargs:
#   xpr        absolute path to the .xpr project file
#   tb         simulation top-level module (e.g. conv_tb)
#   data_dir   absolute directory containing manifest.txt + test_*.hex fixtures
#   report     absolute path for the JSON report the scoreboard writes
#   ip_repo    optional IP repository path; empty string leaves the project's
#              stored ip_repo_paths untouched.  Either way ts_prepare_bd will
#              still upgrade any locked kernel IPs before simulation.
# ---------------------------------------------------------------------------

if {[llength $argv] < 5} {
    puts stderr "run_sim.tcl: expected 5 tclargs (xpr tb data_dir report ip_repo), got [llength $argv]"
    exit 1
}

set xpr      [lindex $argv 0]
set tb       [lindex $argv 1]
set data_dir [lindex $argv 2]
set report   [lindex $argv 3]
set ip_repo  [lindex $argv 4]

source [file join [file dirname [info script]] lib.tcl]

puts "\[run_sim\] Opening $xpr"
open_project $xpr

ts_apply_ip_repo $ip_repo

# The simulation top usually lives in the sim_1 fileset, but ts_prepare_bd
# rewires the synthesis top in [current_fileset].  Pass an empty wrapper-top
# so the proc only handles IP upgrades + BD-target/wrapper regen.
ts_prepare_bd ""

set sim_set [get_filesets sim_1]
puts "\[run_sim\] Setting sim top to $tb"
set_property top $tb $sim_set

set waves   [expr {[info exists ::env(TS_WAVES)]   && $::env(TS_WAVES)   ne "" && $::env(TS_WAVES)   ne "0"}]
set verbose [expr {[info exists ::env(TS_VERBOSE)] && $::env(TS_VERBOSE) ne "" && $::env(TS_VERBOSE) ne "0"}]

# Pass the data dir + report path to the testbench via plusargs.  The
# testbench reads $value$plusargs("DATA_DIR=%s", ...) and ("REPORT=%s", ...).
# We intentionally do NOT escape the values further — Vivado handles
# quoting of -testplusarg internally for xsim.
set more_opts "-testplusarg DATA_DIR=$data_dir -testplusarg REPORT=$report"
if {$verbose} { append more_opts " -testplusarg VERBOSE" }
set_property -name {xsim.simulate.xsim.more_options} \
    -value $more_opts \
    -objects $sim_set

if {$waves} {
    puts "\[run_sim\] TS_WAVES=1: Vivado's default sim tcl (add_wave /)"
    set_property -name {xsim.simulate.custom_tcl} -value {} -objects $sim_set
} else {
    # Batch run: replace Vivado's generated <tb>.tcl (which does `add_wave /`
    # and so logs every top-level signal into the .wdb) with a no-op tcl.
    # Elaboration stays at Vivado's default --debug typical: `--debug off`
    # was tried and bought nothing measurable (xsim is CPU-bound in the
    # design, not in trace capture) while making any .wcfg attached to the
    # project fail with "compiled without trace information".
    puts "\[run_sim\] batch: no wave logging (TS_WAVES=1 to restore add_wave /)"
    set_property -name {xsim.simulate.custom_tcl} \
        -value [file normalize [file join [file dirname [info script]] run_sim_batch.tcl]] \
        -objects $sim_set
}
set_property -name {xsim.simulate.log_all_signals} -value false -objects $sim_set

puts "\[run_sim\] launch_simulation"
launch_simulation -mode behavioral -simset $sim_set

puts "\[run_sim\] run -all"
run -all

puts "\[run_sim\] DONE  report=$report"
close_sim
close_project
