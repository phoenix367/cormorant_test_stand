# Copyright 2026 GradeBuilder SL
# SPDX-License-Identifier: Apache-2.0

# ---------------------------------------------------------------------------
# build_hw.tcl — open a kernel test-stand Vivado project, run synthesis,
# implementation, and (optionally) write the bitstream.
#
# Invoked by scripts/build_hw.sh as:
#   vivado -mode batch -source build_hw.tcl -tclargs <xpr> <top> <jobs> <bit> <ip_repo>
#
# tclargs:
#   xpr      absolute path to the .xpr project file
#   top      synthesis top-level (e.g. design_conv_wrapper)
#   jobs     parallel synth/impl jobs
#   bit      "1" to write a bitstream, "0" to stop after impl
#   ip_repo  optional IP repository path; empty string leaves the project's
#            stored ip_repo_paths untouched.  Either way ts_prepare_bd will
#            still upgrade any locked kernel IPs before synthesis.
# ---------------------------------------------------------------------------

if {[llength $argv] < 5} {
    puts stderr "build_hw.tcl: expected 5 tclargs (xpr top jobs bit ip_repo), got [llength $argv]"
    exit 1
}

set xpr     [lindex $argv 0]
set top     [lindex $argv 1]
set jobs    [lindex $argv 2]
set bit     [lindex $argv 3]
set ip_repo [lindex $argv 4]

source [file join [file dirname [info script]] lib.tcl]

puts "\[build_hw\] Opening $xpr"
open_project $xpr

ts_apply_ip_repo $ip_repo
ts_prepare_bd    $top

# Reset previous runs so a re-build is reproducible (and so a stale failure
# doesn't make Vivado short-circuit on "already done").
foreach run {synth_1 impl_1} {
    if {[get_runs $run] ne ""} {
        puts "\[build_hw\] reset_run $run"
        reset_run $run
    }
}

puts "\[build_hw\] launch_runs synth_1 -jobs $jobs"
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
set s [get_property STATUS [get_runs synth_1]]
if {[string first "Complete!" $s] < 0 && [string first "synth_design Complete" $s] < 0} {
    puts stderr "\[build_hw\] synthesis FAILED (status=$s)"
    exit 2
}

if {$bit eq "1"} {
    puts "\[build_hw\] launch_runs impl_1 -to_step write_bitstream -jobs $jobs"
    launch_runs impl_1 -to_step write_bitstream -jobs $jobs
} else {
    puts "\[build_hw\] launch_runs impl_1 -jobs $jobs (no bitstream)"
    launch_runs impl_1 -jobs $jobs
}
wait_on_run impl_1
set s [get_property STATUS [get_runs impl_1]]
if {[string first "Complete!" $s] < 0 && [string first "Complete, " $s] < 0} {
    puts stderr "\[build_hw\] implementation FAILED (status=$s)"
    exit 3
}

puts "\[build_hw\] DONE  status=$s"
close_project
