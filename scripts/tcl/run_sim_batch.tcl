# run_sim_batch.tcl — xsim -tclbatch script used by run_sim.tcl's default
# (batch) mode in place of Vivado's generated <tb>.tcl.  Deliberately does
# NOT `add_wave /` or `log_wave`: with nothing logged the .wdb stays empty
# and the simulator spends its time on the design, not on waveform capture.
# run_sim.tcl issues `run -all` after launch_simulation returns.
run 0us
