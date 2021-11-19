# 
# Synthesis run script generated by Vivado
# 

proc create_report { reportName command } {
  set status "."
  append status $reportName ".fail"
  if { [file exists $status] } {
    eval file delete [glob $status]
  }
  send_msg_id runtcl-4 info "Executing : $command"
  set retval [eval catch { $command } msg]
  if { $retval != 0 } {
    set fp [open $status w]
    close $fp
    send_msg_id runtcl-5 warning "$msg"
  }
}
set_param xicom.use_bs_reader 1
create_project -in_memory -part xc7a100tfgg484-1

set_param project.singleFileAddWarning.threshold 0
set_param project.compositeFile.enableAutoGeneration 0
set_param synth.vivado.isSynthRun true
set_msg_config -source 4 -id {IP_Flow 19-2162} -severity warning -new_severity info
set_property webtalk.parent_dir E:/sysclassfiles/cpu_1/cpu_1/cpu_1.cache/wt [current_project]
set_property parent.project_path E:/sysclassfiles/cpu_1/cpu_1/cpu_1.xpr [current_project]
set_property XPM_LIBRARIES {XPM_CDC XPM_MEMORY} [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property target_language Verilog [current_project]
set_property ip_output_repo e:/sysclassfiles/cpu_1/cpu_1/cpu_1.cache/ip [current_project]
set_property ip_cache_permissions {read write} [current_project]
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/program_rom0/prgmip32.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/dram32/dram32.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/dram32/ram1.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/dram32/ram0.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/dram32/ram2.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/dram32/ram3.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/program_rom0/result.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram0/data1.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram1/data2.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram2/data3.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram0/data4.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram1/data3.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram2/data2.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram3/data1.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/program_rom0/mip32.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/program_rom0/test.coe
add_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/program_rom0/result1.coe
add_files e:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/program_rom0/bios.coe
read_verilog -library xil_defaultlib {
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/CP0.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/DataSelector_4to1.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/LED.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/define.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/MIPS32.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/Main.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/dialSwitch.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/dmemory32.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/keyboard.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/nixieTube.v
  E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/new/MIPS32_SYS.v
}
read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci
set_property used_in_implementation false [get_files -all e:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_board.xdc]
set_property used_in_implementation false [get_files -all e:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xdc]
set_property used_in_implementation false [get_files -all e:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_ooc.xdc]

read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/multu/multu.xci

read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/mult/mult.xci

read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/div/div.xci

read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/divu/divu.xci

read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram3/ram3.xci
set_property used_in_implementation false [get_files -all e:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram3/ram3_ooc.xdc]

read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram0/ram0.xci
set_property used_in_implementation false [get_files -all e:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram0/ram0_ooc.xdc]

read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram1/ram1.xci
set_property used_in_implementation false [get_files -all e:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram1/ram1_ooc.xdc]

read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram2/ram2.xci
set_property used_in_implementation false [get_files -all e:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/ram2/ram2_ooc.xdc]

read_ip -quiet E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/program_rom0/program_rom0.xci
set_property used_in_implementation false [get_files -all e:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/sources_1/ip/program_rom0/program_rom0_ooc.xdc]

# Mark all dcp files as not used in implementation to prevent them from being
# stitched into the results of this synthesis run. Any black boxes in the
# design are intentionally left as such for best results. Dcp files will be
# stitched into the design at a later time, either when this synthesis run is
# opened, or when it is stitched into a dependent implementation run.
foreach dcp [get_files -quiet -all -filter file_type=="Design\ Checkpoint"] {
  set_property used_in_implementation false $dcp
}
read_xdc E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/constrs_1/new/Minisys.xdc
set_property used_in_implementation false [get_files E:/sysclassfiles/cpu_1/cpu_1/cpu_1.srcs/constrs_1/new/Minisys.xdc]


synth_design -top MIPS32_SYS -part xc7a100tfgg484-1


# disable binary constraint mode for synth run checkpoints
set_param constraints.enableBinaryConstraints false
write_checkpoint -force -noxdef MIPS32_SYS.dcp
create_report "synth_1_synth_report_utilization_0" "report_utilization -file MIPS32_SYS_utilization_synth.rpt -pb MIPS32_SYS_utilization_synth.pb"
