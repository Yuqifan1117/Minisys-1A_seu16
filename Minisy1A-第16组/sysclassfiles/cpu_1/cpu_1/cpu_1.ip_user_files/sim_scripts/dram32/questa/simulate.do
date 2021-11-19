onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib dram32_opt

do {wave.do}

view wave
view structure
view signals

do {dram32.udo}

run -all

quit -force
