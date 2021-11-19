onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib ram3_opt

do {wave.do}

view wave
view structure
view signals

do {ram3.udo}

run -all

quit -force
