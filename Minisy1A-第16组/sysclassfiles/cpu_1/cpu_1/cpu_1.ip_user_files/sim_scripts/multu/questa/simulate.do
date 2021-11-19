onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib multu_opt

do {wave.do}

view wave
view structure
view signals

do {multu.udo}

run -all

quit -force
