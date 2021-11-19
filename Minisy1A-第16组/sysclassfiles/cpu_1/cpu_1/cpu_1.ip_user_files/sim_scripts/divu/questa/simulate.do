onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib divu_opt

do {wave.do}

view wave
view structure
view signals

do {divu.udo}

run -all

quit -force
