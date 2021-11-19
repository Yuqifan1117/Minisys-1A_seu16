onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib ram1_opt

do {wave.do}

view wave
view structure
view signals

do {ram1.udo}

run -all

quit -force
