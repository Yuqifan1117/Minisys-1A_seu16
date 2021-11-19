onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib ram0_opt

do {wave.do}

view wave
view structure
view signals

do {ram0.udo}

run -all

quit -force
