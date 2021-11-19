onbreak {quit -f}
onerror {quit -f}

vsim -voptargs="+acc" -t 1ps -L xil_defaultlib -L xpm -L unisims_ver -L unimacro_ver -L secureip -lib xil_defaultlib xil_defaultlib.mult xil_defaultlib.glbl

do {wave.do}

view wave
view structure
view signals

do {mult.udo}

run -all

quit -force
