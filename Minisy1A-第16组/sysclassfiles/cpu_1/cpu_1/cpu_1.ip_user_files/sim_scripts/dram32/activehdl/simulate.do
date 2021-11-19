onbreak {quit -force}
onerror {quit -force}

asim -t 1ps +access +r +m+dram32 -L xil_defaultlib -L xpm -L unisims_ver -L unimacro_ver -L secureip -O5 xil_defaultlib.dram32 xil_defaultlib.glbl

do {wave.do}

view wave
view structure

do {dram32.udo}

run -all

endsim

quit -force
