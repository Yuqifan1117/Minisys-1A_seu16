`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/10 12:44:05
// Design Name: 
// Module Name: inst_rom
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module inst_rom(
     input rom_clk_i,
     input ena,
     input  [13:0]  rom_adr_i,     //来源于取指单元的取指地址PC
     output [31:0]  Jpadr,      // 给取指单元的读出来的指令
     // UART Programmer Pinouts
     input      upg_rst_i,
     input      upg_clk_i,
     input      upg_wen_i,
     input   [13:0]     upg_adr_i,
     input   [31:0]     upg_dat_i,
     input      upg_done_i
    );
    
    wire kickOff = upg_rst_i | (~upg_rst_i & upg_done_i);
    
    /*program_rom0 instmem (
        .clk (kickOff ? rom_clk_i  : upg_clk_i),
        .ena (ena),
        .wea (kickOff ? 1'b0       : upg_wen_i),
        .addra (kickOff ? rom_adr_i : upg_adr_i),
        .dina (kickOff ? 32'h00000000 : upg_dat_i),
        .douta (Jpadr)    // 取出的32位指令
    );*/
endmodule
