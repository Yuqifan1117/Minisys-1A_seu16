`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/01/03 13:39:28
// Design Name: 
// Module Name: MIPS32_CPU
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

`include  "define.v"

module MIPS32_CPU(
        input wire sys_clk_100M,
        input wire sys_rst_n
    );
    wire                  cpu_clk_22M;
    wire                  uart_clk_10M;
    wire [31 : 0] iaddr;
    wire                  ice;
    wire [31 : 0] inst;
    wire                  dce;
    wire [31 : 0] daddr;
    wire [3 : 0  ] we;
    wire [31 : 0     ] din;
    wire [31 : 0     ] dout;

    clk_wiz_0 clocking
    (        
        // Clock in ports
        .clk_in1(sys_clk_100M),
        // Clock out ports
        .clk_out1(cpu_clk_22M),     // output clk_out1
        .clk_out2(uart_clk_10M)    // output port

    );      
    
    program_rom0 program_rom0 (
      .clka(cpu_clk_22M),    // input wire clka
      .ena(ice),      // input wire ena
      .addra(iaddr[15:2]),  // input wire [13 : 0] addra
      .douta(inst)  // output wire [31 : 0] douta
    );

    MIPS32 mips32 (
        // input
        .cpu_clk_50M(cpu_clk_22M),
        .cpu_rst_n(sys_rst_n),
        .inst(inst),
        // output
        .iaddr(iaddr),
        .ice(ice),
        .dce(dce),
        .daddr(daddr),
        .we(we),
        .din(din),
        .dm(dout)
    );

    dram32 dram32 (
        .clka(cpu_clk_22M),    // input wire clka
        .ena(dce),      // input wire ena
        .wea(we),      // input wire [3 : 0] wea
        .addra(daddr[15:2]),  // input wire [14 : 0] addra
        .dina(din),    // input wire [31 : 0] dina
        .douta(dout)  // output wire [31 : 0] douta
    );
endmodule
