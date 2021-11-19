`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/02 10:27:00
// Design Name: 
// Module Name: Ex_1
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


module Ex_1(
        input wire cpu_clk,
        input wire[31:0] iaddr,
        output wire[31:0] inst
    );
    program_rom0 program_rom0 (
      .clka(cpu_clk),    // input wire clka
      //.ena(ice),      // input wire ena
      .addra(iaddr[15:2]),  // input wire [13 : 0] addra
      .douta(inst)  // output wire [31 : 0] douta
    );
endmodule
