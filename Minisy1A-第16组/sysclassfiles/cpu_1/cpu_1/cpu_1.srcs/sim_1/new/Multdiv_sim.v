`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/01/10 23:22:56
// Design Name: 
// Module Name: Multdiv_sim
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


module Multdiv_sim();
    reg  clk ;
    reg [31:0] pc ;
    wire [31:0] inst;
    
    initial
    begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end
    initial begin
       pc = 32'h00000000;
       #100   pc = 32'h00000004;
       #100   pc = 32'h00000008;
       #100  pc = 32'h0000000c;
       #100   pc = 32'h00000010;
    end
    
    program_rom0 program_rom0(.clka(clk), .addra(pc[15:2]), .douta(inst));
    

endmodule
