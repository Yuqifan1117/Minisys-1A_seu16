`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/01/11 12:36:21
// Design Name: 
// Module Name: dialSwitch
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


module dialSwitch(switclk,switrst,switcs,switaddr,switchData,switch_i);
input switclk;
input switrst;
input switcs;
input [23:0] switch_i;
input [1:0] switaddr;
output reg [15:0] switchData;

always @(posedge switclk)
    begin
        if(switrst == 1'b1) switchData <= 16'b0;
        else if(switcs==1'b1&&switaddr==2'b00) switchData <= switch_i[15:0];
        else if(switcs==1'b1&&switaddr==2'b10) begin
            switchData[15:8] <= 8'b00000000;
            switchData[7:0] <= switch_i[23:16];
        end
    end
endmodule
