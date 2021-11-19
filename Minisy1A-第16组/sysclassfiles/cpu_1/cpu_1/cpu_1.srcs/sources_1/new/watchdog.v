`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/16 18:25:39
// Design Name: 
// Module Name: watchdog
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


  module watchdog(clk,RESET_out,softreset_in,count_out);
input clk;//时钟
input softreset_in;//软件复位输入
output RESET_out;
output [15:0] count_out;
reg [15:0] count;
reg reset;
assign RESET_out=reset;
assign count_out=count;
initial
    begin
        count=16'hffff;
        reset=0;
    end
always @(negedge clk)
    begin
        count=count-1;
        if(count==0)reset=1;
        else reset=0;
    end
always @(softreset_in)
    begin
        if(softreset_in==1)
            begin
                count=16'hffff;
            end
    end
endmodule
