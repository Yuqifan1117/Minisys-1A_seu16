`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/01/11 12:18:27
// Design Name: 
// Module Name: LED
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


module LED(led_clk,led_rst,ledcs,ledaddr,ledwdata,ledData);
input led_clk;
input led_rst;
input ledcs;
input [1:0] ledaddr;
input [15:0] ledwdata;
output reg [23:0] ledData;


always @(posedge led_clk)
    begin
        if(led_rst == 1'b1) ledData <= 24'b0;
        else if(ledcs==1&&ledaddr==2'b00)
            ledData[15:0] <= ledwdata;
        else if(ledcs==1&&ledaddr==2'b10)
            ledData[23:16] <= ledwdata[7:0];
    end
    
    
endmodule
