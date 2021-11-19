`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/10 12:21:36
// Design Name: 
// Module Name: clk_sim
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


module clk_sim();
    // INPUT
    reg pclk = 0;
    // OUTPUT
    wire clock, uclk;
    clk_wiz_0 Uclk(
        .clk_in1 (pclk),      // 仿真输入时钟 100MHz
        .clk_out1 (clock),        // CPU时钟
        .clk_out2 (uclk)          // UART Programmer Clock
    );
    
    always #5 pclk = ~pclk;
endmodule
