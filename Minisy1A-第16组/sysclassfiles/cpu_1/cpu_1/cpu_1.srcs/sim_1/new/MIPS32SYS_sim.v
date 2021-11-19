`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/01/02 18:01:57
// Design Name: 
// Module Name: MIPS32SYS_sim
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


module MIPS32SYS_sim();
    reg CLOCK;
    reg rst;
    reg [23:0] switch2N4;
    wire [23:0] led2N4;
    wire [6:0] nixieTubeC;
    wire DP;
    wire [7:0] nixieTubeA;
    
    initial
    begin
            CLOCK = 1'b0;
            forever #5 CLOCK = ~CLOCK;
    end
        
    initial
    begin
            switch2N4 = 24'h00C;
    end
    
    initial
    begin
            #620 rst = 1'b1;
            #8   rst = 1'b0;
    end
    
    MIPS32_SYS MIPS32_SYS0(
           .sys_clk_100M(CLOCK),
           .sys_rst_n(rst),
           .switch2N4(switch2N4),
           .led2N4(led2N4),
           .nixieTubeC(nixieTubeC),
           .DP(DP),
           .nixieTubeA(nixieTubeA)
    );
endmodule
