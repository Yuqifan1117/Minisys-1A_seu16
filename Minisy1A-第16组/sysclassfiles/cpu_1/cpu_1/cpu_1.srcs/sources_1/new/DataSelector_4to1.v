`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/01/02 10:52:51
// Design Name: 
// Module Name: DataSelector_4to1
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


//////////////////////////////////////////////////////////////////////////////////

// 四选一数据选择器的实现
// @param A 输入1
// @param B 输入2
// @param C 输入3
// @param D 输入4
// @param Control 选择器的控制信号
// @param Result 选择的结果
module DataSelector_4to1(A, B, C, D , Control,rst, Result);
  input [31:0] A, B, C,D;
  input [1:0] Control;
  input rst;
  output reg[31:0] Result;
  always @(Control or A or B or C or D or rst) begin
    if(rst == 1'b1) Result = 0;
    else begin
        case(Control)
            2'b00:Result = A;
            2'b01:Result = B;
            2'b10:Result = C;
            2'b11:Result = D;
         endcase
    end
  end
endmodule
