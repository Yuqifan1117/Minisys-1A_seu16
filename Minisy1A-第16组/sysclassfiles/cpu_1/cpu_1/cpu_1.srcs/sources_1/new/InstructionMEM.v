`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/02 13:22:16
// Design Name: 
// Module Name: InstructionMEM
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
// 指令存储单元的实现
// @param InsMemRW 指令存储单元信号
// @param addr pc上指令的地址
// @param outside_pc 获取初始化的pc
// @param instruction 取得的指令
module InstructionMEM (addr, InsMemRW, instruction);
    input InsMemRW;
    // 从pc上获取下一条指令地址
    input [31:0] addr;
    // 保存获取的32位指令
    output reg [31:0] instruction;
    // 8位内存单元，每条指令的二进制代码占四个内存单元
    reg [7:0] mem [0:127];
     initial begin
     //用于测试的指令，存储在mem中 
         mem[0]=8'b11100000;
         mem[1]=8'b00000000;
         mem[2]=8'b00000000;
         mem[3]=8'b00000010;
         // 初始化指令
         instruction = 0;
     end
    always @(addr or InsMemRW)
        if (InsMemRW) begin
          // 根据地址去四个内存单元中构成指令
          instruction[31:24] = mem[addr];
          instruction[23:16] = mem[addr+1];
          instruction[15:8] = mem[addr+2];
          instruction[7:0] = mem[addr+3];
        end 
endmodule
