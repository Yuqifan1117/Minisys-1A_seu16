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
// ָ��洢��Ԫ��ʵ��
// @param InsMemRW ָ��洢��Ԫ�ź�
// @param addr pc��ָ��ĵ�ַ
// @param outside_pc ��ȡ��ʼ����pc
// @param instruction ȡ�õ�ָ��
module InstructionMEM (addr, InsMemRW, instruction);
    input InsMemRW;
    // ��pc�ϻ�ȡ��һ��ָ���ַ
    input [31:0] addr;
    // �����ȡ��32λָ��
    output reg [31:0] instruction;
    // 8λ�ڴ浥Ԫ��ÿ��ָ��Ķ����ƴ���ռ�ĸ��ڴ浥Ԫ
    reg [7:0] mem [0:127];
     initial begin
     //���ڲ��Ե�ָ��洢��mem�� 
         mem[0]=8'b11100000;
         mem[1]=8'b00000000;
         mem[2]=8'b00000000;
         mem[3]=8'b00000010;
         // ��ʼ��ָ��
         instruction = 0;
     end
    always @(addr or InsMemRW)
        if (InsMemRW) begin
          // ���ݵ�ַȥ�ĸ��ڴ浥Ԫ�й���ָ��
          instruction[31:24] = mem[addr];
          instruction[23:16] = mem[addr+1];
          instruction[15:8] = mem[addr+2];
          instruction[7:0] = mem[addr+3];
        end 
endmodule
