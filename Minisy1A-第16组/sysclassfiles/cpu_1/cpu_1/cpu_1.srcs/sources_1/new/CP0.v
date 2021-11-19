`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/14 16:06:23
// Design Name: 
// Module Name: CP0
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


module CP0(
    input clk,
    input cpu_rst_n,
    input mtc0,    // ��ǰָ��Ϊmtc0
    input mfc0,    // ��ǰָ��Ϊmfc0
    input [31:0] pc,
    input [4:0] addr,    // CP0��Ӧ�ļĴ�����ַ
    // �����Status��Cause��EPC�������Ĵ�������Ӧsel��Ϊ0
    input [31:0] wdata,    //ȫ��ָ��Ĵ���ȡ����������������CP0�Ĵ���
    input eret,            //ָ�� ERET(Exception Return)
    input syscall,
    input break,
    input ri,
    input overflow,
    input ie,
    input keyboardbreak,
    output reg [31:0] rdata,    //��CP0�Ĵ����ж�ȡ�������ݸ�ȫ��ָ��Ĵ���
    output [31:0] exc_addr    //cpu��ת��һ���̶��ĵ�ַ�����쳣
    );
    //  ָ���Ǻ������͵��쳣���ж�
    wire [4:0] cause; // �����жϵ�ԭ��
    assign cause = (syscall) ? 5'b01000 : //ϵͳ���� syscall
            (break) ? 5'b01001 : //���Զϵ�ָ��   break
            (ri) ? 5'b01010 : // ����ָ��,cpuִ�е�һ��δ�����ָ��
            (overflow) ?  5'b01100 : //����������з�������Ӽ����
            (keyboardbreak) ? 5'b00000 : 5'b11111;  //�ⲿ�ж� 
    //   status =  12��
    //   cause = 13��
    //   epc = 14��
    reg [31:0] cp0 [0:31];  // cp0����32���Ĵ���
    wire [31:0] status = cp0[12];   //�Ĵ���12 Status���ڴ�����״̬�Ŀ���
    integer i;
    //  ָ���Ǻ������͵��쳣���жϣ�exceptionΪ1���ܷ����жϻ��쳣
    //  IEΪ1�ж�ʹ��
    wire exception    =    status[0] && cause != 5'b11111;
       
    always@(*) begin
           if(cpu_rst_n)begin
               for(i=0;i<32;i=i+1)    
                        cp0[i]<= 0;   // ��ʼ����cp0�Ĵ���ȫ����ֵ0 
               //cp0[12] <= 32'h0000FF01;
           end
           else begin
               if(mtc0) // ��ǰָ��ΪдCP0�Ĵ���
                   cp0[addr]    <=    wdata; 
               if(exception)begin
                   cp0[14]    <=    pc;  //�жϷ��ؼĴ��������ڱ����жϻ��쳣ִ��ǰ CPUԭ���ĵ�ַ�Ա㷵��ʱʹ�ã�
                   cp0[12]    <=    {status[31:16],16'b1111111100000110};
                   cp0[13]    <=    {25'b0,cause,2'b0};
               end
               else if(eret) begin
                   cp0[12]    <=    {status[31:5],5'b10001};
                   
               end
           end 
     end
    assign exc_addr    =   eret ?  cp0[14]: (exception ? 32'h0000FFF4 :32'h00000000);  //  eret���жϻ��쳣���صĵ�ַ��������ת�������жϵĵ�ַ
    always@(*) begin
            if(cpu_rst_n)
                 rdata <= 32'h00000000;
            else if(mfc0)  rdata    <=    cp0[addr];  // ����mfc0ָ���ͨ�üĴ�����ֵ
    end
       
endmodule
