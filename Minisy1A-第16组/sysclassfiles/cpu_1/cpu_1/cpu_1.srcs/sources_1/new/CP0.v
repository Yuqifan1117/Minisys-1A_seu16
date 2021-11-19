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
    input mtc0,    // 当前指令为mtc0
    input mfc0,    // 当前指令为mfc0
    input [31:0] pc,
    input [4:0] addr,    // CP0对应的寄存器地址
    // 简化设计Status、Cause、EPC这三个寄存器，对应sel均为0
    input [31:0] wdata,    //全局指针寄存器取出的数据用来代替CP0寄存器
    input eret,            //指令 ERET(Exception Return)
    input syscall,
    input break,
    input ri,
    input overflow,
    input ie,
    input keyboardbreak,
    output reg [31:0] rdata,    //从CP0寄存器中读取出的数据给全局指针寄存器
    output [31:0] exc_addr    //cpu跳转到一个固定的地址处理异常
    );
    //  指明是何种类型的异常或中断
    wire [4:0] cause; // 引起中断的原因
    assign cause = (syscall) ? 5'b01000 : //系统调用 syscall
            (break) ? 5'b01001 : //绝对断点指令   break
            (ri) ? 5'b01010 : // 保留指令,cpu执行到一条未定义的指令
            (overflow) ?  5'b01100 : //算术溢出，有符号运算加减溢出
            (keyboardbreak) ? 5'b00000 : 5'b11111;  //外部中断 
    //   status =  12，
    //   cause = 13，
    //   epc = 14，
    reg [31:0] cp0 [0:31];  // cp0包含32个寄存器
    wire [31:0] status = cp0[12];   //寄存器12 Status用于处理器状态的控制
    integer i;
    //  指明是何种类型的异常或中断，exception为1则能发生中断或异常
    //  IE为1中断使能
    wire exception    =    status[0] && cause != 5'b11111;
       
    always@(*) begin
           if(cpu_rst_n)begin
               for(i=0;i<32;i=i+1)    
                        cp0[i]<= 0;   // 初始话对cp0寄存器全部赋值0 
               //cp0[12] <= 32'h0000FF01;
           end
           else begin
               if(mtc0) // 当前指令为写CP0寄存器
                   cp0[addr]    <=    wdata; 
               if(exception)begin
                   cp0[14]    <=    pc;  //中断返回寄存器，用于保存中断或异常执行前 CPU原来的地址以便返回时使用；
                   cp0[12]    <=    {status[31:16],16'b1111111100000110};
                   cp0[13]    <=    {25'b0,cause,2'b0};
               end
               else if(eret) begin
                   cp0[12]    <=    {status[31:5],5'b10001};
                   
               end
           end 
     end
    assign exc_addr    =   eret ?  cp0[14]: (exception ? 32'h0000FFF4 :32'h00000000);  //  eret从中断或异常返回的地址，否则跳转到处理中断的地址
    always@(*) begin
            if(cpu_rst_n)
                 rdata <= 32'h00000000;
            else if(mfc0)  rdata    <=    cp0[addr];  // 用于mfc0指令给通用寄存器赋值
    end
       
endmodule
