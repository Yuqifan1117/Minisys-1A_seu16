`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/09 14:24:21
// Design Name: 
// Module Name: counter16
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


module counter16(enabled,clk,way,state,cout,rw,reset,setinitial,set,present);
input enabled,clk;//使能，时钟/脉冲(下降沿有效）
input rw;//读0写1控制
input reset;//复位
input set;//设置初始值信号
input [15:0] setinitial;//初始值
input [15:0] way;//方式
output cout;//输出脉冲
output [15:0] state;//状态
output [15:0] present;//读出当前值
reg [15:0] theWay;//方式寄存器
reg [15:0] theState;//状态寄存器
reg [15:0] theinitial;//初始值寄存器
reg [15:0] thepresent;//当前值寄存器
assign state=theState;
assign cout=~(~theWay[0]&(thepresent==1));
assign present=thepresent;
initial theState=16'h0000;
always @(reset or set or setinitial)//复位 或 设置初始值
    begin
        if(reset==1)
            if(theWay[0]==0)thepresent=theinitial;
            else thepresent=0;
        if(set==1)theinitial=setinitial;
    end
always @(rw)//写方式寄存器
    begin
        if(rw==1)theWay=way;
    end

always @(negedge clk)//计数、定时
    begin
        theState[15]=enabled;
        if(theWay[1]==1||(theWay[1]==0&&((theWay[0]==0&&theState[0]==0)||(theWay[0]==1&&theState[1]==0))))//-1 +1计数定时
            if(theWay[0]==1)thepresent=(thepresent+1)%65536;
            else thepresent=(thepresent-1)%65536;
        if(theWay[0]==0)
            if(thepresent==1)
                begin
                    theState[0]=1;
                    theState[1]=0;
                end
            else 
                begin
                    theState[0]=0;
                    theState[1]=0;
                end
        else
            if(thepresent==theinitial)
                begin
                    theState[1]=1;
                    theState[0]=0;
                end
            else 
                begin
                    theState[1]=0;
                    theState[0]=0;
                end
        if(theWay[1]==1&&theWay[0]==0&&thepresent==0)thepresent=theinitial;
        else if(theWay[1]==1&&theWay[0]==1&&thepresent==theinitial)thepresent=0;
            if(theinitial==0||((theState[0]==1||theState[1]==1)&&theWay[1]==0))theState[15]=0;
            else theState[15]=1;
    end
endmodule
