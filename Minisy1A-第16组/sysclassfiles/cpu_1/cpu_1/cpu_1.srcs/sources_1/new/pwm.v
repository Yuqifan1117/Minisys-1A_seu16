`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/16 17:01:16
// Design Name: 
// Module Name: pwm
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


module pwm(clk,setmax,maxorcompare,setcompare,,setcontrol,control_in,result,count_in);
input clk;//时钟
input setmax;//设置最大值
input setcompare;//设置对比值
input setcontrol;//设置控制寄存器
input [15:0 ] maxorcompare;//最大值或对比值输入
input [8:0] control_in;//控制输入
output result;
output [15:0] count_in;
reg [15:0] count;
reg [15:0] max;
reg [15:0] compare;
reg [7:0] control;
reg theresult;
assign result=theresult;
assign count_in=count;
initial
    begin
        count=0;
        control=0;
        max=16'hffff;
        compare=16'h8000;
        theresult=1;
    end
always @(setmax or setcompare)
    begin
        if(setmax==1)max=maxorcompare;
        else if(setcompare==1)compare=maxorcompare;
    end
always @(setcontrol)
    begin
        if(setcontrol==1)control=control_in;
    end
always @(negedge clk)
    begin
        if(control[0]==1)
            begin
                count=count+1;
                if(count>max)count=0;
                if(count>compare)theresult=0;
                else theresult=1;               
            end
    end
endmodule
