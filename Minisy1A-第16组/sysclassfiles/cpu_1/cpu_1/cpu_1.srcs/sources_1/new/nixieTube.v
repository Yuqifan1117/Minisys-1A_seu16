`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/17 13:12:18
// Design Name: 
// Module Name: nixieTube
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


module nixieTube(clk,lowData_in,highData_in,specialDis_in,setlow,sethigh,setdis,C,DP,A);
input clk;
input [15:0] lowData_in;//低四位数码管数据(每四位从高到低分别是一个数码管的数 0~F)
input [15:0] highData_in;//高四位数码管数据(每四位从高到低分别是一个数码管的数 0~F)
input [15:0] specialDis_in;//特殊显示寄存器低八位依次对应 8 个小数点，1 为亮，高八位依次
//对应 8 个数码管，1 表示要显示数据
input setlow;//低四位置位
input sethigh;//高四位置位
input setdis;//特殊置位
output [6:0] C;
output DP;
output [7:0] A;
reg [6:0] C_out;
reg DP_out;
reg [7:0] A_out;
reg [15:0] lowData;
reg [15:0] highData;
reg [15:0] specialDis;
reg [7:0] choose;
reg [16:0] count;
reg refreshMes;
reg [3:0] datatmp;
integer i;
assign C=C_out;
assign A=A_out;
assign DP=DP_out;
initial
    begin
        C_out=7'b1111111;
        DP_out=1;
        A_out=8'b11111111;
        lowData=0;
        highData=0;
        specialDis=0;
        choose=8'b00000001;
        count=8'h0004;
        refreshMes=0;
        datatmp=4'h0;
    end
always @(negedge refreshMes)
    begin
         if(choose==8'b10000000)choose=8'b00000001;
                   else choose=choose<<1;
         A_out=~(choose&specialDis[15:8]);
    end
always @(negedge clk)
    begin
        if(count!=0)count=count-1;
        else 
            begin 
                count=16'h0004;
                refreshMes=~refreshMes;
            end
    end
always @(choose)
    begin
        case(choose)
            8'b00000001:begin datatmp=lowData[3:0];DP_out=~specialDis[0];end
            8'b00000010:begin datatmp=lowData[7:4];DP_out=~specialDis[1];end
            8'b00000100:begin datatmp=lowData[11:8];DP_out=~specialDis[2];end
            8'b00001000:begin datatmp=lowData[15:12];DP_out=~specialDis[3];end
            8'b00010000:begin datatmp=highData[3:0];DP_out=~specialDis[4];end
            8'b00100000:begin datatmp=highData[7:4];DP_out=~specialDis[5];end
            8'b01000000:begin datatmp=highData[11:8];DP_out=~specialDis[6];end
            8'b10000000:begin datatmp=highData[15:12];DP_out=~specialDis[7];end
        default:datatmp=4'b0000;
        endcase
    end
always @(datatmp)
    begin
        case(datatmp)
            4'h0:C_out=7'b0000001;
            4'h1:C_out=7'b1001111;
            4'h2:C_out=7'b0010010;
            4'h3:C_out=7'b0000110;
            4'h4:C_out=7'b1001100;
            4'h5:C_out=7'b0100100;
            4'h6:C_out=7'b0100000;
            4'h7:C_out=7'b0001111;
            4'h8:C_out=7'b0000000;
            4'h9:C_out=7'b0000100;
            4'ha:C_out=7'b0001000;
            4'hb:C_out=7'b1100000;
            4'hc:C_out=7'b0110001;
            4'hd:C_out=7'b1000010;
            4'he:C_out=7'b0110000;
            4'hf:C_out=7'b0111000;
        default:C_out=7'b0000000;
        endcase
    end
always @(setlow)
    begin
        if(setlow==1)lowData=lowData_in;
    end
always @(sethigh)
    begin
            if(sethigh==1)highData=highData_in;
    end
always @(setdis)
    begin
            if(setdis==1)specialDis=specialDis_in;
    end
endmodule
