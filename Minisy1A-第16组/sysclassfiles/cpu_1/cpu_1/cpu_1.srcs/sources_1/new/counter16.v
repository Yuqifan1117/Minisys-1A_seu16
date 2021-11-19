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
input enabled,clk;//ʹ�ܣ�ʱ��/����(�½�����Ч��
input rw;//��0д1����
input reset;//��λ
input set;//���ó�ʼֵ�ź�
input [15:0] setinitial;//��ʼֵ
input [15:0] way;//��ʽ
output cout;//�������
output [15:0] state;//״̬
output [15:0] present;//������ǰֵ
reg [15:0] theWay;//��ʽ�Ĵ���
reg [15:0] theState;//״̬�Ĵ���
reg [15:0] theinitial;//��ʼֵ�Ĵ���
reg [15:0] thepresent;//��ǰֵ�Ĵ���
assign state=theState;
assign cout=~(~theWay[0]&(thepresent==1));
assign present=thepresent;
initial theState=16'h0000;
always @(reset or set or setinitial)//��λ �� ���ó�ʼֵ
    begin
        if(reset==1)
            if(theWay[0]==0)thepresent=theinitial;
            else thepresent=0;
        if(set==1)theinitial=setinitial;
    end
always @(rw)//д��ʽ�Ĵ���
    begin
        if(rw==1)theWay=way;
    end

always @(negedge clk)//��������ʱ
    begin
        theState[15]=enabled;
        if(theWay[1]==1||(theWay[1]==0&&((theWay[0]==0&&theState[0]==0)||(theWay[0]==1&&theState[1]==0))))//-1 +1������ʱ
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
