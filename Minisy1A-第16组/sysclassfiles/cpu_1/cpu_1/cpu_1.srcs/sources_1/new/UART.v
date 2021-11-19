`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/01/11 12:51:27
// Design Name: 
// Module Name: UART
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


module UART(clk,inputSerialData,outputParallelData,inputParallelData,outputSerialData,setctl,ctlData,stateData,setinP,reset,resetR,testcount,testcount2);
input clk;//ʱ���ź�
input reset;//���ò�������
input resetR;//���ô�������
output outputSerialData;//��������
input [15:0] outputParallelData;//��������
input setctl;//����д��
input [15:0] ctlData;//��������
input setinP;//������������
output [15:0] inputParallelData;//�������
input inputSerialData;//�������
output [15:0] stateData;//״̬�Ĵ���
output [15:0] testcount;//�ýӿڽ�Ϊ���Խӿ�
output [15:0] testcount2;//�ýӿڽ�Ϊ���Խӿ�
reg [15:0] stateReg;//״̬�Ĵ�����״̬�Ĵ����д���Žӿڵĸ���״̬��Ϣ����������������Ƿ�գ������ַ��Ƿ�׼���õȡ���ͨ�Ź����У�������ĳ��״̬ʱ���ӿ��е�״̬����߼���״̬�Ĵ�������Ӧλ��"1"���Ա���CPU��ѯ��
reg [15:0] ctlReg;//���ƼĴ�����������CPU�����Ŀ����֣��ɿ����ֵ����ݣ���������У�黹��żУ��Ȳ�����
reg [15:0] outBufferReg;//�������Ĵ�����������CPU�����������������Ĳ������ݣ������Ա��档
reg [15:0] inBufferReg;//���뻺��Ĵ���������������λ�Ĵ����н��ղ������ݣ�Ȼ����CPUȡ�ߡ�
reg [15:0] count;//����
reg count_start;//������ʼ��־λ
reg [15:0] count2;//���ռ���
reg count_start2;//���ռ�����ʼ��־
reg outData;//��������Ĵ���
reg [3:0] countCheck;//������żУ��
reg [3:0] countCheck2;//������żУ��
assign stateData=stateReg;
assign inputParallelData=inBufferReg;
assign outputSerialData=outData;
assign testcount=count;
assign testcount2=count2;
integer i;
//״̬�Ĵ�����0λ1����UARTæ��������������ݣ���һλ1�������뻺��Ĵ�����,��2λ1�����������ݵ���żУ�����
//���ƼĴ�����0λ1������У��,0����żУ�飬��1λ1��������żУ��
//���������ʽΪ��������Ϊ1����ʼ����ʱ����һλ0����ʼλ��֮�����16λ����λ�������ݿ��ƼĴ����Ƿ�����żУ������Ƿ���1λ��żУ��λ
initial
    begin
        stateReg<=0;
        ctlReg<=0;
        outBufferReg<=0;
        inBufferReg<=0;
        count<=0;
        count_start<=0;
        count_start2<=0;
        count2<=0;
        countCheck<=0;
        countCheck2<=0;
    end

always @(negedge setctl)
    begin
        ctlReg=ctlData;
    end
//���Ͷ�
always @(negedge setinP or posedge reset)
    begin
        if(reset)
            begin
                outBufferReg=0;
                stateReg[0]=0;
                count_start=0;
                count=0;
                countCheck=ctlReg[0];
            end
        else
            begin
                outBufferReg=outputParallelData;
                stateReg[0]=1;
                count_start=1;
                count=0;
                countCheck=ctlReg[0];
            end
    end
always @(negedge clk)
    begin
        if(count_start==1)count=count+1;
        else count=0;
        if(count== 16'd288)
            begin
                count_start=0;
                countCheck=0;
                count=0;
                stateReg[0]=0;
            end
    end
always @(negedge clk or posedge reset)
    begin
        if(reset)
            begin
                outData=1;
            end
        else if(count_start==1)
            begin
                case(count)
                    16'd0:outData=0;
                    16'd1:outData=0;
                    16'd16:begin 
                            outData=outBufferReg[0];
                            countCheck=countCheck+outData;
                        end
                    16'd32:begin 
                            outData=outBufferReg[1];
                            countCheck=countCheck+outData;
                        end
                    16'd48:begin 
                            outData=outBufferReg[2];
                            countCheck=countCheck+outData;
                        end
                    16'd64:begin 
                            outData=outBufferReg[3];
                            countCheck=countCheck+outData;
                        end
                    16'd80:begin 
                            outData=outBufferReg[4];
                            countCheck=countCheck+outData;
                        end
                    16'd96:begin 
                            outData=outBufferReg[5];
                            countCheck=countCheck+outData;
                        end
                    16'd112:begin 
                            outData=outBufferReg[6];
                            countCheck=countCheck+outData;
                        end
                    16'd128:begin 
                            outData=outBufferReg[7];
                            countCheck=countCheck+outData;
                        end
                    16'd144:begin 
                            outData=outBufferReg[8];
                            countCheck=countCheck+outData;
                        end
                    16'd160:begin 
                            outData=outBufferReg[9];
                            countCheck=countCheck+outData;
                        end
                    16'd176:begin 
                            outData=outBufferReg[10];
                            countCheck=countCheck+outData;
                        end
                    16'd192:begin 
                            outData=outBufferReg[11];
                            countCheck=countCheck+outData;
                        end
                    16'd208:begin 
                            outData=outBufferReg[12];
                            countCheck=countCheck+outData;
                        end
                    16'd224:begin 
                            outData=outBufferReg[13];
                            countCheck=countCheck+outData;
                        end
                    16'd240:begin 
                            outData=outBufferReg[14];
                            countCheck=countCheck+outData;
                        end
                    16'd256:begin 
                            outData=outBufferReg[15];
                            countCheck=countCheck+outData;
                        end
                    16'd272:begin 
                            outData=countCheck[0]|(~ctlData[1]);
                        end
                endcase
            end
            else outData=1;
    end
    
//�����ǽ��ն�
always @(negedge inputSerialData or posedge resetR)
    begin
        if(resetR)
            begin
                count_start2=0;
                count2=0;
                stateReg[1]=0;
                stateReg[2]=0;
                inBufferReg=0;
                countCheck2=ctlReg[0];
            end
        else if(!count_start2)
            begin
                count_start2=1;
                countCheck2=ctlReg[0];
            end
    end
always @(negedge clk)
    begin
        if(count_start2==1)count2=count2+1;
        else count2=0;
        if(count2== 16'd296)
           begin
                if(ctlReg[1]==1)
                   begin
                       if(countCheck2[0]==0)stateReg[2]=0;
                       else stateReg[2]=1;
                   end
                else stateReg[2]=0;
                count_start2=0;
                count2=0;
                countCheck2=0;
                stateReg[1]=1;
           end
    end
always @(negedge clk)
    begin
        if(count_start2==1)
            begin
                case(count2)
                    16'd24:begin
                            inBufferReg[0]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd40:begin
                            inBufferReg[1]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd56:begin
                            inBufferReg[2]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd72:begin
                            inBufferReg[3]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd88:begin
                            inBufferReg[4]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd104:begin
                            inBufferReg[5]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd120:begin
                            inBufferReg[6]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd136:begin
                            inBufferReg[7]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd152:begin
                            inBufferReg[8]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd168:begin
                            inBufferReg[9]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd184:begin
                            inBufferReg[10]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd200:begin
                            inBufferReg[11]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd216:begin
                            inBufferReg[12]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd232:begin
                            inBufferReg[13]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd248:begin
                            inBufferReg[14]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd264:begin
                            inBufferReg[15]=inputSerialData;
                            countCheck2=countCheck2+inputSerialData;
                        end
                    16'd280:begin
                            countCheck2=countCheck2+inputSerialData;
                        end
                endcase
            end
    end
endmodule
