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
input clk;//时钟信号
input reset;//重置并行输入
input resetR;//重置串行输入
output outputSerialData;//串行输入
input [15:0] outputParallelData;//并行输入
input setctl;//控制写入
input [15:0] ctlData;//控制内容
input setinP;//并行输入设置
output [15:0] inputParallelData;//并行输出
input inputSerialData;//串行输出
output [15:0] stateData;//状态寄存器
output [15:0] testcount;//该接口仅为测试接口
output [15:0] testcount2;//该接口仅为测试接口
reg [15:0] stateReg;//状态寄存器。状态寄存器中存放着接口的各种状态信息，例如输出缓冲区是否空，输入字符是否准备好等。在通信过程中，当符合某种状态时，接口中的状态检测逻辑将状态寄存器的相应位置"1"，以便让CPU查询。
reg [15:0] ctlReg;//控制寄存器，它接收CPU送来的控制字，由控制字的内容，决定是奇校验还是偶校验等参数。
reg [15:0] outBufferReg;//输出缓冲寄存器，它接收CPU从数据总线上送来的并行数据，并加以保存。
reg [15:0] inBufferReg;//输入缓冲寄存器，它从输入移位寄存器中接收并行数据，然后由CPU取走。
reg [15:0] count;//计数
reg count_start;//计数开始标志位
reg [15:0] count2;//接收计数
reg count_start2;//接收计数开始标志
reg outData;//串行输出寄存器
reg [3:0] countCheck;//进行奇偶校验
reg [3:0] countCheck2;//进行奇偶校验
assign stateData=stateReg;
assign inputParallelData=inBufferReg;
assign outputSerialData=outData;
assign testcount=count;
assign testcount2=count2;
integer i;
//状态寄存器第0位1代表UART忙，即正在输出数据，第一位1代表输入缓冲寄存器满,第2位1代表输入数据的奇偶校验出错
//控制寄存器第0位1代表奇校验,0代表偶校验，第1位1代表有奇偶校验
//串口输出格式为，无数据为1，开始发送时发送一位0做起始位，之后跟随16位数据位，最后根据控制寄存器是否有奇偶校验决定是否发送1位奇偶校验位
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
//发送端
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
    
//以下是接收端
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
