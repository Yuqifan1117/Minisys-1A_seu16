`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/15 13:04:18
// Design Name: 
// Module Name: keyboard
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


module keyboard(key,state,row,line,reset);
input [3:0] line;//ÁÐÏß
input reset;//¸´Î»
output [3:0] key;
output state;
output [3:0] row;

reg [3:0] keyReg;
reg stateReg;
reg [3:0] rowReg;
reg check;
reg [3:0] checkReg;
integer i;
integer j;
assign row=rowReg;
assign state=stateReg;
assign key=keyReg;
initial
    begin
        check=0;
        stateReg=0;
        keyReg=0;
        checkReg=0;
        forever
            begin
                #10
                rowReg=0;
                if(line!=15&&stateReg==0)
                 begin
                     for(i=0;i<4;i=i+1)
                         begin
                            #10
                             rowReg=15;
                             rowReg[i]=0;
                             for(j=0;j<4;j=j+1)
                                    begin
                                         #10
                                        if(line[j]==0)
                                            begin
                                                if(checkReg!=4*i+j||check==0)
                                                    begin
                                                        checkReg=4*i+j;
                                                        check=1;
                                                    end
                                                else 
                                                    begin
                                                        stateReg=1;
                                                        keyReg=checkReg;
                                                        check=0;
                                                    end
                                                
                                            end
                                    end
                            end
                    end
            end
    end
always @(reset)
    begin
        if(reset==1)
            begin
                stateReg=0;
            end
    end
endmodule
