`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/19 15:09:13
// Design Name: 
// Module Name: BTB
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

//  ����Branch Target Buffer���ж�̬��֧Ԥ��
module BTB(
input  clk, rst,

input [31:0] rd_PC,					//����PC
output reg rd_predicted,			//����������ź�, ��ʾrd_PC����תָ���ʱrd_predicted_PC����Ч����
output reg [31:0] rd_predicted_PC,	//��buffer�еõ���Ԥ��PC
input wr_req,						//д�����ź�
input [31:0] wr_PC,					//Ҫд��ķ�֧PC
input [31:0] wr_predicted_PC,		//Ҫд���Ԥ��PC
input wr_predicted_state_bit		//Ҫд���Ԥ��״̬λ
    );
parameter BUFFER_ADDR_LEN=4;
parameter BUFFER_SIZE=1<<BUFFER_ADDR_LEN;
parameter TAG_LEN=32-BUFFER_ADDR_LEN-2;

reg [TAG_LEN-1:0] tag_pc [0:BUFFER_SIZE-1]; 
reg [31:0] predicted_pc [0:BUFFER_SIZE-1];
reg predicted_state [0:BUFFER_SIZE-1];

wire [TAG_LEN-1:0] rd_pc_tag;
wire [BUFFER_ADDR_LEN-1:0] rd_pc_index;
wire [1:0] rd_pc_offset;
wire [TAG_LEN-1:0] wr_pc_tag;
wire [BUFFER_ADDR_LEN-1:0] wr_pc_index;
wire [1:0] wr_pc_offset;

assign {rd_pc_tag,rd_pc_index,rd_pc_offset}=rd_PC;
assign {wr_pc_tag,wr_pc_index,wr_pc_offset}=wr_PC;

always@(*)
if(tag_pc[rd_pc_index]==rd_pc_tag&&predicted_state[rd_pc_index]==1'b1)
    rd_predicted=1'b1;
else
    rd_predicted=1'b0;

always@(*)
   rd_predicted_PC=predicted_pc[rd_pc_index];

//write buffer,update
integer i;
always@(posedge clk,posedge rst)
if(rst)
begin
    for(i=0;i<BUFFER_SIZE;i=i+1)
    begin
        tag_pc[i]<=0;
        predicted_pc[i]<=0;
        predicted_state[i]<=0;
    end
end
else if(wr_req)
begin
    tag_pc[wr_pc_index]<=wr_pc_tag;
    predicted_pc[wr_pc_index]<=wr_predicted_PC;
    predicted_state[wr_pc_index]<=wr_predicted_state_bit;
end
endmodule
