`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/07 15:39:01
// Design Name: 
// Module Name: dmemory32
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


module dmemory32(
        input   wire    ram_clk_i,
        input   wire[13:0]    ram_adr_i,   //来自执行阶段计算出的存储器地址alu_result
        input   wire[31:0]    dina,   //来自译码单元的read_data2,待写入存储器的数
        input   wire[3:0]     wea,
        input   wire          dce,
        output  reg [31:0]    douta,   //从存储器中取出的数，用于sw指令
        output  reg [31:0]    mem_src
               
    );
    wire clk;
    assign clk = !ram_clk_i;
    wire Memwrite0,Memwrite1,Memwrite2,Memwrite3; // 四个存储器都有自己独立的写使能信号
    assign Memwrite0 = wea[0];
    assign Memwrite1 = wea[1];
    assign Memwrite2 = wea[2];
    assign Memwrite3 = wea[3];  
    wire [7:0] write_data0,write_data1,write_data2,write_data3;   // 写数据
    assign write_data3 = dina[31:24];
    assign write_data2 = dina[23:16];
    assign write_data1 = dina[15:8];
    assign write_data0 = dina[7:0];
    wire[7:0] read_data0,read_data1,read_data2,read_data3;     // 读数据
    ram0 ram0(.clka(ram_clk_i),.wea(Memwrite0),.addra(ram_adr_i),.dina(write_data0),.douta(read_data0));
    ram1 ram1(.clka(ram_clk_i),.wea(Memwrite1),.addra(ram_adr_i),.dina(write_data1),.douta(read_data1));
    ram2 ram2(.clka(ram_clk_i),.wea(Memwrite2),.addra(ram_adr_i),.dina(write_data2),.douta(read_data2));
    ram3 ram3(.clka(ram_clk_i),.wea(Memwrite3),.addra(ram_adr_i),.dina(write_data3),.douta(read_data3));
    always @(*) begin
        if (dce == 1'b1)
            mem_src <= {read_data3,read_data2,read_data1,read_data0}; 
        else
            mem_src <= 32'h00000000;
    end
    always @(posedge clk) begin
        if (dce == 1'b1)
            douta <= {read_data3,read_data2,read_data1,read_data0}; 
        else
            douta <= 32'h00000000;
    end
endmodule
