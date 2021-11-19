
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/13 19:42:03
// Design Name: 
// Module Name: ID32_sim
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

module ID32_sim(); 
    // input
    reg   cpu_rst_n = 1'b1;
    reg [31:0]  id_inst_i = 8'h00000000;  // 来自指令存储器取出的指令
    reg [31:0]  id_pc = 8'h00000000; // 来自取指单元的pc
    reg [31:0      ]    rd1 = 8'h00000001;
    reg [31:0      ]    rd2 = 8'h00000010;
    // output
    wire [2:0 ]    id_alutype_o;  // 译码阶段指令操作类型
    wire [7:0   ]    id_aluop_o;  //指令操作符，8位，全局参数确定
    wire                     id_whilo_o; //HILO寄存器的写使能信号
    wire                     id_mreg_o;
    wire [4:0 ]    id_wa_o;  //译码阶段指令待写入的寄存器5位地址
    wire                     id_wreg_o; //通用寄存器堆写使能信号
    wire [31:0      ]    id_din_o;  // 要写入数据存储器中的数据，从寄存器中读取32位
    wire [31:0]    id_pc_o;  // 传入执行阶段的当前pc，用于计算下一个pc
    // 送至执行阶段的源操作数1、源操作数2
    wire [31:0     ]    id_src1_o;
    wire [31:0      ]    id_src2_o;
         
    // 送至读通用寄存器堆端口的使能和地址
    wire                     rreg1;
    wire [4:0 ]    ra1;
    wire                     rreg2;
    wire [4:0 ]    ra2;
    // 需要把读取到的指令传给执行阶段用于跳转指令的地址计算
    wire [15:0]     id_imm;
    wire [25:0]     id_add;
    // pc来源控制信号
    // 00: pc=pc+4  ; 01:pc=pc+4+(sign-extend)immediate; 10:pc=(rs) ; 11:pc={pc[31:28],addr[27:2],0,0}
    wire [1:0] PCSrc;
    
    // 实例化控制模块
    ID ID0(.cpu_rst_n(cpu_rst_n), .id_pc_i(id_pc), 
            .id_inst_i(id_inst_i),
            .rd1(rd1), .rd2(rd2),
            .rreg1(rreg1), .rreg2(rreg2),       
            .ra1(ra1), .ra2(ra2), 
            .id_aluop_o(id_aluop_o), .id_alutype_o(id_alutype_o),
            .id_src1_o(id_src1_o), .id_src2_o(id_src2_o),
            .id_wa_o(id_wa_o), .id_wreg_o(id_wreg_o),
            .id_whilo_o(id_whilo_o), .id_pc_o(id_pc_o),
            .id_mreg_o(id_mreg_o), .id_din_o(id_din_o), 
            .id_imm(id_imm), .id_add(id_add),
            .PCSrc(PCSrc)
        );
   initial begin
   #100   cpu_rst_n = 1'b0;
   #200   id_inst_i = 32'h2001000A;
   #200   id_inst_i = 32'h20020003;
   #200   id_inst_i = 32'h3403000D;
   #200   id_inst_i = 32'h00222020;
   #200   id_inst_i = 32'h10640002;
   #200   id_inst_i = 32'h00221824;
   #200   id_inst_i = 32'h00223025;
   #200   id_inst_i = 32'h00223022;
   #200   id_inst_i = 32'h14640001;
   #200   id_inst_i = 32'hAD02000A;
   #200   id_inst_i = 32'h8D04000A;
   #200   id_inst_i = 32'h10440001;
   #200   id_inst_i = 32'h00233024;
   #200   id_inst_i = 32'h00C23826;
   end
endmodule
