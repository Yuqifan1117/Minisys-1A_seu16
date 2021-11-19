
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
    reg [31:0]  id_inst_i = 8'h00000000;  // ����ָ��洢��ȡ����ָ��
    reg [31:0]  id_pc = 8'h00000000; // ����ȡָ��Ԫ��pc
    reg [31:0      ]    rd1 = 8'h00000001;
    reg [31:0      ]    rd2 = 8'h00000010;
    // output
    wire [2:0 ]    id_alutype_o;  // ����׶�ָ���������
    wire [7:0   ]    id_aluop_o;  //ָ���������8λ��ȫ�ֲ���ȷ��
    wire                     id_whilo_o; //HILO�Ĵ�����дʹ���ź�
    wire                     id_mreg_o;
    wire [4:0 ]    id_wa_o;  //����׶�ָ���д��ļĴ���5λ��ַ
    wire                     id_wreg_o; //ͨ�üĴ�����дʹ���ź�
    wire [31:0      ]    id_din_o;  // Ҫд�����ݴ洢���е����ݣ��ӼĴ����ж�ȡ32λ
    wire [31:0]    id_pc_o;  // ����ִ�н׶εĵ�ǰpc�����ڼ�����һ��pc
    // ����ִ�н׶ε�Դ������1��Դ������2
    wire [31:0     ]    id_src1_o;
    wire [31:0      ]    id_src2_o;
         
    // ������ͨ�üĴ����Ѷ˿ڵ�ʹ�ܺ͵�ַ
    wire                     rreg1;
    wire [4:0 ]    ra1;
    wire                     rreg2;
    wire [4:0 ]    ra2;
    // ��Ҫ�Ѷ�ȡ����ָ���ִ�н׶�������תָ��ĵ�ַ����
    wire [15:0]     id_imm;
    wire [25:0]     id_add;
    // pc��Դ�����ź�
    // 00: pc=pc+4  ; 01:pc=pc+4+(sign-extend)immediate; 10:pc=(rs) ; 11:pc={pc[31:28],addr[27:2],0,0}
    wire [1:0] PCSrc;
    
    // ʵ��������ģ��
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
