`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/13 21:55:03
// Design Name: 
// Module Name: exe32_sim
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


module exe32_sim();
    // input
    reg 	cpu_rst_n = 1'b1;
    reg   [31:0]  id_inst_i;
    reg [31:0]  id_pc = 8'h00000004; // ����ȡָ��Ԫ��pc
    reg [31:0      ]    rd1 = 8'h00000001;
    reg [31:0      ]    rd2 = 8'h00000010;
        // ������׶λ�õ���Ϣ
        
     // ������ͨ�üĴ����Ѷ˿ڵ�ʹ�ܺ͵�ַ
    wire                     rreg1;
    wire [4:0 ]    ra1;
    wire                     rreg2;
    wire [4:0 ]    ra2;
    // ��Ҫ�Ѷ�ȡ����ָ���ִ�н׶�������תָ��ĵ�ַ����
    // pc��Դ�����ź�
    // 00: pc=pc+4  ; 01:pc=pc+4+(sign-extend)immediate; 10:pc=(rs) ; 11:pc={pc[31:28],addr[27:2],0,0}
    wire [1:0] PCSrc;
    // ������׶λ�õ���Ϣ
    wire [2:0    ]     exe_alutype_i;
    wire [7:0       ]     exe_aluop_i;
    wire [31:0         ]     exe_src1_i; //������1 32λ
    wire [31:0         ]     exe_src2_i;  //������2 32λ
    wire [4:0  ]     exe_wa_i; //ָ��Ŀ�ļĴ�����ַ
    wire                     exe_wreg_i; //Ŀ�ļĴ���дʹ����Ч
    wire                     exe_mreg_i;  //�洢�����Ĵ�����Ч
    wire [31:0         ]     exe_din_i; // ����ִ�н׶δ�д�����ݴ洢��������
    wire                     exe_whilo_i; //����ִ�н׶�hilo�Ĵ���дʹ��
    wire [31:0]    exe_pc_o;
    wire [15:0 ]  exe_imm;
    wire [25:0]   exe_add;
      // ��HILO�Ĵ�����õ����� 
    reg [31:0         ]     hi_i  = 8'h00000011; //����hi������
    reg [31:0         ]     lo_i  = 8'h00000010; //����lo������
    
        // ����ִ�н׶ε���Ϣ
    wire [7:0       ]     exe_aluop_o;
    wire [4:0     ]     exe_wa_o;
    wire                     exe_wreg_o;
    wire [31:0       ]     exe_wd_o;
    wire                     exe_mreg_o;
    wire [31:0        ]     exe_din_o;
    wire                     exe_whilo_o;
    wire [63: 0]     exe_hilo_o;
    wire [31:0]    exe_pc_next;


    initial begin
        #50   cpu_rst_n = 1'b0;
        
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
    ID ID0(.cpu_rst_n(cpu_rst_n), .id_pc_i(id_pc), 
                .id_inst_i(id_inst_i),
                .rd1(rd1), .rd2(rd2),
                .rreg1(rreg1), .rreg2(rreg2),       
                .ra1(ra1), .ra2(ra2), 
                .id_aluop_o(exe_aluop_i), .id_alutype_o(exe_alutype_i),
                .id_src1_o(exe_src1_i), .id_src2_o(exe_src2_i),
                .id_wa_o(exe_wa_i), .id_wreg_o(exe_wreg_i),
                .id_whilo_o(exe_whilo_i), .id_pc_o(exe_pc_o),
                .id_mreg_o(exe_mreg_i), .id_din_o(exe_din_i), 
                .id_imm(exe_imm), .id_add(exe_add),
                .PCSrc(PCSrc)
            ); 
    //  ִ�н׶�   
    exe_stage exe_stage0(.cpu_rst_n(cpu_rst_n),
               .exe_alutype_i(exe_alutype_i), .exe_aluop_i(exe_aluop_i),
               .exe_src1_i(exe_src1_i), .exe_src2_i(exe_src2_i),
               .exe_wa_i(exe_wa_i), .exe_wreg_i(exe_wreg_i),
               .exe_mreg_i(exe_mreg_i), .exe_din_i(exe_din_i),
               .hi_i(hi_i), .lo_i(lo_i), .exe_whilo_i(exe_whilo_i),
               .exe_PCSrc(PCSrc), .exe_pc_o(exe_pc_o), .exe_imm(exe_imm), .exe_add(exe_add),
               .exe_aluop_o(exe_aluop_o),
               .exe_wa_o(exe_wa_o), .exe_wreg_o(exe_wreg_o), .exe_wd_o(exe_wd_o),
               .exe_mreg_o(exe_mreg_o), .exe_din_o(exe_din_o),
               .exe_whilo_o(exe_whilo_o), .exe_hilo_o(exe_hilo_o), .exe_pc_next(exe_pc_next)
           );
endmodule
