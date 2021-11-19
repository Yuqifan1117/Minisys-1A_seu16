`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/09 14:03:20
// Design Name: 
// Module Name: Main
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
`include "define.v"


module Main(CLK,RST,outside_pc,instruction,now_pc);
      // �������������������
      input CLK, RST;
      input [31:0] outside_pc;
      output [31:0] instruction, now_pc;
      parameter endReg = 5'b11111; // 31�żĴ���
      
      // ����ͨ·
      wire [31:0] pc, pc0, pc4, i_IR, instruction, pcChoose3, pcChoose1, extendData, ALUResult, WriteData, ReadData1, ReadData2, DataOut;
      wire [31:0] o_ADR, o_BDR, o_ALUout, i_ALUM2DR,i_ALUData1,i_ALUData2;
      wire zero;
      // �����ź�
      wire [2:0] ALUOp;
      wire [1:0] ExtSel, RegDst, PCSrc;
      wire PCWre, IRWre, InsMemRW, WrRegData, RegWre, ALUSrcB, DataMemRW, DBDataSrc;
endmodule

// ȡֵģ��

// IFģ���ʵ��,ֻ�������pc�ź��޸�pc�Ĵ�����ֵ���������iaddrָ���ַ
// ����ָ���ַ�ļ�����ѡ����ʵ��
// @param cpu_clk ʱ���ź�
// @param ice ָ��洢��ʹ���ź�
// @param cpu_rst_n�ź� �����ź�
// @param i_pc �����pcֵ
// @param iaddr �����ָ���ֵַ
// @param outside_pc ??? Ԥ��һ��pcֵ����RSTΪ1ʱ������λΪinit_pc
// RSTΪ0ʱ�ж�pcWre�źţ�pcWre�ź�Ϊ1ʱ��i_pc��Ϊpc���
module IF(
    input 	wire 		cpu_clk,
    input 	wire 		cpu_rst_n,
    input   wire [`INST_ADDR_BUS] i_pc,
    input wire [5:0] stall,   //  ��ͣ��ˮ�߿����źţ�6λ������˿ڣ����ڼ�����
    input wire exe_branch,   // ����ִ�е�Ԫ��������ת�ź�
    input wire [`INST_ADDR_BUS] branchAddr,
    input wire [`INST_ADDR_BUS] exc_addr,
    //output  reg          ice,
    output 	reg  [`INST_ADDR_BUS] 	pc,
    output 	reg [`INST_ADDR_BUS]	iaddr
    );
    
    /*always @(posedge cpu_clk) begin
		if (cpu_rst_n == `RST_ENABLE) begin
			ice <= `CHIP_DISABLE;		      // ��λ��ʱ��ָ��洢������  
		end else begin
			ice <= `CHIP_ENABLE; 		      // ��λ������ָ��洢��ʹ��
		end
	end*/

    always @(*) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            iaddr <= `PC_INIT;
            //pc    <= `PC_INIT;
        end
        else begin	
            if(stall[0] == `DISABLE)
            begin
                if(exe_branch == 1'b0 && exc_addr != 32'h0000FFF4) begin
                    iaddr <= i_pc;                 // ��÷���ָ��洢���ĵ�ַ       
                end
                else if(exc_addr == 32'h0000FFF4)begin
                    iaddr <= exc_addr;                 
                end    
                else  iaddr <=   branchAddr;                    
            end
        end
    end
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            pc    <= `PC_INIT;
        end
        else begin    
            if(stall[0] == `DISABLE)
            begin
                if(exe_branch == 1'b0 && exc_addr == `ZERO_WORD) begin
                    pc    <= i_pc;      
                end
                else if(exc_addr != `ZERO_WORD)begin
                    pc  <= exc_addr;                 
                end                   
                else begin         
                    pc    <= branchAddr;
                end                              
            end
        end
    end    
    //assign pc = (cpu_rst_n == `RST_ENABLE) ? `PC_INIT : i_pc; // ָ��洢�����õ�ʱ��PC���ֳ�ʼֵ��MIPS32������Ϊ0x00000000��
    
endmodule

// pc��ת�����ӳ������ڻ�ȡ�ӳ���ĵ�ַ
// @param pc ִ�и�ָ��ʱpc��ֵ
// @param i_addr ����ĵ�ַ
// @param o_addr ����ĵ�ַ

module PCJump(pc, i_addr, o_addr);
  input [31:0] pc;
  input [25:0] i_addr;
  output reg[31:0] o_addr;
  reg [27:0] mid; // ���ڴ���м�ֵ
  // �����ַ��ǰ��λ����pc[31:28]���м�26λ����i_addr[27:2], ����λ��0
  always @(i_addr) begin
     mid = i_addr << 2;
     o_addr <= {pc[31:28], mid[27:0]};
  end
endmodule

// PC �������� ,���ڻ�ȡ��ת��ĵ�ַ��ͨ�������źſ����Ƿ���ת
// @param now_pc  ��ǰpcֵ
// @param o_pc ���pcֵ
// @param branch ����Ϊ1������ת
// @param imm ������
module PCAddImm(now_pc, imm,branch, o_pc);
  input wire [31:0] now_pc, imm;
  input wire branch;
  output wire [31:0] o_pc;
  // �ڴ浥Ԫ�����ֽ�Ϊ��λ�ģ�32λ��ַ��СΪ4���ֽڣ�����pc=pc+imm*4
  assign o_pc = (branch == 1'b1) ? now_pc + (imm << 2) : now_pc;
endmodule

// ʵ��PC����������ָ��ռ4�ֽ�
// @param i_pc �����pcֵ
// @param o_pc �����pcֵ
module PCAddFour(i_pc, o_pc);
  input wire [31:0] i_pc;
  output wire [31:0] o_pc;
  assign o_pc[31:0] = i_pc[31:0] + 4;
endmodule

// ������չ��Ԫ��ʵ��
// @param i_num ���������
// @param ExtSel ���Ʒ�����չ��Ԫ���ź�
// @param o_num ���������
module SignExtend(i_num, ExtSel, o_num);
  input wire [15:0] i_num;
  input wire ExtSel;
  output reg [31:0] o_num;
  initial begin
    o_num = 0;
  end
  always @(i_num or ExtSel) begin
     case(ExtSel)
        // ExtSel Ϊ0ʱ���޷�����������չ
        1'b0: o_num <= {{16{1'b0}}, i_num[15:0]};
        // ExtSel Ϊ1ʱ���з�����������չ
        1'b1: o_num <= {{16{i_num[15]}}, i_num[15:0]};
        // �������Ĭ���з�����������չ
        default: o_num <= {{16{i_num[15]}}, i_num[15:0]}; // Ĭ�Ϸ�����չ
    endcase
  end
endmodule

// ָ������Ĵ���ģ��

module IR_reg (
	input  wire  cpu_clk,  //cpuʱ���ź�
	input  wire  cpu_rst_n,  //cpu��λ�ź�

	// ����ȡָ�׶ε���Ϣ  
	input  wire [`INST_ADDR_BUS]       if_pc,
	input  wire [`INST_BUS     ]       rom_inst,
	input wire[5:0] stall,  // ֹͣ��ˮ���ź�
	input wire clean,    // IF/ID�Ĵ�������ź�
	// ��������׶ε���Ϣ  
	output reg [`INST_BUS     ]       id_inst_i,
	output reg  [`INST_ADDR_BUS]       id_pc
	);
    // �����������ش���
	always @(negedge cpu_clk) begin
	    // ��λ��ʱ����������׶ε���Ϣ��0
		if (cpu_rst_n == `RST_ENABLE) begin
			id_pc 	<= `PC_INIT;
			id_inst_i    <= `ZERO_WORD;
		end
		// ������ȡָ�׶ε���Ϣ�Ĵ沢��������׶�
		else begin
            if(clean == 1'b1) begin
                id_pc <= `PC_INIT;
                id_inst_i    <= `ZERO_WORD;
            end		
		    else if(stall[1] == `ENABLE && stall[2] == `DISABLE)
            begin
                id_pc <= `PC_INIT;
                id_inst_i    <= `ZERO_WORD;
            end
            else if(stall[1] == `DISABLE)
            begin
                id_pc <= if_pc;
                id_inst_i   <= rom_inst;
            end
		end
	end

endmodule

// ����ģ��ʵ��

module ID(
    input  wire   cpu_clk,       // ʱ���ź�
    input  wire   cpu_rst_n,   // cpu��λ�ź�
    // ��ȡָ�׶λ�õ�PCֵ��ָ���ַ32λ
    input  wire [`INST_ADDR_BUS]    id_pc_i,
    // ��ָ��洢��������ָ���֣�ָ��32λ
    input  wire [`INST_BUS     ]    id_inst_i,
    // ��ͨ�üĴ����Ѷ��������� 
    input  wire [`REG_BUS      ]    rd1,
    input  wire [`REG_BUS      ]    rd2,
    // �жϷ�������ð�ջ�lw����ð�մ�������ź���
    input wire [`REG_ADDR_BUS ] ex_wrAddr,   // EX������Ҫд�صĵ�ַ��5λ������˿�
    input wire [`REG_BUS      ] ex_wrData,    // EX��Ҫд�ص����ݣ�32λ������˿�
    input wire [`REG_ADDR_BUS ] mem_wrAddr,  //MEM������Ҫд�صĵ�ַ��5λ������˿�
    input wire [`REG_BUS      ] mem_wrData,  //MEM��Ҫд�ص����ݣ�32λ������˿�
    input wire [`REG_ADDR_BUS ] wb_wrAddr,  //WB������Ҫд�صĵ�ַ��5λ������˿�
    input wire [`REG_BUS      ] wb_wrData,  //WB��Ҫд�ص����ݣ�32λ������˿�
    input wire ex_wrn,   //EX�����ݵ�д�źţ�һλ������˿�
    input wire mem_wrn,  //MEM�����ݵ�д�źţ�һλ������˿�
    input wire mem2reg,   // MEM�δӴ洢��������д�ص�ʹ���ź�
    input wire wb_wrn,  //WB�����ݵ�д�źţ�һλ������˿�
    input wire E_Reg2reg,  //LW�ź�Ϊ0��ִ�н׶��ͻ��ж���һ��ָ��Ϊlw
    // ����ִ�е�Ԫ��������Ƿ�������ת
    input wire en_branch,  

    // ����ִ�н׶ε�������Ϣ
    output reg [`ALUTYPE_BUS  ]    id_alutype_o,  // ����׶�ָ���������
    output reg [`ALUOP_BUS    ]    id_aluop_o,  //ָ���������8λ��ȫ�ֲ���ȷ��
    output reg                     id_whilo_o, //HILO�Ĵ�����дʹ���ź�
    output reg                     id_mreg_o,
    output reg [`REG_ADDR_BUS ]    id_wa_o,  //����׶�ָ���д��ļĴ���5λ��ַ
    output reg                     id_wreg_o  , //ͨ�üĴ�����дʹ���ź�
    output reg [`REG_BUS      ]    id_din_o,  // Ҫд�����ݴ洢���е����ݣ��ӼĴ����ж�ȡ32λ
    //output  wire [`INST_ADDR_BUS]    id_pc_o,  // ����ִ�н׶εĵ�ǰpc�����ڼ�����һ��pc
    // ����ִ�н׶ε�Դ������1��Դ������2
    output reg [`REG_BUS      ]    id_src1_o,
    output reg [`REG_BUS      ]    id_src2_o,
    output reg src1mem_e,
    output reg src2mem_e, 
    // ������ͨ�üĴ����Ѷ˿ڵ�ʹ�ܺ͵�ַ
    output wire                     rreg1,
    output wire [`REG_ADDR_BUS ]    ra1,
    output wire                     rreg2,
    output wire [`REG_ADDR_BUS ]    ra2,
    output reg mtc0,
    output reg mfc0,
    // ������ˮ����ͣ�źţ�һλ������˿ڣ��������ɸ�ģ�����ͣ�ź�
    output wire stall_rep,
    output reg [`INST_ADDR_BUS] branchAddr,
    output wire Reg2reg,
    // pc��Դ�����ź�
    // 00: pc=pc+4  ; 01:pc=pc+4+(sign-extend)immediate; 10:pc=(rs) ; 11:pc={pc[31:28],addr[27:2],0,0}
    output reg [31:0] reg31data,  // ������$31�ķ��ص�ַ
    output reg wreg31,     // д31�żĴ����ź�
    output wire [31:0] next_pc,
    output wire syscall,
    output wire break,
    output wire eret,
    output reg [4:0] cp0addr,
    output reg [4:0] mfc0addr,
    output wire ri
    );

    // ����С��ģʽ��ָ֯���֣�ȡ����ַ���ֽ��ڸ�λ
    wire [`INST_BUS] id_inst = id_inst_i; //{id_inst_i[7:0], id_inst_i[15:8], id_inst_i[23:16], id_inst_i[31:24]};
    wire [31:0]  pc4, extendData,pcChoose1,pcChoose2,pcChoose3;
    wire [1:0] PCSrc;
    // ��ȡָ�����и����ֶε���Ϣ��R/I/Jָ��
    wire [5 :0] op   = id_inst[31:26];
    wire [5 :0] func = id_inst[5 : 0];
    wire [4 :0] rd   = id_inst[15:11];
    wire [4 :0] rs   = id_inst[25:21];
    wire [4 :0] rt   = id_inst[20:16];
    wire [4 :0] sa   = id_inst[10: 6];  //��λ��
    wire [15:0] imm  = id_inst[15: 0]; 
    wire [25:0] add  = id_inst[25 :0]; //Jָ����ת��ַ
    /*-------------------- ��һ�������߼���ȷ����ǰ��Ҫ�����ָ�� --------------------*/
    wire inst_reg  = ~|op;
    wire inst_add  = inst_reg& func[5]&~func[4]&~func[3]&~func[2]&~func[1]&~func[0];
    wire inst_addu  = inst_reg& func[5]&~func[4]&~func[3]&~func[2]&~func[1]&func[0]; 
    wire inst_sub  = inst_reg& func[5]&~func[4]&~func[3]&~func[2]&func[1]&~func[0];
    wire inst_subu = inst_reg& func[5]&~func[4]&~func[3]&~func[2]& func[1]& func[0];
    wire inst_and  = inst_reg& func[5]&~func[4]&~func[3]& func[2]&~func[1]&~func[0];
    
    wire inst_mult = inst_reg&~func[5]& func[4]& func[3]&~func[2]&~func[1]&~func[0];
    wire inst_multu  = inst_reg&~func[5]& func[4]& func[3]&~func[2]&~func[1]& func[0];
    wire inst_div  = inst_reg&~func[5]& func[4]& func[3]&~func[2]& func[1]&~func[0];
    wire inst_divu  = inst_reg&~func[5]& func[4]& func[3]&~func[2]& func[1]& func[0];
    wire inst_mfhi = inst_reg&~func[5]& func[4]&~func[3]&~func[2]&~func[1]&~func[0];
    wire inst_mflo = inst_reg&~func[5]& func[4]&~func[3]&~func[2]& func[1]&~func[0];
    wire inst_mthi = inst_reg&~func[5]& func[4]&~func[3]&~func[2]&~func[1]& func[0];
    wire inst_mtlo = inst_reg&~func[5]& func[4]&~func[3]&~func[2]& func[1]& func[0];
    
    // mfc0  ,mtc0 ʵ��
    wire inst_mfc0 = ~op[5]& op[4]&~op[3]&~op[2]&~op[1]&~op[0]&~func[5]&~func[4]&~func[3]&~rs[2];
    wire inst_mtc0 = ~op[5]& op[4]&~op[3]&~op[2]&~op[1]&~op[0]&~func[5]&~func[4]&~func[3]& rs[2];
    
    wire inst_or = inst_reg& func[5]&~func[4]&~func[3]& func[2]&~func[1]& func[0];
    wire inst_xor = inst_reg& func[5]&~func[4]&~func[3]& func[2]& func[1]&~func[0];
    wire inst_nor = inst_reg& func[5]&~func[4]&~func[3]& func[2]& func[1]& func[0];
    
    wire inst_slt  = inst_reg& func[5]&~func[4]& func[3]&~func[2]& func[1]&~func[0];
    wire inst_sltu  = inst_reg& func[5]&~func[4]& func[3]&~func[2]& func[1]& func[0];
    wire inst_sll  = inst_reg&~func[5]&~func[4]&~func[3]&~func[2]&~func[1]&~func[0];
    wire inst_srl  = inst_reg&~func[5]&~func[4]&~func[3]&~func[2]& func[1]&~func[0];
    wire inst_sra  = inst_reg&~func[5]&~func[4]&~func[3]&~func[2]& func[1]& func[0];
    wire inst_sllv  = inst_reg&~func[5]&~func[4]&~func[3]& func[2]&~func[1]&~func[0];
    wire inst_srlv  = inst_reg&~func[5]&~func[4]&~func[3]& func[2]& func[1]&~func[0];
    wire inst_srav  = inst_reg&~func[5]&~func[4]&~func[3]& func[2]& func[1]& func[0];
    
    wire inst_jr  = inst_reg&~func[5]&~func[4]& func[3]&~func[2]&~func[1]&~func[0];
    wire inst_jalr  = inst_reg&~func[5]&~func[4]& func[3]&~func[2]&~func[1]& func[0];
    wire inst_break  = inst_reg&~func[5]&~func[4]& func[3]& func[2]&~func[1]& func[0];
    wire inst_syscall  = inst_reg&~func[5]&~func[4]& func[3]& func[2]&~func[1]&~func[0];
    wire inst_eret = ~op[5]& op[4]&~op[3]&~op[2]&~op[1]&~op[0]&~func[5]& func[4]& func[3];
    // I����ָ��
    wire inst_addi  =~op[5]&~op[4]& op[3]&~op[2]&~op[1]&~op[0];
    wire inst_addiu  =~op[5]&~op[4]& op[3]&~op[2]&~op[1]& op[0];
    wire inst_andi  =~op[5]&~op[4]& op[3]& op[2]&~op[1]&~op[0];
    wire inst_ori  =~op[5]&~op[4]& op[3]& op[2]&~op[1]& op[0];
    wire inst_xori  =~op[5]&~op[4]& op[3]& op[2]& op[1]&~op[0];
    wire inst_lui  =~op[5]&~op[4]& op[3]& op[2]& op[1]& op[0];
       
    wire inst_lb   = op[5]&~op[4]&~op[3]&~op[2]&~op[1]&~op[0];
    wire inst_lbu   = op[5]&~op[4]&~op[3]& op[2]&~op[1]&~op[0];
    wire inst_lh   = op[5]&~op[4]&~op[3]&~op[2]&~op[1]& op[0];
    wire inst_lhu   = op[5]&~op[4]&~op[3]& op[2]&~op[1]& op[0];
    wire inst_sb   = op[5]&~op[4]& op[3]&~op[2]&~op[1]&~op[0];
    wire inst_sh   = op[5]&~op[4]& op[3]&~op[2]&~op[1]& op[0];
    wire inst_lw   = op[5]&~op[4]&~op[3]&~op[2]& op[1]& op[0];
    wire inst_sw   = op[5]&~op[4]& op[3]&~op[2]& op[1]& op[0];
    
    wire inst_beq   =~op[5]&~op[4]&~op[3]& op[2]&~op[1]&~op[0];
    wire inst_bne   =~op[5]&~op[4]&~op[3]& op[2]&~op[1]& op[0];
    wire inst_bgez   =~op[5]&~op[4]&~op[3]&~op[2]&~op[1]& op[0]&~rt[4]&~rt[3]&~rt[2]&~rt[1]& rt[0];
    wire inst_bgtz   =~op[5]&~op[4]&~op[3]& op[2]& op[1]& op[0]&~rt[4]&~rt[3]&~rt[2]&~rt[1]&~rt[0];
    wire inst_blez   =~op[5]&~op[4]&~op[3]& op[2]& op[1]&~op[0]&~rt[4]&~rt[3]&~rt[2]&~rt[1]&~rt[0];
    wire inst_bltz   =~op[5]&~op[4]&~op[3]&~op[2]&~op[1]& op[0]&~rt[4]&~rt[3]&~rt[2]&~rt[1]&~rt[0];
    wire inst_bgezal   =~op[5]&~op[4]&~op[3]&~op[2]&~op[1]& op[0]& rt[4]&~rt[3]&~rt[2]&~rt[1]& rt[0];
    wire inst_bltzal   =~op[5]&~op[4]&~op[3]&~op[2]&~op[1]& op[0]& rt[4]&~rt[3]&~rt[2]&~rt[1]&~rt[0];
    
    wire inst_slti=~op[5]&~op[4]& op[3]&~op[2]& op[1]&~op[0];
    wire inst_sltiu=~op[5]&~op[4]& op[3]&~op[2]& op[1]& op[0];
    // J����ָ��
    wire inst_j=~op[5]&~op[4]&~op[3]&~op[2]& op[1]&~op[0];
    wire inst_jal=~op[5]&~op[4]&~op[3]&~op[2]& op[1]& op[0];
    /*------------------------------------------------------------------------------*/

    /*-------------------- �ڶ��������߼������ɾ�������ź� --------------------*/
    // ��������alutype
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            id_alutype_o[2] <= 1'b0; 
            id_alutype_o[1] <= 1'b0;
            id_alutype_o[0] <= 1'b0;           
        end else begin
            id_alutype_o[2] <= (inst_sll|inst_srl|inst_sra|inst_sllv|inst_srlv|inst_srav|inst_jr|inst_jalr|inst_j|inst_jal);  
            id_alutype_o[1] <= (inst_and | inst_mfhi | inst_mflo | inst_mthi | inst_mtlo | inst_or | inst_xor | inst_nor | inst_andi |inst_ori | inst_xori | inst_lui);
            id_alutype_o[0] <= (inst_add | inst_addu |inst_sub |inst_subu | inst_slt | inst_mfhi | inst_mflo | inst_mthi | inst_mtlo | inst_sltu | inst_jr | inst_jalr |
                            inst_addi| inst_addiu | inst_sltiu |inst_slti| inst_lb | inst_lbu | inst_lh | inst_lhu |inst_lw | inst_sb |inst_sh | inst_sw |
                            inst_beq|inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal|inst_j|inst_jal);
        end
    end
    assign break = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : inst_break;
    assign syscall = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : inst_syscall;
    assign eret = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : inst_eret;                          
    // �ڲ�������aluop
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            id_aluop_o[7]   <= 1'b0;
            id_aluop_o[6]   <= 1'b0;
            id_aluop_o[5]   <= 1'b0;
            id_aluop_o[4]   <= 1'b0;
            id_aluop_o[3]   <= 1'b0;
            id_aluop_o[2]   <= 1'b0;
            id_aluop_o[1]   <= 1'b0; 
            id_aluop_o[0]   <= 1'b0;         
        end else begin
            id_aluop_o[7]   <= 1'b0;
            id_aluop_o[6]   <= 1'b0;
            id_aluop_o[5]   <= (inst_addi | inst_addiu | inst_andi |inst_ori|inst_xori|inst_lui|inst_lb|inst_lbu|inst_lh|inst_lhu|inst_sb|inst_sh|inst_lw|inst_sw|inst_beq|
                             inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal|inst_slti|inst_sltiu|inst_j|inst_jal);
            id_aluop_o[4]   <= (inst_or | inst_xor | inst_nor | inst_slt | inst_sltu|inst_sll|inst_srl |inst_sra|inst_sllv|inst_srlv|inst_srav
                             |inst_jr | inst_jalr | inst_break | inst_syscall | inst_eret |inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal|inst_slti|inst_sltiu|inst_j|inst_jal);
            id_aluop_o[3]   <= (inst_div | inst_divu | inst_mfhi | inst_mflo | inst_mthi | inst_mtlo|inst_mfc0|inst_mtc0|inst_sllv|inst_srlv|inst_srav|inst_jr|inst_jalr| 
                             inst_break|inst_syscall| inst_eret |inst_lh | inst_lhu | inst_sb | inst_sh|inst_lw|inst_sw|inst_beq|inst_bne|inst_j|inst_jal);
            id_aluop_o[2]   <= (inst_subu|inst_and|inst_mult|inst_multu| inst_mthi | inst_mtlo|inst_mfc0|inst_mtc0|inst_sltu | inst_sll | inst_srl | inst_sra | 
                             inst_jalr |inst_break|inst_syscall| inst_eret |inst_xori| inst_lui |inst_lb|inst_lbu|inst_lw|inst_sw|inst_beq|inst_bne|inst_bgezal|inst_bltzal|inst_slti| inst_sltiu);
            id_aluop_o[1]   <= (inst_addu|inst_sub | inst_mult | inst_multu | inst_mfhi| inst_mflo |inst_mfc0|inst_mtc0| inst_nor|inst_slt|inst_srl|inst_sra|inst_srav|inst_jr|inst_syscall| inst_eret |inst_andi|inst_ori|inst_lb|inst_lbu|
                             inst_sb|inst_sh|inst_beq|inst_bne|inst_blez|inst_bltz|inst_slti|inst_sltiu); 
            id_aluop_o[0]   <= (inst_add|inst_sub|inst_and|inst_multu|inst_divu|inst_mflo|inst_mtlo|inst_mtc0|inst_xor|inst_slt|inst_sll|inst_sra|inst_srlv|inst_jr|inst_break|inst_eret|inst_addiu|inst_ori|inst_lui|
                             inst_lbu|inst_sh|inst_sw|inst_bne|inst_bgtz|inst_bltz|inst_bltzal| inst_sltiu|inst_jal);           
        end
    end                             
    
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            id_wreg_o <= 1'b0; 
            id_whilo_o <= 1'b0; 
            id_mreg_o  <= 1'b0;
        end else begin
            // дͨ�üĴ���ʹ���ź�
            id_wreg_o <=  (inst_add |inst_addu|inst_sub| inst_subu |inst_and|inst_mfhi|inst_mflo|inst_mtc0|inst_or|inst_xor|inst_nor| inst_slt |inst_sltu|inst_sll|inst_srl|inst_sra|inst_sllv|inst_srlv|inst_srav|
                              inst_jalr|inst_addi|inst_addiu|inst_andi| inst_ori |inst_xori| inst_lui | inst_lb|inst_lbu|inst_lh|inst_lhu|inst_lw |inst_bgezal|inst_bltzal|inst_slti| inst_sltiu | inst_jal );
            // дHILO�Ĵ���ʹ���ź�
            id_whilo_o <= (inst_mult|inst_multu|inst_div|inst_divu|inst_mthi|inst_mtlo);  
            // �洢�����Ĵ���ʹ���ź� 
            id_mreg_o  <= (inst_lb | inst_lw|inst_lbu|inst_lh|inst_lhu);               
        end
    end 
                                               
    // ��������λʹ��ָ���Чʱ������ѡ��Ϊ Ҫ��λ��������(�Ĵ�����λҲ��Ч)
    wire shift = inst_sll|inst_srl|inst_sra;
    // ������ʹ���ź�
    wire immsel = inst_addi|inst_andi|inst_ori |inst_xori| inst_lui |inst_lb|inst_lbu|inst_lh|inst_lhu|inst_sb|inst_sh| inst_addiu | inst_slti|inst_sltiu| inst_lw| inst_sw;
    // Ŀ�ļĴ���ѡ���źţ�Ŀ�ļĴ���Ϊrt(��Ϊ0��Ŀ�ļĴ���Ϊrd)
    wire rtsel  = inst_mfc0|inst_addi| inst_addiu |inst_andi|inst_ori |inst_xori| inst_lui | inst_lb| inst_lbu |inst_lh|inst_lhu| inst_lw|inst_slti| inst_sltiu ;
    // д�Ĵ�����ַѡ��31�Ĵ��������Ա��淵�ص�ַ
    wire reg31sel = inst_jal|inst_bgezal|inst_bltzal;
    // ������չʹ���źţ���Ϊ0��Ϊ����չ
    wire sext   = inst_addi|inst_addiu | inst_lb | inst_lh | inst_lw | inst_sb |inst_sh | inst_sw|inst_beq|inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal|inst_slti;
    // ���ظ߰���ʹ���ź�
    wire upper  = inst_lui;
    
    
    // ��ͨ�üĴ����Ѷ˿�1ʹ���ź�
    assign rreg1 = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : 
                   (inst_add |inst_addu|inst_sub| inst_subu| inst_and | inst_mult |  inst_multu|inst_div|inst_divu|inst_mthi|inst_mtlo|inst_or|inst_xor|inst_nor|inst_slt|inst_sltu|inst_sllv|
                   inst_srlv|inst_srav|inst_jr|inst_jalr|inst_addi|inst_addiu|inst_andi|inst_ori | inst_xori|inst_lb | inst_lbu|inst_lh|inst_lhu|inst_sb|inst_sh|inst_lw | inst_sw|inst_beq|
                   inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal|inst_slti|inst_sltiu );
    // ��ͨ�üĴ����Ѷ��˿�2ʹ���ź�
    assign rreg2 = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : 
                   (inst_add |inst_addu|inst_sub| inst_subu| inst_and | inst_mult |  inst_multu|inst_div|inst_divu|inst_mtc0|inst_or|inst_xor|inst_nor|inst_slt|inst_sltu|inst_sll|inst_srl|inst_sra|inst_sllv|
                   inst_srlv|inst_srav|inst_sb|inst_sh| inst_sw|inst_beq|inst_bne );    
    // ��ͨ�üĴ����Ѷ˿�1�ĵ�ַΪrs�ֶΣ����˿�2�ĵ�ַΪrt�ֶ�
    assign ra1   = (cpu_rst_n == `RST_ENABLE) ? 5'b00000 : rs ;
    assign ra2   = (cpu_rst_n == `RST_ENABLE) ? 5'b00000 : rt ;
    
    always @(posedge cpu_clk) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            mtc0 <= 1'b0;
            mfc0 <= 1'b0;
            cp0addr <= 5'b00000;
            mfc0addr <= 5'b00000;
        end
        else begin
            mtc0 <= inst_mtc0;
            mfc0 <= inst_mfc0;
            cp0addr <= rd;
            mfc0addr <= rt;
        end
    end
    assign PCSrc = (inst_j|inst_jal) ? 2'b11 :
                        (inst_jr|inst_jalr) ? 2'b10 :
                       (inst_beq| inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal) ? 2'b01 : 2'b00;       

    /*------------------------------------------------------------------------------*/


    
    // ���ָ���������������� 
    wire [31:0] imm_ext = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD :
                          (upper == `UPPER_ENABLE  ) ? (imm << 16) :
                          (sext  == `SIGNED_EXT    ) ? {{16{imm[15]}}, imm} : {{16{1'b0}}, imm};
                                            
    // ��ô�д��Ŀ�ļĴ����ĵ�ַ��rt��rd��
    always @(posedge cpu_clk) begin
            if (cpu_rst_n == `RST_ENABLE) id_wa_o  <=   `REG_NOP;    
            else if(rtsel == `RT_ENABLE) id_wa_o     <=    rt; 
            else if(reg31sel == 1'b1) id_wa_o     <=   (5'b11111);
            else id_wa_o     <=   rd;
    end
                   
    // ��÷ô�׶�Ҫ�������ݴ洢�������ݣ�����ͨ�üĴ����Ѷ����ݶ˿�2��
    always @(posedge cpu_clk) begin
            if (cpu_rst_n == `RST_ENABLE) id_din_o  <=   `ZERO_WORD;
            else id_din_o     <=   (ra2 == ex_wrAddr && ex_wrn == 1'b1 ) ? ex_wrData :((ra2 == mem_wrAddr && mem_wrn == 1'b1) ? mem_wrData : ((ra2 == wb_wrAddr && wb_wrn == 1'b1) ? wb_wrData : rd2));
    end   

    // ���Դ������1�����shift�ź���Ч����Դ������1Ϊ��λλ��������Ϊ�Ӷ�ͨ�üĴ����Ѷ˿�1��õ�����
    always @(posedge cpu_clk) begin
            if (cpu_rst_n == `RST_ENABLE) id_src1_o  <=   `ZERO_WORD;    
            else if(shift == `SHIFT_ENABLE) id_src1_o     <=    {27'b0, sa}; 
            else if(rreg1 == `READ_ENABLE) id_src1_o     <=   (ra1 == ex_wrAddr && ex_wrn == 1'b1 ) ? ex_wrData :((ra1 == mem_wrAddr && mem_wrn == 1'b1) ? mem_wrData : ((ra1 == wb_wrAddr && wb_wrn == 1'b1) ? wb_wrData : rd1));
            else id_src1_o     <=   `ZERO_WORD;
    end
    // ���Դ������2�����immsel�ź���Ч����Դ������1Ϊ������������Ϊ�Ӷ�ͨ�üĴ����Ѷ˿�2��õ�����
    always @(posedge cpu_clk) begin
            if (cpu_rst_n == `RST_ENABLE) id_src2_o  <=   `ZERO_WORD;    
            else if(immsel == `IMM_ENABLE) id_src2_o     <=    imm_ext; 
            else if(rreg2 == `READ_ENABLE) id_src2_o     <=   (ra2 == ex_wrAddr && ex_wrn == 1'b1 ) ? ex_wrData :((ra2 == mem_wrAddr && mem_wrn == 1'b1) ? mem_wrData : ((ra2 == wb_wrAddr && wb_wrn == 1'b1) ? wb_wrData : rd2));
            else id_src2_o     <=   `ZERO_WORD;
    end
    // 
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) src1mem_e  <= 1'b0;
        else if (mem2reg == 1'b1 && ra1 == mem_wrAddr && rreg1 == `READ_ENABLE)  src1mem_e <= 1'b1;
        else src1mem_e <= 1'b0;
    end  
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) src2mem_e  <= 1'b0;
        else if (mem2reg == 1'b1 && ra2 == mem_wrAddr && rreg2 == `READ_ENABLE)  src2mem_e <= 1'b1;
        else src2mem_e <= 1'b0;
    end                    
     // �ж�����lw����ð�գ�lw����ð��Ҫ������ͣ�ź�
    assign stall_rep = ((ra1 == ex_wrAddr && rreg1) | (ra2 == ex_wrAddr && rreg2)) & (E_Reg2reg == 0) & (ex_wrAddr != 0) & (ex_wrn == 1);
    assign Reg2reg = ~(inst_lw| inst_lb | inst_lbu | inst_lh | inst_lhu);
    // ����ð����ͣ�ź�,�жϵ�ǰָ����������תָ��ʱ������ͣ
    //assign stall_beq = inst_beq|inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal;
    
 
    // ������һ��PC��ֵ
    PCAddFour PCAddFour(id_pc_i, pc4);    
    // д31�żĴ�������ת��ַ֮ǰ
    always @(*) begin
            if (cpu_rst_n == `RST_ENABLE)  begin
                reg31data <=   `ZERO_WORD; 
                wreg31      <=   1'b0;
            end   
            else if(reg31sel == 1'b1) begin
                reg31data     <=   pc4;
                wreg31        <=   1'b1;               
            end
    end  
    PCJump PCJump(id_pc_i, add, pcChoose3);
    SignExtend SignExtend(imm, 1'b1, extendData);
    PCAddImm PCAddImm_un(pc4, extendData, 1'b0, pcChoose1);  
    PCAddImm PCAddImm_en(pc4, extendData, 1'b1, pcChoose2);  
    // ����ð����Ҫ����ת��ַ���͵�ִ�е�Ԫ�ж�
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) branchAddr <=  `PC_INIT;  
        else branchAddr <= pcChoose2;            
    end    

    // ���ݵ�ǰ��PCSrc�ź�ȷ���������һ��pcֵ����ѡһ����ѡ����
    DataSelector_4to1 DataSelector_4to1(pc4,pcChoose1,rd1, pcChoose3, PCSrc,cpu_rst_n, next_pc);
    //assign id_pc_o = id_pc_i;
    //assign id_imm = imm;
    //assign id_add = add;
endmodule


//ͨ�üĴ�����ģ��
module regfile(
    input  wire 				 cpu_clk,
	input  wire 				 cpu_rst_n,
	
	// д�˿�
	input  wire  [`REG_ADDR_BUS] wa,
	input  wire  [`REG_BUS 	   ] wd,
	input  wire 				 we,
	
	// ���˿�1
	input  wire  [`REG_ADDR_BUS] ra1,
	output reg   [`REG_BUS 	   ] rd1,
	input  wire 				 re1,
	
	// ���˿�2 
	input  wire  [`REG_ADDR_BUS] ra2,
	output reg   [`REG_BUS 	   ] rd2,
	input  wire 			     re2,
	
	// д$31�����ݺͿ����ź�
	input wire  [`REG_BUS]  reg31data,
	input wire   wreg31,
	// mfc0����cp0�Ĵ�����ֵ
	input wire [`REG_BUS] cp0rdata,
	input wire [4:0]   mfc0addr,
	input wire mfc0write
    );

    //����32��32λ�Ĵ���
	reg [`REG_BUS] 	regs[0:`REG_NUM-1];
	
	always @(posedge cpu_clk) begin
		if (cpu_rst_n == `RST_ENABLE) begin
			regs[ 0] <= 32'h00000000;
			regs[ 1] <= 32'h00000001;
			regs[ 2] <= 32'h00000002;
			regs[ 3] <= 32'h00000003;
			regs[ 4] <= 32'h00000004;
			regs[ 5] <= 32'h00000005;
			regs[ 6] <= 32'h00000006;
			regs[ 7] <= 32'h00000007;
			regs[ 8] <= 32'h00000008;
			regs[ 9] <= 32'h00000009;
			regs[10] <= 32'h0000000A;
			regs[11] <= 32'h0000000B;
			regs[12] <= 32'h0000000C;
			regs[13] <= 32'h0000000D;
			regs[14] <= 32'h0000000E;
			regs[15] <= 32'h0000000F;
			regs[16] <= 32'h00000010;
			regs[17] <= 32'h00000011;
			regs[18] <= 32'h00000012;
			regs[19] <= 32'h00000013;
			regs[20] <= 32'h00000014;
			regs[21] <= 32'h00000015;
			regs[22] <= 32'h00000016;
			regs[23] <= 32'h00000017;
			regs[24] <= 32'h00000018;
			regs[25] <= 32'h00000019;
			regs[26] <= 32'h0000001A;
			regs[27] <= 32'h0000001B;
			regs[28] <= 32'h0000001C;
			regs[29] <= 32'h0000001D;
			regs[30] <= 32'h0000001E;
			regs[31] <= 32'h0000001F;
		end
		else begin
			if ((we == `WRITE_ENABLE) && (wa != 5'b00000) && (wa != 5'b11111))	
				regs[wa] <= wd;
		    else if(wreg31 == 1'b1) regs[31] <= reg31data;
		    else if(mfc0write)  regs[mfc0addr] <= cp0rdata;
		end
	end
	
	
	//���˿�1�Ķ����� 
	// ra1�Ƕ���ַ��wa��д��ַ��we��дʹ�ܡ�wd��Ҫд������� 
	always @(*) begin
		if (cpu_rst_n == `RST_ENABLE)
			rd1 <= `ZERO_WORD;
		else if (ra1 == `REG_NOP)
			rd1 <= `ZERO_WORD;
		else if (re1 == `READ_ENABLE)
			rd1 <= regs[ra1];
		else
			rd1 <= `ZERO_WORD;
	end
	
	//���˿�2�Ķ����� 
	// ra2�Ƕ���ַ��wa��д��ַ��we��дʹ�ܡ�wd��Ҫд������� 
	always @(*) begin
		if (cpu_rst_n == `RST_ENABLE)
			rd2 <= `ZERO_WORD;
		else if (ra2 == `REG_NOP)
			rd2 <= `ZERO_WORD;
		else if (re2 == `READ_ENABLE)
			rd2 <= regs[ra2];
		else
			rd2 <= `ZERO_WORD;
	end

endmodule

//�üĴ�����������׶ε������źţ����͸�ִ�н׶Σ���������ˮ��ָ��
module idexe_reg (
    input  wire 				  cpu_clk,
    input  wire 				  cpu_rst_n,

    // ��������׶ε���Ϣ
    input  wire [`ALUTYPE_BUS  ]  id_alutype,
    input  wire [`ALUOP_BUS    ]  id_aluop,
    input  wire [`REG_BUS      ]  id_src1,
    input  wire [`REG_BUS      ]  id_src2,
    input wire src1mem_e,
    input wire src2mem_e,
    input  wire [`REG_ADDR_BUS ]  id_wa,
    input  wire                   id_wreg,
    input  wire                   id_mreg,
    input  wire [`REG_BUS      ]  id_din,
    input  wire                   id_whilo,
    input wire Reg2reg,
    input wire[5:0] stall,
    input wire clean,   // ID/EXE�Ĵ�������ź�
    input wire [`INST_ADDR_BUS]  id_branchAddr,
    // lw����ð�ղ���������ѡ��
    input wire [`REG_BUS]  mem_src,    // MEM�δӴ洢������������

    //input wire [`INST_ADDR_BUS]    id_pc_o,  // ����׶��������һ��ѡ���pc��IF
    //input wire [15:0 ] id_imm,
    //input wire [25:0] id_add,
    // ����ִ�н׶ε���Ϣ
    output reg  [`ALUTYPE_BUS  ]  exe_alutype,
    output reg  [`ALUOP_BUS    ]  exe_aluop,
    output reg  [`REG_BUS      ]  exe_src1,
    output reg  [`REG_BUS      ]  exe_src2,
    output reg  [`REG_ADDR_BUS ]  exe_wa,
    output reg                    exe_wreg,
    output reg                    exe_mreg,
    output reg  [`REG_BUS      ]  exe_din,
    output reg                    exe_whilo,
    output reg  E_Reg2reg,
    //output reg [`INST_ADDR_BUS]    if_pc_i,
    output reg [`INST_ADDR_BUS]  exe_branchAddr
    //output reg [15:0 ]  exe_imm,
    //output reg [25:0]   exe_add
    );
    
    always @(negedge cpu_clk) begin
        // ��λ��ʱ������ִ�н׶ε���Ϣ��0
        if (cpu_rst_n == `RST_ENABLE) begin
            exe_alutype 	   <= `NOP;
            exe_aluop 		   <= `MIPS32_SLL;
            exe_src1 		   <= `ZERO_WORD;
            exe_src2 		   <= `ZERO_WORD;
            exe_wa 			   <= `REG_NOP;
            exe_wreg    		   <= `WRITE_DISABLE;
            exe_mreg 		   <= `FALSE_V;
            exe_din 		   <= `ZERO_WORD;
            exe_whilo          <= `WRITE_DISABLE;
            E_Reg2reg           <= 1'b1;
            exe_branchAddr      <= `PC_INIT;
        end
        // ����������׶ε���Ϣ�Ĵ沢����ִ�н׶�
        else begin
            if(clean == 1'b1) begin
                exe_alutype 	   <= `NOP;
                exe_aluop            <= `MIPS32_SLL;
                exe_src1            <= `ZERO_WORD;
                exe_src2            <= `ZERO_WORD;
                exe_wa                <= `REG_NOP;
                exe_wreg               <= `WRITE_DISABLE;
                exe_mreg            <= `FALSE_V;
                exe_din            <= `ZERO_WORD;
                exe_whilo          <= `WRITE_DISABLE;
                E_Reg2reg           <= 1'b1;
                exe_branchAddr      <= `PC_INIT;  
            end
            else if(stall[2] == `ENABLE && stall[3] == `DISABLE)
            begin
                exe_alutype 	   <= `NOP;
                exe_aluop            <= `MIPS32_SLL;
                exe_src1            <= `ZERO_WORD;
                exe_src2            <= `ZERO_WORD;         
                exe_wa                <= `REG_NOP;
                exe_wreg               <= `WRITE_DISABLE;
                exe_mreg            <= `FALSE_V;
                exe_din            <= `ZERO_WORD;
                exe_whilo          <= `WRITE_DISABLE;
                E_Reg2reg           <= 1'b1;
                exe_branchAddr      <= `PC_INIT;       
            end
            else if((stall[2] == `DISABLE)) 
            begin
                exe_alutype 	   <= id_alutype;
                exe_aluop 		   <= id_aluop;
                exe_src1 		   <= src1mem_e ? mem_src : id_src1;
                exe_src2 		   <= (src2mem_e && id_aluop != `MIPS32_SW && id_aluop != `MIPS32_SB && id_aluop != `MIPS32_SH) ? mem_src : id_src2;          
                exe_wa 			   <= id_wa;
                exe_wreg			   <= id_wreg;
                exe_mreg 		   <= id_mreg;
                exe_din 		   <= src2mem_e ? mem_src :id_din;
                exe_whilo          <= id_whilo;
                E_Reg2reg           <= Reg2reg;
                exe_branchAddr      <= id_branchAddr;
            end
        end
    end

endmodule


// ִ��ģ�飬����aluopȷ��ָ�����ʹӶ����в�ͬ����
module exe_stage (
    input  wire 					cpu_rst_n,
    input  wire                     cpu_clk,
    // ������׶λ�õ���Ϣ
    input  wire [`ALUTYPE_BUS	] 	exe_alutype_i,
    input  wire [`ALUOP_BUS	    ] 	exe_aluop_i,
    input  wire [`REG_BUS 		] 	exe_src1_i, //������1 32λ
    input  wire [`REG_BUS 		] 	exe_src2_i,  //������2 32λ
    input  wire [`REG_ADDR_BUS 	] 	exe_wa_i,  //ָ��Ŀ�ļĴ�����ַ
    input  wire 					exe_wreg_i, //Ŀ�ļĴ���дʹ����Ч
    input  wire 					exe_mreg_i,  //�洢�����Ĵ�����Ч
    input  wire [`REG_BUS 		] 	exe_din_i, // ����ִ�н׶δ�д�����ݴ洢��������
    input  wire                     exe_whilo_i, //����ִ�н׶�hilo�Ĵ���дʹ��
    input wire Reg2reg,
    input wire [`INST_ADDR_BUS]   exe_branchAddr,

    // ��HILO�Ĵ�����õ����� 
    input  wire [`REG_BUS 		] 	hi_i, //����hi������
    input  wire [`REG_BUS 		] 	lo_i, //����lo������
    output wire [`REG_ADDR_BUS ] ex_wrAddr,   // EX������Ҫд�صĵ�ַ��5λ������˿�
    output wire [`REG_BUS      ] ex_wrData,    // EX��Ҫд�ص����ݣ�32λ������˿�
    output wire ex_wrn,   //EX�����ݵ�д�źţ�һλ������˿�
    output wire E_Reg2reg,  //LW�ź�Ϊ0��ִ�н׶��ͻ��ж���һ��ָ��Ϊlw
    output wire OF,  // ����CP0�ж����޷������
    // ����ִ�н׶ε���Ϣ
    output reg [`ALUOP_BUS	    ] 	exe_aluop_o,
    output reg [`REG_ADDR_BUS 	] 	exe_wa_o,
    output reg 					exe_wreg_o,
    output reg [`REG_BUS 		] 	exe_wd_o,
    //output wire [`REG_BUS 		]   exe_wd_ohi,
    output reg 					exe_mreg_o,
    output reg [`REG_BUS 		] 	exe_din_o,
    output wire 					exe_whilo_o,
    output reg [`REG_BUS] 	exe_hi_o,
    output reg [`REG_BUS]   exe_lo_o,
    // ��������׶ε���Ϣ
    output reg en_branch,
    output reg [`INST_ADDR_BUS] branchAddr
    );
    // �����Ƿ���load-useð��ѡ����Ӧ��������Դ
    // ֱ�Ӵ�����һ�׶�
    
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            exe_aluop_o        <= 8'b0;
            exe_mreg_o            <= 1'b0;
            exe_din_o            <= 32'b0;
        end
        else begin
            exe_aluop_o        <= exe_aluop_i;
            exe_mreg_o            <= exe_mreg_i;
            exe_din_o            <= exe_din_i;   //��д�����ݴ洢��������        
        end    
    end
    // ֱ�Ӵ����˳���HILO�Ĵ���д�ź�
    assign exe_whilo_o  = (cpu_rst_n == `RST_ENABLE) ? 1'b0 :exe_whilo_i;
    wire [`REG_BUS       ]      logicres;       // �����߼�����Ľ��
    wire [`REG_BUS       ]      shiftres;       // ������λ������
    wire [`REG_BUS       ]      moveres;        // �����ƶ������Ľ��
    wire [`REG_BUS       ]      jumpresult;    // ����������ת����������
    wire [`REG_BUS       ]      judgmentresult;  // ����Ƚ�ָ��ļ�����

    wire [`REG_BUS       ]      hi_t;           // ����HI�Ĵ���������ֵ
    wire [`REG_BUS       ]      lo_t;           // ����LO�Ĵ���������ֵ
    wire [`REG_BUS       ]      arithres;       // �������������Ľ��
    reg [`REG_BUS       ]      mult_src1;
    reg [`REG_BUS       ]      mult_src2;
    wire [`DOUBLE_REG_BUS]      mulres;         // ����˷������Ľ��
    wire [`DOUBLE_REG_BUS]      mulres_u;       // �޷��ų˵Ľ��
    wire [`DOUBLE_REG_BUS]     divures;         // �޷��ų��Ľ��
    wire [`DOUBLE_REG_BUS]     divres;          // �з��ų��Ľ��
    wire exe_en_branch;         // ��������������õ����Ƿ���ת�ź�
    wire zero;        //  �����������Ƿ�Ϊ0�������жϴ�С
    wire SF;     //  �����������ķ���λ
    // �����ڲ�������aluop�����߼�����
    assign logicres = (cpu_rst_n == `RST_ENABLE)  ? `ZERO_WORD : 
                      (exe_aluop_i == `MIPS32_AND )  ? (exe_src1_i & exe_src2_i) : 
                      (exe_aluop_i == `MIPS32_ORI )  ? (exe_src1_i | exe_src2_i) : 
                      (exe_aluop_i == `MIPS32_XOR ) ?  (exe_src1_i ^ exe_src2_i):
                      (exe_aluop_i == `MIPS32_NOR ) ?  ~(exe_src1_i | exe_src2_i):
                      (exe_aluop_i == `MIPS32_OR ) ?  (exe_src1_i | exe_src2_i):
                      (exe_aluop_i == `MIPS32_XORI ) ?  (exe_src1_i ^ exe_src2_i):
                      (exe_aluop_i == `MIPS32_LUI )  ? exe_src2_i : `ZERO_WORD;

    // �����ڲ�������aluop������λ����
    wire [63:0] sradata;
    assign sradata = { {32{exe_src2_i[31]}} ,exe_src2_i[31:0]} >> exe_src1_i;
    assign shiftres = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : 
                      (exe_aluop_i == `MIPS32_SRL )  ? (exe_src2_i >> exe_src1_i) :
                      (exe_aluop_i == `MIPS32_SRLV )  ? (exe_src2_i >> exe_src1_i) :
                      (exe_aluop_i == `MIPS32_SRA )  ? sradata[31:0] :
                      (exe_aluop_i == `MIPS32_SRAV )  ? sradata[31:0] :
                      (exe_aluop_i == `MIPS32_SLL )  ? (exe_src2_i << exe_src1_i) : `ZERO_WORD;
    
    // �����ڲ�������aluop���������ƶ����õ����µ�HI��LO�Ĵ�����ֵ
    assign hi_t     = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : hi_i;
    assign lo_t     = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : lo_i;
    assign moveres  = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : 
                      (exe_aluop_i == `MIPS32_MFHI) ? hi_t :
                      (exe_aluop_i == `MIPS32_MFLO) ? lo_t : `ZERO_WORD;
    
    // �����ڲ�������aluop������������    
    assign judgmentresult = (cpu_rst_n == `RST_ENABLE) ? 32'h7FFFFFFF :
                            (exe_aluop_i ==  `MIPS32_SLT|exe_aluop_i ==`MIPS32_SLTI|exe_aluop_i ==  `MIPS32_SLTU|exe_aluop_i ==  `MIPS32_SLTIU ) ? (exe_src1_i + (~exe_src2_i) + 1) :  32'h7FFFFFFF;               
    wire judgmentzero,judgmentSF;
    assign judgmentzero =  (judgmentresult ? 0 : 1);                 
    assign judgmentSF = judgmentresult[31];  
    assign arithres = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : 
                      (exe_aluop_i == `MIPS32_ADD  )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_ADDU  )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_ADDI  )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_ADDIU  )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_SUB  )  ? (exe_src1_i + (~exe_src2_i) + 1) :
                      (exe_aluop_i == `MIPS32_SUBU  )  ? (exe_src1_i + (~exe_src2_i) + 1) :
                      (exe_aluop_i == `MIPS32_LB   )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_LBU   )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_LH   )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_LHU   )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_LW   )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_SB   )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_SH   )  ? (exe_src1_i + exe_src2_i) :
                      (exe_aluop_i == `MIPS32_SW   )  ? (exe_src1_i + exe_src2_i) :   
                      (exe_aluop_i == `MIPS32_SLT |exe_aluop_i == `MIPS32_SLTI|exe_aluop_i ==  `MIPS32_SLTU|exe_aluop_i ==  `MIPS32_SLTIU  )  ? ((judgmentSF) ? 32'h00000001 : 32'h00000000) : `ZERO_WORD;
    assign OF = (exe_aluop_i == `MIPS32_ADD || exe_aluop_i == `MIPS32_ADDI || exe_aluop_i == `MIPS32_SUB) ? 
                            (exe_src1_i[31]^arithres[31]) & (exe_src2_i[31]^arithres[31]) : 1'b0;
    // PC������ת���ָ���
    assign jumpresult = (cpu_rst_n == `RST_ENABLE) ? 32'h7FFFFFFF :
                        (exe_aluop_i ==  `MIPS32_BEQ|exe_aluop_i ==`MIPS32_BNE) ? (exe_src1_i + (~exe_src2_i) + 1) :
                        (exe_aluop_i ==  `MIPS32_BGEZ|exe_aluop_i ==  `MIPS32_BGTZ|exe_aluop_i ==  `MIPS32_BLEZ|exe_aluop_i ==  `MIPS32_BLTZ|exe_aluop_i ==  `MIPS32_BGEZAL|exe_aluop_i ==  `MIPS32_BLTZAL) ? exe_src1_i :32'h7FFFFFFF ;
    assign zero = (jumpresult ? 0 : 1);
    assign SF = jumpresult[31];
    assign exe_en_branch = (cpu_rst_n == `RST_ENABLE) ? 1'b0 :
                           (exe_aluop_i ==  `MIPS32_BEQ ) ? ((zero) ? 1'b1 : 1'b0) :
                            (exe_aluop_i ==  `MIPS32_BNE ) ? ((zero) ? 1'b0 : 1'b1) :
                             (exe_aluop_i ==  `MIPS32_BGEZ ) ? ((SF) ? 1'b0 : 1'b1) :
                              (exe_aluop_i ==  `MIPS32_BGTZ ) ? ((SF|zero) ? 1'b0 : 1'b1) :
                               (exe_aluop_i ==  `MIPS32_BEQ ) ? ((SF|zero) ? 1'b1 : 1'b0) :
                                (exe_aluop_i ==  `MIPS32_BLTZ ) ? ((SF) ? 1'b1 : 1'b0) :  1'b0;
                              
    /*always @ (exe_PCSrc or exe_aluop_i)
    begin
    case (exe_aluop_i)
    `MIPS32_BEQ: exe_PCSrc_now = (zero) ? 1'b1 : 1'b0;
    `MIPS32_BNE: exe_PCSrc_now = (zero) ? 2'b00 : 2'b01;
    `MIPS32_BGEZ: exe_PCSrc_now = (SF) ? 2'b00 : 2'b01;
    `MIPS32_BGTZ: exe_PCSrc_now = (SF|zero) ? 2'b00 : 2'b01;
    `MIPS32_BLEZ: exe_PCSrc_now = (SF|zero) ? 2'b01 : 2'b00;
    `MIPS32_BLTZ: exe_PCSrc_now = (SF) ? 2'b01 : 2'b00;
    endcase
    end
    always @(exe_PCSrc_now or pc4 or pcChoose1 or exe_src1_i or pcChoose3) begin
        case(exe_PCSrc_now)
            2'b00: exe_pc_next = pc4;   //  pc=pc+4
            2'b01: exe_pc_next = pcChoose1;   //  ������ת
            2'b10: exe_pc_next = exe_src1_i;   //  pc= (rs)
            2'b11: exe_pc_next = pcChoose3;   //   jump���ָ��
            default: exe_pc_next = exe_pc_next;
        endcase
    end*/
    
    wire s_axis_divisor_tvalid_u, s_axis_dividend_tvalid_u;
    wire s_axis_divisor_tvalid, s_axis_dividend_tvalid;
    assign  s_axis_divisor_tvalid_u = (cpu_rst_n == `RST_ENABLE) ? 1'b0 :
                        (exe_aluop_i == `MIPS32_DIVU) ? 1'b1 : 1'b0;
    assign s_axis_dividend_tvalid_u = (cpu_rst_n == `RST_ENABLE) ? 1'b0 :
                        (exe_aluop_i == `MIPS32_DIVU) ? 1'b1 : 1'b0;
    assign  s_axis_divisor_tvalid = (cpu_rst_n == `RST_ENABLE) ? 1'b0 :
                        (exe_aluop_i == `MIPS32_DIV) ? 1'b1 : 1'b0;
    assign s_axis_dividend_tvalid = (cpu_rst_n == `RST_ENABLE) ? 1'b0 :
                        (exe_aluop_i == `MIPS32_DIV) ? 1'b1 : 1'b0;                      
    wire  m_axis_dout_tvalid,m_axis_dout_tuser;    
    wire  m_axis_dout_tvalid_u,m_axis_dout_tuser_u;                
    // �����ڲ�������aluop���г˷����㣬������������һ�׶�
    always @(*) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            mult_src1        <= 8'h00000000;
            mult_src2            <= 8'h00000000;
        end
        else begin
            if (exe_aluop_i == `MIPS32_MULT || exe_aluop_i == `MIPS32_MULTU || exe_aluop_i == `MIPS32_DIVU || exe_aluop_i == `MIPS32_DIV) begin
                 mult_src1        <= exe_src1_i;
                 mult_src2            <= exe_src2_i;
            end         
        end    
    end
    multu multu(.A(mult_src1),.B(mult_src2),.P(mulres_u));
    mult mult(.A(mult_src1),.B(mult_src2),.P(mulres));
    divu divu (                                    
          .s_axis_divisor_tvalid(s_axis_divisor_tvalid_u),    // input wire s_axis_divisor_tvalid
          .s_axis_divisor_tdata(mult_src2),      // input wire [31 : 0] s_axis_divisor_tdata
          .s_axis_dividend_tvalid(s_axis_dividend_tvalid_u),  // input wire s_axis_dividend_tvalid
          .s_axis_dividend_tdata(mult_src1),    // input wire [31 : 0] s_axis_dividend_tdata
          .m_axis_dout_tvalid(m_axis_dout_tvalid_u),          // output wire m_axis_dout_tvalid
          .m_axis_dout_tuser(m_axis_dout_tuser_u),            // output wire [0 : 0] m_axis_dout_tuser
          .m_axis_dout_tdata(divures)            // output wire [63 : 0] m_axis_dout_tdata
        );
    div div (
          .s_axis_divisor_tvalid(s_axis_divisor_tvalid),    // input wire s_axis_divisor_tvalid
          .s_axis_divisor_tdata(mult_src2),      // input wire [31 : 0] s_axis_divisor_tdata
          .s_axis_dividend_tvalid(s_axis_dividend_tvalid),  // input wire s_axis_dividend_tvalid
          .s_axis_dividend_tdata(mult_src1),    // input wire [31 : 0] s_axis_dividend_tdata
          .m_axis_dout_tvalid(m_axis_dout_tvalid),          // output wire m_axis_dout_tvalid
          .m_axis_dout_tuser(m_axis_dout_tuser),            // output wire [0 : 0] m_axis_dout_tuser
          .m_axis_dout_tdata(divres)            // output wire [63 : 0] m_axis_dout_tdata
        );    
    //assign mulres = ($signed(exe_src1_i) * $signed(exe_src2_i));
    //assign mulres_u = exe_src1_i*exe_src2_i;
    //wire divres_hi = ($signed(exe_src1_i) % $signed(exe_src2_i));
    //wire divres_lo = ($signed(exe_src1_i) / $signed(exe_src2_i));
    //wire divres_hi_u = exe_src1_i % exe_src2_i;
    //wire divres_lo_u = exe_src1_i / exe_src2_i;
    always @(*) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            exe_hi_o    <=  `ZERO_DWORD;    
            exe_lo_o    <=  `ZERO_DWORD;
        end   
        else if(exe_aluop_i == `MIPS32_DIV) begin
            exe_hi_o    <=  divres[63:32];
            exe_lo_o      <=  divres[31:0];
        end
        else if(exe_aluop_i == `MIPS32_MULTU) begin
            exe_hi_o    <=  mulres_u[63:32];
            exe_lo_o      <=  mulres_u[31:0];
        end
        else if(exe_aluop_i == `MIPS32_DIVU) begin
            exe_hi_o    <=  divures[63:32];
            exe_lo_o      <=  divures[31:0];
        end
        else if(exe_aluop_i == `MIPS32_MULT) begin
            exe_hi_o    <=  mulres[63:32];
            exe_lo_o      <=  mulres[31:0];
        end   
        else if(exe_aluop_i == `MIPS32_MTHI) begin
            exe_hi_o    <= exe_src1_i;
        end
        else if(exe_aluop_i == `MIPS32_MTLO) begin
            exe_lo_o    <= exe_src1_i;
        end                            
        else  begin
            exe_hi_o    <=  `ZERO_DWORD;    
            exe_lo_o    <=  `ZERO_DWORD;
        end   
    end
    
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            exe_wa_o        <= 5'b0;
            exe_wreg_o      <= 1'b0;
        end
        else begin
            // ����洢д�ؽ׶εĴ�д�Ĵ�����ַ
            exe_wa_o        <= exe_wa_i;
            // д�Ĵ���ʹ��
            exe_wreg_o      <= exe_wreg_i;         
        end    
    end
    
    
    // ���ݲ�������alutypeȷ��ִ�н׶����յ����������ȿ����Ǵ�д��Ŀ�ļĴ��������ݣ�Ҳ�����Ƿ������ݴ洢���ĵ�ַ��
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) exe_wd_o    <=  `ZERO_DWORD;
        else if(exe_alutype_i == `LOGIC) exe_wd_o    <=  logicres;
        else if(exe_alutype_i == `SHIFT) exe_wd_o    <=  shiftres;
        else if(exe_alutype_i == `MOVE)  exe_wd_o    <=  moveres;
        else if(exe_alutype_i == `ARITH)  exe_wd_o    <= arithres;
        else if(exe_alutype_i == `JUMP)  exe_wd_o    <=  jumpresult;
        else  exe_wd_o    <=  `ZERO_DWORD;       
    end    
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            en_branch   <=  1'b0;
            branchAddr  <=  `PC_INIT;
        end
        else begin
            en_branch   <= exe_en_branch;
            branchAddr  <= exe_branchAddr;
        end            
    end    
    
    // �������ð����ID��������
    assign E_Reg2reg = (cpu_rst_n == `RST_ENABLE) ? 1'b1 : Reg2reg;
    assign ex_wrAddr = (cpu_rst_n == `RST_ENABLE) ? 5'b0 : exe_wa_i;
    assign ex_wrData =  (cpu_rst_n   == `RST_ENABLE ) ? `ZERO_WORD : 
                        (exe_alutype_i == `LOGIC    ) ? logicres  :
                        (exe_alutype_i == `SHIFT    ) ? shiftres  :
                        (exe_alutype_i == `MOVE     ) ? moveres   :
                        (exe_alutype_i == `ARITH    ) ? arithres  : 
                        (exe_alutype_i == `JUMP) ? jumpresult :`ZERO_WORD;
    assign ex_wrn = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : exe_wreg_i;             

endmodule

    // ����ִ�н׶ε���Ϣ
// hilo�Ĵ���ģ�飬���ڳ˳�����MOV  hiloָ��

module hilo (
    input  wire            cpu_clk,
    input  wire            cpu_rst_n,

    // д�˿� 
    input  wire            we,
    input  wire [`REG_BUS] hi_i,
    input  wire [`REG_BUS] lo_i,
    
    // ���˿� 
    output reg  [`REG_BUS] hi_o,
    output reg  [`REG_BUS] lo_o
    );
    reg [`REG_BUS] 	reghi;
    reg [`REG_BUS] 	reglo;
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            reghi <= `ZERO_WORD;
            reglo <= `ZERO_WORD;
        end
        else if (we == `WRITE_ENABLE)begin
            reghi <= hi_i;            // ���˷����mulres��ǰ32λ��HI�Ĵ�����
            reglo <= lo_i;            // ��32λ��lo�Ĵ���
        end
    end
	// ra1�Ƕ���ַ��wa��д��ַ��we��дʹ�ܡ�wd��Ҫд������� 
    always @(*) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            hi_o <= `ZERO_WORD;
            lo_o <= `ZERO_WORD;
        end
        else begin
            hi_o <= reghi;
            lo_o <= reglo;
        end
    end
endmodule



// ִ��/�ô�Ĵ���ģ�飬����źŵĴ���

module exemem_reg (
    input  wire 				cpu_clk,
    input  wire 				cpu_rst_n,

    // ����ִ�н׶ε���Ϣ
    input  wire [`ALUOP_BUS   ] exe_aluop,
    input  wire [`REG_ADDR_BUS] exe_wa,
    input  wire                 exe_wreg,
    input  wire [`REG_BUS 	  ] exe_wd,
    input  wire                 exe_mreg,
    input  wire [`REG_BUS 	  ] exe_din,
    
    //input  wire                 exe_whilo,
    //input  wire [`DOUBLE_REG_BUS] exe_hilo,
    //input  wire exe_en_branch,   //  ִ�н׶��ж��Ƿ���ת�ź�
    //input  wire E_Reg2reg_i,
    //output reg [`REG_ADDR_BUS ] ex_wrAddr,   // EX������Ҫд�صĵ�ַ��5λ������˿�
    //output reg [`REG_BUS      ] ex_wrData,    // EX��Ҫд�ص����ݣ�32λ������˿�
    //output reg ex_wrn,   //EX�����ݵ�д�źţ�һλ������˿�
    //output reg E_Reg2reg_o,
    // �͵�����׶ε���Ϣ
    // output reg en_branch,
    // �͵��ô�׶ε���Ϣ 
    output reg  [`ALUOP_BUS   ] mem_aluop,
    output reg  [`REG_ADDR_BUS] mem_wa,
    output reg                  mem_wreg,
    output reg  [`REG_BUS 	  ] mem_wd,
    output reg                  mem_mreg,
    output reg  [`REG_BUS 	  ] mem_din
    
    //output reg 					mem_whilo,
    //output reg  [`DOUBLE_REG_BUS] mem_hilo
    );
    always @(negedge cpu_clk) begin
    if (cpu_rst_n == `RST_ENABLE) begin
        mem_aluop              <= `MIPS32_SLL;
        mem_wa 				   <= `REG_NOP;
        mem_wreg   			   <= `WRITE_DISABLE;
        mem_wd   			   <= `ZERO_WORD;
        mem_mreg  			   <= `WRITE_DISABLE;
        mem_din   			   <= `ZERO_WORD;
        //mem_whilo 			   <= `WRITE_DISABLE;
        //mem_hilo     		   <= `ZERO_DWORD;
    end
    else begin
        mem_aluop              <= exe_aluop;
        mem_wa 				   <= exe_wa;
        mem_wreg 			   <= exe_wreg;
        mem_wd 		    	   <= exe_wd;
        mem_mreg  			   <= exe_mreg;
        mem_din   			   <= exe_din;
        //mem_whilo 			   <= exe_whilo;
        //mem_hilo     		   <= exe_hilo;
    end
  end

endmodule


// �ô�ģ�飬���ڷô�ָ��Ҫ��ɼ��غʹ洢���������ڷǷô�ָ�ֱ�ӰѴ�ִ�н׶ε���Ϣֱ�Ӵ��ݸ���һ������

module mem_stage (
    input  wire             cpu_rst_n,
    input  wire             cpu_clk,
    // ��ִ�н׶λ�õ���Ϣ
    input  wire [`ALUOP_BUS     ]       mem_aluop_i,
    input  wire [`REG_ADDR_BUS  ]       mem_wa_i,
    input  wire                         mem_wreg_i,
    input  wire [`REG_BUS       ]       mem_wd_i,

    input  wire                         mem_mreg_i,
    input  wire [`REG_BUS       ]       mem_din_i,
    
    input  wire                         mem_whilo_i,
    input  wire [`DOUBLE_REG_BUS]       mem_hilo_i,

    // ����д�ؽ׶ε���Ϣ
    output reg [`REG_ADDR_BUS  ]       mem_wa_o,
    output reg                         mem_wreg_o,
    output reg [`REG_BUS       ]       mem_dreg_o,


    output reg                         mem_mreg_o,
    output reg [`BSEL_BUS      ]       dre,   // ���ݴ洢���Ķ�ʹ���źţ�4λ���Ʋ�ͬ�ֽ�
    output reg   sext,  // ���ƴӴ洢���ж��������ݽ��з�����չ
    output reg                         mem_whilo_o,
    output reg [`DOUBLE_REG_BUS]       mem_hilo_o,

    // �������ݴ洢�����ź�
    output reg                         dce,   // ���ݴ洢����ʹ���ź�
    output wire [`INST_ADDR_BUS ]       daddr,   // ���ݴ洢���ķô��ַ
    output reg [`BSEL_BUS      ]       we,    // ���ݴ洢����д�ֽ�ʹ���ź�
    output reg [`REG_BUS       ]       din, //  ���ݴ洢��Ҫд�������
    
    // ����IO�������ź�
    output reg [`REG_BUS       ]      IOwdata, // IOҪд�������
    output reg [`INST_ADDR_BUS ]      IOwaddr, // IOҪд��ĵ�ַ
    output wire IORead,
    output wire IOWrite,
    // ��������׶ν������ð��
    output wire [`REG_ADDR_BUS ] mem_wrAddr,   
    output wire [`REG_BUS      ] mem_wrData,  
    output wire mem_wrn,
    output wire mem2reg,
    output reg IO2reg   
    );

    // �����ǰ���Ƿô�ָ���ֻ��Ҫ�Ѵ�ִ�н׶λ�õ���Ϣֱ�����
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            mem_wa_o              <= 5'b0;
            mem_wreg_o            <= 1'b0;
            mem_dreg_o            <= 1'b0;
            mem_whilo_o           <= 1'b0;
            mem_hilo_o            <= 64'b0;
            mem_mreg_o            <= 1'b0;
        end
        else begin
            mem_wa_o              <= mem_wa_i;
            mem_wreg_o            <= mem_wreg_i;
            mem_dreg_o            <= mem_wd_i;
            mem_whilo_o           <= mem_whilo_i;
            mem_hilo_o            <= mem_hilo_i;
            mem_mreg_o            <= mem_mreg_i;
        end    
    end

    // ȷ����ǰ�ķô�ָ��
    wire inst_lb = (mem_aluop_i == 8'h26);
    wire inst_lbu = (mem_aluop_i == 8'h27);
    wire inst_lh = (mem_aluop_i == 8'h28);
    wire inst_lhu = (mem_aluop_i == 8'h29);
    wire inst_sb = (mem_aluop_i == 8'h2A);
    wire inst_sh = (mem_aluop_i == 8'h2B);
    wire inst_lw = (mem_aluop_i == 8'h2C);
    wire inst_sw = (mem_aluop_i == 8'h2D);

    // ������ݴ洢���ķ��ʵ�ַ
    assign daddr = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : mem_wd_i;
    // ����I/O��ؿ����ź�
    wire [21:0] daddrHigh;
    assign daddrHigh = daddr[31:10];
    wire MemWrite,MemRead;
    assign MemWrite = (cpu_rst_n == `RST_ENABLE) ? 1'b0 :(daddrHigh != 22'b1111111111111111111111);
    assign MemRead = (cpu_rst_n == `RST_ENABLE) ? 1'b0 :(daddrHigh != 22'b1111111111111111111111);
    assign  IORead = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : (inst_lb |inst_lbu|inst_lh|inst_lhu| inst_lw) & (daddrHigh == 22'b1111111111111111111111) ;
    assign  IOWrite = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : ( inst_sb |inst_sh| inst_sw) & (daddrHigh == 22'b1111111111111111111111);
    always @(posedge cpu_clk) begin
        if(cpu_rst_n == `RST_ENABLE)
            IO2reg <= 1'b0;
        else
            IO2reg <= (inst_lb |inst_lbu|inst_lh|inst_lhu| inst_lw) & (daddrHigh == 22'b1111111111111111111111);
    end
    // ������ݴ洢�����ֽ�ʹ���ź�
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            dre[0]            <= 1'b0;
            dre[1]            <= 1'b0;
            dre[2]            <= 1'b0;
            dre[3]            <= 1'b0;
            sext              <= 1'b0;
        end
        else begin
            dre[0]            <= ((((inst_lb|inst_lbu) & (daddr[1 : 0] == 2'b00)) | inst_lw|((inst_lh|inst_lhu) & (daddr[1 : 0] == 2'b00)))) ;
            dre[1]            <= ((((inst_lb|inst_lbu) & (daddr[1 : 0] == 2'b01)) | inst_lw|((inst_lh|inst_lhu) & (daddr[1 : 0] == 2'b00)))) ;
            dre[2]            <= ((((inst_lb|inst_lbu) & (daddr[1 : 0] == 2'b10)) | inst_lw|((inst_lh|inst_lhu) & (daddr[1 : 0] == 2'b10)))) ;
            dre[3]            <= ((((inst_lb|inst_lbu) & (daddr[1 : 0] == 2'b11)) | inst_lw|((inst_lh|inst_lhu) & (daddr[1 : 0] == 2'b10)))) ;
            sext              <= (inst_lb|inst_lh);
        end    
    end
    // �������ݴ洢��ֵд��
     always @(*) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            we[0]            <= 1'b0;
            we[1]            <= 1'b0;
            we[2]            <= 1'b0;
            we[3]            <= 1'b0;
            dce              <= 1'b0;
        end
        else begin
            // ������ݴ洢��д�ֽ�ʹ���ź�
            we[0]            <= (((inst_sb & (daddr[1 : 0] == 2'b00)) | inst_sw |inst_sh & (daddr[1 : 0] == 2'b00))) & MemWrite;
            we[1]            <= (((inst_sb & (daddr[1 : 0] == 2'b01)) | inst_sw |inst_sh & (daddr[1 : 0] == 2'b00))) & MemWrite;
            we[2]            <= (((inst_sb & (daddr[1 : 0] == 2'b10)) | inst_sw |inst_sh & (daddr[1 : 0] == 2'b10))) & MemWrite;
            we[3]            <= (((inst_sb & (daddr[1 : 0] == 2'b11)) | inst_sw |inst_sh & (daddr[1 : 0] == 2'b10))) & MemWrite;
            // ������ݴ洢��ʹ���ź�
            dce              <= (inst_lb |inst_lbu|inst_lh|inst_lhu| inst_lw | inst_sb |inst_sh| inst_sw) & (MemWrite| MemRead);
        end    
    end             

    // ȷ����д�����ݴ洢��������
    wire [`WORD_BUS] din_reverse = mem_din_i; //{mem_din_i[7:0], mem_din_i[15:8], mem_din_i[23:16], mem_din_i[31:24]};
    wire [`WORD_BUS] din_byte    = {mem_din_i[7:0], mem_din_i[7:0], mem_din_i[7:0], mem_din_i[7:0]};
    wire [`WORD_BUS] din_2byte    = {mem_din_i[15:0], mem_din_i[15:0]};
    always @(*) begin
       if (cpu_rst_n == `RST_ENABLE) din <= `ZERO_WORD ;
       else if(we == 4'b1111)   din <= din_reverse;
       else if(we == 4'b0001)   din <= din_byte;
       else if(we == 4'b0010)   din <= din_byte;
       else if(we == 4'b0100)   din <= din_byte;
       else if(we == 4'b1000)   din <= din_byte;
       else if(we == 4'b0011)   din <= din_2byte;
       else if(we == 4'b1100)   din <= din_2byte;
       else     din <= `ZERO_WORD ;
   end
    // IO �źŵ����
   wire [31:0] IOdin;
   assign IOdin = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : 
                   (inst_sb) ? din_byte : 
                   (inst_sh) ? din_2byte :
                   (inst_sw) ? din_reverse : `ZERO_WORD ;
   always @(*) begin
       if (cpu_rst_n == `RST_ENABLE) begin
            IOwdata <= `ZERO_WORD;
            IOwaddr <= `ZERO_WORD;
       end 
       else if(IOWrite)   begin
            IOwdata <= IOdin;
            IOwaddr <= mem_wd_i;
       end
       else if(IORead)  begin
            IOwdata <= `ZERO_WORD ;
            IOwaddr <= mem_wd_i ;
       end
       else begin
            IOwdata <= `ZERO_WORD ;
            IOwaddr <= `ZERO_WORD ;            
       end
   end
   
   assign mem_wrAddr = (cpu_rst_n == `RST_ENABLE) ? 5'b0 : mem_wa_i;
   assign mem_wrData =  (cpu_rst_n   == `RST_ENABLE ) ? `ZERO_WORD : mem_wd_i;
   assign mem_wrn = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : mem_wreg_i;  
   assign mem2reg = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : mem_mreg_i;      

endmodule

module IORW (
    input wire cpu_clk,
    input wire ioread,
    input wire iowrite,
    input wire IO2reg,
    input wire [31:0] memread_data,
    input wire [31:0] mem_src,
    input wire [15:0] ioread_data,
    input wire [15:0] IOwdata,
    input wire [31:0] caddress,
    output wire [15:0] IOwdata_o,
    output wire [31:0] rdata,
    output wire [31:0] mem_din_src,
    output wire LEDCtrl,
    output wire [1:0] LEDCtrl_s,
    output wire SwitchCtrl,
    output wire [1:0] SwitchCtrl_s,
    output wire KeyboardCtrl,
    output wire setlow,
    output wire sethigh,
    output wire setdis,
    output wire PwmCtrl,
    output wire WatchdogCtrl,
    output wire Counter16Ctrl
    );
    assign IOwdata_o = IOwdata;
    assign rdata = (IO2reg == 1'b1) ? {16'h0000,ioread_data} : memread_data;
    
    assign mem_din_src = (ioread == 1'b1) ? {16'h0000,ioread_data} : mem_src;
    wire iorw = (iowrite||ioread);
    assign LEDCtrl = ((iorw==1)&&(caddress == 32'hFFFFFC60||caddress == 32'hFFFFFC62)) ? 1'b1 : 1'b0;
    assign LEDCtrl_s = caddress[1:0];
    assign SwitchCtrl = ((iorw==1)&&(caddress == 32'hFFFFFC70||caddress == 32'hFFFFFC72)) ? 1'b1 : 1'b0;
    assign SwitchCtrl_s = caddress[1:0];
    assign KeyboardCtrl = ((iorw==1)&&(caddress == 32'hFFFFFC10)) ? 1'b1 : 1'b0;
    assign setlow = ((iorw==1)&&(caddress == 32'hFFFFFC00)) ? 1'b1 : 1'b0;
    assign sethigh = ((iorw==1)&&(caddress == 32'hFFFFFC02)) ? 1'b1 : 1'b0;
    assign setdis = ((iorw==1)&&(caddress == 32'hFFFFFC04)) ? 1'b1 : 1'b0;
    assign PwmCtrl = ((iorw==1)&&(caddress == 32'hFFFFFC30)) ? 1'b1 : 1'b0;
    assign WatchdogCtrl = ((iorw==1)&&(caddress == 32'hFFFFFC50)) ? 1'b1 : 1'b0;
    assign Counter16Ctrl = ((iorw==1)&&(caddress == 32'hFFFFFC20)) ? 1'b1 : 1'b0;
endmodule

module memwb_reg (
    input  wire                     cpu_clk,
	input  wire                     cpu_rst_n,

	// ���Էô�׶ε���Ϣ
	input  wire [`REG_ADDR_BUS  ]   mem_wa,
	input  wire                     mem_wreg,
	input  wire [`REG_BUS       ] 	mem_dreg,

	input  wire                     mem_mreg,
	input  wire [`BSEL_BUS      ]   mem_dre,
	input  wire   mem_sext,
	input  wire                     mem_whilo,
	input  wire [`DOUBLE_REG_BUS]   mem_hilo,

	// ����д�ؽ׶ε���Ϣ 
	output reg  [`REG_ADDR_BUS  ]   wb_wa,
	output reg                      wb_wreg,
	output reg  [`REG_BUS       ]   wb_dreg,
	output reg                      wb_mreg,
	output reg  [`BSEL_BUS      ]   wb_dre,
	output reg          wb_sext,
	output reg                      wb_whilo,
	output reg  [`DOUBLE_REG_BUS] 	wb_hilo
    );
    always @(negedge cpu_clk) begin
		// ��λ��ʱ������д�ؽ׶ε���Ϣ��0
		if (cpu_rst_n == `RST_ENABLE) begin
			wb_wa       <= `REG_NOP;
			wb_wreg     <= `WRITE_DISABLE;
			wb_dreg     <= `ZERO_WORD;
			wb_dre      <= 4'b0;
			wb_sext     <=  1'b0;
			wb_mreg     <= `WRITE_DISABLE;
			wb_whilo    <= `WRITE_DISABLE;
			wb_hilo	    <= `ZERO_DWORD;
		end
		// �����Էô�׶ε���Ϣ�Ĵ沢����д�ؽ׶�
		else begin
			wb_wa 	    <= mem_wa;
			wb_wreg     <= mem_wreg;
			wb_dreg     <= mem_dreg;
			wb_dre      <= mem_dre;
			wb_sext     <= mem_sext;
			wb_mreg     <= mem_mreg;
			wb_whilo    <= mem_whilo;
			wb_hilo     <= mem_hilo;
		end
	end

endmodule


module wb_stage(
    input  wire                     cpu_clk,
    input  wire                   cpu_rst_n,
    // �ӷô�׶λ�õ���Ϣ
    input  wire                   wb_mreg_i,  // �洢�����Ĵ���ʹ���ź�
    input  wire [`BSEL_BUS      ] wb_dre_i,  // ����д�ؽ׶εĴ洢����ʹ���ź�
    input wire  wb_sext_i,
	input  wire [`REG_ADDR_BUS  ] wb_wa_i,
	input  wire                   wb_wreg_i,
	input  wire [`REG_BUS       ] wb_dreg_i,  //����д�ؽ׶�Ҫд��Ŀ�ļĴ���������
    input  wire                   wb_whilo_i,
	input  wire [`DOUBLE_REG_BUS] wb_hilo_i,
    // �����ݴ洢������������
    input  wire [`WORD_BUS      ] ReadData,
    // д��Ŀ�ļĴ���������
    output wire [`REG_ADDR_BUS  ] wb_wa_o,
	output wire                   wb_wreg_o,
    output wire [`WORD_BUS      ] wb_wd_o,
    output wire                   wb_whilo_o,
	output wire [`DOUBLE_REG_BUS] wb_hilo_o
    );
    reg [`WORD_BUS] dm;
    always @(*) begin
         dm <= ReadData;
    end
    assign wb_wa_o      = (cpu_rst_n == `RST_ENABLE) ? 5'b0 : wb_wa_i;  //д�ؼĴ����ĵ�ַ
    assign wb_wreg_o    = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : wb_wreg_i;  //д�ؼĴ�����ʹ���ź�
    assign wb_whilo_o   = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : wb_whilo_i;
    assign wb_hilo_o    = (cpu_rst_n == `RST_ENABLE) ? 64'b0 : wb_hilo_i;

    wire [`WORD_BUS] data  = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : 
                             (wb_dre_i == 4'b1111     ) ? dm :
                             (wb_dre_i == 4'b0001 & wb_sext_i == 1'b1    ) ? {{24{dm[7]}}, dm[7:0]} :
                             (wb_dre_i == 4'b0010 & wb_sext_i == 1'b1   ) ? {{24{dm[15]}}, dm[15:8]} :
                             (wb_dre_i == 4'b0100 &  wb_sext_i == 1'b1  ) ? {{24{dm[23]}}, dm[23:16 ]} :
                             (wb_dre_i == 4'b1000 &  wb_sext_i == 1'b1   ) ? {{24{dm[31 ]}}, dm[31 :24 ]} :
                             (wb_dre_i == 4'b0001 & wb_sext_i == 1'b0    ) ? {{24{1'b0}}, dm[7:0]} :
                             (wb_dre_i == 4'b0010 & wb_sext_i == 1'b0   ) ? {{24{1'b0}}, dm[15:8]} :
                             (wb_dre_i == 4'b0100 &  wb_sext_i == 1'b0  ) ? {{24{1'b0}}, dm[23:16 ]} :
                             (wb_dre_i == 4'b1000 &  wb_sext_i == 1'b0   ) ? {{24{1'b0}}, dm[31 :24 ]} : 
                             (wb_dre_i == 4'b0011 & wb_sext_i == 1'b0    ) ? {{16{1'b0}}, dm[15:0]} :
                             (wb_dre_i == 4'b0011 & wb_sext_i == 1'b1   ) ? {{16{dm[15]}}, dm[15:0]} :
                             (wb_dre_i == 4'b1100 & wb_sext_i == 1'b0    ) ? {{16{1'b0}}, dm[31:16]} :
                             (wb_dre_i == 4'b1100 & wb_sext_i == 1'b1   ) ? {{16{dm[31]}}, dm[31:16]} :`ZERO_WORD;

    assign wb_wd_o = (cpu_rst_n == `RST_ENABLE ) ? `ZERO_WORD : 
                     (wb_mreg_i == `MREG_ENABLE) ? data : wb_dreg_i;
endmodule


// ��ˮ��ͣ��ղ���

module ctrl(
	input wire rst,
	input wire id_stall,
    input wire enbranch,
    
	output reg[5:0] stall,
	output reg clean

);

	always@(*)
	begin
		if(rst == `ENABLE)
		begin
			stall <= 6'b000000;
		end
		else
		begin
			if(id_stall == `ENABLE)
			begin
				stall <= 6'b000111;
			end
			else
			begin
				stall <= 6'b000000;
			end
		end
	end
	
    always@(*)
    begin
        if(rst == `ENABLE)
        begin
            clean <= 1'b0;
        end
        else
        begin
            if(enbranch == `ENABLE)
            begin
                clean <= 1'b1;
            end
            else
            begin
                clean <= 1'b0;
            end
        end
    end

endmodule


// �쳣��ش���

module Cause(D,Clk,En,Q);
	input[31:0]D;
	input Clk,En;
//	output [31:0]Q,Qn;
	output reg [31:0]Q;
	initial
	begin
	Q=0;
	end

	always@(posedge Clk)
	begin
	if(En==1)Q=D;
	end
	/*
	D_FFEC d0(D[0],Clk,En,Clrn,Q[0],Qn[0]);
	D_FFEC d1(D[1],Clk,En,Clrn,Q[1],Qn[1]);
	D_FFEC d2(D[2],Clk,En,Clrn,Q[2],Qn[2]);
	D_FFEC d3(D[3],Clk,En,Clrn,Q[3],Qn[3]);
	D_FFEC d4(D[4],Clk,En,Clrn,Q[4],Qn[4]);
	D_FFEC d5(D[5],Clk,En,Clrn,Q[5],Qn[5]);
	D_FFEC d6(D[6],Clk,En,Clrn,Q[6],Qn[6]);
	D_FFEC d7(D[7],Clk,En,Clrn,Q[7],Qn[7]);
	D_FFEC d8(D[8],Clk,En,Clrn,Q[8],Qn[8]);
	D_FFEC d9(D[9],Clk,En,Clrn,Q[9],Qn[9]);
	D_FFEC d10(D[10],Clk,En,Clrn,Q[10],Qn[10]);
	D_FFEC d11(D[11],Clk,En,Clrn,Q[11],Qn[11]);
	D_FFEC d12(D[12],Clk,En,Clrn,Q[12],Qn[12]);
	D_FFEC d13(D[13],Clk,En,Clrn,Q[13],Qn[13]);
	D_FFEC d14(D[14],Clk,En,Clrn,Q[14],Qn[14]);
	D_FFEC d15(D[15],Clk,En,Clrn,Q[15],Qn[15]);
	D_FFEC d16(D[16],Clk,En,Clrn,Q[16],Qn[16]);
	D_FFEC d17(D[17],Clk,En,Clrn,Q[17],Qn[17]);
	D_FFEC d18(D[18],Clk,En,Clrn,Q[18],Qn[18]);
	D_FFEC d19(D[19],Clk,En,Clrn,Q[19],Qn[19]);
	D_FFEC d20(D[20],Clk,En,Clrn,Q[20],Qn[20]);
	D_FFEC d21(D[21],Clk,En,Clrn,Q[21],Qn[21]);
	D_FFEC d22(D[22],Clk,En,Clrn,Q[22],Qn[22]);
	D_FFEC d23(D[23],Clk,En,Clrn,Q[23],Qn[23]);
	D_FFEC d24(D[24],Clk,En,Clrn,Q[24],Qn[24]);
	D_FFEC d25(D[25],Clk,En,Clrn,Q[25],Qn[25]);
	D_FFEC d26(D[26],Clk,En,Clrn,Q[26],Qn[26]);
	D_FFEC d27(D[27],Clk,En,Clrn,Q[27],Qn[27]);
	D_FFEC d28(D[28],Clk,En,Clrn,Q[28],Qn[28]);
	D_FFEC d29(D[29],Clk,En,Clrn,Q[29],Qn[29]);
	D_FFEC d30(D[30],Clk,En,Clrn,Q[30],Qn[30]);
	D_FFEC d31(D[31],Clk,En,Clrn,Q[31],Qn[31]);*/
endmodule

module STATUS(D,Clk,En,Q);
	input[31:0]D;
	input Clk,En;
//	output [31:0]Q,Qn;
	output reg [31:0]Q;
	initial
	begin
	Q[8:0]=0;
    Q[10:9]=2'b11;
    Q[31:11]=0;
	end

	always@(posedge Clk)
	begin
	if(En==1)Q=D;
	end
	/*
	D_FFEC d0(D[0],Clk,En,Clrn,Q[0],Qn[0]);
	D_FFEC d1(D[1],Clk,En,Clrn,Q[1],Qn[1]);
	D_FFEC d2(D[2],Clk,En,Clrn,Q[2],Qn[2]);
	D_FFEC d3(D[3],Clk,En,Clrn,Q[3],Qn[3]);
	D_FFEC d4(D[4],Clk,En,Clrn,Q[4],Qn[4]);
	D_FFEC d5(D[5],Clk,En,Clrn,Q[5],Qn[5]);
	D_FFEC d6(D[6],Clk,En,Clrn,Q[6],Qn[6]);
	D_FFEC d7(D[7],Clk,En,Clrn,Q[7],Qn[7]);
	D_FFEC d8(D[8],Clk,En,Clrn,Q[8],Qn[8]);
	D_FFEC d9(D[9],Clk,En,Clrn,Q[9],Qn[9]);
	D_FFEC d10(D[10],Clk,En,Clrn,Q[10],Qn[10]);
	D_FFEC d11(D[11],Clk,En,Clrn,Q[11],Qn[11]);
	D_FFEC d12(D[12],Clk,En,Clrn,Q[12],Qn[12]);
	D_FFEC d13(D[13],Clk,En,Clrn,Q[13],Qn[13]);
	D_FFEC d14(D[14],Clk,En,Clrn,Q[14],Qn[14]);
	D_FFEC d15(D[15],Clk,En,Clrn,Q[15],Qn[15]);
	D_FFEC d16(D[16],Clk,En,Clrn,Q[16],Qn[16]);
	D_FFEC d17(D[17],Clk,En,Clrn,Q[17],Qn[17]);
	D_FFEC d18(D[18],Clk,En,Clrn,Q[18],Qn[18]);
	D_FFEC d19(D[19],Clk,En,Clrn,Q[19],Qn[19]);
	D_FFEC d20(D[20],Clk,En,Clrn,Q[20],Qn[20]);
	D_FFEC d21(D[21],Clk,En,Clrn,Q[21],Qn[21]);
	D_FFEC d22(D[22],Clk,En,Clrn,Q[22],Qn[22]);
	D_FFEC d23(D[23],Clk,En,Clrn,Q[23],Qn[23]);
	D_FFEC d24(D[24],Clk,En,Clrn,Q[24],Qn[24]);
	D_FFEC d25(D[25],Clk,En,Clrn,Q[25],Qn[25]);
	D_FFEC d26(D[26],Clk,En,Clrn,Q[26],Qn[26]);
	D_FFEC d27(D[27],Clk,En,Clrn,Q[27],Qn[27]);
	D_FFEC d28(D[28],Clk,En,Clrn,Q[28],Qn[28]);
	D_FFEC d29(D[29],Clk,En,Clrn,Q[29],Qn[29]);
	D_FFEC d30(D[30],Clk,En,Clrn,Q[30],Qn[30]);
	D_FFEC d31(D[31],Clk,En,Clrn,Q[31],Qn[31]);*/
endmodule








