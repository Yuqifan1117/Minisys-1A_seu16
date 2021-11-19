`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/09 17:14:42
// Design Name: 
// Module Name: define
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

/*------------------- ȫ�ֲ��� -------------------*/
`define ENABLE      1'b1                    // ��Ч�źţ���ˮ��ֹͣ
`define DISABLE     1'b0                    // �ź���Ч����ˮֹͣ�ź�
`define RST_ENABLE      1'b1                // ��λ�ź���Ч  RST_ENABLE
`define RST_DISABLE     1'b0                // ��λ�ź���Ч
`define ZERO_WORD       32'h00000000        // 32λ����ֵ0������0��ֵ������
`define ZERO_DWORD      64'b0               // 64λ����ֵ0
`define WRITE_ENABLE    1'b1                // ʹ��д
`define WRITE_DISABLE   1'b0                // ��ֹд
`define READ_ENABLE     1'b1                // ʹ�ܶ�
`define READ_DISABLE    1'b0                // ��ֹ��
`define ALUOP_BUS       7 : 0               // ����׶ε����aluop_o�Ŀ��
`define SHIFT_ENABLE    1'b1                // ��λָ��ʹ�� 
`define ALUTYPE_BUS     2 : 0               // ����׶ε����alutype_o�Ŀ��  
`define TRUE_V          1'b1                // �߼�"��"  
`define FALSE_V         1'b0                // �߼�"��"  
`define CHIP_ENABLE     1'b1                // оƬʹ��  
`define CHIP_DISABLE    1'b0                // оƬ��ֹ  
`define WORD_BUS        31: 0               // 32λ��
`define DOUBLE_REG_BUS  63: 0               // ������ͨ�üĴ����������߿��
`define RT_ENABLE       1'b1                // rtѡ��ʹ��
`define SIGNED_EXT      1'b1                // ������չʹ��
`define IMM_ENABLE      1'b1                // ������ѡ��ʹ��
`define UPPER_ENABLE    1'b1                // ��������λʹ��
`define MREG_ENABLE     1'b1                // д�ؽ׶δ洢�����ѡ���ź�
`define BSEL_BUS        3 : 0               // ���ݴ洢���ֽ�ѡ���źſ��
`define PC_INIT         32'h00000000        // PC��ʼֵ

/*------------------- ָ���ֲ��� -------------------*/
`define INST_ADDR_BUS   31: 0               // ָ��ĵ�ַ���
`define INST_BUS        31: 0               // ָ������ݿ��

// ��������alutype
`define NOP             3'b000
`define ARITH           3'b001  //��������
`define LOGIC           3'b010  //�߼�����
`define MOVE            3'b011 //���������ƶ����������ɼĴ��������ƶ�
`define SHIFT           3'b100 //��λ����
`define JUMP            3'b101 // ��ַ��ת����

// �ڲ�������aluop
`define MIPS32_ADD             8'h01
`define MIPS32_ADDU            8'h02
`define MIPS32_SUB            8'h03
`define MIPS32_SUBU             8'h04
`define MIPS32_AND            8'h05

// �˳���
`define MIPS32_MULT             8'h06
`define MIPS32_MULTU           8'h07
`define MIPS32_DIV            8'h08
`define MIPS32_DIVU             8'h09
// HI\LO�Ĵ������
`define MIPS32_MFHI             8'h0A
`define MIPS32_MFLO             8'h0B
`define MIPS32_MTHI           8'h0C
`define MIPS32_MTLO              8'h0D
`define MIPS32_MFC0              8'h0E
`define MIPS32_MTC0              8'h0F
`define MIPS32_OR              8'h10
`define MIPS32_XOR              8'h11
`define MIPS32_NOR              8'h12
`define MIPS32_SLT              8'h13
`define MIPS32_SLTU              8'h14
// ��λ���ָ��
`define MIPS32_SLL              8'h15
`define MIPS32_SRL              8'h16
`define MIPS32_SRA              8'h17
`define MIPS32_SLLV              8'h18
`define MIPS32_SRLV             8'h19
`define MIPS32_SRAV              8'h1A
`define MIPS32_JR              8'h1B
`define MIPS32_JALR              8'h1C
`define MIPS32_BREAK              8'h1D
`define MIPS32_SYSCALL             8'h1E
`define MIPS32_ERET              8'h1F

// I��ָ�� 
`define MIPS32_ADDI              8'h20
`define MIPS32_ADDIU              8'h21
`define MIPS32_ANDI              8'h22
`define MIPS32_ORI              8'h23
`define MIPS32_XORI              8'h24
`define MIPS32_LUI             8'h25
// ���ݴ洢�����ָ��
`define MIPS32_LB              8'h26
`define MIPS32_LBU              8'h27
`define MIPS32_LH              8'h28
`define MIPS32_LHU              8'h29
`define MIPS32_SB              8'h2A
`define MIPS32_SH              8'h2B
`define MIPS32_LW              8'h2C
`define MIPS32_SW              8'h2D
// �Ƚ����ָ��
`define MIPS32_BEQ              8'h2E
`define MIPS32_BNE              8'h2F
`define MIPS32_BGEZ              8'h30
`define MIPS32_BGTZ              8'h31
`define MIPS32_BLEZ              8'h32
`define MIPS32_BLTZ              8'h33
`define MIPS32_BGEZAL              8'h34
`define MIPS32_BLTZAL              8'h35

`define MIPS32_SLTI              8'h36
`define MIPS32_SLTIU              8'h37

// J����ָ��
`define MIPS32_J             8'h38
`define MIPS32_JAL              8'h39

// ��ת���ָ��
/*------------------- ͨ�üĴ����Ѳ��� -------------------*/
`define REG_BUS         31: 0               // �Ĵ������ݿ��
`define REG_ADDR_BUS    4 : 0               // �Ĵ����ĵ�ַ��ȣ�����32��ͨ�üĴ���������һ��0�Ĵ���
`define REG_NUM         32                  // �Ĵ�������32��
`define REG_NOP         5'b00000            // ��żĴ���
