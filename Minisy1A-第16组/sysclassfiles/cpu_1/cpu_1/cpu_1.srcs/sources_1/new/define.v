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

/*------------------- 全局参数 -------------------*/
`define ENABLE      1'b1                    // 有效信号，流水不停止
`define DISABLE     1'b0                    // 信号无效，流水停止信号
`define RST_ENABLE      1'b1                // 复位信号有效  RST_ENABLE
`define RST_DISABLE     1'b0                // 复位信号无效
`define ZERO_WORD       32'h00000000        // 32位的数值0，用于0数值的置数
`define ZERO_DWORD      64'b0               // 64位的数值0
`define WRITE_ENABLE    1'b1                // 使能写
`define WRITE_DISABLE   1'b0                // 禁止写
`define READ_ENABLE     1'b1                // 使能读
`define READ_DISABLE    1'b0                // 禁止读
`define ALUOP_BUS       7 : 0               // 译码阶段的输出aluop_o的宽度
`define SHIFT_ENABLE    1'b1                // 移位指令使能 
`define ALUTYPE_BUS     2 : 0               // 译码阶段的输出alutype_o的宽度  
`define TRUE_V          1'b1                // 逻辑"真"  
`define FALSE_V         1'b0                // 逻辑"假"  
`define CHIP_ENABLE     1'b1                // 芯片使能  
`define CHIP_DISABLE    1'b0                // 芯片禁止  
`define WORD_BUS        31: 0               // 32位宽
`define DOUBLE_REG_BUS  63: 0               // 两倍的通用寄存器的数据线宽度
`define RT_ENABLE       1'b1                // rt选择使能
`define SIGNED_EXT      1'b1                // 符号扩展使能
`define IMM_ENABLE      1'b1                // 立即数选择使能
`define UPPER_ENABLE    1'b1                // 立即数移位使能
`define MREG_ENABLE     1'b1                // 写回阶段存储器结果选择信号
`define BSEL_BUS        3 : 0               // 数据存储器字节选择信号宽度
`define PC_INIT         32'h00000000        // PC初始值

/*------------------- 指令字参数 -------------------*/
`define INST_ADDR_BUS   31: 0               // 指令的地址宽度
`define INST_BUS        31: 0               // 指令的数据宽度

// 操作类型alutype
`define NOP             3'b000
`define ARITH           3'b001  //算术运算
`define LOGIC           3'b010  //逻辑运算
`define MOVE            3'b011 //进行数据移动，将数据由寄存器进行移动
`define SHIFT           3'b100 //移位运算
`define JUMP            3'b101 // 地址跳转运算

// 内部操作码aluop
`define MIPS32_ADD             8'h01
`define MIPS32_ADDU            8'h02
`define MIPS32_SUB            8'h03
`define MIPS32_SUBU             8'h04
`define MIPS32_AND            8'h05

// 乘除法
`define MIPS32_MULT             8'h06
`define MIPS32_MULTU           8'h07
`define MIPS32_DIV            8'h08
`define MIPS32_DIVU             8'h09
// HI\LO寄存器相关
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
// 移位相关指令
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

// I类指令 
`define MIPS32_ADDI              8'h20
`define MIPS32_ADDIU              8'h21
`define MIPS32_ANDI              8'h22
`define MIPS32_ORI              8'h23
`define MIPS32_XORI              8'h24
`define MIPS32_LUI             8'h25
// 数据存储器相关指令
`define MIPS32_LB              8'h26
`define MIPS32_LBU              8'h27
`define MIPS32_LH              8'h28
`define MIPS32_LHU              8'h29
`define MIPS32_SB              8'h2A
`define MIPS32_SH              8'h2B
`define MIPS32_LW              8'h2C
`define MIPS32_SW              8'h2D
// 比较相关指令
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

// J类型指令
`define MIPS32_J             8'h38
`define MIPS32_JAL              8'h39

// 跳转相关指令
/*------------------- 通用寄存器堆参数 -------------------*/
`define REG_BUS         31: 0               // 寄存器数据宽度
`define REG_ADDR_BUS    4 : 0               // 寄存器的地址宽度，共有32个通用寄存器，其中一个0寄存器
`define REG_NUM         32                  // 寄存器数量32个
`define REG_NOP         5'b00000            // 零号寄存器
