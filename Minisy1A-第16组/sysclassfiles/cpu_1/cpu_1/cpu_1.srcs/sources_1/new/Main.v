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
      // 定义输入输出参数引脚
      input CLK, RST;
      input [31:0] outside_pc;
      output [31:0] instruction, now_pc;
      parameter endReg = 5'b11111; // 31号寄存器
      
      // 数据通路
      wire [31:0] pc, pc0, pc4, i_IR, instruction, pcChoose3, pcChoose1, extendData, ALUResult, WriteData, ReadData1, ReadData2, DataOut;
      wire [31:0] o_ADR, o_BDR, o_ALUout, i_ALUM2DR,i_ALUData1,i_ALUData2;
      wire zero;
      // 控制信号
      wire [2:0] ALUOp;
      wire [1:0] ExtSel, RegDst, PCSrc;
      wire PCWre, IRWre, InsMemRW, WrRegData, RegWre, ALUSrcB, DataMemRW, DBDataSrc;
endmodule

// 取值模块

// IF模块的实现,只将输入的pc信号修改pc寄存器的值，并输出给iaddr指令地址
// 下条指令地址的计算用选择器实现
// @param cpu_clk 时钟信号
// @param ice 指令存储器使能信号
// @param cpu_rst_n信号 重置信号
// @param i_pc 输入的pc值
// @param iaddr 输出的指令地址值
// @param outside_pc ??? 预设一个pc值，当RST为1时进行置位为init_pc
// RST为0时判断pcWre信号，pcWre信号为1时将i_pc作为pc输出
module IF(
    input 	wire 		cpu_clk,
    input 	wire 		cpu_rst_n,
    input   wire [`INST_ADDR_BUS] i_pc,
    input wire [5:0] stall,   //  暂停流水线控制信号，6位的输入端口，用于加气泡
    input wire exe_branch,   // 来自执行单元的条件跳转信号
    input wire [`INST_ADDR_BUS] branchAddr,
    input wire [`INST_ADDR_BUS] exc_addr,
    //output  reg          ice,
    output 	reg  [`INST_ADDR_BUS] 	pc,
    output 	reg [`INST_ADDR_BUS]	iaddr
    );
    
    /*always @(posedge cpu_clk) begin
		if (cpu_rst_n == `RST_ENABLE) begin
			ice <= `CHIP_DISABLE;		      // 复位的时候指令存储器禁用  
		end else begin
			ice <= `CHIP_ENABLE; 		      // 复位结束后，指令存储器使能
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
                    iaddr <= i_pc;                 // 获得访问指令存储器的地址       
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
    //assign pc = (cpu_rst_n == `RST_ENABLE) ? `PC_INIT : i_pc; // 指令存储器禁用的时候，PC保持初始值（MIPS32中设置为0x00000000）
    
endmodule

// pc跳转调用子程序，用于获取子程序的地址
// @param pc 执行该指令时pc的值
// @param i_addr 输入的地址
// @param o_addr 输出的地址

module PCJump(pc, i_addr, o_addr);
  input [31:0] pc;
  input [25:0] i_addr;
  output reg[31:0] o_addr;
  reg [27:0] mid; // 用于存放中间值
  // 输出地址的前四位来自pc[31:28]，中间26位来自i_addr[27:2], 后两位是0
  always @(i_addr) begin
     mid = i_addr << 2;
     o_addr <= {pc[31:28], mid[27:0]};
  end
endmodule

// PC 加立即数 ,用于获取跳转后的地址，通过控制信号控制是否跳转
// @param now_pc  当前pc值
// @param o_pc 输出pc值
// @param branch 控制为1进行跳转
// @param imm 立即数
module PCAddImm(now_pc, imm,branch, o_pc);
  input wire [31:0] now_pc, imm;
  input wire branch;
  output wire [31:0] o_pc;
  // 内存单元是以字节为单位的，32位地址大小为4个字节，所以pc=pc+imm*4
  assign o_pc = (branch == 1'b1) ? now_pc + (imm << 2) : now_pc;
endmodule

// 实现PC递增，正常指令占4字节
// @param i_pc 输入的pc值
// @param o_pc 输出的pc值
module PCAddFour(i_pc, o_pc);
  input wire [31:0] i_pc;
  output wire [31:0] o_pc;
  assign o_pc[31:0] = i_pc[31:0] + 4;
endmodule

// 符号扩展单元的实现
// @param i_num 输入的数据
// @param ExtSel 控制符号扩展单元的信号
// @param o_num 输出的数据
module SignExtend(i_num, ExtSel, o_num);
  input wire [15:0] i_num;
  input wire ExtSel;
  output reg [31:0] o_num;
  initial begin
    o_num = 0;
  end
  always @(i_num or ExtSel) begin
     case(ExtSel)
        // ExtSel 为0时，无符号立即数扩展
        1'b0: o_num <= {{16{1'b0}}, i_num[15:0]};
        // ExtSel 为1时，有符号立即数扩展
        1'b1: o_num <= {{16{i_num[15]}}, i_num[15:0]};
        // 其它情况默认有符号立即数扩展
        default: o_num <= {{16{i_num[15]}}, i_num[15:0]}; // 默认符号扩展
    endcase
  end
endmodule

// 指令译码寄存器模块

module IR_reg (
	input  wire  cpu_clk,  //cpu时钟信号
	input  wire  cpu_rst_n,  //cpu复位信号

	// 来自取指阶段的信息  
	input  wire [`INST_ADDR_BUS]       if_pc,
	input  wire [`INST_BUS     ]       rom_inst,
	input wire[5:0] stall,  // 停止流水线信号
	input wire clean,    // IF/ID寄存器清空信号
	// 送至译码阶段的信息  
	output reg [`INST_BUS     ]       id_inst_i,
	output reg  [`INST_ADDR_BUS]       id_pc
	);
    // 采用上升沿沿触发
	always @(negedge cpu_clk) begin
	    // 复位的时候将送至译码阶段的信息清0
		if (cpu_rst_n == `RST_ENABLE) begin
			id_pc 	<= `PC_INIT;
			id_inst_i    <= `ZERO_WORD;
		end
		// 将来自取指阶段的信息寄存并送至译码阶段
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

// 译码模块实现

module ID(
    input  wire   cpu_clk,       // 时钟信号
    input  wire   cpu_rst_n,   // cpu复位信号
    // 从取指阶段获得的PC值，指令地址32位
    input  wire [`INST_ADDR_BUS]    id_pc_i,
    // 从指令存储器读出的指令字，指令32位
    input  wire [`INST_BUS     ]    id_inst_i,
    // 从通用寄存器堆读出的数据 
    input  wire [`REG_BUS      ]    rd1,
    input  wire [`REG_BUS      ]    rd2,
    // 判断发生数据冒险或lw数据冒险处理相关信号线
    input wire [`REG_ADDR_BUS ] ex_wrAddr,   // EX段数据要写回的地址，5位的输入端口
    input wire [`REG_BUS      ] ex_wrData,    // EX段要写回的数据，32位的输入端口
    input wire [`REG_ADDR_BUS ] mem_wrAddr,  //MEM段数据要写回的地址，5位的输入端口
    input wire [`REG_BUS      ] mem_wrData,  //MEM段要写回的数据，32位的输入端口
    input wire [`REG_ADDR_BUS ] wb_wrAddr,  //WB段数据要写回的地址，5位的输入端口
    input wire [`REG_BUS      ] wb_wrData,  //WB段要写回的数据，32位的输入端口
    input wire ex_wrn,   //EX段数据的写信号，一位的输入端口
    input wire mem_wrn,  //MEM段数据的写信号，一位的输入端口
    input wire mem2reg,   // MEM段从存储器读数据写回的使能信号
    input wire wb_wrn,  //WB段数据的写信号，一位的输入端口
    input wire E_Reg2reg,  //LW信号为0，执行阶段送回判断上一条指令为lw
    // 来自执行单元计算出的是否条件跳转
    input wire en_branch,  

    // 送至执行阶段的译码信息
    output reg [`ALUTYPE_BUS  ]    id_alutype_o,  // 译码阶段指令操作类型
    output reg [`ALUOP_BUS    ]    id_aluop_o,  //指令操作符，8位，全局参数确定
    output reg                     id_whilo_o, //HILO寄存器的写使能信号
    output reg                     id_mreg_o,
    output reg [`REG_ADDR_BUS ]    id_wa_o,  //译码阶段指令待写入的寄存器5位地址
    output reg                     id_wreg_o  , //通用寄存器堆写使能信号
    output reg [`REG_BUS      ]    id_din_o,  // 要写入数据存储器中的数据，从寄存器中读取32位
    //output  wire [`INST_ADDR_BUS]    id_pc_o,  // 传入执行阶段的当前pc，用于计算下一个pc
    // 送至执行阶段的源操作数1、源操作数2
    output reg [`REG_BUS      ]    id_src1_o,
    output reg [`REG_BUS      ]    id_src2_o,
    output reg src1mem_e,
    output reg src2mem_e, 
    // 送至读通用寄存器堆端口的使能和地址
    output wire                     rreg1,
    output wire [`REG_ADDR_BUS ]    ra1,
    output wire                     rreg2,
    output wire [`REG_ADDR_BUS ]    ra2,
    output reg mtc0,
    output reg mfc0,
    // 请求流水线暂停信号，一位的输出端口，控制生成各模块的暂停信号
    output wire stall_rep,
    output reg [`INST_ADDR_BUS] branchAddr,
    output wire Reg2reg,
    // pc来源控制信号
    // 00: pc=pc+4  ; 01:pc=pc+4+(sign-extend)immediate; 10:pc=(rs) ; 11:pc={pc[31:28],addr[27:2],0,0}
    output reg [31:0] reg31data,  // 保存在$31的返回地址
    output reg wreg31,     // 写31号寄存器信号
    output wire [31:0] next_pc,
    output wire syscall,
    output wire break,
    output wire eret,
    output reg [4:0] cp0addr,
    output reg [4:0] mfc0addr,
    output wire ri
    );

    // 根据小端模式组织指令字，取出地址高字节在高位
    wire [`INST_BUS] id_inst = id_inst_i; //{id_inst_i[7:0], id_inst_i[15:8], id_inst_i[23:16], id_inst_i[31:24]};
    wire [31:0]  pc4, extendData,pcChoose1,pcChoose2,pcChoose3;
    wire [1:0] PCSrc;
    // 提取指令字中各个字段的信息，R/I/J指令
    wire [5 :0] op   = id_inst[31:26];
    wire [5 :0] func = id_inst[5 : 0];
    wire [4 :0] rd   = id_inst[15:11];
    wire [4 :0] rs   = id_inst[25:21];
    wire [4 :0] rt   = id_inst[20:16];
    wire [4 :0] sa   = id_inst[10: 6];  //移位数
    wire [15:0] imm  = id_inst[15: 0]; 
    wire [25:0] add  = id_inst[25 :0]; //J指令跳转地址
    /*-------------------- 第一级译码逻辑：确定当前需要译码的指令 --------------------*/
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
    
    // mfc0  ,mtc0 实现
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
    // I类型指令
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
    // J类型指令
    wire inst_j=~op[5]&~op[4]&~op[3]&~op[2]& op[1]&~op[0];
    wire inst_jal=~op[5]&~op[4]&~op[3]&~op[2]& op[1]& op[0];
    /*------------------------------------------------------------------------------*/

    /*-------------------- 第二级译码逻辑：生成具体控制信号 --------------------*/
    // 操作类型alutype
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
    // 内部操作码aluop
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
            // 写通用寄存器使能信号
            id_wreg_o <=  (inst_add |inst_addu|inst_sub| inst_subu |inst_and|inst_mfhi|inst_mflo|inst_mtc0|inst_or|inst_xor|inst_nor| inst_slt |inst_sltu|inst_sll|inst_srl|inst_sra|inst_sllv|inst_srlv|inst_srav|
                              inst_jalr|inst_addi|inst_addiu|inst_andi| inst_ori |inst_xori| inst_lui | inst_lb|inst_lbu|inst_lh|inst_lhu|inst_lw |inst_bgezal|inst_bltzal|inst_slti| inst_sltiu | inst_jal );
            // 写HILO寄存器使能信号
            id_whilo_o <= (inst_mult|inst_multu|inst_div|inst_divu|inst_mthi|inst_mtlo);  
            // 存储器到寄存器使能信号 
            id_mreg_o  <= (inst_lb | inst_lw|inst_lbu|inst_lh|inst_lhu);               
        end
    end 
                                               
    // 立即数移位使能指令，有效时操作数选择为 要移位的立即数(寄存器移位也无效)
    wire shift = inst_sll|inst_srl|inst_sra;
    // 立即数使能信号
    wire immsel = inst_addi|inst_andi|inst_ori |inst_xori| inst_lui |inst_lb|inst_lbu|inst_lh|inst_lhu|inst_sb|inst_sh| inst_addiu | inst_slti|inst_sltiu| inst_lw| inst_sw;
    // 目的寄存器选择信号，目的寄存器为rt(若为0则目的寄存器为rd)
    wire rtsel  = inst_mfc0|inst_addi| inst_addiu |inst_andi|inst_ori |inst_xori| inst_lui | inst_lb| inst_lbu |inst_lh|inst_lhu| inst_lw|inst_slti| inst_sltiu ;
    // 写寄存器地址选择31寄存器，用以保存返回地址
    wire reg31sel = inst_jal|inst_bgezal|inst_bltzal;
    // 符号扩展使能信号，若为0则为零扩展
    wire sext   = inst_addi|inst_addiu | inst_lb | inst_lh | inst_lw | inst_sb |inst_sh | inst_sw|inst_beq|inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal|inst_slti;
    // 加载高半字使能信号
    wire upper  = inst_lui;
    
    
    // 读通用寄存器堆端口1使能信号
    assign rreg1 = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : 
                   (inst_add |inst_addu|inst_sub| inst_subu| inst_and | inst_mult |  inst_multu|inst_div|inst_divu|inst_mthi|inst_mtlo|inst_or|inst_xor|inst_nor|inst_slt|inst_sltu|inst_sllv|
                   inst_srlv|inst_srav|inst_jr|inst_jalr|inst_addi|inst_addiu|inst_andi|inst_ori | inst_xori|inst_lb | inst_lbu|inst_lh|inst_lhu|inst_sb|inst_sh|inst_lw | inst_sw|inst_beq|
                   inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal|inst_slti|inst_sltiu );
    // 读通用寄存器堆读端口2使能信号
    assign rreg2 = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : 
                   (inst_add |inst_addu|inst_sub| inst_subu| inst_and | inst_mult |  inst_multu|inst_div|inst_divu|inst_mtc0|inst_or|inst_xor|inst_nor|inst_slt|inst_sltu|inst_sll|inst_srl|inst_sra|inst_sllv|
                   inst_srlv|inst_srav|inst_sb|inst_sh| inst_sw|inst_beq|inst_bne );    
    // 读通用寄存器堆端口1的地址为rs字段，读端口2的地址为rt字段
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


    
    // 获得指令操作所需的立即数 
    wire [31:0] imm_ext = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD :
                          (upper == `UPPER_ENABLE  ) ? (imm << 16) :
                          (sext  == `SIGNED_EXT    ) ? {{16{imm[15]}}, imm} : {{16{1'b0}}, imm};
                                            
    // 获得待写入目的寄存器的地址（rt或rd）
    always @(posedge cpu_clk) begin
            if (cpu_rst_n == `RST_ENABLE) id_wa_o  <=   `REG_NOP;    
            else if(rtsel == `RT_ENABLE) id_wa_o     <=    rt; 
            else if(reg31sel == 1'b1) id_wa_o     <=   (5'b11111);
            else id_wa_o     <=   rd;
    end
                   
    // 获得访存阶段要存入数据存储器的数据（来自通用寄存器堆读数据端口2）
    always @(posedge cpu_clk) begin
            if (cpu_rst_n == `RST_ENABLE) id_din_o  <=   `ZERO_WORD;
            else id_din_o     <=   (ra2 == ex_wrAddr && ex_wrn == 1'b1 ) ? ex_wrData :((ra2 == mem_wrAddr && mem_wrn == 1'b1) ? mem_wrData : ((ra2 == wb_wrAddr && wb_wrn == 1'b1) ? wb_wrData : rd2));
    end   

    // 获得源操作数1。如果shift信号有效，则源操作数1为移位位数；否则为从读通用寄存器堆端口1获得的数据
    always @(posedge cpu_clk) begin
            if (cpu_rst_n == `RST_ENABLE) id_src1_o  <=   `ZERO_WORD;    
            else if(shift == `SHIFT_ENABLE) id_src1_o     <=    {27'b0, sa}; 
            else if(rreg1 == `READ_ENABLE) id_src1_o     <=   (ra1 == ex_wrAddr && ex_wrn == 1'b1 ) ? ex_wrData :((ra1 == mem_wrAddr && mem_wrn == 1'b1) ? mem_wrData : ((ra1 == wb_wrAddr && wb_wrn == 1'b1) ? wb_wrData : rd1));
            else id_src1_o     <=   `ZERO_WORD;
    end
    // 获得源操作数2。如果immsel信号有效，则源操作数1为立即数；否则为从读通用寄存器堆端口2获得的数据
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
     // 判断有无lw数据冒险，lw数据冒险要设置暂停信号
    assign stall_rep = ((ra1 == ex_wrAddr && rreg1) | (ra2 == ex_wrAddr && rreg2)) & (E_Reg2reg == 0) & (ex_wrAddr != 0) & (ex_wrn == 1);
    assign Reg2reg = ~(inst_lw| inst_lb | inst_lbu | inst_lh | inst_lhu);
    // 控制冒险暂停信号,判断当前指令是条件跳转指令时进行暂停
    //assign stall_beq = inst_beq|inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bgezal|inst_bltzal;
    
 
    // 计算下一条PC的值
    PCAddFour PCAddFour(id_pc_i, pc4);    
    // 写31号寄存器在跳转地址之前
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
    // 控制冒险需要将跳转地址传送到执行单元判断
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) branchAddr <=  `PC_INIT;  
        else branchAddr <= pcChoose2;            
    end    

    // 根据当前的PCSrc信号确定输出的下一个pc值，四选一数据选择器
    DataSelector_4to1 DataSelector_4to1(pc4,pcChoose1,rd1, pcChoose3, PCSrc,cpu_rst_n, next_pc);
    //assign id_pc_o = id_pc_i;
    //assign id_imm = imm;
    //assign id_add = add;
endmodule


//通用寄存器组模块
module regfile(
    input  wire 				 cpu_clk,
	input  wire 				 cpu_rst_n,
	
	// 写端口
	input  wire  [`REG_ADDR_BUS] wa,
	input  wire  [`REG_BUS 	   ] wd,
	input  wire 				 we,
	
	// 读端口1
	input  wire  [`REG_ADDR_BUS] ra1,
	output reg   [`REG_BUS 	   ] rd1,
	input  wire 				 re1,
	
	// 读端口2 
	input  wire  [`REG_ADDR_BUS] ra2,
	output reg   [`REG_BUS 	   ] rd2,
	input  wire 			     re2,
	
	// 写$31的数据和控制信号
	input wire  [`REG_BUS]  reg31data,
	input wire   wreg31,
	// mfc0来自cp0寄存器的值
	input wire [`REG_BUS] cp0rdata,
	input wire [4:0]   mfc0addr,
	input wire mfc0write
    );

    //定义32个32位寄存器
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
	
	
	//读端口1的读操作 
	// ra1是读地址、wa是写地址、we是写使能、wd是要写入的数据 
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
	
	//读端口2的读操作 
	// ra2是读地址、wa是写地址、we是写使能、wd是要写入的数据 
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

//用寄存器保存译码阶段的所有信号，并送给执行阶段，适用于流水线指令
module idexe_reg (
    input  wire 				  cpu_clk,
    input  wire 				  cpu_rst_n,

    // 来自译码阶段的信息
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
    input wire clean,   // ID/EXE寄存器清空信号
    input wire [`INST_ADDR_BUS]  id_branchAddr,
    // lw数据冒险操作数传递选择
    input wire [`REG_BUS]  mem_src,    // MEM段从存储器读出的数据

    //input wire [`INST_ADDR_BUS]    id_pc_o,  // 译码阶段输出的下一个选择的pc给IF
    //input wire [15:0 ] id_imm,
    //input wire [25:0] id_add,
    // 送至执行阶段的信息
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
        // 复位的时候将送至执行阶段的信息清0
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
        // 将来自译码阶段的信息寄存并送至执行阶段
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


// 执行模块，根据aluop确定指令类型从而进行不同操作
module exe_stage (
    input  wire 					cpu_rst_n,
    input  wire                     cpu_clk,
    // 从译码阶段获得的信息
    input  wire [`ALUTYPE_BUS	] 	exe_alutype_i,
    input  wire [`ALUOP_BUS	    ] 	exe_aluop_i,
    input  wire [`REG_BUS 		] 	exe_src1_i, //操作数1 32位
    input  wire [`REG_BUS 		] 	exe_src2_i,  //操作数2 32位
    input  wire [`REG_ADDR_BUS 	] 	exe_wa_i,  //指令目的寄存器地址
    input  wire 					exe_wreg_i, //目的寄存器写使能有效
    input  wire 					exe_mreg_i,  //存储器到寄存器有效
    input  wire [`REG_BUS 		] 	exe_din_i, // 处于执行阶段待写入数据存储器的数据
    input  wire                     exe_whilo_i, //处于执行阶段hilo寄存器写使能
    input wire Reg2reg,
    input wire [`INST_ADDR_BUS]   exe_branchAddr,

    // 从HILO寄存器获得的数据 
    input  wire [`REG_BUS 		] 	hi_i, //来自hi的输入
    input  wire [`REG_BUS 		] 	lo_i, //来自lo的输入
    output wire [`REG_ADDR_BUS ] ex_wrAddr,   // EX段数据要写回的地址，5位的输入端口
    output wire [`REG_BUS      ] ex_wrData,    // EX段要写回的数据，32位的输入端口
    output wire ex_wrn,   //EX段数据的写信号，一位的输入端口
    output wire E_Reg2reg,  //LW信号为0，执行阶段送回判断上一条指令为lw
    output wire OF,  // 送至CP0判断有无发生溢出
    // 送至执行阶段的信息
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
    // 送至译码阶段的信息
    output reg en_branch,
    output reg [`INST_ADDR_BUS] branchAddr
    );
    // 根据是否有load-use冒险选择相应的数据来源
    // 直接传到下一阶段
    
    always @(posedge cpu_clk) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            exe_aluop_o        <= 8'b0;
            exe_mreg_o            <= 1'b0;
            exe_din_o            <= 32'b0;
        end
        else begin
            exe_aluop_o        <= exe_aluop_i;
            exe_mreg_o            <= exe_mreg_i;
            exe_din_o            <= exe_din_i;   //待写入数据存储器的数据        
        end    
    end
    // 直接传给乘除法HILO寄存器写信号
    assign exe_whilo_o  = (cpu_rst_n == `RST_ENABLE) ? 1'b0 :exe_whilo_i;
    wire [`REG_BUS       ]      logicres;       // 保存逻辑运算的结果
    wire [`REG_BUS       ]      shiftres;       // 保存移位运算结果
    wire [`REG_BUS       ]      moveres;        // 保存移动操作的结果
    wire [`REG_BUS       ]      jumpresult;    // 保存条件跳转条件计算结果
    wire [`REG_BUS       ]      judgmentresult;  // 保存比较指令的计算结果

    wire [`REG_BUS       ]      hi_t;           // 保存HI寄存器的最新值
    wire [`REG_BUS       ]      lo_t;           // 保存LO寄存器的最新值
    wire [`REG_BUS       ]      arithres;       // 保存算术操作的结果
    reg [`REG_BUS       ]      mult_src1;
    reg [`REG_BUS       ]      mult_src2;
    wire [`DOUBLE_REG_BUS]      mulres;         // 保存乘法操作的结果
    wire [`DOUBLE_REG_BUS]      mulres_u;       // 无符号乘的结果
    wire [`DOUBLE_REG_BUS]     divures;         // 无符号除的结果
    wire [`DOUBLE_REG_BUS]     divres;          // 有符号除的结果
    wire exe_en_branch;         // 保存根据运算结果得到的是否跳转信号
    wire zero;        //  保存运算结果是否为0，用于判断大小
    wire SF;     //  保存运算结果的符号位
    // 根据内部操作码aluop进行逻辑运算
    assign logicres = (cpu_rst_n == `RST_ENABLE)  ? `ZERO_WORD : 
                      (exe_aluop_i == `MIPS32_AND )  ? (exe_src1_i & exe_src2_i) : 
                      (exe_aluop_i == `MIPS32_ORI )  ? (exe_src1_i | exe_src2_i) : 
                      (exe_aluop_i == `MIPS32_XOR ) ?  (exe_src1_i ^ exe_src2_i):
                      (exe_aluop_i == `MIPS32_NOR ) ?  ~(exe_src1_i | exe_src2_i):
                      (exe_aluop_i == `MIPS32_OR ) ?  (exe_src1_i | exe_src2_i):
                      (exe_aluop_i == `MIPS32_XORI ) ?  (exe_src1_i ^ exe_src2_i):
                      (exe_aluop_i == `MIPS32_LUI )  ? exe_src2_i : `ZERO_WORD;

    // 根据内部操作码aluop进行移位运算
    wire [63:0] sradata;
    assign sradata = { {32{exe_src2_i[31]}} ,exe_src2_i[31:0]} >> exe_src1_i;
    assign shiftres = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : 
                      (exe_aluop_i == `MIPS32_SRL )  ? (exe_src2_i >> exe_src1_i) :
                      (exe_aluop_i == `MIPS32_SRLV )  ? (exe_src2_i >> exe_src1_i) :
                      (exe_aluop_i == `MIPS32_SRA )  ? sradata[31:0] :
                      (exe_aluop_i == `MIPS32_SRAV )  ? sradata[31:0] :
                      (exe_aluop_i == `MIPS32_SLL )  ? (exe_src2_i << exe_src1_i) : `ZERO_WORD;
    
    // 根据内部操作码aluop进行数据移动，得到最新的HI、LO寄存器的值
    assign hi_t     = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : hi_i;
    assign lo_t     = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : lo_i;
    assign moveres  = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : 
                      (exe_aluop_i == `MIPS32_MFHI) ? hi_t :
                      (exe_aluop_i == `MIPS32_MFLO) ? lo_t : `ZERO_WORD;
    
    // 根据内部操作码aluop进行算术运算    
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
    // PC条件跳转相关指令部分
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
            2'b01: exe_pc_next = pcChoose1;   //  条件跳转
            2'b10: exe_pc_next = exe_src1_i;   //  pc= (rs)
            2'b11: exe_pc_next = pcChoose3;   //   jump相关指令
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
    // 根据内部操作码aluop进行乘法运算，并保存送至下一阶段
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
            // 传入存储写回阶段的待写寄存器地址
            exe_wa_o        <= exe_wa_i;
            // 写寄存器使能
            exe_wreg_o      <= exe_wreg_i;         
        end    
    end
    
    
    // 根据操作类型alutype确定执行阶段最终的运算结果（既可能是待写入目的寄存器的数据，也可能是访问数据存储器的地址）
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
    
    // 解决数据冒险向ID传的数据
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

    // 来自执行阶段的信息
// hilo寄存器模块，用于乘除法和MOV  hilo指令

module hilo (
    input  wire            cpu_clk,
    input  wire            cpu_rst_n,

    // 写端口 
    input  wire            we,
    input  wire [`REG_BUS] hi_i,
    input  wire [`REG_BUS] lo_i,
    
    // 读端口 
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
            reghi <= hi_i;            // 将乘法结果mulres的前32位给HI寄存器，
            reglo <= lo_i;            // 后32位给lo寄存器
        end
    end
	// ra1是读地址、wa是写地址、we是写使能、wd是要写入的数据 
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



// 执行/访存寄存器模块，相关信号的传递

module exemem_reg (
    input  wire 				cpu_clk,
    input  wire 				cpu_rst_n,

    // 来自执行阶段的信息
    input  wire [`ALUOP_BUS   ] exe_aluop,
    input  wire [`REG_ADDR_BUS] exe_wa,
    input  wire                 exe_wreg,
    input  wire [`REG_BUS 	  ] exe_wd,
    input  wire                 exe_mreg,
    input  wire [`REG_BUS 	  ] exe_din,
    
    //input  wire                 exe_whilo,
    //input  wire [`DOUBLE_REG_BUS] exe_hilo,
    //input  wire exe_en_branch,   //  执行阶段判断是否跳转信号
    //input  wire E_Reg2reg_i,
    //output reg [`REG_ADDR_BUS ] ex_wrAddr,   // EX段数据要写回的地址，5位的输入端口
    //output reg [`REG_BUS      ] ex_wrData,    // EX段要写回的数据，32位的输入端口
    //output reg ex_wrn,   //EX段数据的写信号，一位的输入端口
    //output reg E_Reg2reg_o,
    // 送到译码阶段的信息
    // output reg en_branch,
    // 送到访存阶段的信息 
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


// 访存模块，对于访存指令要完成加载和存储操作，对于非访存指令，直接把从执行阶段的信息直接传递给下一级就行

module mem_stage (
    input  wire             cpu_rst_n,
    input  wire             cpu_clk,
    // 从执行阶段获得的信息
    input  wire [`ALUOP_BUS     ]       mem_aluop_i,
    input  wire [`REG_ADDR_BUS  ]       mem_wa_i,
    input  wire                         mem_wreg_i,
    input  wire [`REG_BUS       ]       mem_wd_i,

    input  wire                         mem_mreg_i,
    input  wire [`REG_BUS       ]       mem_din_i,
    
    input  wire                         mem_whilo_i,
    input  wire [`DOUBLE_REG_BUS]       mem_hilo_i,

    // 送至写回阶段的信息
    output reg [`REG_ADDR_BUS  ]       mem_wa_o,
    output reg                         mem_wreg_o,
    output reg [`REG_BUS       ]       mem_dreg_o,


    output reg                         mem_mreg_o,
    output reg [`BSEL_BUS      ]       dre,   // 数据存储器的读使能信号，4位控制不同字节
    output reg   sext,  // 控制从存储器中读出的数据进行符号扩展
    output reg                         mem_whilo_o,
    output reg [`DOUBLE_REG_BUS]       mem_hilo_o,

    // 送至数据存储器的信号
    output reg                         dce,   // 数据存储器的使能信号
    output wire [`INST_ADDR_BUS ]       daddr,   // 数据存储器的访存地址
    output reg [`BSEL_BUS      ]       we,    // 数据存储器的写字节使能信号
    output reg [`REG_BUS       ]       din, //  数据存储器要写入的数据
    
    // 送至IO的数据信号
    output reg [`REG_BUS       ]      IOwdata, // IO要写入的数据
    output reg [`INST_ADDR_BUS ]      IOwaddr, // IO要写入的地址
    output wire IORead,
    output wire IOWrite,
    // 送至译码阶段解决数据冒险
    output wire [`REG_ADDR_BUS ] mem_wrAddr,   
    output wire [`REG_BUS      ] mem_wrData,  
    output wire mem_wrn,
    output wire mem2reg,
    output reg IO2reg   
    );

    // 如果当前不是访存指令，则只需要把从执行阶段获得的信息直接输出
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

    // 确定当前的访存指令
    wire inst_lb = (mem_aluop_i == 8'h26);
    wire inst_lbu = (mem_aluop_i == 8'h27);
    wire inst_lh = (mem_aluop_i == 8'h28);
    wire inst_lhu = (mem_aluop_i == 8'h29);
    wire inst_sb = (mem_aluop_i == 8'h2A);
    wire inst_sh = (mem_aluop_i == 8'h2B);
    wire inst_lw = (mem_aluop_i == 8'h2C);
    wire inst_sw = (mem_aluop_i == 8'h2D);

    // 获得数据存储器的访问地址
    assign daddr = (cpu_rst_n == `RST_ENABLE) ? `ZERO_WORD : mem_wd_i;
    // 增加I/O相关控制信号
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
    // 获得数据存储器读字节使能信号
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
    // 送至数据存储的值写入
     always @(*) begin
        if (cpu_rst_n == `RST_ENABLE) begin
            we[0]            <= 1'b0;
            we[1]            <= 1'b0;
            we[2]            <= 1'b0;
            we[3]            <= 1'b0;
            dce              <= 1'b0;
        end
        else begin
            // 获得数据存储器写字节使能信号
            we[0]            <= (((inst_sb & (daddr[1 : 0] == 2'b00)) | inst_sw |inst_sh & (daddr[1 : 0] == 2'b00))) & MemWrite;
            we[1]            <= (((inst_sb & (daddr[1 : 0] == 2'b01)) | inst_sw |inst_sh & (daddr[1 : 0] == 2'b00))) & MemWrite;
            we[2]            <= (((inst_sb & (daddr[1 : 0] == 2'b10)) | inst_sw |inst_sh & (daddr[1 : 0] == 2'b10))) & MemWrite;
            we[3]            <= (((inst_sb & (daddr[1 : 0] == 2'b11)) | inst_sw |inst_sh & (daddr[1 : 0] == 2'b10))) & MemWrite;
            // 获得数据存储器使能信号
            dce              <= (inst_lb |inst_lbu|inst_lh|inst_lhu| inst_lw | inst_sb |inst_sh| inst_sw) & (MemWrite| MemRead);
        end    
    end             

    // 确定待写入数据存储器的数据
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
    // IO 信号的输出
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

	// 来自访存阶段的信息
	input  wire [`REG_ADDR_BUS  ]   mem_wa,
	input  wire                     mem_wreg,
	input  wire [`REG_BUS       ] 	mem_dreg,

	input  wire                     mem_mreg,
	input  wire [`BSEL_BUS      ]   mem_dre,
	input  wire   mem_sext,
	input  wire                     mem_whilo,
	input  wire [`DOUBLE_REG_BUS]   mem_hilo,

	// 送至写回阶段的信息 
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
		// 复位的时候将送至写回阶段的信息清0
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
		// 将来自访存阶段的信息寄存并送至写回阶段
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
    // 从访存阶段获得的信息
    input  wire                   wb_mreg_i,  // 存储器到寄存器使能信号
    input  wire [`BSEL_BUS      ] wb_dre_i,  // 处于写回阶段的存储器读使能信号
    input wire  wb_sext_i,
	input  wire [`REG_ADDR_BUS  ] wb_wa_i,
	input  wire                   wb_wreg_i,
	input  wire [`REG_BUS       ] wb_dreg_i,  //处于写回阶段要写回目的寄存器的数据
    input  wire                   wb_whilo_i,
	input  wire [`DOUBLE_REG_BUS] wb_hilo_i,
    // 从数据存储器读出的数据
    input  wire [`WORD_BUS      ] ReadData,
    // 写回目的寄存器的数据
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
    assign wb_wa_o      = (cpu_rst_n == `RST_ENABLE) ? 5'b0 : wb_wa_i;  //写回寄存器的地址
    assign wb_wreg_o    = (cpu_rst_n == `RST_ENABLE) ? 1'b0 : wb_wreg_i;  //写回寄存器的使能信号
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


// 流水暂停清空部件

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


// 异常相关处理

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








