`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/09 16:26:55
// Design Name: 
// Module Name: MIPS32
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

// 处理器内核的顶层模块
`include "define.v"

module MIPS32(
    input  wire                  cpu_clk_50M,
    input  wire                  cpu_rst_n,
    
    // inst_rom
    output wire [`INST_ADDR_BUS] iaddr,  // 下一条指令地址
    //output wire                  ice,   // 指令存储器的使能信号
    input  wire [`INST_BUS]      rom_inst,  // 从指令存储器中读取出的指令
    input  wire keyboardbreak,
    // data_ram
    output wire                  dce,   //数据存储器的使能信号
    output wire [`INST_ADDR_BUS] daddr,  // 数据存储器的地址
    output wire [`BSEL_BUS     ] we,   //数据存储器的写使能信号
    output wire [`INST_BUS     ] din,  // 要存入数据存储器的数
    input  wire [`INST_BUS     ] dm,   // 从数据存储器中读取出的数据
    input  wire [`INST_BUS]    mem_src,  // load-use冒险由存储器直接传递的数据
    // 给外设的控制信号
    output wire [15:0] IOwdata_o,
    input wire [15:0] ioread_data,
    output wire LEDCtrl,
    output wire [1:0] LEDCtrl_s,
    output wire SwitchCtrl,
    output wire [1:0] SwitchCtrl_s,
    output wire KeyboardCtrl,
    output wire NixieTubeCtrl,
    output wire PwmCtrl,
    output wire WatchdogCtrl,
    output wire Counter16Ctrl,
    output wire setlow,
    output wire sethigh,
    output wire setdis
    );
    
    wire [`WORD_BUS      ] if_pc_i;
    wire [`WORD_BUS      ] pc;
    wire [`INST_BUS      ] inst;
    wire [`INST_BUS      ] exc_addr;
    // 连接IF/ID模块与译码阶段ID模块的变量 
    wire [`WORD_BUS      ] id_pc_i;
    
    // 连接译码阶段ID模块与通用寄存器Regfile模块的变量 
    wire 				   re1;
    wire [`REG_ADDR_BUS  ] ra1;
    wire [`REG_BUS       ] rd1;
    wire 				   re2;
    wire [`REG_ADDR_BUS  ] ra2;
    wire [`REG_BUS       ] rd2;
    wire [`REG_ADDR_BUS ] ex_wrAddr;
    wire [`REG_BUS      ] ex_wrData;
    wire [`REG_ADDR_BUS ] mem_wrAddr;
    wire [`REG_BUS      ] mem_wrData;
    wire                    ex_wrn;
    wire                    mem_wrn;
    wire                    mem2reg;
    wire                    E_Reg2reg;
    wire [`INST_ADDR_BUS] id_branchAddr;
    wire [`INST_ADDR_BUS] exe_branchAddr;
    
    wire eret;
    wire syscall;
    wire break;
    wire mtc0,mfc0;
    wire [4:0] cp0addr;
    wire [4:0] mfc0addr;
    wire [31:0] cp0rdata;
    
    wire [`ALUOP_BUS     ] id_aluop_o;
    wire [`ALUTYPE_BUS   ] id_alutype_o;
    wire [`REG_BUS 	     ] id_src1_o;
    wire [`REG_BUS 	     ] id_src2_o;
    wire 				   id_wreg_o;
    wire [`REG_ADDR_BUS  ] id_wa_o;
    wire                   id_whilo_o;
    wire                   id_mreg_o;
    wire [`WORD_BUS      ] id_pc_o;
    wire [`REG_BUS 	     ] id_din_o;
    wire                   stall_rep;
    wire                    Reg2reg;
    wire src1mem_e,src2mem_e;
    wire [31:0] reg31data;  // 保存在$31的返回地址
    wire wreg31;     // 写31号寄存器信号
    
    wire [`ALUOP_BUS     ] exe_aluop_i;
    wire [`ALUTYPE_BUS   ] exe_alutype_i;
    wire [`REG_BUS 	     ] exe_src1_i;
    wire [`REG_BUS 	     ] exe_src2_i;
    wire 				   exe_wreg_i;
    wire [`REG_ADDR_BUS  ] exe_wa_i;
    wire                   exe_whilo_i;
    wire                   exe_mreg_i;
    wire [`REG_BUS 	     ] exe_din_i;
    wire                   E_Reg2reg_i;
    wire [`REG_BUS 	     ] exe_hi_i;
    wire [`REG_BUS 	     ] exe_lo_i;
    wire OF;
    wire [`ALUOP_BUS     ] exe_aluop_o;
    wire 				   exe_wreg_o;
    wire [`REG_ADDR_BUS  ] exe_wa_o;
    wire [`REG_BUS 	     ] exe_wd_o;
    wire                   exe_mreg_o;
    wire [`REG_BUS 	     ] exe_din_o;
    wire 				   exe_whilo_o;
    wire [`REG_BUS] exe_hi_o;
    wire [`REG_BUS] exe_lo_o;
    wire exe_en_branch;
    wire [`INST_ADDR_BUS] branchAddr;
    wire                E_Reg2reg_o;
    
    wire [`ALUOP_BUS     ] mem_aluop_i;
    wire 				   mem_wreg_i;
    wire [`REG_ADDR_BUS  ] mem_wa_i;
    wire [`REG_BUS 	     ] mem_wd_i;
    wire                   mem_mreg_i;
    wire [`REG_BUS 	     ] mem_din_i;

    wire IO2reg;
    wire [`REG_BUS 	     ] IOwdata;
    wire [`INST_ADDR_BUS ] IOwaddr;
    wire                  ioread;
    wire                  iowrite;
    wire [`REG_BUS 	     ] rdata;
    wire [`REG_BUS 	     ]mem_din_src;
    wire 				   mem_wreg_o;
    wire [`REG_ADDR_BUS  ] mem_wa_o;
    wire [`REG_BUS 	     ] mem_dreg_o;
    wire                   mem_mreg_o;
    wire [`BSEL_BUS      ] mem_dre_o;
    wire                   mem_sext_o;

    wire 				   wb_wreg_i;
    wire [`REG_ADDR_BUS  ] wb_wa_i;
    wire [`REG_BUS       ] wb_dreg_i;
    wire [`BSEL_BUS      ] wb_dre_i;
    wire                   wb_sext_i;
    wire                   wb_mreg_i;
    
    wire 				   wb_wreg_o;
    wire [`REG_ADDR_BUS  ] wb_wa_o;
    wire [`REG_BUS       ] wb_wd_o;

    
    wire [5:0]          stall;
    wire clean;
    
    IF IF0(.cpu_clk(cpu_clk_50M), .cpu_rst_n(cpu_rst_n),.stall(stall),.exc_addr(exc_addr),
        .i_pc(id_pc_o), .exe_branch(exe_en_branch),.branchAddr(branchAddr), .pc(pc),.iaddr(iaddr));
    
    IR_reg IR_reg0(.cpu_clk(cpu_clk_50M), .cpu_rst_n(cpu_rst_n),
        .if_pc(pc), .id_pc(id_pc_i),.rom_inst(rom_inst), .id_inst_i(inst), .stall(stall), .clean(clean)
    );

    ID ID0(.cpu_rst_n(cpu_rst_n),.cpu_clk(cpu_clk_50M), .id_pc_i(id_pc_i), 
        .id_inst_i(inst), .en_branch(exe_en_branch),.reg31data(reg31data),.wreg31(wreg31),
        .rd1(rd1), .rd2(rd2), .syscall(syscall),.break(break),.eret(eret),
        .ex_wrAddr(ex_wrAddr), .ex_wrData(ex_wrData), .ex_wrn(ex_wrn),
        .mem_wrAddr(mem_wrAddr), .mem_wrData(mem_wrData), .mem_wrn(mem_wrn),.mem2reg(mem2reg),	
        .wb_wrAddr(wb_wa_o), .wb_wrData(wb_wd_o), .wb_wrn(wb_wreg_o),	
        .E_Reg2reg(E_Reg2reg),
        .rreg1(re1), .rreg2(re2),  
        .ra1(ra1), .ra2(ra2), .mfc0(mfc0),.mtc0(mtc0),.cp0addr(cp0addr),.mfc0addr(mfc0addr),
        .id_aluop_o(id_aluop_o), .id_alutype_o(id_alutype_o),
        .id_src1_o(id_src1_o), .id_src2_o(id_src2_o),
        .id_wa_o(id_wa_o), .id_wreg_o(id_wreg_o), .stall_rep(stall_rep),
        .id_whilo_o(id_whilo_o),.id_mreg_o(id_mreg_o),
         .id_din_o(id_din_o), .Reg2reg(Reg2reg), .next_pc(id_pc_o), .branchAddr(id_branchAddr),
         .src1mem_e(src1mem_e),.src2mem_e(src2mem_e)
    );
    
    regfile regfile0(.cpu_clk(cpu_clk_50M), .cpu_rst_n(cpu_rst_n),
        .we(wb_wreg_o), .wa(wb_wa_o), .wd(wb_wd_o),.reg31data(reg31data),.wreg31(wreg31),
        .cp0rdata(cp0rdata),.mfc0addr(mfc0addr),.mfc0write(mfc0),
        .re1(re1), .ra1(ra1), .rd1(rd1),
        .re2(re2), .ra2(ra2), .rd2(rd2)
    );
    
    idexe_reg idexe_reg0(.cpu_clk(cpu_clk_50M), .cpu_rst_n(cpu_rst_n), .stall(stall), 
        .id_alutype(id_alutype_o), .id_aluop(id_aluop_o), .clean(clean),
        .id_src1(id_src1_o), .id_src2(id_src2_o), .mem_src(mem_din_src),
        .id_wa(id_wa_o), .id_wreg(id_wreg_o), .id_whilo(id_whilo_o),
        .id_mreg(id_mreg_o), .id_din(id_din_o), 
        .Reg2reg(Reg2reg), .id_branchAddr(id_branchAddr),
        .exe_alutype(exe_alutype_i), .exe_aluop(exe_aluop_i),
        .exe_src1(exe_src1_i), .exe_src2(exe_src2_i), 
        .E_Reg2reg(E_Reg2reg_i),.exe_branchAddr(exe_branchAddr),
        .exe_wa(exe_wa_i), .exe_wreg(exe_wreg_i), .exe_whilo(exe_whilo_i),
        .exe_mreg(exe_mreg_i), .exe_din(exe_din_i),
        .src1mem_e(src1mem_e),.src2mem_e(src2mem_e)
    );
    
     exe_stage exe_stage0(.cpu_clk(cpu_clk_50M), .cpu_rst_n(cpu_rst_n),
           .exe_alutype_i(exe_alutype_i), .exe_aluop_i(exe_aluop_i),
           .exe_src1_i(exe_src1_i), .exe_src2_i(exe_src2_i), .OF(OF),
           .exe_wa_i(exe_wa_i), .exe_wreg_i(exe_wreg_i),
           .exe_mreg_i(exe_mreg_i), .exe_din_i(exe_din_i), .exe_branchAddr(exe_branchAddr),
           .hi_i(exe_hi_i), .lo_i(exe_lo_i), .exe_whilo_i(exe_whilo_i), .Reg2reg(E_Reg2reg_i),
           .exe_aluop_o(exe_aluop_o),.E_Reg2reg(E_Reg2reg),.exe_hi_o(exe_hi_o),.exe_lo_o(exe_lo_o),.exe_whilo_o(exe_whilo_o),
           .ex_wrAddr(ex_wrAddr), .ex_wrData(ex_wrData), .ex_wrn(ex_wrn),
           .exe_wa_o(exe_wa_o), .exe_wreg_o(exe_wreg_o), .exe_wd_o(exe_wd_o),
           .exe_mreg_o(exe_mreg_o), .exe_din_o(exe_din_o), .en_branch(exe_en_branch), .branchAddr(branchAddr)
       );
        
    exemem_reg exemem_reg0(.cpu_clk(cpu_clk_50M), .cpu_rst_n(cpu_rst_n),
        .exe_aluop(exe_aluop_o),.exe_wa(exe_wa_o), .exe_wreg(exe_wreg_o), .exe_wd(exe_wd_o),
        .exe_mreg(exe_mreg_o), .exe_din(exe_din_o),
        .mem_aluop(mem_aluop_i),
        .mem_wa(mem_wa_i), .mem_wreg(mem_wreg_i), .mem_wd(mem_wd_i),
        .mem_mreg(mem_mreg_i), .mem_din(mem_din_i)

    );

    mem_stage mem_stage0(.cpu_clk(cpu_clk_50M), .cpu_rst_n(cpu_rst_n), .mem_aluop_i(mem_aluop_i),
        .mem_wa_i(mem_wa_i), .mem_wreg_i(mem_wreg_i), .mem_wd_i(mem_wd_i),
        .mem_mreg_i(mem_mreg_i), .mem_din_i(mem_din_i),

        .mem_wa_o(mem_wa_o), .mem_wreg_o(mem_wreg_o), .mem_dreg_o(mem_dreg_o),
        .mem_mreg_o(mem_mreg_o), .dre(mem_dre_o), .sext(mem_sext_o),
        .mem_wrAddr(mem_wrAddr), .mem_wrData(mem_wrData), .mem_wrn(mem_wrn),.mem2reg(mem2reg),

        .dce(dce), .daddr(daddr), .we(we), .din(din),.IO2reg(IO2reg),
        .IOwdata(IOwdata), .IOwaddr(IOwaddr),.IORead(ioread),.IOWrite(iowrite)
    );
    IORW IORW0(.cpu_clk(cpu_clk_50M),.ioread(ioread),.iowrite(iowrite),.IO2reg(IO2reg),.memread_data(dm),.ioread_data(ioread_data),.IOwdata(IOwdata[15:0]),.caddress(IOwaddr),.mem_src(mem_src),.mem_din_src(mem_din_src),
        .IOwdata_o(IOwdata_o), .rdata(rdata),.LEDCtrl(LEDCtrl),.LEDCtrl_s(LEDCtrl_s),.SwitchCtrl(SwitchCtrl),.SwitchCtrl_s(SwitchCtrl_s),.KeyboardCtrl(KeyboardCtrl),
        .setlow(setlow),.sethigh(sethigh),.setdis(setdis),
        .PwmCtrl(PwmCtrl),.WatchdogCtrl(WatchdogCtrl),.Counter16Ctrl(Counter16Ctrl)
    );	
    memwb_reg memwb_reg0(.cpu_clk(cpu_clk_50M), .cpu_rst_n(cpu_rst_n),
        .mem_wa(mem_wa_o), .mem_wreg(mem_wreg_o), .mem_dreg(mem_dreg_o),
        .mem_mreg(mem_mreg_o), .mem_dre(mem_dre_o), .mem_sext(mem_sext_o),

        
        .wb_wa(wb_wa_i), .wb_wreg(wb_wreg_i), .wb_dreg(wb_dreg_i),
        .wb_mreg(wb_mreg_i), .wb_dre(wb_dre_i), .wb_sext(wb_sext_i)
    );

    wb_stage wb_stage0(.cpu_rst_n(cpu_rst_n),.cpu_clk(cpu_clk_50M),
        .wb_mreg_i(wb_mreg_i), .wb_dre_i(wb_dre_i), . wb_sext_i(wb_sext_i),
        .wb_wa_i(wb_wa_i), .wb_wreg_i(wb_wreg_i), .wb_dreg_i(wb_dreg_i), 

        .ReadData(rdata),
        .wb_wa_o(wb_wa_o), .wb_wreg_o(wb_wreg_o), .wb_wd_o(wb_wd_o)

    );

    hilo hilo0(.cpu_clk(cpu_clk_50M), .cpu_rst_n(cpu_rst_n), 
        .we(exe_whilo_o),
        .hi_i(exe_hi_o), .lo_i(exe_lo_o),
        .hi_o(exe_hi_i), .lo_o(exe_lo_i)
    );
    
    ctrl ctrl0(.rst(cpu_rst_n), .id_stall(stall_rep), .enbranch(exe_en_branch),
     .stall(stall), .clean(clean)   
    );
    
    CP0 cp0(.clk(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),.pc(id_pc_o),.mtc0(mtc0),.mfc0(mfc0),.addr(cp0addr),.wdata(exe_src2_i),
    .eret(eret),.syscall(syscall),.break(break),.overflow(OF),.keyboardbreak(keyboardbreak),.exc_addr(exc_addr),.rdata(cp0rdata)
    );

endmodule

