`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/09 18:52:02
// Design Name: 
// Module Name: MIPS32_SYS
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

// 加入指令存储器和数据存储器之后整个MIPS32系统

`include "define.v"

module MIPS32_SYS(
       input wire sys_clk_100M,
       input wire sys_rst_n,
       input wire [23:0] switch2N4,
       output wire [23:0] led2N4,
       input wire [3:0] line,
       output wire [3:0] row,
       output wire [6:0] nixieTubeC,
       output wire DP,
       output wire [7:0] nixieTubeA
   );
   wire                 state;
   wire                  cpu_clk_22M;
   wire                  uart_clk_10M;
   wire [31 : 0] iaddr;
   wire                  ice;
   wire [31 : 0] inst;
   wire                  dce;
   wire [31 : 0] daddr;
   wire [3 : 0  ] we;
   wire [31 : 0     ] din;
   wire [31 : 0     ] dout;
   wire [31 : 0     ] mem_src;
   wire [15:0] ioread_data;
   wire [15:0] IOwdata_o;
   wire LEDCtrl;
   wire [1:0] LEDCtrl_s;
   wire SwitchCtrl;
   wire [1:0] SwitchCtrl_s;
   wire KeyboardCtrl;
   wire setlow;
   wire sethigh;
   wire setdis;
   wire PwmCtrl;
   wire WatchdogCtrl;
   wire Counter16Ctrl;
   wire [3:0] key;
   clk_wiz_0 clocking
   (        
       // Clock in ports
       .clk_in1(sys_clk_100M),
       // Clock out ports
       .clk_out1(cpu_clk_22M),     // output clk_out1
       .clk_out2(uart_clk_10M)    // output port

   );      
   
   program_rom0 program_rom0 (
     .clka(cpu_clk_22M),    // input wire clka
     //.ena(ice),      // input wire ena
     .addra(iaddr[15:2]),  // input wire [13 : 0] addra
     .douta(inst)  // output wire [31 : 0] douta
   );

   MIPS32 mips32 (
       // input
       .cpu_clk_50M(cpu_clk_22M),
       .cpu_rst_n(sys_rst_n),
       .rom_inst(inst),
       // output
       .iaddr(iaddr),
       //.ice(ice),
       .dce(dce),
       .daddr(daddr),
       .we(we),
       .din(din),
       .dm(dout),
       .mem_src(mem_src),
       .ioread_data(ioread_data),
       .IOwdata_o(IOwdata_o),
       .LEDCtrl(LEDCtrl),
       .LEDCtrl_s(LEDCtrl_s),
       .SwitchCtrl(SwitchCtrl),
       .SwitchCtrl_s(SwitchCtrl_s),
       .KeyboardCtrl(KeyboardCtrl),
       .setlow(setlow),
       .sethigh(sethigh),
       .setdis(setdis),
       .PwmCtrl(PwmCtrl),
       .WatchdogCtrl(WatchdogCtrl),
       .Counter16Ctrl(Counter16Ctrl),
       .keyboardbreak(state)
   );

   dmemory32 dmemory32 (
       .ram_clk_i(cpu_clk_22M),    // input wire clka
       .wea(we),      // input wire [3 : 0] wea
       .dce(dce),
       .ram_adr_i(daddr[15:2]),  // input wire [14 : 0] addra
       .dina(din),    // input wire [31 : 0] dina
       .douta(dout),  // output wire [31 : 0] douta
       .mem_src(mem_src)
   );
   LED LED0(.led_clk(cpu_clk_22M),
            .led_rst(sys_rst_n),
            .ledcs(LEDCtrl),
            .ledaddr(LEDCtrl_s),
            .ledwdata(IOwdata_o),
            .ledData(led2N4)) ;
   dialSwitch dialSwitch0(.switclk(cpu_clk_22M),
                     .switrst(sys_rst_n),
                     .switcs(SwitchCtrl),
                     .switaddr(SwitchCtrl_s),
                     .switch_i(switch2N4),
                     .switchData(ioread_data)) ;     
   nixieTube nixieTube0(.clk(cpu_clk_22M),
                        .lowData_in(IOwdata_o),
                        .highData_in(IOwdata_o),
                        .specialDis_in(IOwdata_o),
                        .setlow(setlow),
                        .sethigh(sethigh),
                        .setdis(setdis),
                        .C(nixieTubeC),
                        .DP(DP),
                        .A(nixieTubeA));   
   keyboard keyboard0( .line(line),
                        .reset(sys_rst_n),
                        .key(key),
                        .state(state),
                        .row(row));
endmodule

