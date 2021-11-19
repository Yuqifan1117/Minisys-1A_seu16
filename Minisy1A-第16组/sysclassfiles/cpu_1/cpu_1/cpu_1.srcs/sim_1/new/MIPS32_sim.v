`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/29 15:09:01
// Design Name: 
// Module Name: MIPS32_sim
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


module MIPS32_sim();
        reg CLOCK;
        reg rst;
        reg [31:0] id_inst_i;
    
        initial
        begin
            CLOCK = 1'b0;
            forever #5 CLOCK = ~CLOCK;
        end
        
        initial
        begin
            rst = 1'b1;
            #5 rst = 1'b0;
            #1000 $stop;
        end
        
          initial begin
          #5   id_inst_i = 32'h2001000A;
          #10   id_inst_i = 32'h20020003;
          #10   id_inst_i = 32'h3403000D;
          #10   id_inst_i = 32'h00222020;
          #10   id_inst_i = 32'h10640002;
          #10   id_inst_i = 32'h00221824;
          #10   id_inst_i = 32'h00223025;
          #10   id_inst_i = 32'h00223022;
          #10   id_inst_i = 32'h14640001;
          #10   id_inst_i = 32'hAD02000A;
          #10   id_inst_i = 32'h8D04000A;
          #10   id_inst_i = 32'h10440001;
          #10   id_inst_i = 32'h00233024;
          #10   id_inst_i = 32'h00C23826;
          end      
    
        MIPS32 MIPS32_0
        (    
            .cpu_clk_50M(CLOCK),
            .cpu_rst_n(rst),
            .inst(id_inst_i)
        );
endmodule
