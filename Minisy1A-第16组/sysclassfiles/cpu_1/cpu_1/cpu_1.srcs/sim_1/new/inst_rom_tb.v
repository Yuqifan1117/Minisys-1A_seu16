`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/10 10:57:18
// Design Name: 
// Module Name: inst_rom_tb
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


module inst_rom_tb();
    reg clka,ena;
    reg [14:0] addra;
    wire [31:0] douta;
    
    program_rom0 uut(
        .clka(clka), //input wire clka
        //.ena(ena), //input wire ena
        .addra(addra), //input wire [13:0] addra
        .douta(douta) //output wire [31:0] douta
    );
    
    initial begin
    
        clka=0;ena=0;addra=0;
        #100 ena=1;addra=1;
        #50 addra =3;
        #50 addra =6;
        #50 addra =10;
        #50 $finish;
    end
    
    always begin
        #10 clka=~clka;
    end
endmodule
