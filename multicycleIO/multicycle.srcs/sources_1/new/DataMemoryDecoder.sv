`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/04/09 16:53:30
// Design Name: 
// Module Name: DataMemoryDecoder
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


module DataMemoryDecoder(input  logic           clk, writeEN,
                         input  logic [31:0]    addr,
                         input  logic [31:0]    writeData,
                         output logic [31:0]    readData,
                         
                         input  logic           IOclock,
                         input  logic           reset,
                         input  logic           btnL, btnR,
                         input  logic [15:0]    switch,
                         output logic [7:0]     AN,
                         output logic [6:0]     A2G
    );
    logic we1, we2;
    logic [31:0] out1, out2;
    logic [11:0] led;

     assign we1 = (addr[7]==1'b1)?writeEN:0;
     assign we2 = writeEN&(addr[7]==1'b0);
     
    mem      DataMemory(clk, we2, addr, writeData ,out1);
    IO        IO(IOclock, reset, addr[7], we1, addr[3:2], writeData, out2, btnL, btnR,switch, led);
    mux7seg m7seg({switch, {4'b0000}, led}, IOclock, reset, AN, A2G);     
    assign readData=addr[7]?out2:out1;
endmodule
