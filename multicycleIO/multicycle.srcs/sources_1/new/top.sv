`timescale 1ns / 1ps

module top(input logic clk,reset,
           output logic [31:0] writedata,dataadr,
           output logic memwrite);
           
    logic [31:0] readdata;
    
    mips mips(clk,reset,dataadr,writedata,memwrite,readdata);
    mem  mem(clk,memwrite,dataadr,writedata,readdata);        
    
endmodule

//module top(input  logic CLK100MHZ, BTNC, //reset
//           input  logic BTNL, BTNR,
//           input  logic [15:0] SW,
//           output logic [7:0]  AN,
//           output logic [6:0] A2G     
//    );
//    logic[31:0] pc, readData;
//    logic IOclock;
//    assign IOclock = ~CLK100MHZ;
//    logic Write;
//    logic [31:0] dataAdr,writeData;
    
//    mips mips(CLK100MHZ, BTNC, dataAdr, writeData,Write ,readData);
//    DataMemoryDecoder demeDecoder(CLK100MHZ, Write, dataAdr, writeData, readData, 
//                        IOclock, BTNC, BTNL, BTNR, SW, AN, A2G);

//endmodule







