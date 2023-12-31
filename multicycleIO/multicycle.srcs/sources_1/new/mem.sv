`timescale 1ns / 1ps


module mem(input logic clk,
           input logic we,
           input logic [31:0] addr,
           input logic [31:0] wd,
           output logic [31:0] rd);

    logic [31:0]RAM[63:0];
    
    //imem
    initial
        $readmemh("memfile_task2.dat",RAM);
    
    assign rd=RAM[addr[31:2]];
    //dmem
    always_ff@(posedge clk)
        if(we)
            RAM[addr[31:2]]<=wd;
    
endmodule
