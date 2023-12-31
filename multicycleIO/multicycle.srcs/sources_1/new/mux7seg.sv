`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/10/11 11:14:27
// Design Name: 
// Module Name: SevenSegmentDisplay
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


module mux7seg(
    input logic [31:0] digit,
    input logic        clk,
    input logic        clr, 
    output logic [7:0] an,
    output logic [6:0] a2g
    );
    
    logic [2:0] s;
    logic [4:0] digit_out;
    logic [20:0] clkdiv;
    assign s = clkdiv[20:18];//clkdiv[20:18]


    always_comb
        case(s)
            0: digit_out = {1'b0,digit[3:0]};
            1: digit_out = {1'b0,digit[7:4]};
            2: digit_out = {1'b0,digit[11:8]};
            3: digit_out = {1'b1,digit[15:12]};
            4: digit_out = {1'b0,digit[19:16]};
            5: digit_out = {1'b0,digit[23:20]};
            6: digit_out = {1'b0,digit[27:24]};
            7: digit_out = {1'b0,digit[31:28]};
         default: digit_out = 5'b00000;
       endcase 
    always_comb
        case(s)
        0: an = 8'b11111110;
        1: an = 8'b11111101;
        2: an = 8'b11111011;
        3: an = 8'b11110111;
        4: an = 8'b11101111;
        5: an = 8'b11011111;
        6: an = 8'b10111111;
        7: an = 8'b01111111;
        default: an = 8'b00000000;
    endcase
    always @(posedge clk or posedge clr)
    begin
        if(clr == 1)
            clkdiv <= 0;
        else
            clkdiv <= clkdiv + 1;
    end
        
    Hex7Seg s7(.x(digit_out), .a2g(a2g));   

        
endmodule

