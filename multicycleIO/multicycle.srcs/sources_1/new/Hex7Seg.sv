`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/10/11 11:26:53
// Design Name: 
// Module Name: Hex7Seg
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


module Hex7Seg(
input logic [4:0] x,
output logic [6:0] a2g
    );
    always_comb
    case(x)
        5'b00000: a2g <= 7'b1000000;
        5'b00001: a2g <= 7'b1111001;
        5'b00010: a2g <= 7'b0100100;
        5'b00011: a2g <= 7'b0110000;
        5'b00100: a2g <= 7'b0011001;
        5'b00101: a2g <= 7'b0010010;
        5'b00110: a2g <= 7'b0000010;
        5'b00111: a2g <= 7'b1111000;
        5'b01000: a2g <= 7'b0000000;
        5'b01001: a2g <= 7'b0010000;
        5'b01010: a2g <= 7'b0001000;
        5'b01011: a2g <= 7'b0000011;
        5'b01100: a2g <= 7'b1000110;
        5'b01101: a2g <= 7'b0100001;
        5'b01110: a2g <= 7'b0000110;
        5'b01111: a2g <= 7'b0001110;
        5'b10000: a2g <= 7'b0110111;
        default:   a2g <= 7'b1000000; 
    endcase 
endmodule
