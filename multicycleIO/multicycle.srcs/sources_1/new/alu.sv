`timescale 1ns / 1ps

module alu(input logic [31:0]a,b,
           input logic [2:0]alucontrol,
           output logic [31:0]out,
           output logic zero);
    always_comb
        case(alucontrol)
            3'b000:out=a&b;
            3'b001:out=a|b;
            3'b010:out=a+b;
            3'b011:out=(a<b)?1:0;
            3'b100:out=a&(~b);//-b
            3'b101:out=a|(~b);//-b
            3'b110:out=a-b;
            3'b111:out=(a<b)?1:0;
            default:out={32{1'b0}};
        endcase
    assign zero=(out!=0)?0:1;
endmodule
