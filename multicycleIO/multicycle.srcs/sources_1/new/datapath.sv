`timescale 1ns / 1ps

module datapath(
    input logic clk, reset, memtoreg, regdst, iord,
    input logic alusrca,pcwrite, branch, branchBne,irwrite, regwrite,
    input logic [2:0] alucontrol,
    input logic [1:0] alusrcb, pcsrc,
    input logic [31:0] rd,
    input logic immExt,
    output logic [5:0] op, funct,
    output logic [31:0] adr, writedata);
    
    logic [31:0] pc_,pc,aluout,instr,data,wd3,rd1,rd2,a,signimm,zeroimm,aftersignimm;
    logic [31:0] srca,srcb,aluresult,jump,imm;
    logic [4:0] a3;
    logic pcen,zero;
    
    assign pcen = (branchBne & ~zero) | (branch & zero) | pcwrite;
    assign op = instr[31:26];
    assign funct = instr[5:0];
    assign jump[31:28] = pc[31:28];
    
    
    flopr#(32) datareg(clk,reset,rd,data);
    mux2#(5)     muxa3(instr[20:16],instr[15:11],regdst,a3);
    mux2#(32)    muxwd3(aluout,data,memtoreg,wd3);
    
    flopenr#(32) pcreg(clk,reset,pcen,pc_,pc);
    mux2#(32)    muxaddr(pc,aluout,iord,adr);
    
    regfile      rf(clk,regwrite,instr[25:21],instr[20:16],a3,wd3,
                    rd1,rd2);
    flopr#(32) fa(clk,reset,rd1,a);
    flopr#(32) fb(clk,reset,rd2,writedata);
    flopenr#(32) instrreg(clk,reset,irwrite,rd,instr);
    
    signext      se(instr[15:0],signimm);
    sl2          immsh(signimm, aftersignimm) ;
    
    zenext       ze(instr[15:0],zeroimm);
    mux2#(32)    muximmsimm(zeroimm,signimm,immExt,imm);    // 1 -> sign
    
    mux2#(32)    muxsrca(pc, a, alusrca, srca);
    mux4#(32)    mux4srcb(writedata, 32'b100, imm,aftersignimm,////signimm->imm
                            alusrcb,srcb);
    mux4#(32)    muxpc(aluresult,aluout,jump,0,pcsrc,pc_);//jump problems
    sl2          pcjump(instr[25:0],jump[27:0]);
    
    alu          alu(srca,srcb,alucontrol,aluresult,zero);
    flopr#(32)   falu(clk,reset,aluresult,aluout);
    
    
endmodule
    
    
    
    
    
    
    
    
    
    

