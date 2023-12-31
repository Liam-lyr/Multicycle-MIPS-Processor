
module mips(input logic clk,
            input logic reset,
            output logic [31:0] dataadr,
            output logic [31:0] writedata,
            output logic memwrite,
            input logic [31:0] readdata);
    
    logic [5:0] op,funct;
    logic zero,pcwrite,irwrite,regwrite,alusrca;
    logic branch,branchBne,iord,memtoreg,regdst;
    logic [1:0] alusrcb,pcsrc;
    logic [2:0] alucontrol;
    logic extOp;
    
    controller c(clk,reset,op,funct,pcwrite,memwrite,irwrite,
                regwrite,alusrca,branch,branchBne,iord,memtoreg,regdst,
                alusrcb,pcsrc,alucontrol,extOp);
    
    datapath dp(clk, reset, memtoreg, regdst, iord, alusrca,
            pcwrite, branch,branchBne,irwrite, regwrite,alucontrol,
            alusrcb, pcsrc,readdata,extOp,op, funct,dataadr, writedata);        
          
endmodule