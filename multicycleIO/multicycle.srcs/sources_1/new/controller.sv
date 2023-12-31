
module controller(input  logic      clk,reset,
                  input  logic[5:0] op, funct,
                  output logic      pcwrite,
                  output logic      memwrite, 
                  output logic      irwrite,
                  output logic      regwrite,
                  output logic      alusrca,
                  output logic      branch,branchBne,iord,memtoreg,
                  output logic      regdst,
                  output logic[1:0] alusrcb,pcsrc,
                  output logic[2:0] alucontrol,
                  output logic      immExt  // ExtOP
                  );
    
    logic[2:0] aluop;
    
    maindec md(clk,reset,op,pcwrite,memwrite,irwrite,regwrite,
               alusrca,branch,branchBne,iord,memtoreg,regdst,alusrcb,pcsrc,aluop,immExt);
    aludec  ad(funct, aluop, alucontrol);
    
endmodule