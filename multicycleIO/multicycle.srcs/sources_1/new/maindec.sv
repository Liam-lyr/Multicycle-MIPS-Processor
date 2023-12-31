
module maindec(input logic clk,
               input logic reset,
               input logic [5:0] op,
               output logic pcwrite,memwrite,irwrite,regwrite,
               output logic alusrca,branch,branchBne,iord,memtoreg,regdst,
               output logic [1:0] alusrcb,pcsrc,
               output logic [2:0] aluop,
               output logic immExt);
    // States
    localparam FETCH    =4'b0000;   // IFetch
    localparam DECODE   =4'b0001;   // RFetch/ID
    localparam MEMADR   =4'b0010;   // LW/SW MemAdr
    localparam MEMRD    =4'b0011;   // LW MemFetch
    localparam MEMWB    =4'b0100;   // LW Finish
    localparam MEMWR    =4'b0101;   // SW Finish
    localparam RTYPEEX  =4'b0110;   // R-Type Exec
    localparam RTYPEWB  =4'b0111;   // R-Type Finish
    localparam BEQEX    =4'b1000;   // Beq Exec (Finish)
    localparam ADDIEX   =4'b1001;   // Addi Exec
    localparam IWB      =4'b1010;   // I-Type Finish
    localparam JEX      =4'b1011;   // Jump Exec(Finish)
    localparam BNEEX    =4'b1100;   // Bne Exec(Finish)
    localparam ANDIEX   =4'b1101;   // Andi Exec
    localparam ORIEX    =4'b1110;   // Ori Exec
    
    // OP Code
    localparam LW       =6'b100011;
    localparam SW       =6'b101011;
    localparam RTYPE    =6'b000000;
    localparam BEQ      =6'b000100;
    localparam ADDI     =6'b001000;
    localparam J        =6'b000010;
    localparam ANDI     =6'b001100;
    localparam ORI      =6'b001101;
    localparam BNE      =6'b000101;
    
    logic [3:0] state,nextstate;
    logic [17:0] controls;
    
    //state register
    always_ff@(posedge clk or posedge reset)
        if(reset) state <= FETCH;
        else      state <= nextstate;
    
    //next state logic
    always_comb
        case(state)
            FETCH:  nextstate = DECODE;
            DECODE: case(op)
                    LW:       nextstate = MEMADR;
                    SW:       nextstate = MEMADR;
                    RTYPE:    nextstate = RTYPEEX;
                    BEQ:      nextstate = BEQEX;
                    ADDI:     nextstate = ADDIEX;
                    J:        nextstate = JEX;
                    ANDI:     nextstate = ANDIEX;
                    ORI:      nextstate = ORIEX;
                    BNE:      nextstate = BNEEX;
                    default:  nextstate = 4'bx; // never happen
            endcase
            MEMADR: case(op)                    // LW/SW MemAdr
                    LW:       nextstate = MEMRD;// LW MemFetch
                    SW:       nextstate = MEMWR;// SW Finish
                    default:  nextstate = 4'bx;
            endcase
            MEMRD:  nextstate = MEMWB;          // -> LW Finish
            MEMWB:  nextstate = FETCH;
            MEMWR:  nextstate = FETCH;
            RTYPEEX:nextstate = RTYPEWB;
            RTYPEWB:nextstate = FETCH;
            BEQEX:  nextstate = FETCH;
            ADDIEX: nextstate = IWB;
            IWB:    nextstate = FETCH;
            JEX:    nextstate = FETCH;
            ANDIEX: nextstate = IWB;
            ORIEX:  nextstate = IWB;
            BNEEX:  nextstate = FETCH;
            default:nextstate = 4'bx;//never happen
        endcase
        
    assign {pcwrite,memwrite,irwrite,regwrite,
            alusrca,branch,branchBne,iord,memtoreg,regdst,
            alusrcb,pcsrc,aluop,immExt} = controls;
     
    always_comb
        case(state)
            FETCH:  controls = 18'b1010_000000_01_00_000_1;
            DECODE: controls = 18'b0000_000000_11_00_000_1;
            MEMADR: controls = 18'b0000_100000_10_00_000_1;
            MEMRD:  controls = 18'b0000_000100_00_00_000_1;
            MEMWB:  controls = 18'b0001_000010_00_00_000_1;
            MEMWR:  controls = 18'b0100_000100_00_00_000_1;
            RTYPEEX:controls = 18'b0000_100000_00_00_100_1; // aluop: default(R-Type)
            RTYPEWB:controls = 18'b0001_000001_00_00_000_1;
            BEQEX:  controls = 18'b0000_110000_00_01_001_1; // aluop: Sub
            ADDIEX: controls = 18'b0000_100000_10_00_000_1;
            IWB:    controls = 18'b0001_000000_00_00_000_1;
            JEX:    controls = 18'b1000_000000_00_10_000_1;
            BNEEX:  controls = 18'b0000_101000_00_01_001_1; // aluop: Sub 
            ANDIEX: controls = 18'b0000_100000_10_00_010_0; // aluop: And
            ORIEX:  controls = 18'b0000_100000_10_00_011_0; // aluop: Or
            default:controls = 18'hxxxx; //neverhappen
        endcase            
endmodule



/*FETCH:  controls = 17'h5010;//0101000000010000
            DECODE: controls = 17'h0030;//0000000000110000
            MEMADR: controls = 17'h0420;//0000010000100000
            MEMRD:  controls = 17'h0100;//0000000100000000
            MEMWB:  controls = 17'h0880;//0000100010000000
            MEMWR:  controls = 17'h2100;//0010000100000000
            RTYPEEX:controls = 17'h0402;//0000010000000010
            RTYPEWB:controls = 17'h0840;//0000100001000000
            BEQEX:  controls = 17'h0605;//0000011000000101
            ADDIEX: controls = 17'h0420;//0000010000100000
            ADDIWB: controls = 17'h0800;//0000100000000000
            JEX:    controls = 17'h4008;//0100000000001000
            ANDIEX: controls = 17'h
            default:controls = 17'hxxxx;//neverhappen*/