//`timescale 1ns / 1ps


module textbench_task2();
    logic clk;
    logic reset;

    logic[31:0] writedata , dataadr;
    logic       memwrite;

//instantiate device to be tested
    top dut(clk, reset, writedata, dataadr, memwrite);

//initialize test
    initial
        begin
            reset <= 1; #22; reset <= 0;
        end
   
//generate clock to sequence tests
    always
        begin
            clk <= 1; #5; clk <= 0; #5;
        end
    
    always@(negedge clk)
            begin
                if(memwrite) begin
                    if(dataadr===128&writedata===6)begin
                        $display("Simulation succeeded");
                        $stop;
                    end else if(dataadr!==128) begin
                        $display("Simulation failed");
                        $stop;
                    end
                end
            end
    endmodule
 
//check results


 
