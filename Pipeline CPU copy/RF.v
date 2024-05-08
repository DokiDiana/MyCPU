module RF(
    input clk,
    input rstn,
    input RFWr,
    input [4:0] rs1, rs2, rd,
    input [31:0] WD,
    output [31:0] RD1, RD2
);

    reg [31:0] rf[31:0];

    always @(posedge clk)
        if(RFWr && rd != 0) 
            begin
                rf[rd] <= WD;
//              $display("pc = %h: x%d = %h", pc, rd, WD);
            end

    assign RD1 = (rs1 != 0) ? rf[rs1] : 0;
    assign RD2 = (rs2 != 0) ? rf[rs2] : 0;

endmodule