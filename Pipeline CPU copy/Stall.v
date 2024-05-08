module Stall(
    input[4:0] rs1,
    input[4:0] rs2,
    input[4:0] IDEX_rd,
    input[4:0] MEMWB_rd,
    input IDEX_MemRead,
    input MEMWB_RegWrite,
    output reg Stall
);

always @ (*)
    begin
        if(IDEX_MemRead)
            begin
                if(IDEX_rd == rs1 || IDEX_rd == rs2)
                    Stall <= 1'b1;
            end
        else if(MEMWB_RegWrite)
            begin
                if(MEMWB_rd == rs1 || MEMWB_rd == rs2)
                    Stall <= 1'b1;
            end
        else
            Stall <= 1'b0;
    end

endmodule