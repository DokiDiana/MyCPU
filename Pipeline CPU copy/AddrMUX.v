module AddrMUX(
    input[2:0] PCSrc,
    input[31:0] result,
    input[31:0] PCAddr,
    input[31:0] OffsetAddr,
    input Zero,
    input less,
    output reg[31:0] Addr
);

always @ (*)
    begin
        if(PCSrc == 3'b000)
            Addr <= PCAddr;
        else if(PCSrc == 3'b001) // blt
            begin
                if(less == 1'b1)
                    Addr <= OffsetAddr;
                else
                    Addr <= PCAddr;
            end
        else if(PCSrc == 3'b010) // bge
            begin
                if(less == 1'b0)
                    Addr <= OffsetAddr;
                else
                    Addr <= PCAddr;
            end
        else if(PCSrc == 3'b011) // beq
            begin
                if(Zero == 1'b1)
                    Addr <= OffsetAddr;
                else
                    Addr <= PCAddr;
            end
        else if(PCSrc == 3'b100) // bne
            begin
                if(Zero == 1'b0)
                    Addr <= OffsetAddr;
                else
                    Addr <= PCAddr;
            end
        else if(PCSrc == 3'b101) // jal
            Addr <= OffsetAddr;
        else if(PCSrc == 3'b110) // jalr
            Addr <= result;
    end

endmodule