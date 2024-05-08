module IDEX(
    input            clk,
    input            rstn,
    input[31:0]      AddrIn,
    input[31:0]      RD1In,
    input[31:0]      RD2In,
    input[4:0]       rdIn,
    input[31:0]      immIn,
    input            RegWriteIn,
    input            MemWriteIn,
    input            MemReadIn,
    input[4:0]       ALUOpIn,
    input[2:0]       ALUSrcIn,
    input[1:0]       WDSelIn,
    input[2:0]       DMTypeIn,
    input[2:0]       PCSrcIn,
    output reg[31:0] AddrOut,
    output reg[31:0] RD1Out,
    output reg[31:0] RD2Out,
    output reg[4:0]  rdOut,
    output reg[31:0] immOut,
    output reg       RegWriteOut,
    output reg       MemWriteOut,
    output reg       MemReadOut,
    output reg[4:0]  ALUOpOut,
    output reg[2:0]  ALUSrcOut,
    output reg[1:0]  WDSelOut,
    output reg[2:0]  DMTypeOut,
    output reg[2:0]  PCSrcOut
);

always @ (posedge clk)
    begin
        if(!rstn)
            begin
                AddrOut     <= AddrIn;
                RD1Out      <= RD1In;
                RD2Out      <= RD2In;
                rdOut       <= rdIn;
                immOut      <= immIn;
                RegWriteOut <= RegWriteIn;
                MemWriteOut <= MemWriteIn;
                MemReadOut  <= MemReadIn;
                ALUOpOut    <= ALUOpIn;
                ALUSrcOut   <= ALUSrcIn;
                WDSelOut    <= WDSelIn;
                DMTypeOut   <= DMTypeIn;
                PCSrcOut    <= PCSrcIn;
            end
    end

endmodule