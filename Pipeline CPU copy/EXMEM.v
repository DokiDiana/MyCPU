module EXMEM(
    input            clk,
    input            rstn,
    input[31:0]      AddrIn,
    input[31:0]      resetAddrIn,
    input[31:0]      CIn,
    input            ZeroIn,
    input            lessIn,
    input[31:0]      RD2In,
    input[31:0]      immIn,
    input[4:0]       rdIn,
    input            RegWriteIn,
    input            MemWriteIn,
    input            MemReadIn,
    input[1:0]       WDSelIn,
    input[2:0]       DMTypeIn,
    input[2:0]       PCSrcIn,
    output reg[31:0] AddrOut,
    output reg[31:0] resetAddrOut,
    output reg[31:0] COut,
    output reg       ZeroOut,
    output reg       lessOut,
    output reg[31:0] RD2Out,
    output reg[31:0] immOut,
    output reg[4:0]  rdOut,
    output reg       RegWriteOut,
    output reg       MemWriteOut,
    output reg       MemReadOut,
    output reg[1:0]  WDSelOut,
    output reg[2:0]  DMTypeOut,
    output reg[2:0]  PCSrcOut
);
always @ (posedge clk)
    begin
        if(!rstn)
            begin
                AddrOut      <= AddrIn;
                resetAddrOut <= resetAddrIn;
                COut         <= CIn;
                ZeroOut      <= ZeroIn;
                lessOut      <= lessIn;
                RD2Out       <= RD2In;
                immOut       <= immIn;
                rdOut        <= rdIn;
                RegWriteOut  <= RegWriteIn;
                MemWriteOut  <= MemWriteIn;
                MemReadOut   <= MemReadIn;
                WDSelOut     <= WDSelIn;
                DMTypeOut    <= DMTypeIn;
            end
    end

endmodule