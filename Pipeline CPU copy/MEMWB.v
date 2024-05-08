module MEMWB(
    input            clk,
    input            rstn,
    input[31:0]      resetAddrIn,
    input[31:0]      doutIn,
    input[31:0]      resultIn,
    input[31:0]      immIn,
    input[4:0]       rdIn,
    input            RegWriteIn,
    input[1:0]       WDSelIn,
    input[2:0]       DMTypeIn,
    output reg[31:0] resetAddrOut,
    output reg[31:0] dout,
    output reg[31:0] resultOut,
    output reg[31:0] immOut,
    output reg[4:0]  rdOut,
    output reg       RegWriteOut,
    output reg[1:0]  WDSelOut
);

always @ (posedge clk)
    begin
        if(!rstn)
            begin
                resetAddrOut <= resetAddrIn
                dout         <= doutIn;
                resultOut    <= resultIn;
                immOut       <= immIn;
                rdOut        <= rdIn;
                RegWriteOut  <= RegWriteIn;
                WDSelOut     <= WDSelIn;
            end
    end

endmodule