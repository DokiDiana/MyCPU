module IFID(
    input            clk,
    input            rstn,
    input[31:0]      AddrIn,
    input[31:0]      instrIn,
    output reg[31:0] AddrOut,
    output reg[31:0] instrOut
);

always @ (posedge clk)
    begin
        if(!rstn)
            begin
                AddrOut  <= AddrIn;
                instrOut <= instrIn;
            end
        else
            begin
                AddrOut  <= 32'h0;
                instrOut <= 32'h0;
            end
    end

endmodule