module PC(
    input clk,
    input rstn,
    input[31:0] AddrIn,
    output reg[31:0] AddrOut
);

always @ (posedge clk)
    begin
        if(!rstn)
            AddrOut <= AddrIn;
        else
            AddrOut <= 32'h0;
    end
    
endmodule 
