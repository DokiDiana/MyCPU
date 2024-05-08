module OffsetAdder(
    input[31:0]  immIn,
    input[31:0]  AddrIn,
    output[31:0] AddrOut
);

    assign AddrOut = AddrIn + immIn;

endmodule