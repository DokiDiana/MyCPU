module PCAdder(
    input[31:0]  AddrIn,
    output[31:0] AddrOut
);

    assign AddrOut <= AddrIn + 1;

endmodule