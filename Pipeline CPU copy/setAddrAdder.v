module setAddrAdder(
    input[31:0]   resetAddrIn,
    output[31:0]  resetAddrOut
);

assign resetAddrOut <= resetAddrIn + 1;

endmodule