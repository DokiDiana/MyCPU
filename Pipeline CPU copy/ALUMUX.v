module ALUMUX(
    input        ALUSrc,
    input[31:0]  RD2,
    input[31:0]  immout,
    output[31:0] B
);

assign B = (ALUSrc) ? immout : RD2;

endmodule