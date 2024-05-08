module FORWARDMUX(
    input IDEX_RD1,
    input IDEX_RD2,
    input[2:0] F1,
    input[2:0] F2,
    output reg[31:0] RD1Out,
    output reg[31:0] RD2Out
);