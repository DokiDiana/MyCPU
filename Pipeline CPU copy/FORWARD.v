module FORWARD(
    input IDEX_rs1,
    input IDEX_rs2,
    input EXMEM_RegWrite,
    input EXMEM_rd,
    input MEMWB_RegWrite,
    input MEMWB_rd,
    output reg[2:0] F1,
    output reg[2:0] F2
);