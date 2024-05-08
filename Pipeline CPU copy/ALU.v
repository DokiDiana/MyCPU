`include "defines.v"
module ALU(
    input signed[31:0] A, B,
    input[4:0] ALUOp,
    output reg signed[31:0] C,
    output Zero,
    output less
);

    // 指令对应的数据运算操作
    always @(*)
    begin
        case(ALUOp)
//          `ALUOp_nop:   C = A;
            `ALUOp_add:   C = A + B;
            `ALUOp_sub:   C = A - B;
            `ALUOp_and:   C = A & B;
            `ALUOp_or:    C = A | B;
            `ALUOp_xor:   C = A ^ B;
            `ALUOp_slt:   C = {31'b0, (A < B)};
            `ALUOp_sll:   C = A << B;
            `ALUOp_srl:   C = A >> B;
            `ALUOp_sra:   C = A >>> B;
            `ALUOp_beq:   C = A - B;
            `ALUOp_bne:   C = A - B;
            `ALUOp_blt:   C = A - B;
            `ALUOp_bge:   C = A - B;
//          `ALUOp_bltu:  C = A - B;
//          `ALUOp_bgeu:  C = A - B;
            `ALUOp_slli:  C = A << B;
            `ALUOp_srli:  C = A >> B;
            `ALUOp_xori:  C = A ^ B;
            `ALUOp_ori:   C = A | B;
            `ALUOp_lui:   C = B;
            `ALUOp_sltiu: C = {31'b0,($unsigned(A) < $unsigned(B))};
            `ALUOp_jalr:  C = A + B;
        endcase
    end
    assign Zero = (C == 0);
    assign less = C[31];
endmodule