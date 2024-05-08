module CTRL(
    input[6:0]  Op,
    input[6:0]  Funct7,
    input[3:0]  Funct3,
    output      RegWrite,
    output      MemWrite,
    output      MemRead,
    output[5:0] EXTOp,
    output[4:0] ALUOp,
    output[2:0] ALUSrc,
    output[2:0] DMType,
    output[1:0] WDSel,
    output[2:0] PCSrc
);
    // R type
    wire rtype   = ~Op[6] &  Op[5] &  Op[4] & ~Op[3] & ~Op[2] &  Op[1] &  Op[0];
    wire i_add   = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_sub   = rtype & ~Funct7[6] &  Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_sll   = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] & ~Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire i_slt   = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] & ~Funct3[2] &  Funct3[1] & ~Funct3[0];
    wire i_sltu  = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] & ~Funct3[2] &  Funct3[1] &  Funct3[0];
    wire i_xor   = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] &  Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_srl   = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] &  Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire i_sra   = rtype & ~Funct7[6] &  Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] &  Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire i_or    = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] &  Funct3[2] &  Funct3[1] & ~Funct3[0];
    wire i_and   = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] &  Funct3[2] &  Funct3[1] &  Funct3[0];
    // I type
    wire itype_l = ~Op[6] & ~Op[5] & ~Op[4] & ~Op[3] & ~Op[2] &  Op[1] &  Op[0];
    wire i_lb    = itype_l & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_lh    = itype_l & ~Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire i_lw    = itype_l & ~Funct3[2] &  Funct3[1] & ~Funct3[0];
    wire i_lbu   = itype_l &  Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_lhu   = itype_l &  Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire itype_r = ~Op[6] & ~Op[5] &  Op[4] & ~Op[3] & ~Op[2] &  Op[1] &  Op[0];
    wire i_addi  = itype_r & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_slti  = itype_r & ~Funct3[2] &  Funct3[1] & ~Funct3[0];
    wire i_sltiu = itype_r & ~Funct3[2] &  Funct3[1] &  Funct3[0];
    wire i_xori  = itype_r &  Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_ori   = itype_r &  Funct3[2] &  Funct3[1] & ~Funct3[0];
    wire i_andi  = itype_r &  Funct3[2] &  Funct3[1] &  Funct3[0];
    wire i_slli  = itype_r & ~Funct3[2] & ~Funct3[1] &  Funct3[0] & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0];
    wire i_srli  = itype_r &  Funct3[2] & ~Funct3[1] &  Funct3[0] & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0];
    wire i_srai  = itype_r &  Funct3[2] & ~Funct3[1] &  Funct3[0] & ~Funct7[6] &  Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0];
    // S type
    wire stype   = ~Op[6] &  Op[5] & ~Op[4] & ~Op[3] & ~Op[2] &  Op[1] &  Op[0];
    wire i_sb    = stype & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_sh    = stype & ~Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire i_sw    = stype & ~Funct3[2] &  Funct3[1] & ~Funct3[0];
    // U type
    wire utype   = ~Op[6] &  Op[5] &  Op[4] & ~Op[3] &  Op[2] &  Op[1] &  Op[0];
    wire i_lui   = utype;
    wire i_auipc = utype;
    // B type
    wire sbtype  =  Op[6] &  Op[5] & ~Op[4] & ~Op[3] & ~Op[2] &  Op[1] &  Op[0];
    wire i_beq   = sbtype & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_bne   = sbtype & ~Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire i_blt   = sbtype &  Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_bge   = sbtype &  Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire i_bltu  = sbtype &  Funct3[2] &  Funct3[1] & ~Funct3[0];
    wire i_bgeu  = sbtype &  Funct3[2] &  Funct3[1] &  Funct3[0];
    // J type
    wire jtype_jal   =  Op[6] &  Op[5] & ~Op[4] &  Op[3] &  Op[2] &  Op[1] &  Op[0];
    wire jtype_jalr  =  Op[6] &  Op[5] & ~Op[4] & ~Op[3] &  Op[2] &  Op[1] &  Op[0];
    wire i_jal   = jtype_jal;
    wire i_jalr  = jtype_jalr & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

    // RegWrite MemWrite ALUSrc 信号编码
    assign RegWrite  = rtype | itype_l | itype_r | i_jalr | i_jal | i_lui | i_auipc;
    assign MemWrite  = stype;
    assign MemRead   = itype_l;
    assign ALUSrc    = itype_l | itype_r | stype | i_jal | i_jalr | i_auipc | i_lui;
    // WDSel 信号编码
    assign WDSel[1]  = i_lui   | i_jal | i_jalr;
    assign WDSel[0]  = itype_l | i_lui;
    
    assign ALUOp[4]  = i_srl   | i_sra   | i_srli  | i_srai | i_jalr | i_xori | i_ori   | i_slli;
    assign ALUOp[3]  = i_andi  | i_and   | i_or    | i_bltu | i_bgeu | i_slt  | i_slti  | i_sltiu | i_sltu  | i_xor   | i_sll   | i_beq;
    assign ALUOp[2]  = i_andi  | i_and   | i_or    | i_beq  | i_sub  | i_bne  | i_blt   | i_bge   | i_xor   | i_sll   | i_slli  | i_ori   | i_srli;
    assign ALUOp[1]  = i_jalr  | itype_l | stype   | i_addi | i_add  | i_and  | i_andi  | i_auipc | i_blt   | i_bge   | i_slt   | i_slti  | i_sltiu | i_sltu | i_sll | i_xori | i_srli;
    assign ALUOp[0]  = itype_l | stype   | i_addi  | i_add  | i_or   | i_bne  | i_bge   | i_bgeu  | i_sltiu | i_sltu  | i_slli  | i_sll   | i_sra   | i_srai | i_lui | i_xori;

    assign EXTOp[5]  = i_slli  | i_srli | i_srai;
    assign EXTOp[4]  = itype_l | i_addi | i_slti | i_sltiu | i_xori | i_ori | i_andi | i_jalr;
    assign EXTOp[3]  = stype;
    assign EXTOp[2]  = sbtype;
    assign EXTOp[1]  = i_auipc | i_lui;
    assign EXTOp[0]  = i_jal;

    assign DMType[2] = i_lbu;
    assign DMType[1] = i_lb | i_sb;
    assign DMType[0] = i_lh | i_sh | i_lb | i_sb;

    assign PCSrc[2] = i_jalr | i_bne;
    assign PCSrc[1] = i_beq  | i_bge | i_jal | i_jalr;
    assign PCSrc[0] = i_blt  | i_beq | i_jal;

endmodule