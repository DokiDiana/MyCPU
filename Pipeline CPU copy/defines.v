
// 所用指令的ALUOp编码
`define  ALUOp_sub   5'b00000
`define  ALUOp_lui   5'b00001
`define  ALUOp_add   5'b00011
`define  ALUOp_bne   5'b00101
`define  ALUOp_blt   5'b00110
`define  ALUOp_bge   5'b00111
`define  ALUOp_beq   5'b01000
`define  ALUOp_bltu  5'b01000
`define  ALUOp_bgeu  5'b01001
`define  ALUOp_slt   5'b01010
`define  ALUOp_sltiu 5'b01011
`define  ALUOp_xor   5'b01100
`define  ALUOp_or    5'b01101
`define  ALUOp_and   5'b01110
`define  ALUOp_sll   5'b01111
`define  ALUOp_srl   5'b10000
`define  ALUOp_sra   5'b10001
`define  ALUOp_jalr  5'b10010
`define  ALUOp_xori  5'b10011
`define  ALUOp_ori   5'b10100
`define  ALUOp_slli  5'b10101
`define  ALUOp_srli  5'b10110


// 立即数编码
`define EXT_CTRL_ITYPE_SHAMT  6'b100000
`define EXT_CTRL_ITYPE	      6'b010000
`define EXT_CTRL_STYPE	      6'b001000
`define EXT_CTRL_BTYPE	      6'b000100
`define EXT_CTRL_UTYPE	      6'b000010
`define EXT_CTRL_JTYPE	      6'b000001

// 数据类型编码
`define  dm_word               3'b000
`define  dm_halfword           3'b001
`define  dm_halfword_unsigned  3'b010
`define  dm_byte               3'b011
`define  dm_byte_unsigned      3'b100