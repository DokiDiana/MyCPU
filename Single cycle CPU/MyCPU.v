// Display module
module seg7x16(
    input clk,
    input rstn,
    input disp_mode,
    input [63:0] i_data,
    output [7:0] o_seg,
    output [7:0] o_sel
);
// 时钟分频
reg [14:0] cnt;
wire seg7_clk;
always @ (posedge clk, negedge rstn)
    if(!rstn)
        cnt <= 0;
    else
        cnt <= cnt + 1'b1;
assign seg7_clk = cnt[14];

reg [2:0] seg7_addr;
always @ (posedge seg7_clk, negedge rstn)
    if(!rstn)
        seg7_addr <= 1'b0;
    else
        seg7_addr <= seg7_addr + 1'b1;
// 使能信号
reg [7:0] o_sel_r;
always @ (*)
    case(seg7_addr)
        7: o_sel_r = 8'b01111111;
        6: o_sel_r = 8'b10111111;
        5: o_sel_r = 8'b11011111;
        4: o_sel_r = 8'b11101111;
        3: o_sel_r = 8'b11110111;
        2: o_sel_r = 8'b11111011;
        1: o_sel_r = 8'b11111101;
        0: o_sel_r = 8'b11111110;
    endcase

reg [63:0] i_data_store;
always @ (posedge clk, negedge rstn)
    if(!rstn)
        i_data_store <= 1'b0;
    else
        i_data_store <= i_data;
// 显示模式: 数字模式或图形模式
reg [7:0] seg_data_r;
always @ (*)
    if(disp_mode == 1'b0)
        begin
            case(seg7_addr)
                0: seg_data_r = i_data_store[3:0];
                1: seg_data_r = i_data_store[7:4];
                2: seg_data_r = i_data_store[11:8];
                3: seg_data_r = i_data_store[15:12];
                4: seg_data_r = i_data_store[19:16];
                5: seg_data_r = i_data_store[23:20];
                6: seg_data_r = i_data_store[27:24];
                7: seg_data_r = i_data_store[31:28];
            endcase
        end
    else
        begin
            case(seg7_addr)
                0: seg_data_r = i_data_store[7:0];
                1: seg_data_r = i_data_store[15:8];
                2: seg_data_r = i_data_store[23:16];
                3: seg_data_r = i_data_store[31:24];
                4: seg_data_r = i_data_store[39:32];
                5: seg_data_r = i_data_store[47:40];
                6: seg_data_r = i_data_store[55:48];
                7: seg_data_r = i_data_store[63:56];
            endcase
        end
// 数字模式编码
reg [7:0] o_seg_r;
always @ (posedge clk, negedge rstn)
    if(!rstn)
        o_seg_r <= 8'hff;  
    else
        if(disp_mode == 1'b0)
            begin
                case(seg_data_r)
                        4'h0: o_seg_r <= 8'hC0;
                        4'h1: o_seg_r <= 8'hF9;
                        4'h2: o_seg_r <= 8'hA4;
                        4'h3: o_seg_r <= 8'hB0;
                        4'h4: o_seg_r <= 8'h99;
                        4'h5: o_seg_r <= 8'h92;
                        4'h6: o_seg_r <= 8'h82;
                        4'h7: o_seg_r <= 8'hF8;
                        4'h8: o_seg_r <= 8'h80;
                        4'h9: o_seg_r <= 8'h90;
                        4'hA: o_seg_r <= 8'h88;
                        4'hB: o_seg_r <= 8'h83;
                        4'hC: o_seg_r <= 8'hC6;
                        4'hD: o_seg_r <= 8'hA1;
                        4'hE: o_seg_r <= 8'h86;
                        4'hF: o_seg_r <= 8'h8E;
                default:
                    o_seg_r <= 8'hFF;
                endcase
            end
        else
            begin
                o_seg_r <= seg_data_r;
            end

assign o_sel = o_sel_r;
assign o_seg = o_seg_r;

endmodule

// RF module
module RF(
    input clk,
    input rstn,
    input RFWr,
    input [4:0] A1, A2, A3,
    input [31:0] WD,
    output [31:0] RD1, RD2
);

    reg [31:0] rf[31:0];

    integer i;
    // 初始化32个寄存器
    always @(posedge clk or negedge rstn)
        begin
            if(!rstn)
                begin
                    for(i = 0; i < 32; i = i + 1)
                        rf[i] = 32'b0;
                end
            else if(RFWr) 
                begin
                    rf[A3] <= WD;
                end
        end

    assign RD1 = (A1 != 0) ? rf[A1] : 0;
    assign RD2 = (A2 != 0) ? rf[A2] : 0;

endmodule

// ALU多路选择器 module
module ALUMUX(
    input ALUSrc,
    input [31:0] RD2,
    input[31:0] immout,
    output[31:0] B
);

    assign B = (ALUSrc) ? immout : RD2;

    endmodule

// ALU module
module ALU(
//  input[31:0] PC, 
    input signed[31:0] A, B,
    input[4:0] ALUOp,
    output reg signed[31:0] C
//  output reg[7:0] Zero
);

    // 所用指令的ALUOp编码
    `define  ALUOp_sub   5'b00000
    `define  ALUOp_lui   5'b00001
    `define  ALUOp_add   5'b00011
    `define  ALUOp_bne   5'b00101
    `define  ALUOp_blt   5'b00110
    `define  ALUOp_bge   5'b00111
    `define  ALUOp_bltu  5'b01000
    `define  ALUOp_bgeu  5'b01001
    `define  ALUOp_slt   5'b01010
    `define  ALUOp_sltiu 5'b01011
    `define  ALUOp_xor   5'b01100
    `define  ALUOp_xori  5'b01100
    `define  ALUOp_or    5'b01101
    `define  ALUOp_ori   5'b01101
    `define  ALUOp_and   5'b01110
    `define  ALUOp_sll   5'b01111
    `define  ALUOp_slli  5'b01111
    `define  ALUOp_srl   5'b10000
    `define  ALUOp_srli  5'b10000
    `define  ALUOp_sra   5'b10001

    // 指令对应的数据运算操作
    always @(*)
    begin
        case(ALUOp)
//          `ALUOp_nop:  C = A;
            `ALUOp_add:  C = A + B;
            `ALUOp_sub:  C = A - B;
            `ALUOp_and:  C = A & B;
            `ALUOp_or:   C = A | B;
            `ALUOp_xor:  C = A ^ B;
            `ALUOp_slt:  C = {31'b0, (A < B)};
            `ALUOp_sll:  C = A >> B;
            `ALUOp_srl:  C = A << B;
            `ALUOp_sra:  C = A >>> B;
            `ALUOp_bne:  C = {31'b0, (A == B)};
            `ALUOp_blt:  C = {31'b0, (A >= B)};
            `ALUOp_bge:  C = {31'b0, (A < B)};
            `ALUOp_bltu: C = {31'b0, ($unsigned(A) >= $unsigned(B))};
            `ALUOp_bgeu: C = {31'b0, ($unsigned(A) < $unsigned(B))};
            `ALUOp_slli: C = A << B;
            `ALUOp_srli: C = A >> B;
            `ALUOp_xori: C = A ^ B;
            `ALUOp_ori:  C = A | B;
            `ALUOp_lui:  C = B;
            `ALUOp_sltiu: C = {31'b0,($unsigned(A) < $unsigned(B))};
        endcase
//      Zero = (C == 0)? 1 : 0;
    end

endmodule

// DM module
module DM(
    input clk,
    input DMWr,
    input [7:0] addr,
    input [31:0] din,
    input [2:0] DMType,
    output reg [31:0] dout
);
    // 数据类型编码
    `define  dm_word               3'b000
    `define  dm_halfword           3'b001
    `define  dm_halfword_unsigned  3'b010
    `define  dm_byte               3'b011
    `define  dm_byte_unsigned      3'b100

    reg [7:0] dmem [127:0];

    always @(posedge clk)
        if (DMWr)
        begin 
            case(DMType)
                `dm_byte: dmem[addr]          <= din[7:0];
                `dm_byte_unsigned: dmem[addr] <= din[7:0];
                `dm_halfword: 
                begin 
                    dmem[addr]                <= din[7:0];
                    dmem[addr + 1]            <= din[15:8];
                end
                `dm_halfword_unsigned:
                begin
                    dmem[addr]                <= din[7:0];
                    dmem[addr+1]              <= din[15:8];
                end
                `dm_word: 
                begin
                    dmem[addr]                <= din[7:0];
                    dmem[addr + 1]            <= din[15:8];
                    dmem[addr + 2]            <= din[23:16];
                    dmem[addr + 3]            <= din[31:24];
                end
            endcase
        end
    // 符号扩展方式
    always @(*)
    begin
        case(DMType)
        `dm_byte:
            dout <= {{24{dmem[addr][7]}}, dmem[addr][7:0]};
        `dm_byte_unsigned:
            dout <= {24'b0, dmem[addr][7:0]};
        `dm_halfword:
            dout <= {{16{dmem[addr + 1][7]}}, dmem[addr+1][7:0], dmem[addr][7:0]};
        `dm_halfword_unsigned:
            dout <= {16'b0, dmem[addr + 1][7:0], dmem[addr][7:0]};
        `dm_word: 
            dout <= {dmem[addr + 3][7:0], dmem[addr + 2][7:0], dmem[addr + 1][7:0], dmem[addr][7:0]};
        default: dout <= 32'b0;
        endcase
    end
endmodule

// RF写入多路选择器 module
module RFWDMUX(
    input[31:0] dout,
    input[31:0] aluout,
    input[1:0] WDSel,
    input[31:0] immout,
    output reg[31:0] WD
);
    // 数据来源编码
    `define WDSel_FromALU 2'b00
    `define WDSel_FromMEM 2'b01
    `define WDSel_FromPC  2'b10
    always @(*)
        begin
	        case(WDSel)
		        `WDSel_FromALU: WD <= aluout;
		        `WDSel_FromMEM: WD <= dout;
//		        `WDSel_FromPC:  WD <= PC_out + 4;
	        endcase
        end

endmodule

// Control module
module ctrl (
    input[6:0] Op,
    input[6:0] Funct7,
    input[3:0] Funct3,
//  input Zero,
    output RegWrite,
    output MemWrite,
    output[5:0] EXTOp,
    output[4:0] ALUOp,
    output[2:0] ALUSrc,
    output[2:0] DMType,
    output[1:0] WDSel
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
    wire sbtype   =  Op[6] &  Op[5] & ~Op[4] & ~Op[3] & ~Op[2] &  Op[1] &  Op[0];
    wire i_beq   = sbtype & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_bne   = sbtype & ~Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire i_blt   = sbtype &  Funct3[2] & ~Funct3[1] & ~Funct3[0];
    wire i_bge   = sbtype &  Funct3[2] & ~Funct3[1] &  Funct3[0];
    wire i_bltu  = sbtype &  Funct3[2] &  Funct3[1] & ~Funct3[0];
    wire i_bgeu  = sbtype &  Funct3[2] &  Funct3[1] &  Funct3[0];
    // J type
    wire jtype   = ~Op[6] &  Op[5] &  Op[4] & ~Op[3] &  Op[2] &  Op[1] &  Op[0];
    wire i_jal   = jtype;
    wire i_jalr  = jtype & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

    // RegWrite MemWrite ALUSrc 信号编码
    assign RegWrite  = rtype | itype_l | itype_r | i_jalr | i_jal | i_lui | i_auipc;
    assign MemWrite  = stype;
    assign ALUSrc    = itype_l | itype_r | stype | i_jal | i_jalr | i_auipc | i_lui;
    // WDSel 信号编码
    assign WDSel[1]  = i_lui | i_jal | i_jalr;
    assign WDSel[0]  = itype_l;
    
    assign ALUOp[4]  = i_srl  | i_sra  | i_srli  | i_srai;
    assign ALUOp[3]  = i_andi | i_and  | i_ori   | i_or   | i_bltu | i_bgeu | i_slt | i_slti | i_sltiu | i_sltu | i_xor  | i_xori  | i_sll  | i_slli;
    assign ALUOp[2]  = i_andi | i_and  | i_ori   | i_or   | i_beq  | i_sub  | i_bne | i_blt  | i_bge   | i_xor  | i_xori | i_sll   | i_slli;
    assign ALUOp[1]  = i_jal  | i_jalr | itype_l | stype  | i_addi | i_add  | i_and | i_andi | i_auipc | i_blt  | i_bge  | i_slt   | i_slti | i_sltiu | i_sltu | i_slli | i_sll;
    assign ALUOp[0]  = i_jal  | i_jalr | itype_l | stype  | i_addi | i_ori  | i_add | i_or   | i_bne   | i_bge  | i_bgeu | i_sltiu | i_sltu | i_slli  | i_sll  | i_sra  | i_srai | i_lui;

    assign EXTOp[5]  = i_slli  | i_srli | i_srai;
    assign EXTOp[4]  = itype_l | i_addi | i_slti | i_sltiu | i_xori | i_ori | i_andi | i_jalr;
    assign EXTOp[3]  = stype;
    assign EXTOp[2]  = sbtype;
    assign EXTOp[1]  = i_auipc | i_lui;
    assign EXTOp[0]  = i_jal;

    assign DMType[2] = i_lbu;
    assign DMType[1] = i_lb | i_sb;
    assign DMType[0] = i_lh | i_sh | i_lb | i_sb;

endmodule

// EXT module
module EXT(
    input[4:0] iimm_shamt,
    input[11:0] iimm,
    input[11:0] simm,
    input[11:0] bimm,
    input[19:0] uimm,
    input[19:0] jimm,
    input[5:0] EXTOp,
    output reg[31:0] immout
);
// 立即数编码
`define EXT_CTRL_ITYPE_SHAMT  6'b100000
`define EXT_CTRL_ITYPE	      6'b010000
`define EXT_CTRL_STYPE	      6'b001000
`define EXT_CTRL_BTYPE	      6'b000100
`define EXT_CTRL_UTYPE	      6'b000010
`define EXT_CTRL_JTYPE	      6'b000001
// 立即数符号扩展方式
always@ (*)
    begin
        case (EXTOp)
		    `EXT_CTRL_ITYPE_SHAMT: immout <= {27'b0,iimm_shamt[4:0]};
		    `EXT_CTRL_ITYPE: immout       <= { {20{ iimm[11]}},iimm[11:0]};
		    `EXT_CTRL_STYPE: immout       <= { {20{ simm[11]}},simm[11:0]};
		    `EXT_CTRL_BTYPE: immout       <= { {19{ bimm[11]}},bimm[11:0], 1'b0} ;
		    `EXT_CTRL_UTYPE: immout       <= {uimm[19:0], 12'b0}; 
		    `EXT_CTRL_JTYPE: immout       <= {{11{ jimm[19]}},jimm[19:0],1'b0};
		    default:  immout <= 32'b0;
	    endcase
    end
endmodule


// main module
module MyCPU(clk, rstn, sw_i, disp_seg_o, disp_an_o);
    input clk;
    input rstn;
    input [15:0] sw_i;
    output [7:0] disp_seg_o, disp_an_o;

    // Decode
    wire [6:0]  Op;
    wire [6:0]  Funct7;
    wire [2:0]  Funct3;
    wire [4:0]  rs1, rs2, rd;
    wire [11:0] iimm;
    wire [11:0] simm;
    wire [4:0]  shamt;
    wire [19:0] uimm;

    assign Op         = instr[6:0];                             // op
    assign Funct7     = instr[31:25];                           // funct7
    assign Funct3     = instr[14:12];                           // funct3
    assign rs1        = instr[19:15];                           // rs1
    assign rs2        = instr[24:20];                           // rs2
    assign rd         = instr[11:7];                            // rd
    assign iimm       = instr[31:20];                           // I型指令立即数
    assign iimm_shamt = instr[24:20];                           // UJ型指令立即数
    assign simm       = {instr[31:25],instr[11:7]};             // S型指令立即数
    assign bimm       = {instr[31], instr[7], instr[30:25], instr[11:8]};  // SB型指令立即数
    assign uimm       = instr[31:12];                           // U型指令立即数
    assign jimm       = {instr[31], instr[19:12], instr[20], instr[30:21]}; // J型指令立即数

    // Control signals
    wire RegWrite;
    wire MemWrite;
    wire[5:0] EXTOp;
    wire[4:0] ALUOp;
    wire ALUSrc;
    wire[2:0] DMType;
    wire[1:0] WDSel;

    // choose display mode
    always @ (sw_i)
        if(sw_i[0] == 0)
            begin
                case(sw_i[14:11])
                    4'b1000: display_data = instr;
                    4'b0100: display_data = reg_data;
                    4'b0010: display_data = alu_disp_data;
                    4'b0001: display_data = dmem_data;
                    default:
                    display_data = instr;
                endcase
            end
        else
            begin
                display_data = led_disp_data;
            end


// build ROM module
wire [31:0] instr;
reg [3:0] rom_addr;
parameter IM_CODE_NUM = 15;

 dist_mem_gen U_IM(
    .a(rom_addr),
    .spo(instr)
);

always @ (posedge Clk_CPU or negedge rstn)
    if(!rstn)
        rom_addr = 4'b0;
    else
        if(sw_i[14] == 1'b1)
            begin
                rom_addr = rom_addr + 1'b1;
                if(rom_addr == IM_CODE_NUM)
                    begin
                        rom_addr = 4'b0;
                    end
            end
        else
            rom_addr = rom_addr;


// build RF module
reg [31:0] reg_data;
parameter RF_NUM = 10;
reg [5:0] reg_addr;
wire[31:0] RD1, RD2;
wire[31:0] WD;

RF U_RF(
    .clk(Clk_CPU), 
    .rstn(rstn), 
    .RFWr(RegWrite), 
    .A1(rs1),
    .A2(rs2),
    .A3(rd),
    .WD(WD),
    .RD1(RD1),
    .RD2(RD2)
);

always @ (posedge Clk_CPU or negedge rstn) begin
    if(!rstn)
        reg_addr = 5'b0;
    else if(sw_i[13] == 1'b1 && sw_i[1]==1'b0) begin
            if(reg_addr == RF_NUM)
                reg_addr = 5'b0;
            reg_data = {reg_addr, U_RF.rf[reg_addr][27:0]};
            reg_addr = reg_addr + 1'b1;
    end
    else
        reg_addr = reg_addr;
end


// build ALUMUX module
wire[31:0] immout;
ALUMUX U_ALUMUX(
    .RD2(RD2),
    .immout(immout),
    .ALUSrc(ALUSrc),
    .B(B)
);


// build ALU module
reg [31:0] alu_disp_data;
parameter ALU_NUM = 5;
reg [2:0] alu_addr;
wire[31:0] B;
wire[31:0] aluout;
wire[7:0] Zero;

ALU U_ALU(
    .A(RD1),
    .B(B),
    .ALUOp(ALUOp),
    .C(aluout)
//  .Zero(Zero)
);

always @(posedge Clk_CPU or negedge rstn)
    begin
        if(!rstn)
            alu_addr = 3'b0;
        else if(sw_i[12] == 1)
            begin
                if (alu_addr == ALU_NUM)
                    alu_addr = 3'b0;
                case (alu_addr)
                3'b000: alu_disp_data = RD1;
                3'b001: alu_disp_data = B;
                3'b010: alu_disp_data = aluout;
                3'b011: alu_disp_data = Zero;
                default: alu_disp_data = 32'hFFFFFFFF;
                endcase
                alu_addr = alu_addr + 1'b1;
            end
    end


// build DM module
reg [31:0] dmem_data;
parameter DM_DATA_NUM = 10;
reg [5:0] dmem_addr;
wire [31:0] dout;

DM U_DM (
    .clk(Clk_CPU),
    .DMWr(MemWrite),
    .addr(aluout),
    .din(RD2),
    .DMType(DMType),
    .dout(dout)
);

always @(posedge Clk_CPU or negedge rstn)
    begin
    if (!rstn)
        dmem_addr = 6'b0;
    else if (sw_i[11] == 1 && sw_i[1] == 0)
        begin
            if (dmem_addr == DM_DATA_NUM)
            begin
                dmem_addr = 6'd0;
            end
            dmem_data = {dmem_addr, U_DM.dmem[dmem_addr]};
            dmem_addr = dmem_addr + 1'b1;
        end
    end


// build RFWDMUX module
RFWDMUX U_MUX(
    .dout(dout),
    .aluout(aluout),
    .WDSel(WDSel),
    .immout(immout),
    .WD(WD)
);


// build Crtl module 
ctrl U_ctrl(
    .Op(Op),
    .Funct7(Funct7),
    .Funct3(Funct3),
    .WDSel(WDSel),
    .MemWrite(MemWrite),
    .DMType(DMType),
    .ALUOp(ALUOp),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .EXTOp(EXTOp)
);


// build EXT module
EXT U_EXT(
    .iimm_shamt(iimm_shamt),
    .iimm(iimm),
    .simm(simm),
    .bimm(bimm),
    .uimm(uimm),
    .jimm(jimm),
    .EXTOp(EXTOp),
    .immout(immout)
);


// build seg7x16 module
seg7x16 u_seg7x16(
    .clk(clk),
    .rstn(rstn),
    .i_data(display_data),
    .disp_mode(sw_i[0]),
    .o_seg(disp_seg_o),
    .o_sel(disp_an_o)
);
 
reg [31:0] clk_div;
wire Clk_CPU;

always @ (posedge clk or negedge rstn)
    begin
        if(!rstn)
            clk_div <= 0;
        else
            clk_div <= clk_div + 1'b1;
    end

assign Clk_CPU = (sw_i[15]) ? clk_div[27] : clk_div[25];

reg [63:0] display_data;
reg [5:0] led_data_addr;
reg [63:0] led_disp_data;

parameter LED_DATA_NUM = 48;

reg [63:0] LED_DATA[47:0];
initial 
    begin
        LED_DATA[0]  = 64'hFFFFFFFEFEFEFEFE;
        LED_DATA[1]  = 64'hFFFEFEFEFEFEFFFF;
        LED_DATA[2]  = 64'hDEFEFEFEFFFFFFFF;
        LED_DATA[3]  = 64'hCEFEFEFFFFFFFFFF;
        LED_DATA[4]  = 64'hC2FFFFFFFFFFFFFF;
        LED_DATA[5]  = 64'hC1FEFFFFFFFFFFFF;
        LED_DATA[6]  = 64'hF1FCFFFFFFFFFFFF;
        LED_DATA[7]  = 64'hFDF8F7FFFFFFFFFF;
        LED_DATA[8]  = 64'hFFF8F3FFFFFFFFFF;
        LED_DATA[9]  = 64'hFFFBF1FEFFFFFFFF;
        LED_DATA[10] = 64'hFFFFF9F8FFFFFFFF;
        LED_DATA[11] = 64'hFFFFFDF8F7FFFFFF;
        LED_DATA[12] = 64'hFFFFFFF9F1FFFFFF;
        LED_DATA[13] = 64'hFFFFFFFFF1FCFFFF;
        LED_DATA[14] = 64'hFFFFFFFFF9F8FFFF;
        LED_DATA[15] = 64'hFFFFFFFFFFF8F3FF;
        LED_DATA[16] = 64'hFFFFFFFFFFFBF1FE;
        LED_DATA[17] = 64'hFFFFFFFFFFFFF9BC;
        LED_DATA[18] = 64'hFFFFFFFFFFFFBDBC;
        LED_DATA[19] = 64'hFFFFFFFFBFBFBFBC;
        LED_DATA[20] = 64'hFFFFBFBFBFBFBFFF;
        LED_DATA[21] = 64'hFFBFBFBFBFBFFFFF;
        LED_DATA[22] = 64'hAFBFBFBFFFFFFFFF;
        LED_DATA[23] = 64'h2737FFFFFFFFFFFF;
        LED_DATA[24] = 64'h277777FFFFFFFFFF;
        LED_DATA[25] = 64'h7777777777FFFFFF;
        LED_DATA[26] = 64'hFFFF7777777777FF;
        LED_DATA[27] = 64'hFFFFFF7777777777;
        LED_DATA[28] = 64'hFFFFFFFFFF777771;
        LED_DATA[29] = 64'hFFFFFFFFFFFF7770;
        LED_DATA[30] = 64'hFFFFFFFFFFFFFFC8;
        LED_DATA[31] = 64'hFFFFFFFFFFFFE7CE;
        LED_DATA[32] = 64'hFFFFFFFFFFFFC7CF;
        LED_DATA[33] = 64'hFFFFFFFFFFDEC7FF;
        LED_DATA[34] = 64'hFFFFFFFFF7CEDFFF;
        LED_DATA[35] = 64'hFFFFFFFFC7CFFFFF;
        LED_DATA[36] = 64'hFFFFFFFEC7EFFFFF;
        LED_DATA[37] = 64'hFFFFFFCECFFFFFFF;
        LED_DATA[38] = 64'hFFFFDECEFFFFFFFF;
        LED_DATA[39] = 64'hFFFFC7CFFFFFFFFF;
        LED_DATA[40] = 64'hFFDEC7FFFFFFFFFF;
        LED_DATA[41] = 64'hF7CEDFFFFFFFFFFF;
        LED_DATA[42] = 64'hA7CFFFFFFFFFFFFF;
        LED_DATA[43] = 64'hA7AFFFFFFFFFFFFF;
        LED_DATA[44] = 64'hAFBFBFBFFFFFFFFF;
        LED_DATA[45] = 64'hBFBFBFBFBFFFFFFF;
        LED_DATA[46] = 64'hFFFFBFBFBFBFBFFF;
        LED_DATA[47] = 64'hFFFFFFFFBFBFBFBD;
    end
// LED_DATA
always @ (posedge Clk_CPU or negedge rstn)
    if(!rstn)
        begin
            led_data_addr = 6'd0;
            led_disp_data = 64'b1;
        end
    else
        if(sw_i[0] == 1'b1)
            begin
                if (led_data_addr == LED_DATA_NUM)
                    begin
                        led_data_addr = 6'd0;
                        led_disp_data = 64'b1;
                    end
                led_disp_data = LED_DATA[led_data_addr];
                led_data_addr = led_data_addr + 1'b1;
            end
        else
            led_data_addr = led_data_addr;

endmodule
