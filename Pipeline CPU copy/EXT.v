`include "defines.v"
module EXT(
    input[31:0] instr,
    input[5:0] EXTOp,
    output reg[31:0] immout
);
    wire[4:0] iimm_shamt;
    wire[11:0] iimm;
    wire[11:0] simm;
    wire[11:0] bimm;
    wire[19:0] uimm;
    wire[19:0] jimm;

    assign iimm_shamt = instr[24:20];                           // UJ型指令立即数
    assign iimm       = instr[31:20];                           // I型指令立即数
    assign simm       = {instr[31:25],instr[11:7]};             // S型指令立即数
    assign bimm       = {instr[31], instr[7], instr[30:25], instr[11:8]};  // SB型指令立即数
    assign uimm       = instr[31:12];                           // U型指令立即数
    assign jimm       = {instr[31], instr[19:12], instr[20], instr[30:21]}; // J型指令立即数
// 立即数符号扩展方式
always@ (*)
    begin
        case (EXTOp)
		    `EXT_CTRL_ITYPE_SHAMT: immout <= {27'b0,iimm_shamt[4:0]};
		    `EXT_CTRL_ITYPE:       immout <= { {20{ iimm[11]}},iimm[11:0]};
		    `EXT_CTRL_STYPE:       immout <= { {20{ simm[11]}},simm[11:0]};
		    `EXT_CTRL_BTYPE:       immout <= { {19{ bimm[11]}},bimm[11:0], 1'b0};
		    `EXT_CTRL_UTYPE:       immout <= {uimm[19:0], 12'b0}; 
		    `EXT_CTRL_JTYPE:       immout <= {{11{ jimm[19]}},jimm[19:0],1'b0};
		    default:  immout <= 32'b0;
	    endcase
    end
endmodule