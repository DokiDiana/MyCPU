module RFWDMUX(
    input[31:0] dout,
    input[31:0] aluout,
    input[1:0] WDSel,
    input[31:0] immout,
    input[31:0] resetAddr,
    output reg[31:0] WD
);
    // 数据来源编码
    `define WDSel_FromALU 2'b00
    `define WDSel_FromMEM 2'b01
    `define WDSel_FromPC  2'b10
    `define WDSel_FromImm 2'b11 
    always @(*)
        begin
	        case(WDSel)
		        `WDSel_FromALU: WD <= aluout;
		        `WDSel_FromMEM: WD <= dout;
		        `WDSel_FromPC:  WD <= resetAddr;
                `WDSek_FromImm: WD <= immout;
	        endcase
        end

endmodule