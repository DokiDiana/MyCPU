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

    assign Op       = instr[6:0];             // op
    assign Funct7   = instr[31:25];           // funct7
    assign Funct3   = instr[14:12];           // funct3
    assign rs1      = instr[19:15];           // rs1
    assign rs2      = instr[24:20];           // rs2
    assign rd       = instr[11:7];            // rd

    // Control signals
    wire      RegWrite;
    wire      MemWrite;
    wire      MemRead;
    wire[5:0] EXTOp;
    wire[4:0] ALUOp;
    wire      ALUSrc;
    wire[2:0] DMType;
    wire[1:0] WDSel;    // MemtoReg
    wire[2:0] PCSrc;

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

// PC module
    wire[31:0] PC_AddrIn;
    wire[31:0] PC_AddrOut;
PC U_PC(
    .clk(Clk_CPU),
    .rstn(rstn),
    .AddrIn(PC_AddrIn),
    .AddrOut(PC_AddrOut)
);

// PCAdder
wire[31:0] PCAdder_AddrOut; 
PCAdder U_PCAdder(
    .AddrIn(PC_AddrOut),
    .AddrOut(PCAdder_AddrOut)
);


// build IM module
wire [31:0] IM_instrOut;
// reg [3:0] rom_addr;
parameter IM_CODE_NUM = 15;

IM U_IM(
    .AddrIn(PC_AddrOut),
    .instrOut(IM_instrOut)
);

/*
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
*/

wire[31:0] IFID_AddrOut;
wire[31:0] instr;
IFID U_IFID(
    .clk(Clk_CPU),
    .rstn(rstn),
    .AddrIn(PC_AddrOut),
    .instrIn(IM_instrOut),
    .AddrOut(IFID_AddrOut),
    .instrOut(instr)
);

// build Crtl module 
CTRL U_CTRL(
    .Op(Op),
    .Funct7(Funct7),
    .Funct3(Funct3),
    .WDSel(WDSel),
    .MemWrite(MemWrite),
    .MemRead(MemRead),
    .DMType(DMType),
    .ALUOp(ALUOp),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .EXTOp(EXTOp),
    .PCSrc(PCSrc)
);

// build EXT module
wire[31:0] immout;
EXT U_EXT(
    .instr(instr),
    .EXTOp(EXTOp),
    .immout(immout)
);


// Stall
wire Stall;
Stall U_Stall(
    .rs1(rs1),
    .rs2(rs2),
    .IDEX_rd(IDEX_rdOut),
    .MEMWB_rd(MEMWB_rdOut),
    .IDEX_MemRead(IDEX_MemReadOut),
    .MEMWB_RegWrite(MEMWB_RegWriteOut),
    .Stall(Stall)
);

// build RF module
reg [31:0] reg_data;
parameter  RF_NUM = 10;
reg [5:0]  reg_addr;
wire[31:0] RD1, RD2;
wire[31:0] WD;

RF U_RF(
    .clk(Clk_CPU), 
    .rstn(rstn), 
    .RFWr(MEMWB_RegWriteOut), 
    .rs1(rs1),
    .rs2(rs2),
    .rd(MEMWB_rdOut),
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

// ID/EX流水线寄存器
    wire[31:0] IDEX_AddrOut;  
    wire[31:0] IDEX_RD1Out;
    wire[31:0] IDEX_RD2Out;
    wire[4:0]  IDEX_rdOut;
    wire[31:0] IDEX_immOut;
    wire       IDEX_RegWriteOut;
    wire       IDEX_MemWriteOut;
    wire       IDEX_MemReadOut;
    wire[4:0]  IDEX_ALUOpOut;
    wire[2:0]  IDEX_ALUSrcOut;
    wire[1:0]  IDEX_WDSelOut;
    wire[2:0]  IDEX_DMTypeOut;
    wire[2:0]  IDEX_PCSrcOut;
//  wire[4:0] IDEX_rs1Out;
//  wire[4:0] IDEX_rs2Out;
IDEX U_IDEX(
    .clk(Clk_CPU),
    .rstn(rstn),
    .AddrIn(IFID_AddrOut),
    .RD1In(RD1),
    .RD2In(RD2),
    .rdIn(rd),
    .immIn(immout),
    .RegWriteIn(RegWrite),
    .MemWriteIn(MemWrite),
    .MemReadIn(MemRead),
    .ALUOpIn(ALUOp),
    .ALUSrcIn(ALUSrc),
    .WDSelIn(WDSel),
    .DMTypeIn(DMType),
    .PCSrcIn(PCSrc),
    .AddrOut(IDEX_AddrOut),
    .RD1Out(IDEX_RD1Out),
    .RD2Out(IDEX_RD2Out),
    .rdOut(IDEX_rdOut),
    .immOut(IDEX_immOut),
    .RegWriteOut(IDEX_RegWriteOut),
    .MemWriteOut(IDEX_MemWriteOut),
    .MemReadOut(IDEX_MemReadOut),
    .ALUOpOut(IDEX_ALUOpOut),
    .ALUSrcOut(IDEX_ALUSrcOut),
    .WDSelOut(IDEX_WDSelOut),
    .DMTypeOut(IDEX_DMTypeOut),
    .PCSrcOut(IDEX_PCSrcOut)
);

// OffsetAdder
wire[31:0] OffsetAdder_AddrOut;
OffsetAdder U_OffsetAdder(
    .immIn(IDEX_immOut),
    .AddrIn(IDEX_AddrOut),
    .AddrOut(OffsetAdder_AddrOut)
);

// resetAddrAdder
wire[31:0] Adder_resetAddrOut;
setAddrAdder U_resetAddrAdder(
    .resetAddrIn(IDEX_AddrOut),
    .resetAddrOut(Adder_resetAddrOut)
);

// build ALUMUX module

ALUMUX U_ALUMUX(
    .RD2(IDEX_RD2Out),
    .immout(IDEX_immOut),
    .ALUSrc(IDEX_ALUSrcOut),
    .B(B)
);


// build ALU module
reg [31:0] alu_disp_data;
parameter  ALU_NUM = 5;
reg [2:0]  alu_addr;
wire[31:0] B;
wire[31:0] aluout;
wire Zero;
wire less;

ALU U_ALU(
    .A(IDEX_RD1Out),
    .B(B),
    .ALUOp(IDEX_ALUOpOut),
    .C(aluout),
    .Zero(Zero),
    .less(less)
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

// EX/MEM流水线寄存器
    wire[31:0]  EXMEM_AddrOut;
    wire[31:0]  EXMEM_resetAddrOut;
    wire[31:0]  EXMEM_resultOut;
    wire        EXMEM_ZeroOut;
    wire        EXMEM_lessOut;
    wire[31:0]  EXMEM_RD2Out;
    wire[31:0]  EXMEM_immOut;
    wire[4:0]   EXMEM_rdOut;
    wire        EXMEM_RegWriteOut;
    wire        EXMEM_MemWriteOut;
    wire        EXMEM_MemReadOut;
    wire[1:0]   EXMEM_WDSelOut;
    wire[2:0]   EXMEM_DMTypeOut;
    wire[2:0]   EXMEM_PCSrcOut;

EXMEM U_EXMEM(
    .clk(Clk_CPU),
    .rstn(rstn),
    .AddrIn(OffsetAdder_AddrOut),
    .resetAddrIn(Adder_resetAddrOut),
    .CIn(aluout),
    .ZeroIn(Zero),
    .lessIn(less),
    .RD2In(IDEX_RD2Out),
    .immIn(IDEX_immOut),
    .rdIn(IDEX_rdOut),
    .RegWriteIn(IDEX_RegWriteOut),
    .MemWriteIn(IDEX_MemWriteOut),
    .MemReadIn(IDEX_MemReadOut),
    .WDSelIn(IDEX_WDSelOut),
    .DMTypeIn(IDEX_DMTypeOut),
    .PCSrcIn(IDEX_PCSrcOut),
    .AddrOut(EXMEM_AddrOut),
    .resetAddrOut(EXMEM_resetAddrOut),
    .COut(EXMEM_resultOut),
    .ZeroOut(EXMEM_ZeroOut),
    .lessOut(EXMEM_lessOut),
    .RD2Out(EXMEM_RD2Out),
    .immOut(EXMEM_immOut),
    .rdOut(EXMEM_rdOut),
    .RegWriteOut(EXMEM_RegWriteOut),
    .MemWriteOut(EXMEM_MemWriteOut),
    .MemReadOut(EXMEM_MemReadOut),
    .WDSelOut(EXMEM_WDSelOut),
    .DMTypeOut(EXMEM_DMTypeOut),
    .PCSrcOut(EXMEM_PCSrcOut)
);

// AddrMUX
AddrMUX U_AddrMUX(
    .PCSrc(EXMEM_PCSrcOut),
    .result(EXMEM_resultOut),
    .PCAddr(PCAdder_AddrOut),
    .OffsetAddr(EXMEM_AddrOut),
    .Zero(EXMEM_ZeroOut),
    .less(EXMEM_lessOut),
    .Addr(PC_AddrIn)
);

// Flush
wire Flush;
Flush U_Flush(
    .Zero(EXMEM_ZeroOut),
    .less(EXMEM_lessOut),
    .PCSrc(EXMEM_PCSrcOut),
    .Flush(Flush)
);

// build DM module
reg[31:0]  dmem_data;
parameter  DM_DATA_NUM = 10;
reg[5:0]   dmem_addr;
wire[31:0] dout;

DM U_DM(
    .clk(Clk_CPU),
    .MemWrite(EXMEM_MemWriteOut),
    .MemRead(EXMEM_MemReadOut),
    .addr(EXMEM_resultOut),
    .din(EXMEM_RD2Out),
    .DMType(EXMEM_DMTypeOut),
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

// MEM/WB流水线寄存器
    wire[31:0] MEMWB_resetAddrOut;
    wire[31:0] MEMWB_dout;
    wire[31:0] MEMWB_resultOut;
    wire[31:0] MEMWB_immOut;
    wire[31:0] MEMWB_rdOut;
    wire       MEMWB_RegWriteOut;
    wire[1:0]  MEMWB_WDSelOut;

MEMWB U_MEMWB(
    .clk(Clk_CPU),
    .rstn(rstn),
    .resetAddrIn(EXMEM_resetAddrOut),
    .doutIn(dout),
    .resultIn(EXMEM_resultOut),
    .immIn(EXMEM_immOut),
    .rdIn(EXMEM_rdOut),
    .RegWriteIn(EXMEM_RegWriteOut),
    .WDSelIn(EXMEM_WDSelOut),
    .resetAddrOut(MEMWB_resetAddrOut),
    .dout(MEMWB_dout),
    .resultOut(MEMWB_resultOut),
    .immOut(MEMWB_immOut),
    .rdOut(MEMWB_rdOut),
    .RegWriteOut(MEMWB_RegWriteOut),
    .WDSelOut(MEMWB_WDSelOut)
);


// build RFWDMUX module
RFWDMUX U_MUX(
    .dout(MEMWB_dout),
    .aluout(MEMWB_resultOut),
    .WDSel(MEMWB_WDSelOut),
    .immout(MEMWB_immOut),
    .resetAddr(MEMWB_resetAddrOut),
    .WD(WD)
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
reg [5:0]  led_data_addr;
reg [63:0] led_disp_data;

parameter  LED_DATA_NUM = 48;

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