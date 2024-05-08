`include "defines.v"
module DM(
    input clk,
    input MemWrite,
    input MemRead,
    input [31:0] addr,
    input [31:0] din,
    input [2:0] DMType,
    output reg[31:0] dout
);

    reg [7:0] dmem [127:0];

    always @(posedge clk)
        if (MemWrite)
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
        if(MemRead)
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
    end
endmodule