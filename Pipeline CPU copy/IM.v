module IM(
    input[31:0] AddrIn,
    output[31:0] instrOut
);

reg [31:0] ROM [1:0];
always@(*)
    begin
        ROM[0] <= 32'b0;
        ROM[1] <= 32'b0;
    end
    
assign instrOut = ROM[AddrIn];
endmodule