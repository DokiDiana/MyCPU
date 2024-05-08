module Flush(
    input Zero,
    input less,
    input[2:0] PCSrc,
    output reg Flush
);

always @ (*)
    begin
        if(less == 0 || Zero == 0)
            Flush <= 1'b0;
        else
            Flush <= 1'b1;
    end

endmodule
