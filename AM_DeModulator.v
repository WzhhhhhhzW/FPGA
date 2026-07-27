`timescale 1ns / 1ps

module AM_DeModulator(
    input clk, 
    input rst,
    input signed [9:0]AM_in,
    output signed [15:0]Base_out    //基带输出，采样率降低精度大幅提高，1.5MSPS 15bit
);

reg signed [9:0]abs;
always@(posedge clk)
    abs <= AM_in > 0 ? AM_in : -AM_in;
    
ave_FIR LPF0(
    .clk(clk),
    .rst(rst),
    .din(abs),
    .dout(Base_out)
);

endmodule
