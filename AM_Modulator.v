`timescale 1ns / 1ps


module AM_Modulator(
    input clk,
    input rst,
    input signed [15:0]Base,    //16bit 单声道（采样率<<200MSPS）
    input [6:0]Ma,              //调制度,0-127对应0-99.2%
    input [8:0]LO_freq,         //步进0.1M，值范围0.8M-30M，共292个频点
    output reg signed [9:0]AM_out
);

wire signed [9:0]LO;
wire [11:0]real_freq;    //频率限制
assign real_freq = LO_freq > 300 ? 300 : LO_freq < 8 ? 8 : LO_freq;
DDS LO_dds_0(
    .clk(clk),
    .rst(rst),
    .LO_freq(real_freq),
    .dds_o(LO)
);

reg signed [22:0]Base_r;    
reg signed [32:0]Base_add;
reg signed [15:0]Ma_mul;

always@(posedge clk or negedge rst)begin
    if(!rst)begin
        Base_r <= 0;
        Ma_mul <= 0;
        Base_add <= 0;
        AM_out <= 0;
    end else begin
        Base_r <= Base;
        Ma_mul <= (Base_r * Ma)>>>7;
        Base_add <= Ma_mul + 32768;    //增益偏置0.5
        AM_out <= (LO * Base_add)>>>16;
    end
end

endmodule
