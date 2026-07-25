module IIR_filter #(
    parameter IN_WIDTH  = 12,                                 // 输入数据位宽
    parameter N         = 8,                                  // 右移位数，用于 IIR 反馈路径的缩放
    parameter ACC_WIDTH = IN_WIDTH + N + 2                    // 累加器位宽，确保中间计算不溢出
)(
    input  wire clk,                                          // 时钟信号
    input  wire rst_n,                                        // 低电平有效复位
    input  wire signed [IN_WIDTH-1:0] in_data,                // 有符号输入数据
    output wire signed [IN_WIDTH-1:0] out_data                // 有符号输出数据
);

reg signed [ACC_WIDTH-1:0] cache0;                             // 反馈寄存器 0，保存当前累加值
reg signed [ACC_WIDTH-1:0] cache1;                             // 反馈寄存器 1，保存上一次的截断值
reg signed [ACC_WIDTH-1:0] sum;                                // 中间求和结果寄存器


wire signed [ACC_WIDTH-1:0] cut;                               // 右移后的反馈值
assign cut = cache0 >>> N;                                     // 算术右移，保持符号位


wire signed [ACC_WIDTH-1:0] in_ext;                            // 输入数据符号扩展到累加器宽度
assign in_ext = {{(ACC_WIDTH-IN_WIDTH){in_data[IN_WIDTH-1]}}, in_data};


wire signed [ACC_WIDTH:0] avg_sum;                             // 两份数据求和结果，比 ACC_WIDTH 多 1 位以避免溢出
assign avg_sum = {cut[ACC_WIDTH-1], cut} + {cache1[ACC_WIDTH-1], cache1};

always @(posedge clk or negedge rst_n) begin                   // 时钟上升沿或复位下降沿触发
    if (!rst_n) begin                                          // 复位有效时清零寄存器
        cache0 <= 'd0;                                        // 清除 cache0
        cache1 <= 'd0;                                        // 清除 cache1
        sum    <= 'd0;                                        // 清除 sum
    end else begin                                            // 非复位时更新状态
        cache0 <= cache0 - cut + in_ext;                      // IIR 递推公式：当前累加值减去反馈加上输入
        cache1 <= cut;                                        // 保存当前截断值供下一周期平均使用
        sum    <= avg_sum >>> 1;                              // 求平均并算术右移一位
    end
end

generate
    if (ACC_WIDTH == IN_WIDTH) begin : gen_no_trunc             // 当累加器宽度等于输入宽度时直接截断
        assign out_data = sum[IN_WIDTH-1:0];                    // 直接取低位输出
    end
    else begin : gen_sat_trunc                                  // 否则进行饱和截断处理

        localparam integer CHECK_WIDTH = ACC_WIDTH - IN_WIDTH + 1; // 检查高位是否溢出所需宽度

        wire [CHECK_WIDTH-1:0] high_bits;                      // 用于检测溢出的高位
        wire no_overflow;                                      // 溢出标志

        assign high_bits = sum[ACC_WIDTH-1:IN_WIDTH-1];         // 取出高位用于判断符号扩展是否一致

        assign no_overflow = 
            (high_bits == {CHECK_WIDTH{1'b0}}) ||               // 如果高位全部为 0，则无上溢
            (high_bits == {CHECK_WIDTH{1'b1}});                 // 如果高位全部为 1，则无下溢

        assign out_data = no_overflow ? 
                          sum[IN_WIDTH-1:0] :                 // 无溢出时直接输出低位
                          (sum[ACC_WIDTH-1] ? 
                           {1'b1, {(IN_WIDTH-1){1'b0}}} :     // 负溢出时输出最小值
                           {1'b0, {(IN_WIDTH-1){1'b1}}});      // 正溢出时输出最大值

    end
endgenerate


endmodule




