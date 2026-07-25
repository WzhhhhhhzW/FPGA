`timescale 1ns / 1ps  // 定义仿真时间单位为 1 ns、时间精度为 1 ps

module FIR_V1 #(  // 声明可参数化 FIR 滤波器模块
    parameter integer DATA_WIDTH   = 16,               // 输入采样的有符号位宽
    parameter integer COEFF_WIDTH  = 16,               // 滤波器系数的有符号位宽
    parameter integer NUM_TAPS     = 51,               // FIR 滤波器的抽头数量
    parameter integer OUTPUT_WIDTH = 16,               // 输出采样的有符号位宽
    parameter integer OUTPUT_SHIFT = 15,               // 全精度累加值输出前的算术右移位数
    parameter integer SATURATE     = 1,                // 1 表示溢出饱和，0 表示直接截断
    parameter [NUM_TAPS*COEFF_WIDTH-1:0] COEFFICIENTS = { // 按 h[0] 到 h[50] 顺序填写系数
        16'sh0068,                                     // h[0]：第 0 个抽头系数
        16'sh00ea,                                     // h[1]：第 1 个抽头系数
        16'shffb9,                                     // h[2]：第 2 个抽头系数
        16'shfdad,                                     // h[3]：第 3 个抽头系数
        16'shfe8e,                                     // h[4]：第 4 个抽头系数
        16'sh00a0,                                     // h[5]：第 5 个抽头系数
        16'shff6e,                                     // h[6]：第 6 个抽头系数
        16'shfeb6,                                     // h[7]：第 7 个抽头系数
        16'sh0105,                                     // h[8]：第 8 个抽头系数
        16'sh0037,                                     // h[9]：第 9 个抽头系数
        16'shfe3a,                                     // h[10]：第 10 个抽头系数
        16'sh0112,                                     // h[11]：第 11 个抽头系数
        16'sh0130,                                     // h[12]：第 12 个抽头系数
        16'shfd7b,                                     // h[13]：第 13 个抽头系数
        16'sh00a6,                                     // h[14]：第 14 个抽头系数
        16'sh02ae,                                     // h[15]：第 15 个抽头系数
        16'shfcb0,                                     // h[16]：第 16 个抽头系数
        16'shff6c,                                     // h[17]：第 17 个抽头系数
        16'sh0511,                                     // h[18]：第 18 个抽头系数
        16'shfc01,                                     // h[19]：第 19 个抽头系数
        16'shfc71,                                     // h[20]：第 20 个抽头系数
        16'sh09d7,                                     // h[21]：第 21 个抽头系数
        16'shfb8b,                                     // h[22]：第 22 个抽头系数
        16'shf139,                                     // h[23]：第 23 个抽头系数
        16'sh254a,                                     // h[24]：第 24 个抽头系数
        16'sh50b6,                                     // h[25]：第 25 个抽头系数
        16'sh254a,                                     // h[26]：第 26 个抽头系数
        16'shf139,                                     // h[27]：第 27 个抽头系数
        16'shfb8b,                                     // h[28]：第 28 个抽头系数
        16'sh09d7,                                     // h[29]：第 29 个抽头系数
        16'shfc71,                                     // h[30]：第 30 个抽头系数
        16'shfc01,                                     // h[31]：第 31 个抽头系数
        16'sh0511,                                     // h[32]：第 32 个抽头系数
        16'shff6c,                                     // h[33]：第 33 个抽头系数
        16'shfcb0,                                     // h[34]：第 34 个抽头系数
        16'sh02ae,                                     // h[35]：第 35 个抽头系数
        16'sh00a6,                                     // h[36]：第 36 个抽头系数
        16'shfd7b,                                     // h[37]：第 37 个抽头系数
        16'sh0130,                                     // h[38]：第 38 个抽头系数
        16'sh0112,                                     // h[39]：第 39 个抽头系数
        16'shfe3a,                                     // h[40]：第 40 个抽头系数
        16'sh0037,                                     // h[41]：第 41 个抽头系数
        16'sh0105,                                     // h[42]：第 42 个抽头系数
        16'shfeb6,                                     // h[43]：第 43 个抽头系数
        16'shff6e,                                     // h[44]：第 44 个抽头系数
        16'sh00a0,                                     // h[45]：第 45 个抽头系数
        16'shfe8e,                                     // h[46]：第 46 个抽头系数
        16'shfdad,                                     // h[47]：第 47 个抽头系数
        16'shffb9,                                     // h[48]：第 48 个抽头系数
        16'sh00ea,                                     // h[49]：第 49 个抽头系数
        16'sh0068                                      // h[50]：第 50 个抽头系数
    }                                                  // 结束内嵌系数参数定义
) (  // 开始定义模块端口
    input  wire                           clk,         // 系统时钟，上升沿有效
    input  wire                           rst_n,       // 低有效同步复位
    input  wire                           in_valid,    // 输入采样有效指示
    input  wire signed [DATA_WIDTH-1:0]   in_data,     // 有符号输入采样
    output reg                            out_valid,   // 输出采样有效指示
    output reg  signed [OUTPUT_WIDTH-1:0] out_data     // 有符号滤波输出
);  // 结束模块端口定义

    localparam integer SUM_GROWTH =                       // 定义多抽头累加所需的保护位数
        (NUM_TAPS <= 1) ? 0 : $clog2(NUM_TAPS);           // 保护位数取 ceil(log2(NUM_TAPS))
    localparam integer PRODUCT_WIDTH =                    // 定义单个乘积的完整位宽
        DATA_WIDTH + COEFF_WIDTH;                         // 有符号乘积位宽等于两个操作数位宽之和
    localparam integer ACC_WIDTH =                        // 定义全精度累加器位宽
        PRODUCT_WIDTH + SUM_GROWTH;                       // 乘积位宽加保护位，避免多抽头累加溢出
    localparam signed [ACC_WIDTH-1:0] MAX_OUTPUT =         // 定义输出能够表示的最大正数
        {{(ACC_WIDTH-OUTPUT_WIDTH){1'b0}},                 // 最大正数的高位补零
         1'b0, {(OUTPUT_WIDTH-1){1'b1}}};                  // 输出范围内为符号位0、其余位1
    localparam signed [ACC_WIDTH-1:0] MIN_OUTPUT =         // 定义输出能够表示的最小负数
        {{(ACC_WIDTH-OUTPUT_WIDTH){1'b1}},                 // 最小负数的高位执行符号扩展
         1'b1, {(OUTPUT_WIDTH-1){1'b0}}};                  // 输出范围内为符号位1、其余位0

    wire signed [COEFF_WIDTH-1:0]                         // 声明有符号系数连线的数据类型
        coefficient[0:NUM_TAPS-1];                        // 保存从内嵌参数拆分出的全部 FIR 系数
    reg signed [DATA_WIDTH-1:0]                           // 声明有符号采样延迟线的数据类型
        sample_delay[0:NUM_TAPS-1];                       // 保存先前接收的输入采样

    wire signed [PRODUCT_WIDTH-1:0]                       // 声明各抽头完整精度乘积的数据类型
        product[0:NUM_TAPS-1];                            // 保存所有抽头的乘法结果
    wire signed [ACC_WIDTH-1:0]                           // 声明符号扩展后乘积的数据类型
        product_ext[0:NUM_TAPS-1];                        // 保存扩展至累加器位宽的乘积

    reg signed [ACC_WIDTH-1:0] accumulator;               // 保存全部抽头的全精度组合累加结果
    wire signed [ACC_WIDTH-1:0] scaled_value =            // 保存完成算术右移后的缩放结果
        accumulator >>> OUTPUT_SHIFT;                     // 对有符号累加结果执行算术右移
    reg signed [OUTPUT_WIDTH-1:0] converted_output;       // 保存饱和或截断后的待寄存输出

    integer sum_index;                                    // 多抽头组合累加循环变量
    integer delay_index;                                  // 采样延迟线复位和移位循环变量

    assign product[0] =                                   // 计算第 0 个抽头的乘积
        $signed(in_data) * $signed(coefficient[0]);        // 当前输入采样乘以 h[0]
    assign product_ext[0] = product[0];                   // 将第 0 个乘积符号扩展到累加器位宽

    genvar coefficient_index;                             // 声明 generate 系数拆分索引
    genvar tap;                                           // 声明 generate 乘法抽头索引
    generate                                              // 开始生成其余 FIR 抽头的硬件
        for (coefficient_index = 0;                       // 从第 0 个系数开始拆分内嵌参数
             coefficient_index < NUM_TAPS;                // 为全部抽头生成独立系数连线
             coefficient_index = coefficient_index + 1)   // 每次循环拆分一个系数
            begin : gen_coefficients                      // 创建具名系数拆分代码块
            assign coefficient[coefficient_index] =       // 选择当前抽头对应的系数连线
                COEFFICIENTS[                             // 从打包的 COEFFICIENTS 参数中取值
                    (NUM_TAPS-coefficient_index)*          // 根据当前抽头计算系数所在的最高位
                    COEFF_WIDTH-1 -:                       // h[0] 位于参数最高位，随后依次排列
                    COEFF_WIDTH];                         // 每次固定取出一个完整系数
        end                                               // 结束单个系数的拆分代码块
        for (tap = 1;                                     // 第 1 个抽头从第一个历史采样开始
             tap < NUM_TAPS;                              // 为所有剩余抽头生成乘法器
             tap = tap + 1) begin : gen_taps              // 每次循环生成一个具名抽头实例
            assign product[tap] =                         // 计算当前抽头的完整精度乘积
                $signed(sample_delay[tap-1]) *             // 选择与当前抽头对应的历史输入采样
                $signed(coefficient[tap]);                // 历史采样乘以当前抽头系数
            assign product_ext[tap] = product[tap];       // 将当前乘积符号扩展到累加器位宽
        end                                               // 结束单个抽头的生成代码块
    endgenerate                                           // 结束其余抽头硬件生成

    always @* begin                                       // 定义全抽头组合累加逻辑
        accumulator = {ACC_WIDTH{1'b0}};                  // 每次组合计算开始时将累加器清零
        for (sum_index = 0;                               // 从第 0 个抽头开始累加
             sum_index < NUM_TAPS;                        // 遍历全部 FIR 抽头
             sum_index = sum_index + 1)                   // 每次循环选择下一个抽头
            accumulator = accumulator +                   // 将当前抽头乘积加入已有累加值
                product_ext[sum_index];                   // 使用已符号扩展的完整精度乘积
    end                                                   // 结束全抽头组合累加逻辑

    always @* begin                                       // 定义输出缩放、饱和及截断组合逻辑
        if (SATURATE != 0) begin                          // 参数使能时采用饱和输出
            if (scaled_value > MAX_OUTPUT)                // 判断缩放结果是否超过最大正数
                converted_output =                        // 超过上限时输出最大正数
                    {1'b0, {(OUTPUT_WIDTH-1){1'b1}}};     // 构造输出位宽的最大正数
            else if (scaled_value < MIN_OUTPUT)           // 判断缩放结果是否小于最小负数
                converted_output =                        // 低于下限时输出最小负数
                    {1'b1, {(OUTPUT_WIDTH-1){1'b0}}};     // 构造输出位宽的最小负数
            else                                          // 缩放结果处在合法输出范围内
                converted_output =                        // 直接选择缩放结果的有效输出位
                    scaled_value[OUTPUT_WIDTH-1:0];       // 保留低 OUTPUT_WIDTH 位
        end else begin                                    // 参数关闭时采用直接截断输出
            converted_output =                            // 选择缩放结果的低位作为输出
                scaled_value[OUTPUT_WIDTH-1:0];           // 不执行溢出饱和处理
        end                                               // 结束饱和或截断模式选择
    end                                                   // 结束输出格式转换组合逻辑

    always @(posedge clk) begin                           // 在每个时钟上升沿更新滤波器状态和输出
        if (!rst_n) begin                                 // 低有效同步复位被置位时执行复位
            out_valid <= 1'b0;                            // 复位时清除输出有效标志
            out_data <= {OUTPUT_WIDTH{1'b0}};             // 复位时将输出数据清零
            for (delay_index = 0;                         // 从第 0 级延迟寄存器开始复位
                 delay_index < NUM_TAPS;                  // 遍历全部采样延迟寄存器
                 delay_index = delay_index + 1)           // 每次循环复位下一级寄存器
                sample_delay[delay_index] <=              // 选择当前需要复位的历史采样寄存器
                    {DATA_WIDTH{1'b0}};                   // 将当前历史采样寄存器清零
        end else begin                                    // 未处于复位状态时执行正常滤波操作
            out_valid <= in_valid;                        // 将输入有效状态同步传递到输出有效端
            if (in_valid) begin                           // 仅在输入采样有效时更新输出和延迟线
                out_data <= converted_output;             // 寄存当前输入对应的滤波计算结果
                for (delay_index = NUM_TAPS-1;            // 从延迟线末级开始反向移动
                     delay_index > 0;                     // 第 0 级由当前输入单独写入
                     delay_index = delay_index - 1)       // 每次循环向前处理一级延迟寄存器
                    sample_delay[delay_index] <=           // 选择当前要更新的延迟寄存器
                        sample_delay[delay_index-1];       // 写入前一级保存的历史采样
                sample_delay[0] <= in_data;               // 将当前有效输入保存为最新历史采样
            end                                           // 结束有效输入处理
        end                                               // 结束复位与正常工作模式选择
    end                                                   // 结束时序逻辑

endmodule                                                 // 结束 FIR_V1 模块定义
