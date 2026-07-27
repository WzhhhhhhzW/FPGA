`timescale 1ns / 1ps

module FFT
#(
    parameter OUTPUT_BINS = 512
)
(
    input              clk,
    input              rst_n,

    input      [15:0]  data_in,
    input              data_in_valid,

    input              start,
    input      [31:0]  fs,

    output reg         Store_Finish,
    output reg         busy,

    output reg [31:0]  freq_out,
    output reg [31:0]  amp_out,
    output reg [9:0]   index_out,

    output reg         spectrum_valid,
    input              spectrum_ready,
    output reg         spectrum_last,

    output reg         done
);

    // ============================================================
    // 参数
    // ============================================================
    localparam N      = 1024;
    localparam HALF_N = 512;

    localparam S_IDLE       = 4'd0;
    localparam S_RUN        = 4'd1;
    localparam S_STAGE_WAIT = 4'd2;
    localparam S_SPEC_ADDR  = 4'd3;
    localparam S_SPEC_WAIT  = 4'd4;
    localparam S_SPEC_MUL   = 4'd5;
    localparam S_SPEC_OUT   = 4'd6;
    localparam S_DONE       = 4'd7;

    // ============================================================
    // 旋转因子 ROM
    // Q1.15:
    // w_real[k] = cos(2*pi*k/1024) * 32767
    // w_imag[k] = -sin(2*pi*k/1024) * 32767
    // ============================================================
    (* rom_style = "block" *) reg signed [15:0] w_real [0:HALF_N-1];
    (* rom_style = "block" *) reg signed [15:0] w_imag [0:HALF_N-1];

    initial begin
        $readmemh("E:/xilinx_project/vivado_project/colorlight_i9/FFT/w_real_1024.mem", w_real);
        $readmemh("E:/xilinx_project/vivado_project/colorlight_i9/FFT/w_imag_1024.mem", w_imag);
    end

    // ============================================================
    // bit reverse
    // ============================================================
    function [9:0] bit_reverse_10;
        input [9:0] din;
        begin
            bit_reverse_10 = {
                din[0], din[1], din[2], din[3], din[4],
                din[5], din[6], din[7], din[8], din[9]
            };
        end
    endfunction

    // ============================================================
    // 输入计数
    // ============================================================
    reg [9:0] load_cnt;
    wire [9:0] load_addr;

    assign load_addr = bit_reverse_10(load_cnt);

    // ============================================================
    // 主 FSM
    // ============================================================
    reg [3:0] state;

    // ============================================================
    // FFT 控制
    // ============================================================
    reg [3:0] stage;          // 0~9
    reg [9:0] read_cnt;       // 每级 0~511
    reg [9:0] write_cnt;      // 每级写回计数
    reg       issue_done;     // 当前 stage 是否已经发完 512 个蝶形

    // ============================================================
    // 地址生成
    // ============================================================
    reg [9:0] half;
    reg [9:0] j;
    reg [9:0] group;
    reg [9:0] base;

    reg [9:0] addr_a_next;
    reg [9:0] addr_b_next;
    reg [8:0] tw_idx;

    always @(*) begin
        half        = 10'd1 << stage;
        j           = read_cnt & (half - 1'b1);
        group       = read_cnt >> stage;
        base        = group << (stage + 1'b1);

        addr_a_next = base + j;
        addr_b_next = base + j + half;

        tw_idx      = j << (9 - stage);
    end

    // ============================================================
    // 当前 stage 源/目标 Bank
    //
    // stage 偶数：Bank0 -> Bank1
    // stage 奇数：Bank1 -> Bank0
    // ============================================================
    wire src_bank_sel;
    wire dst_bank_sel;

    assign src_bank_sel = stage[0];   // 0: Bank0, 1: Bank1
    assign dst_bank_sel = ~stage[0];  // 0: Bank0, 1: Bank1

    // ============================================================
    // 发起一次蝶形读请求
    // ============================================================
    wire issue_valid;
    assign issue_valid = (state == S_RUN) && (!issue_done);

    // ============================================================
    // 同步 RAM 读延迟对齐
    //
    // RAM 是同步读：
    // 当前周期给地址，posedge 后 dout 更新。
    //
    // issue_valid_d1 对应 RAM dout 有效。
    // src_bank_d1 也必须跟随 issue_valid 延迟一拍。
    // ============================================================
    reg issue_valid_d1;

    reg [9:0] addr_a_meta_d1;
    reg [9:0] addr_b_meta_d1;
    reg [8:0] tw_idx_d1;
    reg       dst_bank_d1;
    reg       src_bank_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            issue_valid_d1 <= 1'b0;
            addr_a_meta_d1 <= 10'd0;
            addr_b_meta_d1 <= 10'd0;
            tw_idx_d1      <= 9'd0;
            dst_bank_d1    <= 1'b0;
            src_bank_d1    <= 1'b0;
        end
        else begin
            issue_valid_d1 <= issue_valid;

            if (issue_valid) begin
                addr_a_meta_d1 <= addr_a_next;
                addr_b_meta_d1 <= addr_b_next;
                tw_idx_d1      <= tw_idx;
                dst_bank_d1    <= dst_bank_sel;
                src_bank_d1    <= src_bank_sel;
            end
        end
    end

    // ============================================================
    // RAM 端口信号
    // ============================================================

    // Bank0 real
    reg                  b0r_we_a;
    reg                  b0r_we_b;
    reg        [9:0]     b0r_addr_a;
    reg        [9:0]     b0r_addr_b;
    reg signed [15:0]    b0r_din_a;
    reg signed [15:0]    b0r_din_b;
    wire signed [15:0]   b0r_dout_a;
    wire signed [15:0]   b0r_dout_b;

    // Bank0 imag
    reg                  b0i_we_a;
    reg                  b0i_we_b;
    reg        [9:0]     b0i_addr_a;
    reg        [9:0]     b0i_addr_b;
    reg signed [15:0]    b0i_din_a;
    reg signed [15:0]    b0i_din_b;
    wire signed [15:0]   b0i_dout_a;
    wire signed [15:0]   b0i_dout_b;

    // Bank1 real
    reg                  b1r_we_a;
    reg                  b1r_we_b;
    reg        [9:0]     b1r_addr_a;
    reg        [9:0]     b1r_addr_b;
    reg signed [15:0]    b1r_din_a;
    reg signed [15:0]    b1r_din_b;
    wire signed [15:0]   b1r_dout_a;
    wire signed [15:0]   b1r_dout_b;

    // Bank1 imag
    reg                  b1i_we_a;
    reg                  b1i_we_b;
    reg        [9:0]     b1i_addr_a;
    reg        [9:0]     b1i_addr_b;
    reg signed [15:0]    b1i_din_a;
    reg signed [15:0]    b1i_din_b;
    wire signed [15:0]   b1i_dout_a;
    wire signed [15:0]   b1i_dout_b;

    // ============================================================
    // RAM 实例
    // ============================================================
    tdp_ram_1024x16 u_bank0_r (
        .clk    (clk),
        .we_a   (b0r_we_a),
        .addr_a (b0r_addr_a),
        .din_a  (b0r_din_a),
        .dout_a (b0r_dout_a),
        .we_b   (b0r_we_b),
        .addr_b (b0r_addr_b),
        .din_b  (b0r_din_b),
        .dout_b (b0r_dout_b)
    );

    tdp_ram_1024x16 u_bank0_i (
        .clk    (clk),
        .we_a   (b0i_we_a),
        .addr_a (b0i_addr_a),
        .din_a  (b0i_din_a),
        .dout_a (b0i_dout_a),
        .we_b   (b0i_we_b),
        .addr_b (b0i_addr_b),
        .din_b  (b0i_din_b),
        .dout_b (b0i_dout_b)
    );

    tdp_ram_1024x16 u_bank1_r (
        .clk    (clk),
        .we_a   (b1r_we_a),
        .addr_a (b1r_addr_a),
        .din_a  (b1r_din_a),
        .dout_a (b1r_dout_a),
        .we_b   (b1r_we_b),
        .addr_b (b1r_addr_b),
        .din_b  (b1r_din_b),
        .dout_b (b1r_dout_b)
    );

    tdp_ram_1024x16 u_bank1_i (
        .clk    (clk),
        .we_a   (b1i_we_a),
        .addr_a (b1i_addr_a),
        .din_a  (b1i_din_a),
        .dout_a (b1i_dout_a),
        .we_b   (b1i_we_b),
        .addr_b (b1i_addr_b),
        .din_b  (b1i_din_b),
        .dout_b (b1i_dout_b)
    );

    // ============================================================
    // 蝶形输入选择
    //
    // 注意：这里使用 src_bank_d1，而不是 src_bank_sel。
    // 因为 RAM 是同步读，dout 比地址晚一拍。
    // ============================================================
    wire signed [15:0] bf_x_r;
    wire signed [15:0] bf_x_i;
    wire signed [15:0] bf_y_r;
    wire signed [15:0] bf_y_i;
    wire signed [15:0] bf_w_r;
    wire signed [15:0] bf_w_i;
    wire               bf_valid_in;

    assign bf_valid_in = issue_valid_d1;

    assign bf_x_r = (src_bank_d1 == 1'b0) ? b0r_dout_a : b1r_dout_a;
    assign bf_x_i = (src_bank_d1 == 1'b0) ? b0i_dout_a : b1i_dout_a;
    assign bf_y_r = (src_bank_d1 == 1'b0) ? b0r_dout_b : b1r_dout_b;
    assign bf_y_i = (src_bank_d1 == 1'b0) ? b0i_dout_b : b1i_dout_b;

    assign bf_w_r = w_real[tw_idx_d1];
    assign bf_w_i = w_imag[tw_idx_d1];

    // ============================================================
    // 蝶形模块
    // ============================================================
    wire               bf_valid_out;
    wire signed [15:0] bf_out_a_r;
    wire signed [15:0] bf_out_a_i;
    wire signed [15:0] bf_out_b_r;
    wire signed [15:0] bf_out_b_i;

    Butterfly_Pipe u_butterfly_pipe (
        .clk        (clk),
        .rst_n      (rst_n),

        .valid_in   (bf_valid_in),

        .x_r        (bf_x_r),
        .x_i        (bf_x_i),
        .y_r        (bf_y_r),
        .y_i        (bf_y_i),
        .w_r        (bf_w_r),
        .w_i        (bf_w_i),

        .valid_out  (bf_valid_out),

        .out_a_r    (bf_out_a_r),
        .out_a_i    (bf_out_a_i),
        .out_b_r    (bf_out_b_r),
        .out_b_i    (bf_out_b_i)
    );

    // ============================================================
    // 写回地址流水线
    //
    // Butterfly_Pipe 中：
    // valid_in -> valid_out 的寄存器延迟是 4 个 clk 边沿后数据可用于写。
    //
    // 因此地址延迟 4 级，使用 d4 写 RAM。
    // ============================================================
    reg [9:0] wr_addr_a_d1;
    reg [9:0] wr_addr_a_d2;
    reg [9:0] wr_addr_a_d3;
    reg [9:0] wr_addr_a_d4;

    reg [9:0] wr_addr_b_d1;
    reg [9:0] wr_addr_b_d2;
    reg [9:0] wr_addr_b_d3;
    reg [9:0] wr_addr_b_d4;

    reg wr_dst_bank_d1;
    reg wr_dst_bank_d2;
    reg wr_dst_bank_d3;
    reg wr_dst_bank_d4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_addr_a_d1   <= 10'd0;
            wr_addr_a_d2   <= 10'd0;
            wr_addr_a_d3   <= 10'd0;
            wr_addr_a_d4   <= 10'd0;

            wr_addr_b_d1   <= 10'd0;
            wr_addr_b_d2   <= 10'd0;
            wr_addr_b_d3   <= 10'd0;
            wr_addr_b_d4   <= 10'd0;

            wr_dst_bank_d1 <= 1'b0;
            wr_dst_bank_d2 <= 1'b0;
            wr_dst_bank_d3 <= 1'b0;
            wr_dst_bank_d4 <= 1'b0;
        end
        else begin
            if (bf_valid_in) begin
                wr_addr_a_d1   <= addr_a_meta_d1;
                wr_addr_b_d1   <= addr_b_meta_d1;
                wr_dst_bank_d1 <= dst_bank_d1;
            end

            wr_addr_a_d2   <= wr_addr_a_d1;
            wr_addr_a_d3   <= wr_addr_a_d2;
            wr_addr_a_d4   <= wr_addr_a_d3;

            wr_addr_b_d2   <= wr_addr_b_d1;
            wr_addr_b_d3   <= wr_addr_b_d2;
            wr_addr_b_d4   <= wr_addr_b_d3;

            wr_dst_bank_d2 <= wr_dst_bank_d1;
            wr_dst_bank_d3 <= wr_dst_bank_d2;
            wr_dst_bank_d4 <= wr_dst_bank_d3;
        end
    end

    // ============================================================
    // 频谱输出相关
    // ============================================================
    reg [9:0] spec_idx;

    reg signed [15:0] spec_r_reg;
    reg signed [15:0] spec_i_reg;

    reg [31:0] spec_r_sq;
    reg [31:0] spec_i_sq;

    wire [33:0] spec_power_sum;
    assign spec_power_sum = {2'b00, spec_r_sq} + {2'b00, spec_i_sq};

    wire [31:0] spec_freq_calc;
    assign spec_freq_calc = (spec_idx * fs) >> 10;

    // ============================================================
    // RAM 端口组合控制
    // ============================================================
    always @(*) begin
        // 默认全部关闭写使能
        b0r_we_a   = 1'b0;
        b0r_we_b   = 1'b0;
        b0i_we_a   = 1'b0;
        b0i_we_b   = 1'b0;

        b1r_we_a   = 1'b0;
        b1r_we_b   = 1'b0;
        b1i_we_a   = 1'b0;
        b1i_we_b   = 1'b0;

        // 默认地址和数据
        b0r_addr_a = 10'd0;
        b0r_addr_b = 10'd0;
        b0i_addr_a = 10'd0;
        b0i_addr_b = 10'd0;

        b1r_addr_a = 10'd0;
        b1r_addr_b = 10'd0;
        b1i_addr_a = 10'd0;
        b1i_addr_b = 10'd0;

        b0r_din_a  = 16'sd0;
        b0r_din_b  = 16'sd0;
        b0i_din_a  = 16'sd0;
        b0i_din_b  = 16'sd0;

        b1r_din_a  = 16'sd0;
        b1r_din_b  = 16'sd0;
        b1i_din_a  = 16'sd0;
        b1i_din_b  = 16'sd0;

        case (state)

            // ----------------------------------------------------
            // 输入阶段：写 Bank0
            // ----------------------------------------------------
            S_IDLE: begin
                b0r_addr_a = load_addr;
                b0i_addr_a = load_addr;

                b0r_din_a  = $signed(data_in);
                b0i_din_a  = 16'sd0;

                if (!Store_Finish && data_in_valid) begin
                    b0r_we_a = 1'b1;
                    b0i_we_a = 1'b1;
                end
            end

            // ----------------------------------------------------
            // FFT 运算阶段
            // ----------------------------------------------------
            S_RUN,
            S_STAGE_WAIT: begin
                if (stage[0] == 1'b0) begin
                    // stage 偶数：Bank0 读，Bank1 写

                    // Bank0 源读
                    b0r_addr_a = addr_a_next;
                    b0i_addr_a = addr_a_next;
                    b0r_addr_b = addr_b_next;
                    b0i_addr_b = addr_b_next;

                    // Bank1 目标写
                    b1r_addr_a = wr_addr_a_d4;
                    b1i_addr_a = wr_addr_a_d4;
                    b1r_addr_b = wr_addr_b_d4;
                    b1i_addr_b = wr_addr_b_d4;

                    b1r_din_a  = bf_out_a_r;
                    b1i_din_a  = bf_out_a_i;
                    b1r_din_b  = bf_out_b_r;
                    b1i_din_b  = bf_out_b_i;

                    if (bf_valid_out && wr_dst_bank_d4 == 1'b1) begin
                        b1r_we_a = 1'b1;
                        b1i_we_a = 1'b1;
                        b1r_we_b = 1'b1;
                        b1i_we_b = 1'b1;
                    end
                end
                else begin
                    // stage 奇数：Bank1 读，Bank0 写

                    // Bank1 源读
                    b1r_addr_a = addr_a_next;
                    b1i_addr_a = addr_a_next;
                    b1r_addr_b = addr_b_next;
                    b1i_addr_b = addr_b_next;

                    // Bank0 目标写
                    b0r_addr_a = wr_addr_a_d4;
                    b0i_addr_a = wr_addr_a_d4;
                    b0r_addr_b = wr_addr_b_d4;
                    b0i_addr_b = wr_addr_b_d4;

                    b0r_din_a  = bf_out_a_r;
                    b0i_din_a  = bf_out_a_i;
                    b0r_din_b  = bf_out_b_r;
                    b0i_din_b  = bf_out_b_i;

                    if (bf_valid_out && wr_dst_bank_d4 == 1'b0) begin
                        b0r_we_a = 1'b1;
                        b0i_we_a = 1'b1;
                        b0r_we_b = 1'b1;
                        b0i_we_b = 1'b1;
                    end
                end
            end

            // ----------------------------------------------------
            // 频谱输出阶段：最终结果在 Bank0
            // 使用 Bank0 A 口读
            // ----------------------------------------------------
            S_SPEC_ADDR,
            S_SPEC_WAIT,
            S_SPEC_MUL,
            S_SPEC_OUT: begin
                b0r_addr_a = spec_idx;
                b0i_addr_a = spec_idx;
            end

            default: begin
            end

        endcase
    end

    // ============================================================
    // busy
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            busy <= 1'b0;
        else
            busy <= (state != S_IDLE);
    end

    // ============================================================
    // 主 FSM
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;

            load_cnt       <= 10'd0;
            Store_Finish   <= 1'b0;

            stage          <= 4'd0;
            read_cnt       <= 10'd0;
            write_cnt      <= 10'd0;
            issue_done     <= 1'b0;

            spec_idx       <= 10'd0;
            spec_r_reg     <= 16'sd0;
            spec_i_reg     <= 16'sd0;
            spec_r_sq      <= 32'd0;
            spec_i_sq      <= 32'd0;

            freq_out       <= 32'd0;
            amp_out        <= 32'd0;
            index_out      <= 10'd0;

            spectrum_valid <= 1'b0;
            spectrum_last  <= 1'b0;

            done           <= 1'b0;
        end
        else begin
            case (state)

                // ------------------------------------------------
                // IDLE：输入数据并等待 start
                // ------------------------------------------------
                S_IDLE: begin
                    done           <= 1'b0;
                    spectrum_valid <= 1'b0;
                    spectrum_last  <= 1'b0;

                    if (!Store_Finish && data_in_valid) begin
                        if (load_cnt == 10'd1023) begin
                            load_cnt     <= 10'd0;
                            Store_Finish <= 1'b1;
                        end
                        else begin
                            load_cnt <= load_cnt + 1'b1;
                        end
                    end

                    if (start && Store_Finish) begin
                        Store_Finish <= 1'b0;

                        stage        <= 4'd0;
                        read_cnt     <= 10'd0;
                        write_cnt    <= 10'd0;
                        issue_done   <= 1'b0;

                        spec_idx     <= 10'd0;

                        freq_out     <= 32'd0;
                        amp_out      <= 32'd0;
                        index_out    <= 10'd0;

                        state        <= S_RUN;
                    end
                end

                // ------------------------------------------------
                // RUN：连续发起 512 个蝶形读请求
                // ------------------------------------------------
                S_RUN: begin
                    if (!issue_done) begin
                        if (read_cnt == 10'd511) begin
                            issue_done <= 1'b1;
                        end
                        else begin
                            read_cnt <= read_cnt + 1'b1;
                        end
                    end

                    if (bf_valid_out) begin
                        if (write_cnt == 10'd511) begin
                            state <= S_STAGE_WAIT;
                        end
                        else begin
                            write_cnt <= write_cnt + 1'b1;
                        end
                    end
                end

                // ------------------------------------------------
                // stage 切换
                // ------------------------------------------------
                S_STAGE_WAIT: begin
                    read_cnt   <= 10'd0;
                    write_cnt  <= 10'd0;
                    issue_done <= 1'b0;

                    if (stage == 4'd9) begin
                        // 10 级 FFT 完成，最终结果在 Bank0
                        spec_idx <= 10'd0;
                        state    <= S_SPEC_ADDR;
                    end
                    else begin
                        stage <= stage + 1'b1;
                        state <= S_RUN;
                    end
                end

                // ------------------------------------------------
                // 频谱读取地址阶段
                // ------------------------------------------------
                S_SPEC_ADDR: begin
                    spectrum_valid <= 1'b0;
                    spectrum_last  <= 1'b0;

                    // RAM 地址由组合逻辑给出
                    state <= S_SPEC_WAIT;
                end

                // ------------------------------------------------
                // 等待同步 RAM 输出
                // ------------------------------------------------
                S_SPEC_WAIT: begin
                    state <= S_SPEC_MUL;
                end

                // ------------------------------------------------
                // 捕获 RAM 输出并计算平方
                // ------------------------------------------------
                S_SPEC_MUL: begin
                    spec_r_reg <= b0r_dout_a;
                    spec_i_reg <= b0i_dout_a;

                    spec_r_sq <= b0r_dout_a * b0r_dout_a;
                    spec_i_sq <= b0i_dout_a * b0i_dout_a;

                    state <= S_SPEC_OUT;
                end

                // ------------------------------------------------
                // 输出频谱点，等待 ready
                // ------------------------------------------------
                S_SPEC_OUT: begin
                    if (!spectrum_valid) begin
                        index_out <= spec_idx;
                        freq_out  <= spec_freq_calc;
                        amp_out   <= spec_power_sum[31:0];

                        spectrum_valid <= 1'b1;

                        if (spec_idx == OUTPUT_BINS - 1)
                            spectrum_last <= 1'b1;
                        else
                            spectrum_last <= 1'b0;
                    end
                    else begin
                        if (spectrum_ready) begin
                            spectrum_valid <= 1'b0;
                            spectrum_last  <= 1'b0;

                            if (spec_idx == OUTPUT_BINS - 1) begin
                                state <= S_DONE;
                            end
                            else begin
                                spec_idx <= spec_idx + 1'b1;
                                state    <= S_SPEC_ADDR;
                            end
                        end
                    end
                end

                // ------------------------------------------------
                // done 脉冲
                // ------------------------------------------------
                S_DONE: begin
                    done           <= 1'b1;
                    spectrum_valid <= 1'b0;
                    spectrum_last  <= 1'b0;

                    // done 保持一个周期，然后回到 IDLE
                    state <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                end

            endcase
        end
    end

endmodule
module tdp_ram_1024x16
(
    input                   clk,

    input                   we_a,
    input      [9:0]        addr_a,
    input signed [15:0]     din_a,
    output reg signed [15:0] dout_a,

    input                   we_b,
    input      [9:0]        addr_b,
    input signed [15:0]     din_b,
    output reg signed [15:0] dout_b
);

    (* ram_style = "block" *) reg signed [15:0] mem [0:1023];

    integer k;

    // 仿真初始化，避免 X 扩散。
    // 综合到 FPGA 时，大多数工具会把 BRAM 初始化为 0。
    initial begin
        for (k = 0; k < 1024; k = k + 1) begin
            mem[k] = 16'sd0;
        end
        dout_a = 16'sd0;
        dout_b = 16'sd0;
    end

    always @(posedge clk) begin
        if (we_a) begin
            mem[addr_a] <= din_a;
        end
        dout_a <= mem[addr_a];
    end

    always @(posedge clk) begin
        if (we_b) begin
            mem[addr_b] <= din_b;
        end
        dout_b <= mem[addr_b];
    end

endmodule
module Butterfly_Pipe
(
    input                       clk,
    input                       rst_n,

    input                       valid_in,

    input signed [15:0]         x_r,
    input signed [15:0]         x_i,
    input signed [15:0]         y_r,
    input signed [15:0]         y_i,
    input signed [15:0]         w_r,
    input signed [15:0]         w_i,

    output reg                  valid_out,

    output reg signed [15:0]    out_a_r,
    output reg signed [15:0]    out_a_i,
    output reg signed [15:0]    out_b_r,
    output reg signed [15:0]    out_b_i
);

    // ============================================================
    // 饱和函数
    // ============================================================
    function signed [15:0] sat16;
        input signed [19:0] din;
        begin
            if (din > 20'sd32767)
                sat16 = 16'sh7FFF;
            else if (din < -20'sd32768)
                sat16 = 16'sh8000;
            else
                sat16 = din[15:0];
        end
    endfunction

    // ============================================================
    // valid pipeline
    // valid_in -> valid_out 对齐数据输出
    // ============================================================
    reg valid_d1;
    reg valid_d2;
    reg valid_d3;

    // ============================================================
    // 第一级：4 个乘法
    // ============================================================
    (* use_dsp = "yes" *) reg signed [31:0] p1;
    (* use_dsp = "yes" *) reg signed [31:0] p2;
    (* use_dsp = "yes" *) reg signed [31:0] p3;
    (* use_dsp = "yes" *) reg signed [31:0] p4;

    reg signed [15:0] x_r_d1;
    reg signed [15:0] x_i_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_d1 <= 1'b0;

            p1       <= 32'sd0;
            p2       <= 32'sd0;
            p3       <= 32'sd0;
            p4       <= 32'sd0;

            x_r_d1   <= 16'sd0;
            x_i_d1   <= 16'sd0;
        end
        else begin
            valid_d1 <= valid_in;

            if (valid_in) begin
                p1 <= y_r * w_r;
                p2 <= y_i * w_i;
                p3 <= y_r * w_i;
                p4 <= y_i * w_r;

                x_r_d1 <= x_r;
                x_i_d1 <= x_i;
            end
        end
    end

    // ============================================================
    // 第二级：复乘加减
    // ============================================================
    reg signed [33:0] mul_r;
    reg signed [33:0] mul_i;

    reg signed [15:0] x_r_d2;
    reg signed [15:0] x_i_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_d2 <= 1'b0;

            mul_r    <= 34'sd0;
            mul_i    <= 34'sd0;

            x_r_d2   <= 16'sd0;
            x_i_d2   <= 16'sd0;
        end
        else begin
            valid_d2 <= valid_d1;

            if (valid_d1) begin
                mul_r <= {{2{p1[31]}}, p1} - {{2{p2[31]}}, p2};
                mul_i <= {{2{p3[31]}}, p3} + {{2{p4[31]}}, p4};

                x_r_d2 <= x_r_d1;
                x_i_d2 <= x_i_d1;
            end
        end
    end

    // ============================================================
    // 第三级：Q1.15 还原 + 蝶形加减
    // ============================================================
    wire signed [18:0] wy_r;
    wire signed [18:0] wy_i;

    assign wy_r = mul_r >>> 15;
    assign wy_i = mul_i >>> 15;

    reg signed [19:0] sum_r;
    reg signed [19:0] sum_i;
    reg signed [19:0] sub_r;
    reg signed [19:0] sub_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_d3 <= 1'b0;

            sum_r    <= 20'sd0;
            sum_i    <= 20'sd0;
            sub_r    <= 20'sd0;
            sub_i    <= 20'sd0;
        end
        else begin
            valid_d3 <= valid_d2;

            if (valid_d2) begin
                sum_r <= $signed({{4{x_r_d2[15]}}, x_r_d2}) + wy_r;
                sum_i <= $signed({{4{x_i_d2[15]}}, x_i_d2}) + wy_i;

                sub_r <= $signed({{4{x_r_d2[15]}}, x_r_d2}) - wy_r;
                sub_i <= $signed({{4{x_i_d2[15]}}, x_i_d2}) - wy_i;
            end
        end
    end

    // ============================================================
    // 第四级：缩放并输出
    // 每一级右移 1 bit，10 级总共除以 1024
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;

            out_a_r   <= 16'sd0;
            out_a_i   <= 16'sd0;
            out_b_r   <= 16'sd0;
            out_b_i   <= 16'sd0;
        end
        else begin
            valid_out <= valid_d3;

            if (valid_d3) begin
                out_a_r <= sat16(sum_r >>> 1);
                out_a_i <= sat16(sum_i >>> 1);

                out_b_r <= sat16(sub_r >>> 1);
                out_b_i <= sat16(sub_i >>> 1);
            end
        end
    end

endmodule
