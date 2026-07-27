`timescale 1ns / 1ps

module AGC(
    input  clk,                     //时钟
    input  signed [11:0]DIN,        //输入信号,使用的是12位的AD
    input  signed [11:0]A_Target,   //目标幅度
    input  rst_n,
    output wire signed [11:0]DOUT   //输出信号
    );
    parameter  [11:0]Length = 2047; //存储长度2048个数据
    parameter  [4:0] N      = 11  ; //
    parameter  [4:0] I      =  1;   //控制积分项
    wire signed [13:0] K_0;
    reg signed [13:0] K;            //AGC反馈得出的增益,设定最高位为符号位，5位为整数位，后8位为小数位
    reg signed [11:0] A_Last_Output;//上一次输出的幅度，同时也作为反馈幅度，应当由输出数组推出
    wire signed [12:0] Difference;  //与目标值的差
    reg [24:0] A_Sum;               //用来对输入值求和最终得到幅度
    reg [10:0] Count;               //计数值，存满2048个点之后开始运算
    reg [2:0]  MODE;                //0表示第一轮2048个点还没存完，1表示已经存完初始数据，可以开始计算
    wire signed [25:0] Inner_DOUT;  //存储乘以K之后的输出值
    wire [11:0] A_abs;              //幅度的绝对值

    assign Inner_DOUT = (K* DIN)>>>8;
    assign DOUT =   (Inner_DOUT > 26'sd2047)  ?  12'sd2047 :
                    (Inner_DOUT < -26'sd2047) ? -12'sd2047 :
                    Inner_DOUT[11:0];//输出限幅
    
    assign A_abs = DOUT[11]?(-DOUT):DOUT;
    assign Difference = A_Target - A_Last_Output;
    assign K_0 = K + (Difference >>> I);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin           //复位
            K <= 256;               //由于前四位表示整数，那么1就是对应256
            A_Last_Output <= 0;
            A_Sum <= 0;
            Count <= 0;
            MODE <= 0;
        end else begin
            case(MODE)
                0:begin             //累计阶段
                    Count <= Count + 1;
                    //计算幅度总和
                    A_Sum <= A_Sum + A_abs;
                    if(Count == Length)begin
                            A_Last_Output <= (A_Sum + A_abs) >> N;//计算输出幅度
                            MODE <= 1;
                    end
                end
                1:begin//积分项转化为KI
                    if(K_0 > 2560)begin
                        K <= 2560;
                    end
                    else if(K_0 < 0)begin
                        K <= 0;
                    end
                    else begin
                        K <= K_0;
                    end
                    //复位
                    Count <= 0;
                    A_Sum <= 0;
                    MODE <= 0;
                end
                default:
                    MODE <= 0;
            endcase
    end
end
endmodule
