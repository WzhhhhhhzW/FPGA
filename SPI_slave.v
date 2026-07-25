module SPI_slave(
input clk,
input SCK,
input mosi,
output wire miso,
input CS,
output reg update,
input [7:0]slave_SendData,
output reg [7:0]slave_ReceiveData
);

reg [1:0]SCK_r,CS_r,mosi_r; //打两拍取上升下降沿
reg [7:0]rcache,scache;     //发送和接收的移位寄存器

always@(posedge clk)begin
    SCK_r <= {SCK_r[0],SCK};
    CS_r  <= {CS_r[0],CS};
    mosi_r <= {mosi_r[0],mosi};
end
     
wire SCK_dn = SCK_r == 2'b10;    //使用wire连接组合逻辑信号量，产生触发信号
wire CS_up  = CS_r  == 2'b01;
wire CS_dn  = CS_r  == 2'b10;
wire RS_en  = CS_r  == 2'b00;

reg [2:0]bit_cnt,byte_cnt;
reg update_en;

always@(posedge clk)begin
    if(CS_dn)begin                        //Start,IDLE->RS
        rcache <= 0;
        bit_cnt <= 0;
        byte_cnt <= 0;
        update <= 0;
        update_en <= 0;
    end else if(CS_up)begin               //Stop,RS->IDLE
        slave_ReceiveData <= rcache;      //Latch ReceiveData
    end else if(RS_en)begin               //RS state
        if(SCK_dn)begin
            rcache <= {rcache[6:0],mosi_r[1]};
            bit_cnt <= bit_cnt + 1;
            if(bit_cnt == 7)begin
                byte_cnt <= byte_cnt + 1;
                update_en <= 1; 
            end
        end else begin
            if(update)
                update <= 0;
            else if(bit_cnt == 0 && byte_cnt > 0 && update_en)begin
                update <= 1;
                update_en <= 0;
                slave_ReceiveData <= rcache;      
            end 
        end
    end
end

always@(posedge SCK_r[0] or posedge CS_dn)begin
    if(CS_dn)begin
        scache <= 0;
    end else begin
        if(bit_cnt == 0)begin
            scache <= slave_SendData;    //Load SendData
        end else
            scache <= {scache[6:0],1'b0};
    end
end

assign miso = scache[7];

endmodule
