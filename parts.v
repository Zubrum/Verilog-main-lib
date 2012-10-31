/*
        Verilog functions&tasks for everyday
*/

module zrb_bin2gray
/*
zrb_bin2gray #(8) instance_name (binary_input, gray_output);
*/

    #(parameter LENGTH = 8)
    (
    input   wire    [ (LENGTH-1) :  0 ] binary_input,
    output  wire    [ (LENGTH-1) :  0 ] gray_output
    );
genvar k;
generate
    assign gray_output[LENGTH-1] = binary_input[LENGTH-1];
    for(k = LENGTH-2; k >= 0; k = k - 1)
    begin: gray
        assign gray_output[k] = binary_input[k+1] ^ binary_input[k];
    end
endgenerate
endmodule

module zrb_gray2bin
/*
zrb_gray2bin #(8) instance_name (gray_input, binary_output);
*/

    #(parameter LENGTH = 8)
    (
    input   wire    [ (LENGTH-1) :  0 ] gray_input,
    output  wire    [ (LENGTH-1) :  0 ] binary_output
    );
genvar k;
generate
    assign binary_output[LENGTH-1] = gray_input[LENGTH-1];
    for(k = LENGTH-2; k >= 0; k = k - 1)
    begin: bin
        assign binary_output[k] = ^(gray_input >> k);
    end
endgenerate
endmodule


module zrb_fifo
/*
zrb_fifo #(3,8) instance_name (
    RESET,
    WR_CLK,
    WR_EN,
    WR_DATA[7:0],
    RD_CLK,
    RD_EN,
    RD_DATA[7:0],
    FIFO_FULL,
    FIFO_EMPTY
    );
*/
    #(parameter ADDR_WIDTH = 3, DATA_WIDTH = 8)
    (
    input   wire                            reset,

    input   wire                            wr_clk,
    input   wire                            wr_en,
    input   wire    [ (DATA_WIDTH-1) :  0 ] data_in,

    input   wire                            rd_clk,
    input   wire                            rd_en,
    output  wire    [ (DATA_WIDTH-1) :  0 ] data_out,

    output  wire                            fifo_full,
    output  wire                            fifo_empty
    );
localparam DEPTH = 1 << ADDR_WIDTH;
reg     [ (ADDR_WIDTH-1) :  0 ] r_wr_ptr = {ADDR_WIDTH{1'b0}};
wire    [ (ADDR_WIDTH-1) :  0 ] w_wr_bin;
wire    [ (ADDR_WIDTH-1) :  0 ] w_wr_next;
reg     [ (ADDR_WIDTH-1) :  0 ] r_rd_ptr = {ADDR_WIDTH{1'b0}};
wire    [ (ADDR_WIDTH-1) :  0 ] w_rd_bin;
wire    [ (ADDR_WIDTH-1) :  0 ] w_rd_next;

reg     [ (ADDR_WIDTH-1) :  0 ] sync_0 = {ADDR_WIDTH{1'b0}};
reg     [ (ADDR_WIDTH-1) :  0 ] sync_1 = {ADDR_WIDTH{1'b0}};
wire    [ (ADDR_WIDTH-1) :  0 ] w_wr_next_rd_clk = sync_1;

/*
reg     [ (ADDR_WIDTH-1) :  0 ] sync_00 = {ADDR_WIDTH{1'b0}};
reg     [ (ADDR_WIDTH-1) :  0 ] sync_01 = {ADDR_WIDTH{1'b0}};
wire    [ (ADDR_WIDTH-1) :  0 ] w_rd_next_wr_clk = sync_01;
*/

reg     [ (DATA_WIDTH-1) :  0 ] mem [  0 : (DEPTH-1) ];
reg     [ (DATA_WIDTH-1) :  0 ] r_data_out = {DATA_WIDTH{1'b0}};
assign data_out = r_data_out;
assign fifo_empty = r_wr_ptr == r_rd_ptr;
assign fifo_full =  w_wr_next_rd_clk == r_rd_ptr;//w_rd_next == w_wr_bin;

zrb_gray2bin #(ADDR_WIDTH) u0 (r_wr_ptr, w_wr_bin);
zrb_bin2gray #(ADDR_WIDTH) u1 (w_wr_bin + 1'b1, w_wr_next);

zrb_gray2bin #(ADDR_WIDTH) u2 (r_rd_ptr, w_rd_bin);
zrb_bin2gray #(ADDR_WIDTH) u3 (w_rd_bin + 1'b1, w_rd_next);


always@(posedge wr_clk or posedge reset)
if(reset)
    r_wr_ptr <= {ADDR_WIDTH{1'b0}};
else
if(wr_en && !fifo_full)
begin
    mem[r_wr_ptr] <= data_in;
    r_wr_ptr <= w_wr_next;
end


always@(posedge rd_clk or posedge reset)
if(reset)
begin
    r_rd_ptr <= {ADDR_WIDTH{1'b0}};
    r_data_out <= {DATA_WIDTH{1'b0}};
end
else
if(rd_en && !fifo_empty)
begin
    r_data_out <= mem[r_rd_ptr];
    r_rd_ptr <= w_rd_next;
end

always@(posedge rd_clk)
begin
    sync_0 <= w_wr_next;
    sync_1 <= sync_0;
end
endmodule
/*
always@(posedge wr_clk)
begin
    sync_00 <= w_rd_next;
    sync_01 <= sync_00;
end
endmodule
*/

module zrb_baud_generator
/*
zrb_baud_generator #(50000000,9600) instance_name (input_clk, baud_clk, baud_clk_8);
*/
    #(parameter INPUT_CLK = 50000000, parameter BAUD = 9600)
    (
    input   wire                clk,
    output  wire                baud_clk_tx,
    output  wire                baud_clk_rx
    );
localparam ACC_WIDTH = 16;
//localparam ACC_MAX_TX = (INPUT_CLK/BAUD/2);
//localparam ACC_MAX_RX = ACC_MAX_TX / 8;
localparam ACC_INC_TX = ((BAUD << (ACC_WIDTH-4)) + (INPUT_CLK >> 5))/(INPUT_CLK >> 4);
localparam ACC_INC_RX = ACC_INC_TX * 8;

reg [ ACC_WIDTH :  0 ] r_acc_tx = {(ACC_WIDTH+1){1'b0}};
reg [ ACC_WIDTH :  0 ] r_acc_rx = {(ACC_WIDTH+1){1'b0}};
always@(posedge clk)
begin
    r_acc_tx <= r_acc_rx[ (ACC_WIDTH-1) :  0 ] + ACC_INC_TX;
    r_acc_rx <= r_acc_rx[ (ACC_WIDTH-1) :  0 ] + ACC_INC_RX;
end
assign baud_clk_tx = r_acc_tx[ACC_WIDTH];
assign baud_clk_rx = r_acc_rx[ACC_WIDTH];
endmodule

module zrb_uart_tx
/*
zrb_uart_tx instance_name(
    INPUT_CLK, //BAUD RATE
    INPUT_START,
    INPUT_DATA[7:0],
    OUTPUT_TX,
    OUTPUT_READY
);
*/
    (
    input   wire                clk,
    input   wire                start,
    input   wire    [  7 :  0 ] data,

    output  wire                tx,
    output  wire                ready
    );

reg     [  7 :  0 ] r_data = 8'b0;
reg     [  3 :  0 ] r_cnt = 4'b0;
reg                 r_tx = 1'b1;
assign tx = r_tx;
assign ready = r_cnt == 4'd0;

always@(posedge clk)
begin
    if(start && ready)
    begin
        r_data <= data;
        r_cnt <= 4'd1;
    end
    
    if(r_cnt == 4'd10)
        r_cnt <= 4'd0;
    else
        if(~ready)
            r_cnt <= r_cnt + 1'b1;

    case(r_cnt)
        4'd0 : r_tx <= 1'b1;
        4'd1 : r_tx <= 1'b0;
        4'd2 : r_tx <= r_data[0];
        4'd3 : r_tx <= r_data[1];
        4'd4 : r_tx <= r_data[2];
        4'd5 : r_tx <= r_data[3];
        4'd6 : r_tx <= r_data[4];
        4'd7 : r_tx <= r_data[5];
        4'd8 : r_tx <= r_data[6];
        4'd9 : r_tx <= r_data[7];
        4'd10: r_tx <= 1'b1;
        default : r_tx <= 1'b1;
    endcase
end
endmodule

/*
http://ww1.microchip.com/downloads/en/appnotes/00774a.pdf
http://we.easyelectronics.ru/plis/ocherednoe-izobretenie-velosipedov-ili-uart_tx-i-uart_rx-na-yazyke-verilog.html
http://www.fpga4fun.com/SerialInterface4.html
*/


//module zrb_uart_rx
/*
zrb_uart_rx instance_name(
    INPUT_CLK, //HIGH FREQ
    INPUT_RX,
    OUTPUT_DATA[7:0],
    OUTPUT_READY
    );
*/
/*
    (
    input   wire                clk,
    input   wire                rx,

    output  wire    [  7 :  0 ] data_out,
    output  wire                ready
    );

reg     [  7 :  0 ] r_data = 8'b0;
reg     [  1 :  0 ] rx_sync = 2'b0;
always@(posedge clk)
begin
    rx_sync <= {rx_sync[0], rx};
end

*/