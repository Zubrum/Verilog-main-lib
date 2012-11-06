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

reg     [ (DATA_WIDTH-1) :  0 ] mem [  0 : (DEPTH-1) ];
reg     [ (DATA_WIDTH-1) :  0 ] r_data_out = {DATA_WIDTH{1'b0}};
assign data_out = r_data_out;
assign fifo_empty = r_wr_ptr == r_rd_ptr;
assign fifo_full =  w_wr_next == r_rd_ptr;//w_rd_next == w_wr_bin;

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
/*http://www.excamera.com/sphinx/fpga-uart.html#uart*/
localparam ACC_WIDTH = 21;
localparam BAUD_TX = 4*BAUD;
localparam BAUD_RX = 4*8*BAUD;
reg		[ 27 :  0 ] r_tx = 28'b0;
reg		[ 27 :  0 ] r_rx = 28'b0;
wire	[ 27 :  0 ] inc_tx = r_tx[27] ? (BAUD_TX) : (BAUD_TX-INPUT_CLK);
wire	[ 27 :  0 ] inc_rx = r_rx[27] ? (BAUD_RX) : (BAUD_RX-INPUT_CLK);
wire	[ 27 :  0 ] tx_tic = r_tx + inc_tx;
wire	[ 27 :  0 ] rx_tic = r_rx + inc_rx;
reg					tx_clk = 1'b0;
reg					rx_clk = 1'b0;
always@(posedge clk)
begin
	r_tx <= tx_tic;
	if(~r_tx[27])
		tx_clk <= tx_clk + 1'b1;
    r_rx <= rx_tic;
	if(~r_rx[27])
		rx_clk <= rx_clk + 1'b1;
end
assign baud_clk_tx = tx_clk;
assign baud_clk_rx = rx_clk;
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
reg     [  2 :  0 ] r_cnt = 3'b0;
reg                 r_tx = 1'b1;

localparam			IDLE = 2'b00,
					START_BIT = 2'b01,
					DATA_SEND = 2'b11,
					STOP_BIT = 2'b10;
reg		[  1 :  0 ] r_state = IDLE;
assign tx = r_tx;
assign ready = (r_state == IDLE) | (r_state == STOP_BIT);

always@(posedge clk)
begin
	if(start)
		r_data <= data;
	case(r_state)
		IDLE:
			if(start)
				r_state <= START_BIT;
		START_BIT:
			r_state <= DATA_SEND;
		DATA_SEND:
			if(r_cnt == 3'd7)
				r_state <= STOP_BIT;
		STOP_BIT:
			r_state <= IDLE;
	endcase
	
	case(r_state)
		IDLE:
			r_tx <= 1'b1;
		START_BIT:
			r_tx <= 1'b0;
		DATA_SEND:
			begin
				r_cnt <= r_cnt + 1'b1;
				r_tx <= r_data[r_cnt];
			end
		STOP_BIT:
			begin
				r_cnt <= 3'b0;
				r_tx <= 1'b1;
			end
	endcase
end
endmodule

/*
http://ww1.microchip.com/downloads/en/appnotes/00774a.pdf
http://we.easyelectronics.ru/plis/ocherednoe-izobretenie-velosipedov-ili-uart_tx-i-uart_rx-na-yazyke-verilog.html
http://www.fpga4fun.com/SerialInterface4.html
*/


module zrb_uart_rx
/*
zrb_uart_rx instance_name(
    INPUT_CLK, //HIGH FREQ
    INPUT_RX,
    OUTPUT_DATA[7:0],
    OUTPUT_READY
    );
*/
    (
    input   wire                clk,
    input   wire                rx,

    output  wire    [  7 :  0 ] data_out,
    output  wire                ready
    );
reg     [  7 :  0 ] r_data = 8'b0;
reg     [  1 :  0 ] rx_sync = 2'b0;
reg     [  2 :  0 ] cnt = 3'b0;
reg     [  2 :  0 ] cnt_bit = 3'b0;
wire                start = (~rx_sync[0] & rx_sync[1]) & (r_state == IDLE);
localparam  [  1 :  0 ] IDLE  = 2'b01,
						SYNC = 2'b00,
                        READ  = 2'b10,
                        READY = 2'b11;
reg         [  1 :  0 ] r_state = IDLE;

assign ready = r_state == READY;
assign data_out = r_data;
always@(posedge clk)
begin
    rx_sync <= {rx_sync[0], rx};
    case(r_state)
        IDLE:
            if(start)
                r_state <= SYNC;
		SYNC:
			if(cnt == 3'd2)
				r_state <= READ;

        READ:
            if((cnt_bit == 3'd7) & (cnt == 3'd2))
                r_state <= READY;
        READY:
			if(cnt == 3'd2)
				r_state <= IDLE;
        default:
            r_state <= IDLE;
    endcase
    
    case(r_state)
        IDLE:
        begin
            cnt <= 3'b0;
            r_data <= 8'b0;
            cnt_bit <= 3'b0;
        end
        
		SYNC:
		begin
			cnt <= cnt + 1'b1;
		end
		
        READ:
        begin
            cnt <= cnt + 1'b1;
            if(cnt == 3'd2)
            begin
                r_data <= {r_data[6:0], rx};
                cnt_bit <= cnt_bit + 1'b1;
            end
        end
        READY:
            cnt <= cnt + 1'b1;
        default: begin end
    endcase    
end
endmodule

/*
module zrb_sram_controller
	#()
	(
	input 	wire clk,
	output	wire			[ 17 :  0 ] sram_adr,
	inout	wire			[ 15 :  0 ] sram_dat,
	output	wire						sram_we,		//active low
	output	wire						sram_lb,		//active low
	output	wire						sram_ub,		//active low
	output	wire						sram_oe,		//active low
	output	wire						sram_ce			//active low
	);

localparam	[  4 :  0 ] NOT_SELECTED = 					5'b11111,
						OUTPUT_DISABLE = 				5'b10111,
						READ_LOWER =					5'b10001,
						READ_UPPER =					5'b10010,
						READ =							5'b10000,
						WRITE_LOWER =					5'b00101,
						WRITE_UPPER =					5'b00110,
						WRITE =							5'b00100;
reg			[  4 :  0 ] r_state = NOT_SELECTED; //{we,ce,oe,lb,ub,}
reg			[  4 :  0 ] r_state_nxt = NOT_SELECTED; 
	
assign sram_we = r_state[4];
assign sram_ce = r_state[3];
assign sram_oe = r_state[2];
assign sram_lb = r_state[1];	
assign sram_ub = r_state[0];	

reg			[ 17 :  0 ] r_sram_adr = 18'b0;
reg			[ 15 :  0 ] r_write_dat = 16'b0;
reg			[ 15 :  0 ] r_read_dat = 16'b0;

assign sram_adr = r_sram_adr;
assign sram_dat = (r_state == READ) ? 16'bz : r_write_dat;

always@(posedge clk)
begin
    r_state <= r_state_nxt;
    case(r_state_nxt)
		NOT_SELECTED:
		begin
			if(r_state == NOT_SELECTED)r_state_nxt <= WRITE;
			if(r_state == WRITE)r_state_nxt <= READ;
			if(r_state == READ) r_state_nxt <= OUTPUT_DISABLE;
		end
		WRITE:
			if(i == 3'b111)	
				r_state_nxt <= READ;
		READ:
			r_state_nxt <= OUTPUT_DISABLE;
        default:
            r_state_nxt <= OUTPUT_DISABLE;
    endcase
end

always@(negedge clk)
case(r_state)
	WRITE:
		r_write_dat <= 16'b011;
	OUTPUT_DISABLE:
        r_write_dat <= 16'b0;

endcase


always@(posedge clk)
begin

	case(r_state)
		READ:
			r_read_dat <= sram_dat;
	endcase
	
    case(r_state_nxt)        
        WRITE:
        begin
            i <= i + 1'b1;
            r_sram_adr[12-:3] <= i;
        end

        READ:
            r_sram_adr <= 18'd7168;

        OUTPUT_DISABLE:
			r_sram_adr[12:10] <= r_read_dat[2:0];

		default: begin end

    endcase	
end

endmodule
*/
