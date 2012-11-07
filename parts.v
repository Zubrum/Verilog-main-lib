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

module zrb_async_fifo
/*
zrb_sync_fifo #(8,32) instance_name (
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
    #(parameter ADDR_WIDTH = 4, DATA_WIDTH = 32)
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

reg     [ (ADDR_WIDTH) :  0 ] rd_ptr = {(ADDR_WIDTH+1){1'b0}};  //[4:0] = 5'b0
wire    [ (ADDR_WIDTH) :  0 ] wrd_ptr;                          //[4:0]
wire    [ (ADDR_WIDTH) :  0 ] rd_next;

reg     [ (ADDR_WIDTH) :  0 ] wr_ptr = {(ADDR_WIDTH+1){1'b0}};  //[4:0] = 5'b0
wire    [ (ADDR_WIDTH) :  0 ] wwr_ptr;
wire    [ (ADDR_WIDTH) :  0 ] wr_next;

wire    [ (ADDR_WIDTH-1) :  0 ] rd_loc = rd_ptr[ (ADDR_WIDTH-1) :  0 ];//[3:0] = [3:0]
wire    [ (ADDR_WIDTH-1) :  0 ] wr_loc = wr_ptr[ (ADDR_WIDTH-1) :  0 ];//[3:0] = [3:0]

reg     [ (DATA_WIDTH-1) :  0 ] mem [ (DEPTH-1) :  0 ];//[31:0] mem[15:0]
reg     [ (DATA_WIDTH-1) :  0 ] dout = {DATA_WIDTH{1'b0}};

reg     full = 1'b0;
reg     empty = 1'b1;
always@(rd_ptr or wr_ptr)
begin
    full <= (wr_loc == rd_loc) & (wr_ptr[ADDR_WIDTH] != rd_ptr[ADDR_WIDTH]);
    empty <= (rd_loc == wr_loc) & (wr_ptr[ADDR_WIDTH] == rd_ptr[ADDR_WIDTH]);
end

assign  fifo_full = full;
assign  fifo_empty = empty;
/*
zrb_gray2bin #(ADDR_WIDTH+1) u0 (rd_ptr, wrd_ptr);
zrb_bin2gray #(ADDR_WIDTH+1) u1 (wrd_ptr + 1'b1, rd_next);

zrb_gray2bin #(ADDR_WIDTH+1) u2 (wr_ptr, wwr_ptr);
zrb_bin2gray #(ADDR_WIDTH+1) u3 (wwr_ptr + 1'b1, wr_next);
*/

always@(posedge wr_clk or posedge reset)
if(reset)
    wr_ptr <= {(ADDR_WIDTH+1){1'b0}};
else
if(wr_en & !fifo_full)
begin
    mem[wr_loc] <= data_in;
    //wr_ptr <= wr_next;
    wr_ptr <= wr_ptr + 1'b1;
end

always@(posedge rd_clk or posedge reset)
if(reset)
begin
    rd_ptr <= {(ADDR_WIDTH+1){1'b0}};
    dout <= {DATA_WIDTH{1'b0}};
end
else
if(rd_en & !fifo_empty)
begin
    dout <= mem[rd_loc];
    //rd_ptr <= rd_next;
    rd_ptr <= rd_ptr + 1'b1;
end

//assign data_out = rd_en ? mem[rd_ptr] : data_out;
assign data_out = dout;
endmodule

module zrb_baud_generator
/*
zrb_baud_generator #(50000000,9600) instance_name (input_clk, baud_clk, baud_clk_8);
*/
    #(parameter INPUT_CLK = 50000000, parameter BAUD = 9600)
    (
    input   wire                clk,
    output  wire                baud_clk_tx_en,
    output  wire                baud_clk_rx_en
    );
/*http://www.excamera.com/sphinx/fpga-uart.html#uart*/
localparam BAUD_TX = BAUD;
localparam BAUD_RX = 8*BAUD;
reg		[ 28 :  0 ] r_tx = 29'b0;
reg		[ 28 :  0 ] r_rx = 29'b0;
wire	[ 28 :  0 ] inc_tx = r_tx[28] ? (BAUD_TX) : (BAUD_TX-INPUT_CLK);
wire	[ 28 :  0 ] inc_rx = r_rx[28] ? (BAUD_RX) : (BAUD_RX-INPUT_CLK);
wire	[ 28 :  0 ] tx_tic = r_tx + inc_tx;
wire	[ 28 :  0 ] rx_tic = r_rx + inc_rx;
always@(posedge clk)
begin
	r_tx <= tx_tic;
    r_rx <= rx_tic;
end
assign baud_clk_tx_en = ~r_tx[28];
assign baud_clk_rx_en = ~r_rx[28];
endmodule


module zrb_uart_tx
/*
zrb_uart_tx #(8,0,1) instance_name(
    CLK,
    CLK_EN, //BAUD RATE
    RESET,
    WRITE,
    INPUT_DATA[7:0],
    OUTPUT_TX,
    OUTPUT_BUSY
    );
*/
    #(parameter [3:0] NUM_BITS = 8, parameter [3:0]PARITY = 0, parameter [3:0]STOP_BIT = 1)
    (
    input   wire                clk,
    input   wire                clk_en,
    input   wire                reset,
    input   wire                write,
    input   wire    [  7 :  0 ] data,

    output  wire                tx,
    output  wire                busy
    );
localparam [3:0]START_BIT = 1;
reg     [  8 :  0 ] r_data = 9'b0;
reg     [  3 :  0 ] r_cnt = 4'b0;
reg                 r_tx = 1'b1;

wire		        sending = |r_cnt;
assign              busy = |r_cnt[3:1];
assign              tx = r_tx;

always@(posedge clk)
if(reset)
begin
    r_tx <= 1'b1;
    r_cnt <= 4'b0;
    r_data <= 9'b0;
end
else
if(clk_en)
begin
    if(write & ~busy)
    begin
        r_data <= {data, 1'b0};
        r_cnt <= START_BIT+NUM_BITS+STOP_BIT;
    end

    if(sending)
    begin
        {r_data, r_tx} <= {1'b1, r_data};
        r_cnt <= r_cnt - 1'b1;
    end
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
