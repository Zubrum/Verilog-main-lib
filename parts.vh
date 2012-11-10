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

module zrb_sync_fifo
/*
zrb_sync_fifo #(3,8) instance_name (
    RESET,
    WR_CLK,
    WR_EN,
    WR_DATA[7:0],
    RD_EN,
    RD_DATA[7:0],
    FIFO_FULL,
    FIFO_EMPTY
    );
*/
    #(parameter ADDR_WIDTH = 3, DATA_WIDTH = 8)
    (
    input   wire                            reset,

    input   wire                            clk,
    input   wire                            wr_en,
    input   wire    [ (DATA_WIDTH-1) :  0 ] data_in,

    input   wire                            rd_en,
    output  wire    [ (DATA_WIDTH-1) :  0 ] data_out,

    output  wire                            fifo_full,
    output  wire                            fifo_empty
    );
localparam DEPTH = 1 << ADDR_WIDTH;
reg     [ (ADDR_WIDTH) :  0 ] wr_ptr = {(ADDR_WIDTH+1){1'b0}};
reg     [ (ADDR_WIDTH) :  0 ] rd_ptr = {(ADDR_WIDTH+1){1'b0}};
wire	[ (ADDR_WIDTH-1) :  0 ] wr_loc = wr_ptr[ (ADDR_WIDTH-1) :  0 ];
wire	[ (ADDR_WIDTH-1) :  0 ] rd_loc = rd_ptr[ (ADDR_WIDTH-1) :  0 ];

reg     [ (DATA_WIDTH-1) :  0 ] mem [ (DEPTH-1) :  0 ];
//reg     [ (DATA_WIDTH-1) :  0 ] r_data_out = {DATA_WIDTH{1'b0}};

reg	full = 1'b0;
reg	empty = 1'b0;
assign data_out = mem[rd_loc];
assign fifo_empty = empty;
assign fifo_full = full;

always@(wr_ptr or rd_ptr)
begin
	full <= 1'b0;
	empty <= 1'b0;
	if(wr_ptr[ (ADDR_WIDTH-1) :  0 ] == rd_ptr[ (ADDR_WIDTH-1) :  0 ])
		if(rd_ptr[ADDR_WIDTH] == wr_ptr[ADDR_WIDTH])
			empty <= 1'b1;
		else
			full <= 1'b1;
end

always@(posedge clk or posedge reset)
if(reset)
begin
    wr_ptr <= {(ADDR_WIDTH+1){1'b0}};
    rd_ptr <= {(ADDR_WIDTH+1){1'b0}};
end
else
begin
	if(wr_en & !fifo_full)
	begin
		mem[wr_loc] <= data_in;
		wr_ptr <= wr_ptr + 1'b1;
	end
	
	if(rd_en & !fifo_empty)
		rd_ptr <= rd_ptr + 1'b1;
end
endmodule


module zrb_async_fifo_gray
/*
zrb_async_fifo_gray #(3,8) instance_name (
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
reg     [ (ADDR_WIDTH) :  0 ] wr_ptr_bin = {(ADDR_WIDTH+1){1'b0}};
reg     [ (ADDR_WIDTH) :  0 ] rd_ptr_bin = {(ADDR_WIDTH+1){1'b0}};
wire	[ (ADDR_WIDTH-1) :  0 ] wr_loc_gray;
wire	[ (ADDR_WIDTH-1) :  0 ] rd_loc_gray;
zrb_bin2gray #(ADDR_WIDTH) u1 (wr_ptr_bin[ (ADDR_WIDTH-1) :  0 ], wr_loc_gray);
zrb_bin2gray #(ADDR_WIDTH) u2 (rd_ptr_bin[ (ADDR_WIDTH-1) :  0 ], rd_loc_gray);

reg     [ (DATA_WIDTH-1) :  0 ] mem [ (DEPTH-1) :  0 ];
assign data_out = mem[rd_loc_gray];
reg full = 1'b0;
reg empty = 1'b0;
assign fifo_empty = empty;
assign fifo_full =  full;

always@(wr_ptr_bin or rd_ptr_bin)
begin
	full <= 1'b0;
	empty <= 1'b0;
	if(wr_ptr_bin[ (ADDR_WIDTH-1) :  0 ] == rd_ptr_bin[ (ADDR_WIDTH-1) :  0 ])
		if(rd_ptr_bin[ADDR_WIDTH] == wr_ptr_bin[ADDR_WIDTH])
			empty <= 1'b1;
		else
			full <= 1'b1;
end

always@(posedge wr_clk or posedge reset)
if(reset)
    wr_ptr_bin <= {(ADDR_WIDTH+1){1'b0}};
else
if(wr_en && !fifo_full)
begin
    mem[wr_loc_gray] <= data_in;
    wr_ptr_bin <= wr_ptr_bin + 1'b1;
end

always@(posedge rd_clk or posedge reset)
if(reset)
    rd_ptr_bin <= {ADDR_WIDTH{1'b0}};
else
if(rd_en && !fifo_empty)
    rd_ptr_bin <= rd_ptr_bin + 1'b1;
endmodule


module zrb_baud_generator
/*
zrb_baud_generator #(50000000,9600) instance_name(input_clk, baud_clk, baud_clk_8);
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
zrb_uart_tx #(8,"NO",1) instance_name(
    CLK,
    CLK_EN, //BAUD RATE
    RESET,
    WRITE,
    INPUT_DATA[7:0],
    OUTPUT_TX,
    OUTPUT_BUSY
    );
*/
    #(parameter NUM_BITS = 4'd8, parameter PARITY = "NO", parameter STOP_BIT = 4'd1)
    (
    input   wire                clk,
    input   wire                clk_en,
    input   wire                reset,
    input   wire                write,
    input   wire    [  7 :  0 ] data,

    output  wire                tx,
    output  wire                busy
    );
localparam START_BIT = 4'd1;
localparam WIDTH =  PARITY == "NO"   ? 	NUM_BITS + START_BIT + STOP_BIT : 
					PARITY == "EVEN" ?	NUM_BITS + START_BIT + STOP_BIT + 1 :
					PARITY == "ODD"  ? 	NUM_BITS + START_BIT + STOP_BIT + 1 : 0;

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
begin
    if(write & ~busy)
    begin
        r_data <= {data, 1'b0};
        r_cnt <= START_BIT+NUM_BITS+STOP_BIT;
    end
    
    if(sending & clk_en)
    begin
        {r_data, r_tx} <= {1'b1, r_data};
        r_cnt <= r_cnt - 1'b1;
    end
end
endmodule


module zrb_uart_rx
/*
zrb_uart_rx #(8,"NO",1) instance_name(
    INPUT_CLK, //HIGH FREQ
	CLK_EN,
	RESET
    INPUT_RX,
    OUTPUT_DATA[7:0],
	WRITE_EN,
    OUTPUT_BUSY,
    );
*/
    #(parameter [3:0] NUM_BITS = 8, parameter PARITY = "NO", parameter [3:0]STOP_BIT = 1)
    (
    input   wire                clk,
	input	wire				clk_en,
	input	wire				reset,
	
    input   wire                rx,

    output  wire    [  7 :  0 ] data_out,
	output	wire				write_en,
    output  wire                busy
    );

localparam [3:0] START_BIT = 1;
localparam [3:0] WIDTH = PARITY == "NO"   ? 	NUM_BITS + START_BIT + STOP_BIT : 
						 PARITY == "EVEN" ?		NUM_BITS + START_BIT + STOP_BIT + 1 :
						 PARITY == "ODD"  ? 	NUM_BITS + START_BIT + STOP_BIT + 1 : 0;
reg					start_sync = 1'b0;
reg					start_en = 1'b0;
wire				start = ~start_sync & start_en;
reg     [  9 :  0 ] r_data = 10'b0;
reg     [  3 :  0 ] r_cnt = 4'b0;
reg		[  2 :  0 ] clk_en_cnt = 3'b0;
wire				receiving = |r_cnt;
assign				busy = receiving;
assign				write_en = clk_en & r_cnt == 1 & clk_en_cnt == 3'd3;
assign 				data_out = r_data[(WIDTH-2)-:8];

always@(posedge clk)
if(reset)
begin
	start_sync <= 1'b0;
	start_en <= 1'b0;
	r_data <= 10'b0;
	r_cnt <= 4'b0;
	clk_en_cnt <= 3'b0;
end
else
begin
	start_sync <= rx;
	start_en <= start_sync;
	
	if(start & ~receiving)
	begin
        r_cnt <= WIDTH;
		clk_en_cnt <= 3'b0;
	end
	if(receiving & clk_en)
	begin	
		clk_en_cnt <= clk_en_cnt + 1'b1;
		if(clk_en_cnt == 3'd3)
		begin
			//r_data <= {r_data[(WIDTH-2):0], rx};
			r_data <= {rx, r_data[(WIDTH-2):1]};
			r_cnt <= r_cnt - 1'b1;
		end
	end
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
