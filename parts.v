/*
        Verilog functions&tasks for everyday
*/



/*
zrb_bin2gray #(8) instance_name (binary_input, gray_output);
*/
module zrb_bin2gray
    #(parameter LENGTH = 8)
    (
    input   wire    [ (LENGTH-1) :  0 ] binary_input,
    output  wire    [ (LENGTH-1) :  0 ] gray_output
    );
genvar k;
generate
    assign gray_output[LENGTH-1] = binary_input[LENGTH-1];
    for(k = LENGTH-2; k >= 0; k = k - 1)
    begin
        assign gray_output[k] = binary_input[k+1] ^ binary_input[k];
    end
endgenerate
endmodule

/*
zrb_gray2bin #(8) instance_name (gray_input, binary_output);
*/
module zrb_gray2bin
    #(parameter LENGTH = 8)
    (
    input   wire    [ (LENGTH-1) :  0 ] gray_input,
    output  wire    [ (LENGTH-1) :  0 ] binary_output
    );
genvar k;
generate
    assign binary_output[LENGTH-1] = gray_input[LENGTH-1];
    for(k = LENGTH-2; k >= 0; k = k - 1)
    begin
        assign binary_output[k] = ^(gray_input >> k);
    end
endgenerate
endmodule


module zrb_fifo
    #(parameter ADDR_WIDTH, DATA_WIDTH)
    (
    input   wire                            reset,

    input   wire                            wr_clk,
    input   wire                            wr_en,
    input   wire    [ (DATA_WIDTH-1) :  0 ] data_in,

    input   wire							rd_clk,
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
reg     [ (ADDR_WIDTH-1) :  0 ] metric = {ADDR_WIDTH{1'b0}};

reg     [ (ADDR_WIDTH-1) :  0 ] mem [  0 : (DEPTH-1) ];
reg     [ (DATA_WIDTH-1) :  0 ] r_data_out = {DATA_WIDTH{1'b0}};
assign data_out = r_data_out;
assign fifo_empty = w_wr_ptr_rd_clk == w_rd_bin;
/*assign fifo_full = w_wr_bin > w_rd_bin ? (w_wr_bin - w_rd_bin == {(ADDR_WIDTH-1){1'b0}, 1'b1}) : 
                                         ()*/

zrb_gray2bin #(ADDR_WIDTH) u0 (r_wr_ptr, w_wr_bin);
zrb_bin2gray #(ADDR_WIDTH) u1 (w_wr_bin + 1'b1, w_wr_next);

zrb_gray2bin #(ADDR_WIDTH) u2 (r_rd_ptr, w_rd_bin);
zrb_bin2gray #(ADDR_WIDTH) u3 (w_rd_bin + 1'b1, w_rd_next);


always@(posedge wr_clk or posedge reset)
if(reset)
    r_wr_ptr <= {ADDR_WIDTH{1'b0}};
else
if(wr_en)
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
if(rd_en)
begin
    r_data_out <= mem[r_rd_ptr];
    r_rd_ptr <= w_rd_next;
end

reg     [ (ADDR_WIDTH-1) :  0 ] sync_0 = {ADDR_WIDTH{1'b0}};
reg     [ (ADDR_WIDTH-1) :  0 ] sync_1 = {ADDR_WIDTH{1'b0}};
wire    [ (ADDR_WIDTH-1) :  0 ] w_wr_ptr_rd_clk = sync_1;
always@(posedge rd_clk)
begin
    sync_0 <= w_wr_bin;
    sync_1 <= sync_0;
end

