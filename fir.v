module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,// useless
    output  reg                     wready, //handshake with wvalid
    input   wire                     awvalid, // useless
    input   wire [(pADDR_WIDTH-1):0] awaddr, // conf_write addr
    input   wire                     wvalid, // conf_write valid
    input   wire [(pDATA_WIDTH-1):0] wdata, // conf_write data
    output  wire                     arready,
    input   wire                     rready, // conf_read_check ready
    input   wire                     arvalid, // conf_read_check read
    input   wire [(pADDR_WIDTH-1):0] araddr, // conf_read_check addr
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,    
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                     ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, //useless
    
    // bram for tap RAM
    output  reg [3:0]               tap_WE,
    output  reg                     tap_EN,
    output  reg [(pDATA_WIDTH-1):0] tap_Di,
    output  reg [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  reg [3:0]               data_WE,
    output  reg                     data_EN,
    output  reg [(pDATA_WIDTH-1):0] data_Di,
    output  reg [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

localparam  IDLE = 4'd0,
            READ_COEF = 4'd1,
            READ_DATA = 4'd2,
            TEMP = 4'd3,
            CALCULATE = 4'd4,
            DONE = 4'd5,
            OUTPUT = 4'd6;

reg [3:0] state_r, state_w;
reg [31:0] aps_reg_r;
reg [31:0] data_length_r, data_length_w;
reg ap_start_w, ap_done_w, ap_idle_w;
reg aps_r, aps_w;
wire [31:0] aps_reg_w;
wire ap_start_r, ap_done_r, ap_idle_r;

reg [10:0] data_addr_r, data_addr_w;
reg last;

reg [3:0] calculate_num;
reg [3:0] calculate_cnter;
reg [31:0] partial_sum_r, partial_sum_w;


assign ap_start_r = aps_reg_r[0];
assign ap_done_r = aps_reg_r[1];
assign ap_idle_r = aps_reg_r[2];
assign aps_reg_w = {29'd0, ap_idle_w, ap_done_w, ap_start_w};
assign sm_tvalid = (state_r == OUTPUT);
assign sm_tdata = partial_sum_r;
//FSM

always @(*) begin
    case (state_r)
        IDLE: begin
            if (awvalid) state_w = READ_COEF;
            else state_w = IDLE;
        end
        READ_COEF: begin
            if (ap_start_r) state_w = READ_DATA;
            else state_w = READ_COEF;
        end
        READ_DATA: begin
            state_w = TEMP;
        end
        TEMP:state_w = CALCULATE;
        CALCULATE: begin
            state_w = (calculate_cnter + 1 > calculate_num) ? OUTPUT: CALCULATE;
        end
        OUTPUT:begin
            if (last) state_w = DONE;
            else state_w = READ_DATA;
        end
        DONE: state_w = DONE;
        default: begin
            state_w = IDLE;
        end
    endcase

end

//ap params
always @(*) begin
    case (state_r)
        IDLE: begin
            ap_done_w = 0;
            ap_start_w = 0;
            ap_idle_w = 1;
        end
        READ_COEF: begin
            if (ap_start_r) ap_idle_w = 0;
            else ap_idle_w = 1;
            if (awaddr == 12'h00) ap_start_w = wdata[0];
            else ap_start_w = 0;
        end
        READ_DATA: begin
            ap_start_w = 0;
        end
        OUTPUT: begin
            if (last) begin
                ap_done_w = 1;
                ap_idle_w = 1;
            end else begin
                ap_done_w = 0;
                ap_idle_w = 0;
            end
                
        end
        default: begin
            ap_done_w = ap_done_r;
            ap_start_w = ap_start_r;
            ap_idle_w = ap_idle_r;
        end
    endcase
end
reg rvalid_w, rvalid_r;
assign rvalid = rvalid_r;
assign rdata = (aps_r) ? aps_reg_r : tap_Do;
// handle config write / read TAP

always @(*) begin
    tap_A = 0;
    tap_WE = 0;
    tap_Di = 0;
    tap_EN = 0;
    wready = 0;
    rvalid_w = 0;
    aps_w = 0;
    data_length_w = data_length_r;
    if (state_r == READ_COEF)begin
        wready = 1;
    end
    if (wvalid) begin // write
        if (awaddr == 12'h10) begin
            tap_A = awaddr;
            tap_WE = 4'b0000;
            tap_Di = wdata;
            tap_EN = 0;
            data_length_w = wdata;
        end else 
        if (awaddr == 12'h00) begin
            tap_A = awaddr;
            tap_WE = 4'b0000;
            tap_Di = wdata;
            tap_EN = 0;
        end else begin
            tap_A = awaddr-12'h20;
            tap_WE = 4'b1111;
            tap_Di = wdata;
            tap_EN = 1;
        end
    end
    if (rready) begin
        if (araddr == 12'h00) begin
            tap_A = araddr;
            tap_WE = 0;
            tap_EN = 0;
            rvalid_w = arvalid;
            aps_w = 1;
        end else begin
            if (arvalid) begin
                tap_A = araddr-12'h20;
                tap_WE = 0;
                tap_EN = 1;
                rvalid_w = 1;   
            end
            else begin 
                tap_A = araddr-12'h20;
                tap_WE = 0;
                tap_EN = 0;
                rvalid_w = 0;
            end                 
        end
    end
    if (state_r == TEMP) begin
        tap_A = (calculate_num-1) << 2;
        tap_WE = 0;
        tap_EN = 1;
    end
    if (state_r == CALCULATE) begin
        tap_A = (calculate_num - calculate_cnter - 1) << 2;
        tap_WE = 0;
        tap_EN = 1;
    end
    
end
// handle ss read
reg [4:0] offset;

always @(*) begin
    ss_tready = 0;
    data_Di = 0;
    data_A = 0;
    data_addr_w = data_addr_r;
    data_EN = 0;
    data_WE = 0;
    if (state_r == READ_DATA && ss_tvalid) begin
        ss_tready = 1;
        data_Di = ss_tdata;
        data_A = data_addr_r << 2;
        data_addr_w = (data_addr_r == 10) ? 0 : data_addr_r + 1;
        data_EN = 1;
        data_WE = 4'b1111;
    end
    if (state_r == TEMP) begin
        data_A = offset << 2;
        data_WE = 0;
        data_EN = 1;
    end
    if (state_r == CALCULATE) begin
        data_A = ((calculate_cnter+offset) > 10) ? (calculate_cnter+offset - 11) << 2:((calculate_cnter+offset) << 2);
        data_WE = 0;
        data_EN = 1;
    end
end
// handle fir 



always @(*) begin
    partial_sum_w = partial_sum_r;
    if (state_r == CALCULATE) begin
        partial_sum_w = partial_sum_r + tap_Do * data_Do;
    end
    if (state_r == TEMP) partial_sum_w = 0;// reset 
end




//sequential block
always@(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
		state_r <= IDLE; // FSM
        aps_reg_r <= {29'd0, 1'b1, 1'b0, 1'b0};
        rvalid_r <= 0;
        data_length_r <= 0;
        aps_r <= 0;
        data_addr_r <= 0;
        calculate_num <= 0;
        calculate_cnter <= 0;
        partial_sum_r <= 0;
        last <= 0;
        offset <= 0;
    end else begin
		state_r <= state_w; // FSM
        aps_reg_r <= aps_reg_w;
        rvalid_r <= rvalid_w;
        data_length_r <= data_length_w;
        aps_r <= aps_w;
        data_addr_r <= data_addr_w;
        partial_sum_r <= partial_sum_w;
        if (state_r == READ_DATA) calculate_num <= (calculate_num == 4'd11) ? 4'd11 :calculate_num + 1;
        if (state_r == CALCULATE) calculate_cnter <= calculate_cnter + 1;
        else if (state_r == TEMP) calculate_cnter <= 1;
        last <= ((state_r == OUTPUT) && ss_tlast) || last;
        if (state_r == READ_DATA && calculate_num == 4'd11) offset <= (offset == 4'd10) ? 0 :offset + 1;
    end
end


endmodule
