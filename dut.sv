//---------------------------------------------------------------------------
// DUT - 564/464 Project
//---------------------------------------------------------------------------
`include "common.vh"

module MyDesign(
//---------------------------------------------------------------------------
//System signals
  input wire reset_n                      ,  
  input wire clk                          ,

//---------------------------------------------------------------------------
//Control signals
  input wire dut_valid                    , 
  output wire dut_ready                   ,

//---------------------------------------------------------------------------
//input SRAM interface
  output wire                           dut__tb__sram_input_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_input_write_data    ,
  output reg  [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_read_address  , 
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_input_read_data     ,     

//weight SRAM interface
  output wire                           dut__tb__sram_weight_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_weight_write_data    ,
  output reg  [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_read_address  , 
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_weight_read_data     ,     

//result SRAM interface
  output wire                           dut__tb__sram_result_write_enable  ,
  output reg  [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_result_write_data    ,
  output reg  [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_read_address  , 
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_result_read_data     ,    

//scratchpad SRAM interface
  output wire                           dut__tb__sram_scratchpad_write_enable  ,
  output reg  [`SRAM_ADDR_RANGE     ]   dut__tb__sram_scratchpad_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_scratchpad_write_data    ,
  output reg  [`SRAM_ADDR_RANGE     ]   dut__tb__sram_scratchpad_read_address  , 
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_scratchpad_read_data  

);

//delcare section
parameter one_address_offset = 12'h1;

//FSM variable
parameter idle_state = 4'd0;
parameter get_dimension_state = 4'd1;
parameter accumulating_state = 4'd2;
parameter writing_state = 4'd3;
parameter next_multiplication_setup_state = 4'd4;
parameter setup_multiplication_2_state = 4'd5;
parameter setup_multiplication_3_state = 4'd6;
parameter setup_multiplication_4_state = 4'd7;
parameter setup_multiplication_5_state = 4'd8;

//this just for me to easy to see the clock waveform and debug
wire my_clock; 
assign my_clock = clk; 
reg [3:0] next_state, current_state;

//data read out from the SRAM that is going to perform matrix multiplication
//We write accumulated_result to the result SRAM 
reg  [`SRAM_DATA_RANGE     ]  input_data;
reg  [`SRAM_DATA_RANGE     ]  weight_data;
reg  [`SRAM_DATA_RANGE     ]  accumulated_result; 
wire [`SRAM_DATA_RANGE     ]  mac_result_z;

//We calculatd the correct address in these read/write_address_next, then send it to read/write_address that connected to SRAM to fetch data
reg [`SRAM_ADDR_RANGE     ] input_read_address_next;
reg [`SRAM_ADDR_RANGE     ] weight_read_address_next;
reg [`SRAM_ADDR_RANGE     ] result_write_address_next;
reg [`SRAM_ADDR_RANGE     ] scratchpad_write_address_next;

//These variables help us to fetch the data in a correctly
//These variables document some important address in the matrix, 
//we use input_read_address_next and weight_read_address_next to dectect:
//what is the "next address" we should sent to "address" in order to fetch the correct data 
reg [`SRAM_ADDR_RANGE     ] next_input_matrix_first_address;
reg [`SRAM_ADDR_RANGE     ] next_weight_matrix_first_address;
reg [`SRAM_ADDR_RANGE     ] first_element_of_calculated_input_row_address;
reg [`SRAM_ADDR_RANGE     ] last_element_of_calculated_input_row_address;
reg [`SRAM_ADDR_RANGE     ] first_element_of_calculated_weight_matrix_address;
reg [`SRAM_ADDR_RANGE     ] last_element_of_calculated_weight_matrix_address;



//These counters helps us to change the state of FSM
//since the size of test data is not too large, therefore, the size following registers is set at most 8 bits to save area
reg [7:0] counter_total_element_in_result_matrix; 
reg [7:0] counter_go_write; 
reg [2:0] counter_next_step;
reg [1:0] counter_3_cycle;

//These registers document the matrix dimension(row and col)
//Notice, these only document the fisrt three matrix pair(which located at Input and Weight SRAM)
//This is ok since the rest of matrix land thier dimension from the first three matrix(but this let us take some effort to track the dimension of them)
//so there are acutally only three possible value(number_of_input_cols == number_of_weight_rows actually)
reg [7:0] number_of_input_rows, number_of_input_cols;
reg [7:0] number_of_weight_rows, number_of_weight_cols;



/* ================The following section is datapath including next address calculation, counter, and matrix multiplication ================ */

//we dont write to input and weight SRAM, but still need to make sure it active low
assign dut__tb__sram_input_write_enable = 0;
assign dut__tb__sram_weight_write_enable = 0;

//sub-circuit 1: next address calculation (read)
//input_read_address calculate the next address for input matrix
//input matrix means the first matrix, for example, C = A * B, then A is the input matrix
//it't not always refers to input SRAM, based on my design, result SRAM will be the input matrix in the step 4 and 5(read the value from result SRAM).
/*
The code below seems complicated here, but it's actually not. There are only few SAME logics behind this procedural block, 
and this logic is what human will think when perform matrix multiplication. 
Here are the logics:
1. Once the dut_valid == 1, which serve as a botton to activite the circuit, our input_read_address_next start to calculate address to fetch correct data
2. Circle back to the first element of the calculated row in input matrix just like what human always do when perform matrix multiplication.
   We use variable "last_element_of_calculated_weight_matrix_address" to detect when the read_address_next need to circle back to the beginning of the row.
   Before circle back to the next first element of the row, we fetches one useless data from the SRAM, so in the following circuit will see the condition like:
   "(input_read_address_next == last_element_of_calculated_input_row_address + one_address_offset)" this(key: + one_address_offset), 
   the reason I do this is because I want to avoid "read and write" in my writing state although this is not require in the SPEC,
   but I think it's common and need to obey in the actual situation(industry).
   Based on my design, this data will pop up when we perform writing.
3. Change to next row, instead of circle back to the first element of the calculated row in input matrix, 
   we also need to change next row, we change to next row when the weight_read_address_next reach the last element address in the weight matrix,
   at this time, the result matrix also change to next row as it is natural property of matrix multiplication.
   Notice, point 3 will has PRIORITY over point 2,
   point 3 and point 2 condition will overlap as its natural properity of matrix multiplication,
   but point 3 has more strict condition.
4. Change to the correct matrix address when we going to perform next matrix multiplication, like:
   input_read_address_next <= next_input_matrix_first_address;
Don't be afraid of the condition like (counter_next_step == 6)..., it actually perform the things mention above,
unlike regular address offset which is one,
it just has different address offset(i.e. number_of_weight_col) since it perform the multiplication with transposed weight matrix
(fyi, step5 looks like the one which perform with transposed matrix, but it's actually last steps, see the SPEC address allocation will figure it out)
*/
always @(posedge clk or negedge reset_n)begin
  if(!reset_n)
    input_read_address_next <= 0;
  else if(dut_valid == 1)
    input_read_address_next <= input_read_address_next + one_address_offset; 
  else if(current_state == idle_state)
    input_read_address_next <= 0;
  else if(current_state == get_dimension_state)
    input_read_address_next <= input_read_address_next + one_address_offset;
  else if((counter_next_step == 6) && (weight_read_address_next == last_element_of_calculated_weight_matrix_address + number_of_weight_cols)) 
    input_read_address_next <= last_element_of_calculated_input_row_address + one_address_offset; 
  else if((counter_next_step == 6) && (weight_read_address_next > last_element_of_calculated_weight_matrix_address))
    input_read_address_next <= first_element_of_calculated_input_row_address; 
  else if((weight_read_address_next == last_element_of_calculated_weight_matrix_address + one_address_offset) && ((current_state == accumulating_state) || (current_state == writing_state) || (current_state == get_dimension_state) || (current_state == setup_multiplication_2_state) || (current_state == setup_multiplication_3_state) || (current_state == setup_multiplication_4_state) || (current_state == setup_multiplication_5_state)))
    input_read_address_next <= last_element_of_calculated_input_row_address + one_address_offset; 
  else if((input_read_address_next == last_element_of_calculated_input_row_address + one_address_offset) && ((current_state == accumulating_state) || (current_state == writing_state) || (current_state == get_dimension_state) || (current_state == setup_multiplication_2_state) || (current_state == setup_multiplication_3_state) || (current_state == setup_multiplication_4_state) || (current_state == setup_multiplication_5_state)))
    input_read_address_next <= first_element_of_calculated_input_row_address; 
  else if((current_state == accumulating_state) || (current_state == writing_state))
    input_read_address_next <= input_read_address_next + one_address_offset;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 2))
    input_read_address_next <= next_input_matrix_first_address;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 3))
    input_read_address_next <= next_input_matrix_first_address;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 4))
    input_read_address_next <= next_input_matrix_first_address;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 5))
    input_read_address_next <= next_input_matrix_first_address;
  else 
    input_read_address_next <= input_read_address_next + one_address_offset;
end

//Assign input_read_address_next to the read_address that is going to read data from SRAM(either input or result SRAM)
//result SRAM serve as role like input SRAM in the step 4 and 5 since we have to fetch the Q and S matrix's data from it
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    dut__tb__sram_input_read_address <= 0; 
  else if(current_state == idle_state)
    dut__tb__sram_input_read_address <= 0; 
  else if(counter_next_step <= 4)
    dut__tb__sram_input_read_address <= input_read_address_next;
  else
    dut__tb__sram_result_read_address <= input_read_address_next;      
end


//weight_read_address_next
/* 
Similar to logics in input_read_next, weight_read_address_next
1. Start to calculate the correct address once the dut_valid activite high
2. Keep fetch the next address til the end of the weight matrix(we do fetch one useless/wrong data just like input_read_address_next does)
   Circle back to the first element when weight_read_address_next reaches the last element's address in the weight address
3. Update address when we are going to perform next martix muliplication
Notice: there are some condition (counter_next_step == 6) ... looks complicated, but it actually just the logic mention in the above, 
the reason why it looks complicated is just because it perform mutiplication with a transposed matrix, therefore,
its address offset is not just simply + one_address_offset, instead, it's a variable based on the weight matrix V dimension.
If dont understand the notice section, list the address of five matrix and their dimension can get a clear view(it's a lot of work, but need to if want to understand)
*/
always @(posedge clk or negedge reset_n)begin
  if(!reset_n)
    weight_read_address_next <= 0; 
  else if(dut_valid == 1)
    weight_read_address_next <= weight_read_address_next + one_address_offset;
  else if(current_state == idle_state)
    weight_read_address_next <= 0;
  else if(current_state == get_dimension_state)
    weight_read_address_next <= weight_read_address_next + one_address_offset;
  else if((counter_next_step == 6) && (weight_read_address_next == last_element_of_calculated_weight_matrix_address + number_of_weight_cols) && ((current_state == accumulating_state) || (current_state == writing_state) || (current_state == get_dimension_state) || (current_state == setup_multiplication_2_state) || (current_state == setup_multiplication_3_state) || (current_state == setup_multiplication_4_state) || (current_state == setup_multiplication_5_state)))
    weight_read_address_next <= first_element_of_calculated_weight_matrix_address;
  else if((counter_next_step == 6) && (weight_read_address_next > last_element_of_calculated_weight_matrix_address) && ((current_state == accumulating_state) || (current_state == writing_state) || (current_state == get_dimension_state) || (current_state == setup_multiplication_2_state) || (current_state == setup_multiplication_3_state) || (current_state == setup_multiplication_4_state) || (current_state == setup_multiplication_5_state)))
    weight_read_address_next <= weight_read_address_next - number_of_input_rows*(number_of_weight_cols) + one_address_offset; //change to the second element in the transpose matrix
  else if(counter_next_step == 6) 
    weight_read_address_next <= weight_read_address_next + number_of_weight_cols; // number_of_weight_cols serve as address offset in transpose matrix
  else if((counter_next_step < 6) && (weight_read_address_next == last_element_of_calculated_weight_matrix_address + one_address_offset) && (input_read_address_next == last_element_of_calculated_input_row_address + one_address_offset))// this take pripority over the next condition!!
    weight_read_address_next <= first_element_of_calculated_weight_matrix_address; 
  else if(input_read_address_next == last_element_of_calculated_input_row_address + one_address_offset)
    weight_read_address_next <= weight_read_address_next; 
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 2))
    weight_read_address_next <= next_weight_matrix_first_address;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 3))
    weight_read_address_next <= next_weight_matrix_first_address;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 4))
    weight_read_address_next <= next_weight_matrix_first_address;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 5))
    weight_read_address_next <= next_weight_matrix_first_address;
  else 
    weight_read_address_next <= weight_read_address_next + one_address_offset;
end



//Assign weight_read_address_next to the read_address that is going to read data from SRAM(either weight or scratchpad SRAM)
//scratchpad SRAM serve as role like weight SRAM in the step 4 and 5
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    dut__tb__sram_weight_read_address <= 0; 
  else if(current_state == idle_state)
    dut__tb__sram_weight_read_address <= 0; 
  else if(counter_next_step <= 4)
    dut__tb__sram_weight_read_address <= weight_read_address_next;
  else
    dut__tb__sram_scratchpad_read_address <= weight_read_address_next;      
end

//sub-circuit 1: next address calculation (write)
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    result_write_address_next <= 0;
  else if(current_state == idle_state)
    result_write_address_next <= 0;
  else if(current_state == writing_state)
    result_write_address_next <= result_write_address_next + one_address_offset;
  else 
    result_write_address_next <= result_write_address_next;
end

//dut__tb__sram_result_write_address
always @(posedge clk or negedge reset_n) 
begin
    if (!reset_n) 
      dut__tb__sram_result_write_address <= 0;         
    else if(current_state == idle_state)
      dut__tb__sram_result_write_address <= 0; 
    else
      dut__tb__sram_result_write_address <= result_write_address_next;           
end

//scratchpad_write_address_next
//we write all the result to result AND scratchpad SRAM so that we can fetch two data from both of them to perform
//matrix multiplication step 4 and 5 which is faster way(since we can one fetch one data from result SRAM at a time which will slow down the multiplication)
always @(posedge clk or negedge reset_n)begin
  if(!reset_n)
      scratchpad_write_address_next <= 0;            
  else if(current_state == idle_state)
      scratchpad_write_address_next <= 0;  
  else if(current_state == writing_state)
      scratchpad_write_address_next <= scratchpad_write_address_next + one_address_offset; 
  else 
      scratchpad_write_address_next <= scratchpad_write_address_next; 
end
  
//dut__tb__sram_scratchpad_write_address
always @(posedge clk or negedge reset_n)begin
  if (!reset_n) 
    dut__tb__sram_scratchpad_write_address <= 0;         
  else if(current_state == idle_state)
    dut__tb__sram_scratchpad_write_address <= 0;
  else
    dut__tb__sram_scratchpad_write_address <= scratchpad_write_address_next;           
end

//first_element_of_calculated_input_row_address
//See the SPEC's address allocation can easily gain the insight of the code
/*The first three step we all use input SRAM's input matrix which first data located at address 1, 
in step 4, where we use Q in result SRAM, its first data address located at address 0,
the last one, which is step 5, it's input matrix is S, it is located at result SRAM, 
since result SRAM also have matrix Q, K, and V before matrix S, therefore, matrix S locate at 3*(number_of_input_rows * number_of_weight_cols)
as 3*(number_of_input_rows * number_of_weight_cols) is the total number of elements in previous three matrix*/
/*Notice, the offset of step 4 and step 5 is different as both dimension is diff from matirx in input&weight matrix*/
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)  
    first_element_of_calculated_input_row_address <= 0;
  else if(current_state == idle_state)
    first_element_of_calculated_input_row_address <= 0;
  else if(current_state == get_dimension_state) 
    first_element_of_calculated_input_row_address <= 1;
  else if((counter_next_step == 6) && (weight_read_address_next == last_element_of_calculated_weight_matrix_address + number_of_weight_cols)) //number_of_weight_cols serve as address_offset for transpose 
    first_element_of_calculated_input_row_address <= first_element_of_calculated_input_row_address + number_of_input_rows;
  else if((counter_next_step == 5) && (weight_read_address_next == last_element_of_calculated_weight_matrix_address + one_address_offset))
    first_element_of_calculated_input_row_address <= first_element_of_calculated_input_row_address + number_of_weight_cols;
  else if((counter_next_step < 5) && (weight_read_address_next == last_element_of_calculated_weight_matrix_address + one_address_offset))
    first_element_of_calculated_input_row_address <= first_element_of_calculated_input_row_address + number_of_input_cols;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 2))
    first_element_of_calculated_input_row_address <= 1;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 3))
    first_element_of_calculated_input_row_address <= 1;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 4))
    first_element_of_calculated_input_row_address <= 0;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 5))
    first_element_of_calculated_input_row_address <= 3*(number_of_input_rows * number_of_weight_cols); //S
  else 
    first_element_of_calculated_input_row_address <= first_element_of_calculated_input_row_address;
end

//last_element_of_calculated_input_row_address
/*Dont be intimitated by the complicated number there, it literally just the address of last element in cacluated row,
see the SPEC(the address sectoin) can easily understand, 
if having query about setup_multiplication_5_state, then draw the five matrix by hand*/
/*Notice, the offset of step 4 and step 5 is different as both dimension is diff from matirx in input&weight matrix*/
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    last_element_of_calculated_input_row_address <= 0;
  else if(current_state == idle_state)
    last_element_of_calculated_input_row_address <= 0;
  else if(current_state == get_dimension_state)
    last_element_of_calculated_input_row_address <= input_data[15:0];
  else if((counter_next_step == 6) && (weight_read_address_next == last_element_of_calculated_weight_matrix_address + number_of_weight_cols) && (input_read_address_next == last_element_of_calculated_input_row_address + one_address_offset))
    last_element_of_calculated_input_row_address <= last_element_of_calculated_input_row_address + number_of_input_rows;//one_address_offset;//number_of_weight_cols; // this need debug//last_element_of_calculated_input_row_address + number_of_weight_cols;
  else if((counter_next_step == 5) && (weight_read_address_next == last_element_of_calculated_weight_matrix_address + one_address_offset) && (input_read_address_next == last_element_of_calculated_input_row_address + one_address_offset))
    last_element_of_calculated_input_row_address <= last_element_of_calculated_input_row_address + number_of_weight_cols;
  else if((counter_next_step < 5) && (weight_read_address_next == last_element_of_calculated_weight_matrix_address + one_address_offset) && (input_read_address_next == last_element_of_calculated_input_row_address + one_address_offset))
    last_element_of_calculated_input_row_address <= last_element_of_calculated_input_row_address + number_of_input_cols;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 2))
    last_element_of_calculated_input_row_address <= number_of_input_cols;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 3))
    last_element_of_calculated_input_row_address <= number_of_input_cols; 
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 4))
    last_element_of_calculated_input_row_address <= number_of_weight_cols - 1;
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 5))
    last_element_of_calculated_input_row_address <=  (3*(number_of_input_rows * number_of_weight_cols)) + number_of_input_rows - 1; //S
  else 
    last_element_of_calculated_input_row_address <= last_element_of_calculated_input_row_address;
end

//first_element_of_calculated_weight_matrix_address
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    first_element_of_calculated_weight_matrix_address <= 0;
  else if(current_state == idle_state)
    first_element_of_calculated_weight_matrix_address <= 0;
  else if(current_state == get_dimension_state)
    first_element_of_calculated_weight_matrix_address <= 1; //Wq
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 2))
    first_element_of_calculated_weight_matrix_address <= first_element_of_calculated_weight_matrix_address + number_of_weight_rows * number_of_weight_cols; //Wk
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 3))
    first_element_of_calculated_weight_matrix_address <= first_element_of_calculated_weight_matrix_address + number_of_weight_rows * number_of_weight_cols; //Wv
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 4))
    first_element_of_calculated_weight_matrix_address <= number_of_input_rows * number_of_weight_cols; //K
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 5))
    first_element_of_calculated_weight_matrix_address <= first_element_of_calculated_weight_matrix_address + (number_of_input_rows * number_of_weight_cols); //V
  else
    first_element_of_calculated_weight_matrix_address <= first_element_of_calculated_weight_matrix_address;
end

//last_element_of_calculated_weight_matrix_address
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    last_element_of_calculated_weight_matrix_address <= 0;
  else if(current_state == idle_state)
    last_element_of_calculated_weight_matrix_address <= 0;
  else if(current_state == get_dimension_state)
    last_element_of_calculated_weight_matrix_address <= number_of_weight_rows * number_of_weight_cols; //Wq
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 2))
    last_element_of_calculated_weight_matrix_address <= last_element_of_calculated_weight_matrix_address + number_of_weight_rows * number_of_weight_cols; //Wk
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 3))
    last_element_of_calculated_weight_matrix_address <= last_element_of_calculated_weight_matrix_address + number_of_weight_rows * number_of_weight_cols; //Wv
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 4))
    last_element_of_calculated_weight_matrix_address <= 2 * number_of_input_rows * number_of_weight_cols - 1; //K
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 5))
    last_element_of_calculated_weight_matrix_address <= last_element_of_calculated_weight_matrix_address + (number_of_input_rows * number_of_weight_cols); //V
  else
    last_element_of_calculated_weight_matrix_address <= last_element_of_calculated_weight_matrix_address;
end


//next_input_matrix_first_address
//We prepare this so that the input_read_address can land the correct address IMMEDIATELY at one-cycle next_multiplication_setup_state 
always @(posedge clk or negedge reset_n)
begin
  if(!reset_n) 
    next_input_matrix_first_address <= 0;
  else if(current_state == idle_state)
    next_input_matrix_first_address <= 0;
  else if(current_state == get_dimension_state)
    next_input_matrix_first_address <= 1;//Matrix I(for step2)
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 2))
    next_input_matrix_first_address <= 1;//Matrix I(for step3)
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 3))
    next_input_matrix_first_address <= 0;//Matrix Q(for step4)
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 4))
    next_input_matrix_first_address <= 3*(number_of_input_rows * number_of_weight_cols); //Matrix S(for step5)
  else 
    next_input_matrix_first_address <= next_input_matrix_first_address;
end

//next_weight_matrix_first_address
//same logic like next_input_matrix_first_address
//see the SPEC can gain insight of how these first_address calculated
always @(posedge clk or negedge reset_n)begin
  if(!reset_n)
    next_weight_matrix_first_address <= 0;
  else if(current_state == idle_state)
    next_weight_matrix_first_address <= 0;
  else if(current_state == get_dimension_state)
    next_weight_matrix_first_address <= number_of_weight_rows * number_of_weight_cols + 1; //Matrix WQ(for step2)
  else if((current_state == next_multiplication_setup_state) && ((counter_next_step == 2)))
    next_weight_matrix_first_address <= next_weight_matrix_first_address + (number_of_weight_rows * number_of_weight_cols); //Matrix WV(for step3)
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 3))
    next_weight_matrix_first_address <= number_of_input_rows * number_of_weight_cols; //Matrix K(for step4)
  else if((current_state == next_multiplication_setup_state) && (counter_next_step == 4))
    next_weight_matrix_first_address <= 2 * number_of_input_rows * number_of_weight_cols; //Matrix V(for step5)
  else 
    next_weight_matrix_first_address <= next_weight_matrix_first_address;
end



//sub-circuit 2: retriving matrix dimension
//Notice, this just the dimension of matrix in INPUT SRAM and WEIGHT SRAM,
//we use these data and the relationship of matrix to know the following matrix dimension(i.e matrix Q, K, V, and S)
//The following is actually three type of number as number of input col MUST == number of weight row to perform matrix multiplication
always @(posedge clk or negedge reset_n) 
begin
    if(!reset_n)begin
      number_of_input_rows <= 0;
      number_of_input_cols <= 0;
      number_of_weight_rows <= 0;
      number_of_weight_cols <= 0;
    end else if(current_state == idle_state)begin
      number_of_input_rows <= 0;
      number_of_input_cols <= 0;
      number_of_weight_rows <= 0;
      number_of_weight_cols <= 0;
    end else if(current_state == get_dimension_state)   begin
      number_of_input_rows <= input_data[31:16];  
      number_of_input_cols <= input_data[15:0];
      number_of_weight_rows <= weight_data[31:16];
      number_of_weight_cols <= weight_data[15:0];
    end else begin
      number_of_input_rows <= number_of_input_rows;
      number_of_input_cols <= number_of_input_cols;
      number_of_weight_rows <= number_of_weight_rows;
      number_of_weight_cols <= number_of_weight_cols;
    end        
end


//counter_go_write
/*This counter = number of input cols - 1, when it count down to 0(which means it counts for number_of_input_cols times)
we will write the accumulated result to the result SRAM, and set the input of accumulated result to 0 to perform next one.
The reason why the last step assign counter_go_write <= number_of_input_rows - 1 is because matrix S = Q x K transpose,
Q's number of input cols == number of inputs row as K transpose's cols = number of input row from Input matrix.
If don't understand above, it's fine, just list the five matrix multiplication equation will understand.*/
/*To summarize, same logic behind this is nothing but land the number of "current" input matrix's cols, when it counts to 0,
that mean we already fetch the end of the row, it's time to write the accumulated_result to the result SRAM*/
always @(posedge clk or negedge reset_n)begin
  if(!reset_n)
    counter_go_write <= 0;
  else if(current_state == idle_state)
    counter_go_write <= 0; 
  else if(current_state == get_dimension_state)
    counter_go_write <= input_data[15:0] - 1; //number of input cols - 1
  else if(current_state == accumulating_state) 
    counter_go_write <= counter_go_write - 1;
  else if((current_state == writing_state) && (counter_next_step == 6))
    counter_go_write <= number_of_input_rows - 1; 
  else if((current_state == writing_state) && (counter_next_step == 5)) 
    counter_go_write <= number_of_weight_cols - 1; //notice: in the step4, input matrix Q's number of cols land from the "number_of_weight_cols" of Wq so I assgin this instead of number_of_input_cols which belongs to Input matrix in input SRAM
  else if((current_state == writing_state) && (counter_next_step < 5))
    counter_go_write <= number_of_input_cols - 1;
  else if(current_state == setup_multiplication_2_state)
    counter_go_write <= number_of_input_cols - 1;
  else if(current_state == setup_multiplication_3_state)
    counter_go_write <= number_of_input_cols - 1;
  else if(current_state == setup_multiplication_4_state)
    counter_go_write <= number_of_weight_cols - 1; //notice: in the step4, input matrix Q's number of cols land from the "number_of_weight_cols" of Wq so I assgin this instead of number_of_input_cols which belongs to Input matrix in input SRAM
  else if(current_state == setup_multiplication_5_state)
    counter_go_write <= number_of_input_rows - 1; // S matrix's cols = number of input rows done
  else 
    counter_go_write <= counter_go_write; 
end

//counter_total_element_in_result_matrix 
/*this counter = # of input row x # of weight column, this is key concept, and this counter help us to change to 
next_multiplication_setup_state as it dectects whether we finish writing the result matrix or not*/
/*If see the assign value not exactly == (number_of_input_rows * number_of_weight_cols) - 1,
it just becasue some matrix perform tranpose or do the second time of calculation, dont worry too much, not a problem*/
always @(posedge clk or negedge reset_n) 
begin
  if(!reset_n)
    counter_total_element_in_result_matrix <= 0;
  else if (current_state == idle_state)
    counter_total_element_in_result_matrix <= 0;     
  else if (current_state == get_dimension_state)
    counter_total_element_in_result_matrix <= (input_data[31:16] * weight_data[15:0]) - 1; 
  else if (current_state == writing_state) 
    counter_total_element_in_result_matrix <= counter_total_element_in_result_matrix - 1;
  else if (current_state == setup_multiplication_2_state) 
    counter_total_element_in_result_matrix <= (number_of_input_rows * number_of_weight_cols) - 1;
  else if (current_state == setup_multiplication_3_state) 
    counter_total_element_in_result_matrix <= (number_of_input_rows * number_of_weight_cols) - 1;
  else if (current_state == setup_multiplication_4_state) 
    counter_total_element_in_result_matrix <= (number_of_input_rows * number_of_input_rows) - 1;  //notice: K tranpose
  else if (current_state == setup_multiplication_5_state) 
    counter_total_element_in_result_matrix <= (number_of_input_rows * number_of_weight_cols) - 1; 
  else 
    counter_total_element_in_result_matrix <= counter_total_element_in_result_matrix; // just prevent latches
end

//counter_next_step
//document the next step that tell the next_multiplication_setup_state jump to correct setup_multiplication_n_state
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)begin
    counter_next_step <= 0;
  end else if(current_state == idle_state) begin
    counter_next_step <= 2;
  end else if(current_state == next_multiplication_setup_state)begin
    counter_next_step <= counter_next_step + 1;
  end else
    counter_next_step <= counter_next_step;
end


//counter_3_cycle
//reason we setup this cycle to let S1, S5, S6, S7, S8 state last three cycle is becasue the machnisim of 
//SRAM data fetching, it takes three cycle to land the data, that is,
//count from read_address_next -> read_address -> read_data -> input/weight data,
//we send the desired data's address at either S0 or S4 state, the actual data we want to calculate pop up 4 cycle later,
//(S0&S4 take one cycle, and the S1, S5, S6, S7, S8 take the rest of three cycles)
//so when the data pop up at input/weight data, it will be at S2(accumulating state) just in time to perform mutiplication.
//This is a very deep pipeline, with only stall during one-cycle at writing state(MUST)
always @(posedge clk or negedge reset_n)begin
  if(!reset_n)
    counter_3_cycle <= 0;
  else if(current_state == idle_state) 
    counter_3_cycle <= 2;
  else if (current_state == get_dimension_state)
    counter_3_cycle <= counter_3_cycle - 1;
  else if(current_state == setup_multiplication_2_state) 
    counter_3_cycle <= counter_3_cycle - 1;
  else if(current_state == setup_multiplication_3_state) 
    counter_3_cycle <= counter_3_cycle - 1;
  else if(current_state == setup_multiplication_4_state) 
    counter_3_cycle <= counter_3_cycle - 1;
  else if(current_state == setup_multiplication_5_state) 
    counter_3_cycle <= counter_3_cycle - 1;
  else 
    counter_3_cycle <= 2;
end

//sub-circuit 4: Matrix Multiplication
//retriving data from data bus then perform multiplication
//Notice: first three multiplication data came from input and weight SRAM,
//and the last of two matrix multiplication data came from the result SRAM and scratchpad SRAM
always @(posedge clk or negedge reset_n) 
begin
    if(!reset_n)begin
      input_data <= 0;      
      weight_data <= 0;        
    end else if(current_state == idle_state)begin
      input_data <= 0;      
      weight_data <= 0;  
    end else if(counter_next_step <= 4)begin 
      input_data <= tb__dut__sram_input_read_data;      
      weight_data <= tb__dut__sram_weight_read_data;
    end else begin 
      input_data <= tb__dut__sram_result_read_data;      
      weight_data <= tb__dut__sram_scratchpad_read_data;
    end 
end

//Matrix Multiplication, determine accumulation or write the result to result SRAM
//mac_result_z is the output of this MAC circuit, and we write the accumulated_result to the result SRAM
//The circuit perform like this: mac_result_z = input_data * weight_data + accumulated_result
interger_accumulated_matrix_multiplication u1( 
  .input_data(input_data),   
  .weight_data(weight_data),  
  .accumulated_result_data(accumulated_result),
  .result_data(mac_result_z)
);


//accumuated_result will be clean when writing, this is for calculating next result element
always @(posedge clk or negedge reset_n)begin
  if(!reset_n)
    accumulated_result <= 0;
  else if (current_state == idle_state)
    accumulated_result <= 0;
  else if((current_state == accumulating_state))
    accumulated_result <= mac_result_z;
  else if(current_state == writing_state)
    accumulated_result <= 0;
  else if(current_state == next_multiplication_setup_state)
    accumulated_result <= 0;
  else if(current_state == setup_multiplication_2_state)
    accumulated_result <= 0;
  else if(current_state == setup_multiplication_3_state)
    accumulated_result <= 0;
  else if(current_state == setup_multiplication_4_state)
    accumulated_result <= 0;
  else if(current_state == setup_multiplication_5_state)
    accumulated_result <= 0;
  else 
    accumulated_result <= accumulated_result;
end

//Write final result to SRAM and setting write enable signal.
assign dut__tb__sram_result_write_data = accumulated_result;
assign dut__tb__sram_result_write_enable = (current_state == writing_state) ? 1 : 0;

//writeh transpose matrix K to the scratchpad SRAM
assign dut__tb__sram_scratchpad_write_data = accumulated_result;
assign dut__tb__sram_scratchpad_write_enable = (current_state == writing_state) ? 1 : 0;



/* ==============================================Above is datapath, controlled by the following FSM) =============================================*/

//FSM
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    current_state <= 0;
  else 
    current_state <= next_state;
end


always @(*) begin
  case (current_state)

    idle_state:begin
      if (dut_valid == 1)  
        next_state = get_dimension_state;
      else // if dut_valid == 1
        next_state = idle_state;
    end

    //this state will last 3 cycle to let the first desired data pop up just in time when switch to accumulating state
    get_dimension_state:begin
      if(counter_3_cycle != 0)
        next_state = get_dimension_state;
      else 
        next_state = accumulating_state;
    end

    accumulating_state: begin 
      if(counter_go_write == 0)
        next_state = writing_state;
      else 
        next_state = accumulating_state;
    end

    //counter_total_element_in_result_matrix == 0 mean we already write all the result element in the result SRAM, ready to perform next one.
    writing_state:begin
      if(counter_total_element_in_result_matrix != 0) 
        next_state = accumulating_state;
      else 
        next_state = next_multiplication_setup_state;
    end
    
    //looks the counter to know which setup state we should jump to,
    //we actually setup most address in this step(combine with counter_next_step to know how to setup address)
    //the setup_mutiplication_n_state main function is more like get_dimension_state, 
    //last for 3 cycle to let the first desired data pop up just in time when switching back to accumulating state
    next_multiplication_setup_state:begin
      if(counter_next_step == 2)
        next_state = setup_multiplication_2_state;
      else if(counter_next_step == 3)
        next_state = setup_multiplication_3_state;
      else if(counter_next_step == 4)
        next_state = setup_multiplication_4_state;
      else if(counter_next_step == 5)
        next_state = setup_multiplication_5_state;
      else if(counter_next_step == 6)
        next_state = idle_state; 
      else 
        next_state = next_multiplication_setup_state;
    end

    //As mention before:
    /*setup_mutiplication_n_state main function is more like get_dimension_state, 
    last for 3 cycle to let the first desired data pop up just in time when switching back to accumulating state*/
    //But these do setup some useful info like counter_go_write(this can actually be improved next time,
    //we might can just merge the following 4 state to one state, but I didnt do due to time limits)
    setup_multiplication_2_state:begin
      if(counter_3_cycle == 0) 
        next_state = accumulating_state;
      else  
        next_state = setup_multiplication_2_state;
    end

    setup_multiplication_3_state:begin
      if(counter_3_cycle == 0) 
        next_state = accumulating_state;
      else  
        next_state = setup_multiplication_3_state;
    end

    setup_multiplication_4_state:begin
      if(counter_3_cycle == 0) 
        next_state = accumulating_state;
      else  
        next_state = setup_multiplication_4_state;
    end

    setup_multiplication_5_state:begin
      if(counter_3_cycle == 0) 
        next_state = accumulating_state;
      else  
        next_state = setup_multiplication_5_state;
    end

    default: next_state = idle_state;
  endcase
end

//Based on SPEC, ready signal active high at idle state, telling testbench that the circuit is ready to compute next computation
assign dut_ready = (current_state == idle_state) ? 1:0;

endmodule


//My interger matrix multiplication
module interger_accumulated_matrix_multiplication ( 
  input wire [`SRAM_DATA_MSB  : 0] input_data,
  input wire [`SRAM_DATA_MSB  : 0] weight_data,
  input wire [`SRAM_DATA_MSB : 0] accumulated_result_data,
  output wire [`SRAM_DATA_MSB : 0] result_data
);
  assign result_data = (input_data * weight_data) + accumulated_result_data;
endmodule: interger_accumulated_matrix_multiplication










