module decoder
(
   input             clk,
   input             rst,
   input             enable,
   input [1:0]       d_in,
   output logic      d_out);

   logic             decoder_o_reg;
		
//bmc module signals
   wire  [1:0]       bmc0_path_0_bmc;  // K=0,1,...7
    wire  [1:0]       bmc1_path_0_bmc;
     wire  [1:0]       bmc2_path_0_bmc;
      wire  [1:0]       bmc3_path_0_bmc;
       wire  [1:0]       bmc4_path_0_bmc;
        wire  [1:0]       bmc5_path_0_bmc;
         wire  [1:0]       bmc6_path_0_bmc;
          wire  [1:0]       bmc7_path_0_bmc;

   wire  [1:0]       bmc0_path_1_bmc;  // K=0,1,...7
    wire  [1:0]       bmc1_path_1_bmc;
     wire  [1:0]       bmc2_path_1_bmc;
      wire  [1:0]       bmc3_path_1_bmc;
       wire  [1:0]       bmc4_path_1_bmc;
        wire  [1:0]       bmc5_path_1_bmc;
         wire  [1:0]       bmc6_path_1_bmc;
          wire  [1:0]       bmc7_path_1_bmc;

//ACS modules signals
   logic   [7:0]       validity;
   logic   [7:0]       selection;
   logic   [7:0]       path_cost   [8];
   wire    [7:0]       validity_nets;
   wire    [7:0]       selection_nets;

   wire              ACS0_selection;  // K=0,1,...7
   wire              ACS1_selection;
   wire              ACS2_selection;
   wire              ACS3_selection;
   wire              ACS4_selection;
   wire              ACS5_selection;
   wire              ACS6_selection;
   wire              ACS7_selection;
   
   wire              ACS0_valid_o;	  // K=0,1,...7
   wire              ACS1_valid_o;
   wire              ACS2_valid_o;
   wire              ACS3_valid_o;
   wire              ACS4_valid_o;
   wire              ACS5_valid_o;
   wire              ACS6_valid_o;
   wire              ACS7_valid_o;

   wire  [7:0]       ACS0_path_cost;  // K=0,1,...7
   wire  [7:0]       ACS1_path_cost;
   wire  [7:0]       ACS2_path_cost;
   wire  [7:0]       ACS3_path_cost;
   wire  [7:0]       ACS4_path_cost;
   wire  [7:0]       ACS5_path_cost;
   wire  [7:0]       ACS6_path_cost;
   wire  [7:0]       ACS7_path_cost;

//Trelis memory write operation
   logic   [1:0]       mem_bank;
   logic   [1:0]       mem_bank_Q;
   logic   [1:0]       mem_bank_Q2;
   logic               mem_bank_Q3;
   logic               mem_bank_Q4;
   logic               mem_bank_Q5;
   logic   [9:0]       wr_mem_counter;
   logic   [9:0]       rd_mem_counter;

// 4 memory banks -- address pointers 
   logic   [9:0]       addr_mem_A;  // K=A,B,C,D
   logic   [9:0]       addr_mem_B;
   logic   [9:0]       addr_mem_C;
   logic   [9:0]       addr_mem_D;
// write enables
   logic               wr_mem_A;	// K=A,B,C,D
   logic               wr_mem_B;
   logic               wr_mem_C;
   logic               wr_mem_D;

// data to memories
   logic   [7:0]       d_in_mem_A;	// K=A,B,C,D
   logic   [7:0]       d_in_mem_B;
   logic   [7:0]       d_in_mem_C;
   logic   [7:0]       d_in_mem_D;
// data from memories
   wire    [7:0]       d_o_mem_A;	// K=A,B,C,D
   wire    [7:0]       d_o_mem_B;
   wire    [7:0]       d_o_mem_C;
   wire    [7:0]       d_o_mem_D;
		  
//Trace back module signals
   logic               selection_tbu_0;
   logic               selection_tbu_1;

   logic   [7:0]       d_in_0_tbu_0;
   logic   [7:0]       d_in_1_tbu_0;
   logic   [7:0]       d_in_0_tbu_1;
   logic   [7:0]       d_in_1_tbu_1;

   wire                d_o_tbu_0;
   wire                d_o_tbu_1;

   logic               enable_tbu_0;
   logic               enable_tbu_1;

//Display memory operations 
   wire                wr_disp_mem_0;
   wire                wr_disp_mem_1;

   wire                d_in_disp_mem_0;
   wire                d_in_disp_mem_1;

   logic   [9:0]       wr_mem_counter_disp;
   logic   [9:0]       rd_mem_counter_disp;

   logic   [9:0]       addr_disp_mem_0;
   logic   [9:0]       addr_disp_mem_1;

//Branch matrc calculation modules
   bmc0   bmc0_inst(d_in,bmc0_path_0_bmc,bmc0_path_1_bmc);
   bmc1   bmc1_inst(d_in,bmc1_path_0_bmc,bmc1_path_1_bmc);
   bmc2   bmc2_inst(d_in,bmc2_path_0_bmc,bmc2_path_1_bmc);
   bmc3   bmc3_inst(d_in,bmc3_path_0_bmc,bmc3_path_1_bmc);
   bmc4   bmc4_inst(d_in,bmc4_path_0_bmc,bmc4_path_1_bmc);
   bmc5   bmc5_inst(d_in,bmc5_path_0_bmc,bmc5_path_1_bmc);
   bmc6   bmc6_inst(d_in,bmc6_path_0_bmc,bmc6_path_1_bmc);
   bmc7   bmc7_inst(d_in,bmc7_path_0_bmc,bmc7_path_1_bmc);
/* K=0,1,...7 
*/


//Add Compare Select Modules
   ACS      ACS0(validity[0],validity[1],bmc0_path_0_bmc,bmc0_path_1_bmc,path_cost[0],path_cost[1],ACS0_selection,ACS0_valid_o,ACS0_path_cost);
   ACS      ACS1(validity[3],validity[2],bmc1_path_0_bmc,bmc1_path_1_bmc,path_cost[3],path_cost[2],ACS1_selection,ACS1_valid_o,ACS1_path_cost);
   ACS      ACS2(validity[4],validity[5],bmc2_path_0_bmc,bmc2_path_1_bmc,path_cost[4],path_cost[5],ACS2_selection,ACS2_valid_o,ACS2_path_cost);
   ACS      ACS3(validity[7],validity[6],bmc3_path_0_bmc,bmc3_path_1_bmc,path_cost[7],path_cost[6],ACS3_selection,ACS3_valid_o,ACS3_path_cost);
   ACS      ACS4(validity[1],validity[0],bmc4_path_0_bmc,bmc4_path_1_bmc,path_cost[1],path_cost[0],ACS4_selection,ACS4_valid_o,ACS4_path_cost);
   ACS      ACS5(validity[2],validity[3],bmc5_path_0_bmc,bmc5_path_1_bmc,path_cost[2],path_cost[3],ACS5_selection,ACS5_valid_o,ACS5_path_cost);
   ACS      ACS6(validity[5],validity[4],bmc6_path_0_bmc,bmc6_path_1_bmc,path_cost[5],path_cost[4],ACS6_selection,ACS6_valid_o,ACS6_path_cost);
   ACS      ACS7(validity[6],validity[7],bmc7_path_0_bmc,bmc7_path_1_bmc,path_cost[6],path_cost[7],ACS7_selection,ACS7_valid_o,ACS7_path_cost);
   
   // selection_nets  = // concatenate ACS7 ,,, ACS0 _selections 
   // validity_nets   = // same for ACSK_valid_os 
   
    assign selection_nets  =  {ACS7_selection,ACS6_selection,ACS5_selection,ACS4_selection,
                              ACS3_selection,ACS2_selection,ACS1_selection,ACS0_selection};
   assign validity_nets    =  {ACS7_valid_o,ACS6_valid_o,ACS5_valid_o,ACS4_valid_o,
                              ACS3_valid_o,ACS2_valid_o,ACS1_valid_o,ACS0_valid_o};

   always @ (posedge clk, negedge rst) begin
      if(!rst)  begin
         validity          <= 8'b11111111;
         selection         <= 8'b00000000;
/* clear all 8 path costs
         path_cost[i]      <= 8'd0;
*/
      path_cost[0]      <= 8'b00000000;
      path_cost[1]     <= 8'b00000000;
      path_cost[2]      <= 8'b00000000;
      path_cost[3]      <= 8'b00000000;
      path_cost[4]      <= 8'b00000000;
      path_cost[5]      <= 8'b00000000;
      path_cost[6]     <= 8'b00000000;
      path_cost[7]      <= 8'b00000000;
      end
      else if(!enable)   begin
         validity          <= 8'b11111111;
         selection        <= 8'b00000000;
/* clear all 8 path costs
         path_cost[i]      <= 8'd0;
*/
      path_cost[0]     <= 8'b00000000;
      path_cost[1]      <= 8'b00000000;
      path_cost[2]     <= 8'b00000000;
      path_cost[3]     <= 8'b00000000;
      path_cost[4]     <= 8'b00000000;
      path_cost[5]      <= 8'b00000000;
      path_cost[6]     <= 8'b00000000;
      path_cost[7]      <= 8'b00000000;
      end
      else if( path_cost[0][7] && path_cost[1][7] && path_cost[2][7] && path_cost[3][7] &&
             path_cost[4][7] && path_cost[5][7] && path_cost[6][7] && path_cost[7][7] )
      begin

         validity          <= validity_nets;
         selection         <= selection_nets;
         
         //path_cost[K]      <= 8'b01111111 & ACSK_path_cost;
         
         path_cost[0]      <= 8'b01111111 & ACS0_path_cost;
         path_cost[1]      <= 8'b01111111 & ACS1_path_cost;
         path_cost[2]      <= 8'b01111111 & ACS2_path_cost;
         path_cost[3]      <= 8'b01111111 & ACS3_path_cost;
         path_cost[4]      <= 8'b01111111 & ACS4_path_cost;
         path_cost[5]      <= 8'b01111111 & ACS5_path_cost;
         path_cost[6]      <= 8'b01111111 & ACS6_path_cost;
         path_cost[7]      <= 8'b01111111 & ACS7_path_cost;
      end
      else   begin
         validity          <= validity_nets;
         selection         <= selection_nets;

         path_cost[0]      <= ACS0_path_cost;
/*  K again = 0, 1, ... 7
*/
path_cost[1]      <= ACS1_path_cost;
path_cost[2]      <= ACS2_path_cost;
path_cost[3]      <= ACS3_path_cost;
path_cost[4]      <= ACS4_path_cost;
path_cost[5]      <= ACS5_path_cost;
path_cost[6]      <= ACS6_path_cost;
path_cost[7]      <= ACS7_path_cost;
      end
   end

   always @ (posedge clk, negedge rst) begin
      if(!rst)
         wr_mem_counter <= 10'b0000000000;
      else if(!enable)
         wr_mem_counter <= 10'b0000000000;
      else
         wr_mem_counter <= wr_mem_counter +10'b0000000001;
   end

   always @ (posedge clk, negedge rst) begin
      if(!rst)
         rd_mem_counter <= 10'b1111111111;// -1   how do you handle this in 10 bit binary?
      else if(enable)
         rd_mem_counter <= rd_mem_counter - 10'b0000000001;
   end

   always @ (posedge clk, negedge rst) begin
      if(!rst)
         mem_bank <= 2'b0;
      else begin
       //  /*if(wr_mem_counter = -1  fill in the guts*/
        //       mem_bank <= mem_bank + 2'b1;
                        if(wr_mem_counter==10'b1111111111)
               mem_bank <= mem_bank + 2'b01;
      end
      end

   always @ (posedge clk)    begin
      d_in_mem_A  <= selection;
      d_in_mem_B  <= selection;
      d_in_mem_C  <= selection;
      d_in_mem_D  <= selection;
   end

   always @ (posedge clk)     begin
      case(mem_bank)
         2'b00:         begin
            addr_mem_A        <= wr_mem_counter;
            addr_mem_B        <= rd_mem_counter;
            addr_mem_C        <= 10'b0000000000;
            addr_mem_D        <= rd_mem_counter;

            wr_mem_A          <= 1'b1;
     
/* other wr_mems = 0
*/	        

       wr_mem_B          <= 1'b0;
            wr_mem_C          <= 1'b0;
            wr_mem_D          <= 1'b0;
         end
         2'b01:         begin
            addr_mem_A        <= rd_mem_counter;
            addr_mem_B        <= wr_mem_counter;
            addr_mem_C        <= rd_mem_counter;
            addr_mem_D        <= 10'b0000000000;

            wr_mem_B          <= 1'b1;
/* other wr_mems = 0
*/	        
wr_mem_A          <= 1'b0;
            wr_mem_C          <= 1'b0;
            wr_mem_D          <= 1'b0;
         end		       
         2'b10:    begin
            addr_mem_A        <= 10'b0000000000;
            addr_mem_B        <= rd_mem_counter;
            addr_mem_C        <= wr_mem_counter;
            addr_mem_D        <= rd_mem_counter;

            wr_mem_C       <= 1'b1;
/* other wr_mems = 0
*/	       
wr_mem_B          <= 1'b0;
            wr_mem_A          <= 1'b0;
            wr_mem_D          <= 1'b0;
         end
         2'b11:     begin
            addr_mem_A        <= rd_mem_counter;
            addr_mem_B        <= 10'b0000000000;
            addr_mem_C        <= rd_mem_counter;
            addr_mem_D        <= wr_mem_counter;

            wr_mem_D       <= 1'b1;
/* other wr_mems = 0
*/	       
wr_mem_B          <= 1'b0;
            wr_mem_C          <= 1'b0;
            wr_mem_A          <= 1'b0;
         end		       
      endcase
  end

//Trelis memory module instantiation

   mem   trelis_mem_A
   (
      .clk,
      .wr(wr_mem_A),
      .addr(addr_mem_A),
      .d_i(d_in_mem_A),
      .d_o(d_o_mem_A)
   );
/* likewise for trelis_memB, C, D
*/
   mem   trelis_mem_B
   (
      .clk,
      .wr(wr_mem_B),
      .addr(addr_mem_B),
      .d_i(d_in_mem_B),
      .d_o(d_o_mem_B)
   );
      mem   trelis_mem_C
   (
      .clk,
      .wr(wr_mem_C),
      .addr(addr_mem_C),
      .d_i(d_in_mem_C),
      .d_o(d_o_mem_C)
   );
      mem   trelis_mem_D
   (
      .clk,
      .wr(wr_mem_D),
      .addr(addr_mem_D),
      .d_i(d_in_mem_D),
      .d_o(d_o_mem_D)
   );

//Trace back module operation

/* create mem_bank to mem_bank_Q2 pipeline */

   always @(posedge clk)
   mem_bank_Q<=mem_bank;

 always @(posedge clk)
   mem_bank_Q2<=mem_bank_Q;



   always @ (posedge clk, negedge rst)
      if(!rst)
            enable_tbu_0   <= 1'b0;
      else if(mem_bank_Q2==2'b10)
            enable_tbu_0   <= 1'b1;

   always @ (posedge clk, negedge rst)
      if(!rst)
            enable_tbu_1   <= 1'b0;
      else if(mem_bank_Q2==2'b11)
            enable_tbu_1   <= 1'b1;
   
   always @ (posedge clk)
      case(mem_bank_Q2)
         2'b00:	  begin
            d_in_0_tbu_0   <= d_o_mem_D;
            d_in_1_tbu_0   <= d_o_mem_C;
            
            d_in_0_tbu_1   <= d_o_mem_C;
            d_in_1_tbu_1   <= d_o_mem_B;

            selection_tbu_0<= 1'b0;
            selection_tbu_1<= 1'b1;

         end
         2'b01:	   begin
            d_in_0_tbu_0   <= d_o_mem_D;
            d_in_1_tbu_0   <= d_o_mem_C;
            
            d_in_0_tbu_1   <= d_o_mem_A;
            d_in_1_tbu_1   <= d_o_mem_D;
            
            selection_tbu_0<= 1'b1;
            selection_tbu_1<= 1'b0;
         end
         2'b10:	   begin
            d_in_0_tbu_0   <= d_o_mem_B;
            d_in_1_tbu_0   <= d_o_mem_A;
            
            d_in_0_tbu_1   <= d_o_mem_A;
            d_in_1_tbu_1   <= d_o_mem_D;

            selection_tbu_0<= 1'b0;
            selection_tbu_1<= 1'b1;
         end
         2'b11:	  begin
            d_in_0_tbu_0   <= d_o_mem_B;
            d_in_1_tbu_0   <= d_o_mem_A;
            
            d_in_0_tbu_1   <= d_o_mem_C;
            d_in_1_tbu_1   <= d_o_mem_B;

            selection_tbu_0<= 1'b1;
            selection_tbu_1<= 1'b0;
         end
      endcase

//Trace-Back modules instantiation

   tbu tbu_0   (
      .clk,
      .rst,
      .enable(enable_tbu_0),
      .selection(selection_tbu_0),
      .d_in_0(d_in_0_tbu_0),
      .d_in_1(d_in_1_tbu_0),
      .d_o(d_o_tbu_0),
      .wr_en(wr_disp_mem_0)
   );

/* analogous for tbu_1
*/

  tbu tbu_1   (
      .clk,
      .rst,
      .enable(enable_tbu_1),
      .selection(selection_tbu_1),
      .d_in_0(d_in_0_tbu_1),
      .d_in_1(d_in_1_tbu_1),
      .d_o(d_o_tbu_1),
      .wr_en(wr_disp_mem_1)
   );

//Display Memory modules Instantioation
//   d_in_disp_mem_K   =  d_o_tbu_K;  K=0,1

assign   d_in_disp_mem_0   =  d_o_tbu_0;
assign   d_in_disp_mem_1   =  d_o_tbu_1;

  //mem_disp   disp_mem_0	  (
  mem_disp   disp_mem_0	  (
      .clk              ,
      .wr(wr_disp_mem_0),
      .addr(addr_disp_mem_0),
      .d_i(d_in_disp_mem_0),
      .d_o(d_o_disp_mem_0)
   );
/* analogous for disp_mem_1
*/

mem_disp   disp_mem_1	  (
  //mem_disp   disp_mem_1	  (
      .clk              ,
      .wr(wr_disp_mem_1),
      .addr(addr_disp_mem_1),
      .d_i(d_in_disp_mem_1),
      .d_o(d_o_disp_mem_1)
   );

// Display memory module operation
   always @ (posedge clk)
      mem_bank_Q3 <= mem_bank_Q2[0];

   always @ (posedge clk) begin
      if(!rst)
         wr_mem_counter_disp  <= 10'b0000000010;
      else if(!enable)
         wr_mem_counter_disp  <= 10'b0000000010;
      else
//       decrement wr_mem_counter_disp 
wr_mem_counter_disp  <= wr_mem_counter_disp -10'd1;
end

   always @ (posedge clk) begin
      if(!rst)
         rd_mem_counter_disp  <= 10'b1111111101;
      else if(!enable)
         rd_mem_counter_disp  <= 10'b1111111101;
      else
// increment    rd_mem_counter_disp     
   rd_mem_counter_disp  <= rd_mem_counter_disp +10'b0000000001;;
   end
   always @ (posedge clk)
   begin
      // if !mem_bank_Q3
        // begin
        //    addr_disp_mem_0   <= rd_mem_counter_disp; 
            addr_disp_mem_1   <= wr_mem_counter_disp;
       //  end
     //  else:	 swap rd and wr 
       case(mem_bank_Q3)
         1'b0:
         begin
            addr_disp_mem_0   <= rd_mem_counter_disp; 
            addr_disp_mem_1   <= wr_mem_counter_disp;
         end
         1'b1:
         begin
            addr_disp_mem_0   <= wr_mem_counter_disp; 
            addr_disp_mem_1   <= rd_mem_counter_disp; 
         end
      endcase
    
end
   //always @ (posedge clk) 	 
// pipeline mem_bank_Q3 to Q5

always @ (posedge clk)
      mem_bank_Q4   <= mem_bank_Q3;

 always @ (posedge clk)
      mem_bank_Q5 <= mem_bank_Q4;
      
      
 //also  d_out = d_o_disp_mem_i 
   // i = mem_bank_buf_buf_buf_buf_buf 
//*/

   always @ (posedge clk)
   begin
      case(mem_bank_Q5)
         1'b0:
         begin
            d_out  <= d_o_disp_mem_0;
         end
         1'b1:
         begin
            d_out  <= d_o_disp_mem_1;
         end
      endcase
   end

endmodule
