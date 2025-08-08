/* for our constraint length = 3, there will be 2**3 = 8
of these
*/

module bmc5				  // branch metric computation
(
   input    [1:0] rx_pair,
   output   [1:0] path_0_bmc,
   output   [1:0] path_1_bmc);

/* Fill in the guts per BMC instructions
*/
 assign tmp00         =  (rx_pair[0] ^ 1'b0);
   assign tmp01         =  (rx_pair[1] ^ 1'b1);
   

   assign tmp10         =  (rx_pair[0] ^ 1'b1);
   assign tmp11         =  (rx_pair[1] ^ 1'b0);

   assign path_0_bmc    =  {(tmp00 & tmp01),(tmp00 ^ tmp01)}; 
   assign path_1_bmc    =  {(tmp10 & tmp11),(tmp10 ^ tmp11)};

endmodule
