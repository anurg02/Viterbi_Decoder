/*make the data path K bits wide for mem_Kx1024
   K=8 for module mem, K=1 for module mem_disp */ 
module mem					(
   input                  clk,
   input                  wr,	 // write enable
   input         [9:0]    addr,
   input         [7:0]         d_i,		// data
   output logic  [7:0]         d_o);
//memory core itself   
   logic       [7:0]           mem   [1024];
/*
   always @ (posedge clk) 

   write synchronously to memory core if enabled
   read synchronously at all times (equiv. to DFF at mem data out)
     
endmodule
*/

initial
begin
  mem[addr] <= 0;
end


   always @ (posedge clk)
   begin
      if(wr)
         mem[addr]   <= d_i;
      d_o  <=  mem[addr];
         
   end
endmodule
