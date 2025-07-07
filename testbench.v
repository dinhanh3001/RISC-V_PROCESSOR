module testbench; 
   reg clk,reset; 
	 
	 PROCESSOR uut(.clk(clk), .reset(reset)); 
	 initial begin 
	 clk =1'b0; 
	 reset = 1'b0; 
	 #5; 
	 reset = 1'b1;
	 #5; reset = 1'b0;  
	 #100; 
	 end 
	 
	 always begin 
	  #5 clk = ~clk; 
	  end 
endmodule 
