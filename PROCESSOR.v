module PROCESSOR(clk, reset);
   input clk, reset; 
	
	wire [31:0]	pc_top, instruction_top, rd1_top, read_data2_top, sign_out,mux1_out,sum_out_top, nextPC_top, mux_out2_top, address_select,
	memdata_out_top, write_back_top; 
	wire RegWrite_top, alusrc_top, zero_top, banch_top, select2_top, MemtoReg_top, MemWrite_top,MemRead_top; 
	wire [1:0] ALUOp_top; 
	
	// program counter 
   wire [3:0] control_top; 
	PC program_counter(.reset(reset), .pc_in(mux_out2_top), .clk(clk), .pc_out(pc_top)); 
	
	// pc cong 4 
	pc_cong_4 adder_4(.fromPC(pc_top), .nextPC(nextPC_top)); 
	
	
	// instruction memory 
	instruction_memory IM(.clk(clk), .reset(reset), .read_address(pc_top), .instruction_out(instruction_top)); 

	// register file 
	Register_File RF(.Clk(clk), .Reset(reset), .RegWrite(RegWrite_top), .Rs1(instruction_top[19:15]), .Rs2(instruction_top[24:20]), 
	      .Rd(instruction_top[11:7]), .Write_data(write_back_top), .Read_data1(rd1_top), .Read_data2(read_data2_top)); 
	
	
   // sign extend 
	 sign_extend sign_ex(.Opcode(instruction_top[6:0]), .Instruction(instruction_top), .ImmExt(sign_out)); 
	 
	 // control unit 
	 control_unit control_(.Instruction(instruction_top[6:0]), .Branch(banch_top), .MemRead(MemRead_top), .MemtoReg(MemtoReg_top), 
	 .MemWrite(MemWrite_top),.AluSrc(alusrc_top),.RegWrite(RegWrite_top),.ALUOp(ALUOp_top)); 
	 
    // alu control 
	 
	  ALU_Control ALU_con(.ALUOp(ALUOp_top), .Fun7(instruction_top[30]), .Fun3(instruction_top[14:12]), .Control_out(control_top));
    
	  // ALU 
	  
	  ALU alu(.A(rd1_top),.B(mux1_out),.Control_in(control_top), .ALU_result(address_select), .zero(zero_top));
	 
	 // alu mux 
	 mux1 alu_mux(.sel1(alusrc_top), .a1(read_data2_top), .b1(sign_out), .mux_out(mux1_out)); 
	 
	 // ADDER
	 
	 Adder ADDER_(.in_1(pc_top),.in_2(sign_out), .sum_out(sum_out_top));  
	 
	 // AND GATE 
	 And_Logic  AND_LOGIC(.branch(banch_top), .zero(zero_top), .and_out(select2_top)); 
	
    // mux 
	 mux2 sum_mux(.sel2(select2_top), .a2(nextPC_top), .b2(sum_out_top), .mux_out2(mux_out2_top)); 
	 
	 // data memory 
	 
	 Data_Memory ram(.clk(clk), .reset(reset), .MemWrite(MemWrite_top), .MemRead(MemRead_top), 
	 .Read_address(address_select), .Write_data(read_data2_top), .Memdata_out(memdata_out_top)); 
	 
	 // mux data memory 
	 
	 mux3 mux_data_mem(.sel3(MemtoReg_top), .a3(address_select), .b3(memdata_out_top), .mux_out3(write_back_top)); 
	 
endmodule 

// program counter 
module PC(
   input reset, 
	input [31:0] pc_in, 
	input clk, 
	output reg [31:0] pc_out ); 
	always@(posedge clk or posedge reset)
	  begin 
	     if(reset)
		     pc_out <= 32'b0; 
		  else 
		      pc_out <= pc_in; 
		end 		
endmodule 
// pc +4; 

module pc_cong_4(
     
	  input [31:0] fromPC, 
	  output [31:0] nextPC 
);
   assign nextPC = 4+ fromPC; 
endmodule 

// instruction memory (ROM); 
module instruction_memory(
   input clk, 
	input reset, 
	input [31:0] read_address, 
	output [31:0] instruction_out );
	
   reg [31:0] I_Mem [63:0];
	integer i; 
assign  instruction_out = I_Mem[read_address[5:0]];
   initial begin 
     for(i=0; i<64; i= i+1)
	  begin 
	     I_Mem[i] <= 32'b0; 
		end 
		end 
		
   always @(posedge clk or posedge reset)
	begin 
	  if(reset)
	    begin 
		    for(i =0; i<64; i = i+1 )
              begin 
                 I_Mem[i] <= 32'b0; 
	           end 
       end 
     else 
// r type 	  
      I_Mem[0] = 32'b0; 
		I_Mem[4] = 32'b00000001100110000000011010110011; // add x13, x16, x25  
		I_Mem[8] = 32'b01000000001101000000001010110011; // sub x5, x8, x3 
		I_Mem[12] = 32'b00000000001100010111000010110011;// and x1, x2, x3 
		I_Mem[16] = 32'b00000000010100101110001000110011; // or x4, x3, x5 
	// i type 
      I_Mem[20] = 32'b00000000001110101000101100010011; // addi x22, x21, 3
	   I_Mem[24] = 32'b00000000000101000110010010010011; // ori x9, x8, 1
	// l type 
      I_Mem[28] = 32'b00000000111100101010010000000011; // lw x8, 15(x5)
	   I_Mem[32] = 32'b00000000001100011010010010000011; // lw x9, 3(x3)
	 // s- type 
	   I_Mem[36] = 32'b00000000111100101010011000100011; // sw x15, 12(x5)
	   I_Mem[40] = 32'b00000000111000110010010100100011; // sw x14, 10(x6)
	 // sb type 
	   I_Mem[44] = 32'h00948663; // beq x9,x9,12  
			
end 
endmodule 
// resgister file 
module Register_File(
   input Clk, 
	input Reset, 
	input RegWrite,  
	input [4:0] Rs1, // read select 
	input [4:0] Rs2, 
	input [4:0] Rd, // 
	input [31:0] Write_data, 
	output [31:0] Read_data1, 
	output [31:0] Read_data2 
    ); 
	 
	 reg [31:0] Register [31:0]; 
	 
	 initial begin 
	 Register[0] = 0; 
	 Register[1] = 4; 
	 Register[2] = 2; 
	 Register[3] = 24; 
	 Register[4] = 4; 
	 Register[5] = 1; 
	 Register[6] = 44; 
	 Register[7] = 4; 
	 Register[8] = 2; 
	 Register[9] = 1; 
	 Register[10] = 23; 
	 Register[11] = 4; 
	 Register[12] = 90; 
	 Register[13] = 10; 
	 Register[14] = 20; 
	 Register[15] = 30; 
	 Register[16] = 40; 
	 Register[17] = 50; 
	 Register[18] = 60; 
	 Register[19] = 70; 
	 Register[20] = 80; 
	 Register[21] = 80; 
	 Register[22] = 90; 
	 Register[23] = 70; 
	 Register[24] = 60; 
	 Register[25] = 65; 
	 Register[26] = 4; 
	 Register[27] = 32; 
	 Register[28] = 12; 
	 Register[29] = 34; 
	 Register[30] = 5; 
	 Register[31] = 10; 
	 end 
	 
	 integer i; 
	 always @(posedge Clk or posedge Reset)
	 begin 
	    if(Reset)
		   begin 
			   for(i =0; i<32; i = i+1)
				begin
			     Register[i]<= 32'b0;  
				end 
			end 
	 else if (RegWrite) 
	 begin 
	       Register [Rd] <= Write_data; // ghi du lieu 
	  end 
	 end 
assign Read_data1 = Register[Rs1]; 
assign Read_data2 = Register[Rs2]; 

endmodule 
// sign extend 
module sign_extend(
    input [6:0] Opcode,
    input [31:0] Instruction,
    output reg [31:0] ImmExt);
    
    always @(*)
    begin
        case(Opcode)
            7'b0000011 : ImmExt = {{20{Instruction[31]}}, Instruction[31:20]}; // lw
            7'b0100011 : ImmExt = {{20{Instruction[31]}}, Instruction[31:25], Instruction[11:7]}; // sw
            7'b1100011 : ImmExt = {{19{Instruction[31]}}, Instruction[31], Instruction[30:25], Instruction[11:8], 1'b0}; // beq
            default    : ImmExt = 32'b0; // 
        endcase
    end
endmodule

// control unit 
module control_unit(
    input [6:0] Instruction,
    output reg Branch,
    output reg MemRead,
    output reg MemtoReg,
    output reg MemWrite,
    output reg AluSrc,
    output reg RegWrite,
    output reg [1:0] ALUOp);
    
    always @(*)
    begin
        case(Instruction)
            7'b0110011 : {AluSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b00100001; // r_format
            7'b0000011 : {AluSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b11110000; // lw
            7'b0100011 : {AluSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b10001000; // sw
            7'b1100011 : {AluSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b00000101; // beq
            default    : {AluSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b00000000; // 
        endcase
    end
endmodule
	
// alu 
module ALU(A, B, Control_in, ALU_result, zero);
    input [31:0] A, B;
    input [3:0] Control_in;
    output reg zero;
    output reg [31:0] ALU_result;
    
    always @(Control_in or A or B)
    begin
        case(Control_in)
            4'b0000: begin zero <= 1'b0; ALU_result <= A & B; end
            4'b0001: begin zero <= 1'b0; ALU_result <= A | B; end
            4'b0010: begin zero <= 1'b0; ALU_result <= A + B; end
            4'b0110: begin zero <= (A == B) ? 1'b1 : 1'b0; ALU_result <= A - B; end
            default: begin zero <= 1'b0; ALU_result <= 32'b0; end // Trường hợp mặc định
        endcase
    end
endmodule
 	 
// ALU CONTROL 
module ALU_Control(ALUOp, Fun7, Fun3, Control_out);
   input [1:0] ALUOp; 
   input Fun7; 
   input [2:0] Fun3; 
   output reg [3:0] Control_out; 
	
	always@(*)
	begin 
	    case({ALUOp, Fun7, Fun3})
		 6'b000000: Control_out <= 4'b0010; 
		 6'b010000: Control_out <= 4'b0110; 
		 6'b100000: Control_out <= 4'b0010; 
		 6'b101000: Control_out <= 4'b0110; 
		 6'b100111: Control_out <= 4'b0000; 
		 6'b100110: Control_out <= 4'b0001; 
		 default: Control_out <=4'b0; 
		endcase 
	end 	 
endmodule
// data memory: 
module Data_Memory(clk, reset, MemWrite, MemRead, Read_address, Write_data, Memdata_out);
    input clk, reset, MemWrite, MemRead;
    input [31:0] Read_address, Write_data;
    output [31:0] Memdata_out;
    reg [31:0] D_Memory[63:0];
    integer i;
    
    always @(posedge clk or posedge reset)
    begin
        if(reset)
        begin
            for(i = 0; i < 64; i = i + 1)
                D_Memory[i] <= 32'b0;
        end
        else if (MemWrite)
            D_Memory[Read_address] <= Write_data;
    end
    
    assign Memdata_out = (MemRead) ? D_Memory[Read_address] : 32'b0;
endmodule

// multiplexer 
module mux1(sel1, a1, b1, mux_out); 
   input sel1; 
	input [31:0] a1, b1; 
	output [31:0] mux_out; 
	
assign mux_out =(sel1 == 1'b0) ? a1 :b1; 
endmodule 

module mux2(sel2, a2, b2, mux_out2); 
   input sel2; 
	input [31:0] a2, b2; 
	output [31:0] mux_out2; 
	
assign mux_out2 =(sel2 == 1'b0) ? a2 :b2; 
endmodule 

module mux3(sel3, a3, b3, mux_out3); 
   input sel3; 
	input [31:0] a3, b3; 
	output [31:0] mux_out3; 
	
assign mux_out3 =(sel3 == 1'b0) ? a3 :b3; 
endmodule 

// and logic 
module And_Logic(branch, zero, and_out);
   input branch, zero; 
   output and_out; 
assign and_out = branch & zero; 	
endmodule 
// adder 
module Adder(in_1,in_2, sum_out);

input [31:0] in_1, in_2; 
output [31:0] sum_out; 
assign sum_out = in_1 + in_2;  
endmodule 



	 
	 
	 
	 
	 
	 
	 
	 
