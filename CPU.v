`timescale 1ns / 1ps

module processor ( input clk, reset, output [31:0] PC, input [31:0] instruction, output WE, output [31:0] address_to_mem, output [31:0] data_to_mem, input [31:0] data_from_mem);

	//Wires
		wire [31:0]	SrcA, SrcB, RD2, SignImm, PCBeqAddr, PCJalAddr, ALUOut, Result, WD3;
		
		wire [4:0]	WriteReg, A3;
		wire [3:0]	ALUControl;
		wire 		ControlJal, ControlJr, ControlRegWrite, ControlMemToReg, ControlALUSrc, ControlRegDst, ControlBranch;

		wire 		Zero, Carry, Overflow;
	
		wire 		PCBranch;
		wire [31:0]	isBranchOut, isJalOut;

		reg [31:0] PCNew, PCIDC;

		reg [31:0] PCPlus4;

	//Assigning
		assign SignImm			= {{16{instruction[15]}}, instruction[15:0]}; 
		assign PCBranch			= Zero & ControlBranch;
		assign PCJalAddr 		= {PCPlus4[31:28], instruction[25:0], 2'b00};
		assign PCBeqAddr 		= (SignImm << 2) + PCPlus4;
		assign address_to_mem	= ALUOut;
		assign data_to_mem		= RD2; 
		assign PC 				= PCIDC;


	//Multiplexer
		assign isBranchOut	= (PCBranch) ? PCBeqAddr : PCPlus4;
		assign isJalOut		= (ControlJal) ? PCJalAddr : isBranchOut;
		

		assign WD3			= (ControlJal) ? PCPlus4 : Result;
		assign SrcB			= (ControlALUSrc) ? SignImm : RD2;
		assign Result		= (ControlMemToReg) ? data_from_mem : ALUOut;

		assign WriteReg		= (ControlRegDst) ? instruction[15:11] : instruction[20:16];
		assign A3 			= (ControlJal) ? 5'b11111 : WriteReg;


	//Other Instances
		reg32x32		regFile			(instruction[25:21], instruction[20:16], A3, WD3, clk, reset, ControlRegWrite, SrcA, RD2);
		alu				ALU				(SrcA, SrcB, ALUControl, Zero, Carry, Overflow, ALUOut);
		control_unit	CU				(instruction[31:26], instruction[5:0], instruction[10:6], ControlJal, ControlJr, ControlRegWrite, ControlMemToReg, WE, ControlALUSrc, ControlRegDst, ControlBranch, ALUControl);


		always @ (posedge clk)
			PCIDC = PCNew;

		always @ (negedge clk)
		begin
			if(reset)
				PCNew = 32'h00000000;
			else
				PCNew = ((ControlJr) ? SrcA : isJalOut);
		end

		always @ (posedge clk)
		begin
			if(reset)
			begin
				@(negedge reset)
				PCPlus4 = 32'h00000000;
			end
			else
				PCPlus4 = PCNew + 4;
		end

endmodule

//-----------------------------------------------------------
module reg32x32 (input [4:0] A1, A2, A3, input [31:0] WD3, input CLK, RESET, WE3, output [31:0] RD1, RD2);
				 
	reg [31:0] rf [31:0];
	integer k = 0;

	always @ (posedge CLK)
	begin
		if(RESET)
			for(k = 0; k < 32; k = k+1)
			begin
				rf[k] = 0; 
			end
	end

	//always @ (*)
	//begin
	//     RD1 <= rf[A1];
	//     RD2 <= rf[A2];
	// end 

	assign RD1 = (A1 == 0) ? 32'h00000000 : rf[A1];
	assign RD2 = (A2 == 0) ? 32'h00000000 : rf[A2];

	always @ (posedge CLK)
	begin
		if(WE3 && !RESET && A3 != 5'b00000) rf[A3] <= WD3;
	end
	
	always @(rf[2])
	$display("Change of [2]: %h", rf[2]);
	always @(rf[4])
	$display("Change of [4]: %h", rf[4]);
	always @(rf[5])
	$display("Change of [5]: %h", rf[5]);
		
		
	always @(rf[8])
	$display("Change of [8]: %h", rf[8]);
	always @(rf[9])
	$display("Change of [9]: %h", rf[9]);
	always @(rf[10])
	$display("Change of [10]: %h", rf[10]);
	always @(rf[11])
	$display("Change of [11]: %h", rf[11]);
	always @(rf[12])
	$display("Change of [12]: %h", rf[12]);
	always @(rf[13])
	$display("Change of [13]: %h", rf[13]);
	always @(rf[14])
	$display("Change of [14]: %h", rf[14]);
	always @(rf[15])
	$display("Change of [15]: %h", rf[15]);
	always @(rf[24])
	$display("Change of [24]: %h", rf[24]);
	always @(rf[25])
	$display("Change of [25]: %h", rf[25]);
	always @(rf[31])
	$display("Change of [31]: %h", rf[31]);
endmodule
//-----------------------------------------------------------
module alu (input [31:0]  SrcA, SrcB, input [3:0]ALUControl, output reg Zero, Carry, Overflow, output reg  [31:0]  ALUResult);
	initial
	begin
	ALUResult = 32'h00000000;
	Zero = 0;
	Carry = 0;
	Overflow = 0;
	end
			
			
	reg [32:0] res;
	reg [32:0] signext;
	reg [8:0]  addu_res;

	function [7:0] SatSum;
		input [7:0] a, b;
		reg cout;
		reg [7:0] res;
		begin
			cout = 0;
			{cout, res} = a + b;
			SatSum = (cout) ? 8'hff : res;
		end
	endfunction
			   
	always @ (*)
	begin
				
		case(ALUControl)
			4'b0000: //and
			begin
				ALUResult <= SrcA & SrcB;
			
			end	
			4'b0001: //or
			begin
				ALUResult <= SrcA | SrcB;
			end
			4'b0010: //add
			begin
				res <= SrcA + SrcB;
				ALUResult <= res [31:0];
				if(res [32] == 1)
					Carry <= 1;
				if((SrcA[31] ^ SrcB[31] == 0) && (SrcA[31] ^ ALUResult[31] == 1)) 
					Overflow <= 1;
			end
			4'b0011: //xor
			begin
				ALUResult <= SrcA ^ SrcB;
			end	
			4'b0110: //sub
			begin
				ALUResult <= SrcA - SrcB;
				signext <= {1'b1,SrcA[31:0]};
				res <= signext - SrcB;
				if(res[32] == 0)
					Carry <= 1;
				if((SrcA[31] ^ SrcB[31] == 1) && (SrcA[31] ^ ALUResult[31] == 1))
					Overflow <= 1;
					
			end				
			4'b0111: //slt
			begin
				ALUResult <= $signed (SrcA) < $signed (SrcB);
			end	
			4'b1000: // four sums
			begin
				ALUResult[7:0] <= SrcA[7:0] + SrcB [7:0];
				ALUResult[15:8] <= SrcA[15:8] + SrcB [15:8];
				ALUResult[23:16] <= SrcA[23:16] + SrcB [23:16];
				ALUResult[31:24] <= SrcA[31:24] + SrcB [31:24];    
			end
				
 			4'b1001: // saturated four sums
			begin
				//$display("Change of ALUControl: %h", ALUControl);
	/*
				addu_res <= (SrcA[7:0] + SrcB [7:0]);
				ALUResult[7:0] <= (addu_res[8]) ? 8'hFF : addu_res[7:0];
				addu_res <= (SrcA[15:8] + SrcB [15:8]);
				ALUResult[15:8] <= (addu_res[8]) ? 8'hFF : addu_res[7:0];
				addu_res <= (SrcA[23:16] + SrcB [23:16]);
				ALUResult[23:16] <= (addu_res[8]) ? 8'hFF : addu_res[7:0];
				addu_res <= (SrcA[31:24] + SrcB [31:24]);
				ALUResult[31:24] <= (addu_res[8]) ? 8'hFF : addu_res[7:0];
	*/
				ALUResult[7:0] = SatSum(SrcA[7:0] , SrcB[7:0]);
				ALUResult[15:8] = SatSum(SrcA[15:8] , SrcB[15:8]);
				ALUResult[23:16] = SatSum(SrcA[23:16] , SrcB[23:16]);
				ALUResult[31:24] = SatSum(SrcA[31:24] , SrcB[31:24]);
				//$display("end of addu_s.qb");
			end
			4'b1100: //sllv
			begin
				ALUResult <= SrcB << SrcA;

			end
			4'b1110: //srlv
			begin
				ALUResult <= SrcB >> SrcA;
			end
			4'b1111: //srav
			begin
				ALUResult <= $signed(SrcB) >>> SrcA;
			end
		endcase
		
		Zero <= ~(|ALUResult);

	end            
endmodule
//-----------------------------------------------------------
module control_unit (input  [5:0] OpCode, Funct, input  [4:0] Shamt, output       Jal, Jr, RegWrite, MemToReg, MemWrite, ALUSrc, RegDst, Branch,output [3:0] ALUControl);
	wire [1:0] ALUOp;

	mainDecoder MD (OpCode, ALUOp, Jal, Jr, RegWrite, MemToReg, MemWrite, ALUSrc, RegDst, Branch);
	AluOpDecoder AOD (Funct, Shamt, ALUOp, ALUControl );
endmodule
//-----------------------------------------------------------
module mainDecoder (input [5:0] OpCode, output reg [1:0] ALUOp, output reg Jal, Jr, RegWrite, MemToReg, MemWrite, ALUSrc, RegDst, Branch);
	always @ (*)
	begin

		case(OpCode)
			6'b000000: //R type
			begin
				Jal <= 1'b0;
				Jr  <= 1'b0;
				MemToReg <= 1'b0;
				MemWrite <= 1'b0;
				Branch <= 1'b0;
				ALUOp <= 2'b10;
				ALUSrc <= 1'b0;
				RegDst <= 1'b1;
				RegWrite <= 1'b1;
			end
			
			6'b100011: //lw
			begin 
				Jal <= 1'b0;
				Jr  <= 1'b0;
				MemToReg <= 1'b1;
				MemWrite <= 1'b0;
				Branch <= 1'b0;
				ALUOp <= 2'b00;
				ALUSrc <= 1'b1;
				RegDst <= 1'b0;
				RegWrite <= 1'b1;
			end
			
			6'b101011: //sw
			begin 
				Jal <= 1'b0;
				Jr  <= 1'b0;
				MemToReg <= 1'bx;
				MemWrite <= 1'b1;
				Branch <= 1'b0;
				ALUOp <= 2'b00;
				ALUSrc <= 1'b1;
				RegDst <= 1'bx;
				RegWrite <= 1'b0;
			end
				6'b000100: //beq
			begin 
				Jal <= 1'b0;
				Jr  <= 1'b0;
				MemToReg <= 1'bx;
				MemWrite <= 1'b0;
				Branch <= 1'b1;
				ALUOp <= 2'b01;
				ALUSrc <= 1'b0;
				RegDst <= 1'bx;
				RegWrite <= 1'b0;
			end
				6'b001000: //addi
			begin 
				Jal <= 1'b0;
				Jr  <= 1'b0;
				MemToReg <= 1'b0;
				MemWrite <= 1'b0;
				Branch <= 1'b0;
				ALUOp <= 2'b00;
				ALUSrc <= 1'b1;
				RegDst <= 1'b0;
				RegWrite <= 1'b1;
			end
			
				6'b000011: //jal
			begin 
				Jal <= 1'b1;
				Jr  <= 1'b0;
				MemToReg <= 1'bx;
				MemWrite <= 1'b0;
				Branch <= 1'bx;
				ALUOp <= 2'bxx;
				ALUSrc <= 1'bx;
				RegDst <= 1'bx;
				RegWrite <= 1'b1;
			end
			
				6'b000111: //jr
			begin 
				Jal <= 1'b0;
				Jr  <= 1'b1;
				MemToReg <= 1'bx;
				MemWrite <= 1'b0;
				Branch <= 1'bx;
				ALUOp <= 2'bxx;
				ALUSrc <= 1'bx;
				RegDst <= 1'bx;
				RegWrite <= 1'b0;
			end
			
				6'b011111: //addu[_s].qb
			begin 
				Jal <= 1'b0;
				Jr  <= 1'b0;
				MemToReg <= 1'b0;
				MemWrite <= 1'b0;
				Branch <= 1'b0;
				ALUOp <= 2'b11;
				ALUSrc <= 1'b0;
				RegDst <= 1'b1;
				RegWrite <= 1'b1;
			end
		endcase

	end          
endmodule
//-----------------------------------------------------------
module AluOpDecoder (input [5:0] Funct, input [4:0] Shamt, input [1:0] ALUOp, output reg [3:0] ALUControl);
									  
	always @ (*)
	begin
		case (ALUOp)
			2'b00: //addition
			begin
				ALUControl <= 4'b0010;
			end
			2'b01: //substraction
			begin
				ALUControl <= 4'b0110;
			end
			
			2'b10: 
			begin
				case (Funct)
					6'b100000: //addition
					begin
						ALUControl <= 4'b0010;
					end
					6'b100010: //substraction
					begin
						ALUControl <= 4'b0110;
					end
					6'b100100: //and
					begin
						ALUControl <= 4'b0000;
					end 
					6'b100101: //or
					begin
						ALUControl <= 4'b0001;
					end 
					6'b101010: //set less than
					begin   
						ALUControl <= 4'b0111;
					end 
					6'b000100: // logical shift left
					begin   
						ALUControl <= 4'b1100;
					end 
					6'b000110: // logical shift right
					begin   
						ALUControl <= 4'b1110;
					end 
					6'b000111: // arithmetic shift right
					begin   
						ALUControl <= 4'b1111; 
					end 
				endcase
			end
			
			2'b11:
			begin
				//$display("Change of Shamt: %h", Shamt);
				case (Shamt)
					5'b00000:
					begin
						ALUControl <= 4'b1000;
					end
					5'b00100:
					begin
						ALUControl <= 4'b1001;
					end
				endcase
			end
		endcase
	end
endmodule
//-----------------------------------------------------------
