`timescale 1ns / 1ps
typedef enum {ADD, SUB, AND, OR, SLT, ADDI, BEQ, JAL, JR, ADDU_QB, ADDU_S_QB, SLLV, SRLV, SRAV, J/*, LW, SW*/} instr_op;
class instruction #(parameter MEM_WIDTH = 6);

	
	
	rand instr_op inst_type;
	rand logic [5:0] opcode;
	rand logic [4:0] d, s, t;
	rand logic signed  [15:0] imm;
	rand logic signed [25:0] address;
	rand logic [4:0] shamt;
	rand logic [5:0] funct;



	constraint opcode_first 
	{
		solve inst_type before imm;
		solve inst_type before address;
	}

	constraint fill_opcode 
	{
		(inst_type == ADD 		) -> opcode == 6'b000000;
		
		(inst_type == SUB 		) -> opcode == 6'b000000;
		(inst_type == AND 		) -> opcode == 6'b000000;
		(inst_type == OR  		) -> opcode == 6'b000000;
		(inst_type == SLT 		) -> opcode == 6'b000000;
		(inst_type == ADDI		) -> opcode == 6'b001000;
		(inst_type == BEQ 		) -> opcode == 6'b000100;
		(inst_type == JAL 		) -> opcode == 6'b000011;
		(inst_type == JR  		) -> opcode == 6'b000111;
		(inst_type == ADDU_QB 	) -> opcode == 6'b011111;
		(inst_type == ADDU_S_QB ) -> opcode == 6'b011111;
		(inst_type == SLLV		) -> opcode == 6'b000000;
		(inst_type == SRLV		) -> opcode == 6'b000000;
		(inst_type == SRAV		) -> opcode == 6'b000000;
		(inst_type == J 		) -> opcode == 6'b000010;
		/*
		(inst_type == LW 		) -> opcode == 6'b100011;
		(inst_type == SW 		) -> opcode == 6'b101011;
		*/
	}

	constraint fill_shamt
	{
		(inst_type == ADD 		) -> shamt == 5'b00000;
		(inst_type == SUB 		) -> shamt == 5'b00000;
		(inst_type == AND 		) -> shamt == 5'b00000;
		(inst_type == OR  		) -> shamt == 5'b00000;
		(inst_type == SLT 		) -> shamt == 5'b00000;
		(inst_type == ADDU_QB 	) -> shamt == 5'b00000;
		(inst_type == ADDU_S_QB ) -> shamt == 5'b00100;
		(inst_type == SLLV		) -> shamt == 5'b00000;
		(inst_type == SRLV		) -> shamt == 5'b00000;
		(inst_type == SRAV		) -> shamt == 5'b00000;
	}

	constraint fill_funct
	{
		(inst_type == ADD 		) -> funct == 6'b100000;
		(inst_type == SUB 		) -> funct == 6'b100010;
		(inst_type == AND 		) -> funct == 6'b100100;
		(inst_type == OR  		) -> funct == 6'b100101;
		(inst_type == SLT 		) -> funct == 6'b101010;
		(inst_type == ADDU_QB 	) -> funct == 6'b010000;
		(inst_type == ADDU_S_QB ) -> funct == 6'b010000;
		(inst_type == SLLV		) -> funct == 6'b000100;
		(inst_type == SRLV		) -> funct == 6'b000110;
		(inst_type == SRAV		) -> funct == 6'b000111;
	}

	constraint max_address 
	{		
		address < (1 << MEM_WIDTH);
		address > 0;
		if(inst_type == BEQ ) imm inside {[-127:127]} ;

		/*
		if(inst_type == LW || inst_type == SW) imm > 0;
		*/
	}

	constraint fill_jr 
	{
		(inst_type == JR) -> address == {s, 1'b0, 20'h00008};
	}



	task print;
	begin
		$display("Time: %g", $time);
		$display("Instuction: %s", inst_type.name());
		$display("OPCode: %x", opcode);
		if(inst_type != J && inst_type != JAL)
			$display("$s: %d", s);

		if(inst_type != J && inst_type != JAL && inst_type != JR)
			$display("$t: %d", t);

		if(inst_type != J && inst_type != JAL && inst_type != JR && inst_type != ADDI && inst_type != BEQ /* && inst_type != LW && inst_type != SW*/ )
			$display("$d: %d", d);

		if(inst_type == JAL || inst_type == J )
			$display("Address: %x", address);

		if(inst_type == ADDI || inst_type == BEQ /* || inst_type == LW || inst_type == SW*/)
			$display("Imm: %x", imm);

		if(inst_type == ADD || inst_type == SUB || inst_type == AND || inst_type == OR || inst_type == SLT || inst_type == ADDU_QB || inst_type == ADDU_S_QB || inst_type == SRAV || inst_type == SLLV || inst_type == SRLV )
			$display("Shamt: %x", shamt);

		if(inst_type == ADD || inst_type == SUB || inst_type == AND || inst_type == OR || inst_type == SLT || inst_type == ADDU_QB || inst_type == ADDU_S_QB || inst_type == SRAV || inst_type == SLLV || inst_type == SRLV )
			$display("Funct: %x", funct);

		$display("-------------------------------------------");
		
	end
	endtask : print 

endclass : instruction


module test ();
	instruction #(9) instr;

	initial
	begin
		instr = new();
		repeat (10)
		begin 
			assert(instr.randomize());
			instr.print();
			#10;
		end

		repeat (10)
		begin 
			assert (instr.randomize() with {instr.inst_type == BEQ;});
			instr.print();
			#10;
		end

	end

endmodule
