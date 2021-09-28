`timescale 1ns / 1ps
`ifndef INSTRUCTION_YVSAGBHCIUAHWSIUC
	`define INSTRUCTION_YVSAGBHCIUAHWSIUC
	`include "instruction.sv"
`endif

module driver (CPU_IF.DRIVER_IN cpu_if);
	parameter MEM_WIDTH = 8;
	parameter RAM_LEN = 1<<MEM_WIDTH;

	integer i;
	instruction #(.MEM_WIDTH(MEM_WIDTH)) instr;
	instruction #(.MEM_WIDTH(MEM_WIDTH)) decoded_instr;

	logic [31:0] built_instr;


	logic [31:0] RAM[0:RAM_LEN-1];
	logic [31:0] REG_FIELD [31:0];

	bit timeout;


	task reseting();
		integer k;
		begin
			$display("reseting");
			cpu_if.IF_RESET = 1'b1;
			cpu_if.IF_INSTRUCTION = RAM[0];
			timeout = 0;
			for (int k = 0; k < 32; k = k + 1) begin
				REG_FIELD[k] = 0;
			end
			#30;
			cpu_if.IF_RESET = 1'b0;
		end

	endtask : reseting


	function logic [31:0] instruction_builder (input instruction #(MEM_WIDTH) x);
		case (x.inst_type)
			ADD			: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			SUB			: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			AND			: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			OR			: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			SLT			: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			ADDU_QB		: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			ADDU_S_QB	: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			SLLV		: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			SRLV		: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			SRAV		: instruction_builder = {x.opcode, x.s, x.t, x.d, x.shamt, x.funct};
			ADDI		: instruction_builder = {x.opcode, x.s, x.t, x.imm};
			BEQ			: instruction_builder = {x.opcode, x.s, x.t, x.imm};
			JAL			: instruction_builder = {x.opcode, x.address};
			JR			: instruction_builder = {x.opcode, x.s, 21'b000000000000000001000};
			/*
			J			: instruction_builder = {x.opcode, x.address};
			LW			: instruction_builder = {x.opcode, x.s, x.t, x.imm};
			SW			: instruction_builder = {x.opcode, x.s, x.t, x.imm};
			*/
			default: instruction_builder = {6'b001000, 5'b00000, 5'b00000, 16'h0000}; // NOP -> addi $0, $0, 0
		endcase
		/*
		$display("*******************************************");
		$display("** Instruction: %x",instruction_builder);
		$display("*******************************************");
		instr.print();
		*/
	endfunction : instruction_builder

	task decode_instruction;
		input reg [31:0] coded_instr;
		begin 
			case (coded_instr[31:26])
				6'b000000,
				6'b011111:
					begin
						decoded_instr.opcode = coded_instr[31:26];
						decoded_instr.s = coded_instr[25:21];
						decoded_instr.t = coded_instr[20:16];
						decoded_instr.d = coded_instr[15:11];
						decoded_instr.shamt = coded_instr[10:6];
						decoded_instr.funct = coded_instr[5:0];
						if(coded_instr[31:26] == 6'b000000)
						begin
							case (coded_instr[5:0])
								6'b100000: decoded_instr.inst_type = ADD;
								6'b100010: decoded_instr.inst_type = SUB;
								6'b100100: decoded_instr.inst_type = AND;
								6'b100101: decoded_instr.inst_type = OR;
								6'b101010: decoded_instr.inst_type = SLT;
								6'b000100: decoded_instr.inst_type = SLLV;
								6'b000110: decoded_instr.inst_type = SRLV;
								6'b000111: decoded_instr.inst_type = SRAV;
							endcase
						end
						else
						begin
							if(coded_instr[10:6] == 5'b00100)
								decoded_instr.inst_type = ADDU_S_QB;
							else
								decoded_instr.inst_type = ADDU_QB;
						end
					end
				6'b000010,
				6'b000011:
					begin
						if(coded_instr[31:26] == 6'b000010)
							decoded_instr.inst_type = J;
						else
							decoded_instr.inst_type = JAL;
						decoded_instr.opcode = coded_instr[31:26];
						decoded_instr.address = coded_instr[25:0];
					end
				6'b000100,
				6'b001000:
					begin
						if(coded_instr[31:26] == 6'b000100)
							decoded_instr.inst_type = BEQ;
						else
							decoded_instr.inst_type = ADDI;
						decoded_instr.opcode = coded_instr[31:26];
						decoded_instr.s = coded_instr[25:21];
						decoded_instr.t = coded_instr[20:16];
						decoded_instr.imm = coded_instr[15:0];
					end
				6'b000111:
					begin
						decoded_instr.inst_type = JR;
						decoded_instr.opcode = coded_instr[31:26];
						decoded_instr.s = coded_instr[25:21];
					end
		/*				
				6'b100011:
					begin
						decoded_instr.inst_type = LW;
						decoded_instr.s = coded_instr[25:21];
						decoded_instr.t = coded_instr[20:16];
						decoded_instr.imm = coded_instr[15:0];
					end
				6'b101011:
					begin
						decoded_instr.inst_type = SW;
						decoded_instr.s = coded_instr[25:21];
						decoded_instr.t = coded_instr[20:16];
						decoded_instr.imm = coded_instr[15:0];
					end
		*/
			endcase
		end

	endtask : decode_instruction

	task do_instruction;
		input logic [31:0] coded_instr;
		begin
			decode_instruction(coded_instr);
			$display("*|* Instruction: %x",coded_instr);
			decoded_instr.print();
			cpu_if.IF_INSTRUCTION = coded_instr;
		end
	endtask : do_instruction


	task fill_all_registers();
	begin	
		$display("FILL ALL REGISTERS!");
		for (i = 0; i < 32; i = i + 1) begin
			assert (instr.randomize() with {instr.inst_type == ADDI; instr.t == i; instr.s == 5'b00000;});
			RAM[i] = instruction_builder(instr);
		end

	end	
	endtask : fill_all_registers

	task generate_loads_only();
		$display("GENERATE LOADS ONLY!");
		for (i = 0; i < RAM_LEN; i = i + 1) begin
			assert (instr.randomize() with {instr.inst_type == ADDI; instr.s == 5'b00000;});
			RAM[i] = instruction_builder(instr);
		end
	endtask : generate_loads_only

	task generate_instructions();
		$display("GENERATE INSTRUCTIONS");
		for (i = 0; i < RAM_LEN; i = i + 1) begin
			assert (instr.randomize() with {instr.inst_type != J;});
			RAM[i] = instruction_builder(instr);
		end
	endtask : generate_instructions

	task generate_jumps_only();
		$display("GENERATE JUMPS ONLY");
		for (i = 0; i < RAM_LEN; i = i + 1) begin
			assert (instr.randomize() with {instr.inst_type == J;});
			RAM[i] = instruction_builder(instr);
		end	
	endtask : generate_jumps_only

	task generate_loop;
		input integer start, len, cycle_reg, control_reg;
		integer k;
		begin 
			if(len >= 4 && control_reg != cycle_reg && control_reg != 5'b00000 && start + len < (1 << MEM_WIDTH))
			begin 
				assert (instr.randomize() with {instr.inst_type == ADDI; instr.s == 5'b00000; instr.t == cycle_reg; instr.imm < 10; instr.imm > 1;});
				RAM[start] = instruction_builder(instr);

				for (k = start + 1; k < start + len - 3; k = k + 1) begin
					assert (instr.randomize() with {instr.inst_type == ADD; instr.s == 5'b00000; instr.t == 5'b00000; instr.d == 5'b0000;});
					RAM[k] = instruction_builder(instr);
				end
				assert (instr.randomize() with {instr.inst_type == ADDI; instr.s == cycle_reg; instr.t == cycle_reg; instr.imm == -1;});
				RAM[start + len - 3] = instruction_builder(instr);

				assert (instr.randomize() with {instr.inst_type == SLT; instr.d == control_reg; instr.t == 5'b0000; instr.s == cycle_reg;});
				RAM[start + len - 2] = instruction_builder(instr);

				assert (instr.randomize() with {instr.inst_type == BEQ; instr.s == control_reg; instr.t == 5'b0000; instr.imm == (1 - len);});
				RAM[start + len - 1] = instruction_builder(instr);
			end	
		end
	endtask : generate_loop

	task generate_alu_only();
		$display("GENERATE ALU ONLY!");
		fill_all_registers();
		for (i = 32; i < RAM_LEN; i = i + 1) begin
			assert (instr.randomize() with {instr.inst_type inside {ADD, SUB, AND, OR, SLT, ADDI, ADDU_QB, ADDU_S_QB, SLLV, SRLV, SRAV};});
			RAM[i] = instruction_builder(instr);
		end
	endtask : generate_alu_only


	initial
	begin
		instr = new();
		decoded_instr = new();			
		// ADDI only
		generate_loads_only();
		run_test();


		// ADDI + ALU only
		generate_alu_only();
		run_test();

		$finish;
		// ADDI + ALU + loops only
		generate_alu_only();
		// generate_loop(0, 5, 4, 1);
		run_test();

		// special loops (beq on PC=0)
		assert (instr.randomize() with {instr.inst_type == BEQ; instr.s == 5'b00000; instr.t == 5'b0000; instr.imm == -1;});
		RAM[0] = instruction_builder(instr);
		run_test();

		// J only
		//generate_jumps_only(); // muj procesor nezna J
		//run_test();

		// ALL instruction
		generate_instructions();
		run_test();


		#100;
		$display("Every test passed.");
		$finish;
	end


	task run_test();
		
		begin
			reseting();
			assert (instr.randomize() with {instr.inst_type == BEQ; instr.s == 5'b00000; instr.t == 5'b0000; instr.imm == -1;});
			RAM[((1<<MEM_WIDTH) - 1)] = instruction_builder(instr);
			while(timeout == 0)
			begin
				$display("Program Counter: %x", cpu_if.IF_PC);
				$display("%d. Instruction: %x", (cpu_if.IF_PC >> 2), RAM[(cpu_if.IF_PC >> 2)]);

				do_instruction(RAM[(cpu_if.IF_PC >> 2)]);
				decode_instruction(RAM[(cpu_if.IF_PC >> 2)]);
				
				wait_or_timeout();

				if(timeout != 1)
				begin 
					#1;
					if(decoded_instr.inst_type inside {ADD, SUB, AND, OR, SLT, ADDI, ADDU_QB, ADDU_S_QB, SLLV, SRLV, SRAV, JAL})
					begin
						$display("REGISTER CHANGING INSTRUCTION");
						change_shadow_copy();
					end

					check_shadow_copy();
				end

					
			end

			if((cpu_if.IF_PC >> 2) != ((1<<MEM_WIDTH) - 1))
			begin 
				$display("PROGRAM DIDNT FINISH ON LAST ADDRESS, TERMINATING TEST!");
				$display("LAST INSTRUCTION:");
				decoded_instr.print();
				$finish;
			end
		end
	endtask

	task change_shadow_copy ();
		logic [31:0] s,t;
		begin
			s = REG_FIELD[decoded_instr.s];
			t = REG_FIELD[decoded_instr.t];
			case (decoded_instr.inst_type)
				ADD			: REG_FIELD[decoded_instr.d] = s + t;
				SUB			: REG_FIELD[decoded_instr.d] = s - t;
				AND			: REG_FIELD[decoded_instr.d] = s & t;
				OR			: REG_FIELD[decoded_instr.d] = s | t;
				SLT			: REG_FIELD[decoded_instr.d] = ($signed(s) < $signed(t)) ? 1'b1 : 1'b0;
				ADDU_QB		: REG_FIELD[decoded_instr.d] = {s[31:24] + t[31:24], s[23:16] + t[23:16], s[15:8] + t[15:8], s[7:0] + t[7:0]};
				ADDU_S_QB	: REG_FIELD[decoded_instr.d] = {SatSum(s[31:24], t[31:24]), SatSum(s[23:16], t[23:16]), SatSum(s[15:8], t[15:8]), SatSum(s[7:0], t[7:0])};
				SLLV		: REG_FIELD[decoded_instr.d] = REG_FIELD[decoded_instr.t] << REG_FIELD[decoded_instr.s];
				SRLV		: REG_FIELD[decoded_instr.d] = REG_FIELD[decoded_instr.t] >> REG_FIELD[decoded_instr.s];
				SRAV		: REG_FIELD[decoded_instr.d] = $signed(REG_FIELD[decoded_instr.t]) >>> REG_FIELD[decoded_instr.s];
				ADDI		: REG_FIELD[decoded_instr.t] = REG_FIELD[decoded_instr.s] + 32'(signed'(decoded_instr.imm));
				JAL			: REG_FIELD[decoded_instr.d] = cpu_if.IF_PC + 4;
			endcase
			REG_FIELD[0]=32'h00000000;
		end
	endtask

	task check_shadow_copy();
		integer k;
		begin
			for(k = 0; k < 32; k = k + 1)
			begin
				if(REG_FIELD[k] != $root.top._cpu.CPU.regFile.rf[k])
				begin 
					$display("REGISTER %d DIFFERS FROM SHADOW COPY", k);
					//$finish;
				end
			end
			//$display("EVERYTHING IS OK");
		end
	endtask


	task wait_or_timeout();
		begin
			fork : wait_or_timeout_f
				begin
					//$display($time,"\tProcess-1 of fork-1 Started");
					#100;
					timeout = 1; 
					//$display($time,"\tProcess-1 of fork-1 ENDED");
					disable wait_or_timeout_f;
				end
				begin
					//$display($time,"\tProcess-2 of fork-1 Started");
					
					@(cpu_if.IF_PC);
					timeout = 0;
					//$display($time,"\tProcess-2 of fork-1 ENDED");
					disable wait_or_timeout_f;
				end
			join
			
		end
	endtask

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


endmodule