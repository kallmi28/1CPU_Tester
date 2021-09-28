`timescale 1ns / 1ps
module wrapper_CPU (CPU_IF.CPU_IN cpu_if);

	// module processor ( input clk, reset, output [31:0] PC, input [31:0] instruction, output WE, output [31:0] address_to_mem, output [31:0] data_to_mem, input [31:0] data_from_mem);
	processor CPU (.clk(cpu_if.clk), .reset(cpu_if.IF_RESET),.PC(cpu_if.IF_PC), .instruction(cpu_if.IF_INSTRUCTION), .WE(cpu_if.IF_WE), .address_to_mem(cpu_if.IF_ADDR_TO_MEM), .data_to_mem(cpu_if.IF_DATA_TO_MEM), .data_from_mem(cpu_if.IF_DATA_FROM_MEM));

endmodule

interface CPU_IF (input clk);

	logic [31:0] IF_PC, IF_INSTRUCTION;
	logic IF_WE, IF_RESET;
	logic [31:0] IF_ADDR_TO_MEM, IF_DATA_TO_MEM, IF_DATA_FROM_MEM;

	modport CPU_IN 		(input clk, IF_RESET, IF_INSTRUCTION, IF_DATA_FROM_MEM, output IF_PC, IF_WE, IF_ADDR_TO_MEM, IF_DATA_TO_MEM);
	//modport DRIVER_IN 	(input clk, IF_instruction, IF_DATA_FROM_MEM, output IF_RESET, IF_PC, IF_WE, IF_ADDR_TO_MEM, IF_DATA_TO_MEM);
	modport DRIVER_IN 	(input clk, IF_PC, output IF_RESET, IF_INSTRUCTION);

endinterface : CPU_IF