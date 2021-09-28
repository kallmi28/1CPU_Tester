`timescale 1ns / 1ps
module top ();

	logic clk;

	always
	begin
		clk = 1'b1;
		#5;
		clk = 1'b0;
		#5;
	end

	CPU_IF _cpu_drv_if (clk);

	// module wrapper_CPU (CPU_IF.CPU_IN cpu_if);
	wrapper_CPU _cpu (_cpu_drv_if);

	// module driver (CPU_IF.DRIVER_IN cpu_if);
	driver _drv (_cpu_drv_if);



endmodule