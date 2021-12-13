`include "single.v"
module testbench;
	reg clk;
	reg reset;
	wire [31:0] WriteData;
	wire [31:0] DataAdr;
	wire MemWrite;
	top dut(
		.clk(clk),
		.reset(reset),
		.WriteData(WriteData),
		.DataAdr(DataAdr),
		.MemWrite(MemWrite)
	);
	initial begin
		reset <= 1;
		#(22);
		reset <= 0;
	end
	always begin
		clk <= 1;
		#(5);
		clk <= 0;
		#(5);
	end
	always @(negedge clk) begin
		if (MemWrite) begin
			if ((DataAdr === 100) & (WriteData === 7)) begin
				$display("Simulation succeeded");
				$finish;
			end
			else if (DataAdr !== 96) begin
				$display("Simulation failed");
				$finish;
			end
		end
	end

	initial begin
		$dumpfile("arm.vcd");
		$dumpvars;
	end
endmodule