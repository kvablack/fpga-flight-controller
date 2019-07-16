module lpf(
	input logic clk,
	input logic rst,
	input logic data_ready,
	input logic signed[15:0] value,
	output logic signed[15:0] filtered
);
	parameter signed ALPHA = 14; // alpha * 32 (5 bits to right of binary point)
	localparam signed q = $signed(1 <<< 5) - ALPHA;

	logic signed[15:0] prev_filtered;
	
	wire signed[20:0] term_a = ALPHA * value;
	wire signed[20:0] term_b = q * prev_filtered;
	wire signed[20:0] result_full = term_a - term_b;
	wire signed[15:0] result_rounded = result_full >>> 5; // truncate to integer
	always @(posedge clk) begin
		if (rst) begin
			prev_filtered <= 0;
		end else if (data_ready) begin
			prev_filtered <= result_rounded;
		end
	end
	
	assign filtered = prev_filtered;

endmodule
