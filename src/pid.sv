module pid(
	input logic clk,
	input logic rst,
	input logic data_ready,
	input logic signed[15:0] gyro,
	input logic[11:0] stick,
	output logic signed[12:0] power
);

	parameter signed P = 1; // deg/s -> power
	parameter signed D = 0;
	parameter signed I = 0;
	parameter signed FS = 1000; // full scale setting of gyro
	parameter signed MAX = 400; // maximum value of stick in deg/s 
	parameter signed MAX_ITERM = 1024 <<< 18;
	localparam signed max_scaled = MAX * 262144 / 250000;
	
	wire signed[60:0] scaled_gyro = gyro * FS; // 15 bits to right of binary point
	wire signed[63:0] scaled_gyro_aligned = {scaled_gyro, {3{1'b0}}}; // 18 bits to right of binary point
	
	wire signed[11:0] shifted_stick = stick - 500;
	wire signed[63:0] squared_stick = shifted_stick < 0 ? (-shifted_stick * shifted_stick) : (shifted_stick * shifted_stick);
	wire signed[63:0] scaled_stick = squared_stick * max_scaled; // 18 bits to right of binary point
	
	wire signed[63:0] diff = scaled_stick - scaled_gyro_aligned; // in deg/s, 18 bits to right of binary point
	
	logic signed[63:0] prevValues[0:7];
	logic[2:0] prevValuePtr;
	logic signed[63:0] accumulator;
	wire signed[63:0] nextAcc = accumulator + diff;
	always @(posedge clk) begin
		if (rst) begin
			prevValues <= '{default:0};
			prevValuePtr <= 0;
		end else if (data_ready) begin
			prevValues[prevValuePtr] <= diff;
			prevValuePtr <= prevValuePtr + 1;
			accumulator <= nextAcc;
		end
	end
	
	wire signed[63:0] delta = prevValues[prevValuePtr - 1] - prevValues[prevValuePtr];
	wire signed[63:0] iterm = accumulator * I;
	wire signed[63:0] capped_iterm = iterm > MAX_ITERM ? MAX_ITERM : (iterm < -MAX_ITERM ? -MAX_ITERM : iterm);
	wire signed[48:0] result = (diff * P) - (delta * D) + capped_iterm; // 36 bits to the right of the binary point
	assign power = result[48:36]; // truncated to integer
endmodule
