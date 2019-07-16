module pid_test(
	input logic signed[15:0] gyro,
	input logic[11:0] stick,
	output logic signed[12:0] power
);

	pid(.gyro, .stick, .power);
	
endmodule