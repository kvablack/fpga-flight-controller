module pwm_encoder(clk, rst, val, pwm);

input logic clk;
input logic rst;
input logic [11:0] val [0:3];
output logic [3:0] pwm = 4'b0;

logic [63:0] micros = 63'b0;
logic [63:0] t0 = 63'b0;
wire [63:0] dt = micros - t0;

always @(posedge clk) begin
	if(rst) begin
		pwm <= 4'b0;
		micros <= 63'b0;
		t0 <= 63'b0;
	end else begin
		if(dt > 2083) begin
			t0 <= micros;
			pwm <= 4'b1111;
		end else begin
			if(dt > val[0] + 1000) begin
				pwm[0] <= 1'b0;
			end			
			if(dt > val[1] + 1000) begin
				pwm[1] <= 1'b0;
			end
			if(dt > val[2] + 1000) begin
				pwm[2] <= 1'b0;
			end
			if(dt > val[3] + 1000) begin
				pwm[3] <= 1'b0;
			end
		end
		micros <= micros + 1;
	end
end

endmodule
