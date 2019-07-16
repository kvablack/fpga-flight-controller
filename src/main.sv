module pb(
	input logic CLK, // 50 MHz
	input logic BTN,
	input logic INTERRUPT_PIN,
	inout logic SCL_PIN,
	inout logic SDA_PIN,
	input logic PPM_PIN,
	output logic[3:0] ESC_PINS,
	output logic[2:0] LEDS
);
	localparam signed MOTOR_MIN = 190;
	localparam signed MOTOR_MAX = 600;
	
	wire rst = ~BTN;
	logic led1, led2, led3;
	assign LEDS[0] = ~led1;
	assign LEDS[1] = ~led2;
	assign LEDS[2] = ~led3;
	
	logic divclk;
	logic[5:0] divclk_counter;
	
	logic scl_i;
	logic sda_i;
	logic scl_o;
	logic sda_o;
    // open drain I2C pins
	assign scl_i = SCL_PIN;
	assign SCL_PIN = scl_o ? 1'bz : 1'b0; 
	assign sda_i = SDA_PIN;
	assign SDA_PIN = sda_o ? 1'bz : 1'b0;
	
	logic signed[15:0] gyro_xout;
	logic signed[15:0] gyro_yout;
	logic signed[15:0] gyro_zout;
	logic booting;
	logic data_ready;
	mpu6050_driver(.clk(CLK), .rst, .mpu6050_interrupt(INTERRUPT_PIN), .scl_i, .sda_i, .scl_o, .sda_o, .gyro_xout, .gyro_yout, .gyro_zout, .data_ready, .booting);

	logic[11:0] ch[0:5];
	ppm_decoder decoder(divclk, rst, PPM_PIN, ch);
	
	wire[11:0] lx = ch[3]; 
   wire[11:0] ly = ch[2];
   wire[11:0] rx = ch[0];
   wire[11:0] ry = ch[1];
	wire[11:0] arm = ch[5];
	
	logic armed = 0;
	
    // scale throttle between min and max
	wire signed[12:0] throttle = (ly * (MOTOR_MAX - MOTOR_MIN) / 1000) + MOTOR_MIN;
	
	logic signed[12:0] rollNet;
	logic signed[12:0] pitchNet;
	logic signed[12:0] yawNet;
	pid #(.P(4 <<< 18), .D(160 <<< 18), .I('b1100010010011), .MAX(300)) roll(.gyro(x_filtered), .stick(rx), .power(rollNet));
	pid #(.P(4 <<< 18), .D(160 <<< 18), .I('b1100010010011), .MAX(300)) pitch(.gyro(y_filtered), .stick(ry), .power(pitchNet));
	pid #(.P(4 <<< 18), .D(160 <<< 18), .I('b1100010010011), .MAX(300)) yaw(.gyro(z_filtered * -1), .stick(lx), .power(yawNet));
	
	// motor mixing
	wire signed[15:0] fl = throttle +rollNet -pitchNet -yawNet;
   wire signed[15:0] fr = throttle -rollNet -pitchNet +yawNet;
   wire signed[15:0] bl = throttle +rollNet +pitchNet +yawNet;
   wire signed[15:0] br = throttle -rollNet +pitchNet -yawNet;
	
   logic[11:0] motorOutput [0:3];
	
	// arm safety logic and cap motor values from overflow
   assign motorOutput[0] = !armed ? 0 : (br < MOTOR_MIN ? MOTOR_MIN : (br > MOTOR_MAX ? MOTOR_MAX : br));
	assign motorOutput[1] = !armed ? 0 : (fr < MOTOR_MIN ? MOTOR_MIN : (fr > MOTOR_MAX ? MOTOR_MAX : fr));
	assign motorOutput[2] = !armed ? 0 : (fl < MOTOR_MIN ? MOTOR_MIN : (fl > MOTOR_MAX ? MOTOR_MAX : fl));
   assign motorOutput[3] = !armed ? 0 : (bl < MOTOR_MIN ? MOTOR_MIN : (bl > MOTOR_MAX ? MOTOR_MAX : bl));
	
	pwm_encoder encoder(divclk, rst, motorOutput, ESC_PINS);

	logic signed[15:0] x_filtered;
	logic signed[15:0] y_filtered;
	logic signed[15:0] z_filtered;
	lpf x_lpf(.clk(CLK), .rst, .data_ready, .value(gyro_xout), .filtered(x_filtered));
	lpf y_lpf(.clk(CLK), .rst, .data_ready, .value(gyro_yout), .filtered(y_filtered));
	lpf z_lpf(.clk(CLK), .rst, .data_ready, .value(gyro_zout), .filtered(z_filtered));
	
	logic[11:0] prev_arm;
	always @(posedge CLK) begin
		if (rst) begin
			divclk_counter <= 0;
			armed <= 0;
			prev_arm <= 0;
		end else begin
            // drive ppm and pwm with a divided clock
			if (divclk_counter < 25) begin
				divclk = 0;
				divclk_counter <= divclk_counter + 1;
			end else if (divclk_counter < 49) begin
				divclk = 1;
				divclk_counter <= divclk_counter + 1;
			end else begin
				divclk_counter <= 0;
			end
			
            // arm posedge detector
			prev_arm <= arm;
			if (arm > 500 && prev_arm < 500) begin
				if (ly < 50) armed <= 1;
			end
			if (arm < 500) armed <= 0;
		end
	end
	
	parameter signed cutoff = 3276;
	assign led1 = x_filtered > cutoff;
	assign led2 = x_filtered < -cutoff;
	assign led3 = booting;
endmodule

