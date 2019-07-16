module mpu6050_driver(
	input logic clk,
	input logic rst,
	
	input logic mpu6050_interrupt,
	/* I2C pins, use like this:
	 * 	assign scl_i = scl_pin;
	 * 	assign scl_pin = scl_o ? 1'bz : 1'b0; 
	 */
	 input logic scl_i,
	 input logic sda_i,
	 output logic scl_o,
	 output logic sda_o,
	 
	 output logic signed[15:0] gyro_xout,
	 output logic signed[15:0] gyro_yout,
	 output logic signed[15:0] gyro_zout,
	 output logic booting,
	 output logic data_ready // pulses when gyro outputs are valid with new values
);
	parameter slave_addr = 7'b1101000;
	parameter boot_delay = 50000000; // 1 second
	
	parameter CMD_IDLE = 3'b000;
	parameter CMD_START_READ = 3'b001;
	parameter CMD_START_WRITE = 3'b010;
	parameter CMD_WRITE_DATA = 3'b011;
	parameter CMD_READ_DATA = 3'b100;
	parameter CMD_STOP = 3'b101;
	parameter STATE_IDLE = 2'b00;
	parameter STATE_BUSY = 2'b01;
	parameter STATE_READY = 2'b10;
	parameter STATE_ERROR = 2'b11;
	
	localparam REG_GYRO = 8'd67;
	localparam REG_INT_PIN_CFG = 8'd55;
	localparam REG_INT_ENABLE = 8'd56;
	localparam REG_INT_STATUS = 8'd58;
	localparam REG_PWR_MGMNT_1 = 8'd107;
	localparam REG_GYRO_CONFIG = 8'd27;
	
	logic[2:0] cmd;
	logic[7:0] data_in;
	logic done_reading;
	
	logic[1:0] master_state;
	logic[7:0] data_out;

	i2c_master(.clk, .rst, .scl_i, .sda_i, .scl_o, .sda_o, .cmd, .slave_addr(slave_addr), .data_in, .done_reading, .state(master_state), .data_out);
		
	enum logic[2:0]{
		BOOT,
		CONFIG1,
		CONFIG2,
		CONFIG3,
		CONFIG4,
		MAIN_READING,
		MAIN_DONE,
		RECOVERING_ERROR
	} state;
	
	assign booting = state == BOOT;
	assign data_ready = state == MAIN_DONE;
	
	enum logic[2:0]{
		WRITE_WSTART,
		WRITE_WADDR,
		WRITE_WDATA,
		WRITE_STOP,
		WRITE_DONE
	} write_state;
	
	enum logic[2:0]{
		READ_WSTART,
		READ_WADDR,
		READ_RSTART,
		READ_RDATA,
		READ_STOP,
		READ_DONE
	} read_state;
	logic[3:0] read_rdata_index;
		
	logic[31:0] boot_delay_counter;
	logic[7:0] reg_addr;
	logic[7:0] write_reg_data;
	
	
	always @(posedge clk) begin
		if (rst) begin
			state <= BOOT;
			boot_delay_counter <= 0;
			write_state <= WRITE_DONE;
			read_state <= READ_DONE;
			cmd <= CMD_STOP;
		end else if (cmd != CMD_IDLE) cmd <= CMD_IDLE; // only pulse each command for 1 cycle
		
		else begin
			// primary state transitions
			if (state == BOOT) begin
				if (boot_delay_counter < boot_delay) begin
					boot_delay_counter <= boot_delay_counter + 1;
				end else begin
					state <= CONFIG1;
					write_state <= WRITE_WSTART;
					cmd <= CMD_START_WRITE;
					reg_addr <= REG_INT_ENABLE;
					write_reg_data <= 8'b00000001; // enable data ready interrupt
				end
			end else if (master_state == STATE_ERROR) begin
				state <= RECOVERING_ERROR;
				cmd <= CMD_STOP;
				write_state <= WRITE_DONE;
				read_state <= READ_DONE;
			end else if (state == CONFIG1 && write_state == WRITE_DONE) begin
				state <= CONFIG2;
				write_state <= WRITE_WSTART;
				cmd <= CMD_START_WRITE;
				reg_addr <= REG_INT_PIN_CFG;
				write_reg_data <= 8'b00110000; // enable int latch and any read clears
			end else if (state == CONFIG2 && write_state == WRITE_DONE) begin
				state <= CONFIG3;
				write_state <= WRITE_WSTART;
				cmd <= CMD_START_WRITE;
				reg_addr <= REG_PWR_MGMNT_1;
				write_reg_data <= 8'b00000001; // select x axis gyroscope reference for clock
			end else if (state == CONFIG3 && write_state == WRITE_DONE) begin
				state <= CONFIG4;
				write_state <= WRITE_WSTART;
				cmd <= CMD_START_WRITE;
				reg_addr <= REG_GYRO_CONFIG;
				write_reg_data <= 8'b00010000; // select 0b10 FS (1000 deg/s)
			end else if (state == CONFIG4 && write_state == WRITE_DONE) begin
				state <= MAIN_READING;
				read_state <= READ_WSTART;
				cmd <= CMD_START_WRITE;
				reg_addr <= REG_GYRO;
			end else if (state == MAIN_READING && read_state == READ_DONE) begin
				state <= MAIN_DONE;
			end else if (state == MAIN_DONE/* && mpu6050_interrupt*/) begin
				state <= MAIN_READING;
				read_state <= READ_WSTART;
				cmd <= CMD_START_WRITE;
				reg_addr <= REG_GYRO;
			end else if (state == RECOVERING_ERROR && master_state == STATE_IDLE) begin
				state <= CONFIG1;
				write_state <= WRITE_WSTART;
				cmd <= CMD_START_WRITE;
				reg_addr <= REG_INT_ENABLE;
				write_reg_data <= 8'b00000001; // enable data ready interrupt
			end
			
			// write cycle state transitions
			else if (write_state == WRITE_WSTART && master_state == STATE_READY) begin
				cmd <= CMD_WRITE_DATA;
				data_in <= reg_addr;
				write_state <= WRITE_WADDR;
			end else if (write_state == WRITE_WADDR && master_state == STATE_READY) begin
				cmd <= CMD_WRITE_DATA;
				data_in <= write_reg_data;
				write_state <= WRITE_WDATA;
			end else if (write_state == WRITE_WDATA && master_state == STATE_READY) begin
				cmd <= CMD_STOP;
				write_state <= WRITE_STOP;
			end else if (write_state == WRITE_STOP && master_state == STATE_IDLE) begin
				write_state <= WRITE_DONE;
			end
			
			// read cycle state transitions
			else if (read_state == READ_WSTART && master_state == STATE_READY) begin
				cmd <= CMD_WRITE_DATA;
				data_in <= reg_addr;
				read_state <= READ_WADDR;
			end else if (read_state == READ_WADDR && master_state == STATE_READY) begin
				cmd <= CMD_START_READ;
				read_state <= READ_RSTART;
			end else if (read_state == READ_RSTART && master_state == STATE_READY) begin
				cmd <= CMD_READ_DATA;
				read_state <= READ_RDATA;
				done_reading <= 0;
				read_rdata_index <= 0;
			end else if (read_state == READ_RDATA && master_state == STATE_READY) begin
				case (read_rdata_index)
					0: gyro_xout[15:8] <= data_out;
					1: gyro_xout[7:0] <= data_out;
					2: gyro_yout[15:8] <= data_out;
					3: gyro_yout[7:0] <= data_out;
					4: gyro_zout[15:8] <= data_out;
					5: gyro_zout[7:0] <= data_out;
				endcase
				
				if (read_rdata_index == 4) done_reading <= 1;
				
				if (read_rdata_index < 5) begin
					read_rdata_index <= read_rdata_index + 1;
					cmd <= CMD_READ_DATA;
				end else begin
					cmd <= CMD_STOP;
					read_state <= READ_STOP;
				end
			end else if (read_state == READ_STOP && master_state == STATE_IDLE) begin
				read_state <= READ_DONE;
			end
		end
		
	end

endmodule
