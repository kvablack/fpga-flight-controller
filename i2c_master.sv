module i2c_master(
	input logic clk,
	input logic rst,
	
	/* I2C pins, use like this:
	 * 	assign scl_i = scl_pin;
	 * 	assign scl_pin = scl_o ? 1'bz : 1'b0; 
	 */
	 input logic scl_i,
	 input logic sda_i,
	 output logic scl_o,
	 output logic sda_o,
	 
	 // input interface
	 /* command | required inputs:
	  * 000: nothing (idle) | none
	  *	001: start read | slave_addr
	  *	010: start write | slave_addr
	  *	011: write data | data_in
	  *	100: read data | done_reading
	  * 101: stop | none
	  */
	 input logic[2:0] cmd,
	 input logic[6:0] slave_addr,
	 input logic[7:0] data_in,
	 input logic done_reading,
	 
	 // state interface
	 /* state | valid outputs | valid commands:
	  *	00: idle | none | start read, start write
	  *	01: busy | none | none
	  *	10: ready (slave acked) | data_out (if prev was read) | stop, start read, start write, read data, write data
	  * 11: error (slave nack) | none | stop, start read, start write
	  */
	  output logic[1:0] state,
	  output logic[7:0] data_out
);
	parameter clkdiv = 500;
	
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
	 
	enum logic[3:0] {
		IDLE,
		START_1,
		START_2,
		START_3,
		WAITING_ACK,
		GIVING_ACK,
		MISSED_ACK,
		PRE_READY,
		READY,
		PRE_WRITE,
		ACTIVE_WRITE,
		ACTIVE_READ,
		STOP
	} state_reg;

	
	logic divclk;
	logic[15:0] delay_counter;
	logic[2:0] bit_index;
	logic rw_bit;
	logic done_reading_reg;
	
	logic[6:0] slave_addr_reg;
	logic[7:0] read_reg;
	assign data_out = read_reg;
	logic[7:0] write_reg;
	
	
	always @(posedge clk) begin
		if (rst) begin
			state_reg <= IDLE;
			delay_counter <= 0;
			divclk <= 0;
		end
		
		// clock scl, paying attention to stretching
		else if (delay_counter < (clkdiv >> 1)) begin
			divclk <= 0;
			delay_counter <= delay_counter + 1;
		end else if (delay_counter < clkdiv - 1) begin
			divclk <= 1;
			if (scl_i || PRE_WRITE) delay_counter <= delay_counter + 1;
		end else begin
			delay_counter <= 0;
		end
		
		// process all commands
		if ((state_reg == IDLE || state_reg == READY || state_reg == MISSED_ACK) && (cmd == CMD_START_READ || cmd == CMD_START_WRITE)) begin
			slave_addr_reg <= slave_addr;
			state_reg <= START_1;
			rw_bit <= cmd[0];
		end else if (state_reg == READY && cmd == CMD_READ_DATA) begin
			state_reg <= ACTIVE_READ;
			done_reading_reg <= done_reading;
			bit_index <= 0;
		end else if (state_reg == READY && cmd == CMD_WRITE_DATA) begin
			state_reg <= PRE_WRITE;
			write_reg <= data_in;
			bit_index <= 0;
		end else if ((state_reg == READY || state_reg == MISSED_ACK) && cmd == CMD_STOP) begin
			state_reg <= STOP;
		end
		
		// state transitions on positive edge of scl
		else if (delay_counter == clkdiv >> 1) begin
			case (state_reg)
				ACTIVE_READ: read_reg[7 - bit_index] <= sda_i;
				WAITING_ACK: begin
					if (sda_i) begin
						state_reg <= MISSED_ACK;
					end else begin
						state_reg <= PRE_READY;
					end
				end
			endcase
		end
		
		// state transitions on middle of scl high
		else if (delay_counter == clkdiv - (clkdiv >> 2)) begin
			if (state_reg == START_1) begin
				state_reg <= START_2;
				bit_index <= 0;
			end else if (state_reg == STOP) begin
				state_reg <= IDLE;
			end
		end
		
		// state transitions on middle of scl low
		else if (delay_counter == clkdiv >> 2) begin
			case (state_reg)
				START_2: begin
					state_reg <= START_3;
					bit_index <= 0;
				end START_3, ACTIVE_WRITE: begin
					if (bit_index < 7) begin
						bit_index <= bit_index + 1;
					end else begin
						state_reg <= WAITING_ACK;
					end
				end ACTIVE_READ: begin
					if (bit_index < 7) begin
						bit_index <= bit_index + 1;
					end else begin
						state_reg <= GIVING_ACK;
					end
				end GIVING_ACK: begin
					state_reg <= READY;
				end
				PRE_WRITE: state_reg <= ACTIVE_WRITE;
				PRE_READY: state_reg <= READY;
			endcase
		end
	end
	
	// FSM combinational output logic
	always @* begin
		case (state_reg)
			IDLE: sda_o = 1;
			START_1: sda_o = 1;
			START_2: sda_o = 0;
			START_3: sda_o = bit_index == 7 ? rw_bit : slave_addr_reg[6 - bit_index];
			WAITING_ACK: sda_o = 1;
			GIVING_ACK: sda_o = done_reading_reg;
			READY: sda_o = 1;
			PRE_READY: sda_o = 1;
			PRE_WRITE, ACTIVE_WRITE: sda_o = write_reg[7 - bit_index];
			ACTIVE_READ: sda_o = 1;
			STOP: sda_o = 0;
			MISSED_ACK: sda_o = 1;
		endcase
		
		case (state_reg)
			IDLE: state = STATE_IDLE;
			READY: state = STATE_READY;
			MISSED_ACK: state = STATE_ERROR;
			default: state = STATE_BUSY;
		endcase
		
		case (state_reg)
			READY, MISSED_ACK, PRE_WRITE: scl_o = 0;
			IDLE, START_1: scl_o = 1;
			default: scl_o = divclk;
		endcase
	end
	
endmodule
