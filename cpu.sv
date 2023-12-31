module cpu_fib_tb ();
	localparam CLK_PERIOD = 10;
	
	logic clk, reset;
	logic [7:0] memory [0:15];
	logic [0:15] control_signals;
	logic [7:0] out;
	
	cpu dut (clk, reset, memory, control_signals, out);
	
	initial begin
		clk = 0;
		forever #CLK_PERIOD clk = ~clk;
	end

	initial begin
		memory = {8'b01010001, 8'b01001110, 8'b01010000, 8'b01001111, 8'b11100000, 8'b00011110, 8'b00101111, 8'b01001110, 8'b11100000, 8'b00011111, 8'b00101110, 8'b01111101, 8'b01100011, 8'b11110000, 8'b0, 8'b0};
		reset = 1;
		#100;
		reset = 0;
		#10000;
		$stop;
	end
endmodule

module cpu (
	input logic clk,
	input logic reset,
	input logic [7:0] memory [0:15],
	output logic [0:15] control_signals, // output the control signals for fun (blinking leds look cool)
	output logic [7:0] out
);
	parameter FLAG_C = 1;
	parameter FLAG_Z = 0;
	// Define instructions
	parameter OP_NOP = 4'b0000;
	parameter OP_LDA = 4'b0001;
	parameter OP_ADD = 4'b0010;
	parameter OP_SUB = 4'b0011;
	parameter OP_STA = 4'b0100;
	parameter OP_LDI = 4'b0101;
	parameter OP_JMP = 4'b0110;
	parameter OP_JC  = 4'b0111;
	parameter OP_JZ  = 4'b1000;
	parameter OP_OUT = 4'b1110;
	parameter OP_HLT = 4'b1111;
	
	logic [3:0] pc;
	logic [2:0] stage;
	logic [7:0] ir;
	logic [3:0] mar;
	logic [7:0] bus;
	logic [7:0] mem[16];
	logic [7:0] a_reg;
	logic [7:0] b_reg;
	logic [7:0] b_reg_out;
	logic [8:0] alu;
	logic [1:0] flags;
	
	logic ctrl_co, ctrl_ri, ctrl_mi, ctrl_io, ctrl_ht, ctrl_ao, ctrl_fi, ctrl_ro, ctrl_bi, ctrl_ii, ctrl_ai, ctrl_su, ctrl_eo, ctrl_oi, ctrl_ce, ctrl_jp, flag_z, flag_c;
	
	assign control_signals = {ctrl_ht, ctrl_mi, ctrl_ri, ctrl_ro, ctrl_io, ctrl_ii, ctrl_ai, ctrl_ao, ctrl_eo, ctrl_su, ctrl_bi, ctrl_oi, ctrl_ce, ctrl_co, ctrl_jp, ctrl_fi};
	
	// Bus
	always_comb begin
		bus = ctrl_co ? pc :
			  ctrl_ro ? mem[mar]:
			  ctrl_io ? ir[3:0] :
			  ctrl_ao ? a_reg :
			  ctrl_eo ? alu :
			  8'b0;
	end
	
	// Program Counter
	always_ff @(posedge clk or posedge reset) begin
		if (reset)
			pc <= 0;
		else if (ctrl_ce)
			pc <= pc + 1;
		else if (ctrl_jp)
			pc <= bus[3:0];
	end
	
	// Instruction Step Counter
	always_ff @(posedge clk or posedge reset) begin
		if (reset)
			stage <= 0;
		else if (stage == 5 || ctrl_jp)
			stage <= 0;
		else if (ctrl_ht || stage == 6)
			stage <= 6;
		else
			stage <= stage + 1;
	end
	
	// Instruction Register
	always_ff @(posedge clk or posedge reset) begin
		if (reset)
			ir <= 0;
		else if (ctrl_ii)
			ir <= bus;
	end
	
	// Memory Address Register
	always_ff @(posedge clk or posedge reset) begin
		if (reset)
			mar <= 0;
		else if (ctrl_mi)
			mar <= bus[3:0];
	end
	
	// Memory
	always_ff @(posedge clk) begin
		if (reset) begin
			mem[0]  = memory[0];
			mem[1]  = memory[1];
			mem[2]  = memory[2];
			mem[3]  = memory[3];
			mem[4]  = memory[4];
			mem[5]  = memory[5];
			mem[6]  = memory[6];
			mem[7]  = memory[7];
			mem[8]  = memory[8];
			mem[9]  = memory[9]; 
			mem[10] = memory[10];
			mem[11] = memory[11];
			mem[12] = memory[12];
			mem[13] = memory[13];
			mem[14] = memory[14];
			mem[15] = memory[15];
		end
		else if (ctrl_ri)
			mem[mar] = bus;
	end
	
	// Output Register
	always_ff @(posedge clk or posedge reset) begin
		if (reset)
			out <= 0;
		else if (ctrl_oi)
			out <= bus;
	end
	
	// ALU	
	always_ff @(posedge clk or posedge reset) begin
		if (reset)
			a_reg <= 0;
		else if (ctrl_ai)
			a_reg <= bus;
	end
	
	always_ff @(posedge clk or posedge reset) begin
		if (reset)
			b_reg <= 0;
		else if (ctrl_bi)
			b_reg <= bus;
	end
	
	// Zero flag is set if ALU is zero
	assign flag_z = (alu[7:0] == 0) ? 1 : 0;

	// Use twos-complement for subtraction
	assign b_reg_out = ctrl_su ? ~b_reg + 1 : b_reg;

	// Carry flag is set if there's an overflow into bit 8 of the ALU
	assign flag_c = alu[8];

	assign alu = a_reg + b_reg_out;
	
	// Flags Register
	always_ff @(posedge clk or posedge reset) begin
		if (reset)
			flags <= 0;
		else if (ctrl_fi)
			flags <= {flag_c, flag_z};
	end
	
	
	// Control Signals

	// Halt
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_HLT && stage == 2)
			ctrl_ht <= 1;
		else
			ctrl_ht <= 0;
	end

	// Memory Address Register In
	always_ff @(negedge clk) begin
		if (stage == 0)
			ctrl_mi <= 1;
		else if (ir[7:4] == OP_LDA && stage == 2)
			ctrl_mi <= 1;
		else if (ir[7:4] == OP_ADD && stage == 2)
			ctrl_mi <= 1;
		else if (ir[7:4] == OP_SUB && stage == 2)
			ctrl_mi <= 1;
		else if (ir[7:4] == OP_STA && stage == 2)
			ctrl_mi <= 1;
		else
			ctrl_mi <= 0;
	end

	// RAM In
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_STA && stage == 3)
			ctrl_ri <= 1;
		else
			ctrl_ri <= 0;
	end

	// RAM Out
	always_ff @(negedge clk) begin
		if (stage == 1)
			ctrl_ro <= 1;
		else if (ir[7:4] == OP_LDA && stage == 3)
			ctrl_ro <= 1;
		else if (ir[7:4] == OP_ADD && stage == 3)
			ctrl_ro <= 1;
		else if (ir[7:4] == OP_SUB && stage == 3)
			ctrl_ro <= 1;
		else
			ctrl_ro <= 0;
	end

	// Instruction Register Out
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_LDA && stage == 2)
			ctrl_io <= 1;
		else if (ir[7:4] == OP_LDI && stage == 2)
			ctrl_io <= 1;
		else if (ir[7:4] == OP_ADD && stage == 2)
			ctrl_io <= 1;
		else if (ir[7:4] == OP_SUB && stage == 2)
			ctrl_io <= 1;
		else if (ir[7:4] == OP_STA && stage == 2)
			ctrl_io <= 1;
		else if (ir[7:4] == OP_JMP && stage == 2)
			ctrl_io <= 1;
		else if (ir[7:4] == OP_JC && stage == 2)
			ctrl_io <= 1;
		else if (ir[7:4] == OP_JZ && stage == 2)
			ctrl_io <= 1;
		else
			ctrl_io <= 0;
	end

	// Instruction Register In
	always_ff @(negedge clk) begin
		if (stage == 1)
			ctrl_ii <= 1;
		else
			ctrl_ii <= 0;
	end

	// A Register In
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_LDI && stage == 2)
			ctrl_ai <= 1;
		else if (ir[7:4] == OP_LDA && stage == 3)
			ctrl_ai <= 1;
		else if (ir[7:4] == OP_ADD && stage == 4)
			ctrl_ai <= 1;
		else if (ir[7:4] == OP_SUB && stage == 4)
			ctrl_ai <= 1;
		else
			ctrl_ai <= 0;
	end

	// A Register Out
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_STA && stage == 3)
			ctrl_ao <= 1;
		else if (ir[7:4] == OP_OUT && stage == 2)
			ctrl_ao <= 1;
		else
			ctrl_ao <= 0;
	end

	// Sum Out
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_ADD && stage == 4)
			ctrl_eo <= 1;
		else if (ir[7:4] == OP_SUB && stage == 4)
			ctrl_eo <= 1;
		else
			ctrl_eo <= 0;
	end

	// Subtract
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_SUB && stage == 4)
			ctrl_su <= 1;
		else
			ctrl_su <= 0;
	end

	// B Register In
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_ADD && stage == 3)
			ctrl_bi <= 1;
		else if (ir[7:4] == OP_SUB && stage == 3)
			ctrl_bi <= 1;
		else
			ctrl_bi <= 0;
	end

	// Output Register In
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_OUT && stage == 2)
			ctrl_oi <= 1;
		else
			ctrl_oi <= 0;
	end

	// Counter Enable
	always_ff @(negedge clk) begin
		if (stage == 1)
			ctrl_ce <= 1;
		else
			ctrl_ce <= 0;
	end

	// Counter Out

	always_ff @(negedge clk) begin
		// Always in Stage 0
		if (stage == 0)
			ctrl_co <= 1;
		else
			ctrl_co <= 0;
	end

	// Jump
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_JMP && stage == 2)
			ctrl_jp <= 1;
		else if (ir[7:4] == OP_JC && stage == 2 && flags[FLAG_C] == 1)
			ctrl_jp <= 1;
		else if (ir[7:4] == OP_JZ && stage == 2 && flags[FLAG_Z] == 1)
			ctrl_jp <= 1;
		else
			ctrl_jp <= 0;
	end

	// Flags Register In
	always_ff @(negedge clk) begin
		if (ir[7:4] == OP_ADD && stage == 4)
			ctrl_fi <= 1;
		else if (ir[7:4] == OP_SUB && stage == 4)
			ctrl_fi <= 1;
		else
			ctrl_fi <= 0;
	end
	
endmodule