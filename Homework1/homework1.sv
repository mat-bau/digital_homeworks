module Homework_1(input logic[7:0] Input_1,
		 input logic[7:0] Input_2,
		 input logic CLK,
		 input logic RST,
		 input logic[2:0] Control,
		 output logic[7:0] Output);

	/* Flip-flop avec reset asynchrone mets àjours
	   les registres sur les positive edges de la CLK */
	logic MODE;
	logic OP;
	logic DIR;

	assign MODE = Control[2];
	assign OP   = Control[1];
	assign DIR  = Control[0];

	always_ff @(posedge CLK or posedge RST) begin
		if (RST) begin
			Output <= 8'b0;
		end
		else begin
			// 1. MODE = 1 : Calcul via OP
			// 2. MODE = 0 : Deplacement à gauche (DIR = 1) ou	a à droite (DIR = 0)
			Output <= MODE ? (OP ? (Input_1 + Input_2) : (Input_1 - Input_2))
				       : (DIR ? {Output[6:0], Output[7]} : {Output[0], Output[7:1]});
		end
	end
endmodule