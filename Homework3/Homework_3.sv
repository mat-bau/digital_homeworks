module Homework_4( input  logic [31:0] a, b,
		   input  logic [1:0]  ALUControl,
		   output logic [31:0] Result,
		   output logic [3:0]  ALUFlags);

logic [31:0] S, b_out;
logic c_out;

assign b_out = ALUControl[0] ? ~b : b;
assign {c_out,S} = a + b_out + ALUControl[0];

always_comb
	case (ALUControl)
		2'b00: begin 
		       Result          = S;
		       ALUFlags[0]     = a[31] & b[31] & ~Result[31] | ~a[31] & ~b[31] & Result[31];
		       end
		2'b01: begin
		       Result 	       = S;
		       ALUFlags[0]     = ~a[31] & b[31] & Result[31] | a[31] & ~b[31] & ~Result[31];
		       end
		2'b10: begin 
		       Result 	       = a & b;
		       ALUFlags[0]     = 2'b0;
		       end
		2'b11: begin
		       Result          = a | b;
		       ALUFlags[0]     = 2'b0;
		       end
		default: begin
		         Result        = 32'b0;
		         ALUFlags[0]   = 2'b0;
		         end
	endcase

assign ALUFlags[1] = c_out & ~ALUControl[1];

assign ALUFlags[2] = (Result == 32'b0);

assign ALUFlags[3] = Result[31];

endmodule
