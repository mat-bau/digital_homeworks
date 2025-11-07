module Homework_4_tb();

logic [31:0] a, b, Result, Resultexpected;
logic [1:0]  ALUControl;
logic [3:0]  ALUFlags, ALUFlagsexpected;

logic clk;

logic [31:0] vectornum, errors;
logic [103:0] testvectors [0:15];

// instantiate device under test
Homework_4 dut(a, b, ALUControl, Result, ALUFlags);

always
	begin
		clk = 1; #2; clk = 0; #2;
	end

// at start of test, load vectors
initial
	begin
		$readmemh("alu.tv", testvectors);
		vectornum = 0; errors = 0;
	end

// apply test vectors on rising edge of clk
always @(posedge clk)
	begin
		#1; {ALUControl, a, b, Resultexpected, ALUFlagsexpected} = testvectors[vectornum];
		$display("Test %d ...",vectornum);
	end

// check results on falling edge of clk
always @(negedge clk)
	begin
		if ((Result !== Resultexpected) | (ALUFlags !== ALUFlagsexpected)) begin			
			$display("Error: a = %h, b = %h, ALUControl = %h", a, b, ALUControl);
			$display("Result = %h (%h expected), ALUFlags = %b (%b expected)", 
				Result, Resultexpected, ALUFlags, ALUFlagsexpected);
			errors = errors + 1;
		end
		vectornum = vectornum + 1;
		if (testvectors[vectornum] === 104'hx) begin
			$display("%d tests completed with %d errors",
				vectornum, errors);
		end
	end
endmodule
