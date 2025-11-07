module Homework_2 (input logic clk,
                   input logic reset,
                   input logic [2:0] buttons,   // B=001, Y=010, X=100
                   output logic [1:0] action);  // 00=Stand, 01=Jump, 10=Double Jump, 11=Run

                // pls encode the output action as
                typedef enum logic [1:0] {
                    STAND = 2'b00
                    JUMP = 2'b01
                    DOUBLE JUMP = 2'b10
                    RUN = 2'b11
                } state_t;

                state_t current_state, next_state;

            // Synchronous reset and state register
                always_ff @(posedge clk) begin
                    if(reset)
                        current_state <= STAND;
                    else
                        current_state <= next_state;
                end

                always_comb begin
                    next_state = current_state;
                    unique case (current_state)