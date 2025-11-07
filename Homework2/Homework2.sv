module Homework_2 (input logic clk,
                   input logic reset,
                   input logic [2:0] buttons,   // B=001, Y=010, X=100
                   output logic [1:0] action);  // 00=Stand, 01=Jump, 10=Double Jump, 11=Run

    // pls encode the output action as
    typedef enum logic [1:0] {
        STAND = 2'b00
        JUMP = 2'b01
        DOUBLE_JUMP = 2'b10
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

// next state logic
    always_comb begin
        next_state = current_state;

        logic xb, yb, bb;
        xb = buttons[2];
        yb = buttons[1];
        bb = buttons[0];

        unique case (current_state)
            STAND: begin
                if (xb) 
                    next_state = JUMP;
                else if (yb)
                    next_state = RUN;
                else
                    next_state = STAND;
            end

            RUN: begin
                if (xb)
                    next_state = JUMP;
                if (yb)
                    next_state = RUN;
                else
                    next_state = STAND;
            end

            JUMP: begin
                if (bb && xb)
                    next_state = DOUBLE_JUMP;
                if (yb)
                    next_state = RUN;
                else
                    next_state = STAND;
            end

            DOUBLE_JUMP: begin
                next_state = STAND;
            end
        endcase
    end

    always_comb begin
        action = current_state
    end
endmode