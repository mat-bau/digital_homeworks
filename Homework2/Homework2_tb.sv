`timescale 1ns/1ps

module Homework_2_tb;

    // Inputs
    logic clk;
    logic reset;
    logic [2:0] buttons;

    // Output
    logic [1:0] action;

    // Instantiate DUT
    Homework_2 dut (
        .clk(clk),
        .reset(reset),
        .buttons(buttons),
        .action(action)
    );

    // Clock generation (period = 10ns)
    always #5 clk = ~clk;

    // Task pour afficher l’état
    task display_state(input string label);
        $display("[%0t] %s | buttons=%b | action=%b",
                 $time, label, buttons, action);
    endtask

    // Test sequence
    initial begin
        // Initialisation
        clk = 0;
        reset = 1;
        buttons = 3'b000;
        #10;
        reset = 0;

        // Etat initial : STAND
        display_state("Initial (STAND)");

        // Appui sur X → JUMP
        buttons = 3'b100;
        #10; display_state("Press X (JUMP)");

        // Appui simultané sur B et X → DOUBLE JUMP
        buttons = 3'b101;
        #10; display_state("Press B+X (DOUBLE JUMP)");

        // Retour à STAND
        buttons = 3'b000;
        #10; display_state("Back to STAND");

        // Appui sur Y → RUN
        buttons = 3'b010;
        #10; display_state("Press Y (RUN)");

        // Reste sur RUN tant que Y maintenu
        #10; display_state("Still RUN");

        // Relâchement de Y → STAND
        buttons = 3'b000;
        #10; display_state("Release Y (STAND)");

        // JUMP → puis Y → retour RUN
        buttons = 3'b100;
        #10; display_state("Jump again");
        buttons = 3'b010;
        #10; display_state("Then press Y (RUN)");

        // RESET pour vérifier retour à STAND
        reset = 1;
        #10; reset = 0;
        #10; display_state("After Reset (STAND)");

        $display("Test completed.");
        $finish;
    end

endmodule
