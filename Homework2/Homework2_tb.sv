`timescale 1ns/1ps

module tb_Homework_2;

    // DUT ports
    logic clk;
    logic reset;
    logic [2:0] buttons;
    logic [1:0] action;

    // instantiate DUT
    Homework_2 dut (
        .clk(clk),
        .reset(reset),
        .buttons(buttons),
        .action(action)
    );

    // clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period
    end

    integer errors = 0;
    integer tests = 0;

    // helper: apply buttons for N rising edges
    task automatic apply_buttons(input logic [2:0] b, input int cycles);
        int i;
        begin
            buttons = b;
            for (i = 0; i < cycles; i = i + 1) begin
                @(posedge clk);
                #1;
            end
            // release
            buttons = 3'b000;
            @(posedge clk); #1;
        end
    endtask

    initial begin
        $display("\nStarting tb at time %0t\n", $time);
        $dumpfile("tb_homework2.vcd");
        $dumpvars(0, tb_Homework_2, dut);

        // init
        reset = 1;
        buttons = 3'b000;
        @(posedge clk);
        @(posedge clk);
        reset = 0;
        @(posedge clk);
        #1;

        // Test 1: after reset -> STAND (00)
        tests = tests + 1;
        if (action !== 2'b00) begin
            $display("TEST %0d FAIL: after reset action=%b expected=00", tests, action);
            errors = errors + 1;
        end else $display("TEST %0d PASS: after reset action=%b", tests, action);

        // Test 2: Press X -> Jump (01)
        tests = tests + 1;
        apply_buttons(3'b100, 1); // X
        if (action !== 2'b01) begin
            $display("TEST %0d FAIL: X -> action=%b expected=01 (Jump)", tests, action);
            errors = errors + 1;
        end else $display("TEST %0d PASS: X -> Jump (%b)", tests, action);

        // Test 3: While in Jump, press B+X -> Double Jump (10)
        tests = tests + 1;
        // first go to jump again (ensure we're in JUMP)
        apply_buttons(3'b100, 1); // X -> Jump
        apply_buttons(3'b101, 1); // B+X -> Double Jump
        if (action !== 2'b10) begin
            $display("TEST %0d FAIL: B+X in Jump -> action=%b expected=10 (Double Jump)", tests, action);
            errors = errors + 1;
        end else $display("TEST %0d PASS: B+X -> Double Jump (%b)", tests, action);

        // After DoubleJump, should return to Stand
        tests = tests + 1;
        @(posedge clk); #1;
        if (action !== 2'b00) begin
            $display("TEST %0d FAIL: after DoubleJump action=%b expected=00 (Stand)", tests, action);
            errors = errors + 1;
        end else $display("TEST %0d PASS: after DoubleJump -> Stand (%b)", tests, action);

        // Test 4: From Stand, press Y -> Run (11)
        tests = tests + 1;
        apply_buttons(3'b010, 1); // Y
        if (action !== 2'b11) begin
            $display("TEST %0d FAIL: Y in Stand -> action=%b expected=11 (Run)", tests, action);
            errors = errors + 1;
        end else $display("TEST %0d PASS: Y -> Run (%b)", tests, action);

        // Test 5: In Run, release Y -> Stand
        tests = tests + 1;
        @(posedge clk); #1; // no button
        if (action !== 2'b00) begin
            $display("TEST %0d FAIL: release Y -> action=%b expected=00 (Stand)", tests, action);
            errors = errors + 1;
        end else $display("TEST %0d PASS: release Y -> Stand (%b)", tests, action);

        // Test 6: From Jump, pressing Y -> Run
        tests = tests + 1;
        apply_buttons(3'b100, 1); // X -> Jump
        apply_buttons(3'b010, 1); // Y -> Run
        if (action !== 2'b11) begin
            $display("TEST %0d FAIL: Jump then Y -> action=%b expected=11 (Run)", tests, action);
            errors = errors + 1;
        end else $display("TEST %0d PASS: Jump then Y -> Run (%b)", tests, action);

        // summary
        $display("\nTESTS completed: %0d, ERRORS: %0d\n", tests, errors);
        if (errors == 0) $display("ALL TESTS PASS\n");
        else $display("SOME TESTS FAILED - see messages\n");

        #20;
        $finish;
    end

endmodule
