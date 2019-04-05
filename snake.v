// Part 2 skeleton

module snake
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
        KEY,
        SW,
		  HEX0,
		  LEDR,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B   						//	VGA Blue[9:0]
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;
	output   [6:0]   HEX0;
	output [9:0] LEDR;
 
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	wire resetn;
	assign resetn = KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "snake8.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
    
    // GENERAL COUNTER CONTROLS 
	wire [7:0] counter_load_value;
	wire counter_load;
	wire counter_clear; 
	wire [7:0] counter_value;
	
   wire update; 

   integer count;
	integer size = 1;

   reg [7:0] snakeX [0:100];
   reg [6:0] snakeY [0:100];
	reg [2:0] snakeC [0:100];
	
	reg [7:0] appleX;
	reg [6:0] appleY;
	reg [2:0] appleC;

	integer count_segment;
	 
   Clock_divider update_clock(
       .clock_in(CLOCK_50),
       .clock_out(update)
   );
	assign LEDR[0] = update;

   wire [4:0] direction;
	assign direction = SW[4:0];

	// GENERATE APPLE ////////////////////////////////////////////////////////////////////
	 
	////////////////////////////////////////////////////////////////////////////////////
	
	//reg game_over = 1'b0;
	wire started; 
   //assign started = SW[0];
	 always@(posedge update)
	 begin
//		if (snakeX[0] == 119 | snakeX[0] == 0 | snakeY[0] == 119 | snakeY[0] == 0)
//		begin
//			game_over = 1'b1;
//		end
		
		if(started)
		begin
			  for(count = 6; count > 0; count = count - 1)
			  begin
				if(count <= size - 1)
		      begin
					snakeX[count] <= snakeX[count - 1];
					snakeY[count] <= snakeY[count - 1];
					snakeC[count] <= 3'b000;
				end
				else 
				begin
				   snakeX[count] <= 50;
					snakeY[count] <= 50;
					snakeC[count] <= 3'b011;
				end
			  end

			case(direction)
				5'b00010: begin
					snakeY[0] <= snakeY[0] - 1;
					snakeC[count] = 3'b101;
				end
				5'b00100: begin 
					snakeX[0] <= snakeX[0] - 1;
					snakeC[count] <= 3'b101;
				end
				5'b01000: begin
					snakeY[0] <= snakeY[0] + 1;
					snakeC[count] <= 3'b101;
				end
				5'b10000: begin
					snakeX[0] <= snakeX[0] + 1;
					snakeC[count] <= 3'b101;
				end
				default: begin 
					snakeX[0] <= snakeX[0] + 1;
					snakeC[count] <= 3'b101;
				end
			endcase
			
		  end
		  else if (!started)
		  begin
				snakeX[0] <= 11;
				snakeY[0] <= 10;
				snakeC[0] <= 3'b101;
//				snakeX[1] <= 11;
//				snakeY[1] <= 11;
//				snakeC[1] <= 3'b101;
//				snakeX[2] <= 11;
//				snakeY[2] <= 12;
//				snakeC[2] <= 3'b101;
//				snakeX[3] <= 11;
//				snakeY[3] <= 13;
//				snakeC[3] <= 3'b101;
//				snakeX[4] <= 11;
//				snakeY[4] <= 14;
//				snakeC[4] <= 3'b101;
//				snakeX[5] <= 11;
//				snakeY[5] <= 15;
//				snakeC[5] <= 3'b101;
//				appleX <= 8'b01110110;
//				appleY <= 7'b1110110;
//				appleC <= 3'b010;
				//game_over = 1'b0;
		  end
	  end


    //CHECK COLLISIONS 
	 // Check if snakeX[0] and snakeY[0] exists in snake[i]
	 // Check if snakeX[0] and snakeY[0] is same as appleX and appleY
	 // Check if passed borders 
	 
	 //

    behav_counter general_counter(
    	.d(counter_load_value),
    	.clk(CLOCK_50),
    	.clear(counter_clear),
    	.load(1'b0),
    	.qd(counter_value)
    );
	 
	//wire plot;
	//assign writeEn = (plot & !started) | (!update & started);  
	 
//	always@(*)
//	begin 
//		for(count_segment = 100; count_segment > 0; count_segment = count_segment - 1)
//		   begin
//				current_snakeX = snakeX[count_segment];
//				current_snakeY = snakeY[count_segment];
//				current_snakeC = snakeC[count_segment];
//		   end
//	end
	
//	 assign enable = (!(body_segment == 100) & !update);
//    //assign enable = !update;
//	//assign writeEN = !update;
//    snake_body_counter segment_counter(
//        .out(body_segment),
//        .enable(enable),
//        .clk(CLOCK_50),
//        .reset(resetn)
//    );
//	
	control C0(
		.clk(CLOCK_50),
		.resetn(resetn),
		.key1(KEY[1]),
		.key3(KEY[3]),
		.update(update),
		//.game_over(game_over),
		.gcounter_value(counter_value),
		.snakeX(snakeX[counter_value]),
		.snakeY(snakeY[counter_value]),
		.snakeC(snakeC[counter_value]),
		.writeEN(writeEn),
		.x(x),
		.y(y),
		.col(colour),
		.gcounter_load_value(counter_load_value),
		.gcounter_load(counter_load),
		.gcounter_clear(counter_clear),
		.started(started),
		.state(state)
	);
	
	wire [5:0] state;
	//assign state = 4'b1001;
	hex_display hex0(
		.OUT(HEX0),
		.IN(state)
		);
		


//		assign colour = (c_C & !(state == 4'b1001)) | (snakeC[body_segment] & (state == 4'b1001));
//		assign x = (c_X & !(state == 4'b1001)) | (snakeX[body_segment] & (state == 4'b1001));
//		assign y = (c_Y & !(state == 4'b1001)) | (snakeY[body_segment] & (state == 4'b1001));
		
//		assign colour = snakeC[body_segment];
//		assign x = snakeX[body_segment];
//		assign y = snakeY[body_segment];
	
		

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


module control(
    input clk,
    input resetn,
    input key1,
    input key3,
    input update,
    
    //////////       INPUTS    /////////////
    input [7:0] gcounter_value,
	 input [7:0] snakeX,
	 input [6:0] snakeY,
	 input [2:0] snakeC,
	 input [7:0] appleX,
	 input [6:0] appleY,
	 input [2:0] appleC,
	 //input game_over,

    ////////// OUTPUT CONTROLS /////////////
    
    // VGA CONTROLS
    output reg writeEN,
    output reg [7:0] x,
    output reg [6:0] y,
    output reg [2:0] col,
	 output reg [7:0] a_x,
    output reg [6:0] a_y,
    output reg [2:0] a_col,

    // GENERAL COUNTER CONTROLS
    output reg [7:0] gcounter_load_value,
    output reg gcounter_load,
    output reg gcounter_clear,

    // TESTING 
	 output reg started,
	 output reg [3:0] state
	 
    );

    reg [5:0] current_state, next_state; 
    
    localparam  S_INITIALIZE_TOP_BORDER           = 5'd0,
    			S_DRAW_TOP_BORDER                     = 5'd1,
    			S_INITIALIZE_LEFT_BORDER              = 5'd2,
    			S_DRAW_LEFT_BORDER                    = 5'd3,
    			S_INITIALIZE_RIGHT_BORDER             = 5'd4,
    			S_DRAW_RIGHT_BORDER                   = 5'd5,
    			S_INITIALIZE_BOTTOM_BORDER            = 5'd6,
    			S_DRAW_BOTTOM_BORDER                  = 5'd7,
    			S_INITIALIZE_START                    = 5'd8,
            S_UPDATE_SNAKE_APPLE                  = 5'd9,
				S_DRAW_SNAKE_APPLE                    = 5'd10,
            S_GAME_OVER     	                    = 5'd11;
         
    
    // Next state logic aka our state table
    always@(*)
    begin: state_table 
        case (current_state)
            S_INITIALIZE_TOP_BORDER: next_state = S_DRAW_TOP_BORDER;
            S_DRAW_TOP_BORDER: next_state = (gcounter_value == 119) ? S_INITIALIZE_LEFT_BORDER : S_DRAW_TOP_BORDER; 
            S_INITIALIZE_LEFT_BORDER: next_state = S_DRAW_LEFT_BORDER;
            S_DRAW_LEFT_BORDER: next_state = (gcounter_value == 119) ? S_INITIALIZE_RIGHT_BORDER : S_DRAW_LEFT_BORDER;
            S_INITIALIZE_RIGHT_BORDER: next_state = S_DRAW_RIGHT_BORDER;
            S_DRAW_RIGHT_BORDER: next_state = (gcounter_value == 119) ? S_INITIALIZE_BOTTOM_BORDER : S_DRAW_RIGHT_BORDER;
            S_INITIALIZE_BOTTOM_BORDER: next_state = S_DRAW_BOTTOM_BORDER;
            S_DRAW_BOTTOM_BORDER: next_state = (gcounter_value == 119) ? S_INITIALIZE_START : S_DRAW_BOTTOM_BORDER;
				S_INITIALIZE_START: next_state = !key1 ? S_UPDATE_SNAKE_APPLE : S_INITIALIZE_START;
				S_UPDATE_SNAKE_APPLE: next_state = !update ? S_DRAW_SNAKE_APPLE : S_UPDATE_SNAKE_APPLE;
//				begin
//					if (game_over == 1'b1) begin 
//						next_state = S_GAME_OVER;
//					end 
//					else begin 
//						next_state = !update ? S_DRAW_SNAKE_APPLE : S_UPDATE_SNAKE_APPLE;
//						end 
//					end 
				//next_state = !update ? S_DRAW_SNAKE_APPLE : S_UPDATE_SNAKE_APPLE;
				S_DRAW_SNAKE_APPLE: next_state = (gcounter_value == 4) ? S_UPDATE_SNAKE_APPLE : S_DRAW_SNAKE_APPLE;
				//S_GAME_OVER: next_state = S_GAME_OVER;
				default:     next_state = S_INITIALIZE_TOP_BORDER;
			endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // By default make all our signals 0
        //gcounter_load = 1'b0;
        //gcounter_clear = 1'b0;
        writeEN = 1'b0;

        case (current_state)
            S_INITIALIZE_TOP_BORDER: begin
            	gcounter_load = 1'b1;
			      gcounter_clear = 1'b0;
				   state = 4'b0000;
            	end
            S_DRAW_TOP_BORDER: begin
            	writeEN = 1'b1;
            	gcounter_load = 1'b0;
					gcounter_clear = 1'b1;
            	x = gcounter_value;
            	y = 8'b00000000;
            	col = 3'b101;
					state = 4'b0001;
            	end
            S_INITIALIZE_LEFT_BORDER: begin
            	gcounter_load = 1'b1;
            	gcounter_load_value = 0;
					gcounter_clear = 1'b0;
					state = 4'b0010;
            	end
            S_DRAW_LEFT_BORDER: begin
            	writeEN = 1'b1;
            	gcounter_load = 1'b0;
					gcounter_clear = 1'b1;
            	x = 8'b00000000;
            	y = gcounter_value;
            	col = 3'b101;
					state = 4'b0011;
            	end
            S_INITIALIZE_RIGHT_BORDER: begin
            	gcounter_load = 1'b1;
            	gcounter_load_value = 0;
					gcounter_clear = 1'b0;
					state = 4'b0100;
            	end
            S_DRAW_RIGHT_BORDER: begin
            	writeEN = 1'b1;
            	gcounter_load = 1'b0;
					gcounter_clear = 1'b1;
            	x = 119;
            	y = gcounter_value;
            	col = 3'b101;
					state = 4'b0101;
            	end
            S_INITIALIZE_BOTTOM_BORDER: begin
            	gcounter_load = 1'b1;
            	gcounter_load_value = 0;
					gcounter_clear = 1'b0;
					state = 4'b0110;
            	end
            S_DRAW_BOTTOM_BORDER: begin
            	writeEN = 1'b1;
            	gcounter_load = 1'b0;
					gcounter_clear = 1'b1;
            	x = gcounter_value;
            	y = 8'b01110111;
            	col = 3'b101;
					state = 4'b0111;
            	end
            S_INITIALIZE_START: begin
					started = 1'b0;
					gcounter_clear = 1'b0;
               writeEN = 1'b1;
               x = snakeX;
            	y = snakeY;
            	col = snakeC;
					state = 4'b1000;
            	end
				S_UPDATE_SNAKE_APPLE: begin 
					gcounter_clear = 1'b0;
               writeEN = 1'b0;
					started = 1'b1;
					//x = snakeX;
            	//y = snakeY;
            	//col = snakeC;
					state = 4'b1001;
					end
				S_DRAW_SNAKE_APPLE: begin 
					//gcounter_load = 1'b0;
					gcounter_clear = 1'b1;
					writeEN = 1'b1;
					x = snakeX;
            	y = snakeY;
            	col = snakeC;
					state = 4'b1010;
					end 
//				S_GAME_OVER: begin 
//					//gcounter_load = 1'b0;
//					gcounter_clear = 1'b0;
//					writeEN = 1'b0;
//					state = 4'b1011;
//					end 
        // default:    // don't need default since we already made sure all of our outputs were assigned a value at the start of the always block
        endcase
    end // enable_signals
   
    // current_state registers
    always@(posedge clk)
    begin: state_FFs
        if(!resetn)
            current_state <= S_INITIALIZE_TOP_BORDER;
        else
            current_state <= next_state;
    end // state_FFS
	 
endmodule


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module joystick_direction ();

endmodule 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// https://www.fpga4student.com/2017/08/verilog-code-for-clock-divider-on-fpga.html
// fpga4student.com: FPGA projects, VHDL projects, Verilog projects
// Verilog project: Verilog code for clock divider on FPGA
// Top level Verilog code for clock divider on FPGA
module Clock_divider(clock_in, clock_out);

    input clock_in; // input clock on FPGA
    output reg clock_out; // output clock after dividing the input clock by divisor

    reg[27:0] counter=28'd0;
    parameter DIVISOR = 28'd10000000;


    // The frequency of the output clk_out
    //  = The frequency of the input clk_in divided by DIVISOR
    // For example: Fclk_in = 50Mhz, if you want to get 1Hz signal to blink LEDs
    // You will modify the DIVISOR parameter value to 28'd50.000.000
    // Then the frequency of the output clk_out = 50Mhz/50.000.000 = 1Hz
    always @(posedge clock_in)
    begin
        counter <= counter + 28'd1;
        if(counter>=(DIVISOR-1)) begin
            counter <= 28'd0;
				clock_out = 1'b1;
				end 
		  else 
				clock_out = 1'b0;
    end

    //assign clock_out = (counter==28'd0)?1'b1:1'b0;
endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module snake_body_counter(
    out,  // Output of the counter
    enable,  // enable for counter
    clk,  // clock Input
    reset  // reset Input
    );
    
    output integer out;
    
    input enable, clk, reset;
    
    always @(posedge clk)
    begin
        if (!reset) begin
            out = 0;
        end 
		  else if (out == 100) begin
				out = 0;
		  end
        else if (enable) begin
            out = out + 1;
        end
    end
endmodule


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.intel.com/content/www/us/en/programmable/support/support-resources/design-examples/design-software/verilog/ver_behav_counter.html
module behav_counter( d, clk, clear, load, qd);

	// Port Declaration

	input   [7:0] d;
	input   clk;
	input   clear;
	input   load;
	output  [7:0] qd;

	reg     [7:0] cnt;

	always @ (posedge clk)
	begin
	    if (!clear)
	        cnt <= 8'b00000000;
	    else if (load)
	        cnt <= d;
	    else
	        cnt <= cnt + 1;
	end 
	assign qd = cnt;

endmodule


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module hex_display(IN, OUT);
    input [3:0] IN;
	 output reg [7:0] OUT;
	 
	 always @(*)
	 begin
		case(IN[3:0])
			4'b0000: OUT = 7'b1000000;
			4'b0001: OUT = 7'b1111001;
			4'b0010: OUT = 7'b0100100;
			4'b0011: OUT = 7'b0110000;
			4'b0100: OUT = 7'b0011001;
			4'b0101: OUT = 7'b0010010;
			4'b0110: OUT = 7'b0000010;
			4'b0111: OUT = 7'b1111000;
			4'b1000: OUT = 7'b0000000;
			4'b1001: OUT = 7'b0011000;
			4'b1010: OUT = 7'b0001000;
			4'b1011: OUT = 7'b0000011;
			4'b1100: OUT = 7'b1000110;
			4'b1101: OUT = 7'b0100001;
			4'b1110: OUT = 7'b0000110;
			4'b1111: OUT = 7'b0001110;
			
			default: OUT = 7'b0111111;
		endcase

	end
endmodule