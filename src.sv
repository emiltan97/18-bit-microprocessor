/********************TOP MODULE********************/
module src (
	input  logic [9:0] SW,
	input  logic [3:0] KEY,
	output logic [7:0] LEDG, 
	output logic [9:0] LEDR,
	output logic [6:0] HEX0, HEX1, HEX2, HEX3	
);

endmodule
/**************************************************/
/********************R-TYPE ALU********************/
module RALU #(parameter n = 4) (
	input  logic [3:0] F, 
	input  logic [n - 1:0] A, B,
	output logic [n - 1:0] Y, 
	output logic Cout, OV 
); 

	logic [n - 1:0] S, AUout, logicOut; 
	logic OVs; 

	OVadder #(n) add0(F[1:0], A, B, Cout, OV, OVs, S);
	slt #(n) slt0(OVs, Cout, F[0], F[3], S, AUout);
	mux4to1 #(n) mux0(
		(A & B), (A | B), (A^B), ~(A | B), 
		F[1:0], 
		logicOut
	);
	mux2to1 #(n) mux1(F[2], AUout, logicOut, Y);
	
endmodule 

module OVadder #(parameter n = 4) (
	input  logic [1:0] s, 
	input  logic [n - 1:0] a, b, 
	output logic Cout, OV, OVs,
	output logic [n - 1:0] S
);

	logic OVu;
	logic [n - 1:0] B;
	
	addsub #(n) add0(a, b, s[1], S, B, Cout); 
	
	assign OVs = (S[n-1] ^ a[n-1]) & ~(a[n-1] ^ B[n-1]);
	assign OVu = s[1] ^ Cout; 
	
	mux2to1 #(n) mux1(s[0], OVs, OVu, OV);

endmodule

module slt #(parameter n = 4) (
	input  logic  OVs, Cout, s0, s2,
	input  logic [n - 1:0] S,
	output logic [n - 1:0] AUout 
); 

	logic x, y;
	logic [n - 1:0] z;

	mux2to1 #(1) mux0 (OVs, S[n-1], Cout, x); 	
	mux2to1 #(1) mux1 (s0, x, ~Cout, y); 
	
	assign z = { {(n - 1){1'b0}}, y };
	
	mux2to1 #(n) mux2 (s2, S, z, AUout); 

endmodule 
/**************************************************/
/******************R-TYPE MULTDIV******************/
module rmultdiv #(parameter n = 4) (
	input  logic [n - 1:0] a, b, 
	input  logic clk,
	input  logic [3:0] F, 
	output logic [n - 1:0] Y, hiNext, loNext
);

logic en1, en2; 
logic [n - 1:0] hi, lo, h, l, r, q; 

assign en1 = (
	(~F[3] & ~F[2] & ~F[1] &  F[0]) |
	( F[3] & ~F[2] & ~F[1] & ~F[0]) 
); 
assign en2 = (
	(~F[3] & ~F[2] &  F[1] &  F[0]) |
	( F[3] & ~F[2] &  F[1] & ~F[0])
);

assign {h, l} = a * b; 
assign  r     = a % b; 
assign  q     = a / b; 

mux2to1 #(n) mux0(F[1], hiNext, loNext, Y);
mux2to1 #(n) mux1(F[1], h, r, hi);
mux2to1 #(n) mux2(F[1], l, q, lo);

always_ff @(posedge clk) 
	begin
		if(en1) hiNext <= F[3]? hi : a;
		if(en2) loNext <= F[3]? lo : a;
	end

endmodule
/**************************************************/
/*****************R-TYPE SHIFTER*******************/
module shifter #(parameter n = 4) (
	input  logic [1:0] F, 
	input  logic [2**n - 1:0] A, 
	input  logic [n - 1:0] Sh, 
	output logic [2**n - 1:0] Y 
);

	logic [2**n - 1:0] D0, D1; 
	
	assign D0 = F[0]?  (A >> Sh)           : (A << Sh);
	assign D1 = F[0]?  ($signed(A) >>> Sh) : (A);
	assign Y  = F[1]?  D1                  : D0;

endmodule
/**************************************************/
/*********************MISC*************************/
module mux4to1 #(parameter W = 2) (
	input  logic [W - 1:0] d0, d1, d2, d3,
	input  logic [1:0] s,
	output logic [W - 1:0] y
); 	

	logic [W - 1:0] lo, hi; 
	
	assign lo = s[0]?  d1 : d0;
	assign hi = s[0]?  d3 : d2;
	assign y  = s[1]?  hi : lo;
	
endmodule

module mux2to1 #(parameter W = 1) (
	input  logic s, 
	input  logic [W - 1:0] d0, d1,
	output logic [W - 1:0] y
);

	assign y = s?  d1 : d0;

endmodule

module addsub #(parameter W = 4) (
	input  logic [W - 1:0] A, B,
	input  logic s,
	output logic [W - 1:0] Y,	
	output logic [W - 1:0] b,
	output logic cout
); 

	mux2to1 #(W) mux1(s, B, ~B, b);
	assign {cout,Y} = A + b + s;

endmodule 
/**************************************************/