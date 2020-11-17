// TOP LEVEL MODULE
module src (); 
endmodule
// CPU
module CPU (
	input  logic  clk, 
);		

	ROM #() rom(, instruction); 
	assign rd = instruction[3:0]; 
	assign rt = instruction[7:4]; 
	assign rs = instruction[11:8]; 
	assign op = instruction[17:12];

endmodule 
// ROM
module ROM #(parameter m = 7, w = 4) (
	input  logic [m - 1:0] Ad; 
	output logic [w - 1:0] Dout
); 

	logic [w - 1:0] mem[2**m - 1:0]; 
	assign Dout = mem[Ad]; 
	
	initial begin
		$readmemb("", mem); 
	end 

endmodule 
// Program Counter
module PC (
	input  logic clk, 
	output logic [31:0] q
); 

	always_ff @(posedge clk) 
		q <= q + 1; 

endmodule 
// Opcode Priority 
module OP (
	input  logic [4:0] opcode, 
	output logic rtype, branch1, jump, branch2, itype, mem
); 

	assign jump    = (~opcode[4] & ~opcode[3] & ~opcode[2] & ~opcode[1] & ~opcode[0]); 
	assign branch1 = (~opcode[4] & ~opcode[3] & ~opcode[2] & ~opcode[1] &  opcode[0]); 
	assign branch2 = (~opcode[4] & ~opcode[3] & ~opcode[2] &  opcode[1]); 
	assign itype   = (~opcode[4] & ~opcode[3] &  opcode[2]); 
	assign mem     = (~opcode[4] &  opcode[3]);
	assign rtype   =   opcode[4]; 

endmodule 
// ALU
module ALU #(parameter n = 4) (
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
// Multiply-Division 
module multdiv #(parameter n = 4) (
	input  logic [n - 1:0] a, b, 
	input  logic clk,
	input  logic [3:0] F, 
	output logic [n - 1:0] Y, hiNext, loNext
);

	logic en1, en2; 
	logic [n - 1:0] hi, lo, h, l, r, q; 

	assign en1 = (); 
	assign en2 = ();

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
// Shifter 
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
// Immediate operand extension 
module IE ();
endmodule; 
// Register call function 
module regfile (); 

	logic [15:0] rf[15:0]; 
	always_ff @(posedge clk) 
		rf[RA1_WA] <= WD; 
	assign RD1 = rf[RA1_WA]; 
	assign RD2 = rf[RA2]; 
	assign BTD = rf[BTA];

endmodule 
// Adder with overflow 
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
// Set less than 
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
// 4 to 1 mux 
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
// 2 to 1 mux 
module mux2to1 #(parameter W = 1) (
	input  logic s, 
	input  logic [W - 1:0] d0, d1,
	output logic [W - 1:0] y
);

	assign y = s?  d1 : d0;

endmodule
// Adder 
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