module src(
	input  logic [9:0] SW, 
	input  logic [3:0] KEY, 
	input  logic CLOCK_50, 
	output logic [9:0] LEDR, 
	output logic [7:0] LEDG, 
	output logic [6:0] HEX0,HEX1,HEX2,HEX3
); 

	logic [31:0] q;
//	logic clk; 

	counter32 myclocks(CLOCK_50, q);

//	assign clk = CLOCK_50;
	
	CPU cpu(q[25], HEX0, HEX1, HEX2, HEX3);

endmodule
// CPU
module CPU (
	input  logic clk,
	output logic [6:0] HEX0, HEX1, HEX2, HEX3
);		

	logic rtype, branch1, jump, branch2, itype, mem; 
	logic [2:0] pc, pc_next; 
	logic [17:0] instruction; 
	

	always_ff @(posedge clk) begin 
		pc <= pc_next; 
	end 

	ROM #(16, 18) rom(pc, instruction); 
	assign {pc_carry, pc_plus_one} = pc + 1; 
	SSD inst0(instruction[3:0], HEX0);
	SSD inst1(instruction[7:4], HEX1);
	SSD inst2(instruction[11:8], HEX2);
	SSD inst3(instruction[15:12], HEX3);
	
	assign rd = instruction[3:0]; 
	assign rt = instruction[7:4]; 
	assign rs = instruction[11:8]; 
	assign op = instruction[17:12]; 
	OP myopprio(op, rtype, branch1, jump, branch2, itype, mem);
	
	assign pc_next = pc_plus_one;
	
	
//	assign rd = instruction[3:0]; 
//	assign rt = instruction[7:4]; 
//	assign rs = instruction[11:8]; 
//	assign op = instruction[17:12];
//	regfile #() myreg(clk, ra1, ra2, wa, wd, rd1, rd2); 

endmodule 
// ROM
module ROM #(parameter m = 7, w = 4) (
	input  logic [m - 1:0] Ad,
	output logic [w - 1:0] Dout
);

	logic [w - 1:0] mem[2**m - 1:0]; 
	assign Dout = mem[Ad]; 
	
	initial begin
		$readmemb("instructions.txt", mem); 
	end 

endmodule 

module counter32 (
  input logic clk,
  output logic [31:0] q
);

  always_ff @(posedge clk)
    q <= q+1;

endmodule	

module regfile
#(parameter w=4, m=3) (
	input  logic clk,
	input  logic [m-1:0] RAd0, RAd1, WAd,
	input  logic [w-1:0] Din,
	output logic [w-1:0] Dout0, Dout1
);
	logic [2**m-1:0] WEn;
	logic [w-1:0] Q[2**m-1:0];
	parameterized_decoder #(m) decw(WAd,WEn);
	assign Dout0 = Q[RAd0];
	assign Dout1 = Q[RAd1];
	
	genvar k;
	generate
	for(k=0; k<2**m; k=k+1) begin: bloop
		EnabledReg #(w) itk(clk,WEn[k],Din,Q[k]);
	end
	endgenerate
		
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

module EnabledReg
#(parameter w=3) (
	input  logic clk, en,
	input  logic [w-1:0] D,
	output logic [w-1:0] Q
);
	always @(posedge clk) 
		if(en) Q <= D;

endmodule

module parameterized_decoder
#(parameter n=3) (
	input  logic [n-1:0] a,
	output logic [2**n-1:0] y
);
	parameter w = 2**n;
	always_comb begin
		y = {w{1'b0}};
		y[a] = 1'b1;
	end
endmodule

module SSD (
	input  logic [3:0] SW,
	output logic [6:0] HEX0
);

	assign HEX0[0] = (
		(~SW[3] & ~SW[2] & ~SW[1] &  SW[0]) |
		(~SW[3] &  SW[2] & ~SW[1] & ~SW[0]) | 
		( SW[3] & ~SW[2] &  SW[1] &  SW[0]) | 
		( SW[3] &  SW[2] & ~SW[1] &  SW[0])
	);
	assign HEX0[1] = (
		(~SW[3] &  SW[2] & ~SW[1] &  SW[0]) | 
		(~SW[3] &  SW[2] &  SW[1] & ~SW[0]) | 
		( SW[3] & ~SW[2] &  SW[1] &  SW[0]) | 
		( SW[3] &  SW[2] & ~SW[1] & ~SW[0]) | 
		( SW[3] &  SW[2] &  SW[1] & ~SW[0]) | 
		( SW[3] &  SW[2] &  SW[1] &  SW[0])
	);
	assign HEX0[2] = (
		(~SW[3] & ~SW[2] &  SW[1] & ~SW[0]) | 
		( SW[3] &  SW[2] & ~SW[1] & ~SW[0]) | 
		( SW[3] &  SW[2] &  SW[1] & ~SW[0]) | 
		( SW[3] &  SW[2] &  SW[1] &  SW[0])
	);
	assign HEX0[3] = (
		(~SW[3] & ~SW[2] & ~SW[1] &  SW[0]) | 
		(~SW[3] &  SW[2] & ~SW[1] & ~SW[0]) | 
		(~SW[3] &  SW[2] &  SW[1] &  SW[0]) | 
		( SW[3] & ~SW[2] &  SW[1] & ~SW[0]) | 
		( SW[3] &  SW[2] &  SW[1] &  SW[0])
	);
	assign HEX0[4] = (
		(~SW[3] & ~SW[2] & ~SW[1] &  SW[0]) | 
		(~SW[3] & ~SW[2] &  SW[1] &  SW[0]) | 
		(~SW[3] &  SW[2] & ~SW[1] & ~SW[0]) | 
		(~SW[3] &  SW[2] & ~SW[1] &  SW[0]) | 
		(~SW[3] &  SW[2] &  SW[1] &  SW[0]) | 
		( SW[3] & ~SW[2] & ~SW[1] &  SW[0])
	);
	assign HEX0[5] = (
		(~SW[3] & ~SW[2] & ~SW[1] &  SW[0]) | 
		(~SW[3] & ~SW[2] &  SW[1] & ~SW[0]) | 
		(~SW[3] & ~SW[2] &  SW[1] &  SW[0]) | 
		(~SW[3] &  SW[2] &  SW[1] &  SW[0]) | 
		( SW[3] &  SW[2] & ~SW[1] &  SW[0])
	);
	assign HEX0[6] = (
		(~SW[3] & ~SW[2] & ~SW[1] & ~SW[0]) | 
		(~SW[3] & ~SW[2] & ~SW[1] &  SW[0]) | 
		(~SW[3] &  SW[2] &  SW[1] &  SW[0]) | 
		( SW[3] &  SW[2] & ~SW[1] & ~SW[0])
	);

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

//// ALU
//module ALU #(parameter n = 4) (
//	input  logic [3:0] F, 
//	input  logic [n - 1:0] A, B,
//	output logic [n - 1:0] Y, 
//	output logic Cout, OV 
//); 
//
//	logic [n - 1:0] S, AUout, logicOut; 
//	logic OVs; 
//
//	OVadder #(n) add0(F[1:0], A, B, Cout, OV, OVs, S);
//	slt #(n) slt0(OVs, Cout, F[0], F[3], S, AUout);
//	mux4to1 #(n) mux0(
//		(A & B), (A | B), (A^B), ~(A | B), 
//		F[1:0], 
//		logicOut
//	);
//	mux2to1 #(n) mux1(F[2], AUout, logicOut, Y);
//	
//endmodule 
//// Multiply-Division 
//module multdiv #(parameter n = 4) (
//	input  logic [n - 1:0] a, b, 
//	input  logic clk,
//	input  logic [3:0] F, 
//	output logic [n - 1:0] Y, hiNext, loNext
//);
//
//	logic en1, en2; 
//	logic [n - 1:0] hi, lo, h, l, r, q; 
//
//	assign en1 = (); 
//	assign en2 = ();
//
//	assign {h, l} = a * b; 
//	assign  r     = a % b; 
//	assign  q     = a / b; 
//
//	mux2to1 #(n) mux0(F[1], hiNext, loNext, Y);
//	mux2to1 #(n) mux1(F[1], h, r, hi);
//	mux2to1 #(n) mux2(F[1], l, q, lo);
//
//	always_ff @(posedge clk) 
//		begin
//			if(en1) hiNext <= F[3]? hi : a;
//			if(en2) loNext <= F[3]? lo : a;
//		end
//
//endmodule
//// Shifter 
//module shifter #(parameter n = 4) (
//	input  logic [1:0] F, 
//	input  logic [2**n - 1:0] A, 
//	input  logic [n - 1:0] Sh, 
//	output logic [2**n - 1:0] Y 
//);
//
//	logic [2**n - 1:0] D0, D1; 
//	
//	assign D0 = F[0]?  (A >> Sh)           : (A << Sh);
//	assign D1 = F[0]?  ($signed(A) >>> Sh) : (A);
//	assign Y  = F[1]?  D1                  : D0;
//
//endmodule
//// Immediate operand extension 
//module IE ();
//endmodule; 
//// Adder with overflow 
//module OVadder #(parameter n = 4) (
//	input  logic [1:0] s, 
//	input  logic [n - 1:0] a, b, 
//	output logic Cout, OV, OVs,
//	output logic [n - 1:0] S
//);
//
//	logic OVu;
//	logic [n - 1:0] B;
//	
//	addsub #(n) add0(a, b, s[1], S, B, Cout); 
//	
//	assign OVs = (S[n-1] ^ a[n-1]) & ~(a[n-1] ^ B[n-1]);
//	assign OVu = s[1] ^ Cout; 
//	
//	mux2to1 #(n) mux1(s[0], OVs, OVu, OV);
//
//endmodule
//// Set less than 
//module slt #(parameter n = 4) (
//	input  logic  OVs, Cout, s0, s2,
//	input  logic [n - 1:0] S,
//	output logic [n - 1:0] AUout 
//); 
//
//	logic x, y;
//	logic [n - 1:0] z;
//
//	mux2to1 #(1) mux0 (OVs, S[n-1], Cout, x); 	
//	mux2to1 #(1) mux1 (s0, x, ~Cout, y); 
//	
//	assign z = { {(n - 1){1'b0}}, y };
//	
//	mux2to1 #(n) mux2 (s2, S, z, AUout); 
//
//endmodule 
