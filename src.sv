// TOP LEVEL MODULE
module src(
	input  logic [9:0] SW, 
	input  logic [3:0] KEY, 
	input  logic CLOCK_50,
   output logic [9:0] LEDR,
   output logic [7:0] LEDG,
	output logic [6:0] HEX0, HEX1, HEX2, HEX3
); 

	logic [31:0] q; 
	
	counter32 myclock(CLOCK_50, q); 
	
	assign LEDG[0] = q[25]; 
	
	CPU mycpu(q[25], HEX0, HEX1, HEX2, HEX3, LEDR);
	
endmodule
// Slower clock
module counter32 (
  input logic clk,
  output logic [31:0] q
);

  always_ff @(posedge clk)
    q <= q+1;

endmodule
// CPU
module CPU (
	input  logic clk, 
	output logic [6:0] HEX0, HEX1, HEX2, HEX3,
	output logic [9:0] LEDR
); 

	logic pc_carry, funct; 
	logic [3:0] rprio;
	logic [3:0] ALUSrc; 
	logic [4:0] opcode;
	logic [5:0] opprio;
	logic [15:0] pc, pc_plus_one, pc_next;
	logic [3:0] RA1, RA2, WA, Imm; 
	logic [15:0] RD1, RD2, WD;
	logic [15:0] ZeroImm, CompImm, SignImm; 
	logic [15:0] immOut0, immOut1, immOut2;
	logic [15:0] shiftout, multout, ALUout;
	logic [17:0] instruction;

	always_ff @(posedge clk) 
		begin
			pc <= pc_next; 
		end
	
	ROM #(16, 18) myrom(pc, instruction); 
	
	assign funct  = instruction[12];
	assign opcode = instruction[17:13];
	assign Imm    = instruction[11:8];
	assign RA1    = instruction[11:8]; 
	assign RA2    = instruction[7:4]; 
	assign WA     = instruction[3:0]; 
	
	OpPrioDecoder myopdecoder(opcode, opprio); 
	RTypePrioDecoder myrtypedecoder(opcode[3:1], rprio); 
	assign LEDR[5:0] = opprio[5:0]; 
	
	regfile #(16, 4) myreg(clk, RA1, RA2, WA, WD, RD1, RD2); 
	
	assign ALUSrc[0] = instruction[12] & instruction[13]; 
	assign ALUSrc[1] = instruction[14]; 
	assign ALUSrc[2] = instruction[17]; 
	
	immExtend myimm(Imm, ZeroImm, CompImm, SignImm);
	mux2to1 #(16) mux0 (ALUSrc[0], ZeroImm, CompImm, immOut0); 
	mux2to1 #(16) mux1 (ALUSrc[1], SignImm, immOut0, immOut1); 
	mux2to1 #(16) mux2 (ALUSrc[2], immOut1, RD1, immOut2); 

	ALU #(16) myalu({opcode[2:0], funct}, RD2, immOut2, ALUout);
	shifter #(16) myshift({opcode[0], funct}, RD2, RD1, shiftout);
	
	tristate_active_hi#(16) ALUTest(ALUout, rprio[3], WD); 
	tristate_active_hi#(15) ShiftTest(shiftout, rprio[1], WD); 
	
	ssd ins0(WD[3:0], HEX0); 
	ssd ins1(WD[7:4], HEX1); 
	ssd ins2(WD[11:8], HEX2); 
	ssd ins3(WD[15:12], HEX3); 
	
	assign {pc_carry, pc_plus_one} = pc + 1; 
	assign pc_next = pc_plus_one; 
	
endmodule
// ROM
module ROM #(parameter m=7,w=4) (
  input  logic [m-1:0] Ad,
  output logic [w-1:0] Dout
);
  logic [w-1:0] mem[2**m-1:0];
  assign Dout = mem[Ad];
  
  initial begin
    $readmemb("instructions.txt",mem); 
  end

endmodule
// Register file 
module regfile
#(parameter w=4, m=4) (
	input logic clk,
	input logic [m-1:0] RAd0, RAd1, WAd,
	input logic [w-1:0] Din,
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
// ALU 
module ALU #(parameter n = 4) (
	input  logic [3:0] F, 
	input  logic [n - 1:0] A, B,
	output logic [n - 1:0] Y
); 

	logic [n - 1:0] S, AUout, logicOut; 
	logic OVs; 

	adderOV #(n) add0(F[1:0], A, B, Cout, OV, OVs, S);
	SLT #(n) slt0(OVs, Cout, F[0], F[3], S, AUout);
	mux4to1 #(n) mux0(
		(A & B), (A | B), (A^B), ~(A | B), 
		F[1:0], 
		logicOut
	);
	mux2to1 #(n) mux1(F[2], AUout, logicOut, Y);
	
endmodule 
// Adder with overflow module
module adderOV #(parameter n = 4) (
	input  logic [1:0] s, 
	input  logic [n - 1:0] a, b, 
	output logic Cout, OV, OVs,
	output logic [n - 1:0] S
);

	logic OVu;
	logic [n - 1:0] B;
	
	adder #(n) add0(a, b, s[1], S, B, Cout); 
	
	assign OVs = (S[n-1] ^ a[n-1]) & ~(a[n-1] ^ B[n-1]);
	assign OVu = s[1] ^ Cout; 
	
	mux2to1 #(n) mux1(s[0], OVs, OVu, OV);

endmodule
// Adder module in ALU
module adder #(parameter W = 4) (
	input  logic [W - 1:0] A, B,
	input  logic s,
	output logic [W - 1:0] Y,	
	output logic [W - 1:0] b,
	output logic cout
); 


	mux2to1 #(W) mux1(s, B, ~B, b);

	assign {cout,Y} = A + b + s;

endmodule 
// SLT module in ALU
module SLT #(parameter n = 4) (
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
// Opcode Priority Instruction 
module OpPrioDecoder (
	input  logic [5:0] a, 
	output logic [6:0] b
); 
	
	logic [4:0] e, f; 
	
	assign e[0] = (~a[4] & ~a[5]);
	assign e[1] = ( a[4] & ~a[5]);
	assign e[2] = a[5];
	
	assign f[0] = (~a[0] & ~a[1] & ~a[2] & ~a[3]);
	assign f[1] = ( a[0] & ~a[1] & ~a[2] & ~a[3]);
	assign f[2] = ( a[1] & ~a[2] & ~a[3]); 
	assign f[3] = ( a[2] & ~a[3]);
	assign f[4] = a[3];
	
	assign b[6:5] = e[2:1];
	assign b[4:0] = f[4:0] & {5{e[0]}};

endmodule
// RType Instructions Priority
module RTypePrioDecoder (
	input  logic [2:0] instruction, 
	output logic [3:0] outcome
);

	logic [1:0] a,b;

	assign b[1] =   instruction[2];
	assign b[0] = (~instruction[2] & instruction[1]);

	assign a[1] = ~instruction[2] & (~instruction[1] &  instruction[0]);
	assign a[0] = ~instruction[2] & (~instruction[1] & ~instruction[0]);

	assign outcome[3:2] = b[1:0];
	assign outcome[1:0] = a[1:0];

endmodule
// Decoder 
module parameterized_decoder
#(parameter n=3) (
	input logic [n-1:0] a,
	output logic [2**n-1:0] y
);
	parameter w = 2**n;
	always_comb begin
		y = {w{1'b0}};
		y[a] = 1'b1;
	end
endmodule
// Register Enabler
module EnabledReg
#(parameter w=3) (
	input logic clk, en,
	input logic [w-1:0] D,
	output logic [w-1:0] Q
);
	always @(posedge clk) 
		if(en) Q <= D;

endmodule
// SSD
module ssd (
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
// Immediate constant extension 
module immExtend(
	input  logic [3:0] Imm, 
	output logic [15:0] ZeroImm,CompImm,SignImm
);

	assign ZeroImm = {12'b0, Imm};
	assign SignImm = {{12{Imm[3]}}, Imm};
	assign CompImm = {~Imm, {12{1'b1}}};

endmodule
// Tristate decoder 
module tristate_active_hi #(parameter n = 16) (
	input  logic [n-1:0] a,
	input  logic en,
	output tri [n-1:0] y
);

	assign y = en? a:16'bz;
	
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