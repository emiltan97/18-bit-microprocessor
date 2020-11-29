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
	logic [15:0] oup; 
	
	counter32 myclock(CLOCK_50, q); 
	
	assign LEDG[0] = SW[0] ? KEY[0] : q[25]; 
	
	CPU mycpu(SW[0] ? KEY[0] : q[25], oup, LEDR[5:0], LEDR[9:6]);
	
	ssd ins0(oup[3:0], HEX0);
	ssd ins1(oup[7:4], HEX1);
	ssd ins2(oup[11:8], HEX2);
	ssd ins3(oup[15:12], HEX3);

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
	output logic [15:0] outport,
	output logic [5:0] opprio,
	output logic [3:0] rprio
); 

	logic pc_carry, funct, jump, branch, MemEn, isBranch, jumpWrite; 
	logic [3:0] ALUSrc; 
	logic [4:0] opcode;
	logic [15:0] pc, pc_plus_one, JTA, pc_next, BTA;
	logic [3:0] RA1, RA2, WA, Imm; 
	logic [15:0] RD1, RD2, WD;
	logic [15:0] ZeroImm, CompImm, SignImm; 
	logic [15:0] immOut0, immOut1, immOut2;
	logic [15:0] shiftout, multout, ALUout, opout; // opout stands for operation output
	logic [17:0] instruction;
	logic [15:0] RAMout, MemDin, MemRd;
	
	always_ff @(posedge clk) 
		begin
			pc <= pc_next; 
			outport <= WD; 
		end
	
	ROM #(4, 18) myrom(pc, instruction); 
	assign {pc_carry, pc_plus_one} = pc + 1; 
	assign funct  = instruction[12];
	assign opcode = instruction[17:13];
	
	OpPrioDecoder myopdecoder(opcode, opprio); 
	RTypePrioDecoder myrtypedecoder(opcode[3:1], rprio);  
	
	assign isBranch = opprio[1] | opprio[2];
	assign jump = (opprio[5] & rprio[0]) | opprio[0];
	mux2to1 #(4) branchmux0(isBranch, instruction[11:8], instruction[3:0], RA1);
	assign Imm    = instruction[11:8];
	assign RA2    = instruction[7:4]; 
	mux2to1 #(4) jumpmux0(jump & funct, instruction[3:0], 4'b1111, WA); 
	assign JTA    = {4'b0, instruction[11:0]}; 
 
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
	multiDiv #(16) mymult(RD2, RD1, clk, {opcode[1:0], funct}, multout);
	
	tristate_active_hi #(16) ALUTest(ALUout, rprio[3], opout); 
	tristate_active_hi #(16) ShiftTest(shiftout, rprio[1] & instruction[17], opout); 
	tristate_active_hi #(16) MultTest(multout, rprio[2] & instruction[17], opout);
	
	assign BTA  = pc_plus_one + (SignImm << 2);
	
	logic [15:0] js; // jump signal
	logic [15:0] jb; // jump to branch signal 
	logic [15:0] jm; // Jump to mem signal
	
	BranchControl mybranchctrl(RD1, RD2, funct, opcode[0], opprio[1], opprio[2], branch); 

	mux2to1 #(16) jtamux1(opprio[5], JTA, RD1, js); 
	mux2to1 #(16) jtamux2(~jump, js, pc_plus_one, jb); 
	mux2to1 #(16) jtamux3(branch, jb, BTA, pc_next);
	
	MemoryAccess mem1(RD2, RAMout, instruction, ALUout[0], MemRd, MemDin, MemEn);
	RAM #(6, 16) mem2 (ALUout[6:1], MemDin, clk, MemEn, RAMout);
	
	mux2to1 #(16) jumpmux1(jump & funct, opout, pc_plus_one, jm); 
	mux2to1 #(16) memmux1(opprio[3], jm, MemRd, WD);
	
endmodule
// ROM
module ROM #(parameter m=7,w=4) (
  input  logic [m-1:0] Ad,
  output logic [w-1:0] Dout
);
  logic [w-1:0] mem[2**m-1:0];
  assign Dout = mem[Ad];
  
  initial begin
    $readmemb("instruction_mult.txt",mem); 
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
		EnabledReg #(w) itk(clk, WEn[k], Din,Q[k]);
	end
	endgenerate
		
endmodule
// ALU
module ALU#(parameter n = 4)(
	input  logic [3:0] S, 
	input  logic [n-1:0] A,
	input  logic [n-1:0] B, 
	output logic [n-1:0] Y
);

	logic Cin, Cout;
	logic OVs, Ovu;
	logic [n-1:0] Result, LogicOut, SLTResult;
	logic [n-1:0] B_out;
	logic [n-1:0] D0,D1;


	assign D0 = B;
	assign D1 = ~B;
	assign CoutFlag = Cout;

	mux2to1 #(n) ALUmux(S[1], D0, D1, B_out);

	assign Cin = S[1];
	ALU_Adder #(n)Test123(A,B_out,Cin, Result,Cout);
	assign OVs = (Result[n-1] ^ A[n-1]) & ~(A[n-1] ^ B_out[n-1]);
	assign Ovu = S[1] ^ Cout;

	mux2to1 #(n) OVmux(S[0], OVs, Ovu, Z);

	SLTout#(n) SLTTest(Result, Cout, OVs, S[0], (S[3]&S[1]), SLTResult);

	mux4_1Test#(n) LogicOutcome(A,B,S[0],S[1],LogicOut);

	mux2to1 #(n) ResultLogicmux2(S[2], SLTResult, LogicOut, Y);

endmodule


module ALU_Adder#(parameter n = 4)(
	input  logic [n-1:0] a,b, 
	input  logic Cin, 
	output logic [n-1:0] Y, 
	output logic Cout
);

	assign {Cout,Y} = a + b + Cin;

endmodule

 
module SLTout#(parameter n = 4)(
	input  logic [n-1:0] Result, 
	input  logic Cout, OVs, S0, S3, 
	output logic [n-1:0] SLToutcome
);

	logic outcome0,outcome1;
	logic [n-1:0] ZeroExt;

	mux2to1 #(n) SLTmux(OVs, Result[n-1], Cout, outcome0);

	mux2to1 #(n) SLTmux2(S0, outcome0, ~Cout, outcome1);

	assign ZeroExt = {{(n-1){1'b0}}, outcome1};

	mux2to1 #(n) SLTmux3(S3, Result, ZeroExt, SLToutcome);

endmodule
module mux4_1Test #(parameter n = 4) (
	input  logic [n-1:0] A,B, 
	input  logic [n-1:0] F0,F1,
	output logic [n-1:0] Y
);
	logic [n-1:0] lo, hi;
	logic [n-1:0] D0,D1,D2,D3;

	assign D0 = A&B;
	assign D1 = A|B;
	assign D2 = A^B;
	assign D3 = ~(A|B);

	mux2to1 #(n) lomux (F0, D0, D1, lo); //width = W
	mux2to1 #(n) himux (F0, D2, D3, hi); //width = W
	mux2to1 #(n) oumux (F1, lo, hi, Y); //width = W

endmodule
// Shifter
module shifter #(parameter n = 4) (
	input  logic [1:0] F, 
	input  logic [n - 1:0] A, 
	input  logic [n - 1:0] Sh, 
	output logic [n - 1:0] Y 
);

	logic [n - 1:0] D0, D1; 
	
	assign D0 = F[0]?  (A >> Sh)           : (A << Sh);
	assign D1 = F[0]?  ($signed(A) >>> Sh) : (A);
	assign Y  = F[1]?  D1                  : D0;

endmodule
// Multiplier & Divider 
module multiDiv #(parameter n = 16)(
	input  logic [n-1:0] a, 
	input  logic [n-1:0] b, 
	input  logic clk, 
	input  logic [2:0] F, 
	output logic [n-1:0] y
);

	logic [n-1:0] B,H,C,L,R,Q;
	logic [n-1:0] D[3:0];
	logic [n-1:0] hi,lo, en1,en2;

	assign en1 = F[2] | ( F[1] & F[0]);
	assign en2 = F[2] | (~F[1] & F[0]);

	assign B = a;
	assign C = b;

	assign {H,L} = B * C;
	assign  Q    = B / C;
	assign  R    = B % C;

	mux2to1 #(n) mux1 (F[1], H, R, D[0]);
	mux2to1 #(n) mux2 (F[1], L, Q, D[1]);
	mux2to1 #(n) mux3 (F[2], a, D[0], D[2]);
	mux2to1 #(n) mux4 (F[2], a, D[1], D[3]);

	always_ff @(posedge clk) 
		begin
			if (en1) hi <= D[2];
			if (en2) lo <= D[3];
		end

	mux2to1 #(n) mux5 (F[1], hi, lo, y);

endmodule
// Branch control signals 
module BranchControl (
	input  logic [15:0] a, b, 
	input  logic funct, op0, b1, b2, 
	output logic branch
);

	logic [15:0] y [9:0];

	// type 2 branch control signals
	assign y[0] = a ^ b; 
	assign y[1] = ~(|y[0]); 
	assign y[2] = funct ^ y[1]; 
	
	assign y[3] = ~(|b); 
	assign y[4] = b[15] | y[3]; 
	assign y[5] = y[4] ^ funct; 
	
	mux2to1 #(16) inst0(op0, y[2], y[5], y[6]); 
	
	assign y[7] = y[6] & b2;
	
	// type 1 branch control signals 
	assign y[8] = a[0] ^ b[15]; 
	assign y[9] = y[8] & b1; 
	
	assign branch = y[7] | y[9]; 

endmodule
// Opcode Decoder Priority
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
//		if(en & enable) Q <= D;
		if(en) Q <= D;

endmodule
// Mem access memin control signals
module MemoryIn(
	input logic  [15:0] WB0, WB1, WD, Ad, Op12, 
	output logic [15:0] MemIn
);

	logic [15:0] Y;
	mux2to1 #(16) MemIn1(Ad, WB0, WB1, Y);
	mux2to1 #(16) MemIn2(Op12, Y, WD, MemIn);

endmodule
// RAM module
module RAM #(parameter N = 5, W = 8) (
	input  logic [N-1:0] Ad, 
	input  logic [W-1:0] Din, 
	input  logic clk, En, 
	output logic [W-1:0] Dout
); 

	logic [W-1:0] array[2**N - 1:0]; 
	assign Dout = array[Ad]; 
	
	always_ff @(posedge clk)
		if(En) array[Ad] <= Din; 

endmodule 
// Memory access top module
module MemoryAccess(
	input  logic [15:0] WD, Mout, 
	input  logic [17:0] instruction, 
	input  logic Ad, 
	output logic [15:0] Rd, MemIn,
	output logic MemEn 
); 

	logic [15:0] WB0, WB1, SignB0, SignB1, ZeroB0, ZeroB1; 
	logic s1, s2; 

	assign s2 = instruction[12]; 
	assign s1 = instruction[13]; 
	assign MemEn = instruction[14] | instruction[15];
	
	assign WB0 = {Mout[15:8], WD[7:0]};
	assign WB1 = {WD[15:8], Mout[7:0]}; 
	
	assign SignB0 = {{8{Mout[7]}}, Mout[7:0]}; 
	assign SignB1 = {{8{Mout[15]}}, Mout[15:8]}; 
	
	assign ZeroB0 = {8'b0, Mout[7:0]}; 
	assign ZeroB1 = {8'b0, Mout[15:8]}; 
	
	MemoryIn mymem (WB0, WB1, WD, Ad, s2, MemIn); // Memory In control signals 

	// RD control signals
	logic [15:0] x, y, z; 

	mux2to1 #(16) Rd1(s1, SignB0, ZeroB0, x);
	mux2to1 #(16) Rd2(s1, SignB1, ZeroB1, y);
	mux2to1 #(16) Rd3(Ad, x, y, z);
	mux2to1 #(16) Rd4(s2, z, Mout, Rd);

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