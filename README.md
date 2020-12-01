# 18-bit Microprocesor
## Overview
This project is to design and implement the instruction set architecture of a 18-bit microprocessor. The processor is desgined to be **single cycle**, and the instructions coverage include : 
- R-Type ALU instructions
- R-Type Multiplier and Divider instructions
- R-Type Shifter instructions
- R-Type Jump instructions
- I-Type ALU instructions
- Memory Access instructions
- Type-2 Branch instructions
- Type-1 Branch instructions
- J-Type Jump instructions 
The number of registers is **16**, and the dataword width is **16 bits** as well.  
## Testing  
### Requirements 
- Software : [Quartus Prime Standard Edition 18.1](https://fpgasoftware.intel.com/18.1/?edition=standard)
- Hardware : [Cyclone V GX Starter Kit](https://www.intel.com/content/www/us/en/programmable/solutions/partners/partner-profile/terasic-inc-/board/cyclone-v-gx-starter-kit.html#:~:text=Button%2C%20Slider%20Switch-,Overview,logic%20for%20ultimate%20design%20flexibility.)
### Steps 
1. Clone this repository. 
2. Import this repository as a project into the Quartus Prime 18.1 workspace. 
3. Compile the program. 
4. Run the programmer and load the program into your Cyclove V GX. 
## Creating your own Instructions 
### Bits field of an instruction 
- bit[17:12] : Opcode 
- bit[11:8] : IMM / SRC_REG2 
- bit[7:4] : SRC_REG1 
- bit[3:0] : DEST_REG
For example, ADDI 2 1 3 would be ADDing IMM value 2 to the content stored in register 1, and assign the content into destination register 3. 
## Reference 
- [Report](https://github.com/emiltan97/18-bit-microprocessor/blob/master/18-Bit%20Microprocessor.pdf)
- [Instruction Set](https://github.com/emiltan97/18-bit-microprocessor/blob/master/instruction%20set.xlsx)
