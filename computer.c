#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include "computer.h"
#undef mips			/* gcc already has a def for mips */

unsigned int endianSwap(unsigned int);

void PrintInfo (int changedReg, int changedMem);
unsigned int Fetch (int);
void Decode (unsigned int, DecodedInstr*, RegVals*);
int Execute (DecodedInstr*, RegVals*);
int Mem(DecodedInstr*, int, int *);
void RegWrite(DecodedInstr*, int, int *);
void UpdatePC(DecodedInstr*, int);
void PrintInstruction (DecodedInstr*);

/*Globally accessible Computer variable*/
Computer mips;
RegVals rVals;
Control ctrl;
///////////////////////////////////////////////////////////////////////////////////////////
/*
 *  Return an initialized computer with the stack pointer set to the
 *  address of the end of data memory, the remaining registers initialized
 *  to zero, and the instructions read from the given file.
 *  The other arguments govern how the program interacts with the user.
 */
void InitComputer (FILE* filein, int printingRegisters, int printingMemory,
		   int debugging, int interactive) {
  int k;
  unsigned int instr;

  /* Initialize registers and memory */

  for (k=0; k<32; k++) {
    mips.registers[k] = 0;
  }
    
  /* stack pointer - Initialize to highest address of data segment */
  mips.registers[29] = 0x00400000 + (MAXNUMINSTRS+MAXNUMDATA)*4;

  for (k=0; k<MAXNUMINSTRS+MAXNUMDATA; k++) {
    mips.memory[k] = 0;
  }

  k = 0;
  //Read next instruction w/ size 4 bytes (int) into buffer instr from file
  while (fread(&instr, 4, 1, filein)) {
    /*swap to big endian, convert to host byte order. Ignore this.*/
    mips.memory[k] = ntohl(endianSwap(instr));
    k++;
    if (k>MAXNUMINSTRS) {
      fprintf (stderr, "Program too big.\n");
      exit (1);
    }
  }

  mips.printingRegisters = printingRegisters;
  mips.printingMemory = printingMemory;
  mips.interactive = interactive;
  mips.debugging = debugging;
}
///////////////////////////////////////////////////////////////////////////////////////////
unsigned int endianSwap(unsigned int i) {
  return (i>>24)|(i>>8&0x0000ff00)|(i<<8&0x00ff0000)|(i<<24);
}
///////////////////////////////////////////////////////////////////////////////////////////
/*
 *  Run the simulation.
 */
void Simulate () {
  char s[40];  /* used for handling interactive input */
  unsigned int instr;
  int changedReg=-1, changedMem=-1, val;
  DecodedInstr d;
    
  /* Initialize the PC to the start of the code section */
  mips.pc = 0x00400000;
  while (1) {
    if (mips.interactive) {
      printf ("> ");
      fgets (s,sizeof(s),stdin);
      if (s[0] == 'q') {
	return;
      }
    }

    /* Fetch instr at mips.pc, returning it in instr */
    instr = Fetch (mips.pc);

    printf ("Executing instruction at %08x: %08x\n", mips.pc, instr);

    /* 
     * Decode instr, putting decoded instr in d
     * Note that we reuse the d struct for each instruction.
     */
    Decode (instr, &d, &rVals);

    /*Print decoded instruction*/
    PrintInstruction(&d);

    /* 
     * Perform computation needed to execute d, returning computed value 
     * in val 
     */
    val = Execute(&d, &rVals);

    UpdatePC(&d,val);

    /* 
     * Perform memory load or store. Place the
     * address of any updated memory in *changedMem, 
     * otherwise put -1 in *changedMem. 
     * Return any memory value that is read, otherwise return -1.
     */
    val = Mem(&d, val, &changedMem);

    /* 
     * Write back to register. If the instruction modified a register--
     * (including jal, which modifies $ra) --
     * put the index of the modified register in *changedReg,
     * otherwise put -1 in *changedReg.
     */
    RegWrite(&d, val, &changedReg);

    PrintInfo (changedReg, changedMem);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
/*
 *  Print relevant information about the state of the computer.
 *  changedReg is the index of the register changed by the instruction
 *  being simulated, otherwise -1.
 *  changedMem is the address of the memory location changed by the
 *  simulated instruction, otherwise -1.
 *  Previously initialized flags indicate whether to print all the
 *  registers or just the one that changed, and whether to print
 *  all the nonzero memory or just the memory location that changed.
 */
void PrintInfo ( int changedReg, int changedMem) {
  int k, addr;
  printf ("New pc = %08x\n", mips.pc);
  if (!mips.printingRegisters && changedReg == -1) {
    printf ("No register was updated.\n");
  } else if (!mips.printingRegisters) {
    printf ("Updated r%2.2d to %08x\n",
	    changedReg, mips.registers[changedReg]);
  } else {
    for (k=0; k<32; k++) {
      printf ("r%2.2d: %08x  ", k, mips.registers[k]);
      if ((k+1)%4 == 0) {
	printf ("\n");
      }
    }
  }
  if (!mips.printingMemory && changedMem == -1) {
    printf ("No memory location was updated.\n");
  } else if (!mips.printingMemory) {
    printf ("Updated memory at address %08x to %08x\n",
	    changedMem, Fetch (changedMem));
  } else {
    printf ("Nonzero memory\n");
    printf ("ADDR	  CONTENTS\n");
    for (addr = 0x00400000+4*MAXNUMINSTRS;
	 addr < 0x00400000+4*(MAXNUMINSTRS+MAXNUMDATA);
	 addr = addr+4) {
      if (Fetch (addr) != 0) {
	printf ("%08x  %08x\n", addr, Fetch (addr));
      }
    }
  }
  printf("\n");
}
///////////////////////////////////////////////////////////////////////////////////////////
/*
 *  Return the contents of memory at the given address. Simulates
 *  instruction fetch. 
 */
unsigned int Fetch ( int addr) {
  return mips.memory[(addr-0x00400000)/4];
}

/* Decode instr, returning decoded instruction. */
void Decode ( unsigned int instr, DecodedInstr* d, RegVals* rVals) {
  /* Your code goes here */
  unsigned int tmp, opcode, funct, shamt, rs, rt, rd, target, immed;
  int i, rtype = 0;
  int jal, j, jr, addu, addiu, subu, sll, srl, and, andi, or, ori, lui, slt, beq, bne, lw, sw;
  //Array for the funct bits
  int f[6];
  //Array for the opcode bits
  int o[6];

  //Default Control values
  ctrl.nPC_sel = '4';
  ctrl.RegWr = 0;
  ctrl.RegDst = 0;
  ctrl.ExtOp = 0;
  ctrl.ALUSrc = 0;
  ctrl.ALUCtr = -1;
  ctrl.MemWr = 0;
  ctrl.MemtoReg = 0;
  ctrl.Jump = 0;

  //decode registers (could contain garbage so fill regs w/ appropriate values depending on opcode/funct)
  opcode = instr >> 26;
  //rs register
  tmp = instr << 6;
  tmp = tmp >> 27;
  rs = tmp;
  //rt register
  tmp = instr << 11;
  tmp = tmp >> 27;
  rt = tmp;
  //rd register
  tmp = instr << 16;
  tmp = tmp >> 27;
  rd = tmp;
  //shift amount
  tmp = instr << 21;
  tmp = tmp >> 27;
  shamt = tmp;
  //function field
  tmp = instr << 26;
  tmp = tmp >> 26;
  funct = tmp;
  //target address
  tmp = instr << 6;
  tmp = tmp >> 4;
  target = tmp;
  tmp = mips.pc >> 28;
  tmp = tmp << 28;
  target = target | tmp;
  
  d->op = opcode;
  if(d->op == 0) {
    rtype = 1;
  }
  
  for(i = 0; i < 6; i++) {
    f[5-i] = (funct >> i) & 1;
    o[5-i] = (opcode >> i) & 1;
  }

  //Used boolean algebra to reduce some of the expressions, may go back and reduce further if time
  j = ~(o[0]|o[1]|o[2]|o[3]|o[5]) & o[4];
  jal = ~(o[0]|o[1]|o[2]|o[3]) & o[4] & o[5];
  jr = rtype & f[2] & ~(f[0]|f[1]|f[3]|f[4]|f[5]);
  addu = rtype & f[0] & f[5] & ~(f[1]|f[2]|f[3]|f[4]);
  subu = rtype & f[0] & f[4] & f[5] & ~(f[1]|f[2]|f[3]);
  sll = rtype & ~(f[0]|f[1]|f[2]|f[3]|f[4]|f[5]);
  srl = rtype & f[4] & ~(f[0]|f[1]|f[2]|f[3]|f[5]);
  and = rtype & f[0] & f[3] & ~(f[1]|f[2]|f[4]|f[5]);
  or = rtype & f[0] & f[3] & f[5] & ~(f[1]|f[2]|f[4]);
  slt = rtype & f[0] & f[2] & f[4] & ~(f[1]|f[3]|f[5]);
  addiu = o[2] & o[5] & ~(o[0]|o[1]|o[3]|o[4]);
  andi = o[2] & o[3] & ~(o[0]|o[1]|o[4]|o[5]);
  ori = o[2] & o[3] & o[5] & ~(o[0]|o[1]|o[4]);
  lui = o[2] & o[3] & o[4] & o[5] & ~(o[0]|o[1]);
  beq = o[3] & ~(o[0]|o[1]|o[2]|o[4]|o[5]);
  bne = o[3] & o[5] & ~(o[0]|o[1]|o[2]|o[4]);
  lw = o[0] & o[4] & o[5] & ~(o[1]|o[2]|o[3]);
  sw = o[0] & o[2] & o[4] & o[5] & ~(o[1]|o[3]);

  //Jump triggers
  ctrl.Jump = j|jal|jr;
  //RegWr = 1 -> write to reg rd or rt, selected by RegDst
  ctrl.RegWr = addu|subu|sll|srl|and|or|slt|addiu|andi|ori|lui|lw|jal;
  //RegDst = 1 -> write to rd, 0 -> write to rt
  ctrl.RegDst = addu|subu|sll|srl|and|or|slt;
  //ExtOp = 1 -> SignExt, 0 -> ZeroExt
  ctrl.ExtOp = lw|sw|addiu;
  //ALUSrc = 1 -> immed, 0 -> rt
  ctrl.ALUSrc = addiu|andi|ori|lui|beq|bne|lw|sw;
  //ALUCtr = 0 -> ADD, 1 -> SUB, 2 -> AND, 3 -> OR
  if(addu|addiu|lw|sw) {
    ctrl.ALUCtr = ADD;
  } else if(subu|slt|beq|bne) {
    ctrl.ALUCtr = SUB;
  } else if(and|andi) {
    ctrl.ALUCtr = AND;
  } else if(or|ori) {
    ctrl.ALUCtr = OR;
  } else if(sll|srl|lui) {
    ctrl.ALUCtr = SHIFT;
  }
  //MemWr = 1 -> Write to mem
  ctrl.MemWr = sw;
  //MemtoReg = 1 -> ALU, 0 -> Mem
  ctrl.MemtoReg = addu|addiu|subu|sll|srl|and|andi|or|ori|lui|slt|jal|lw;

  //Immediate sign extension taken care of in decoding not in execution
  tmp = instr << 16;
  tmp = tmp >> 16;
  if(ctrl.ExtOp) {
    //if 15th bit is 1, sign extend w/ 1
    if(tmp >> 15) {
      tmp = tmp | 0xFFFF0000;
    } else {
      //else sign extend 0
      tmp = tmp & 0x0000FFFF;
    }
  } else {
    tmp = tmp & 0x0000FFFF;
  }
  immed = tmp;

  if(j|jal|jr) {
    ctrl.nPC_sel = 'j';
  }

  //Fill DecodedInstr struct w/ appropriate values
  if(d->op == 0) {
    //R type instruction (we're using rs, rt, rd, and shamt probably)
    d->type = R;
    d->regs.r.rs = rs;
    d->regs.r.rt = rt;
    d->regs.r.rd = rd;
    d->regs.r.shamt = shamt;
    d->regs.r.funct = funct;
  } else if (d->op != 2 && d->op != 3) {
    d->type = I;
    d->regs.i.rs = rs;
    d->regs.i.rt = rt;
    d->regs.i.addr_or_immed = immed;
  } else {
    d->type = J;
    d->regs.j.target = target;
  }

  //Leaving this line in here to remind you that if you try to write out the boolean algebraic expressions for these flags and try to simplify them, you're going to have a bad day.
  //ctrl.Jump = (~(o[0]|o[1]|o[2]|o[3])&o[4]) | (~(o[0]|o[1]|o[2]|o[3]|o[4]|o[5])&~(f[0]|f[1]|f[3]|f[4]|f[5])&f[2]);  
  //Update rVals struct
  rVals->R_rs = mips.registers[rs];
  rVals->R_rt = mips.registers[rt];
  rVals->R_rd = mips.registers[rd];

  /* if(d->type == R) { */
  /*   printf("R-type instruction: rs: %d, rt: %d, rd: %d contains %d, %d, %d\n", rs, rt, rd, rVals->R_rs, rVals->R_rt, rVals->R_rd); */
  /* } else if(d->type == I) { */
  /*   printf("I-type instruction: rs: %d, rt: %d, immed: 0x%08x contains %d, %d\n", rs, rt, immed, rVals->R_rs, rVals->R_rt); */
  /* } else if(d->type == J) { */
  /*   printf("J-type instruction: target: 0x%08x\n", target); */
  /* } */
}
///////////////////////////////////////////////////////////////////////////////////////////
/*
 *  Print the disassembled version of the given instruction
 *  followed by a newline.
 */
void PrintInstruction ( DecodedInstr* d) {
  /* Your code goes here */
  //Max size of instruction name + 1
 
  char* instr = (char*) malloc(6*sizeof(char));
  int supported = 1;
  switch(d->op) {
  case 0: 
    switch(d->regs.r.funct) {
    case 0:
      instr = "sll";
      break;
    case 2:
      instr = "srl";
      break;
    case 8:
      instr = "jr";
      break;
    case 33:
      instr = "addu";
      break;
    case 35:
      instr = "subu";
      break;
    case 36:
      instr = "and";
      break;
    case 37:
      instr = "or";
      break;
    case 42:
      instr = "slt";
      break;
    default:
      supported = 0;
      break;
    }
    break;
  case 2:
    instr = "j";
    break;
  case 3:
    instr = "jal";
    break;
  case 4:
    instr = "beq";
    break;
  case 5:
    instr = "bne";
    break;
  case 9:
    instr = "addiu";
    break;
  case 12:
    instr = "andi";
    break;
  case 13:
    instr = "ori";
    break;
  case 15:
    instr = "lui";
    break;
  case 35:
    instr = "lw";
    break;
  case 43:
    instr = "sw";
    break;
  default:
    supported = 0;
    break;
  }

  if(supported == 0) {
    printf("Unsupported instruction. Terminating.\n");
    exit(0);    
  }

  if(d->type == R) {
    printf("%s\t$%d, $%d, $%d\n", instr, d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
  } else if(d->type == I) {
    if(d->op == 35 || d->op == 43) {
      printf("%s\t$%d, %d($%d)\n", instr, d->regs.i.rt, d->regs.i.addr_or_immed, d->regs.i.rs);
    } else if(d->op == 12 || d->op == 13 || d->op == 15){
      printf("%s\t$%d, $%d, 0x%x\n", instr, d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);
    } else if(d->op == 4 || d->op == 5) {
       printf("%s\t$%d, $%d, 0x%08x\n", instr, d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);
    } else {
      printf("%s\t$%d, $%d, %d\n", instr, d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);
    }
  } else if(d->type == J) {
    printf("%s\t0x%08x\n", instr, d->regs.j.target);
  } 
  
}
///////////////////////////////////////////////////////////////////////////////////////////
/* Perform computation needed to execute d, returning computed value */
int Execute ( DecodedInstr* d, RegVals* rVals) {
  /* Your code goes here */
  int rs, rt, shamt, immed, diff, result, bitmask, i;
  //Load values
  if(d->type == R) {
    rs = d->regs.r.rs;
    rt = d->regs.r.rt;
    shamt = d->regs.r.shamt;
  } else if(d->type == I) {
    rs = d->regs.i.rs;
    rt = d->regs.i.rt;
    immed = d->regs.i.addr_or_immed;
  }
  //Handle R, I, J type instructions
  if(d->type == R) {
    if(ctrl.ALUCtr == ADD) {
      result = mips.registers[rs] + mips.registers[rt];
    } else if(ctrl.ALUCtr == SUB) {
      diff = mips.registers[rs] - mips.registers[rt];
      if(d->regs.r.funct == 42) {//slt
	if(diff < 0) {
	  result = 1;
	} else {
	  result = 0;
	}
      } else {
	result = diff;
      }
    } else if(ctrl.ALUCtr == AND) {
      result = mips.registers[rs] & mips.registers[rt];
    } else if(ctrl.ALUCtr == OR) {
      result = mips.registers[rs] | mips.registers[rt];
    } else if(ctrl.ALUCtr == SHIFT) {
      if(d->regs.r.funct == 0) {
	result = mips.registers[rt] << shamt;
      } else {
	bitmask = 0;
	for(i = 0; i < 32-shamt; i++) {
	  bitmask = (bitmask << i) | 1;
	}
	result = (mips.registers[rt] >> shamt) & bitmask;
      }
    }
    if(d->regs.r.funct == 8) {      
      result = mips.registers[31];
    }
  } else if(d->type == I) {
    if(ctrl.ALUCtr == ADD) {
      if(d->op == 35 || d->op == 43) {
	result = mips.registers[rs] - (immed/4+1)*4;
      } else {
	result = mips.registers[rs] + immed;
      }
    } else if(ctrl.ALUCtr == SUB) {
      diff = mips.registers[rs] - mips.registers[rt];
      if(diff == 0 && d->op == 4) {
	ctrl.nPC_sel = 'b';
	result = mips.pc+4+4*d->regs.i.addr_or_immed;
      } else if(diff != 0 && d->op == 5) {
	ctrl.nPC_sel = 'b';
	result = mips.pc+4+4*d->regs.i.addr_or_immed;
      }
    } else if(ctrl.ALUCtr == AND) {
      result = mips.registers[rs] & immed;
    } else if(ctrl.ALUCtr == OR) {
      result = mips.registers[rs] | immed;
    } else if(ctrl.ALUCtr == SHIFT) {
      result = (immed << 16) & 0xFFFF0000;
    }
  } else if(d->type == J) {
    if(d->op == 3) {//jal
      result = mips.pc + 4;
      //printf("Updating Reg 31 to 0x%08x\n", result);
    }
  }

  return result;
}
///////////////////////////////////////////////////////////////////////////////////////////
/* 
 * Update the program counter based on the current instruction. For
 * instructions other than branches and jumps, for example, the PC
 * increments by 4 (which we have provided).
 */
void UpdatePC ( DecodedInstr* d, int val) {
  /* Your code goes here */  
  if(ctrl.nPC_sel == 'j') {
    //printf("Jump instruction\n");
    if(d->type == R) {
      mips.pc = val;
    } else {
      mips.pc = d->regs.j.target;
    }
  } else if(ctrl.nPC_sel == 'b') {
    //printf("Branch instruction\n");
    mips.pc = val;
  } else {
    mips.pc+=4;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
/*
 * Perform memory load or store. Place the address of any updated memory 
 * in *changedMem, otherwise put -1 in *changedMem. Return any memory value 
 * that is read, otherwise return -1. 
 *
 * Remember that we're mapping MIPS addresses to indices in the mips.memory 
 * array. mips.memory[0] corresponds with address 0x00400000, mips.memory[1] 
 * with address 0x00400004, and so forth.
 *
 */
int Mem( DecodedInstr* d, int val, int *changedMem) {
  /* Your code goes here */
  //if lw or sw then deal with memory. Otherwise don't touch val
  int rt, addr;
  //Default to no changes
  *changedMem = -1;
  
  rt = d->regs.i.rt;
  if(d->op == 43) {
    if(val < 0x00401000 || val > 0x00404000 || val % 4 != 0) {
      printf("Memory Access Exception at [0x%08x]: address [0x%08x]\n", mips.pc, val);
      exit(0);
    }
    addr = (val-0x00400000)/4;
    mips.memory[addr] = mips.registers[rt];
    *changedMem = val;
  } else if(d->op == 35) {
    //Only lw changes val since it's reading from memory to write to registers
    if(val < 0x00401000 || val > 0x00404000 || val % 4 != 0) {
      printf("Memory Access Exception at [0x%08x]: address [0x%08x]\n", mips.pc, val);
      exit(0);
    }
    mips.registers[rt] = Fetch(val);
    *changedMem = -1;
    val = mips.registers[rt];
  }

  return val;
}
///////////////////////////////////////////////////////////////////////////////////////////
/* 
 * Write back to register. If the instruction modified a register--
 * (including jal, which modifies $ra) --
 * put the index of the modified register in *changedReg,
 * otherwise put -1 in *changedReg.
 */
void RegWrite( DecodedInstr* d, int val, int *changedReg) {
  /* Your code goes here */
  int rt = 0, rd = 0;
  if(d->type == R) {
    rd = d->regs.r.rd;
  } else if(d->type == I) {
    rt = d->regs.i.rt;
  }
  //Default to no changes
  *changedReg = -1;
  if(ctrl.RegWr) {    
    if(d->type == R) {
      mips.registers[rd] = val;
      *changedReg = rd;
    } else if(d->type == I) {
      mips.registers[rt] = val;
      *changedReg = rt;
    }
    //Aside from jal, no other jump instructions should write to registers
    if(d->op == 3) {
      mips.registers[31] = val;
      *changedReg = 31;
    }
  }

}
