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

unsigned int endianSwap(unsigned int i) {
    return (i>>24)|(i>>8&0x0000ff00)|(i<<8&0x00ff0000)|(i<<24);
}

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

        printf ("Executing instruction at %8.8x: %8.8x\n", mips.pc, instr);

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
    printf ("New pc = %8.8x\n", mips.pc);
    if (!mips.printingRegisters && changedReg == -1) {
        printf ("No register was updated.\n");
    } else if (!mips.printingRegisters) {
        printf ("Updated r%2.2d to %8.8x\n",
        changedReg, mips.registers[changedReg]);
    } else {
        for (k=0; k<32; k++) {
            printf ("r%2.2d: %8.8x  ", k, mips.registers[k]);
            if ((k+1)%4 == 0) {
                printf ("\n");
            }
        }
    }
    if (!mips.printingMemory && changedMem == -1) {
        printf ("No memory location was updated.\n");
    } else if (!mips.printingMemory) {
        printf ("Updated memory at address %8.8x to %8.8x\n",
        changedMem, Fetch (changedMem));
    } else {
        printf ("Nonzero memory\n");
        printf ("ADDR	  CONTENTS\n");
        for (addr = 0x00400000+4*MAXNUMINSTRS;
             addr < 0x00400000+4*(MAXNUMINSTRS+MAXNUMDATA);
             addr = addr+4) {
            if (Fetch (addr) != 0) {
                printf ("%8.8x  %8.8x\n", addr, Fetch (addr));
            }
        }
    }
}

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
  int i;
  int bits[6];

  //Default Control values
  ctrl.nPC_sel = '4';
  ctrl.RegWr = 0;
  ctrl.RegDst = 0;
  ctrl.ExtOp = 0;
  ctrl.ALUSrc = 0;
  ctrl.MemWr = 0;
  ctrl.MemtoReg = 0;

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
  //immediate
  tmp = instr << 16;
  tmp = tmp >> 16;
  immed = tmp;
  //target address
  tmp = instr << 6;
  tmp = tmp >> 4;
  target = tmp;
  tmp = mips.pc >> 28;
  tmp = tmp << 28;
  target = target | tmp;
  
  d->op = opcode;

  if(opcode == 0) {
    //rtype = 1;
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
  
  for(i = 0; i < 6; i++) {
    bits[5-i] = (funct >> i) & 1;
  }

  if(d->type == R) {
    printf("Decoded instruction: %8.8x has opcode: %d, rs: %d, rt: %d, rd: %d, shamt: %d, funct: %d -> ", instr, opcode, rs, rt, rd, shamt, funct);
    for(i = 0; i < 6; i++) {
      printf("%d", bits[i]);  
    }
    printf("\n");
  } else if(d->type == I) {
    printf("Decoded instruction: %8.8x has opcode: %d, rs: %d, rt: %d, immed: %d\n", instr, opcode, rs, rt, immed);
  } else {
    printf("Decoded instruction: %8.8x has opcode: %d, target: %8.8x\n", instr, opcode, target);
  }
}

/*
 *  Print the disassembled version of the given instruction
 *  followed by a newline.
 */
void PrintInstruction ( DecodedInstr* d) {
    /* Your code goes here */
  //Max size of instruction name + 1
  printf("Printing instruction\n");
  char* instr = (char*) malloc(6*sizeof(char));
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
    break;
  }

  //instr[5] = 0;

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
  } else {
    printf("%s\t0x%8.8x\n", instr, d->regs.j.target);
  }  
  
}

/* Perform computation needed to execute d, returning computed value */
int Execute ( DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
  return 0;
}

/* 
 * Update the program counter based on the current instruction. For
 * instructions other than branches and jumps, for example, the PC
 * increments by 4 (which we have provided).
 */
void UpdatePC ( DecodedInstr* d, int val) {
    mips.pc+=4;
    /* Your code goes here */
}

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
  return 0;
}

/* 
 * Write back to register. If the instruction modified a register--
 * (including jal, which modifies $ra) --
 * put the index of the modified register in *changedReg,
 * otherwise put -1 in *changedReg.
 */
void RegWrite( DecodedInstr* d, int val, int *changedReg) {
    /* Your code goes here */
}
