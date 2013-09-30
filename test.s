#subu/addu/addiu/sll/srl/and/andi/or/ori/lui/slt/beq/bne/j/jal/jr/lw/sw
	.text
	addiu 	$a0,$0,3
	addiu 	$a1,$0,2

	lui 	$t2,0x8FFF
	srl 	$t2,$t2,30
	sll 	$t2,$t2,2
	addu 	$t3,$a1,$a0
	addu 	$t3,$t3,$a1
	or 	$t2,$t2,$t3
	ori 	$t2,0xFFFF0000
	andi 	$t2,$t2,0x0002
	lui	$a1,0x0000

	or	$a1,$a1,$t2

	slt	$t2,$a1,$a0
	and	$t2,$t2,$a0
	subu	$t3,$t3,$a0
	subu	$t3,$t3,$a1
	addu	$t2,$t2,$t3
	and	$a0,$a0,$t2

	sw	$a0, 0($sp)
	sw	$a1, 4($sp)
	lw	$a0, 4($sp)
	lw	$a1, 0($sp)
	lui	$s0, 0x0001

	jal	Mystery
	addi	$0,$0,0

Mystery:
	addiu	$v0,$0,0
	bne	$a0,$s0,Loop
	addiu	$0,$0,0	#Should never execute
Loop:
	beq	$a0,$0,Done
	addu	$v0,$v0,$a1
	addiu	$a0,$a0,-1
	j	Loop
Done:
	jr	$ra
