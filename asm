warning: unused import: `LitInt`
  --> statics\src\lib.rs:10:5
   |
10 |     LitInt,
   |     ^^^^^^
   |
   = note: `#[warn(unused_imports)]` on by default

warning: 1 warning emitted


etc_slightly_different:	file format elf32-littlearm

Disassembly of section .text:

00000100 <__stext>:
     100: f006 fdee    	bl	0x6ce0 <__pre_init>     @ imm = #0x6bdc
     104: 480e         	ldr	r0, [pc, #0x38]         @ 0x140 <__stext+0x40>
     106: 490f         	ldr	r1, [pc, #0x3c]         @ 0x144 <__stext+0x44>
     108: 2200         	movs	r2, #0x0
     10a: 4281         	cmp	r1, r0
     10c: d001         	beq	0x112 <__stext+0x12>    @ imm = #0x2
     10e: c004         	stm	r0!, {r2}
     110: e7fb         	b	0x10a <__stext+0xa>     @ imm = #-0xa
     112: 480d         	ldr	r0, [pc, #0x34]         @ 0x148 <__stext+0x48>
     114: 490d         	ldr	r1, [pc, #0x34]         @ 0x14c <__stext+0x4c>
     116: 4a0e         	ldr	r2, [pc, #0x38]         @ 0x150 <__stext+0x50>
     118: 4281         	cmp	r1, r0
     11a: d002         	beq	0x122 <__stext+0x22>    @ imm = #0x4
     11c: ca08         	ldm	r2!, {r3}
     11e: c008         	stm	r0!, {r3}
     120: e7fa         	b	0x118 <__stext+0x18>    @ imm = #-0xc
     122: 480c         	ldr	r0, [pc, #0x30]         @ 0x154 <__stext+0x54>
     124: f44f 0170    	mov.w	r1, #0xf00000
     128: 6802         	ldr	r2, [r0]
     12a: ea42 0201    	orr.w	r2, r2, r1
     12e: 6002         	str	r2, [r0]
     130: f3bf 8f4f    	dsb	sy
     134: f3bf 8f6f    	isb	sy
     138: f003 ff70    	bl	0x401c <main>           @ imm = #0x3ee0
     13c: de00         	udf	#0x0
     13e: 0000         	movs	r0, r0
     140: 38 00 00 20  	.word	0x20000038
     144: e0 02 00 20  	.word	0x200002e0
     148: 00 00 00 20  	.word	0x20000000
     14c: 38 00 00 20  	.word	0x20000038
     150: 38 7d 00 00  	.word	0x00007d38
     154: 88 ed 00 e0  	.word	0xe000ed88

00000158 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb>:
     158: b5f0         	push	{r4, r5, r6, r7, lr}
     15a: af03         	add	r7, sp, #0xc
     15c: e92d 0f00    	push.w	{r8, r9, r10, r11}
     160: b085         	sub	sp, #0x14
     162: f640 0c08    	movw	r12, #0x808
     166: 7809         	ldrb	r1, [r1]
     168: f2c5 0c00    	movt	r12, #0x5000
     16c: f44f 7280    	mov.w	r2, #0x100
     170: f8cc 2004    	str.w	r2, [r12, #0x4]
     174: f243 521c    	movw	r2, #0x351c
     178: 2960         	cmp	r1, #0x60
     17a: f243 1108    	movw	r1, #0x3108
     17e: f04f 0394    	mov.w	r3, #0x94
     182: 4604         	mov	r4, r0
     184: f2c4 0200    	movt	r2, #0x4000
     188: f2c4 0100    	movt	r1, #0x4000
     18c: bf08         	it	eq
     18e: 2390         	moveq	r3, #0x90
     190: 6013         	str	r3, [r2]
     192: e008         	b	0x1a6 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x4e> @ imm = #0x10
     194: bf10         	yield
     196: 680b         	ldr	r3, [r1]
     198: 2b00         	cmp	r3, #0x0
     19a: bf02         	ittt	eq
     19c: bf10         	yieldeq
     19e: 680b         	ldreq	r3, [r1]
     1a0: 2b00         	cmpeq	r3, #0x0
     1a2: d107         	bne	0x1b4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x5c> @ imm = #0xe
     1a4: bf10         	yield
     1a6: 680b         	ldr	r3, [r1]
     1a8: 2b00         	cmp	r3, #0x0
     1aa: bf02         	ittt	eq
     1ac: bf10         	yieldeq
     1ae: 680b         	ldreq	r3, [r1]
     1b0: 2b00         	cmpeq	r3, #0x0
     1b2: d0ef         	beq	0x194 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x3c> @ imm = #-0x22
     1b4: f8d1 3410    	ldr.w	r3, [r1, #0x410]
     1b8: 2300         	movs	r3, #0x0
     1ba: 600b         	str	r3, [r1]
     1bc: 6013         	str	r3, [r2]
     1be: e008         	b	0x1d2 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x7a> @ imm = #0x10
     1c0: bf10         	yield
     1c2: 680e         	ldr	r6, [r1]
     1c4: 2e00         	cmp	r6, #0x0
     1c6: bf02         	ittt	eq
     1c8: bf10         	yieldeq
     1ca: 680e         	ldreq	r6, [r1]
     1cc: 2e00         	cmpeq	r6, #0x0
     1ce: d107         	bne	0x1e0 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x88> @ imm = #0xe
     1d0: bf10         	yield
     1d2: 680e         	ldr	r6, [r1]
     1d4: 2e00         	cmp	r6, #0x0
     1d6: bf02         	ittt	eq
     1d8: bf10         	yieldeq
     1da: 680e         	ldreq	r6, [r1]
     1dc: 2e00         	cmpeq	r6, #0x0
     1de: d0ef         	beq	0x1c0 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x68> @ imm = #-0x22
     1e0: f8d1 0410    	ldr.w	r0, [r1, #0x410]
     1e4: 9004         	str	r0, [sp, #0x10]
     1e6: 600b         	str	r3, [r1]
     1e8: 6013         	str	r3, [r2]
     1ea: e008         	b	0x1fe <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0xa6> @ imm = #0x10
     1ec: bf10         	yield
     1ee: 680b         	ldr	r3, [r1]
     1f0: 2b00         	cmp	r3, #0x0
     1f2: bf02         	ittt	eq
     1f4: bf10         	yieldeq
     1f6: 680b         	ldreq	r3, [r1]
     1f8: 2b00         	cmpeq	r3, #0x0
     1fa: d107         	bne	0x20c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0xb4> @ imm = #0xe
     1fc: bf10         	yield
     1fe: 680b         	ldr	r3, [r1]
     200: 2b00         	cmp	r3, #0x0
     202: bf02         	ittt	eq
     204: bf10         	yieldeq
     206: 680b         	ldreq	r3, [r1]
     208: 2b00         	cmpeq	r3, #0x0
     20a: d0ef         	beq	0x1ec <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x94> @ imm = #-0x22
     20c: f8d1 0410    	ldr.w	r0, [r1, #0x410]
     210: 2300         	movs	r3, #0x0
     212: 9003         	str	r0, [sp, #0xc]
     214: 600b         	str	r3, [r1]
     216: 6013         	str	r3, [r2]
     218: e008         	b	0x22c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0xd4> @ imm = #0x10
     21a: bf10         	yield
     21c: 680e         	ldr	r6, [r1]
     21e: 2e00         	cmp	r6, #0x0
     220: bf02         	ittt	eq
     222: bf10         	yieldeq
     224: 680e         	ldreq	r6, [r1]
     226: 2e00         	cmpeq	r6, #0x0
     228: d107         	bne	0x23a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0xe2> @ imm = #0xe
     22a: bf10         	yield
     22c: 680e         	ldr	r6, [r1]
     22e: 2e00         	cmp	r6, #0x0
     230: bf02         	ittt	eq
     232: bf10         	yieldeq
     234: 680e         	ldreq	r6, [r1]
     236: 2e00         	cmpeq	r6, #0x0
     238: d0ef         	beq	0x21a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0xc2> @ imm = #-0x22
     23a: f8d1 6410    	ldr.w	r6, [r1, #0x410]
     23e: 600b         	str	r3, [r1]
     240: 6013         	str	r3, [r2]
     242: e008         	b	0x256 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0xfe> @ imm = #0x10
     244: bf10         	yield
     246: 680b         	ldr	r3, [r1]
     248: 2b00         	cmp	r3, #0x0
     24a: bf02         	ittt	eq
     24c: bf10         	yieldeq
     24e: 680b         	ldreq	r3, [r1]
     250: 2b00         	cmpeq	r3, #0x0
     252: d107         	bne	0x264 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x10c> @ imm = #0xe
     254: bf10         	yield
     256: 680b         	ldr	r3, [r1]
     258: 2b00         	cmp	r3, #0x0
     25a: bf02         	ittt	eq
     25c: bf10         	yieldeq
     25e: 680b         	ldreq	r3, [r1]
     260: 2b00         	cmpeq	r3, #0x0
     262: d0ef         	beq	0x244 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0xec> @ imm = #-0x22
     264: f8d1 3410    	ldr.w	r3, [r1, #0x410]
     268: 2300         	movs	r3, #0x0
     26a: 600b         	str	r3, [r1]
     26c: 6013         	str	r3, [r2]
     26e: e008         	b	0x282 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x12a> @ imm = #0x10
     270: bf10         	yield
     272: 680e         	ldr	r6, [r1]
     274: 2e00         	cmp	r6, #0x0
     276: bf02         	ittt	eq
     278: bf10         	yieldeq
     27a: 680e         	ldreq	r6, [r1]
     27c: 2e00         	cmpeq	r6, #0x0
     27e: d107         	bne	0x290 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x138> @ imm = #0xe
     280: bf10         	yield
     282: 680e         	ldr	r6, [r1]
     284: 2e00         	cmp	r6, #0x0
     286: bf02         	ittt	eq
     288: bf10         	yieldeq
     28a: 680e         	ldreq	r6, [r1]
     28c: 2e00         	cmpeq	r6, #0x0
     28e: d0ef         	beq	0x270 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x118> @ imm = #-0x22
     290: f8d1 6410    	ldr.w	r6, [r1, #0x410]
     294: 600b         	str	r3, [r1]
     296: 6013         	str	r3, [r2]
     298: e008         	b	0x2ac <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x154> @ imm = #0x10
     29a: bf10         	yield
     29c: 680b         	ldr	r3, [r1]
     29e: 2b00         	cmp	r3, #0x0
     2a0: bf02         	ittt	eq
     2a2: bf10         	yieldeq
     2a4: 680b         	ldreq	r3, [r1]
     2a6: 2b00         	cmpeq	r3, #0x0
     2a8: d107         	bne	0x2ba <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x162> @ imm = #0xe
     2aa: bf10         	yield
     2ac: 680b         	ldr	r3, [r1]
     2ae: 2b00         	cmp	r3, #0x0
     2b0: bf02         	ittt	eq
     2b2: bf10         	yieldeq
     2b4: 680b         	ldreq	r3, [r1]
     2b6: 2b00         	cmpeq	r3, #0x0
     2b8: d0ef         	beq	0x29a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x142> @ imm = #-0x22
     2ba: f8d1 0410    	ldr.w	r0, [r1, #0x410]
     2be: 2300         	movs	r3, #0x0
     2c0: 9002         	str	r0, [sp, #0x8]
     2c2: 600b         	str	r3, [r1]
     2c4: 6013         	str	r3, [r2]
     2c6: e008         	b	0x2da <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x182> @ imm = #0x10
     2c8: bf10         	yield
     2ca: 680e         	ldr	r6, [r1]
     2cc: 2e00         	cmp	r6, #0x0
     2ce: bf02         	ittt	eq
     2d0: bf10         	yieldeq
     2d2: 680e         	ldreq	r6, [r1]
     2d4: 2e00         	cmpeq	r6, #0x0
     2d6: d107         	bne	0x2e8 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x190> @ imm = #0xe
     2d8: bf10         	yield
     2da: 680e         	ldr	r6, [r1]
     2dc: 2e00         	cmp	r6, #0x0
     2de: bf02         	ittt	eq
     2e0: bf10         	yieldeq
     2e2: 680e         	ldreq	r6, [r1]
     2e4: 2e00         	cmpeq	r6, #0x0
     2e6: d0ef         	beq	0x2c8 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x170> @ imm = #-0x22
     2e8: f8d1 0410    	ldr.w	r0, [r1, #0x410]
     2ec: 9001         	str	r0, [sp, #0x4]
     2ee: 600b         	str	r3, [r1]
     2f0: 6013         	str	r3, [r2]
     2f2: e008         	b	0x306 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x1ae> @ imm = #0x10
     2f4: bf10         	yield
     2f6: 680b         	ldr	r3, [r1]
     2f8: 2b00         	cmp	r3, #0x0
     2fa: bf02         	ittt	eq
     2fc: bf10         	yieldeq
     2fe: 680b         	ldreq	r3, [r1]
     300: 2b00         	cmpeq	r3, #0x0
     302: d107         	bne	0x314 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x1bc> @ imm = #0xe
     304: bf10         	yield
     306: 680b         	ldr	r3, [r1]
     308: 2b00         	cmp	r3, #0x0
     30a: bf02         	ittt	eq
     30c: bf10         	yieldeq
     30e: 680b         	ldreq	r3, [r1]
     310: 2b00         	cmpeq	r3, #0x0
     312: d0ef         	beq	0x2f4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x19c> @ imm = #-0x22
     314: f8d1 0410    	ldr.w	r0, [r1, #0x410]
     318: 2300         	movs	r3, #0x0
     31a: 9000         	str	r0, [sp]
     31c: 600b         	str	r3, [r1]
     31e: 6013         	str	r3, [r2]
     320: e008         	b	0x334 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x1dc> @ imm = #0x10
     322: bf10         	yield
     324: 680e         	ldr	r6, [r1]
     326: 2e00         	cmp	r6, #0x0
     328: bf02         	ittt	eq
     32a: bf10         	yieldeq
     32c: 680e         	ldreq	r6, [r1]
     32e: 2e00         	cmpeq	r6, #0x0
     330: d107         	bne	0x342 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x1ea> @ imm = #0xe
     332: bf10         	yield
     334: 680e         	ldr	r6, [r1]
     336: 2e00         	cmp	r6, #0x0
     338: bf02         	ittt	eq
     33a: bf10         	yieldeq
     33c: 680e         	ldreq	r6, [r1]
     33e: 2e00         	cmpeq	r6, #0x0
     340: d0ef         	beq	0x322 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x1ca> @ imm = #-0x22
     342: f8d1 6410    	ldr.w	r6, [r1, #0x410]
     346: 600b         	str	r3, [r1]
     348: 6013         	str	r3, [r2]
     34a: e008         	b	0x35e <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x206> @ imm = #0x10
     34c: bf10         	yield
     34e: 680b         	ldr	r3, [r1]
     350: 2b00         	cmp	r3, #0x0
     352: bf02         	ittt	eq
     354: bf10         	yieldeq
     356: 680b         	ldreq	r3, [r1]
     358: 2b00         	cmpeq	r3, #0x0
     35a: d107         	bne	0x36c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x214> @ imm = #0xe
     35c: bf10         	yield
     35e: 680b         	ldr	r3, [r1]
     360: 2b00         	cmp	r3, #0x0
     362: bf02         	ittt	eq
     364: bf10         	yieldeq
     366: 680b         	ldreq	r3, [r1]
     368: 2b00         	cmpeq	r3, #0x0
     36a: d0ef         	beq	0x34c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x1f4> @ imm = #-0x22
     36c: f8d1 8410    	ldr.w	r8, [r1, #0x410]
     370: 2300         	movs	r3, #0x0
     372: 600b         	str	r3, [r1]
     374: 6013         	str	r3, [r2]
     376: e008         	b	0x38a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x232> @ imm = #0x10
     378: bf10         	yield
     37a: 6808         	ldr	r0, [r1]
     37c: 2800         	cmp	r0, #0x0
     37e: bf02         	ittt	eq
     380: bf10         	yieldeq
     382: 6808         	ldreq	r0, [r1]
     384: 2800         	cmpeq	r0, #0x0
     386: d107         	bne	0x398 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x240> @ imm = #0xe
     388: bf10         	yield
     38a: 6808         	ldr	r0, [r1]
     38c: 2800         	cmp	r0, #0x0
     38e: bf02         	ittt	eq
     390: bf10         	yieldeq
     392: 6808         	ldreq	r0, [r1]
     394: 2800         	cmpeq	r0, #0x0
     396: d0ef         	beq	0x378 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x220> @ imm = #-0x22
     398: f8d1 9410    	ldr.w	r9, [r1, #0x410]
     39c: 600b         	str	r3, [r1]
     39e: 6013         	str	r3, [r2]
     3a0: e008         	b	0x3b4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x25c> @ imm = #0x10
     3a2: bf10         	yield
     3a4: 6808         	ldr	r0, [r1]
     3a6: 2800         	cmp	r0, #0x0
     3a8: bf02         	ittt	eq
     3aa: bf10         	yieldeq
     3ac: 6808         	ldreq	r0, [r1]
     3ae: 2800         	cmpeq	r0, #0x0
     3b0: d107         	bne	0x3c2 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x26a> @ imm = #0xe
     3b2: bf10         	yield
     3b4: 6808         	ldr	r0, [r1]
     3b6: 2800         	cmp	r0, #0x0
     3b8: bf02         	ittt	eq
     3ba: bf10         	yieldeq
     3bc: 6808         	ldreq	r0, [r1]
     3be: 2800         	cmpeq	r0, #0x0
     3c0: d0ef         	beq	0x3a2 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x24a> @ imm = #-0x22
     3c2: f8d1 b410    	ldr.w	r11, [r1, #0x410]
     3c6: f04f 0a00    	mov.w	r10, #0x0
     3ca: f8c1 a000    	str.w	r10, [r1]
     3ce: f8c2 a000    	str.w	r10, [r2]
     3d2: e008         	b	0x3e6 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x28e> @ imm = #0x10
     3d4: bf10         	yield
     3d6: 6808         	ldr	r0, [r1]
     3d8: 2800         	cmp	r0, #0x0
     3da: bf02         	ittt	eq
     3dc: bf10         	yieldeq
     3de: 6808         	ldreq	r0, [r1]
     3e0: 2800         	cmpeq	r0, #0x0
     3e2: d107         	bne	0x3f4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x29c> @ imm = #0xe
     3e4: bf10         	yield
     3e6: 6808         	ldr	r0, [r1]
     3e8: 2800         	cmp	r0, #0x0
     3ea: bf02         	ittt	eq
     3ec: bf10         	yieldeq
     3ee: 6808         	ldreq	r0, [r1]
     3f0: 2800         	cmpeq	r0, #0x0
     3f2: d0ef         	beq	0x3d4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb+0x27c> @ imm = #-0x22
     3f4: f44f 7080    	mov.w	r0, #0x100
     3f8: f8d1 5410    	ldr.w	r5, [r1, #0x410]
     3fc: f8c1 a000    	str.w	r10, [r1]
     400: f8cc 0000    	str.w	r0, [r12]
     404: f006 fd3f    	bl	0x6e86 <_defmt_acquire> @ imm = #0x6a7e
     408: f240 0016    	movw	r0, #0x16
     40c: f2c0 0000    	movt	r0, #0x0
     410: f006 fcbe    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x697c
     414: fa5f f08b    	uxtb.w	r0, r11
     418: 0629         	lsls	r1, r5, #0x18
     41a: ea41 4000    	orr.w	r0, r1, r0, lsl #16
     41e: fa5f f189    	uxtb.w	r1, r9
     422: ea40 2001    	orr.w	r0, r0, r1, lsl #8
     426: fa5f f188    	uxtb.w	r1, r8
     42a: 1845         	adds	r5, r0, r1
     42c: f006 fd80    	bl	0x6f30 <_defmt_release> @ imm = #0x6b00
     430: 9800         	ldr	r0, [sp]
     432: 9901         	ldr	r1, [sp, #0x4]
     434: 9a04         	ldr	r2, [sp, #0x10]
     436: 0400         	lsls	r0, r0, #0x10
     438: fa3f f080    	uxtb16	r0, r0
     43c: b2c9         	uxtb	r1, r1
     43e: ea40 6006    	orr.w	r0, r0, r6, lsl #24
     442: ea40 2001    	orr.w	r0, r0, r1, lsl #8
     446: 9902         	ldr	r1, [sp, #0x8]
     448: b2c9         	uxtb	r1, r1
     44a: 4408         	add	r0, r1
     44c: 2108         	movs	r1, #0x8
     44e: e9c4 0502    	strd	r0, r5, [r4, #8]
     452: 7421         	strb	r1, [r4, #0x10]
     454: 9903         	ldr	r1, [sp, #0xc]
     456: f3c1 1142    	ubfx	r1, r1, #0x5, #0x3
     45a: f362 01ca    	bfi	r1, r2, #3, #8
     45e: 0409         	lsls	r1, r1, #0x10
     460: e9c4 1a00    	strd	r1, r10, [r4]
     464: b005         	add	sp, #0x14
     466: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
     46a: bdf0         	pop	{r4, r5, r6, r7, pc}

0000046c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3>:
     46c: b5f0         	push	{r4, r5, r6, r7, lr}
     46e: af03         	add	r7, sp, #0xc
     470: e92d 0b00    	push.w	{r8, r9, r11}
     474: f640 0c08    	movw	r12, #0x808
     478: f243 581c    	movw	r8, #0x351c
     47c: f2c5 0c00    	movt	r12, #0x5000
     480: f44f 7380    	mov.w	r3, #0x100
     484: f8cc 3004    	str.w	r3, [r12, #0x4]
     488: f2c4 0800    	movt	r8, #0x4000
     48c: 2302         	movs	r3, #0x2
     48e: b2c0         	uxtb	r0, r0
     490: f8c8 3000    	str.w	r3, [r8]
     494: f04f 0e09    	mov.w	lr, #0x9
     498: 2860         	cmp	r0, #0x60
     49a: f04f 0324    	mov.w	r3, #0x24
     49e: f04f 0525    	mov.w	r5, #0x25
     4a2: bf08         	it	eq
     4a4: f04f 0e01    	moveq.w	lr, #0x1
     4a8: bf08         	it	eq
     4aa: 2320         	moveq	r3, #0x20
     4ac: bf08         	it	eq
     4ae: 2521         	moveq	r5, #0x21
     4b0: f1b0 0960    	subs.w	r9, r0, #0x60
     4b4: b288         	uxth	r0, r1
     4b6: ea4f 2610    	lsr.w	r6, r0, #0x8
     4ba: f243 1008    	movw	r0, #0x3108
     4be: f2c4 0000    	movt	r0, #0x4000
     4c2: bf18         	it	ne
     4c4: f04f 0901    	movne.w	r9, #0x1
     4c8: e008         	b	0x4dc <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x70> @ imm = #0x10
     4ca: bf10         	yield
     4cc: 6804         	ldr	r4, [r0]
     4ce: 2c00         	cmp	r4, #0x0
     4d0: bf02         	ittt	eq
     4d2: bf10         	yieldeq
     4d4: 6804         	ldreq	r4, [r0]
     4d6: 2c00         	cmpeq	r4, #0x0
     4d8: d107         	bne	0x4ea <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x7e> @ imm = #0xe
     4da: bf10         	yield
     4dc: 6804         	ldr	r4, [r0]
     4de: 2c00         	cmp	r4, #0x0
     4e0: bf02         	ittt	eq
     4e2: bf10         	yieldeq
     4e4: 6804         	ldreq	r4, [r0]
     4e6: 2c00         	cmpeq	r4, #0x0
     4e8: d0ef         	beq	0x4ca <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x5e> @ imm = #-0x22
     4ea: f8d0 4410    	ldr.w	r4, [r0, #0x410]
     4ee: 2400         	movs	r4, #0x0
     4f0: 6004         	str	r4, [r0]
     4f2: f8c8 3000    	str.w	r3, [r8]
     4f6: e008         	b	0x50a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x9e> @ imm = #0x10
     4f8: bf10         	yield
     4fa: 6803         	ldr	r3, [r0]
     4fc: 2b00         	cmp	r3, #0x0
     4fe: bf02         	ittt	eq
     500: bf10         	yieldeq
     502: 6803         	ldreq	r3, [r0]
     504: 2b00         	cmpeq	r3, #0x0
     506: d107         	bne	0x518 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0xac> @ imm = #0xe
     508: bf10         	yield
     50a: 6803         	ldr	r3, [r0]
     50c: 2b00         	cmp	r3, #0x0
     50e: bf02         	ittt	eq
     510: bf10         	yieldeq
     512: 6803         	ldreq	r3, [r0]
     514: 2b00         	cmpeq	r3, #0x0
     516: d0ef         	beq	0x4f8 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x8c> @ imm = #-0x22
     518: f8d0 3410    	ldr.w	r3, [r0, #0x410]
     51c: 6004         	str	r4, [r0]
     51e: f8c8 6000    	str.w	r6, [r8]
     522: e008         	b	0x536 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0xca> @ imm = #0x10
     524: bf10         	yield
     526: 6803         	ldr	r3, [r0]
     528: 2b00         	cmp	r3, #0x0
     52a: bf02         	ittt	eq
     52c: bf10         	yieldeq
     52e: 6803         	ldreq	r3, [r0]
     530: 2b00         	cmpeq	r3, #0x0
     532: d107         	bne	0x544 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0xd8> @ imm = #0xe
     534: bf10         	yield
     536: 6803         	ldr	r3, [r0]
     538: 2b00         	cmp	r3, #0x0
     53a: bf02         	ittt	eq
     53c: bf10         	yieldeq
     53e: 6803         	ldreq	r3, [r0]
     540: 2b00         	cmpeq	r3, #0x0
     542: d0ef         	beq	0x524 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0xb8> @ imm = #-0x22
     544: f8d0 4410    	ldr.w	r4, [r0, #0x410]
     548: 2300         	movs	r3, #0x0
     54a: f001 01e0    	and	r1, r1, #0xe0
     54e: f44f 7480    	mov.w	r4, #0x100
     552: 6003         	str	r3, [r0]
     554: f8cc 4000    	str.w	r4, [r12]
     558: f8cc 4004    	str.w	r4, [r12, #0x4]
     55c: 2402         	movs	r4, #0x2
     55e: f8c8 4000    	str.w	r4, [r8]
     562: e008         	b	0x576 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x10a> @ imm = #0x10
     564: bf10         	yield
     566: 6804         	ldr	r4, [r0]
     568: 2c00         	cmp	r4, #0x0
     56a: bf02         	ittt	eq
     56c: bf10         	yieldeq
     56e: 6804         	ldreq	r4, [r0]
     570: 2c00         	cmpeq	r4, #0x0
     572: d107         	bne	0x584 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x118> @ imm = #0xe
     574: bf10         	yield
     576: 6804         	ldr	r4, [r0]
     578: 2c00         	cmp	r4, #0x0
     57a: bf02         	ittt	eq
     57c: bf10         	yieldeq
     57e: 6804         	ldreq	r4, [r0]
     580: 2c00         	cmpeq	r4, #0x0
     582: d0ef         	beq	0x564 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0xf8> @ imm = #-0x22
     584: f8d0 4410    	ldr.w	r4, [r0, #0x410]
     588: 6003         	str	r3, [r0]
     58a: f8c8 5000    	str.w	r5, [r8]
     58e: e008         	b	0x5a2 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x136> @ imm = #0x10
     590: bf10         	yield
     592: 6803         	ldr	r3, [r0]
     594: 2b00         	cmp	r3, #0x0
     596: bf02         	ittt	eq
     598: bf10         	yieldeq
     59a: 6803         	ldreq	r3, [r0]
     59c: 2b00         	cmpeq	r3, #0x0
     59e: d107         	bne	0x5b0 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x144> @ imm = #0xe
     5a0: bf10         	yield
     5a2: 6803         	ldr	r3, [r0]
     5a4: 2b00         	cmp	r3, #0x0
     5a6: bf02         	ittt	eq
     5a8: bf10         	yieldeq
     5aa: 6803         	ldreq	r3, [r0]
     5ac: 2b00         	cmpeq	r3, #0x0
     5ae: d0ef         	beq	0x590 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x124> @ imm = #-0x22
     5b0: f8d0 5410    	ldr.w	r5, [r0, #0x410]
     5b4: b293         	uxth	r3, r2
     5b6: 2500         	movs	r5, #0x0
     5b8: ea4f 04c9    	lsl.w	r4, r9, #0x3
     5bc: 6005         	str	r5, [r0]
     5be: f8c8 1000    	str.w	r1, [r8]
     5c2: e008         	b	0x5d6 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x16a> @ imm = #0x10
     5c4: bf10         	yield
     5c6: 6801         	ldr	r1, [r0]
     5c8: 2900         	cmp	r1, #0x0
     5ca: bf02         	ittt	eq
     5cc: bf10         	yieldeq
     5ce: 6801         	ldreq	r1, [r0]
     5d0: 2900         	cmpeq	r1, #0x0
     5d2: d107         	bne	0x5e4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x178> @ imm = #0xe
     5d4: bf10         	yield
     5d6: 6801         	ldr	r1, [r0]
     5d8: 2900         	cmp	r1, #0x0
     5da: bf02         	ittt	eq
     5dc: bf10         	yieldeq
     5de: 6801         	ldreq	r1, [r0]
     5e0: 2900         	cmpeq	r1, #0x0
     5e2: d0ef         	beq	0x5c4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x158> @ imm = #-0x22
     5e4: 0a19         	lsrs	r1, r3, #0x8
     5e6: f8d0 3410    	ldr.w	r3, [r0, #0x410]
     5ea: f44f 7380    	mov.w	r3, #0x100
     5ee: 6005         	str	r5, [r0]
     5f0: f8cc 3000    	str.w	r3, [r12]
     5f4: f8cc 3004    	str.w	r3, [r12, #0x4]
     5f8: 2302         	movs	r3, #0x2
     5fa: f8c8 3000    	str.w	r3, [r8]
     5fe: e008         	b	0x612 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x1a6> @ imm = #0x10
     600: bf10         	yield
     602: 6803         	ldr	r3, [r0]
     604: 2b00         	cmp	r3, #0x0
     606: bf02         	ittt	eq
     608: bf10         	yieldeq
     60a: 6803         	ldreq	r3, [r0]
     60c: 2b00         	cmpeq	r3, #0x0
     60e: d107         	bne	0x620 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x1b4> @ imm = #0xe
     610: bf10         	yield
     612: 6803         	ldr	r3, [r0]
     614: 2b00         	cmp	r3, #0x0
     616: bf02         	ittt	eq
     618: bf10         	yieldeq
     61a: 6803         	ldreq	r3, [r0]
     61c: 2b00         	cmpeq	r3, #0x0
     61e: d0ef         	beq	0x600 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x194> @ imm = #-0x22
     620: f8d0 3410    	ldr.w	r3, [r0, #0x410]
     624: 2300         	movs	r3, #0x0
     626: 6003         	str	r3, [r0]
     628: f8c8 4000    	str.w	r4, [r8]
     62c: e008         	b	0x640 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x1d4> @ imm = #0x10
     62e: bf10         	yield
     630: 6804         	ldr	r4, [r0]
     632: 2c00         	cmp	r4, #0x0
     634: bf02         	ittt	eq
     636: bf10         	yieldeq
     638: 6804         	ldreq	r4, [r0]
     63a: 2c00         	cmpeq	r4, #0x0
     63c: d107         	bne	0x64e <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x1e2> @ imm = #0xe
     63e: bf10         	yield
     640: 6804         	ldr	r4, [r0]
     642: 2c00         	cmp	r4, #0x0
     644: bf02         	ittt	eq
     646: bf10         	yieldeq
     648: 6804         	ldreq	r4, [r0]
     64a: 2c00         	cmpeq	r4, #0x0
     64c: d0ef         	beq	0x62e <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x1c2> @ imm = #-0x22
     64e: f8d0 4410    	ldr.w	r4, [r0, #0x410]
     652: 6003         	str	r3, [r0]
     654: f8c8 1000    	str.w	r1, [r8]
     658: e008         	b	0x66c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x200> @ imm = #0x10
     65a: bf10         	yield
     65c: 6801         	ldr	r1, [r0]
     65e: 2900         	cmp	r1, #0x0
     660: bf02         	ittt	eq
     662: bf10         	yieldeq
     664: 6801         	ldreq	r1, [r0]
     666: 2900         	cmpeq	r1, #0x0
     668: d107         	bne	0x67a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x20e> @ imm = #0xe
     66a: bf10         	yield
     66c: 6801         	ldr	r1, [r0]
     66e: 2900         	cmp	r1, #0x0
     670: bf02         	ittt	eq
     672: bf10         	yieldeq
     674: 6801         	ldreq	r1, [r0]
     676: 2900         	cmpeq	r1, #0x0
     678: d0ef         	beq	0x65a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x1ee> @ imm = #-0x22
     67a: f8d0 3410    	ldr.w	r3, [r0, #0x410]
     67e: f002 01e0    	and	r1, r2, #0xe0
     682: 2200         	movs	r2, #0x0
     684: f44f 7380    	mov.w	r3, #0x100
     688: 6002         	str	r2, [r0]
     68a: f8cc 3000    	str.w	r3, [r12]
     68e: f8cc 3004    	str.w	r3, [r12, #0x4]
     692: 2302         	movs	r3, #0x2
     694: f8c8 3000    	str.w	r3, [r8]
     698: e008         	b	0x6ac <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x240> @ imm = #0x10
     69a: bf10         	yield
     69c: 6803         	ldr	r3, [r0]
     69e: 2b00         	cmp	r3, #0x0
     6a0: bf02         	ittt	eq
     6a2: bf10         	yieldeq
     6a4: 6803         	ldreq	r3, [r0]
     6a6: 2b00         	cmpeq	r3, #0x0
     6a8: d107         	bne	0x6ba <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x24e> @ imm = #0xe
     6aa: bf10         	yield
     6ac: 6803         	ldr	r3, [r0]
     6ae: 2b00         	cmp	r3, #0x0
     6b0: bf02         	ittt	eq
     6b2: bf10         	yieldeq
     6b4: 6803         	ldreq	r3, [r0]
     6b6: 2b00         	cmpeq	r3, #0x0
     6b8: d0ef         	beq	0x69a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x22e> @ imm = #-0x22
     6ba: f8d0 3410    	ldr.w	r3, [r0, #0x410]
     6be: 6002         	str	r2, [r0]
     6c0: f8c8 e000    	str.w	lr, [r8]
     6c4: e008         	b	0x6d8 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x26c> @ imm = #0x10
     6c6: bf10         	yield
     6c8: 6802         	ldr	r2, [r0]
     6ca: 2a00         	cmp	r2, #0x0
     6cc: bf02         	ittt	eq
     6ce: bf10         	yieldeq
     6d0: 6802         	ldreq	r2, [r0]
     6d2: 2a00         	cmpeq	r2, #0x0
     6d4: d107         	bne	0x6e6 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x27a> @ imm = #0xe
     6d6: bf10         	yield
     6d8: 6802         	ldr	r2, [r0]
     6da: 2a00         	cmp	r2, #0x0
     6dc: bf02         	ittt	eq
     6de: bf10         	yieldeq
     6e0: 6802         	ldreq	r2, [r0]
     6e2: 2a00         	cmpeq	r2, #0x0
     6e4: d0ef         	beq	0x6c6 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x25a> @ imm = #-0x22
     6e6: f8d0 2410    	ldr.w	r2, [r0, #0x410]
     6ea: 2200         	movs	r2, #0x0
     6ec: 6002         	str	r2, [r0]
     6ee: f8c8 1000    	str.w	r1, [r8]
     6f2: e008         	b	0x706 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x29a> @ imm = #0x10
     6f4: bf10         	yield
     6f6: 6801         	ldr	r1, [r0]
     6f8: 2900         	cmp	r1, #0x0
     6fa: bf02         	ittt	eq
     6fc: bf10         	yieldeq
     6fe: 6801         	ldreq	r1, [r0]
     700: 2900         	cmpeq	r1, #0x0
     702: d107         	bne	0x714 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x2a8> @ imm = #0xe
     704: bf10         	yield
     706: 6801         	ldr	r1, [r0]
     708: 2900         	cmp	r1, #0x0
     70a: bf02         	ittt	eq
     70c: bf10         	yieldeq
     70e: 6801         	ldreq	r1, [r0]
     710: 2900         	cmpeq	r1, #0x0
     712: d0ef         	beq	0x6f4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3+0x288> @ imm = #-0x22
     714: f8d0 1410    	ldr.w	r1, [r0, #0x410]
     718: 6002         	str	r2, [r0]
     71a: f44f 7080    	mov.w	r0, #0x100
     71e: f8cc 0000    	str.w	r0, [r12]
     722: e8bd 0b00    	pop.w	{r8, r9, r11}
     726: bdf0         	pop	{r4, r5, r6, r7, pc}

00000728 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400>:
     728: b5f0         	push	{r4, r5, r6, r7, lr}
     72a: af03         	add	r7, sp, #0xc
     72c: f84d bd04    	str	r11, [sp, #-4]!
     730: f640 0c08    	movw	r12, #0x808
     734: f243 5e1c    	movw	lr, #0x351c
     738: f243 1308    	movw	r3, #0x3108
     73c: f2c5 0c00    	movt	r12, #0x5000
     740: f44f 7280    	mov.w	r2, #0x100
     744: f2c4 0e00    	movt	lr, #0x4000
     748: f8cc 2004    	str.w	r2, [r12, #0x4]
     74c: 2203         	movs	r2, #0x3
     74e: f2c4 0300    	movt	r3, #0x4000
     752: f8ce 2000    	str.w	r2, [lr]
     756: e008         	b	0x76a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x42> @ imm = #0x10
     758: bf10         	yield
     75a: 681a         	ldr	r2, [r3]
     75c: 2a00         	cmp	r2, #0x0
     75e: bf02         	ittt	eq
     760: bf10         	yieldeq
     762: 681a         	ldreq	r2, [r3]
     764: 2a00         	cmpeq	r2, #0x0
     766: d107         	bne	0x778 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x50> @ imm = #0xe
     768: bf10         	yield
     76a: 681a         	ldr	r2, [r3]
     76c: 2a00         	cmp	r2, #0x0
     76e: bf02         	ittt	eq
     770: bf10         	yieldeq
     772: 681a         	ldreq	r2, [r3]
     774: 2a00         	cmpeq	r2, #0x0
     776: d0ef         	beq	0x758 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x30> @ imm = #-0x22
     778: f8d3 4410    	ldr.w	r4, [r3, #0x410]
     77c: 2200         	movs	r2, #0x0
     77e: 242c         	movs	r4, #0x2c
     780: 601a         	str	r2, [r3]
     782: f8ce 4000    	str.w	r4, [lr]
     786: e008         	b	0x79a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x72> @ imm = #0x10
     788: bf10         	yield
     78a: 681c         	ldr	r4, [r3]
     78c: 2c00         	cmp	r4, #0x0
     78e: bf02         	ittt	eq
     790: bf10         	yieldeq
     792: 681c         	ldreq	r4, [r3]
     794: 2c00         	cmpeq	r4, #0x0
     796: d107         	bne	0x7a8 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x80> @ imm = #0xe
     798: bf10         	yield
     79a: 681c         	ldr	r4, [r3]
     79c: 2c00         	cmp	r4, #0x0
     79e: bf02         	ittt	eq
     7a0: bf10         	yieldeq
     7a2: 681c         	ldreq	r4, [r3]
     7a4: 2c00         	cmpeq	r4, #0x0
     7a6: d0ef         	beq	0x788 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x60> @ imm = #-0x22
     7a8: f8d3 4410    	ldr.w	r4, [r3, #0x410]
     7ac: 601a         	str	r2, [r3]
     7ae: f8ce 2000    	str.w	r2, [lr]
     7b2: e008         	b	0x7c6 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x9e> @ imm = #0x10
     7b4: bf10         	yield
     7b6: 681a         	ldr	r2, [r3]
     7b8: 2a00         	cmp	r2, #0x0
     7ba: bf02         	ittt	eq
     7bc: bf10         	yieldeq
     7be: 681a         	ldreq	r2, [r3]
     7c0: 2a00         	cmpeq	r2, #0x0
     7c2: d107         	bne	0x7d4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0xac> @ imm = #0xe
     7c4: bf10         	yield
     7c6: 681a         	ldr	r2, [r3]
     7c8: 2a00         	cmp	r2, #0x0
     7ca: bf02         	ittt	eq
     7cc: bf10         	yieldeq
     7ce: 681a         	ldreq	r2, [r3]
     7d0: 2a00         	cmpeq	r2, #0x0
     7d2: d0ef         	beq	0x7b4 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x8c> @ imm = #-0x22
     7d4: 7a80         	ldrb	r0, [r0, #0xa]
     7d6: f44f 7580    	mov.w	r5, #0x100
     7da: f8d3 2410    	ldr.w	r2, [r3, #0x410]
     7de: 2400         	movs	r4, #0x0
     7e0: f001 0107    	and	r1, r1, #0x7
     7e4: 601c         	str	r4, [r3]
     7e6: f8cc 5000    	str.w	r5, [r12]
     7ea: 2605         	movs	r6, #0x5
     7ec: f8cc 5004    	str.w	r5, [r12, #0x4]
     7f0: 2501         	movs	r5, #0x1
     7f2: f8ce 6000    	str.w	r6, [lr]
     7f6: e008         	b	0x80a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0xe2> @ imm = #0x10
     7f8: bf10         	yield
     7fa: 681e         	ldr	r6, [r3]
     7fc: 2e00         	cmp	r6, #0x0
     7fe: bf02         	ittt	eq
     800: bf10         	yieldeq
     802: 681e         	ldreq	r6, [r3]
     804: 2e00         	cmpeq	r6, #0x0
     806: d107         	bne	0x818 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0xf0> @ imm = #0xe
     808: bf10         	yield
     80a: 681e         	ldr	r6, [r3]
     80c: 2e00         	cmp	r6, #0x0
     80e: bf02         	ittt	eq
     810: bf10         	yieldeq
     812: 681e         	ldreq	r6, [r3]
     814: 2e00         	cmpeq	r6, #0x0
     816: d0ef         	beq	0x7f8 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0xd0> @ imm = #-0x22
     818: fa05 f101    	lsl.w	r1, r5, r1
     81c: f8d3 5410    	ldr.w	r5, [r3, #0x410]
     820: 601c         	str	r4, [r3]
     822: 242c         	movs	r4, #0x2c
     824: f8ce 4000    	str.w	r4, [lr]
     828: e008         	b	0x83c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x114> @ imm = #0x10
     82a: bf10         	yield
     82c: 681c         	ldr	r4, [r3]
     82e: 2c00         	cmp	r4, #0x0
     830: bf02         	ittt	eq
     832: bf10         	yieldeq
     834: 681c         	ldreq	r4, [r3]
     836: 2c00         	cmpeq	r4, #0x0
     838: d107         	bne	0x84a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x122> @ imm = #0xe
     83a: bf10         	yield
     83c: 681c         	ldr	r4, [r3]
     83e: 2c00         	cmp	r4, #0x0
     840: bf02         	ittt	eq
     842: bf10         	yieldeq
     844: 681c         	ldreq	r4, [r3]
     846: 2c00         	cmpeq	r4, #0x0
     848: d0ef         	beq	0x82a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x102> @ imm = #-0x22
     84a: 4051         	eors	r1, r2
     84c: f8d3 2410    	ldr.w	r2, [r3, #0x410]
     850: 2200         	movs	r2, #0x0
     852: 601a         	str	r2, [r3]
     854: f8ce 0000    	str.w	r0, [lr]
     858: e008         	b	0x86c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x144> @ imm = #0x10
     85a: bf10         	yield
     85c: 6818         	ldr	r0, [r3]
     85e: 2800         	cmp	r0, #0x0
     860: bf02         	ittt	eq
     862: bf10         	yieldeq
     864: 6818         	ldreq	r0, [r3]
     866: 2800         	cmpeq	r0, #0x0
     868: d107         	bne	0x87a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x152> @ imm = #0xe
     86a: bf10         	yield
     86c: 6818         	ldr	r0, [r3]
     86e: 2800         	cmp	r0, #0x0
     870: bf02         	ittt	eq
     872: bf10         	yieldeq
     874: 6818         	ldreq	r0, [r3]
     876: 2800         	cmpeq	r0, #0x0
     878: d0ef         	beq	0x85a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x132> @ imm = #-0x22
     87a: f8d3 0410    	ldr.w	r0, [r3, #0x410]
     87e: b2c8         	uxtb	r0, r1
     880: 601a         	str	r2, [r3]
     882: f8ce 0000    	str.w	r0, [lr]
     886: e008         	b	0x89a <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x172> @ imm = #0x10
     888: bf10         	yield
     88a: 6818         	ldr	r0, [r3]
     88c: 2800         	cmp	r0, #0x0
     88e: bf02         	ittt	eq
     890: bf10         	yieldeq
     892: 6818         	ldreq	r0, [r3]
     894: 2800         	cmpeq	r0, #0x0
     896: d107         	bne	0x8a8 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x180> @ imm = #0xe
     898: bf10         	yield
     89a: 6818         	ldr	r0, [r3]
     89c: 2800         	cmp	r0, #0x0
     89e: bf02         	ittt	eq
     8a0: bf10         	yieldeq
     8a2: 6818         	ldreq	r0, [r3]
     8a4: 2800         	cmpeq	r0, #0x0
     8a6: d0ef         	beq	0x888 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400+0x160> @ imm = #-0x22
     8a8: 2000         	movs	r0, #0x0
     8aa: f8d3 1410    	ldr.w	r1, [r3, #0x410]
     8ae: 6018         	str	r0, [r3]
     8b0: f44f 7080    	mov.w	r0, #0x100
     8b4: f8cc 0000    	str.w	r0, [r12]
     8b8: f85d bb04    	ldr	r11, [sp], #4
     8bc: bdf0         	pop	{r4, r5, r6, r7, pc}

000008be <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70>:
     8be: b580         	push	{r7, lr}
     8c0: 466f         	mov	r7, sp
     8c2: b0a2         	sub	sp, #0x88
     8c4: 460a         	mov	r2, r1
     8c6: 69c9         	ldr	r1, [r1, #0x1c]
     8c8: 06cb         	lsls	r3, r1, #0x1b
     8ca: d416         	bmi	0x8fa <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70+0x3c> @ imm = #0x2c
     8cc: 0689         	lsls	r1, r1, #0x1a
     8ce: d421         	bmi	0x914 <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70+0x56> @ imm = #0x42
     8d0: 7801         	ldrb	r1, [r0]
     8d2: 2964         	cmp	r1, #0x64
     8d4: d34c         	blo	0x970 <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70+0xb2> @ imm = #0x98
     8d6: 2029         	movs	r0, #0x29
     8d8: 4348         	muls	r0, r1, r0
     8da: 0b03         	lsrs	r3, r0, #0xc
     8dc: f06f 0063    	mvn	r0, #0x63
     8e0: fb13 1000    	smlabb	r0, r3, r0, r1
     8e4: f647 01e6    	movw	r1, #0x78e6
     8e8: f2c0 0100    	movt	r1, #0x0
     8ec: b2c0         	uxtb	r0, r0
     8ee: f831 0010    	ldrh.w	r0, [r1, r0, lsl #1]
     8f2: f8ad 002d    	strh.w	r0, [sp, #0x2d]
     8f6: 2024         	movs	r0, #0x24
     8f8: e03e         	b	0x978 <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70+0xba> @ imm = #0x7c
     8fa: 7800         	ldrb	r0, [r0]
     8fc: f000 010f    	and	r1, r0, #0xf
     900: f101 0357    	add.w	r3, r1, #0x57
     904: 290a         	cmp	r1, #0xa
     906: bf38         	it	lo
     908: f101 0330    	addlo.w	r3, r1, #0x30
     90c: f88d 3087    	strb.w	r3, [sp, #0x87]
     910: 2157         	movs	r1, #0x57
     912: e00b         	b	0x92c <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70+0x6e> @ imm = #0x16
     914: 7800         	ldrb	r0, [r0]
     916: f000 010f    	and	r1, r0, #0xf
     91a: f101 0337    	add.w	r3, r1, #0x37
     91e: 290a         	cmp	r1, #0xa
     920: bf38         	it	lo
     922: f101 0330    	addlo.w	r3, r1, #0x30
     926: 2137         	movs	r1, #0x37
     928: f88d 3087    	strb.w	r3, [sp, #0x87]
     92c: eb01 1110    	add.w	r1, r1, r0, lsr #4
     930: 0903         	lsrs	r3, r0, #0x4
     932: 28a0         	cmp	r0, #0xa0
     934: bf38         	it	lo
     936: f103 0130    	addlo.w	r1, r3, #0x30
     93a: 2810         	cmp	r0, #0x10
     93c: bf38         	it	lo
     93e: 2100         	movlo	r1, #0x0
     940: f88d 1086    	strb.w	r1, [sp, #0x86]
     944: f04f 0102    	mov.w	r1, #0x2
     948: bf38         	it	lo
     94a: 2101         	movlo	r1, #0x1
     94c: 9100         	str	r1, [sp]
     94e: a902         	add	r1, sp, #0x8
     950: 2810         	cmp	r0, #0x10
     952: f101 037e    	add.w	r3, r1, #0x7e
     956: 4610         	mov	r0, r2
     958: bf38         	it	lo
     95a: f101 037f    	addlo.w	r3, r1, #0x7f
     95e: f647 01e4    	movw	r1, #0x78e4
     962: f2c0 0100    	movt	r1, #0x0
     966: 2202         	movs	r2, #0x2
     968: f005 f9e0    	bl	0x5d2c <core::fmt::Formatter::pad_integral::h9814144aa367965a> @ imm = #0x53c0
     96c: b022         	add	sp, #0x88
     96e: bd80         	pop	{r7, pc}
     970: 290a         	cmp	r1, #0xa
     972: d206         	bhs	0x982 <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70+0xc4> @ imm = #0xc
     974: 2026         	movs	r0, #0x26
     976: 460b         	mov	r3, r1
     978: f043 0130    	orr	r1, r3, #0x30
     97c: ab02         	add	r3, sp, #0x8
     97e: 5419         	strb	r1, [r3, r0]
     980: e008         	b	0x994 <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70+0xd6> @ imm = #0x10
     982: f647 00e6    	movw	r0, #0x78e6
     986: f2c0 0000    	movt	r0, #0x0
     98a: f830 0011    	ldrh.w	r0, [r0, r1, lsl #1]
     98e: f8ad 002d    	strh.w	r0, [sp, #0x2d]
     992: 2025         	movs	r0, #0x25
     994: f1c0 0127    	rsb.w	r1, r0, #0x27
     998: 9100         	str	r1, [sp]
     99a: a902         	add	r1, sp, #0x8
     99c: 180b         	adds	r3, r1, r0
     99e: 4610         	mov	r0, r2
     9a0: 2101         	movs	r1, #0x1
     9a2: 2200         	movs	r2, #0x0
     9a4: f005 f9c2    	bl	0x5d2c <core::fmt::Formatter::pad_integral::h9814144aa367965a> @ imm = #0x5384
     9a8: b022         	add	sp, #0x88
     9aa: bd80         	pop	{r7, pc}

000009ac <<&T as core::fmt::Debug>::fmt::hd00725aac1204ea9>:
     9ac: b5f0         	push	{r4, r5, r6, r7, lr}
     9ae: af03         	add	r7, sp, #0xc
     9b0: e92d 0f00    	push.w	{r8, r9, r10, r11}
     9b4: b08d         	sub	sp, #0x34
     9b6: 6806         	ldr	r6, [r0]
     9b8: 460c         	mov	r4, r1
     9ba: 7830         	ldrb	r0, [r6]
     9bc: b180         	cbz	r0, 0x9e0 <<&T as core::fmt::Debug>::fmt::hd00725aac1204ea9+0x34> @ imm = #0x20
     9be: e9d4 b905    	ldrd	r11, r9, [r4, #20]
     9c2: f247 61b7    	movw	r1, #0x76b7
     9c6: f8d9 500c    	ldr.w	r5, [r9, #0xc]
     9ca: f2c0 0100    	movt	r1, #0x0
     9ce: 2204         	movs	r2, #0x4
     9d0: 4658         	mov	r0, r11
     9d2: 47a8         	blx	r5
     9d4: b190         	cbz	r0, 0x9fc <<&T as core::fmt::Debug>::fmt::hd00725aac1204ea9+0x50> @ imm = #0x24
     9d6: 2001         	movs	r0, #0x1
     9d8: b00d         	add	sp, #0x34
     9da: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
     9de: bdf0         	pop	{r4, r5, r6, r7, pc}
     9e0: f247 61bb    	movw	r1, #0x76bb
     9e4: e9d4 0205    	ldrd	r0, r2, [r4, #20]
     9e8: 68d3         	ldr	r3, [r2, #0xc]
     9ea: f2c0 0100    	movt	r1, #0x0
     9ee: 2204         	movs	r2, #0x4
     9f0: b00d         	add	sp, #0x34
     9f2: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
     9f6: e8bd 40f0    	pop.w	{r4, r5, r6, r7, lr}
     9fa: 4718         	bx	r3
     9fc: f8d4 a01c    	ldr.w	r10, [r4, #0x1c]
     a00: f106 0801    	add.w	r8, r6, #0x1
     a04: ea5f 704a    	lsls.w	r0, r10, #0x1d
     a08: d415         	bmi	0xa36 <<&T as core::fmt::Debug>::fmt::hd00725aac1204ea9+0x8a> @ imm = #0x2a
     a0a: f647 01bd    	movw	r1, #0x78bd
     a0e: 4658         	mov	r0, r11
     a10: f2c0 0100    	movt	r1, #0x0
     a14: 2201         	movs	r2, #0x1
     a16: 47a8         	blx	r5
     a18: 4601         	mov	r1, r0
     a1a: 2001         	movs	r0, #0x1
     a1c: 2900         	cmp	r1, #0x0
     a1e: d1db         	bne	0x9d8 <<&T as core::fmt::Debug>::fmt::hd00725aac1204ea9+0x2c> @ imm = #-0x4a
     a20: 4640         	mov	r0, r8
     a22: 4621         	mov	r1, r4
     a24: f7ff ff4b    	bl	0x8be <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70> @ imm = #-0x16a
     a28: 2800         	cmp	r0, #0x0
     a2a: d046         	beq	0xaba <<&T as core::fmt::Debug>::fmt::hd00725aac1204ea9+0x10e> @ imm = #0x8c
     a2c: 2001         	movs	r0, #0x1
     a2e: b00d         	add	sp, #0x34
     a30: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
     a34: bdf0         	pop	{r4, r5, r6, r7, pc}
     a36: f647 01be    	movw	r1, #0x78be
     a3a: 4658         	mov	r0, r11
     a3c: f2c0 0100    	movt	r1, #0x0
     a40: 2202         	movs	r2, #0x2
     a42: 47a8         	blx	r5
     a44: 4601         	mov	r1, r0
     a46: 2001         	movs	r0, #0x1
     a48: 2900         	cmp	r1, #0x0
     a4a: d1c5         	bne	0x9d8 <<&T as core::fmt::Debug>::fmt::hd00725aac1204ea9+0x2c> @ imm = #-0x76
     a4c: f104 0608    	add.w	r6, r4, #0x8
     a50: f1a7 0141    	sub.w	r1, r7, #0x41
     a54: f647 0598    	movw	r5, #0x7898
     a58: f807 0c41    	strb	r0, [r7, #-65]
     a5c: ce4c         	ldm	r6, {r2, r3, r6}
     a5e: 4668         	mov	r0, sp
     a60: f2c0 0500    	movt	r5, #0x0
     a64: e9cd 9101    	strd	r9, r1, [sp, #4]
     a68: e9d4 c100    	ldrd	r12, r1, [r4]
     a6c: 9009         	str	r0, [sp, #0x24]
     a6e: a806         	add	r0, sp, #0x18
     a70: e9cd c104    	strd	r12, r1, [sp, #16]
     a74: a904         	add	r1, sp, #0x10
     a76: 950a         	str	r5, [sp, #0x28]
     a78: f894 5020    	ldrb.w	r5, [r4, #0x20]
     a7c: c04c         	stm	r0!, {r2, r3, r6}
     a7e: 4640         	mov	r0, r8
     a80: f8cd b000    	str.w	r11, [sp]
     a84: f8cd a02c    	str.w	r10, [sp, #0x2c]
     a88: f88d 5030    	strb.w	r5, [sp, #0x30]
     a8c: f7ff ff17    	bl	0x8be <<&T as core::fmt::Debug>::fmt::h0b15e5a4be1f6b70> @ imm = #-0x1d2
     a90: b120         	cbz	r0, 0xa9c <<&T as core::fmt::Debug>::fmt::hd00725aac1204ea9+0xf0> @ imm = #0x8
     a92: 2001         	movs	r0, #0x1
     a94: b00d         	add	sp, #0x34
     a96: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
     a9a: bdf0         	pop	{r4, r5, r6, r7, pc}
     a9c: e9dd 0109    	ldrd	r0, r1, [sp, #36]
     aa0: 2202         	movs	r2, #0x2
     aa2: 68cb         	ldr	r3, [r1, #0xc]
     aa4: f647 01b8    	movw	r1, #0x78b8
     aa8: f2c0 0100    	movt	r1, #0x0
     aac: 4798         	blx	r3
     aae: b120         	cbz	r0, 0xaba <<&T as core::fmt::Debug>::fmt::hd00725aac1204ea9+0x10e> @ imm = #0x8
     ab0: 2001         	movs	r0, #0x1
     ab2: b00d         	add	sp, #0x34
     ab4: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
     ab8: bdf0         	pop	{r4, r5, r6, r7, pc}
     aba: e9d4 0105    	ldrd	r0, r1, [r4, #20]
     abe: 2201         	movs	r2, #0x1
     ac0: 68cb         	ldr	r3, [r1, #0xc]
     ac2: f647 010c    	movw	r1, #0x780c
     ac6: f2c0 0100    	movt	r1, #0x0
     aca: 4798         	blx	r3
     acc: b00d         	add	sp, #0x34
     ace: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
     ad2: bdf0         	pop	{r4, r5, r6, r7, pc}

00000ad4 <<() as core::fmt::Debug>::fmt::he6e1f5846864a732>:
     ad4: 4608         	mov	r0, r1
     ad6: f247 6198    	movw	r1, #0x7698
     ada: f2c0 0100    	movt	r1, #0x0
     ade: 2202         	movs	r2, #0x2
     ae0: f005 ba6a    	b.w	0x5fb8 <core::fmt::Formatter::pad::h2e2439709cd2f134> @ imm = #0x54d4

00000ae4 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e>:
     ae4: b5f0         	push	{r4, r5, r6, r7, lr}
     ae6: af03         	add	r7, sp, #0xc
     ae8: e92d 0f00    	push.w	{r8, r9, r10, r11}
     aec: b0a3         	sub	sp, #0x8c
     aee: 69ca         	ldr	r2, [r1, #0x1c]
     af0: 4689         	mov	r9, r1
     af2: 06d3         	lsls	r3, r2, #0x1b
     af4: d450         	bmi	0xb98 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0xb4> @ imm = #0xa0
     af6: 0692         	lsls	r2, r2, #0x1a
     af8: f100 8090    	bmi.w	0xc1c <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x138> @ imm = #0x120
     afc: 6805         	ldr	r5, [r0]
     afe: f242 7010    	movw	r0, #0x2710
     b02: f64f 719c    	movw	r1, #0xff9c
     b06: 4285         	cmp	r5, r0
     b08: f647 00e6    	movw	r0, #0x78e6
     b0c: f2c0 0000    	movt	r0, #0x0
     b10: f0c0 80cc    	blo.w	0xcac <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x1c8> @ imm = #0x198
     b14: f241 7c59    	movw	r12, #0x1759
     b18: f24e 0bff    	movw	r11, #0xe0ff
     b1c: f10d 0a0c    	add.w	r10, sp, #0xc
     b20: f8cd 9008    	str.w	r9, [sp, #0x8]
     b24: 2200         	movs	r2, #0x0
     b26: f2cd 1cb7    	movt	r12, #0xd1b7
     b2a: f64d 08f0    	movw	r8, #0xd8f0
     b2e: f241 497b    	movw	r9, #0x147b
     b32: f2c0 5bf5    	movt	r11, #0x5f5
     b36: fba5 360c    	umull	r3, r6, r5, r12
     b3a: eb0a 0e02    	add.w	lr, r10, r2
     b3e: 3a04         	subs	r2, #0x4
     b40: 455d         	cmp	r5, r11
     b42: ea4f 3356    	lsr.w	r3, r6, #0xd
     b46: fb03 5608    	mla	r6, r3, r8, r5
     b4a: 461d         	mov	r5, r3
     b4c: b2b4         	uxth	r4, r6
     b4e: ea4f 0494    	lsr.w	r4, r4, #0x2
     b52: fb04 f409    	mul	r4, r4, r9
     b56: ea4f 4454    	lsr.w	r4, r4, #0x11
     b5a: fb04 6601    	mla	r6, r4, r1, r6
     b5e: f830 4014    	ldrh.w	r4, [r0, r4, lsl #1]
     b62: f8ae 4023    	strh.w	r4, [lr, #0x23]
     b66: b2b6         	uxth	r6, r6
     b68: f830 6016    	ldrh.w	r6, [r0, r6, lsl #1]
     b6c: f8ae 6025    	strh.w	r6, [lr, #0x25]
     b70: d8e1         	bhi	0xb36 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x52> @ imm = #-0x3e
     b72: f8dd 9008    	ldr.w	r9, [sp, #0x8]
     b76: f102 0c27    	add.w	r12, r2, #0x27
     b7a: 461d         	mov	r5, r3
     b7c: 2d63         	cmp	r5, #0x63
     b7e: f200 809a    	bhi.w	0xcb6 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x1d2> @ imm = #0x134
     b82: 462a         	mov	r2, r5
     b84: 2a0a         	cmp	r2, #0xa
     b86: f0c0 80a9    	blo.w	0xcdc <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x1f8> @ imm = #0x152
     b8a: f1ac 0302    	sub.w	r3, r12, #0x2
     b8e: f830 0012    	ldrh.w	r0, [r0, r2, lsl #1]
     b92: a903         	add	r1, sp, #0xc
     b94: 52c8         	strh	r0, [r1, r3]
     b96: e0a7         	b	0xce8 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x204> @ imm = #0x14e
     b98: 6800         	ldr	r0, [r0]
     b9a: f10d 0c0c    	add.w	r12, sp, #0xc
     b9e: 2400         	movs	r4, #0x0
     ba0: f04f 0e57    	mov.w	lr, #0x57
     ba4: f000 030f    	and	r3, r0, #0xf
     ba8: 4622         	mov	r2, r4
     baa: f103 0557    	add.w	r5, r3, #0x57
     bae: 2b0a         	cmp	r3, #0xa
     bb0: 4464         	add	r4, r12
     bb2: f102 0880    	add.w	r8, r2, #0x80
     bb6: bf38         	it	lo
     bb8: f103 0530    	addlo.w	r5, r3, #0x30
     bbc: 2810         	cmp	r0, #0x10
     bbe: f884 507f    	strb.w	r5, [r4, #0x7f]
     bc2: d36b         	blo	0xc9c <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x1b8> @ imm = #0xd6
     bc4: b2c5         	uxtb	r5, r0
     bc6: 2da0         	cmp	r5, #0xa0
     bc8: eb0e 1615    	add.w	r6, lr, r5, lsr #4
     bcc: ea4f 1315    	lsr.w	r3, r5, #0x4
     bd0: bf38         	it	lo
     bd2: f103 0630    	addlo.w	r6, r3, #0x30
     bd6: 0a05         	lsrs	r5, r0, #0x8
     bd8: f884 607e    	strb.w	r6, [r4, #0x7e]
     bdc: d061         	beq	0xca2 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x1be> @ imm = #0xc2
     bde: f005 030f    	and	r3, r5, #0xf
     be2: f103 0557    	add.w	r5, r3, #0x57
     be6: 2b0a         	cmp	r3, #0xa
     be8: bf38         	it	lo
     bea: f103 0530    	addlo.w	r5, r3, #0x30
     bee: f884 507d    	strb.w	r5, [r4, #0x7d]
     bf2: 0b05         	lsrs	r5, r0, #0xc
     bf4: f000 8086    	beq.w	0xd04 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x220> @ imm = #0x10c
     bf8: f005 030f    	and	r3, r5, #0xf
     bfc: f103 0557    	add.w	r5, r3, #0x57
     c00: 2b0a         	cmp	r3, #0xa
     c02: bf38         	it	lo
     c04: f103 0530    	addlo.w	r5, r3, #0x30
     c08: f884 507c    	strb.w	r5, [r4, #0x7c]
     c0c: 1f14         	subs	r4, r2, #0x4
     c0e: 0c00         	lsrs	r0, r0, #0x10
     c10: d1c8         	bne	0xba4 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0xc0> @ imm = #-0x70
     c12: f102 007c    	add.w	r0, r2, #0x7c
     c16: f1a8 0803    	sub.w	r8, r8, #0x3
     c1a: e077         	b	0xd0c <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x228> @ imm = #0xee
     c1c: 6800         	ldr	r0, [r0]
     c1e: f10d 0c0c    	add.w	r12, sp, #0xc
     c22: 2400         	movs	r4, #0x0
     c24: f04f 0e37    	mov.w	lr, #0x37
     c28: f000 010f    	and	r1, r0, #0xf
     c2c: 4622         	mov	r2, r4
     c2e: f101 0337    	add.w	r3, r1, #0x37
     c32: 4464         	add	r4, r12
     c34: 290a         	cmp	r1, #0xa
     c36: bf38         	it	lo
     c38: f101 0330    	addlo.w	r3, r1, #0x30
     c3c: f884 307f    	strb.w	r3, [r4, #0x7f]
     c40: f102 0380    	add.w	r3, r2, #0x80
     c44: 2810         	cmp	r0, #0x10
     c46: d367         	blo	0xd18 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x234> @ imm = #0xce
     c48: b2c1         	uxtb	r1, r0
     c4a: 29a0         	cmp	r1, #0xa0
     c4c: eb0e 1511    	add.w	r5, lr, r1, lsr #4
     c50: ea4f 1611    	lsr.w	r6, r1, #0x4
     c54: bf38         	it	lo
     c56: f106 0530    	addlo.w	r5, r6, #0x30
     c5a: f884 507e    	strb.w	r5, [r4, #0x7e]
     c5e: 0a05         	lsrs	r5, r0, #0x8
     c60: d05c         	beq	0xd1c <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x238> @ imm = #0xb8
     c62: f005 010f    	and	r1, r5, #0xf
     c66: f101 0537    	add.w	r5, r1, #0x37
     c6a: 290a         	cmp	r1, #0xa
     c6c: bf38         	it	lo
     c6e: f101 0530    	addlo.w	r5, r1, #0x30
     c72: f884 507d    	strb.w	r5, [r4, #0x7d]
     c76: 0b05         	lsrs	r5, r0, #0xc
     c78: d053         	beq	0xd22 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x23e> @ imm = #0xa6
     c7a: f005 010f    	and	r1, r5, #0xf
     c7e: f101 0537    	add.w	r5, r1, #0x37
     c82: 290a         	cmp	r1, #0xa
     c84: bf38         	it	lo
     c86: f101 0530    	addlo.w	r5, r1, #0x30
     c8a: f884 507c    	strb.w	r5, [r4, #0x7c]
     c8e: 1f14         	subs	r4, r2, #0x4
     c90: 0c00         	lsrs	r0, r0, #0x10
     c92: d1c9         	bne	0xc28 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x144> @ imm = #-0x6e
     c94: f102 007c    	add.w	r0, r2, #0x7c
     c98: 3b03         	subs	r3, #0x3
     c9a: e044         	b	0xd26 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x242> @ imm = #0x88
     c9c: f1a8 0001    	sub.w	r0, r8, #0x1
     ca0: e034         	b	0xd0c <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x228> @ imm = #0x68
     ca2: f1a8 0002    	sub.w	r0, r8, #0x2
     ca6: f1a8 0801    	sub.w	r8, r8, #0x1
     caa: e02f         	b	0xd0c <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x228> @ imm = #0x5e
     cac: f04f 0c27    	mov.w	r12, #0x27
     cb0: 2d63         	cmp	r5, #0x63
     cb2: f67f af66    	bls.w	0xb82 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x9e> @ imm = #-0x134
     cb6: b2aa         	uxth	r2, r5
     cb8: f241 437b    	movw	r3, #0x147b
     cbc: 0892         	lsrs	r2, r2, #0x2
     cbe: f1ac 0c02    	sub.w	r12, r12, #0x2
     cc2: 435a         	muls	r2, r3, r2
     cc4: ab03         	add	r3, sp, #0xc
     cc6: 0c52         	lsrs	r2, r2, #0x11
     cc8: fb02 5101    	mla	r1, r2, r1, r5
     ccc: b289         	uxth	r1, r1
     cce: f830 1011    	ldrh.w	r1, [r0, r1, lsl #1]
     cd2: f823 100c    	strh.w	r1, [r3, r12]
     cd6: 2a0a         	cmp	r2, #0xa
     cd8: f4bf af57    	bhs.w	0xb8a <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0xa6> @ imm = #-0x152
     cdc: f1ac 0301    	sub.w	r3, r12, #0x1
     ce0: a903         	add	r1, sp, #0xc
     ce2: f042 0030    	orr	r0, r2, #0x30
     ce6: 54c8         	strb	r0, [r1, r3]
     ce8: f1c3 0027    	rsb.w	r0, r3, #0x27
     cec: 9000         	str	r0, [sp]
     cee: a803         	add	r0, sp, #0xc
     cf0: 2101         	movs	r1, #0x1
     cf2: 4403         	add	r3, r0
     cf4: 4648         	mov	r0, r9
     cf6: 2200         	movs	r2, #0x0
     cf8: f005 f818    	bl	0x5d2c <core::fmt::Formatter::pad_integral::h9814144aa367965a> @ imm = #0x5030
     cfc: b023         	add	sp, #0x8c
     cfe: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
     d02: bdf0         	pop	{r4, r5, r6, r7, pc}
     d04: f1a8 0003    	sub.w	r0, r8, #0x3
     d08: f1a8 0802    	sub.w	r8, r8, #0x2
     d0c: 2881         	cmp	r0, #0x81
     d0e: d21d         	bhs	0xd4c <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x268> @ imm = #0x3a
     d10: f1c8 0281    	rsb.w	r2, r8, #0x81
     d14: 9200         	str	r2, [sp]
     d16: e00b         	b	0xd30 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x24c> @ imm = #0x16
     d18: 1e58         	subs	r0, r3, #0x1
     d1a: e004         	b	0xd26 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x242> @ imm = #0x8
     d1c: 1e98         	subs	r0, r3, #0x2
     d1e: 3b01         	subs	r3, #0x1
     d20: e001         	b	0xd26 <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x242> @ imm = #0x2
     d22: 1ed8         	subs	r0, r3, #0x3
     d24: 3b02         	subs	r3, #0x2
     d26: 2881         	cmp	r0, #0x81
     d28: d210         	bhs	0xd4c <core::fmt::num::<impl core::fmt::Debug for usize>::fmt::h645c439b15c4758e+0x268> @ imm = #0x20
     d2a: f1c3 0181    	rsb.w	r1, r3, #0x81
     d2e: 9100         	str	r1, [sp]
     d30: f647 01e4    	movw	r1, #0x78e4
     d34: eb0c 0300    	add.w	r3, r12, r0
     d38: f2c0 0100    	movt	r1, #0x0
     d3c: 4648         	mov	r0, r9
     d3e: 2202         	movs	r2, #0x2
     d40: f004 fff4    	bl	0x5d2c <core::fmt::Formatter::pad_integral::h9814144aa367965a> @ imm = #0x4fe8
     d44: b023         	add	sp, #0x8c
     d46: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
     d4a: bdf0         	pop	{r4, r5, r6, r7, pc}
     d4c: f647 02d4    	movw	r2, #0x78d4
     d50: 2180         	movs	r1, #0x80
     d52: f2c0 0200    	movt	r2, #0x0
     d56: f004 ff24    	bl	0x5ba2 <core::slice::index::slice_start_index_len_fail::h0109ceeb844e56f0> @ imm = #0x4e48

00000d5a <core::ops::function::FnOnce::call_once::h38acd8772f0ad170>:
     d5a: f240 2050    	movw	r0, #0x250
     d5e: 2101         	movs	r1, #0x1
     d60: f2c2 0000    	movt	r0, #0x2000
     d64: 6c00         	ldr	r0, [r0, #0x40]
     d66: f3bf 8f5f    	dmb	sy
     d6a: f880 1099    	strb.w	r1, [r0, #0x99]
     d6e: f24e 2000    	movw	r0, #0xe200
     d72: f2ce 0000    	movt	r0, #0xe000
     d76: f44f 7100    	mov.w	r1, #0x200
     d7a: 6001         	str	r1, [r0]
     d7c: 4770         	bx	lr

00000d7e <core::ops::function::FnOnce::call_once::h3be2bacdc512def3>:
     d7e: f240 2050    	movw	r0, #0x250
     d82: 2101         	movs	r1, #0x1
     d84: f2c2 0000    	movt	r0, #0x2000
     d88: 6b80         	ldr	r0, [r0, #0x38]
     d8a: f3bf 8f5f    	dmb	sy
     d8e: f880 1021    	strb.w	r1, [r0, #0x21]
     d92: f24e 2000    	movw	r0, #0xe200
     d96: f2ce 0000    	movt	r0, #0xe000
     d9a: f44f 7100    	mov.w	r1, #0x200
     d9e: 6001         	str	r1, [r0]
     da0: 4770         	bx	lr

00000da2 <core::ops::function::FnOnce::call_once::h64b6d5178835f3f3>:
     da2: f240 2050    	movw	r0, #0x250
     da6: 2101         	movs	r1, #0x1
     da8: f2c2 0000    	movt	r0, #0x2000
     dac: 6bc0         	ldr	r0, [r0, #0x3c]
     dae: f3bf 8f5f    	dmb	sy
     db2: f880 10c1    	strb.w	r1, [r0, #0xc1]
     db6: f24e 2000    	movw	r0, #0xe200
     dba: f2ce 0000    	movt	r0, #0xe000
     dbe: f44f 7180    	mov.w	r1, #0x100
     dc2: 6001         	str	r1, [r0]
     dc4: 4770         	bx	lr

00000dc6 <core::ops::function::FnOnce::call_once::hc29d19ccf1f9caef>:
     dc6: f240 2050    	movw	r0, #0x250
     dca: 2101         	movs	r1, #0x1
     dcc: f2c2 0000    	movt	r0, #0x2000
     dd0: 6c40         	ldr	r0, [r0, #0x44]
     dd2: f3bf 8f5f    	dmb	sy
     dd6: f880 10a9    	strb.w	r1, [r0, #0xa9]
     dda: f24e 2000    	movw	r0, #0xe200
     dde: f2ce 0000    	movt	r0, #0xe000
     de2: f44f 7180    	mov.w	r1, #0x100
     de6: 6001         	str	r1, [r0]
     de8: 4770         	bx	lr

00000dea <<core::str::error::Utf8Error as core::fmt::Debug>::fmt::he8185bd1721fda51>:
     dea: b5b0         	push	{r4, r5, r7, lr}
     dec: af02         	add	r7, sp, #0x8
     dee: b084         	sub	sp, #0x10
     df0: 4604         	mov	r4, r0
     df2: 3004         	adds	r0, #0x4
     df4: 460d         	mov	r5, r1
     df6: 9001         	str	r0, [sp, #0x4]
     df8: e9d1 0105    	ldrd	r0, r1, [r1, #20]
     dfc: 2209         	movs	r2, #0x9
     dfe: 68cb         	ldr	r3, [r1, #0xc]
     e00: f247 619a    	movw	r1, #0x769a
     e04: f2c0 0100    	movt	r1, #0x0
     e08: 4798         	blx	r3
     e0a: f88d 000c    	strb.w	r0, [sp, #0xc]
     e0e: f640 20e5    	movw	r0, #0xae5
     e12: 2100         	movs	r1, #0x0
     e14: f2c0 0000    	movt	r0, #0x0
     e18: f88d 100d    	strb.w	r1, [sp, #0xd]
     e1c: f247 61a3    	movw	r1, #0x76a3
     e20: 9000         	str	r0, [sp]
     e22: a802         	add	r0, sp, #0x8
     e24: f2c0 0100    	movt	r1, #0x0
     e28: 220b         	movs	r2, #0xb
     e2a: 4623         	mov	r3, r4
     e2c: 9502         	str	r5, [sp, #0x8]
     e2e: f005 fd1b    	bl	0x6868 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714> @ imm = #0x5a36
     e32: f640 11ad    	movw	r1, #0x9ad
     e36: ab01         	add	r3, sp, #0x4
     e38: f2c0 0100    	movt	r1, #0x0
     e3c: 2209         	movs	r2, #0x9
     e3e: 9100         	str	r1, [sp]
     e40: f247 61ae    	movw	r1, #0x76ae
     e44: f2c0 0100    	movt	r1, #0x0
     e48: f005 fd0e    	bl	0x6868 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714> @ imm = #0x5a1c
     e4c: f89d 200d    	ldrb.w	r2, [sp, #0xd]
     e50: f89d 100c    	ldrb.w	r1, [sp, #0xc]
     e54: 2a01         	cmp	r2, #0x1
     e56: ea42 0001    	orr.w	r0, r2, r1
     e5a: bf08         	it	eq
     e5c: ea5f 71c1    	lslseq.w	r1, r1, #0x1f
     e60: d003         	beq	0xe6a <<core::str::error::Utf8Error as core::fmt::Debug>::fmt::he8185bd1721fda51+0x80> @ imm = #0x6
     e62: f000 0001    	and	r0, r0, #0x1
     e66: b004         	add	sp, #0x10
     e68: bdb0         	pop	{r4, r5, r7, pc}
     e6a: 9802         	ldr	r0, [sp, #0x8]
     e6c: 7f01         	ldrb	r1, [r0, #0x1c]
     e6e: 0749         	lsls	r1, r1, #0x1d
     e70: d40c         	bmi	0xe8c <<core::str::error::Utf8Error as core::fmt::Debug>::fmt::he8185bd1721fda51+0xa2> @ imm = #0x18
     e72: e9d0 0105    	ldrd	r0, r1, [r0, #20]
     e76: 2202         	movs	r2, #0x2
     e78: 68cb         	ldr	r3, [r1, #0xc]
     e7a: f647 01bb    	movw	r1, #0x78bb
     e7e: f2c0 0100    	movt	r1, #0x0
     e82: 4798         	blx	r3
     e84: f000 0001    	and	r0, r0, #0x1
     e88: b004         	add	sp, #0x10
     e8a: bdb0         	pop	{r4, r5, r7, pc}
     e8c: e9d0 0105    	ldrd	r0, r1, [r0, #20]
     e90: 2201         	movs	r2, #0x1
     e92: 68cb         	ldr	r3, [r1, #0xc]
     e94: f647 01ba    	movw	r1, #0x78ba
     e98: f2c0 0100    	movt	r1, #0x0
     e9c: 4798         	blx	r3
     e9e: f000 0001    	and	r0, r0, #0x1
     ea2: b004         	add	sp, #0x10
     ea4: bdb0         	pop	{r4, r5, r7, pc}

00000ea6 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6>:
     ea6: b5f0         	push	{r4, r5, r6, r7, lr}
     ea8: af03         	add	r7, sp, #0xc
     eaa: e92d 0f00    	push.w	{r8, r9, r10, r11}
     eae: b089         	sub	sp, #0x24
     eb0: 4683         	mov	r11, r0
     eb2: f890 0088    	ldrb.w	r0, [r0, #0x88]
     eb6: f24b 5a04    	movw	r10, #0xb504
     eba: f240 2950    	movw	r9, #0x250
     ebe: f2c4 0a00    	movt	r10, #0x4000
     ec2: f2c2 0900    	movt	r9, #0x2000
     ec6: b178         	cbz	r0, 0xee8 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x42> @ imm = #0x1e
     ec8: 2803         	cmp	r0, #0x3
     eca: d10b         	bne	0xee4 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3e> @ imm = #0x16
     ecc: f89b 0084    	ldrb.w	r0, [r11, #0x84]
     ed0: b1f0         	cbz	r0, 0xf10 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x6a> @ imm = #0x3c
     ed2: 2803         	cmp	r0, #0x3
     ed4: d106         	bne	0xee4 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3e> @ imm = #0xc
     ed6: f89b 0079    	ldrb.w	r0, [r11, #0x79]
     eda: 2800         	cmp	r0, #0x0
     edc: f000 812b    	beq.w	0x1136 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x290> @ imm = #0x256
     ee0: 2803         	cmp	r0, #0x3
     ee2: d077         	beq	0xfd4 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x12e> @ imm = #0xee
     ee4: f005 fee5    	bl	0x6cb2 <core::panicking::panic_const::panic_const_async_fn_resumed::h274f44943fe874ff> @ imm = #0x5dca
     ee8: 9102         	str	r1, [sp, #0x8]
     eea: 4659         	mov	r1, r11
     eec: e9db b200    	ldrd	r11, r2, [r11]
     ef0: 2000         	movs	r0, #0x0
     ef2: f881 0084    	strb.w	r0, [r1, #0x84]
     ef6: f240 20c8    	movw	r0, #0x2c8
     efa: f2c2 0000    	movt	r0, #0x2000
     efe: f100 0810    	add.w	r8, r0, #0x10
     f02: f8c1 8080    	str.w	r8, [r1, #0x80]
     f06: 460c         	mov	r4, r1
     f08: 9201         	str	r2, [sp, #0x4]
     f0a: e9c1 b202    	strd	r11, r2, [r1, #8]
     f0e: e007         	b	0xf20 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x7a> @ imm = #0xe
     f10: 4658         	mov	r0, r11
     f12: 9102         	str	r1, [sp, #0x8]
     f14: e9db b102    	ldrd	r11, r1, [r11, #8]
     f18: 4604         	mov	r4, r0
     f1a: f8d0 8080    	ldr.w	r8, [r0, #0x80]
     f1e: 9101         	str	r1, [sp, #0x4]
     f20: f006 fa36    	bl	0x7390 <__primask_r>    @ imm = #0x646c
     f24: 4606         	mov	r6, r0
     f26: f006 fa2c    	bl	0x7382 <__cpsid>        @ imm = #0x6458
     f2a: 07f0         	lsls	r0, r6, #0x1f
     f2c: f8d9 9048    	ldr.w	r9, [r9, #0x48]
     f30: f8da a000    	ldr.w	r10, [r10]
     f34: bf08         	it	eq
     f36: f006 fa26    	bleq	0x7386 <__cpsie>        @ imm = #0x644c
     f3a: 2000         	movs	r0, #0x0
     f3c: ea4f 2159    	lsr.w	r1, r9, #0x9
     f40: f884 0079    	strb.w	r0, [r4, #0x79]
     f44: f009 0001    	and	r0, r9, #0x1
     f48: 9e01         	ldr	r6, [sp, #0x4]
     f4a: 4625         	mov	r5, r4
     f4c: ea8a 50c0    	eor.w	r0, r10, r0, lsl #23
     f50: f8c4 8074    	str.w	r8, [r4, #0x74]
     f54: eb10 50c9    	adds.w	r0, r0, r9, lsl #23
     f58: f24b 5a04    	movw	r10, #0xb504
     f5c: f141 0300    	adc	r3, r1, #0x0
     f60: eb1b 0100    	adds.w	r1, r11, r0
     f64: eb46 0203    	adc.w	r2, r6, r3
     f68: 1c4c         	adds	r4, r1, #0x1
     f6a: f240 2950    	movw	r9, #0x250
     f6e: f142 0200    	adc	r2, r2, #0x0
     f72: ea56 060b    	orrs.w	r6, r6, r11
     f76: 46ab         	mov	r11, r5
     f78: f105 0110    	add.w	r1, r5, #0x10
     f7c: bf04         	itt	eq
     f7e: 4604         	moveq	r4, r0
     f80: 461a         	moveq	r2, r3
     f82: c119         	stm	r1!, {r0, r3, r4}
     f84: f105 0020    	add.w	r0, r5, #0x20
     f88: f2c4 0a00    	movt	r10, #0x4000
     f8c: e9cb 4214    	strd	r4, r2, [r11, #80]
     f90: f2c2 0900    	movt	r9, #0x2000
     f94: 61ea         	str	r2, [r5, #0x1c]
     f96: 9902         	ldr	r1, [sp, #0x8]
     f98: 465b         	mov	r3, r11
     f9a: f843 4f58    	str	r4, [r3, #88]!
     f9e: 2400         	movs	r4, #0x0
     fa0: 605a         	str	r2, [r3, #0x4]
     fa2: f883 4020    	strb.w	r4, [r3, #0x20]
     fa6: f898 2004    	ldrb.w	r2, [r8, #0x4]
     faa: 2a00         	cmp	r2, #0x0
     fac: f000 8162    	beq.w	0x1274 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3ce> @ imm = #0x2c4
     fb0: 465a         	mov	r2, r11
     fb2: 2601         	movs	r6, #0x1
     fb4: f842 0f6c    	str	r0, [r2, #108]!
     fb8: 7316         	strb	r6, [r2, #0xc]
     fba: 4616         	mov	r6, r2
     fbc: f846 4f04    	str	r4, [r6, #4]!
     fc0: f842 4c4c    	str	r4, [r2, #-76]
     fc4: e942 8603    	strd	r8, r6, [r2, #-12]
     fc8: f842 0c04    	str	r0, [r2, #-4]
     fcc: e942 320b    	strd	r3, r2, [r2, #-44]
     fd0: e942 8609    	strd	r8, r6, [r2, #-36]
     fd4: f8d1 8000    	ldr.w	r8, [r1]
     fd8: f006 f9da    	bl	0x7390 <__primask_r>    @ imm = #0x63b4
     fdc: 4605         	mov	r5, r0
     fde: f006 f9d0    	bl	0x7382 <__cpsid>        @ imm = #0x63a0
     fe2: 07e8         	lsls	r0, r5, #0x1f
     fe4: f8d9 6048    	ldr.w	r6, [r9, #0x48]
     fe8: f8da 4000    	ldr.w	r4, [r10]
     fec: bf08         	it	eq
     fee: f006 f9ca    	bleq	0x7386 <__cpsie>        @ imm = #0x6394
     ff2: f006 0001    	and	r0, r6, #0x1
     ff6: f8db 5040    	ldr.w	r5, [r11, #0x40]
     ffa: 0a73         	lsrs	r3, r6, #0x9
     ffc: ea84 50c0    	eor.w	r0, r4, r0, lsl #23
    1000: e9d5 1200    	ldrd	r1, r2, [r5]
    1004: eb10 50c6    	adds.w	r0, r0, r6, lsl #23
    1008: f143 0300    	adc	r3, r3, #0x0
    100c: 1a40         	subs	r0, r0, r1
    100e: eb63 0a02    	sbc.w	r10, r3, r2
    1012: f1ba 3fff    	cmp.w	r10, #0xffffffff
    1016: dd10         	ble	0x103a <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x194> @ imm = #0x20
    1018: f8db 0068    	ldr.w	r0, [r11, #0x68]
    101c: 6801         	ldr	r1, [r0]
    101e: 2900         	cmp	r1, #0x0
    1020: d075         	beq	0x110e <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x268> @ imm = #0xea
    1022: 7c00         	ldrb	r0, [r0, #0x10]
    1024: 2800         	cmp	r0, #0x0
    1026: bf1c         	itt	ne
    1028: 2000         	movne	r0, #0x0
    102a: f88b 0078    	strbne.w	r0, [r11, #0x78]
    102e: f89b 0078    	ldrb.w	r0, [r11, #0x78]
    1032: 2800         	cmp	r0, #0x0
    1034: f040 8094    	bne.w	0x1160 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2ba> @ imm = #0x128
    1038: e0c4         	b	0x11c4 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x31e> @ imm = #0x188
    103a: f8db 0044    	ldr.w	r0, [r11, #0x44]
    103e: 6806         	ldr	r6, [r0]
    1040: 6830         	ldr	r0, [r6]
    1042: 2800         	cmp	r0, #0x0
    1044: f040 8109    	bne.w	0x125a <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3b4> @ imm = #0x212
    1048: e9d8 1000    	ldrd	r1, r0, [r8]
    104c: 6809         	ldr	r1, [r1]
    104e: 4788         	blx	r1
    1050: 4680         	mov	r8, r0
    1052: 6830         	ldr	r0, [r6]
    1054: 4689         	mov	r9, r1
    1056: e9d5 4500    	ldrd	r4, r5, [r5]
    105a: 2800         	cmp	r0, #0x0
    105c: bf1e         	ittt	ne
    105e: 68c1         	ldrne	r1, [r0, #0xc]
    1060: 6870         	ldrne	r0, [r6, #0x4]
    1062: 4788         	blxne	r1
    1064: 2000         	movs	r0, #0x0
    1066: 60b4         	str	r4, [r6, #0x8]
    1068: 61b0         	str	r0, [r6, #0x18]
    106a: 7430         	strb	r0, [r6, #0x10]
    106c: e9c6 8900    	strd	r8, r9, [r6]
    1070: f8db 4048    	ldr.w	r4, [r11, #0x48]
    1074: 60f5         	str	r5, [r6, #0xc]
    1076: f006 f98b    	bl	0x7390 <__primask_r>    @ imm = #0x6316
    107a: 4680         	mov	r8, r0
    107c: f006 f981    	bl	0x7382 <__cpsid>        @ imm = #0x6302
    1080: f3bf 8f5f    	dmb	sy
    1084: 6820         	ldr	r0, [r4]
    1086: 2800         	cmp	r0, #0x0
    1088: f000 80c0    	beq.w	0x120c <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x366> @ imm = #0x180
    108c: e9d0 3502    	ldrd	r3, r5, [r0, #8]
    1090: e9d6 1202    	ldrd	r1, r2, [r6, #8]
    1094: 1acb         	subs	r3, r1, r3
    1096: eb72 0305    	sbcs.w	r3, r2, r5
    109a: f100 80b6    	bmi.w	0x120a <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x364> @ imm = #0x16c
    109e: 6983         	ldr	r3, [r0, #0x18]
    10a0: 2b00         	cmp	r3, #0x0
    10a2: f000 80d1    	beq.w	0x1248 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3a2> @ imm = #0x1a2
    10a6: e9d3 5402    	ldrd	r5, r4, [r3, #8]
    10aa: 1b4d         	subs	r5, r1, r5
    10ac: eb72 0504    	sbcs.w	r5, r2, r4
    10b0: f100 80bd    	bmi.w	0x122e <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x388> @ imm = #0x17a
    10b4: 6998         	ldr	r0, [r3, #0x18]
    10b6: 2800         	cmp	r0, #0x0
    10b8: f000 80c1    	beq.w	0x123e <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x398> @ imm = #0x182
    10bc: e9d0 5402    	ldrd	r5, r4, [r0, #8]
    10c0: 1b4d         	subs	r5, r1, r5
    10c2: eb72 0504    	sbcs.w	r5, r2, r4
    10c6: f100 80bc    	bmi.w	0x1242 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x39c> @ imm = #0x178
    10ca: 6983         	ldr	r3, [r0, #0x18]
    10cc: 2b00         	cmp	r3, #0x0
    10ce: f000 80bb    	beq.w	0x1248 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3a2> @ imm = #0x176
    10d2: e9d3 5402    	ldrd	r5, r4, [r3, #8]
    10d6: 1b4d         	subs	r5, r1, r5
    10d8: eb72 0504    	sbcs.w	r5, r2, r4
    10dc: f100 80a7    	bmi.w	0x122e <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x388> @ imm = #0x14e
    10e0: 6998         	ldr	r0, [r3, #0x18]
    10e2: 2800         	cmp	r0, #0x0
    10e4: f000 80ab    	beq.w	0x123e <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x398> @ imm = #0x156
    10e8: e9d0 5402    	ldrd	r5, r4, [r0, #8]
    10ec: 1b4d         	subs	r5, r1, r5
    10ee: eb72 0504    	sbcs.w	r5, r2, r4
    10f2: f100 80a6    	bmi.w	0x1242 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x39c> @ imm = #0x14c
    10f6: 6983         	ldr	r3, [r0, #0x18]
    10f8: 2b00         	cmp	r3, #0x0
    10fa: f000 80a5    	beq.w	0x1248 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3a2> @ imm = #0x14a
    10fe: e9d3 5402    	ldrd	r5, r4, [r3, #8]
    1102: 1b4d         	subs	r5, r1, r5
    1104: eb72 0504    	sbcs.w	r5, r2, r4
    1108: 461d         	mov	r5, r3
    110a: d5d3         	bpl	0x10b4 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x20e> @ imm = #-0x5a
    110c: e09b         	b	0x1246 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3a0> @ imm = #0x136
    110e: 2100         	movs	r1, #0x0
    1110: e9db 4018    	ldrd	r4, r0, [r11, #96]
    1114: f88b 1078    	strb.w	r1, [r11, #0x78]
    1118: 6806         	ldr	r6, [r0]
    111a: f006 f939    	bl	0x7390 <__primask_r>    @ imm = #0x6272
    111e: 4680         	mov	r8, r0
    1120: f006 f92f    	bl	0x7382 <__cpsid>        @ imm = #0x625e
    1124: f3bf 8f5f    	dmb	sy
    1128: 6820         	ldr	r0, [r4]
    112a: b188         	cbz	r0, 0x1150 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2aa> @ imm = #0x22
    112c: 6981         	ldr	r1, [r0, #0x18]
    112e: 4286         	cmp	r6, r0
    1130: d108         	bne	0x1144 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x29e> @ imm = #0x10
    1132: 6021         	str	r1, [r4]
    1134: e00c         	b	0x1150 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2aa> @ imm = #0x18
    1136: e9db 4214    	ldrd	r4, r2, [r11, #80]
    113a: f10b 0020    	add.w	r0, r11, #0x20
    113e: f8db 8074    	ldr.w	r8, [r11, #0x74]
    1142: e729         	b	0xf98 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0xf2> @ imm = #-0x1ae
    1144: b121         	cbz	r1, 0x1150 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2aa> @ imm = #0x8
    1146: 428e         	cmp	r6, r1
    1148: d11b         	bne	0x1182 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2dc> @ imm = #0x36
    114a: 460a         	mov	r2, r1
    114c: 6991         	ldr	r1, [r2, #0x18]
    114e: 6181         	str	r1, [r0, #0x18]
    1150: ea5f 70c8    	lsls.w	r0, r8, #0x1f
    1154: bf08         	it	eq
    1156: f006 f916    	bleq	0x7386 <__cpsie>        @ imm = #0x622c
    115a: f89b 0078    	ldrb.w	r0, [r11, #0x78]
    115e: b388         	cbz	r0, 0x11c4 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x31e> @ imm = #0x62
    1160: e9db 4018    	ldrd	r4, r0, [r11, #96]
    1164: 6806         	ldr	r6, [r0]
    1166: f006 f913    	bl	0x7390 <__primask_r>    @ imm = #0x6226
    116a: 4680         	mov	r8, r0
    116c: f006 f909    	bl	0x7382 <__cpsid>        @ imm = #0x6212
    1170: f3bf 8f5f    	dmb	sy
    1174: 6820         	ldr	r0, [r4]
    1176: b300         	cbz	r0, 0x11ba <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x314> @ imm = #0x40
    1178: 6981         	ldr	r1, [r0, #0x18]
    117a: 4286         	cmp	r6, r0
    117c: d117         	bne	0x11ae <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x308> @ imm = #0x2e
    117e: 6021         	str	r1, [r4]
    1180: e01b         	b	0x11ba <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x314> @ imm = #0x36
    1182: 6988         	ldr	r0, [r1, #0x18]
    1184: 2800         	cmp	r0, #0x0
    1186: d0e3         	beq	0x1150 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2aa> @ imm = #-0x3a
    1188: 4286         	cmp	r6, r0
    118a: d055         	beq	0x1238 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x392> @ imm = #0xaa
    118c: 6981         	ldr	r1, [r0, #0x18]
    118e: 2900         	cmp	r1, #0x0
    1190: d0de         	beq	0x1150 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2aa> @ imm = #-0x44
    1192: 428e         	cmp	r6, r1
    1194: d0d9         	beq	0x114a <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2a4> @ imm = #-0x4e
    1196: 6988         	ldr	r0, [r1, #0x18]
    1198: 2800         	cmp	r0, #0x0
    119a: d0d9         	beq	0x1150 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2aa> @ imm = #-0x4e
    119c: 4286         	cmp	r6, r0
    119e: d04b         	beq	0x1238 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x392> @ imm = #0x96
    11a0: 6981         	ldr	r1, [r0, #0x18]
    11a2: 2900         	cmp	r1, #0x0
    11a4: d0d4         	beq	0x1150 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2aa> @ imm = #-0x58
    11a6: 428e         	cmp	r6, r1
    11a8: 460a         	mov	r2, r1
    11aa: d1ea         	bne	0x1182 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2dc> @ imm = #-0x2c
    11ac: e7ce         	b	0x114c <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2a6> @ imm = #-0x64
    11ae: b121         	cbz	r1, 0x11ba <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x314> @ imm = #0x8
    11b0: 428e         	cmp	r6, r1
    11b2: d114         	bne	0x11de <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x338> @ imm = #0x28
    11b4: 460a         	mov	r2, r1
    11b6: 6991         	ldr	r1, [r2, #0x18]
    11b8: 6181         	str	r1, [r0, #0x18]
    11ba: ea5f 70c8    	lsls.w	r0, r8, #0x1f
    11be: d101         	bne	0x11c4 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x31e> @ imm = #0x2
    11c0: f006 f8e1    	bl	0x7386 <__cpsie>        @ imm = #0x61c2
    11c4: 2000         	movs	r0, #0x0
    11c6: f88b 0078    	strb.w	r0, [r11, #0x78]
    11ca: f8db 0020    	ldr.w	r0, [r11, #0x20]
    11ce: 2800         	cmp	r0, #0x0
    11d0: bf1e         	ittt	ne
    11d2: 68c1         	ldrne	r1, [r0, #0xc]
    11d4: f8db 0024    	ldrne.w	r0, [r11, #0x24]
    11d8: 4788         	blxne	r1
    11da: 2001         	movs	r0, #0x1
    11dc: e03e         	b	0x125c <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3b6> @ imm = #0x7c
    11de: 6988         	ldr	r0, [r1, #0x18]
    11e0: 2800         	cmp	r0, #0x0
    11e2: d0ea         	beq	0x11ba <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x314> @ imm = #-0x2c
    11e4: 4286         	cmp	r6, r0
    11e6: d024         	beq	0x1232 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x38c> @ imm = #0x48
    11e8: 6981         	ldr	r1, [r0, #0x18]
    11ea: 2900         	cmp	r1, #0x0
    11ec: d0e5         	beq	0x11ba <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x314> @ imm = #-0x36
    11ee: 428e         	cmp	r6, r1
    11f0: d0e0         	beq	0x11b4 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x30e> @ imm = #-0x40
    11f2: 6988         	ldr	r0, [r1, #0x18]
    11f4: 2800         	cmp	r0, #0x0
    11f6: d0e0         	beq	0x11ba <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x314> @ imm = #-0x40
    11f8: 4286         	cmp	r6, r0
    11fa: d01a         	beq	0x1232 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x38c> @ imm = #0x34
    11fc: 6981         	ldr	r1, [r0, #0x18]
    11fe: 2900         	cmp	r1, #0x0
    1200: d0db         	beq	0x11ba <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x314> @ imm = #-0x4a
    1202: 428e         	cmp	r6, r1
    1204: 460a         	mov	r2, r1
    1206: d1ea         	bne	0x11de <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x338> @ imm = #-0x2c
    1208: e7d5         	b	0x11b6 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x310> @ imm = #-0x56
    120a: 61b0         	str	r0, [r6, #0x18]
    120c: ea5f 70c8    	lsls.w	r0, r8, #0x1f
    1210: 6026         	str	r6, [r4]
    1212: bf08         	it	eq
    1214: f006 f8b7    	bleq	0x7386 <__cpsie>        @ imm = #0x616e
    1218: f8db 004c    	ldr.w	r0, [r11, #0x4c]
    121c: f44f 6100    	mov.w	r1, #0x800
    1220: 6006         	str	r6, [r0]
    1222: f24e 2000    	movw	r0, #0xe200
    1226: f2ce 0000    	movt	r0, #0xe000
    122a: 6001         	str	r1, [r0]
    122c: e015         	b	0x125a <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3b4> @ imm = #0x2a
    122e: 461d         	mov	r5, r3
    1230: e009         	b	0x1246 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3a0> @ imm = #0x12
    1232: 4602         	mov	r2, r0
    1234: 4608         	mov	r0, r1
    1236: e7be         	b	0x11b6 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x310> @ imm = #-0x84
    1238: 4602         	mov	r2, r0
    123a: 4608         	mov	r0, r1
    123c: e786         	b	0x114c <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x2a6> @ imm = #-0xf4
    123e: 4618         	mov	r0, r3
    1240: e002         	b	0x1248 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6+0x3a2> @ imm = #0x4
    1242: 4605         	mov	r5, r0
    1244: 4618         	mov	r0, r3
    1246: 61b5         	str	r5, [r6, #0x18]
    1248: 6186         	str	r6, [r0, #0x18]
    124a: ea5f 70c8    	lsls.w	r0, r8, #0x1f
    124e: bf08         	it	eq
    1250: f006 f899    	bleq	0x7386 <__cpsie>        @ imm = #0x6132
    1254: f8db 004c    	ldr.w	r0, [r11, #0x4c]
    1258: 6006         	str	r6, [r0]
    125a: 2003         	movs	r0, #0x3
    125c: f88b 0088    	strb.w	r0, [r11, #0x88]
    1260: f88b 0084    	strb.w	r0, [r11, #0x84]
    1264: f88b 0079    	strb.w	r0, [r11, #0x79]
    1268: ea4f 70da    	lsr.w	r0, r10, #0x1f
    126c: b009         	add	sp, #0x24
    126e: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    1272: bdf0         	pop	{r4, r5, r6, r7, pc}
    1274: 2001         	movs	r0, #0x1
    1276: f647 31ac    	movw	r1, #0x7bac
    127a: 9004         	str	r0, [sp, #0x10]
    127c: f247 7038    	movw	r0, #0x7738
    1280: f2c0 0000    	movt	r0, #0x0
    1284: f2c0 0100    	movt	r1, #0x0
    1288: 9003         	str	r0, [sp, #0xc]
    128a: 2004         	movs	r0, #0x4
    128c: 9005         	str	r0, [sp, #0x14]
    128e: a803         	add	r0, sp, #0xc
    1290: 9407         	str	r4, [sp, #0x1c]
    1292: 9406         	str	r4, [sp, #0x18]
    1294: f004 fd39    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #0x4a72

00001298 <RTC0>:
    1298: b5f0         	push	{r4, r5, r6, r7, lr}
    129a: af03         	add	r7, sp, #0xc
    129c: e92d 0f00    	push.w	{r8, r9, r10, r11}
    12a0: b089         	sub	sp, #0x24
    12a2: f64f 413c    	movw	r1, #0xfc3c
    12a6: f24b 5a04    	movw	r10, #0xb504
    12aa: f6cf 71ff    	movt	r1, #0xffff
    12ae: f2c4 0a00    	movt	r10, #0x4000
    12b2: 2000         	movs	r0, #0x0
    12b4: f240 2950    	movw	r9, #0x250
    12b8: f84a 0001    	str.w	r0, [r10, r1]
    12bc: f64f 4100    	movw	r1, #0xfc00
    12c0: f6cf 71ff    	movt	r1, #0xffff
    12c4: f85a 2001    	ldr.w	r2, [r10, r1]
    12c8: f2c2 0900    	movt	r9, #0x2000
    12cc: 2a01         	cmp	r2, #0x1
    12ce: d10a         	bne	0x12e6 <RTC0+0x4e>      @ imm = #0x14
    12d0: f84a 0001    	str.w	r0, [r10, r1]
    12d4: e859 0f12    	ldrex	r0, [r9, #0x48]
    12d8: 1c41         	adds	r1, r0, #0x1
    12da: e849 1212    	strex	r2, r1, [r9, #0x48]
    12de: 2a00         	cmp	r2, #0x0
    12e0: d1f8         	bne	0x12d4 <RTC0+0x3c>      @ imm = #-0x10
    12e2: 07c0         	lsls	r0, r0, #0x1f
    12e4: d013         	beq	0x130e <RTC0+0x76>      @ imm = #0x26
    12e6: f64f 4040    	movw	r0, #0xfc40
    12ea: f6cf 70ff    	movt	r0, #0xffff
    12ee: f85a 1000    	ldr.w	r1, [r10, r0]
    12f2: 2901         	cmp	r1, #0x1
    12f4: d121         	bne	0x133a <RTC0+0xa2>      @ imm = #0x42
    12f6: 2100         	movs	r1, #0x0
    12f8: f84a 1000    	str.w	r1, [r10, r0]
    12fc: e859 0f12    	ldrex	r0, [r9, #0x48]
    1300: 1c41         	adds	r1, r0, #0x1
    1302: e849 1212    	strex	r2, r1, [r9, #0x48]
    1306: 2a00         	cmp	r2, #0x0
    1308: d1f8         	bne	0x12fc <RTC0+0x64>      @ imm = #-0x10
    130a: 07c0         	lsls	r0, r0, #0x1f
    130c: d015         	beq	0x133a <RTC0+0xa2>      @ imm = #0x2a
    130e: 2101         	movs	r1, #0x1
    1310: 2000         	movs	r0, #0x0
    1312: 9104         	str	r1, [sp, #0x10]
    1314: f647 31e8    	movw	r1, #0x7be8
    1318: f2c0 0100    	movt	r1, #0x0
    131c: 9007         	str	r0, [sp, #0x1c]
    131e: 9006         	str	r0, [sp, #0x18]
    1320: 2004         	movs	r0, #0x4
    1322: 9103         	str	r1, [sp, #0xc]
    1324: f647 31ac    	movw	r1, #0x7bac
    1328: 9005         	str	r0, [sp, #0x14]
    132a: a803         	add	r0, sp, #0xc
    132c: f2c0 0100    	movt	r1, #0x0
    1330: f004 fceb    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #0x49d6
    1334: 6869         	ldr	r1, [r5, #0x4]
    1336: 4620         	mov	r0, r4
    1338: 4788         	blx	r1
    133a: f006 f829    	bl	0x7390 <__primask_r>    @ imm = #0x6052
    133e: 4604         	mov	r4, r0
    1340: f006 f81f    	bl	0x7382 <__cpsid>        @ imm = #0x603e
    1344: f240 25c8    	movw	r5, #0x2c8
    1348: f3bf 8f5f    	dmb	sy
    134c: f2c2 0500    	movt	r5, #0x2000
    1350: 07e6         	lsls	r6, r4, #0x1f
    1352: f8d5 8010    	ldr.w	r8, [r5, #0x10]
    1356: f1b8 0f00    	cmp.w	r8, #0x0
    135a: f000 80a5    	beq.w	0x14a8 <RTC0+0x210>     @ imm = #0x14a
    135e: e9d8 1002    	ldrd	r1, r0, [r8, #8]
    1362: e9cd 0101    	strd	r0, r1, [sp, #4]
    1366: f006 f813    	bl	0x7390 <__primask_r>    @ imm = #0x6026
    136a: 4604         	mov	r4, r0
    136c: f006 f809    	bl	0x7382 <__cpsid>        @ imm = #0x6012
    1370: 07e0         	lsls	r0, r4, #0x1f
    1372: f8d9 9048    	ldr.w	r9, [r9, #0x48]
    1376: 46d3         	mov	r11, r10
    1378: f8da a000    	ldr.w	r10, [r10]
    137c: bf08         	it	eq
    137e: f006 f802    	bleq	0x7386 <__cpsie>        @ imm = #0x6004
    1382: f009 0201    	and	r2, r9, #0x1
    1386: e9d8 0102    	ldrd	r0, r1, [r8, #8]
    138a: ea4f 2359    	lsr.w	r3, r9, #0x9
    138e: ea8a 52c2    	eor.w	r2, r10, r2, lsl #23
    1392: eb12 52c9    	adds.w	r2, r2, r9, lsl #23
    1396: f143 0300    	adc	r3, r3, #0x0
    139a: 1a10         	subs	r0, r2, r0
    139c: eb73 0001    	sbcs.w	r0, r3, r1
    13a0: d41b         	bmi	0x13da <RTC0+0x142>     @ imm = #0x36
    13a2: 2001         	movs	r0, #0x1
    13a4: f888 0010    	strb.w	r0, [r8, #0x10]
    13a8: f8d8 0018    	ldr.w	r0, [r8, #0x18]
    13ac: 6128         	str	r0, [r5, #0x10]
    13ae: e9d8 1000    	ldrd	r1, r0, [r8]
    13b2: 6809         	ldr	r1, [r1]
    13b4: 4788         	blx	r1
    13b6: f240 2950    	movw	r9, #0x250
    13ba: 2e00         	cmp	r6, #0x0
    13bc: 4605         	mov	r5, r0
    13be: 460c         	mov	r4, r1
    13c0: f898 0010    	ldrb.w	r0, [r8, #0x10]
    13c4: 46da         	mov	r10, r11
    13c6: f2c2 0900    	movt	r9, #0x2000
    13ca: bf08         	it	eq
    13cc: f005 ffdb    	bleq	0x7386 <__cpsie>        @ imm = #0x5fb6
    13d0: f8dd b004    	ldr.w	r11, [sp, #0x4]
    13d4: 2d00         	cmp	r5, #0x0
    13d6: d1ad         	bne	0x1334 <RTC0+0x9c>      @ imm = #-0xa6
    13d8: e00d         	b	0x13f6 <RTC0+0x15e>     @ imm = #0x1a
    13da: 2000         	movs	r0, #0x0
    13dc: f240 2950    	movw	r9, #0x250
    13e0: 2e00         	cmp	r6, #0x0
    13e2: f888 0010    	strb.w	r0, [r8, #0x10]
    13e6: 46da         	mov	r10, r11
    13e8: f2c2 0900    	movt	r9, #0x2000
    13ec: f8dd b004    	ldr.w	r11, [sp, #0x4]
    13f0: bf08         	it	eq
    13f2: f005 ffc8    	bleq	0x7386 <__cpsie>        @ imm = #0x5f90
    13f6: f005 ffcb    	bl	0x7390 <__primask_r>    @ imm = #0x5f96
    13fa: 4680         	mov	r8, r0
    13fc: f005 ffc1    	bl	0x7382 <__cpsid>        @ imm = #0x5f82
    1400: f005 ffc6    	bl	0x7390 <__primask_r>    @ imm = #0x5f8c
    1404: 4605         	mov	r5, r0
    1406: f005 ffbc    	bl	0x7382 <__cpsid>        @ imm = #0x5f78
    140a: 07e8         	lsls	r0, r5, #0x1f
    140c: f8d9 6048    	ldr.w	r6, [r9, #0x48]
    1410: f8da 4000    	ldr.w	r4, [r10]
    1414: bf08         	it	eq
    1416: f005 ffb6    	bleq	0x7386 <__cpsie>        @ imm = #0x5f6c
    141a: f006 0001    	and	r0, r6, #0x1
    141e: 9d02         	ldr	r5, [sp, #0x8]
    1420: 0a71         	lsrs	r1, r6, #0x9
    1422: 2300         	movs	r3, #0x0
    1424: ea84 50c0    	eor.w	r0, r4, r0, lsl #23
    1428: eb10 50c6    	adds.w	r0, r0, r6, lsl #23
    142c: f141 0100    	adc	r1, r1, #0x0
    1430: 1a2a         	subs	r2, r5, r0
    1432: eb6b 0101    	sbc.w	r1, r11, r1
    1436: f1b2 7680    	subs.w	r6, r2, #0x1000000
    143a: f171 0600    	sbcs	r6, r1, #0x0
    143e: bf38         	it	lo
    1440: 2301         	movlo	r3, #0x1
    1442: 3a03         	subs	r2, #0x3
    1444: f171 0100    	sbcs	r1, r1, #0x0
    1448: 4629         	mov	r1, r5
    144a: bf38         	it	lo
    144c: 1cc1         	addlo	r1, r0, #0x3
    144e: f24b 5040    	movw	r0, #0xb540
    1452: 2b00         	cmp	r3, #0x0
    1454: f2c4 0000    	movt	r0, #0x4000
    1458: bf18         	it	ne
    145a: f021 437f    	bicne	r3, r1, #0xff000000
    145e: 6003         	str	r3, [r0]
    1460: ea5f 70c8    	lsls.w	r0, r8, #0x1f
    1464: bf08         	it	eq
    1466: f005 ff8e    	bleq	0x7386 <__cpsie>        @ imm = #0x5f1c
    146a: f005 ff91    	bl	0x7390 <__primask_r>    @ imm = #0x5f22
    146e: 4604         	mov	r4, r0
    1470: f005 ff87    	bl	0x7382 <__cpsid>        @ imm = #0x5f0e
    1474: 07e0         	lsls	r0, r4, #0x1f
    1476: f8d9 5048    	ldr.w	r5, [r9, #0x48]
    147a: f8da 6000    	ldr.w	r6, [r10]
    147e: bf08         	it	eq
    1480: f005 ff81    	bleq	0x7386 <__cpsie>        @ imm = #0x5f02
    1484: f005 0001    	and	r0, r5, #0x1
    1488: 9a02         	ldr	r2, [sp, #0x8]
    148a: 0a69         	lsrs	r1, r5, #0x9
    148c: ea86 50c0    	eor.w	r0, r6, r0, lsl #23
    1490: eb10 50c5    	adds.w	r0, r0, r5, lsl #23
    1494: f141 0100    	adc	r1, r1, #0x0
    1498: 1a80         	subs	r0, r0, r2
    149a: eb61 000b    	sbc.w	r0, r1, r11
    149e: f1b0 3fff    	cmp.w	r0, #0xffffffff
    14a2: f73f af4a    	bgt.w	0x133a <RTC0+0xa2>      @ imm = #-0x16c
    14a6: e007         	b	0x14b8 <RTC0+0x220>     @ imm = #0xe
    14a8: b936         	cbnz	r6, 0x14b8 <RTC0+0x220> @ imm = #0xc
    14aa: b009         	add	sp, #0x24
    14ac: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    14b0: e8bd 40f0    	pop.w	{r4, r5, r6, r7, lr}
    14b4: f005 bf67    	b.w	0x7386 <__cpsie>        @ imm = #0x5ece
    14b8: b009         	add	sp, #0x24
    14ba: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    14be: bdf0         	pop	{r4, r5, r6, r7, pc}

000014c0 <GPIOTE>:
    14c0: b5f0         	push	{r4, r5, r6, r7, lr}
    14c2: af03         	add	r7, sp, #0xc
    14c4: e92d 0f00    	push.w	{r8, r9, r10, r11}
    14c8: b08d         	sub	sp, #0x34
    14ca: f005 ff69    	bl	0x73a0 <__basepri_r>    @ imm = #0x5ed2
    14ce: 9001         	str	r0, [sp, #0x4]
    14d0: f005 ff5e    	bl	0x7390 <__primask_r>    @ imm = #0x5ebc
    14d4: 4604         	mov	r4, r0
    14d6: f005 ff54    	bl	0x7382 <__cpsid>        @ imm = #0x5ea8
    14da: f240 2650    	movw	r6, #0x250
    14de: f24b 5004    	movw	r0, #0xb504
    14e2: f2c2 0600    	movt	r6, #0x2000
    14e6: f2c4 0000    	movt	r0, #0x4000
    14ea: 6cb5         	ldr	r5, [r6, #0x48]
    14ec: f8d0 8000    	ldr.w	r8, [r0]
    14f0: 07e0         	lsls	r0, r4, #0x1f
    14f2: bf08         	it	eq
    14f4: f005 ff47    	bleq	0x7386 <__cpsie>        @ imm = #0x5e8e
    14f8: 2000         	movs	r0, #0x0
    14fa: 0a69         	lsrs	r1, r5, #0x9
    14fc: 9006         	str	r0, [sp, #0x18]
    14fe: f240 3008    	movw	r0, #0x308
    1502: f2c2 0000    	movt	r0, #0x2000
    1506: f04f 0b02    	mov.w	r11, #0x2
    150a: 9005         	str	r0, [sp, #0x14]
    150c: f005 0001    	and	r0, r5, #0x1
    1510: ea88 50c0    	eor.w	r0, r8, r0, lsl #23
    1514: f240 3808    	movw	r8, #0x308
    1518: eb10 59c5    	adds.w	r9, r0, r5, lsl #23
    151c: f2c2 0800    	movt	r8, #0x2000
    1520: f141 0400    	adc	r4, r1, #0x0
    1524: 2500         	movs	r5, #0x0
    1526: 9403         	str	r4, [sp, #0xc]
    1528: a805         	add	r0, sp, #0x14
    152a: f004 fab7    	bl	0x5a9c <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430> @ imm = #0x456e
    152e: b2c0         	uxtb	r0, r0
    1530: e8df f000    	tbb	[pc, r0]
    1534: 0b 40 23 04  	.word	0x0423400b
    1538: 17 47 4e e0  	.word	0xe04e4717
    153c: f8d8 0000    	ldr.w	r0, [r8]
    1540: f000 0005    	and	r0, r0, #0x5
    1544: f8c8 0000    	str.w	r0, [r8]
    1548: e7ee         	b	0x1528 <GPIOTE+0x68>    @ imm = #-0x24
    154a: f8d8 2000    	ldr.w	r2, [r8]
    154e: 2501         	movs	r5, #0x1
    1550: e9d6 1014    	ldrd	r1, r0, [r6, #80]
    1554: 7833         	ldrb	r3, [r6]
    1556: f042 0201    	orr	r2, r2, #0x1
    155a: e9c6 9414    	strd	r9, r4, [r6, #80]
    155e: 7035         	strb	r5, [r6]
    1560: e016         	b	0x1590 <GPIOTE+0xd0>    @ imm = #0x2c
    1562: f8d8 2000    	ldr.w	r2, [r8]
    1566: 2501         	movs	r5, #0x1
    1568: e9d6 1018    	ldrd	r1, r0, [r6, #96]
    156c: 7c33         	ldrb	r3, [r6, #0x10]
    156e: f042 0204    	orr	r2, r2, #0x4
    1572: e9c6 9418    	strd	r9, r4, [r6, #96]
    1576: 7435         	strb	r5, [r6, #0x10]
    1578: e00a         	b	0x1590 <GPIOTE+0xd0>    @ imm = #0x14
    157a: f8d8 2000    	ldr.w	r2, [r8]
    157e: 2501         	movs	r5, #0x1
    1580: e9d6 1016    	ldrd	r1, r0, [r6, #88]
    1584: 7a33         	ldrb	r3, [r6, #0x8]
    1586: f042 0202    	orr	r2, r2, #0x2
    158a: e9c6 9416    	strd	r9, r4, [r6, #88]
    158e: 7235         	strb	r5, [r6, #0x8]
    1590: 2500         	movs	r5, #0x0
    1592: f04f 0b02    	mov.w	r11, #0x2
    1596: 2b00         	cmp	r3, #0x0
    1598: f8c8 2000    	str.w	r2, [r8]
    159c: d0c4         	beq	0x1528 <GPIOTE+0x68>    @ imm = #-0x78
    159e: ebb9 0101    	subs.w	r1, r9, r1
    15a2: f04f 0b01    	mov.w	r11, #0x1
    15a6: eb64 0a00    	sbc.w	r10, r4, r0
    15aa: 9104         	str	r1, [sp, #0x10]
    15ac: f1ba 3fff    	cmp.w	r10, #0xffffffff
    15b0: dcba         	bgt	0x1528 <GPIOTE+0x68>    @ imm = #-0x8c
    15b2: e0fc         	b	0x17ae <GPIOTE+0x2ee>   @ imm = #0x1f8
    15b4: f8d8 0000    	ldr.w	r0, [r8]
    15b8: f000 0006    	and	r0, r0, #0x6
    15bc: f8c8 0000    	str.w	r0, [r8]
    15c0: e7b2         	b	0x1528 <GPIOTE+0x68>    @ imm = #-0x9c
    15c2: f8d8 0000    	ldr.w	r0, [r8]
    15c6: f000 0003    	and	r0, r0, #0x3
    15ca: f8c8 0000    	str.w	r0, [r8]
    15ce: e7ab         	b	0x1528 <GPIOTE+0x68>    @ imm = #-0xaa
    15d0: f240 0038    	movw	r0, #0x38
    15d4: f2c2 0000    	movt	r0, #0x2000
    15d8: f8d0 00d0    	ldr.w	r0, [r0, #0xd0]
    15dc: 2800         	cmp	r0, #0x0
    15de: d1a3         	bne	0x1528 <GPIOTE+0x68>    @ imm = #-0xba
    15e0: 464e         	mov	r6, r9
    15e2: f005 fed5    	bl	0x7390 <__primask_r>    @ imm = #0x5daa
    15e6: 4604         	mov	r4, r0
    15e8: f005 fecb    	bl	0x7382 <__cpsid>        @ imm = #0x5d96
    15ec: f240 0038    	movw	r0, #0x38
    15f0: f2c2 0000    	movt	r0, #0x2000
    15f4: f890 9104    	ldrb.w	r9, [r0, #0x104]
    15f8: 07e0         	lsls	r0, r4, #0x1f
    15fa: bf08         	it	eq
    15fc: f005 fec3    	bleq	0x7386 <__cpsie>        @ imm = #0x5d86
    1600: 9c03         	ldr	r4, [sp, #0xc]
    1602: f1b9 0f00    	cmp.w	r9, #0x0
    1606: 46b1         	mov	r9, r6
    1608: f240 2650    	movw	r6, #0x250
    160c: f2c2 0600    	movt	r6, #0x2000
    1610: f47f af8a    	bne.w	0x1528 <GPIOTE+0x68>    @ imm = #-0xec
    1614: f005 febc    	bl	0x7390 <__primask_r>    @ imm = #0x5d78
    1618: 4604         	mov	r4, r0
    161a: f005 feb2    	bl	0x7382 <__cpsid>        @ imm = #0x5d64
    161e: f240 0338    	movw	r3, #0x38
    1622: f2c2 0300    	movt	r3, #0x2000
    1626: e9d3 0136    	ldrd	r0, r1, [r3, #216]
    162a: 4288         	cmp	r0, r1
    162c: bf04         	itt	eq
    162e: f893 10ea    	ldrbeq.w	r1, [r3, #0xea]
    1632: ea5f 71c1    	lslseq.w	r1, r1, #0x1f
    1636: d056         	beq	0x16e6 <GPIOTE+0x226>   @ imm = #0xac
    1638: 2100         	movs	r1, #0x0
    163a: f883 10ea    	strb.w	r1, [r3, #0xea]
    163e: 1c41         	adds	r1, r0, #0x1
    1640: f1b1 020a    	subs.w	r2, r1, #0xa
    1644: 4418         	add	r0, r3
    1646: bf18         	it	ne
    1648: 460a         	movne	r2, r1
    164a: f8c3 20d8    	str.w	r2, [r3, #0xd8]
    164e: f890 10e0    	ldrb.w	r1, [r0, #0xe0]
    1652: 07e0         	lsls	r0, r4, #0x1f
    1654: d107         	bne	0x1666 <GPIOTE+0x1a6>   @ imm = #0xe
    1656: 460c         	mov	r4, r1
    1658: f005 fe95    	bl	0x7386 <__cpsie>        @ imm = #0x5d2a
    165c: f240 0338    	movw	r3, #0x38
    1660: 4621         	mov	r1, r4
    1662: f2c2 0300    	movt	r3, #0x2000
    1666: eb01 0081    	add.w	r0, r1, r1, lsl #2
    166a: 460c         	mov	r4, r1
    166c: 2102         	movs	r1, #0x2
    166e: 2212         	movs	r2, #0x12
    1670: eb03 0080    	add.w	r0, r3, r0, lsl #2
    1674: 8101         	strh	r1, [r0, #0x8]
    1676: 300a         	adds	r0, #0xa
    1678: a907         	add	r1, sp, #0x1c
    167a: f005 fecd    	bl	0x7418 <__aeabi_memcpy> @ imm = #0x5d9a
    167e: f005 fe87    	bl	0x7390 <__primask_r>    @ imm = #0x5d0e
    1682: 9002         	str	r0, [sp, #0x8]
    1684: f005 fe7d    	bl	0x7382 <__cpsid>        @ imm = #0x5cfa
    1688: f240 0338    	movw	r3, #0x38
    168c: f2c2 0300    	movt	r3, #0x2000
    1690: f8d3 00f0    	ldr.w	r0, [r3, #0xf0]
    1694: 4418         	add	r0, r3
    1696: f880 40f4    	strb.w	r4, [r0, #0xf4]
    169a: e9d3 013b    	ldrd	r0, r1, [r3, #236]
    169e: 3101         	adds	r1, #0x1
    16a0: f1b1 020a    	subs.w	r2, r1, #0xa
    16a4: bf18         	it	ne
    16a6: 460a         	movne	r2, r1
    16a8: f8c3 20f0    	str.w	r2, [r3, #0xf0]
    16ac: 4290         	cmp	r0, r2
    16ae: bf04         	itt	eq
    16b0: 2001         	moveq	r0, #0x1
    16b2: f883 00fe    	strbeq.w	r0, [r3, #0xfe]
    16b6: 9802         	ldr	r0, [sp, #0x8]
    16b8: 07c0         	lsls	r0, r0, #0x1f
    16ba: bf08         	it	eq
    16bc: f005 fe63    	bleq	0x7386 <__cpsie>        @ imm = #0x5cc6
    16c0: f3bf 8f5f    	dmb	sy
    16c4: f005 fe64    	bl	0x7390 <__primask_r>    @ imm = #0x5cc8
    16c8: 4604         	mov	r4, r0
    16ca: f005 fe5a    	bl	0x7382 <__cpsid>        @ imm = #0x5cb4
    16ce: f240 0338    	movw	r3, #0x38
    16d2: 2200         	movs	r2, #0x0
    16d4: f2c2 0300    	movt	r3, #0x2000
    16d8: e9d3 1000    	ldrd	r1, r0, [r3]
    16dc: 601a         	str	r2, [r3]
    16de: 2900         	cmp	r1, #0x0
    16e0: bf1c         	itt	ne
    16e2: 6849         	ldrne	r1, [r1, #0x4]
    16e4: 4788         	blxne	r1
    16e6: 07e0         	lsls	r0, r4, #0x1f
    16e8: 9c03         	ldr	r4, [sp, #0xc]
    16ea: f47f af1d    	bne.w	0x1528 <GPIOTE+0x68>    @ imm = #-0x1c6
    16ee: f005 fe4a    	bl	0x7386 <__cpsie>        @ imm = #0x5c94
    16f2: e719         	b	0x1528 <GPIOTE+0x68>    @ imm = #-0x1ce
    16f4: f005 fe54    	bl	0x73a0 <__basepri_r>    @ imm = #0x5ca8
    16f8: 4604         	mov	r4, r0
    16fa: 2060         	movs	r0, #0x60
    16fc: f005 fe4d    	bl	0x739a <__basepri_max>  @ imm = #0x5c9a
    1700: f8d8 0000    	ldr.w	r0, [r8]
    1704: 2808         	cmp	r0, #0x8
    1706: d264         	bhs	0x17d2 <GPIOTE+0x312>   @ imm = #0xc8
    1708: f247 7150    	movw	r1, #0x7750
    170c: f247 7248    	movw	r2, #0x7748
    1710: f2c0 0100    	movt	r1, #0x0
    1714: f2c0 0200    	movt	r2, #0x0
    1718: 5c09         	ldrb	r1, [r1, r0]
    171a: 5c10         	ldrb	r0, [r2, r0]
    171c: f240 7238    	movw	r2, #0x738
    1720: f2c2 0200    	movt	r2, #0x2000
    1724: 7050         	strb	r0, [r2, #0x1]
    1726: 4620         	mov	r0, r4
    1728: 7011         	strb	r1, [r2]
    172a: f005 fe3d    	bl	0x73a8 <__basepri_w>    @ imm = #0x5c7a
    172e: f08b 0002    	eor	r0, r11, #0x2
    1732: 4328         	orrs	r0, r5
    1734: 9d03         	ldr	r5, [sp, #0xc]
    1736: d033         	beq	0x17a0 <GPIOTE+0x2e0>   @ imm = #0x66
    1738: 6eb0         	ldr	r0, [r6, #0x68]
    173a: 2301         	movs	r3, #0x1
    173c: 7e32         	ldrb	r2, [r6, #0x18]
    173e: 6ef1         	ldr	r1, [r6, #0x6c]
    1740: 2a01         	cmp	r2, #0x1
    1742: f8c6 9068    	str.w	r9, [r6, #0x68]
    1746: 66f5         	str	r5, [r6, #0x6c]
    1748: 7633         	strb	r3, [r6, #0x18]
    174a: d129         	bne	0x17a0 <GPIOTE+0x2e0>   @ imm = #0x52
    174c: ebb9 0600    	subs.w	r6, r9, r0
    1750: 418d         	sbcs	r5, r1
    1752: f1b5 3fff    	cmp.w	r5, #0xffffffff
    1756: dd43         	ble	0x17e0 <GPIOTE+0x320>   @ imm = #0x86
    1758: f005 fe22    	bl	0x73a0 <__basepri_r>    @ imm = #0x5c44
    175c: 4604         	mov	r4, r0
    175e: 2060         	movs	r0, #0x60
    1760: f005 fe1b    	bl	0x739a <__basepri_max>  @ imm = #0x5c36
    1764: 9b04         	ldr	r3, [sp, #0x10]
    1766: f643 5009    	movw	r0, #0x3d09
    176a: fba6 1200    	umull	r1, r2, r6, r0
    176e: fba3 3600    	umull	r3, r6, r3, r0
    1772: fb05 2200    	mla	r2, r5, r0, r2
    1776: 0a49         	lsrs	r1, r1, #0x9
    1778: fb0a 6000    	mla	r0, r10, r0, r6
    177c: f240 26e8    	movw	r6, #0x2e8
    1780: 0a5b         	lsrs	r3, r3, #0x9
    1782: f2c2 0600    	movt	r6, #0x2000
    1786: ea41 51c2    	orr.w	r1, r1, r2, lsl #23
    178a: 0a52         	lsrs	r2, r2, #0x9
    178c: e9c6 1202    	strd	r1, r2, [r6, #8]
    1790: ea43 53c0    	orr.w	r3, r3, r0, lsl #23
    1794: 0a40         	lsrs	r0, r0, #0x9
    1796: e9c6 3000    	strd	r3, r0, [r6]
    179a: 4620         	mov	r0, r4
    179c: f005 fe04    	bl	0x73a8 <__basepri_w>    @ imm = #0x5c08
    17a0: 9801         	ldr	r0, [sp, #0x4]
    17a2: f005 fe01    	bl	0x73a8 <__basepri_w>    @ imm = #0x5c02
    17a6: b00d         	add	sp, #0x34
    17a8: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    17ac: bdf0         	pop	{r4, r5, r6, r7, pc}
    17ae: f247 60dc    	movw	r0, #0x76dc
    17b2: 950b         	str	r5, [sp, #0x2c]
    17b4: f2c0 0000    	movt	r0, #0x0
    17b8: f8cd b020    	str.w	r11, [sp, #0x20]
    17bc: 9007         	str	r0, [sp, #0x1c]
    17be: 950a         	str	r5, [sp, #0x28]
    17c0: 2004         	movs	r0, #0x4
    17c2: f647 31ac    	movw	r1, #0x7bac
    17c6: 9009         	str	r0, [sp, #0x24]
    17c8: a807         	add	r0, sp, #0x1c
    17ca: f2c0 0100    	movt	r1, #0x0
    17ce: f004 fa9c    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #0x4538
    17d2: f647 32ac    	movw	r2, #0x7bac
    17d6: 2108         	movs	r1, #0x8
    17d8: f2c0 0200    	movt	r2, #0x0
    17dc: f004 fa00    	bl	0x5be0 <core::panicking::panic_bounds_check::h69088854c118b8d4> @ imm = #0x4400
    17e0: 2101         	movs	r1, #0x1
    17e2: 2000         	movs	r0, #0x0
    17e4: 9108         	str	r1, [sp, #0x20]
    17e6: f247 61dc    	movw	r1, #0x76dc
    17ea: f2c0 0100    	movt	r1, #0x0
    17ee: 900b         	str	r0, [sp, #0x2c]
    17f0: 9107         	str	r1, [sp, #0x1c]
    17f2: 900a         	str	r0, [sp, #0x28]
    17f4: e7e4         	b	0x17c0 <GPIOTE+0x300>   @ imm = #-0x38
    17f6: d4d4         	bmi	0x17a2 <GPIOTE+0x2e2>   @ imm = #-0x58

000017f8 <TIMER3>:
    17f8: b5f0         	push	{r4, r5, r6, r7, lr}
    17fa: af03         	add	r7, sp, #0xc
    17fc: e92d 0f00    	push.w	{r8, r9, r10, r11}
    1800: b081         	sub	sp, #0x4
    1802: ed2d 8b0e    	vpush	{d8, d9, d10, d11, d12, d13, d14}
    1806: b088         	sub	sp, #0x20
    1808: f005 fdca    	bl	0x73a0 <__basepri_r>    @ imm = #0x5b94
    180c: 9001         	str	r0, [sp, #0x4]
    180e: f005 fdbf    	bl	0x7390 <__primask_r>    @ imm = #0x5b7e
    1812: 4605         	mov	r5, r0
    1814: f005 fdb5    	bl	0x7382 <__cpsid>        @ imm = #0x5b6a
    1818: f240 2050    	movw	r0, #0x250
    181c: f2c2 0000    	movt	r0, #0x2000
    1820: f8d0 b048    	ldr.w	r11, [r0, #0x48]
    1824: f24b 5004    	movw	r0, #0xb504
    1828: f2c4 0000    	movt	r0, #0x4000
    182c: f8d0 8000    	ldr.w	r8, [r0]
    1830: 07e8         	lsls	r0, r5, #0x1f
    1832: bf08         	it	eq
    1834: f005 fda7    	bleq	0x7386 <__cpsie>        @ imm = #0x5b4e
    1838: f24a 1140    	movw	r1, #0xa140
    183c: 2000         	movs	r0, #0x0
    183e: f2c4 0101    	movt	r1, #0x4001
    1842: 6008         	str	r0, [r1]
    1844: f005 fdac    	bl	0x73a0 <__basepri_r>    @ imm = #0x5b58
    1848: 4605         	mov	r5, r0
    184a: 2080         	movs	r0, #0x80
    184c: f005 fda5    	bl	0x739a <__basepri_max>  @ imm = #0x5b4a
    1850: f240 20fc    	movw	r0, #0x2fc
    1854: f2c2 0000    	movt	r0, #0x2000
    1858: ed90 8a00    	vldr	s16, [r0]
    185c: 4628         	mov	r0, r5
    185e: f005 fda3    	bl	0x73a8 <__basepri_w>    @ imm = #0x5b46
    1862: f005 fd9d    	bl	0x73a0 <__basepri_r>    @ imm = #0x5b3a
    1866: 4604         	mov	r4, r0
    1868: 2060         	movs	r0, #0x60
    186a: f005 fd96    	bl	0x739a <__basepri_max>  @ imm = #0x5b2c
    186e: f240 20e8    	movw	r0, #0x2e8
    1872: f2c2 0000    	movt	r0, #0x2000
    1876: e890 0660    	ldm.w	r0, {r5, r6, r9, r10}
    187a: 4620         	mov	r0, r4
    187c: f005 fd94    	bl	0x73a8 <__basepri_w>    @ imm = #0x5b28
    1880: 4628         	mov	r0, r5
    1882: 4631         	mov	r1, r6
    1884: f005 fd93    	bl	0x73ae <__aeabi_ul2f>   @ imm = #0x5b26
    1888: ee00 0a10    	vmov	s0, r0
    188c: ed9f 9ae2    	vldr	s18, [pc, #904]         @ 0x1c18 <TIMER3+0x420>
    1890: ed9f 1ae2    	vldr	s2, [pc, #904]          @ 0x1c1c <TIMER3+0x424>
    1894: f00b 0001    	and	r0, r11, #0x1
    1898: ee80 0a09    	vdiv.f32	s0, s0, s18
    189c: f240 2550    	movw	r5, #0x250
    18a0: ea88 50c0    	eor.w	r0, r8, r0, lsl #23
    18a4: ea4f 215b    	lsr.w	r1, r11, #0x9
    18a8: eb10 56cb    	adds.w	r6, r0, r11, lsl #23
    18ac: f2c2 0500    	movt	r5, #0x2000
    18b0: f141 0800    	adc	r8, r1, #0x0
    18b4: ee81 ba00    	vdiv.f32	s22, s2, s0
    18b8: ed9f aad9    	vldr	s20, [pc, #868]         @ 0x1c20 <TIMER3+0x428>
    18bc: eeb4 ba4a    	vcmp.f32	s22, s20
    18c0: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    18c4: dd0a         	ble	0x18dc <TIMER3+0xe4>    @ imm = #0x14
    18c6: 6b68         	ldr	r0, [r5, #0x34]
    18c8: 280a         	cmp	r0, #0xa
    18ca: d207         	bhs	0x18dc <TIMER3+0xe4>    @ imm = #0xe
    18cc: 3001         	adds	r0, #0x1
    18ce: eeb4 ba4a    	vcmp.f32	s22, s20
    18d2: 6368         	str	r0, [r5, #0x34]
    18d4: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    18d8: d50f         	bpl	0x18fa <TIMER3+0x102>   @ imm = #0x1e
    18da: e050         	b	0x197e <TIMER3+0x186>   @ imm = #0xa0
    18dc: eeb4 ba4a    	vcmp.f32	s22, s20
    18e0: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    18e4: d849         	bhi	0x197a <TIMER3+0x182>   @ imm = #0x92
    18e6: 6b68         	ldr	r0, [r5, #0x34]
    18e8: 2803         	cmp	r0, #0x3
    18ea: d346         	blo	0x197a <TIMER3+0x182>   @ imm = #0x8c
    18ec: 3801         	subs	r0, #0x1
    18ee: eeb4 ba4a    	vcmp.f32	s22, s20
    18f2: 6368         	str	r0, [r5, #0x34]
    18f4: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    18f8: d441         	bmi	0x197e <TIMER3+0x186>   @ imm = #0x82
    18fa: 2803         	cmp	r0, #0x3
    18fc: d33f         	blo	0x197e <TIMER3+0x186>   @ imm = #0x7e
    18fe: 4648         	mov	r0, r9
    1900: 4651         	mov	r1, r10
    1902: f005 fd54    	bl	0x73ae <__aeabi_ul2f>   @ imm = #0x5aa8
    1906: ee00 0a10    	vmov	s0, r0
    190a: ed95 2a09    	vldr	s4, [r5, #36]
    190e: ee80 1a09    	vdiv.f32	s2, s0, s18
    1912: ee3b 2a42    	vsub.f32	s4, s22, s4
    1916: ed9f 9ac3    	vldr	s18, [pc, #780]         @ 0x1c24 <TIMER3+0x42c>
    191a: ed95 0a0a    	vldr	s0, [r5, #40]
    191e: ee82 1a01    	vdiv.f32	s2, s4, s2
    1922: ee11 0a10    	vmov	r0, s2
    1926: eeb9 2a04    	vmov.f32	s4, #-5.000000e+00
    192a: f020 4000    	bic	r0, r0, #0x80000000
    192e: f1b0 4fff    	cmp.w	r0, #0x7f800000
    1932: bfb8         	it	lt
    1934: eeb0 9a41    	vmovlt.f32	s18, s2
    1938: eeb1 1a04    	vmov.f32	s2, #5.000000e+00
    193c: eeb4 9a42    	vcmp.f32	s18, s4
    1940: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1944: bf48         	it	mi
    1946: eeb0 9a42    	vmovmi.f32	s18, s4
    194a: eeb4 9a41    	vcmp.f32	s18, s2
    194e: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1952: bfc8         	it	gt
    1954: eeb0 9a41    	vmovgt.f32	s18, s2
    1958: ee39 1a40    	vsub.f32	s2, s18, s0
    195c: ed85 ba09    	vstr	s22, [r5, #36]
    1960: eeb4 1a4a    	vcmp.f32	s2, s20
    1964: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1968: dd57         	ble	0x1a1a <TIMER3+0x222>   @ imm = #0xae
    196a: eeb4 1a4a    	vcmp.f32	s2, s20
    196e: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1972: bf58         	it	pl
    1974: ee30 9a0a    	vaddpl.f32	s18, s0, s20
    1978: e058         	b	0x1a2c <TIMER3+0x234>   @ imm = #0xb0
    197a: 2000         	movs	r0, #0x0
    197c: 6368         	str	r0, [r5, #0x34]
    197e: ed85 ba09    	vstr	s22, [r5, #36]
    1982: f005 fd0d    	bl	0x73a0 <__basepri_r>    @ imm = #0x5a1a
    1986: 4604         	mov	r4, r0
    1988: 2080         	movs	r0, #0x80
    198a: f005 fd06    	bl	0x739a <__basepri_max>  @ imm = #0x5a0c
    198e: f240 20e0    	movw	r0, #0x2e0
    1992: f04f 0900    	mov.w	r9, #0x0
    1996: f2c2 0000    	movt	r0, #0x2000
    199a: f8c0 9000    	str.w	r9, [r0]
    199e: 4620         	mov	r0, r4
    19a0: f005 fd02    	bl	0x73a8 <__basepri_w>    @ imm = #0x5a04
    19a4: f005 fcf4    	bl	0x7390 <__primask_r>    @ imm = #0x59e8
    19a8: 4604         	mov	r4, r0
    19aa: f005 fcea    	bl	0x7382 <__cpsid>        @ imm = #0x59d4
    19ae: f24b 5004    	movw	r0, #0xb504
    19b2: f116 0a20    	adds.w	r10, r6, #0x20
    19b6: f2c4 0000    	movt	r0, #0x4000
    19ba: 6cad         	ldr	r5, [r5, #0x48]
    19bc: 6806         	ldr	r6, [r0]
    19be: f148 0800    	adc	r8, r8, #0x0
    19c2: 07e0         	lsls	r0, r4, #0x1f
    19c4: bf08         	it	eq
    19c6: f005 fcde    	bleq	0x7386 <__cpsie>        @ imm = #0x59bc
    19ca: f005 0001    	and	r0, r5, #0x1
    19ce: 0a69         	lsrs	r1, r5, #0x9
    19d0: f643 5209    	movw	r2, #0x3d09
    19d4: ea86 50c0    	eor.w	r0, r6, r0, lsl #23
    19d8: eb10 50c5    	adds.w	r0, r0, r5, lsl #23
    19dc: f141 0100    	adc	r1, r1, #0x0
    19e0: ebba 0000    	subs.w	r0, r10, r0
    19e4: eb68 0101    	sbc.w	r1, r8, r1
    19e8: fba0 5302    	umull	r5, r3, r0, r2
    19ec: fba1 1202    	umull	r1, r2, r1, r2
    19f0: 18c9         	adds	r1, r1, r3
    19f2: f149 0300    	adc	r3, r9, #0x0
    19f6: 2a00         	cmp	r2, #0x0
    19f8: bf18         	it	ne
    19fa: 2201         	movne	r2, #0x1
    19fc: 431a         	orrs	r2, r3
    19fe: 2a01         	cmp	r2, #0x1
    1a00: f040 80d0    	bne.w	0x1ba4 <TIMER3+0x3ac>   @ imm = #0x1a0
    1a04: 2101         	movs	r1, #0x1
    1a06: 2000         	movs	r0, #0x0
    1a08: 9103         	str	r1, [sp, #0xc]
    1a0a: f247 6190    	movw	r1, #0x7690
    1a0e: f2c0 0100    	movt	r1, #0x0
    1a12: 9006         	str	r0, [sp, #0x18]
    1a14: 9102         	str	r1, [sp, #0x8]
    1a16: 9005         	str	r0, [sp, #0x14]
    1a18: e0f4         	b	0x1c04 <TIMER3+0x40c>   @ imm = #0x1e8
    1a1a: eeb4 1a4a    	vcmp.f32	s2, s20
    1a1e: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1a22: bf44         	itt	mi
    1a24: ed9f 1a80    	vldrmi	s2, [pc, #512]          @ 0x1c28 <TIMER3+0x430>
    1a28: ee30 9a01    	vaddmi.f32	s18, s0, s2
    1a2c: ed95 aa0b    	vldr	s20, [r5, #44]
    1a30: eeb6 0a00    	vmov.f32	s0, #5.000000e-01
    1a34: ee38 8a49    	vsub.f32	s16, s16, s18
    1a38: ed9f ca7c    	vldr	s24, [pc, #496]         @ 0x1c2c <TIMER3+0x434>
    1a3c: eeb0 1a4a    	vmov.f32	s2, s20
    1a40: ed95 ba0c    	vldr	s22, [r5, #48]
    1a44: ed9f da7a    	vldr	s26, [pc, #488]         @ 0x1c30 <TIMER3+0x438>
    1a48: ed9f ea7a    	vldr	s28, [pc, #488]         @ 0x1c34 <TIMER3+0x43c>
    1a4c: ee08 1a00    	vmla.f32	s2, s16, s0
    1a50: ee01 ba0c    	vmla.f32	s22, s2, s24
    1a54: eeb4 ba4d    	vcmp.f32	s22, s26
    1a58: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1a5c: bf48         	it	mi
    1a5e: eeb0 ba4d    	vmovmi.f32	s22, s26
    1a62: eeb4 ba4e    	vcmp.f32	s22, s28
    1a66: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1a6a: bfc8         	it	gt
    1a6c: eeb0 ba4e    	vmovgt.f32	s22, s28
    1a70: ed85 9a0a    	vstr	s18, [r5, #40]
    1a74: ed85 8a0b    	vstr	s16, [r5, #44]
    1a78: ed85 ba0c    	vstr	s22, [r5, #48]
    1a7c: f005 fc90    	bl	0x73a0 <__basepri_r>    @ imm = #0x5920
    1a80: 4604         	mov	r4, r0
    1a82: 2080         	movs	r0, #0x80
    1a84: f005 fc89    	bl	0x739a <__basepri_max>  @ imm = #0x5912
    1a88: ee38 0a4a    	vsub.f32	s0, s16, s20
    1a8c: f240 20e0    	movw	r0, #0x2e0
    1a90: ee28 1a0e    	vmul.f32	s2, s16, s28
    1a94: f2c2 0000    	movt	r0, #0x2000
    1a98: eeb7 2a00    	vmov.f32	s4, #1.000000e+00
    1a9c: ee80 0a0c    	vdiv.f32	s0, s0, s24
    1aa0: eeb4 1a4d    	vcmp.f32	s2, s26
    1aa4: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1aa8: bf48         	it	mi
    1aaa: eeb0 1a4d    	vmovmi.f32	s2, s26
    1aae: eeb4 1a4e    	vcmp.f32	s2, s28
    1ab2: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1ab6: bfc8         	it	gt
    1ab8: eeb0 1a4e    	vmovgt.f32	s2, s28
    1abc: ee31 1a0b    	vadd.f32	s2, s2, s22
    1ac0: eeb4 0a4d    	vcmp.f32	s0, s26
    1ac4: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1ac8: bf48         	it	mi
    1aca: eeb0 0a4d    	vmovmi.f32	s0, s26
    1ace: eeb4 0a4e    	vcmp.f32	s0, s28
    1ad2: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1ad6: bfc8         	it	gt
    1ad8: eeb0 0a4e    	vmovgt.f32	s0, s28
    1adc: ee30 0a01    	vadd.f32	s0, s0, s2
    1ae0: ed9f 1a55    	vldr	s2, [pc, #340]          @ 0x1c38 <TIMER3+0x440>
    1ae4: ee30 0a01    	vadd.f32	s0, s0, s2
    1ae8: ed9f 1a54    	vldr	s2, [pc, #336]          @ 0x1c3c <TIMER3+0x444>
    1aec: ee20 0a01    	vmul.f32	s0, s0, s2
    1af0: ed9f 1a53    	vldr	s2, [pc, #332]          @ 0x1c40 <TIMER3+0x448>
    1af4: ee80 0a01    	vdiv.f32	s0, s0, s2
    1af8: eebf 1a00    	vmov.f32	s2, #-1.000000e+00
    1afc: ee30 0a01    	vadd.f32	s0, s0, s2
    1b00: eeb4 0a41    	vcmp.f32	s0, s2
    1b04: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1b08: bf48         	it	mi
    1b0a: eeb0 0a41    	vmovmi.f32	s0, s2
    1b0e: eeb4 0a42    	vcmp.f32	s0, s4
    1b12: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    1b16: bfc8         	it	gt
    1b18: eeb0 0a42    	vmovgt.f32	s0, s4
    1b1c: ed80 0a00    	vstr	s0, [r0]
    1b20: 4620         	mov	r0, r4
    1b22: f005 fc41    	bl	0x73a8 <__basepri_w>    @ imm = #0x5882
    1b26: f005 fc3b    	bl	0x73a0 <__basepri_r>    @ imm = #0x5876
    1b2a: 4604         	mov	r4, r0
    1b2c: 2080         	movs	r0, #0x80
    1b2e: f005 fc34    	bl	0x739a <__basepri_max>  @ imm = #0x5868
    1b32: f240 20f8    	movw	r0, #0x2f8
    1b36: f2c2 0000    	movt	r0, #0x2000
    1b3a: ed80 9a00    	vstr	s18, [r0]
    1b3e: 4620         	mov	r0, r4
    1b40: f005 fc32    	bl	0x73a8 <__basepri_w>    @ imm = #0x5864
    1b44: f005 fc24    	bl	0x7390 <__primask_r>    @ imm = #0x5848
    1b48: 4604         	mov	r4, r0
    1b4a: f005 fc1a    	bl	0x7382 <__cpsid>        @ imm = #0x5834
    1b4e: f24b 5004    	movw	r0, #0xb504
    1b52: f116 0920    	adds.w	r9, r6, #0x20
    1b56: f2c4 0000    	movt	r0, #0x4000
    1b5a: 6cad         	ldr	r5, [r5, #0x48]
    1b5c: 6806         	ldr	r6, [r0]
    1b5e: f148 0800    	adc	r8, r8, #0x0
    1b62: 07e0         	lsls	r0, r4, #0x1f
    1b64: bf08         	it	eq
    1b66: f005 fc0e    	bleq	0x7386 <__cpsie>        @ imm = #0x581c
    1b6a: f005 0001    	and	r0, r5, #0x1
    1b6e: 0a69         	lsrs	r1, r5, #0x9
    1b70: f643 5209    	movw	r2, #0x3d09
    1b74: ea86 50c0    	eor.w	r0, r6, r0, lsl #23
    1b78: eb10 50c5    	adds.w	r0, r0, r5, lsl #23
    1b7c: f141 0100    	adc	r1, r1, #0x0
    1b80: ebb9 0000    	subs.w	r0, r9, r0
    1b84: eb68 0101    	sbc.w	r1, r8, r1
    1b88: fba0 5302    	umull	r5, r3, r0, r2
    1b8c: fba1 1602    	umull	r1, r6, r1, r2
    1b90: 2200         	movs	r2, #0x0
    1b92: 18c9         	adds	r1, r1, r3
    1b94: f142 0300    	adc	r3, r2, #0x0
    1b98: 2e00         	cmp	r6, #0x0
    1b9a: bf18         	it	ne
    1b9c: 2601         	movne	r6, #0x1
    1b9e: 4333         	orrs	r3, r6
    1ba0: 2b01         	cmp	r3, #0x1
    1ba2: d026         	beq	0x1bf2 <TIMER3+0x3fa>   @ imm = #0x4c
    1ba4: f24a 1340    	movw	r3, #0xa140
    1ba8: 2200         	movs	r2, #0x0
    1baa: f2c4 0301    	movt	r3, #0x4001
    1bae: 9801         	ldr	r0, [sp, #0x4]
    1bb0: 601a         	str	r2, [r3]
    1bb2: 0a6a         	lsrs	r2, r5, #0x9
    1bb4: ea42 51c1    	orr.w	r1, r2, r1, lsl #23
    1bb8: f64f 62cc    	movw	r2, #0xfecc
    1bbc: f8c3 1400    	str.w	r1, [r3, #0x400]
    1bc0: f6cf 72ff    	movt	r2, #0xffff
    1bc4: 2101         	movs	r1, #0x1
    1bc6: 5099         	str	r1, [r3, r2]
    1bc8: f64f 62c0    	movw	r2, #0xfec0
    1bcc: f6cf 72ff    	movt	r2, #0xffff
    1bd0: 5099         	str	r1, [r3, r2]
    1bd2: f8d3 11c4    	ldr.w	r1, [r3, #0x1c4]
    1bd6: f441 3180    	orr	r1, r1, #0x10000
    1bda: f8c3 11c4    	str.w	r1, [r3, #0x1c4]
    1bde: b008         	add	sp, #0x20
    1be0: ecbd 8b0e    	vpop	{d8, d9, d10, d11, d12, d13, d14}
    1be4: b001         	add	sp, #0x4
    1be6: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    1bea: e8bd 40f0    	pop.w	{r4, r5, r6, r7, lr}
    1bee: f005 bbdb    	b.w	0x73a8 <__basepri_w>    @ imm = #0x57b6
    1bf2: 2001         	movs	r0, #0x1
    1bf4: 9206         	str	r2, [sp, #0x18]
    1bf6: 9003         	str	r0, [sp, #0xc]
    1bf8: f247 6090    	movw	r0, #0x7690
    1bfc: f2c0 0000    	movt	r0, #0x0
    1c00: 9205         	str	r2, [sp, #0x14]
    1c02: 9002         	str	r0, [sp, #0x8]
    1c04: 2004         	movs	r0, #0x4
    1c06: f647 31ac    	movw	r1, #0x7bac
    1c0a: 9004         	str	r0, [sp, #0x10]
    1c0c: a802         	add	r0, sp, #0x8
    1c0e: f2c0 0100    	movt	r1, #0x0
    1c12: f004 f87a    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #0x40f4
    1c16: bf00         	nop
    1c18: 00 24 74 49  	.word	0x49742400
    1c1c: a3 a0 95 3d  	.word	0x3d95a0a3
    1c20: cd cc cc 3d  	.word	0x3dcccccd
    1c24: 00 00 00 00  	.word	0x00000000
    1c28: cd cc cc bd  	.word	0xbdcccccd
    1c2c: 6f 12 83 3a  	.word	0x3a83126f
    1c30: 00 00 c8 c2  	.word	0xc2c80000
    1c34: 00 00 c8 42  	.word	0x42c80000
    1c38: 00 00 96 43  	.word	0x43960000
    1c3c: 9a 99 f9 3f  	.word	0x3ff9999a
    1c40: 00 00 16 44  	.word	0x44160000

00001c44 <PWM0>:
    1c44: b5b0         	push	{r4, r5, r7, lr}
    1c46: af02         	add	r7, sp, #0x8
    1c48: f005 fbaa    	bl	0x73a0 <__basepri_r>    @ imm = #0x5754
    1c4c: 4604         	mov	r4, r0
    1c4e: f005 fba7    	bl	0x73a0 <__basepri_r>    @ imm = #0x574e
    1c52: 4605         	mov	r5, r0
    1c54: 20a0         	movs	r0, #0xa0
    1c56: f005 fba0    	bl	0x739a <__basepri_max>  @ imm = #0x5740
    1c5a: f247 0000    	movw	r0, #0x7000
    1c5e: 2100         	movs	r1, #0x0
    1c60: f2c4 0000    	movt	r0, #0x4000
    1c64: f240 3200    	movw	r2, #0x300
    1c68: f8c0 1104    	str.w	r1, [r0, #0x104]
    1c6c: f247 612c    	movw	r1, #0x762c
    1c70: f2c4 0100    	movt	r1, #0x4000
    1c74: f2c2 0200    	movt	r2, #0x2000
    1c78: 600a         	str	r2, [r1]
    1c7a: 2204         	movs	r2, #0x4
    1c7c: 604a         	str	r2, [r1, #0x4]
    1c7e: 2101         	movs	r1, #0x1
    1c80: f8c0 1500    	str.w	r1, [r0, #0x500]
    1c84: 6001         	str	r1, [r0]
    1c86: 6041         	str	r1, [r0, #0x4]
    1c88: 2102         	movs	r1, #0x2
    1c8a: f8c0 1300    	str.w	r1, [r0, #0x300]
    1c8e: f8c0 1304    	str.w	r1, [r0, #0x304]
    1c92: 4628         	mov	r0, r5
    1c94: f005 fb88    	bl	0x73a8 <__basepri_w>    @ imm = #0x5710
    1c98: f24e 1080    	movw	r0, #0xe180
    1c9c: f04f 5180    	mov.w	r1, #0x10000000
    1ca0: f2ce 0000    	movt	r0, #0xe000
    1ca4: 6001         	str	r1, [r0]
    1ca6: 4620         	mov	r0, r4
    1ca8: e8bd 40b0    	pop.w	{r4, r5, r7, lr}
    1cac: f005 bb7c    	b.w	0x73a8 <__basepri_w>    @ imm = #0x56f8

00001cb0 <SAADC>:
    1cb0: b5b0         	push	{r4, r5, r7, lr}
    1cb2: af02         	add	r7, sp, #0x8
    1cb4: f005 fb74    	bl	0x73a0 <__basepri_r>    @ imm = #0x56e8
    1cb8: 4604         	mov	r4, r0
    1cba: f005 fb71    	bl	0x73a0 <__basepri_r>    @ imm = #0x56e2
    1cbe: 4605         	mov	r5, r0
    1cc0: 20a0         	movs	r0, #0xa0
    1cc2: f005 fb6a    	bl	0x739a <__basepri_max>  @ imm = #0x56d4
    1cc6: f247 0000    	movw	r0, #0x7000
    1cca: 2101         	movs	r1, #0x1
    1ccc: f2c4 0000    	movt	r0, #0x4000
    1cd0: 6001         	str	r1, [r0]
    1cd2: 6041         	str	r1, [r0, #0x4]
    1cd4: 2100         	movs	r1, #0x0
    1cd6: f8c0 1104    	str.w	r1, [r0, #0x104]
    1cda: 4628         	mov	r0, r5
    1cdc: f005 fb64    	bl	0x73a8 <__basepri_w>    @ imm = #0x56c8
    1ce0: f005 fb5e    	bl	0x73a0 <__basepri_r>    @ imm = #0x56bc
    1ce4: 4605         	mov	r5, r0
    1ce6: 2080         	movs	r0, #0x80
    1ce8: f005 fb57    	bl	0x739a <__basepri_max>  @ imm = #0x56ae
    1cec: 4628         	mov	r0, r5
    1cee: f005 fb5b    	bl	0x73a8 <__basepri_w>    @ imm = #0x56b6
    1cf2: 4620         	mov	r0, r4
    1cf4: e8bd 40b0    	pop.w	{r4, r5, r7, lr}
    1cf8: f005 bb56    	b.w	0x73a8 <__basepri_w>    @ imm = #0x56ac

00001cfc <etc_slightly_different::etc::motor_driver::h44161d4f3804bd0a>:
    1cfc: e9d1 3200    	ldrd	r3, r2, [r1]
    1d00: f04f 0c00    	mov.w	r12, #0x0
    1d04: 6889         	ldr	r1, [r1, #0x8]
    1d06: f880 c0b8    	strb.w	r12, [r0, #0xb8]
    1d0a: e9c0 3226    	strd	r3, r2, [r0, #152]
    1d0e: f8c0 10a0    	str.w	r1, [r0, #0xa0]
    1d12: 4770         	bx	lr

00001d14 <apply>:
    1d14: b2c0         	uxtb	r0, r0
    1d16: 283f         	cmp	r0, #0x3f
    1d18: f200 8129    	bhi.w	0x1f6e <apply+0x25a>    @ imm = #0x252
    1d1c: e8df f010    	tbh	[pc, r0, lsl #1]
    1d20: 40 00 41 00  	.word	0x00410040
    1d24: 46 00 40 00  	.word	0x00400046
    1d28: 85 00 79 00  	.word	0x00790085
    1d2c: 59 00 85 00  	.word	0x00850059
    1d30: 90 00 67 00  	.word	0x00670090
    1d34: 88 00 90 00  	.word	0x00900088
    1d38: 40 00 41 00  	.word	0x00410040
    1d3c: 46 00 40 00  	.word	0x00400046
    1d40: f0 00 4f 00  	.word	0x004f00f0
    1d44: 74 00 f0 00  	.word	0x00f00074
    1d48: e4 00 d8 00  	.word	0x00d800e4
    1d4c: a9 00 e4 00  	.word	0x00e400a9
    1d50: be 00 b2 00  	.word	0x00b200be
    1d54: 93 00 be 00  	.word	0x00be0093
    1d58: f0 00 4f 00  	.word	0x004f00f0
    1d5c: 74 00 f0 00  	.word	0x00f00074
    1d60: 1e 01 62 00  	.word	0x0062011e
    1d64: 54 00 1e 01  	.word	0x011e0054
    1d68: 12 01 fd 00  	.word	0x00fd0112
    1d6c: 0a 01 12 01  	.word	0x0112010a
    1d70: cf 00 9c 00  	.word	0x009c00cf
    1d74: c7 00 cf 00  	.word	0x00cf00c7
    1d78: 1e 01 62 00  	.word	0x0062011e
    1d7c: 54 00 1e 01  	.word	0x011e0054
    1d80: 40 00 41 00  	.word	0x00410040
    1d84: 46 00 40 00  	.word	0x00400046
    1d88: 85 00 79 00  	.word	0x00790085
    1d8c: 59 00 85 00  	.word	0x00850059
    1d90: 90 00 67 00  	.word	0x00670090
    1d94: 88 00 90 00  	.word	0x00900088
    1d98: 40 00 41 00  	.word	0x00410040
    1d9c: 46 00 40 00  	.word	0x00400046
    1da0: 4770         	bx	lr
    1da2: f24c 5060    	movw	r0, #0xc560
    1da6: f2c4 0001    	movt	r0, #0x4001
    1daa: e0ad         	b	0x1f08 <apply+0x1f4>    @ imm = #0x15a
    1dac: f24c 5060    	movw	r0, #0xc560
    1db0: f2c4 0001    	movt	r0, #0x4001
    1db4: 6801         	ldr	r1, [r0]
    1db6: f041 4100    	orr	r1, r1, #0x80000000
    1dba: 6001         	str	r1, [r0]
    1dbc: 4770         	bx	lr
    1dbe: f24c 5060    	movw	r0, #0xc560
    1dc2: f2c4 0001    	movt	r0, #0x4001
    1dc6: e093         	b	0x1ef0 <apply+0x1dc>    @ imm = #0x126
    1dc8: f24c 5060    	movw	r0, #0xc560
    1dcc: f2c4 0001    	movt	r0, #0x4001
    1dd0: e079         	b	0x1ec6 <apply+0x1b2>    @ imm = #0xf2
    1dd2: f24c 5060    	movw	r0, #0xc560
    1dd6: f2c4 0001    	movt	r0, #0x4001
    1dda: 6801         	ldr	r1, [r0]
    1ddc: f041 4100    	orr	r1, r1, #0x80000000
    1de0: 6001         	str	r1, [r0]
    1de2: e022         	b	0x1e2a <apply+0x116>    @ imm = #0x44
    1de4: f24c 5060    	movw	r0, #0xc560
    1de8: f2c4 0001    	movt	r0, #0x4001
    1dec: e0ae         	b	0x1f4c <apply+0x238>    @ imm = #0x15c
    1dee: f24c 5060    	movw	r0, #0xc560
    1df2: f2c4 0001    	movt	r0, #0x4001
    1df6: 6801         	ldr	r1, [r0]
    1df8: f041 4100    	orr	r1, r1, #0x80000000
    1dfc: 6001         	str	r1, [r0]
    1dfe: 6841         	ldr	r1, [r0, #0x4]
    1e00: f021 4100    	bic	r1, r1, #0x80000000
    1e04: 6041         	str	r1, [r0, #0x4]
    1e06: e01b         	b	0x1e40 <apply+0x12c>    @ imm = #0x36
    1e08: f24c 5060    	movw	r0, #0xc560
    1e0c: f2c4 0001    	movt	r0, #0x4001
    1e10: e048         	b	0x1ea4 <apply+0x190>    @ imm = #0x90
    1e12: f24c 5060    	movw	r0, #0xc560
    1e16: f2c4 0001    	movt	r0, #0x4001
    1e1a: 6801         	ldr	r1, [r0]
    1e1c: f041 4100    	orr	r1, r1, #0x80000000
    1e20: 6001         	str	r1, [r0]
    1e22: 6841         	ldr	r1, [r0, #0x4]
    1e24: f021 4100    	bic	r1, r1, #0x80000000
    1e28: 6041         	str	r1, [r0, #0x4]
    1e2a: f241 5060    	movw	r0, #0x1560
    1e2e: e069         	b	0x1f04 <apply+0x1f0>    @ imm = #0xd2
    1e30: f24c 5060    	movw	r0, #0xc560
    1e34: f2c4 0001    	movt	r0, #0x4001
    1e38: 6801         	ldr	r1, [r0]
    1e3a: f041 4100    	orr	r1, r1, #0x80000000
    1e3e: 6001         	str	r1, [r0]
    1e40: f241 5060    	movw	r0, #0x1560
    1e44: e08c         	b	0x1f60 <apply+0x24c>    @ imm = #0x118
    1e46: f24c 5060    	movw	r0, #0xc560
    1e4a: f2c4 0001    	movt	r0, #0x4001
    1e4e: 6801         	ldr	r1, [r0]
    1e50: f041 4100    	orr	r1, r1, #0x80000000
    1e54: 6001         	str	r1, [r0]
    1e56: e021         	b	0x1e9c <apply+0x188>    @ imm = #0x42
    1e58: f24c 5060    	movw	r0, #0xc560
    1e5c: f2c4 0001    	movt	r0, #0x4001
    1e60: 6801         	ldr	r1, [r0]
    1e62: f041 4100    	orr	r1, r1, #0x80000000
    1e66: 6001         	str	r1, [r0]
    1e68: 6841         	ldr	r1, [r0, #0x4]
    1e6a: f021 4100    	bic	r1, r1, #0x80000000
    1e6e: 6041         	str	r1, [r0, #0x4]
    1e70: e025         	b	0x1ebe <apply+0x1aa>    @ imm = #0x4a
    1e72: f24c 5060    	movw	r0, #0xc560
    1e76: f2c4 0001    	movt	r0, #0x4001
    1e7a: 6801         	ldr	r1, [r0]
    1e7c: f041 4100    	orr	r1, r1, #0x80000000
    1e80: 6001         	str	r1, [r0]
    1e82: e031         	b	0x1ee8 <apply+0x1d4>    @ imm = #0x62
    1e84: f24c 5060    	movw	r0, #0xc560
    1e88: f2c4 0001    	movt	r0, #0x4001
    1e8c: 6801         	ldr	r1, [r0]
    1e8e: f041 4100    	orr	r1, r1, #0x80000000
    1e92: 6001         	str	r1, [r0]
    1e94: 6841         	ldr	r1, [r0, #0x4]
    1e96: f021 4100    	bic	r1, r1, #0x80000000
    1e9a: 6041         	str	r1, [r0, #0x4]
    1e9c: f241 5060    	movw	r0, #0x1560
    1ea0: f2c4 0002    	movt	r0, #0x4002
    1ea4: 6801         	ldr	r1, [r0]
    1ea6: f041 4100    	orr	r1, r1, #0x80000000
    1eaa: 6001         	str	r1, [r0]
    1eac: e028         	b	0x1f00 <apply+0x1ec>    @ imm = #0x50
    1eae: f24c 5060    	movw	r0, #0xc560
    1eb2: f2c4 0001    	movt	r0, #0x4001
    1eb6: 6801         	ldr	r1, [r0]
    1eb8: f041 4100    	orr	r1, r1, #0x80000000
    1ebc: 6001         	str	r1, [r0]
    1ebe: f241 5060    	movw	r0, #0x1560
    1ec2: f2c4 0002    	movt	r0, #0x4002
    1ec6: 6801         	ldr	r1, [r0]
    1ec8: f041 4100    	orr	r1, r1, #0x80000000
    1ecc: 6001         	str	r1, [r0]
    1ece: e045         	b	0x1f5c <apply+0x248>    @ imm = #0x8a
    1ed0: f24c 5060    	movw	r0, #0xc560
    1ed4: f2c4 0001    	movt	r0, #0x4001
    1ed8: 6801         	ldr	r1, [r0]
    1eda: f041 4100    	orr	r1, r1, #0x80000000
    1ede: 6001         	str	r1, [r0]
    1ee0: 6841         	ldr	r1, [r0, #0x4]
    1ee2: f021 4100    	bic	r1, r1, #0x80000000
    1ee6: 6041         	str	r1, [r0, #0x4]
    1ee8: f241 5060    	movw	r0, #0x1560
    1eec: f2c4 0002    	movt	r0, #0x4002
    1ef0: 6801         	ldr	r1, [r0]
    1ef2: f041 4100    	orr	r1, r1, #0x80000000
    1ef6: 6001         	str	r1, [r0]
    1ef8: 6841         	ldr	r1, [r0, #0x4]
    1efa: f021 4100    	bic	r1, r1, #0x80000000
    1efe: 6041         	str	r1, [r0, #0x4]
    1f00: f242 5060    	movw	r0, #0x2560
    1f04: f2c4 0002    	movt	r0, #0x4002
    1f08: 6801         	ldr	r1, [r0]
    1f0a: f041 4100    	orr	r1, r1, #0x80000000
    1f0e: 6001         	str	r1, [r0]
    1f10: 6841         	ldr	r1, [r0, #0x4]
    1f12: f021 4100    	bic	r1, r1, #0x80000000
    1f16: 6041         	str	r1, [r0, #0x4]
    1f18: 4770         	bx	lr
    1f1a: f24c 5060    	movw	r0, #0xc560
    1f1e: f2c4 0001    	movt	r0, #0x4001
    1f22: 6801         	ldr	r1, [r0]
    1f24: f041 4100    	orr	r1, r1, #0x80000000
    1f28: 6001         	str	r1, [r0]
    1f2a: 6841         	ldr	r1, [r0, #0x4]
    1f2c: f021 4100    	bic	r1, r1, #0x80000000
    1f30: 6041         	str	r1, [r0, #0x4]
    1f32: e007         	b	0x1f44 <apply+0x230>    @ imm = #0xe
    1f34: f24c 5060    	movw	r0, #0xc560
    1f38: f2c4 0001    	movt	r0, #0x4001
    1f3c: 6801         	ldr	r1, [r0]
    1f3e: f041 4100    	orr	r1, r1, #0x80000000
    1f42: 6001         	str	r1, [r0]
    1f44: f241 5060    	movw	r0, #0x1560
    1f48: f2c4 0002    	movt	r0, #0x4002
    1f4c: 6801         	ldr	r1, [r0]
    1f4e: f041 4100    	orr	r1, r1, #0x80000000
    1f52: 6001         	str	r1, [r0]
    1f54: 6841         	ldr	r1, [r0, #0x4]
    1f56: f021 4100    	bic	r1, r1, #0x80000000
    1f5a: 6041         	str	r1, [r0, #0x4]
    1f5c: f242 5060    	movw	r0, #0x2560
    1f60: f2c4 0002    	movt	r0, #0x4002
    1f64: 6801         	ldr	r1, [r0]
    1f66: f041 4100    	orr	r1, r1, #0x80000000
    1f6a: 6001         	str	r1, [r0]
    1f6c: 4770         	bx	lr
    1f6e: b580         	push	{r7, lr}
    1f70: 466f         	mov	r7, sp
    1f72: f247 6058    	movw	r0, #0x7658
    1f76: f647 32ac    	movw	r2, #0x7bac
    1f7a: f2c0 0000    	movt	r0, #0x0
    1f7e: f2c0 0200    	movt	r2, #0x0
    1f82: 2128         	movs	r1, #0x28
    1f84: f004 fb95    	bl	0x66b2 <core::panicking::panic::h26f8d9ea1f810f6d> @ imm = #0x472a

00001f88 <TIMER0>:
    1f88: b5f0         	push	{r4, r5, r6, r7, lr}
    1f8a: af03         	add	r7, sp, #0xc
    1f8c: e92d 0f00    	push.w	{r8, r9, r10, r11}
    1f90: b081         	sub	sp, #0x4
    1f92: ed2d 8b02    	vpush	{d8}
    1f96: b08e         	sub	sp, #0x38
    1f98: f005 fa02    	bl	0x73a0 <__basepri_r>    @ imm = #0x5404
    1f9c: f240 2a50    	movw	r10, #0x250
    1fa0: f2c2 0a00    	movt	r10, #0x2000
    1fa4: f8da 403c    	ldr.w	r4, [r10, #0x3c]
    1fa8: f894 10c0    	ldrb.w	r1, [r4, #0xc0]
    1fac: b3d1         	cbz	r1, 0x2024 <TIMER0+0x9c> @ imm = #0x74
    1fae: f104 01c1    	add.w	r1, r4, #0xc1
    1fb2: 2200         	movs	r2, #0x0
    1fb4: e8d1 3f4f    	ldrexb	r3, [r1]
    1fb8: 2b01         	cmp	r3, #0x1
    1fba: d131         	bne	0x2020 <TIMER0+0x98>    @ imm = #0x62
    1fbc: e8c1 2f43    	strexb	r3, r2, [r1]
    1fc0: 2b00         	cmp	r3, #0x0
    1fc2: d1f7         	bne	0x1fb4 <TIMER0+0x2c>    @ imm = #-0x12
    1fc4: 9000         	str	r0, [sp]
    1fc6: f640 50a3    	movw	r0, #0xda3
    1fca: a902         	add	r1, sp, #0x8
    1fcc: f3bf 8f5f    	dmb	sy
    1fd0: f2c0 0000    	movt	r0, #0x0
    1fd4: f24c 1b08    	movw	r11, #0xc108
    1fd8: 9003         	str	r0, [sp, #0xc]
    1fda: f647 3090    	movw	r0, #0x7b90
    1fde: e9cd 1104    	strd	r1, r1, [sp, #16]
    1fe2: f241 1908    	movw	r9, #0x1108
    1fe6: f894 10b8    	ldrb.w	r1, [r4, #0xb8]
    1fea: f242 1808    	movw	r8, #0x2108
    1fee: f2c0 0000    	movt	r0, #0x0
    1ff2: f2c4 0b01    	movt	r11, #0x4001
    1ff6: 9002         	str	r0, [sp, #0x8]
    1ff8: 2000         	movs	r0, #0x0
    1ffa: f2c4 0902    	movt	r9, #0x4002
    1ffe: f2c4 0802    	movt	r8, #0x4002
    2002: 9006         	str	r0, [sp, #0x18]
    2004: 9401         	str	r4, [sp, #0x4]
    2006: bb91         	cbnz	r1, 0x206e <TIMER0+0xe6> @ imm = #0x64
    2008: f104 0398    	add.w	r3, r4, #0x98
    200c: f104 0ca4    	add.w	r12, r4, #0xa4
    2010: f8a4 00b4    	strh.w	r0, [r4, #0xb4]
    2014: cb0e         	ldm	r3, {r1, r2, r3}
    2016: f8c4 00b0    	str.w	r0, [r4, #0xb0]
    201a: e88c 000e    	stm.w	r12, {r1, r2, r3}
    201e: e009         	b	0x2034 <TIMER0+0xac>    @ imm = #0x12
    2020: f3bf 8f2f    	clrex
    2024: f8da a044    	ldr.w	r10, [r10, #0x44]
    2028: f89a 10a8    	ldrb.w	r1, [r10, #0xa8]
    202c: 2900         	cmp	r1, #0x0
    202e: f040 821b    	bne.w	0x2468 <TIMER0+0x4e0>   @ imm = #0x436
    2032: e335         	b	0x26a0 <TIMER0+0x718>   @ imm = #0x66a
    2034: f005 f9ac    	bl	0x7390 <__primask_r>    @ imm = #0x5358
    2038: 4604         	mov	r4, r0
    203a: f005 f9a2    	bl	0x7382 <__cpsid>        @ imm = #0x5344
    203e: f24b 5004    	movw	r0, #0xb504
    2042: f8da 5048    	ldr.w	r5, [r10, #0x48]
    2046: f2c4 0000    	movt	r0, #0x4000
    204a: 6806         	ldr	r6, [r0]
    204c: 07e0         	lsls	r0, r4, #0x1f
    204e: 9c01         	ldr	r4, [sp, #0x4]
    2050: bf08         	it	eq
    2052: f005 f998    	bleq	0x7386 <__cpsie>        @ imm = #0x5330
    2056: f005 0001    	and	r0, r5, #0x1
    205a: 0a69         	lsrs	r1, r5, #0x9
    205c: ea86 50c0    	eor.w	r0, r6, r0, lsl #23
    2060: eb10 50c5    	adds.w	r0, r0, r5, lsl #23
    2064: f141 0100    	adc	r1, r1, #0x0
    2068: e9c4 0100    	strd	r0, r1, [r4]
    206c: e014         	b	0x2098 <TIMER0+0x110>   @ imm = #0x28
    206e: f104 0008    	add.w	r0, r4, #0x8
    2072: a904         	add	r1, sp, #0x10
    2074: f7fe ff17    	bl	0xea6 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6> @ imm = #-0x11d2
    2078: 2800         	cmp	r0, #0x0
    207a: f040 81e6    	bne.w	0x244a <TIMER0+0x4c2>   @ imm = #0x3cc
    207e: f894 0090    	ldrb.w	r0, [r4, #0x90]
    2082: 2803         	cmp	r0, #0x3
    2084: d108         	bne	0x2098 <TIMER0+0x110>   @ imm = #0x10
    2086: f894 008c    	ldrb.w	r0, [r4, #0x8c]
    208a: 2803         	cmp	r0, #0x3
    208c: bf04         	itt	eq
    208e: f894 0081    	ldrbeq.w	r0, [r4, #0x81]
    2092: 2803         	cmpeq	r0, #0x3
    2094: f000 81bc    	beq.w	0x2410 <TIMER0+0x488>   @ imm = #0x378
    2098: f005 f97a    	bl	0x7390 <__primask_r>    @ imm = #0x52f4
    209c: 4604         	mov	r4, r0
    209e: f005 f970    	bl	0x7382 <__cpsid>        @ imm = #0x52e0
    20a2: f24b 5004    	movw	r0, #0xb504
    20a6: f8da 5048    	ldr.w	r5, [r10, #0x48]
    20aa: f2c4 0000    	movt	r0, #0x4000
    20ae: 6806         	ldr	r6, [r0]
    20b0: 07e0         	lsls	r0, r4, #0x1f
    20b2: bf08         	it	eq
    20b4: f005 f967    	bleq	0x7386 <__cpsie>        @ imm = #0x52ce
    20b8: 9801         	ldr	r0, [sp, #0x4]
    20ba: f005 0201    	and	r2, r5, #0x1
    20be: 0a6b         	lsrs	r3, r5, #0x9
    20c0: ea86 52c2    	eor.w	r2, r6, r2, lsl #23
    20c4: e9d0 0100    	ldrd	r0, r1, [r0]
    20c8: eb12 52c5    	adds.w	r2, r2, r5, lsl #23
    20cc: f143 0300    	adc	r3, r3, #0x0
    20d0: 1a15         	subs	r5, r2, r0
    20d2: eb63 0601    	sbc.w	r6, r3, r1
    20d6: f1b6 3fff    	cmp.w	r6, #0xffffffff
    20da: f340 8227    	ble.w	0x252c <TIMER0+0x5a4>   @ imm = #0x44e
    20de: f005 f95f    	bl	0x73a0 <__basepri_r>    @ imm = #0x52be
    20e2: 4604         	mov	r4, r0
    20e4: 2060         	movs	r0, #0x60
    20e6: f005 f958    	bl	0x739a <__basepri_max>  @ imm = #0x52b0
    20ea: f643 5109    	movw	r1, #0x3d09
    20ee: fba5 0201    	umull	r0, r2, r5, r1
    20f2: fba6 3601    	umull	r3, r6, r6, r1
    20f6: 2100         	movs	r1, #0x0
    20f8: 18d2         	adds	r2, r2, r3
    20fa: f141 0300    	adc	r3, r1, #0x0
    20fe: 2e00         	cmp	r6, #0x0
    2100: bf18         	it	ne
    2102: 2601         	movne	r6, #0x1
    2104: 4333         	orrs	r3, r6
    2106: d112         	bne	0x212e <TIMER0+0x1a6>   @ imm = #0x24
    2108: f244 0300    	movw	r3, #0x4000
    210c: f2c0 330d    	movt	r3, #0x30d
    2110: 1a18         	subs	r0, r3, r0
    2112: eb71 0002    	sbcs.w	r0, r1, r2
    2116: d20a         	bhs	0x212e <TIMER0+0x1a6>   @ imm = #0x14
    2118: f240 7038    	movw	r0, #0x738
    211c: 2600         	movs	r6, #0x0
    211e: f2c2 0000    	movt	r0, #0x2000
    2122: 8006         	strh	r6, [r0]
    2124: 4620         	mov	r0, r4
    2126: f005 f93f    	bl	0x73a8 <__basepri_w>    @ imm = #0x527e
    212a: 9c01         	ldr	r4, [sp, #0x4]
    212c: e039         	b	0x21a2 <TIMER0+0x21a>   @ imm = #0x72
    212e: f005 f937    	bl	0x73a0 <__basepri_r>    @ imm = #0x526e
    2132: 4606         	mov	r6, r0
    2134: 2080         	movs	r0, #0x80
    2136: f005 f930    	bl	0x739a <__basepri_max>  @ imm = #0x5260
    213a: f240 7038    	movw	r0, #0x738
    213e: f2c2 0000    	movt	r0, #0x2000
    2142: 7805         	ldrb	r5, [r0]
    2144: f890 a001    	ldrb.w	r10, [r0, #0x1]
    2148: f240 20e0    	movw	r0, #0x2e0
    214c: f2c2 0000    	movt	r0, #0x2000
    2150: ed90 8a00    	vldr	s16, [r0]
    2154: 4630         	mov	r0, r6
    2156: f005 f927    	bl	0x73a8 <__basepri_w>    @ imm = #0x524e
    215a: 4620         	mov	r0, r4
    215c: f005 f924    	bl	0x73a8 <__basepri_w>    @ imm = #0x5248
    2160: 9c01         	ldr	r4, [sp, #0x4]
    2162: f894 00b4    	ldrb.w	r0, [r4, #0xb4]
    2166: f884 a0b7    	strb.w	r10, [r4, #0xb7]
    216a: 42a8         	cmp	r0, r5
    216c: f884 50b6    	strb.w	r5, [r4, #0xb6]
    2170: d107         	bne	0x2182 <TIMER0+0x1fa>   @ imm = #0xe
    2172: ed94 0a2c    	vldr	s0, [r4, #176]
    2176: eeb4 8a40    	vcmp.f32	s16, s0
    217a: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    217e: f000 8159    	beq.w	0x2434 <TIMER0+0x4ac>   @ imm = #0x2b2
    2182: eeb0 0a48    	vmov.f32	s0, s16
    2186: 4628         	mov	r0, r5
    2188: f003 fcf3    	bl	0x5b72 <get_u8>         @ imm = #0x39e6
    218c: f884 a0b5    	strb.w	r10, [r4, #0xb5]
    2190: f240 2a50    	movw	r10, #0x250
    2194: 4606         	mov	r6, r0
    2196: f2c2 0a00    	movt	r10, #0x2000
    219a: f884 50b4    	strb.w	r5, [r4, #0xb4]
    219e: ed84 8a2c    	vstr	s16, [r4, #176]
    21a2: 4630         	mov	r0, r6
    21a4: f7ff fdb6    	bl	0x1d14 <apply>          @ imm = #-0x494
    21a8: f24e 1100    	movw	r1, #0xe100
    21ac: ed94 0a2c    	vldr	s0, [r4, #176]
    21b0: f04f 5080    	mov.w	r0, #0x10000000
    21b4: f2ce 0100    	movt	r1, #0xe000
    21b8: 6008         	str	r0, [r1]
    21ba: eeb7 1a00    	vmov.f32	s2, #1.000000e+00
    21be: f8c1 0180    	str.w	r0, [r1, #0x180]
    21c2: eeb0 0ac0    	vabs.f32	s0, s0
    21c6: f8db 0400    	ldr.w	r0, [r11, #0x400]
    21ca: f24c 526c    	movw	r2, #0xc56c
    21ce: f2c4 0201    	movt	r2, #0x4001
    21d2: f244 2640    	movw	r6, #0x4240
    21d6: f2c0 060f    	movt	r6, #0xf
    21da: f36f 30df    	bfc	r0, #15, #17
    21de: ee02 0a10    	vmov	s4, r0
    21e2: f8db 0400    	ldr.w	r0, [r11, #0x400]
    21e6: ee31 1a40    	vsub.f32	s2, s2, s0
    21ea: ed9f 0ada    	vldr	s0, [pc, #872]          @ 0x2554 <TIMER0+0x5cc>
    21ee: eeb8 2a42    	vcvt.f32.u32	s4, s4
    21f2: f36f 30df    	bfc	r0, #15, #17
    21f6: ee21 2a02    	vmul.f32	s4, s2, s4
    21fa: eebc 3ac2    	vcvt.u32.f32	s6, s4
    21fe: eeb5 2a40    	vcmp.f32	s4, #0
    2202: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    2206: eeb4 2a40    	vcmp.f32	s4, s0
    220a: ee13 1a10    	vmov	r1, s6
    220e: bfb8         	it	lt
    2210: 2100         	movlt	r1, #0x0
    2212: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    2216: bfc8         	it	gt
    2218: f64f 71ff    	movwgt	r1, #0xffff
    221c: 4288         	cmp	r0, r1
    221e: bf28         	it	hs
    2220: 4608         	movhs	r0, r1
    2222: 4651         	mov	r1, r10
    2224: f821 0f70    	strh	r0, [r1, #112]!
    2228: 80c8         	strh	r0, [r1, #0x6]
    222a: 8088         	strh	r0, [r1, #0x4]
    222c: 8048         	strh	r0, [r1, #0x2]
    222e: 2000         	movs	r0, #0x0
    2230: f842 0c58    	str	r0, [r2, #-88]
    2234: f8cb 00f8    	str.w	r0, [r11, #0xf8]
    2238: f8db 0408    	ldr.w	r0, [r11, #0x408]
    223c: f020 0003    	bic	r0, r0, #0x3
    2240: f8cb 0408    	str.w	r0, [r11, #0x408]
    2244: 6810         	ldr	r0, [r2]
    2246: f020 4000    	bic	r0, r0, #0x80000000
    224a: 6010         	str	r0, [r2]
    224c: f842 1c4c    	str	r1, [r2, #-76]
    2250: 2001         	movs	r0, #0x1
    2252: f06f 01ff    	mvn	r1, #0xff
    2256: f842 0c48    	str	r0, [r2, #-72]
    225a: f8cb 03f8    	str.w	r0, [r11, #0x3f8]
    225e: f84b 0001    	str.w	r0, [r11, r1]
    2262: 4630         	mov	r0, r6
    2264: e000         	b	0x2268 <TIMER0+0x2e0>   @ imm = #0x0
    2266: 3804         	subs	r0, #0x4
    2268: f8db 1000    	ldr.w	r1, [r11]
    226c: b969         	cbnz	r1, 0x228a <TIMER0+0x302> @ imm = #0x1a
    226e: 2800         	cmp	r0, #0x0
    2270: f000 8155    	beq.w	0x251e <TIMER0+0x596>   @ imm = #0x2aa
    2274: f8db 1000    	ldr.w	r1, [r11]
    2278: b939         	cbnz	r1, 0x228a <TIMER0+0x302> @ imm = #0xe
    227a: f8db 1000    	ldr.w	r1, [r11]
    227e: 2900         	cmp	r1, #0x0
    2280: bf04         	itt	eq
    2282: f8db 1000    	ldreq.w	r1, [r11]
    2286: 2900         	cmpeq	r1, #0x0
    2288: d0ed         	beq	0x2266 <TIMER0+0x2de>   @ imm = #-0x26
    228a: 2100         	movs	r1, #0x0
    228c: f8cb 1008    	str.w	r1, [r11, #0x8]
    2290: f8cb 100c    	str.w	r1, [r11, #0xc]
    2294: f8d9 0400    	ldr.w	r0, [r9, #0x400]
    2298: f8d9 2400    	ldr.w	r2, [r9, #0x400]
    229c: f36f 30df    	bfc	r0, #15, #17
    22a0: f36f 32df    	bfc	r2, #15, #17
    22a4: ee02 0a10    	vmov	s4, r0
    22a8: eeb8 2a42    	vcvt.f32.u32	s4, s4
    22ac: ee21 2a02    	vmul.f32	s4, s2, s4
    22b0: eebc 3ac2    	vcvt.u32.f32	s6, s4
    22b4: eeb5 2a40    	vcmp.f32	s4, #0
    22b8: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    22bc: eeb4 2a40    	vcmp.f32	s4, s0
    22c0: ee13 0a10    	vmov	r0, s6
    22c4: bfb8         	it	lt
    22c6: 2000         	movlt	r0, #0x0
    22c8: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    22cc: bfc8         	it	gt
    22ce: f64f 70ff    	movwgt	r0, #0xffff
    22d2: 4282         	cmp	r2, r0
    22d4: bf28         	it	hs
    22d6: 4602         	movhs	r2, r0
    22d8: f240 20c8    	movw	r0, #0x2c8
    22dc: f2c2 0000    	movt	r0, #0x2000
    22e0: 80c2         	strh	r2, [r0, #0x6]
    22e2: 8082         	strh	r2, [r0, #0x4]
    22e4: 8042         	strh	r2, [r0, #0x2]
    22e6: 8002         	strh	r2, [r0]
    22e8: f241 526c    	movw	r2, #0x156c
    22ec: f2c4 0202    	movt	r2, #0x4002
    22f0: f842 1c58    	str	r1, [r2, #-88]
    22f4: f8c9 10f8    	str.w	r1, [r9, #0xf8]
    22f8: f8d9 1408    	ldr.w	r1, [r9, #0x408]
    22fc: f021 0103    	bic	r1, r1, #0x3
    2300: f8c9 1408    	str.w	r1, [r9, #0x408]
    2304: 6811         	ldr	r1, [r2]
    2306: f021 4100    	bic	r1, r1, #0x80000000
    230a: 6011         	str	r1, [r2]
    230c: 2101         	movs	r1, #0x1
    230e: f842 0c4c    	str	r0, [r2, #-76]
    2312: f842 1c48    	str	r1, [r2, #-72]
    2316: f06f 02ff    	mvn	r2, #0xff
    231a: f8c9 13f8    	str.w	r1, [r9, #0x3f8]
    231e: f849 1002    	str.w	r1, [r9, r2]
    2322: 4631         	mov	r1, r6
    2324: e000         	b	0x2328 <TIMER0+0x3a0>   @ imm = #0x0
    2326: 3904         	subs	r1, #0x4
    2328: f8d9 2000    	ldr.w	r2, [r9]
    232c: b96a         	cbnz	r2, 0x234a <TIMER0+0x3c2> @ imm = #0x1a
    232e: 2900         	cmp	r1, #0x0
    2330: f000 80f5    	beq.w	0x251e <TIMER0+0x596>   @ imm = #0x1ea
    2334: f8d9 2000    	ldr.w	r2, [r9]
    2338: b93a         	cbnz	r2, 0x234a <TIMER0+0x3c2> @ imm = #0xe
    233a: f8d9 2000    	ldr.w	r2, [r9]
    233e: 2a00         	cmp	r2, #0x0
    2340: bf04         	itt	eq
    2342: f8d9 2000    	ldreq.w	r2, [r9]
    2346: 2a00         	cmpeq	r2, #0x0
    2348: d0ed         	beq	0x2326 <TIMER0+0x39e>   @ imm = #-0x26
    234a: 2100         	movs	r1, #0x0
    234c: f8c9 1008    	str.w	r1, [r9, #0x8]
    2350: f8c9 100c    	str.w	r1, [r9, #0xc]
    2354: f8d8 2400    	ldr.w	r2, [r8, #0x400]
    2358: f36f 32df    	bfc	r2, #15, #17
    235c: ee02 2a10    	vmov	s4, r2
    2360: f8d8 2400    	ldr.w	r2, [r8, #0x400]
    2364: eeb8 2a42    	vcvt.f32.u32	s4, s4
    2368: f36f 32df    	bfc	r2, #15, #17
    236c: ee21 1a02    	vmul.f32	s2, s2, s4
    2370: eebc 2ac1    	vcvt.u32.f32	s4, s2
    2374: eeb5 1a40    	vcmp.f32	s2, #0
    2378: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    237c: eeb4 1a40    	vcmp.f32	s2, s0
    2380: ee12 3a10    	vmov	r3, s4
    2384: bfb8         	it	lt
    2386: 2300         	movlt	r3, #0x0
    2388: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    238c: bfc8         	it	gt
    238e: f64f 73ff    	movwgt	r3, #0xffff
    2392: 429a         	cmp	r2, r3
    2394: bf28         	it	hs
    2396: 461a         	movhs	r2, r3
    2398: f820 2f08    	strh	r2, [r0, #8]!
    239c: 80c2         	strh	r2, [r0, #0x6]
    239e: 8082         	strh	r2, [r0, #0x4]
    23a0: 8042         	strh	r2, [r0, #0x2]
    23a2: f242 526c    	movw	r2, #0x256c
    23a6: f2c4 0202    	movt	r2, #0x4002
    23aa: f842 1c58    	str	r1, [r2, #-88]
    23ae: f8c8 10f8    	str.w	r1, [r8, #0xf8]
    23b2: f8d8 1408    	ldr.w	r1, [r8, #0x408]
    23b6: f021 0103    	bic	r1, r1, #0x3
    23ba: f8c8 1408    	str.w	r1, [r8, #0x408]
    23be: 6811         	ldr	r1, [r2]
    23c0: f021 4100    	bic	r1, r1, #0x80000000
    23c4: 6011         	str	r1, [r2]
    23c6: f842 0c4c    	str	r0, [r2, #-76]
    23ca: 2001         	movs	r0, #0x1
    23cc: f06f 01ff    	mvn	r1, #0xff
    23d0: f842 0c48    	str	r0, [r2, #-72]
    23d4: f8c8 03f8    	str.w	r0, [r8, #0x3f8]
    23d8: f848 0001    	str.w	r0, [r8, r1]
    23dc: 4630         	mov	r0, r6
    23de: e000         	b	0x23e2 <TIMER0+0x45a>   @ imm = #0x0
    23e0: 3804         	subs	r0, #0x4
    23e2: f8d8 1000    	ldr.w	r1, [r8]
    23e6: b969         	cbnz	r1, 0x2404 <TIMER0+0x47c> @ imm = #0x1a
    23e8: 2800         	cmp	r0, #0x0
    23ea: f000 8098    	beq.w	0x251e <TIMER0+0x596>   @ imm = #0x130
    23ee: f8d8 1000    	ldr.w	r1, [r8]
    23f2: b939         	cbnz	r1, 0x2404 <TIMER0+0x47c> @ imm = #0xe
    23f4: f8d8 1000    	ldr.w	r1, [r8]
    23f8: 2900         	cmp	r1, #0x0
    23fa: bf04         	itt	eq
    23fc: f8d8 1000    	ldreq.w	r1, [r8]
    2400: 2900         	cmpeq	r1, #0x0
    2402: d0ed         	beq	0x23e0 <TIMER0+0x458>   @ imm = #-0x26
    2404: 2000         	movs	r0, #0x0
    2406: f8c8 0008    	str.w	r0, [r8, #0x8]
    240a: f8c8 000c    	str.w	r0, [r8, #0xc]
    240e: e611         	b	0x2034 <TIMER0+0xac>    @ imm = #-0x3de
    2410: e9d4 601a    	ldrd	r6, r0, [r4, #104]
    2414: 6805         	ldr	r5, [r0]
    2416: f004 ffbb    	bl	0x7390 <__primask_r>    @ imm = #0x4f76
    241a: 4604         	mov	r4, r0
    241c: f004 ffb1    	bl	0x7382 <__cpsid>        @ imm = #0x4f62
    2420: f3bf 8f5f    	dmb	sy
    2424: 6830         	ldr	r0, [r6]
    2426: 2800         	cmp	r0, #0x0
    2428: d04c         	beq	0x24c4 <TIMER0+0x53c>   @ imm = #0x98
    242a: 6981         	ldr	r1, [r0, #0x18]
    242c: 4285         	cmp	r5, r0
    242e: d143         	bne	0x24b8 <TIMER0+0x530>   @ imm = #0x86
    2430: 6031         	str	r1, [r6]
    2432: e047         	b	0x24c4 <TIMER0+0x53c>   @ imm = #0x8e
    2434: f240 2a50    	movw	r10, #0x250
    2438: 2000         	movs	r0, #0x0
    243a: 2101         	movs	r1, #0x1
    243c: f884 0090    	strb.w	r0, [r4, #0x90]
    2440: e9c4 1002    	strd	r1, r0, [r4, #8]
    2444: f2c2 0a00    	movt	r10, #0x2000
    2448: e611         	b	0x206e <TIMER0+0xe6>    @ imm = #-0x3de
    244a: e9dd 1002    	ldrd	r1, r0, [sp, #8]
    244e: 2203         	movs	r2, #0x3
    2450: f884 20b8    	strb.w	r2, [r4, #0xb8]
    2454: 68c9         	ldr	r1, [r1, #0xc]
    2456: 4788         	blx	r1
    2458: 9800         	ldr	r0, [sp]
    245a: f8da a044    	ldr.w	r10, [r10, #0x44]
    245e: f89a 10a8    	ldrb.w	r1, [r10, #0xa8]
    2462: 2900         	cmp	r1, #0x0
    2464: f000 811c    	beq.w	0x26a0 <TIMER0+0x718>   @ imm = #0x238
    2468: f10a 01a9    	add.w	r1, r10, #0xa9
    246c: 2200         	movs	r2, #0x0
    246e: e8d1 3f4f    	ldrexb	r3, [r1]
    2472: 2b01         	cmp	r3, #0x1
    2474: d11d         	bne	0x24b2 <TIMER0+0x52a>   @ imm = #0x3a
    2476: e8c1 2f43    	strexb	r3, r2, [r1]
    247a: 2b00         	cmp	r3, #0x0
    247c: d1f7         	bne	0x246e <TIMER0+0x4e6>   @ imm = #-0x12
    247e: 4680         	mov	r8, r0
    2480: f640 50c7    	movw	r0, #0xdc7
    2484: f2c0 0000    	movt	r0, #0x0
    2488: f3bf 8f5f    	dmb	sy
    248c: 9005         	str	r0, [sp, #0x14]
    248e: f647 3090    	movw	r0, #0x7b90
    2492: f2c0 0000    	movt	r0, #0x0
    2496: a904         	add	r1, sp, #0x10
    2498: 9004         	str	r0, [sp, #0x10]
    249a: 2000         	movs	r0, #0x0
    249c: 900a         	str	r0, [sp, #0x28]
    249e: f89a 00a4    	ldrb.w	r0, [r10, #0xa4]
    24a2: e9cd 1108    	strd	r1, r1, [sp, #32]
    24a6: b390         	cbz	r0, 0x250e <TIMER0+0x586> @ imm = #0x64
    24a8: 2803         	cmp	r0, #0x3
    24aa: f000 80cc    	beq.w	0x2646 <TIMER0+0x6be>   @ imm = #0x198
    24ae: f004 fc00    	bl	0x6cb2 <core::panicking::panic_const::panic_const_async_fn_resumed::h274f44943fe874ff> @ imm = #0x4800
    24b2: f3bf 8f2f    	clrex
    24b6: e0f3         	b	0x26a0 <TIMER0+0x718>   @ imm = #0x1e6
    24b8: b121         	cbz	r1, 0x24c4 <TIMER0+0x53c> @ imm = #0x8
    24ba: 428d         	cmp	r5, r1
    24bc: d111         	bne	0x24e2 <TIMER0+0x55a>   @ imm = #0x22
    24be: 460a         	mov	r2, r1
    24c0: 6991         	ldr	r1, [r2, #0x18]
    24c2: 6181         	str	r1, [r0, #0x18]
    24c4: 07e0         	lsls	r0, r4, #0x1f
    24c6: bf08         	it	eq
    24c8: f004 ff5d    	bleq	0x7386 <__cpsie>        @ imm = #0x4eba
    24cc: 9a01         	ldr	r2, [sp, #0x4]
    24ce: 2100         	movs	r1, #0x0
    24d0: 6a90         	ldr	r0, [r2, #0x28]
    24d2: f882 1080    	strb.w	r1, [r2, #0x80]
    24d6: 2800         	cmp	r0, #0x0
    24d8: bf1e         	ittt	ne
    24da: 68c1         	ldrne	r1, [r0, #0xc]
    24dc: 6ad0         	ldrne	r0, [r2, #0x2c]
    24de: 4788         	blxne	r1
    24e0: e5da         	b	0x2098 <TIMER0+0x110>   @ imm = #-0x44c
    24e2: 6988         	ldr	r0, [r1, #0x18]
    24e4: 2800         	cmp	r0, #0x0
    24e6: d0ed         	beq	0x24c4 <TIMER0+0x53c>   @ imm = #-0x26
    24e8: 4285         	cmp	r5, r0
    24ea: d015         	beq	0x2518 <TIMER0+0x590>   @ imm = #0x2a
    24ec: 6981         	ldr	r1, [r0, #0x18]
    24ee: 2900         	cmp	r1, #0x0
    24f0: d0e8         	beq	0x24c4 <TIMER0+0x53c>   @ imm = #-0x30
    24f2: 428d         	cmp	r5, r1
    24f4: d0e3         	beq	0x24be <TIMER0+0x536>   @ imm = #-0x3a
    24f6: 6988         	ldr	r0, [r1, #0x18]
    24f8: 2800         	cmp	r0, #0x0
    24fa: d0e3         	beq	0x24c4 <TIMER0+0x53c>   @ imm = #-0x3a
    24fc: 4285         	cmp	r5, r0
    24fe: d00b         	beq	0x2518 <TIMER0+0x590>   @ imm = #0x16
    2500: 6981         	ldr	r1, [r0, #0x18]
    2502: 2900         	cmp	r1, #0x0
    2504: d0de         	beq	0x24c4 <TIMER0+0x53c>   @ imm = #-0x44
    2506: 428d         	cmp	r5, r1
    2508: 460a         	mov	r2, r1
    250a: d1ea         	bne	0x24e2 <TIMER0+0x55a>   @ imm = #-0x2c
    250c: e7d8         	b	0x24c0 <TIMER0+0x538>   @ imm = #-0x50
    250e: f8da 0090    	ldr.w	r0, [r10, #0x90]
    2512: f8ca 0094    	str.w	r0, [r10, #0x94]
    2516: e01f         	b	0x2558 <TIMER0+0x5d0>   @ imm = #0x3e
    2518: 4602         	mov	r2, r0
    251a: 4608         	mov	r0, r1
    251c: e7d0         	b	0x24c0 <TIMER0+0x538>   @ imm = #-0x60
    251e: f247 6148    	movw	r1, #0x7648
    2522: a808         	add	r0, sp, #0x20
    2524: f2c0 0100    	movt	r1, #0x0
    2528: f004 fb8c    	bl	0x6c44 <core::result::unwrap_failed::h6d5ad66472df96c4> @ imm = #0x4718
    252c: 2101         	movs	r1, #0x1
    252e: 2000         	movs	r0, #0x0
    2530: 9109         	str	r1, [sp, #0x24]
    2532: f247 61dc    	movw	r1, #0x76dc
    2536: f2c0 0100    	movt	r1, #0x0
    253a: 900c         	str	r0, [sp, #0x30]
    253c: 900b         	str	r0, [sp, #0x2c]
    253e: 2004         	movs	r0, #0x4
    2540: 9108         	str	r1, [sp, #0x20]
    2542: f647 31ac    	movw	r1, #0x7bac
    2546: 900a         	str	r0, [sp, #0x28]
    2548: a808         	add	r0, sp, #0x20
    254a: f2c0 0100    	movt	r1, #0x0
    254e: f003 fbdc    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #0x37b8
    2552: bf00         	nop
    2554: 00 ff 7f 47  	.word	0x477fff00
    2558: 6a81         	ldr	r1, [r0, #0x28]
    255a: 2200         	movs	r2, #0x0
    255c: f04f 0900    	mov.w	r9, #0x0
    2560: 290a         	cmp	r1, #0xa
    2562: bf38         	it	lo
    2564: 1c4a         	addlo	r2, r1, #0x1
    2566: 6282         	str	r2, [r0, #0x28]
    2568: bf28         	it	hs
    256a: 4649         	movhs	r1, r9
    256c: f850 0021    	ldr.w	r0, [r0, r1, lsl #2]
    2570: f8ca 0098    	str.w	r0, [r10, #0x98]
    2574: f004 ff14    	bl	0x73a0 <__basepri_r>    @ imm = #0x4e28
    2578: 4683         	mov	r11, r0
    257a: 2080         	movs	r0, #0x80
    257c: f004 ff0d    	bl	0x739a <__basepri_max>  @ imm = #0x4e1a
    2580: f004 ff0e    	bl	0x73a0 <__basepri_r>    @ imm = #0x4e1c
    2584: 4606         	mov	r6, r0
    2586: 2080         	movs	r0, #0x80
    2588: f004 ff07    	bl	0x739a <__basepri_max>  @ imm = #0x4e0e
    258c: f240 20f8    	movw	r0, #0x2f8
    2590: f2c2 0000    	movt	r0, #0x2000
    2594: 6804         	ldr	r4, [r0]
    2596: f240 20e0    	movw	r0, #0x2e0
    259a: f2c2 0000    	movt	r0, #0x2000
    259e: 6805         	ldr	r5, [r0]
    25a0: 4630         	mov	r0, r6
    25a2: f004 ff01    	bl	0x73a8 <__basepri_w>    @ imm = #0x4e02
    25a6: 4658         	mov	r0, r11
    25a8: f004 fefe    	bl	0x73a8 <__basepri_w>    @ imm = #0x4dfc
    25ac: e9ca 4527    	strd	r4, r5, [r10, #156]
    25b0: f004 fef6    	bl	0x73a0 <__basepri_r>    @ imm = #0x4dec
    25b4: 4605         	mov	r5, r0
    25b6: 2080         	movs	r0, #0x80
    25b8: f004 feef    	bl	0x739a <__basepri_max>  @ imm = #0x4dde
    25bc: f240 21fc    	movw	r1, #0x2fc
    25c0: f8da 0098    	ldr.w	r0, [r10, #0x98]
    25c4: f2c2 0100    	movt	r1, #0x2000
    25c8: 6008         	str	r0, [r1]
    25ca: 4628         	mov	r0, r5
    25cc: f004 feec    	bl	0x73a8 <__basepri_w>    @ imm = #0x4dd8
    25d0: f004 fc59    	bl	0x6e86 <_defmt_acquire> @ imm = #0x48b2
    25d4: f240 000a    	movw	r0, #0xa
    25d8: f2c0 0000    	movt	r0, #0x0
    25dc: f004 fbd8    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x47b0
    25e0: f240 0504    	movw	r5, #0x4
    25e4: a802         	add	r0, sp, #0x8
    25e6: f2c0 0500    	movt	r5, #0x0
    25ea: 2102         	movs	r1, #0x2
    25ec: f8ad 5008    	strh.w	r5, [sp, #0x8]
    25f0: f004 fd15    	bl	0x701e <_defmt_write>   @ imm = #0x4a2a
    25f4: f8da 009c    	ldr.w	r0, [r10, #0x9c]
    25f8: 2104         	movs	r1, #0x4
    25fa: 9002         	str	r0, [sp, #0x8]
    25fc: a802         	add	r0, sp, #0x8
    25fe: f004 fd0e    	bl	0x701e <_defmt_write>   @ imm = #0x4a1c
    2602: a802         	add	r0, sp, #0x8
    2604: 2102         	movs	r1, #0x2
    2606: f8ad 5008    	strh.w	r5, [sp, #0x8]
    260a: f004 fd08    	bl	0x701e <_defmt_write>   @ imm = #0x4a10
    260e: f8da 00a0    	ldr.w	r0, [r10, #0xa0]
    2612: 2104         	movs	r1, #0x4
    2614: 9002         	str	r0, [sp, #0x8]
    2616: a802         	add	r0, sp, #0x8
    2618: f004 fd01    	bl	0x701e <_defmt_write>   @ imm = #0x4a02
    261c: a802         	add	r0, sp, #0x8
    261e: 2102         	movs	r1, #0x2
    2620: f8ad 5008    	strh.w	r5, [sp, #0x8]
    2624: f004 fcfb    	bl	0x701e <_defmt_write>   @ imm = #0x49f6
    2628: f8da 0098    	ldr.w	r0, [r10, #0x98]
    262c: 2104         	movs	r1, #0x4
    262e: 9002         	str	r0, [sp, #0x8]
    2630: a802         	add	r0, sp, #0x8
    2632: f004 fcf4    	bl	0x701e <_defmt_write>   @ imm = #0x49e8
    2636: f004 fc7b    	bl	0x6f30 <_defmt_release> @ imm = #0x48f6
    263a: f44f 3020    	mov.w	r0, #0x28000
    263e: f88a 9088    	strb.w	r9, [r10, #0x88]
    2642: e9ca 0900    	strd	r0, r9, [r10]
    2646: a908         	add	r1, sp, #0x20
    2648: 4650         	mov	r0, r10
    264a: f7fe fc2c    	bl	0xea6 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6> @ imm = #-0x17a8
    264e: b9f8         	cbnz	r0, 0x2690 <TIMER0+0x708> @ imm = #0x3e
    2650: f89a 0088    	ldrb.w	r0, [r10, #0x88]
    2654: 2803         	cmp	r0, #0x3
    2656: d107         	bne	0x2668 <TIMER0+0x6e0>   @ imm = #0xe
    2658: f89a 0084    	ldrb.w	r0, [r10, #0x84]
    265c: 2803         	cmp	r0, #0x3
    265e: bf04         	itt	eq
    2660: f89a 0079    	ldrbeq.w	r0, [r10, #0x79]
    2664: 2803         	cmpeq	r0, #0x3
    2666: d002         	beq	0x266e <TIMER0+0x6e6>   @ imm = #0x4
    2668: f8da 0094    	ldr.w	r0, [r10, #0x94]
    266c: e774         	b	0x2558 <TIMER0+0x5d0>   @ imm = #-0x118
    266e: e9da 5018    	ldrd	r5, r0, [r10, #96]
    2672: 6806         	ldr	r6, [r0]
    2674: f004 fe8c    	bl	0x7390 <__primask_r>    @ imm = #0x4d18
    2678: 4681         	mov	r9, r0
    267a: f004 fe82    	bl	0x7382 <__cpsid>        @ imm = #0x4d04
    267e: f3bf 8f5f    	dmb	sy
    2682: 6828         	ldr	r0, [r5]
    2684: b1d8         	cbz	r0, 0x26be <TIMER0+0x736> @ imm = #0x36
    2686: 6981         	ldr	r1, [r0, #0x18]
    2688: 4286         	cmp	r6, r0
    268a: d112         	bne	0x26b2 <TIMER0+0x72a>   @ imm = #0x24
    268c: 6029         	str	r1, [r5]
    268e: e016         	b	0x26be <TIMER0+0x736>   @ imm = #0x2c
    2690: e9dd 1004    	ldrd	r1, r0, [sp, #16]
    2694: 2203         	movs	r2, #0x3
    2696: f88a 20a4    	strb.w	r2, [r10, #0xa4]
    269a: 68c9         	ldr	r1, [r1, #0xc]
    269c: 4788         	blx	r1
    269e: 4640         	mov	r0, r8
    26a0: f004 fe82    	bl	0x73a8 <__basepri_w>    @ imm = #0x4d04
    26a4: b00e         	add	sp, #0x38
    26a6: ecbd 8b02    	vpop	{d8}
    26aa: b001         	add	sp, #0x4
    26ac: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    26b0: bdf0         	pop	{r4, r5, r6, r7, pc}
    26b2: b121         	cbz	r1, 0x26be <TIMER0+0x736> @ imm = #0x8
    26b4: 428e         	cmp	r6, r1
    26b6: d113         	bne	0x26e0 <TIMER0+0x758>   @ imm = #0x26
    26b8: 460a         	mov	r2, r1
    26ba: 6991         	ldr	r1, [r2, #0x18]
    26bc: 6181         	str	r1, [r0, #0x18]
    26be: ea5f 70c9    	lsls.w	r0, r9, #0x1f
    26c2: bf08         	it	eq
    26c4: f004 fe5f    	bleq	0x7386 <__cpsie>        @ imm = #0x4cbe
    26c8: f8da 0020    	ldr.w	r0, [r10, #0x20]
    26cc: 2100         	movs	r1, #0x0
    26ce: f88a 1078    	strb.w	r1, [r10, #0x78]
    26d2: 2800         	cmp	r0, #0x0
    26d4: bf1e         	ittt	ne
    26d6: 68c1         	ldrne	r1, [r0, #0xc]
    26d8: f8da 0024    	ldrne.w	r0, [r10, #0x24]
    26dc: 4788         	blxne	r1
    26de: e7c3         	b	0x2668 <TIMER0+0x6e0>   @ imm = #-0x7a
    26e0: 6988         	ldr	r0, [r1, #0x18]
    26e2: 2800         	cmp	r0, #0x0
    26e4: d0eb         	beq	0x26be <TIMER0+0x736>   @ imm = #-0x2a
    26e6: 4286         	cmp	r6, r0
    26e8: d010         	beq	0x270c <TIMER0+0x784>   @ imm = #0x20
    26ea: 6981         	ldr	r1, [r0, #0x18]
    26ec: 2900         	cmp	r1, #0x0
    26ee: d0e6         	beq	0x26be <TIMER0+0x736>   @ imm = #-0x34
    26f0: 428e         	cmp	r6, r1
    26f2: d0e1         	beq	0x26b8 <TIMER0+0x730>   @ imm = #-0x3e
    26f4: 6988         	ldr	r0, [r1, #0x18]
    26f6: 2800         	cmp	r0, #0x0
    26f8: d0e1         	beq	0x26be <TIMER0+0x736>   @ imm = #-0x3e
    26fa: 4286         	cmp	r6, r0
    26fc: d006         	beq	0x270c <TIMER0+0x784>   @ imm = #0xc
    26fe: 6981         	ldr	r1, [r0, #0x18]
    2700: 2900         	cmp	r1, #0x0
    2702: d0dc         	beq	0x26be <TIMER0+0x736>   @ imm = #-0x48
    2704: 428e         	cmp	r6, r1
    2706: 460a         	mov	r2, r1
    2708: d1ea         	bne	0x26e0 <TIMER0+0x758>   @ imm = #-0x2c
    270a: e7d6         	b	0x26ba <TIMER0+0x732>   @ imm = #-0x54
    270c: 4602         	mov	r2, r0
    270e: 4608         	mov	r0, r1
    2710: e7d3         	b	0x26ba <TIMER0+0x732>   @ imm = #-0x5a
    2712: d4d4         	bmi	0x26be <TIMER0+0x736>   @ imm = #-0x58

00002714 <TIMER1>:
    2714: b5f0         	push	{r4, r5, r6, r7, lr}
    2716: af03         	add	r7, sp, #0xc
    2718: e92d 0f00    	push.w	{r8, r9, r10, r11}
    271c: b09b         	sub	sp, #0x6c
    271e: f004 fe3f    	bl	0x73a0 <__basepri_r>    @ imm = #0x4c7e
    2722: 4682         	mov	r10, r0
    2724: f240 2050    	movw	r0, #0x250
    2728: f2c2 0000    	movt	r0, #0x2000
    272c: f240 2be0    	movw	r11, #0x2e0
    2730: 6b83         	ldr	r3, [r0, #0x38]
    2732: f2c2 0b00    	movt	r11, #0x2000
    2736: f893 0020    	ldrb.w	r0, [r3, #0x20]
    273a: b348         	cbz	r0, 0x2790 <TIMER1+0x7c> @ imm = #0x52
    273c: f103 0021    	add.w	r0, r3, #0x21
    2740: 2100         	movs	r1, #0x0
    2742: e8d0 2f4f    	ldrexb	r2, [r0]
    2746: 2a01         	cmp	r2, #0x1
    2748: d116         	bne	0x2778 <TIMER1+0x64>    @ imm = #0x2c
    274a: e8c0 1f42    	strexb	r2, r1, [r0]
    274e: 2a00         	cmp	r2, #0x0
    2750: d1f7         	bne	0x2742 <TIMER1+0x2e>    @ imm = #-0x12
    2752: f8cd a008    	str.w	r10, [sp, #0x8]
    2756: f243 1a08    	movw	r10, #0x3108
    275a: f3bf 8f5f    	dmb	sy
    275e: f2c4 0a00    	movt	r10, #0x4000
    2762: 7f18         	ldrb	r0, [r3, #0x1c]
    2764: 9307         	str	r3, [sp, #0x1c]
    2766: b150         	cbz	r0, 0x277e <TIMER1+0x6a> @ imm = #0x14
    2768: 2803         	cmp	r0, #0x3
    276a: d10f         	bne	0x278c <TIMER1+0x78>    @ imm = #0x1e
    276c: 7e18         	ldrb	r0, [r3, #0x18]
    276e: b158         	cbz	r0, 0x2788 <TIMER1+0x74> @ imm = #0x16
    2770: 2803         	cmp	r0, #0x3
    2772: d10b         	bne	0x278c <TIMER1+0x78>    @ imm = #0x16
    2774: 695e         	ldr	r6, [r3, #0x14]
    2776: e0d6         	b	0x2926 <TIMER1+0x212>   @ imm = #0x1ac
    2778: f3bf 8f2f    	clrex
    277c: e008         	b	0x2790 <TIMER1+0x7c>    @ imm = #0x10
    277e: e9d3 0100    	ldrd	r0, r1, [r3]
    2782: e9c3 0102    	strd	r0, r1, [r3, #8]
    2786: e38b         	b	0x2ea0 <TIMER1+0x78c>   @ imm = #0x716
    2788: 691e         	ldr	r6, [r3, #0x10]
    278a: e0cb         	b	0x2924 <TIMER1+0x210>   @ imm = #0x196
    278c: f004 fa91    	bl	0x6cb2 <core::panicking::panic_const::panic_const_async_fn_resumed::h274f44943fe874ff> @ imm = #0x4522
    2790: f240 2050    	movw	r0, #0x250
    2794: f2c2 0000    	movt	r0, #0x2000
    2798: 6c05         	ldr	r5, [r0, #0x40]
    279a: f895 0098    	ldrb.w	r0, [r5, #0x98]
    279e: 2800         	cmp	r0, #0x0
    27a0: f000 808a    	beq.w	0x28b8 <TIMER1+0x1a4>   @ imm = #0x114
    27a4: f105 0099    	add.w	r0, r5, #0x99
    27a8: 2100         	movs	r1, #0x0
    27aa: e8d0 2f4f    	ldrexb	r2, [r0]
    27ae: 2a01         	cmp	r2, #0x1
    27b0: d11b         	bne	0x27ea <TIMER1+0xd6>    @ imm = #0x36
    27b2: e8c0 1f42    	strexb	r2, r1, [r0]
    27b6: 2a00         	cmp	r2, #0x0
    27b8: d1f7         	bne	0x27aa <TIMER1+0x96>    @ imm = #-0x12
    27ba: f640 505b    	movw	r0, #0xd5b
    27be: f3bf 8f5f    	dmb	sy
    27c2: f2c0 0000    	movt	r0, #0x0
    27c6: a90e         	add	r1, sp, #0x38
    27c8: 900f         	str	r0, [sp, #0x3c]
    27ca: f647 3090    	movw	r0, #0x7b90
    27ce: f2c0 0000    	movt	r0, #0x0
    27d2: f04f 0900    	mov.w	r9, #0x0
    27d6: 900e         	str	r0, [sp, #0x38]
    27d8: f895 0090    	ldrb.w	r0, [r5, #0x90]
    27dc: f8cd 9054    	str.w	r9, [sp, #0x54]
    27e0: 2800         	cmp	r0, #0x0
    27e2: e9cd 1113    	strd	r1, r1, [sp, #76]
    27e6: d13e         	bne	0x2866 <TIMER1+0x152>   @ imm = #0x7c
    27e8: e002         	b	0x27f0 <TIMER1+0xdc>    @ imm = #0x4
    27ea: f3bf 8f2f    	clrex
    27ee: e063         	b	0x28b8 <TIMER1+0x1a4>   @ imm = #0xc6
    27f0: f004 fdd6    	bl	0x73a0 <__basepri_r>    @ imm = #0x4bac
    27f4: 4604         	mov	r4, r0
    27f6: 2080         	movs	r0, #0x80
    27f8: f004 fdcf    	bl	0x739a <__basepri_max>  @ imm = #0x4b9e
    27fc: ed9f 0ad8    	vldr	s0, [pc, #864]          @ 0x2b60 <TIMER1+0x44c>
    2800: ed9b 1a00    	vldr	s2, [r11]
    2804: ee31 0a00    	vadd.f32	s0, s2, s0
    2808: ed9f 1ad6    	vldr	s2, [pc, #856]          @ 0x2b64 <TIMER1+0x450>
    280c: eeb4 0a41    	vcmp.f32	s0, s2
    2810: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    2814: bf98         	it	ls
    2816: eeb0 0a41    	vmovls.f32	s0, s2
    281a: ed8b 0a00    	vstr	s0, [r11]
    281e: f004 fb32    	bl	0x6e86 <_defmt_acquire> @ imm = #0x4664
    2822: f240 0009    	movw	r0, #0x9
    2826: f2c0 0000    	movt	r0, #0x0
    282a: f004 fab1    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x4562
    282e: f240 0004    	movw	r0, #0x4
    2832: 2102         	movs	r1, #0x2
    2834: f2c0 0000    	movt	r0, #0x0
    2838: f8ad 0020    	strh.w	r0, [sp, #0x20]
    283c: a808         	add	r0, sp, #0x20
    283e: f004 fbee    	bl	0x701e <_defmt_write>   @ imm = #0x47dc
    2842: f8db 0000    	ldr.w	r0, [r11]
    2846: 2104         	movs	r1, #0x4
    2848: 9008         	str	r0, [sp, #0x20]
    284a: a808         	add	r0, sp, #0x20
    284c: f004 fbe7    	bl	0x701e <_defmt_write>   @ imm = #0x47ce
    2850: f004 fb6e    	bl	0x6f30 <_defmt_release> @ imm = #0x46dc
    2854: 4620         	mov	r0, r4
    2856: f004 fda7    	bl	0x73a8 <__basepri_w>    @ imm = #0x4b4e
    285a: f44f 20a0    	mov.w	r0, #0x50000
    285e: f885 9088    	strb.w	r9, [r5, #0x88]
    2862: e9c5 0900    	strd	r0, r9, [r5]
    2866: a913         	add	r1, sp, #0x4c
    2868: 4628         	mov	r0, r5
    286a: f7fe fb1c    	bl	0xea6 <rtic_time::monotonic::timer_queue_based_monotonic::<impl rtic_time::Monotonic for T>::delay::{{closure}}::he80b7d3cf3920fa6> @ imm = #-0x19c8
    286e: b9e0         	cbnz	r0, 0x28aa <TIMER1+0x196> @ imm = #0x38
    2870: f895 0088    	ldrb.w	r0, [r5, #0x88]
    2874: 2803         	cmp	r0, #0x3
    2876: d1bb         	bne	0x27f0 <TIMER1+0xdc>    @ imm = #-0x8a
    2878: f895 0084    	ldrb.w	r0, [r5, #0x84]
    287c: 2803         	cmp	r0, #0x3
    287e: bf04         	itt	eq
    2880: f895 0079    	ldrbeq.w	r0, [r5, #0x79]
    2884: 2803         	cmpeq	r0, #0x3
    2886: d1b3         	bne	0x27f0 <TIMER1+0xdc>    @ imm = #-0x9a
    2888: e9d5 6018    	ldrd	r6, r0, [r5, #96]
    288c: 6804         	ldr	r4, [r0]
    288e: f004 fd7f    	bl	0x7390 <__primask_r>    @ imm = #0x4afe
    2892: 4680         	mov	r8, r0
    2894: f004 fd75    	bl	0x7382 <__cpsid>        @ imm = #0x4aea
    2898: f3bf 8f5f    	dmb	sy
    289c: 6830         	ldr	r0, [r6]
    289e: b1c0         	cbz	r0, 0x28d2 <TIMER1+0x1be> @ imm = #0x30
    28a0: 6981         	ldr	r1, [r0, #0x18]
    28a2: 4284         	cmp	r4, r0
    28a4: d10f         	bne	0x28c6 <TIMER1+0x1b2>   @ imm = #0x1e
    28a6: 6031         	str	r1, [r6]
    28a8: e013         	b	0x28d2 <TIMER1+0x1be>   @ imm = #0x26
    28aa: e9dd 100e    	ldrd	r1, r0, [sp, #56]
    28ae: 2203         	movs	r2, #0x3
    28b0: f885 2090    	strb.w	r2, [r5, #0x90]
    28b4: 68c9         	ldr	r1, [r1, #0xc]
    28b6: 4788         	blx	r1
    28b8: 4650         	mov	r0, r10
    28ba: f004 fd75    	bl	0x73a8 <__basepri_w>    @ imm = #0x4aea
    28be: b01b         	add	sp, #0x6c
    28c0: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    28c4: bdf0         	pop	{r4, r5, r6, r7, pc}
    28c6: b121         	cbz	r1, 0x28d2 <TIMER1+0x1be> @ imm = #0x8
    28c8: 428c         	cmp	r4, r1
    28ca: d112         	bne	0x28f2 <TIMER1+0x1de>   @ imm = #0x24
    28cc: 460a         	mov	r2, r1
    28ce: 6991         	ldr	r1, [r2, #0x18]
    28d0: 6181         	str	r1, [r0, #0x18]
    28d2: ea5f 70c8    	lsls.w	r0, r8, #0x1f
    28d6: bf08         	it	eq
    28d8: f004 fd55    	bleq	0x7386 <__cpsie>        @ imm = #0x4aaa
    28dc: 6a28         	ldr	r0, [r5, #0x20]
    28de: 2100         	movs	r1, #0x0
    28e0: f885 1078    	strb.w	r1, [r5, #0x78]
    28e4: 2800         	cmp	r0, #0x0
    28e6: f43f af83    	beq.w	0x27f0 <TIMER1+0xdc>    @ imm = #-0xfa
    28ea: 68c1         	ldr	r1, [r0, #0xc]
    28ec: 6a68         	ldr	r0, [r5, #0x24]
    28ee: 4788         	blx	r1
    28f0: e77e         	b	0x27f0 <TIMER1+0xdc>    @ imm = #-0x104
    28f2: 6988         	ldr	r0, [r1, #0x18]
    28f4: 2800         	cmp	r0, #0x0
    28f6: d0ec         	beq	0x28d2 <TIMER1+0x1be>   @ imm = #-0x28
    28f8: 4284         	cmp	r4, r0
    28fa: d010         	beq	0x291e <TIMER1+0x20a>   @ imm = #0x20
    28fc: 6981         	ldr	r1, [r0, #0x18]
    28fe: 2900         	cmp	r1, #0x0
    2900: d0e7         	beq	0x28d2 <TIMER1+0x1be>   @ imm = #-0x32
    2902: 428c         	cmp	r4, r1
    2904: d0e2         	beq	0x28cc <TIMER1+0x1b8>   @ imm = #-0x3c
    2906: 6988         	ldr	r0, [r1, #0x18]
    2908: 2800         	cmp	r0, #0x0
    290a: d0e2         	beq	0x28d2 <TIMER1+0x1be>   @ imm = #-0x3c
    290c: 4284         	cmp	r4, r0
    290e: d006         	beq	0x291e <TIMER1+0x20a>   @ imm = #0xc
    2910: 6981         	ldr	r1, [r0, #0x18]
    2912: 2900         	cmp	r1, #0x0
    2914: d0dd         	beq	0x28d2 <TIMER1+0x1be>   @ imm = #-0x46
    2916: 428c         	cmp	r4, r1
    2918: 460a         	mov	r2, r1
    291a: d1ea         	bne	0x28f2 <TIMER1+0x1de>   @ imm = #-0x2c
    291c: e7d7         	b	0x28ce <TIMER1+0x1ba>   @ imm = #-0x52
    291e: 4602         	mov	r2, r0
    2920: 4608         	mov	r0, r1
    2922: e7d4         	b	0x28ce <TIMER1+0x1ba>   @ imm = #-0x58
    2924: 615e         	str	r6, [r3, #0x14]
    2926: 6834         	ldr	r4, [r6]
    2928: f004 fd32    	bl	0x7390 <__primask_r>    @ imm = #0x4a64
    292c: 4680         	mov	r8, r0
    292e: f004 fd28    	bl	0x7382 <__cpsid>        @ imm = #0x4a50
    2932: 6825         	ldr	r5, [r4]
    2934: b1cd         	cbz	r5, 0x296a <TIMER1+0x256> @ imm = #0x32
    2936: f8d4 9004    	ldr.w	r9, [r4, #0x4]
    293a: f647 3090    	movw	r0, #0x7b90
    293e: f2c0 0000    	movt	r0, #0x0
    2942: 4285         	cmp	r5, r0
    2944: bf02         	ittt	eq
    2946: f640 507f    	movweq	r0, #0xd7f
    294a: f2c0 0000    	movteq	r0, #0x0
    294e: 4581         	cmpeq	r9, r0
    2950: d013         	beq	0x297a <TIMER1+0x266>   @ imm = #0x26
    2952: f640 507f    	movw	r0, #0xd7f
    2956: f2c0 0000    	movt	r0, #0x0
    295a: f004 fd0a    	bl	0x7372 <rtic::export::executor::waker_clone::hb3709afbf1c107f0> @ imm = #0x4a14
    295e: e9c4 0100    	strd	r0, r1, [r4]
    2962: 4648         	mov	r0, r9
    2964: 6869         	ldr	r1, [r5, #0x4]
    2966: 4788         	blx	r1
    2968: e007         	b	0x297a <TIMER1+0x266>   @ imm = #0xe
    296a: f640 507f    	movw	r0, #0xd7f
    296e: f2c0 0000    	movt	r0, #0x0
    2972: f004 fcfe    	bl	0x7372 <rtic::export::executor::waker_clone::hb3709afbf1c107f0> @ imm = #0x49fc
    2976: e9c4 0100    	strd	r0, r1, [r4]
    297a: ea5f 70c8    	lsls.w	r0, r8, #0x1f
    297e: bf08         	it	eq
    2980: f004 fd01    	bleq	0x7386 <__cpsie>        @ imm = #0x4a02
    2984: f8d6 8000    	ldr.w	r8, [r6]
    2988: f004 fd02    	bl	0x7390 <__primask_r>    @ imm = #0x4a04
    298c: 4604         	mov	r4, r0
    298e: f004 fcf8    	bl	0x7382 <__cpsid>        @ imm = #0x49f0
    2992: e9d8 013b    	ldrd	r0, r1, [r8, #236]
    2996: 4288         	cmp	r0, r1
    2998: bf04         	itt	eq
    299a: f898 10fe    	ldrbeq.w	r1, [r8, #0xfe]
    299e: ea5f 71c1    	lslseq.w	r1, r1, #0x1f
    29a2: f000 8717    	beq.w	0x37d4 <TIMER1+0x10c0>  @ imm = #0xe2e
    29a6: 2100         	movs	r1, #0x0
    29a8: f888 10fe    	strb.w	r1, [r8, #0xfe]
    29ac: 1c41         	adds	r1, r0, #0x1
    29ae: f1b1 020a    	subs.w	r2, r1, #0xa
    29b2: 4440         	add	r0, r8
    29b4: bf18         	it	ne
    29b6: 460a         	movne	r2, r1
    29b8: f8c8 20ec    	str.w	r2, [r8, #0xec]
    29bc: f890 50f4    	ldrb.w	r5, [r0, #0xf4]
    29c0: 07e0         	lsls	r0, r4, #0x1f
    29c2: bf08         	it	eq
    29c4: f004 fcdf    	bleq	0x7386 <__cpsie>        @ imm = #0x49be
    29c8: eb05 0085    	add.w	r0, r5, r5, lsl #2
    29cc: eb08 0080    	add.w	r0, r8, r0, lsl #2
    29d0: e9d0 2104    	ldrd	r2, r1, [r0, #16]
    29d4: e9cd 2105    	strd	r2, r1, [sp, #20]
    29d8: f8d0 100b    	ldr.w	r1, [r0, #0xb]
    29dc: 9103         	str	r1, [sp, #0xc]
    29de: 7a81         	ldrb	r1, [r0, #0xa]
    29e0: 9104         	str	r1, [sp, #0x10]
    29e2: f8b0 b008    	ldrh.w	r11, [r0, #0x8]
    29e6: f004 fcd3    	bl	0x7390 <__primask_r>    @ imm = #0x49a6
    29ea: 4604         	mov	r4, r0
    29ec: f004 fcc9    	bl	0x7382 <__cpsid>        @ imm = #0x4992
    29f0: f8d8 00dc    	ldr.w	r0, [r8, #0xdc]
    29f4: 4440         	add	r0, r8
    29f6: f880 50e0    	strb.w	r5, [r0, #0xe0]
    29fa: e9d8 0136    	ldrd	r0, r1, [r8, #216]
    29fe: 3101         	adds	r1, #0x1
    2a00: f1b1 020a    	subs.w	r2, r1, #0xa
    2a04: bf18         	it	ne
    2a06: 460a         	movne	r2, r1
    2a08: 4290         	cmp	r0, r2
    2a0a: f8c8 20dc    	str.w	r2, [r8, #0xdc]
    2a0e: bf04         	itt	eq
    2a10: 2001         	moveq	r0, #0x1
    2a12: f888 00ea    	strbeq.w	r0, [r8, #0xea]
    2a16: 07e0         	lsls	r0, r4, #0x1f
    2a18: bf08         	it	eq
    2a1a: f004 fcb4    	bleq	0x7386 <__cpsie>        @ imm = #0x4968
    2a1e: f3bf 8f5f    	dmb	sy
    2a22: f004 fcb5    	bl	0x7390 <__primask_r>    @ imm = #0x496a
    2a26: 4604         	mov	r4, r0
    2a28: f004 fcab    	bl	0x7382 <__cpsid>        @ imm = #0x4956
    2a2c: f3bf 8f5f    	dmb	sy
    2a30: f8d8 90d0    	ldr.w	r9, [r8, #0xd0]
    2a34: f1b9 0f00    	cmp.w	r9, #0x0
    2a38: d029         	beq	0x2a8e <TIMER1+0x37a>   @ imm = #0x52
    2a3a: f8d9 0008    	ldr.w	r0, [r9, #0x8]
    2a3e: f8c8 00d0    	str.w	r0, [r8, #0xd0]
    2a42: e9d9 1000    	ldrd	r1, r0, [r9]
    2a46: 6809         	ldr	r1, [r1]
    2a48: 4788         	blx	r1
    2a4a: 4606         	mov	r6, r0
    2a4c: f8d8 00d4    	ldr.w	r0, [r8, #0xd4]
    2a50: 460d         	mov	r5, r1
    2a52: 4581         	cmp	r9, r0
    2a54: bf04         	itt	eq
    2a56: 2000         	moveq	r0, #0x0
    2a58: f8c8 00d4    	streq.w	r0, [r8, #0xd4]
    2a5c: f8d9 0008    	ldr.w	r0, [r9, #0x8]
    2a60: 2800         	cmp	r0, #0x0
    2a62: bf1c         	itt	ne
    2a64: 2100         	movne	r1, #0x0
    2a66: 60c1         	strne	r1, [r0, #0xc]
    2a68: 2000         	movs	r0, #0x0
    2a6a: f8c9 0008    	str.w	r0, [r9, #0x8]
    2a6e: f8c9 000c    	str.w	r0, [r9, #0xc]
    2a72: 2001         	movs	r0, #0x1
    2a74: f889 0010    	strb.w	r0, [r9, #0x10]
    2a78: 07e0         	lsls	r0, r4, #0x1f
    2a7a: 9c07         	ldr	r4, [sp, #0x1c]
    2a7c: bf08         	it	eq
    2a7e: f004 fc82    	bleq	0x7386 <__cpsie>        @ imm = #0x4904
    2a82: 2e00         	cmp	r6, #0x0
    2a84: bf1e         	ittt	ne
    2a86: 6871         	ldrne	r1, [r6, #0x4]
    2a88: 4628         	movne	r0, r5
    2a8a: 4788         	blxne	r1
    2a8c: e004         	b	0x2a98 <TIMER1+0x384>   @ imm = #0x8
    2a8e: 07e0         	lsls	r0, r4, #0x1f
    2a90: 9c07         	ldr	r4, [sp, #0x1c]
    2a92: d101         	bne	0x2a98 <TIMER1+0x384>   @ imm = #0x2
    2a94: f004 fc77    	bl	0x7386 <__cpsie>        @ imm = #0x48ee
    2a98: f1bb 0f04    	cmp.w	r11, #0x4
    2a9c: f000 86ab    	beq.w	0x37f6 <TIMER1+0x10e2>  @ imm = #0xd56
    2aa0: f1bb 0f03    	cmp.w	r11, #0x3
    2aa4: f000 86ab    	beq.w	0x37fe <TIMER1+0x10ea>  @ imm = #0xd56
    2aa8: 2001         	movs	r0, #0x1
    2aaa: f1bb 0f02    	cmp.w	r11, #0x2
    2aae: 7620         	strb	r0, [r4, #0x18]
    2ab0: d10d         	bne	0x2ace <TIMER1+0x3ba>   @ imm = #0x1a
    2ab2: f640 0108    	movw	r1, #0x808
    2ab6: f243 521c    	movw	r2, #0x351c
    2aba: f44f 7080    	mov.w	r0, #0x100
    2abe: f2c5 0100    	movt	r1, #0x5000
    2ac2: 6048         	str	r0, [r1, #0x4]
    2ac4: f2c4 0200    	movt	r2, #0x4000
    2ac8: 2003         	movs	r0, #0x3
    2aca: 6010         	str	r0, [r2]
    2acc: e036         	b	0x2b3c <TIMER1+0x428>   @ imm = #0x6c
    2ace: f8d4 c008    	ldr.w	r12, [r4, #0x8]
    2ad2: 2340         	movs	r3, #0x40
    2ad4: f81c 2f12    	ldrb	r2, [r12, #18]!
    2ad8: f89c 1001    	ldrb.w	r1, [r12, #0x1]
    2adc: 07c9         	lsls	r1, r1, #0x1f
    2ade: f04f 0140    	mov.w	r1, #0x40
    2ae2: bf18         	it	ne
    2ae4: 2150         	movne	r1, #0x50
    2ae6: 2a00         	cmp	r2, #0x0
    2ae8: bf08         	it	eq
    2aea: 2130         	moveq	r1, #0x30
    2aec: e9dd 2003    	ldrd	r2, r0, [sp, #12]
    2af0: 2930         	cmp	r1, #0x30
    2af2: ea40 2202    	orr.w	r2, r0, r2, lsl #8
    2af6: d003         	beq	0x2b00 <TIMER1+0x3ec>   @ imm = #0x6
    2af8: 2940         	cmp	r1, #0x40
    2afa: bf14         	ite	ne
    2afc: 2344         	movne	r3, #0x44
    2afe: 2342         	moveq	r3, #0x42
    2b00: f640 0508    	movw	r5, #0x808
    2b04: f44f 7680    	mov.w	r6, #0x100
    2b08: f2c5 0500    	movt	r5, #0x5000
    2b0c: f3c2 02c7    	ubfx	r2, r2, #0x3, #0x8
    2b10: 606e         	str	r6, [r5, #0x4]
    2b12: f243 561c    	movw	r6, #0x351c
    2b16: f2c4 0600    	movt	r6, #0x4000
    2b1a: 6033         	str	r3, [r6]
    2b1c: 0143         	lsls	r3, r0, #0x5
    2b1e: f8da 6000    	ldr.w	r6, [r10]
    2b22: b2db         	uxtb	r3, r3
    2b24: e02d         	b	0x2b82 <TIMER1+0x46e>   @ imm = #0x5a
    2b26: bf10         	yield
    2b28: f8da 0000    	ldr.w	r0, [r10]
    2b2c: 2800         	cmp	r0, #0x0
    2b2e: bf02         	ittt	eq
    2b30: bf10         	yieldeq
    2b32: f8da 0000    	ldreq.w	r0, [r10]
    2b36: 2800         	cmpeq	r0, #0x0
    2b38: d109         	bne	0x2b4e <TIMER1+0x43a>   @ imm = #0x12
    2b3a: bf10         	yield
    2b3c: f8da 0000    	ldr.w	r0, [r10]
    2b40: 2800         	cmp	r0, #0x0
    2b42: bf02         	ittt	eq
    2b44: bf10         	yieldeq
    2b46: f8da 0000    	ldreq.w	r0, [r10]
    2b4a: 2800         	cmpeq	r0, #0x0
    2b4c: d0eb         	beq	0x2b26 <TIMER1+0x412>   @ imm = #-0x2a
    2b4e: f8da 1410    	ldr.w	r1, [r10, #0x410]
    2b52: 2000         	movs	r0, #0x0
    2b54: 210e         	movs	r1, #0xe
    2b56: f8ca 0000    	str.w	r0, [r10]
    2b5a: 6011         	str	r1, [r2]
    2b5c: e041         	b	0x2be2 <TIMER1+0x4ce>   @ imm = #0x82
    2b5e: bf00         	nop
    2b60: cd cc cc bd  	.word	0xbdcccccd
    2b64: cd cc cc 3d  	.word	0x3dcccccd
    2b68: bf10         	yield
    2b6a: f8da 6000    	ldr.w	r6, [r10]
    2b6e: 2e00         	cmp	r6, #0x0
    2b70: bf02         	ittt	eq
    2b72: bf10         	yieldeq
    2b74: f8da 6000    	ldreq.w	r6, [r10]
    2b78: 2e00         	cmpeq	r6, #0x0
    2b7a: d109         	bne	0x2b90 <TIMER1+0x47c>   @ imm = #0x12
    2b7c: bf10         	yield
    2b7e: f8da 6000    	ldr.w	r6, [r10]
    2b82: 2e00         	cmp	r6, #0x0
    2b84: bf02         	ittt	eq
    2b86: bf10         	yieldeq
    2b88: f8da 6000    	ldreq.w	r6, [r10]
    2b8c: 2e00         	cmpeq	r6, #0x0
    2b8e: d0eb         	beq	0x2b68 <TIMER1+0x454>   @ imm = #-0x2a
    2b90: f243 541c    	movw	r4, #0x351c
    2b94: f8da 6410    	ldr.w	r6, [r10, #0x410]
    2b98: 2600         	movs	r6, #0x0
    2b9a: f2c4 0400    	movt	r4, #0x4000
    2b9e: f8ca 6000    	str.w	r6, [r10]
    2ba2: ea5f 75cb    	lsls.w	r5, r11, #0x1f
    2ba6: bf18         	it	ne
    2ba8: 2200         	movne	r2, #0x0
    2baa: 6022         	str	r2, [r4]
    2bac: 9805         	ldr	r0, [sp, #0x14]
    2bae: ea4f 6910    	lsr.w	r9, r0, #0x18
    2bb2: ea4f 4b10    	lsr.w	r11, r0, #0x10
    2bb6: 0a02         	lsrs	r2, r0, #0x8
    2bb8: 9806         	ldr	r0, [sp, #0x18]
    2bba: 0e04         	lsrs	r4, r0, #0x18
    2bbc: ea4f 4e10    	lsr.w	lr, r0, #0x10
    2bc0: 9404         	str	r4, [sp, #0x10]
    2bc2: ea4f 2810    	lsr.w	r8, r0, #0x8
    2bc6: f8da 4000    	ldr.w	r4, [r10]
    2bca: e026         	b	0x2c1a <TIMER1+0x506>   @ imm = #0x4c
    2bcc: bf10         	yield
    2bce: f8da 1000    	ldr.w	r1, [r10]
    2bd2: 2900         	cmp	r1, #0x0
    2bd4: bf02         	ittt	eq
    2bd6: bf10         	yieldeq
    2bd8: f8da 1000    	ldreq.w	r1, [r10]
    2bdc: 2900         	cmpeq	r1, #0x0
    2bde: d109         	bne	0x2bf4 <TIMER1+0x4e0>   @ imm = #0x12
    2be0: bf10         	yield
    2be2: f8da 1000    	ldr.w	r1, [r10]
    2be6: 2900         	cmp	r1, #0x0
    2be8: bf02         	ittt	eq
    2bea: bf10         	yieldeq
    2bec: f8da 1000    	ldreq.w	r1, [r10]
    2bf0: 2900         	cmpeq	r1, #0x0
    2bf2: d0eb         	beq	0x2bcc <TIMER1+0x4b8>   @ imm = #-0x2a
    2bf4: f8da 1410    	ldr.w	r1, [r10, #0x410]
    2bf8: f8ca 0000    	str.w	r0, [r10]
    2bfc: 6010         	str	r0, [r2]
    2bfe: e02b         	b	0x2c58 <TIMER1+0x544>   @ imm = #0x56
    2c00: bf10         	yield
    2c02: f8da 4000    	ldr.w	r4, [r10]
    2c06: 2c00         	cmp	r4, #0x0
    2c08: bf02         	ittt	eq
    2c0a: bf10         	yieldeq
    2c0c: f8da 4000    	ldreq.w	r4, [r10]
    2c10: 2c00         	cmpeq	r4, #0x0
    2c12: d109         	bne	0x2c28 <TIMER1+0x514>   @ imm = #0x12
    2c14: bf10         	yield
    2c16: f8da 4000    	ldr.w	r4, [r10]
    2c1a: 2c00         	cmp	r4, #0x0
    2c1c: bf02         	ittt	eq
    2c1e: bf10         	yieldeq
    2c20: f8da 4000    	ldreq.w	r4, [r10]
    2c24: 2c00         	cmpeq	r4, #0x0
    2c26: d0eb         	beq	0x2c00 <TIMER1+0x4ec>   @ imm = #-0x2a
    2c28: 2d00         	cmp	r5, #0x0
    2c2a: f243 551c    	movw	r5, #0x351c
    2c2e: f2c4 0500    	movt	r5, #0x4000
    2c32: f8da 4410    	ldr.w	r4, [r10, #0x410]
    2c36: f8ca 6000    	str.w	r6, [r10]
    2c3a: bf18         	it	ne
    2c3c: 2300         	movne	r3, #0x0
    2c3e: 602b         	str	r3, [r5]
    2c40: e05b         	b	0x2cfa <TIMER1+0x5e6>   @ imm = #0xb6
    2c42: bf10         	yield
    2c44: f8da 0000    	ldr.w	r0, [r10]
    2c48: 2800         	cmp	r0, #0x0
    2c4a: bf02         	ittt	eq
    2c4c: bf10         	yieldeq
    2c4e: f8da 0000    	ldreq.w	r0, [r10]
    2c52: 2800         	cmpeq	r0, #0x0
    2c54: d109         	bne	0x2c6a <TIMER1+0x556>   @ imm = #0x12
    2c56: bf10         	yield
    2c58: f8da 0000    	ldr.w	r0, [r10]
    2c5c: 2800         	cmp	r0, #0x0
    2c5e: bf02         	ittt	eq
    2c60: bf10         	yieldeq
    2c62: f8da 0000    	ldreq.w	r0, [r10]
    2c66: 2800         	cmpeq	r0, #0x0
    2c68: d0eb         	beq	0x2c42 <TIMER1+0x52e>   @ imm = #-0x2a
    2c6a: f640 0808    	movw	r8, #0x808
    2c6e: f04f 0b00    	mov.w	r11, #0x0
    2c72: f44f 7680    	mov.w	r6, #0x100
    2c76: f2c5 0800    	movt	r8, #0x5000
    2c7a: f8da 5410    	ldr.w	r5, [r10, #0x410]
    2c7e: f8ca b000    	str.w	r11, [r10]
    2c82: f8c8 6000    	str.w	r6, [r8]
    2c86: f004 f8fe    	bl	0x6e86 <_defmt_acquire> @ imm = #0x41fc
    2c8a: f240 0017    	movw	r0, #0x17
    2c8e: f2c0 0000    	movt	r0, #0x0
    2c92: f004 f87d    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x40fa
    2c96: f240 000b    	movw	r0, #0xb
    2c9a: 2102         	movs	r1, #0x2
    2c9c: f2c0 0000    	movt	r0, #0x0
    2ca0: f8ad 004c    	strh.w	r0, [sp, #0x4c]
    2ca4: a813         	add	r0, sp, #0x4c
    2ca6: f004 f9ba    	bl	0x701e <_defmt_write>   @ imm = #0x4374
    2caa: f647 30f0    	movw	r0, #0x7bf0
    2cae: f3c5 0542    	ubfx	r5, r5, #0x1, #0x3
    2cb2: f2c0 0000    	movt	r0, #0x0
    2cb6: f10d 094c    	add.w	r9, sp, #0x4c
    2cba: f850 0025    	ldr.w	r0, [r0, r5, lsl #2]
    2cbe: 2101         	movs	r1, #0x1
    2cc0: 7800         	ldrb	r0, [r0]
    2cc2: f88d 004c    	strb.w	r0, [sp, #0x4c]
    2cc6: 4648         	mov	r0, r9
    2cc8: f004 f9a9    	bl	0x701e <_defmt_write>   @ imm = #0x4352
    2ccc: f004 f930    	bl	0x6f30 <_defmt_release> @ imm = #0x4260
    2cd0: 68a4         	ldr	r4, [r4, #0x8]
    2cd2: f243 511c    	movw	r1, #0x351c
    2cd6: f2c4 0100    	movt	r1, #0x4000
    2cda: 2003         	movs	r0, #0x3
    2cdc: f8c8 6004    	str.w	r6, [r8, #0x4]
    2ce0: 6008         	str	r0, [r1]
    2ce2: e025         	b	0x2d30 <TIMER1+0x61c>   @ imm = #0x4a
    2ce4: bf10         	yield
    2ce6: f8da 3000    	ldr.w	r3, [r10]
    2cea: 2b00         	cmp	r3, #0x0
    2cec: bf02         	ittt	eq
    2cee: bf10         	yieldeq
    2cf0: f8da 3000    	ldreq.w	r3, [r10]
    2cf4: 2b00         	cmpeq	r3, #0x0
    2cf6: d109         	bne	0x2d0c <TIMER1+0x5f8>   @ imm = #0x12
    2cf8: bf10         	yield
    2cfa: f8da 3000    	ldr.w	r3, [r10]
    2cfe: 2b00         	cmp	r3, #0x0
    2d00: bf02         	ittt	eq
    2d02: bf10         	yieldeq
    2d04: f8da 3000    	ldreq.w	r3, [r10]
    2d08: 2b00         	cmpeq	r3, #0x0
    2d0a: d0eb         	beq	0x2ce4 <TIMER1+0x5d0>   @ imm = #-0x2a
    2d0c: f8da 3410    	ldr.w	r3, [r10, #0x410]
    2d10: 2300         	movs	r3, #0x0
    2d12: f8ca 3000    	str.w	r3, [r10]
    2d16: 602b         	str	r3, [r5]
    2d18: e025         	b	0x2d66 <TIMER1+0x652>   @ imm = #0x4a
    2d1a: bf10         	yield
    2d1c: f8da 0000    	ldr.w	r0, [r10]
    2d20: 2800         	cmp	r0, #0x0
    2d22: bf02         	ittt	eq
    2d24: bf10         	yieldeq
    2d26: f8da 0000    	ldreq.w	r0, [r10]
    2d2a: 2800         	cmpeq	r0, #0x0
    2d2c: d109         	bne	0x2d42 <TIMER1+0x62e>   @ imm = #0x12
    2d2e: bf10         	yield
    2d30: f8da 0000    	ldr.w	r0, [r10]
    2d34: 2800         	cmp	r0, #0x0
    2d36: bf02         	ittt	eq
    2d38: bf10         	yieldeq
    2d3a: f8da 0000    	ldreq.w	r0, [r10]
    2d3e: 2800         	cmpeq	r0, #0x0
    2d40: d0eb         	beq	0x2d1a <TIMER1+0x606>   @ imm = #-0x2a
    2d42: f8da 0410    	ldr.w	r0, [r10, #0x410]
    2d46: 202c         	movs	r0, #0x2c
    2d48: f8ca b000    	str.w	r11, [r10]
    2d4c: 6008         	str	r0, [r1]
    2d4e: e024         	b	0x2d9a <TIMER1+0x686>   @ imm = #0x48
    2d50: bf10         	yield
    2d52: f8da 4000    	ldr.w	r4, [r10]
    2d56: 2c00         	cmp	r4, #0x0
    2d58: bf02         	ittt	eq
    2d5a: bf10         	yieldeq
    2d5c: f8da 4000    	ldreq.w	r4, [r10]
    2d60: 2c00         	cmpeq	r4, #0x0
    2d62: d109         	bne	0x2d78 <TIMER1+0x664>   @ imm = #0x12
    2d64: bf10         	yield
    2d66: f8da 4000    	ldr.w	r4, [r10]
    2d6a: 2c00         	cmp	r4, #0x0
    2d6c: bf02         	ittt	eq
    2d6e: bf10         	yieldeq
    2d70: f8da 4000    	ldreq.w	r4, [r10]
    2d74: 2c00         	cmpeq	r4, #0x0
    2d76: d0eb         	beq	0x2d50 <TIMER1+0x63c>   @ imm = #-0x2a
    2d78: f8da 4410    	ldr.w	r4, [r10, #0x410]
    2d7c: f8ca 3000    	str.w	r3, [r10]
    2d80: 602b         	str	r3, [r5]
    2d82: e025         	b	0x2dd0 <TIMER1+0x6bc>   @ imm = #0x4a
    2d84: bf10         	yield
    2d86: f8da 0000    	ldr.w	r0, [r10]
    2d8a: 2800         	cmp	r0, #0x0
    2d8c: bf02         	ittt	eq
    2d8e: bf10         	yieldeq
    2d90: f8da 0000    	ldreq.w	r0, [r10]
    2d94: 2800         	cmpeq	r0, #0x0
    2d96: d109         	bne	0x2dac <TIMER1+0x698>   @ imm = #0x12
    2d98: bf10         	yield
    2d9a: f8da 0000    	ldr.w	r0, [r10]
    2d9e: 2800         	cmp	r0, #0x0
    2da0: bf02         	ittt	eq
    2da2: bf10         	yieldeq
    2da4: f8da 0000    	ldreq.w	r0, [r10]
    2da8: 2800         	cmpeq	r0, #0x0
    2daa: d0eb         	beq	0x2d84 <TIMER1+0x670>   @ imm = #-0x2a
    2dac: f8da 0410    	ldr.w	r0, [r10, #0x410]
    2db0: 2000         	movs	r0, #0x0
    2db2: f8ca 0000    	str.w	r0, [r10]
    2db6: 6008         	str	r0, [r1]
    2db8: e026         	b	0x2e08 <TIMER1+0x6f4>   @ imm = #0x4c
    2dba: bf10         	yield
    2dbc: f8da 3000    	ldr.w	r3, [r10]
    2dc0: 2b00         	cmp	r3, #0x0
    2dc2: bf02         	ittt	eq
    2dc4: bf10         	yieldeq
    2dc6: f8da 3000    	ldreq.w	r3, [r10]
    2dca: 2b00         	cmpeq	r3, #0x0
    2dcc: d109         	bne	0x2de2 <TIMER1+0x6ce>   @ imm = #0x12
    2dce: bf10         	yield
    2dd0: f8da 3000    	ldr.w	r3, [r10]
    2dd4: 2b00         	cmp	r3, #0x0
    2dd6: bf02         	ittt	eq
    2dd8: bf10         	yieldeq
    2dda: f8da 3000    	ldreq.w	r3, [r10]
    2dde: 2b00         	cmpeq	r3, #0x0
    2de0: d0eb         	beq	0x2dba <TIMER1+0x6a6>   @ imm = #-0x2a
    2de2: f8da 4410    	ldr.w	r4, [r10, #0x410]
    2de6: 2300         	movs	r3, #0x0
    2de8: 2408         	movs	r4, #0x8
    2dea: f8ca 3000    	str.w	r3, [r10]
    2dee: 602c         	str	r4, [r5]
    2df0: e07f         	b	0x2ef2 <TIMER1+0x7de>   @ imm = #0xfe
    2df2: bf10         	yield
    2df4: f8da 1000    	ldr.w	r1, [r10]
    2df8: 2900         	cmp	r1, #0x0
    2dfa: bf02         	ittt	eq
    2dfc: bf10         	yieldeq
    2dfe: f8da 1000    	ldreq.w	r1, [r10]
    2e02: 2900         	cmpeq	r1, #0x0
    2e04: d109         	bne	0x2e1a <TIMER1+0x706>   @ imm = #0x12
    2e06: bf10         	yield
    2e08: f8da 1000    	ldr.w	r1, [r10]
    2e0c: 2900         	cmp	r1, #0x0
    2e0e: bf02         	ittt	eq
    2e10: bf10         	yieldeq
    2e12: f8da 1000    	ldreq.w	r1, [r10]
    2e16: 2900         	cmpeq	r1, #0x0
    2e18: d0eb         	beq	0x2df2 <TIMER1+0x6de>   @ imm = #-0x2a
    2e1a: f8da 6410    	ldr.w	r6, [r10, #0x410]
    2e1e: f640 0108    	movw	r1, #0x808
    2e22: f8ca 0000    	str.w	r0, [r10]
    2e26: f44f 7080    	mov.w	r0, #0x100
    2e2a: f2c5 0100    	movt	r1, #0x5000
    2e2e: 6008         	str	r0, [r1]
    2e30: b270         	sxtb	r0, r6
    2e32: f1b0 3fff    	cmp.w	r0, #0xffffffff
    2e36: dc0d         	bgt	0x2e54 <TIMER1+0x740>   @ imm = #0x1a
    2e38: f004 f825    	bl	0x6e86 <_defmt_acquire> @ imm = #0x404a
    2e3c: f240 0021    	movw	r0, #0x21
    2e40: f2c0 0000    	movt	r0, #0x0
    2e44: f003 ffa4    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3f48
    2e48: f004 f872    	bl	0x6f30 <_defmt_release> @ imm = #0x40e4
    2e4c: 4620         	mov	r0, r4
    2e4e: 2107         	movs	r1, #0x7
    2e50: f7fd fc6a    	bl	0x728 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400> @ imm = #-0x272c
    2e54: f004 f817    	bl	0x6e86 <_defmt_acquire> @ imm = #0x402e
    2e58: f240 0022    	movw	r0, #0x22
    2e5c: f2c0 0000    	movt	r0, #0x0
    2e60: f003 ff96    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3f2c
    2e64: f240 0001    	movw	r0, #0x1
    2e68: 2102         	movs	r1, #0x2
    2e6a: f2c0 0000    	movt	r0, #0x0
    2e6e: f8ad 0020    	strh.w	r0, [sp, #0x20]
    2e72: a808         	add	r0, sp, #0x20
    2e74: f004 f8d3    	bl	0x701e <_defmt_write>   @ imm = #0x41a6
    2e78: f10d 0b20    	add.w	r11, sp, #0x20
    2e7c: 2101         	movs	r1, #0x1
    2e7e: f88d 6020    	strb.w	r6, [sp, #0x20]
    2e82: 4658         	mov	r0, r11
    2e84: f004 f8cb    	bl	0x701e <_defmt_write>   @ imm = #0x4196
    2e88: f004 f852    	bl	0x6f30 <_defmt_release> @ imm = #0x40a4
    2e8c: e8df f015    	tbh	[pc, r5, lsl #1]
    2e90: 08 00 0e 00  	.word	0x000e0008
    2e94: 97 01 6c 01  	.word	0x016c0197
    2e98: 8a 01 5f 01  	.word	0x015f018a
    2e9c: a2 01 cd 01  	.word	0x01cd01a2
    2ea0: 9b07         	ldr	r3, [sp, #0x1c]
    2ea2: 2000         	movs	r0, #0x0
    2ea4: 68de         	ldr	r6, [r3, #0xc]
    2ea6: 7618         	strb	r0, [r3, #0x18]
    2ea8: 611e         	str	r6, [r3, #0x10]
    2eaa: e53b         	b	0x2924 <TIMER1+0x210>   @ imm = #-0x58a
    2eac: f003 ffeb    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3fd6
    2eb0: f240 0023    	movw	r0, #0x23
    2eb4: f2c0 0000    	movt	r0, #0x0
    2eb8: f003 ff6a    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3ed4
    2ebc: f004 f838    	bl	0x6f30 <_defmt_release> @ imm = #0x4070
    2ec0: f640 0108    	movw	r1, #0x808
    2ec4: f243 531c    	movw	r3, #0x351c
    2ec8: f44f 7080    	mov.w	r0, #0x100
    2ecc: f2c5 0100    	movt	r1, #0x5000
    2ed0: 6048         	str	r0, [r1, #0x4]
    2ed2: f2c4 0300    	movt	r3, #0x4000
    2ed6: 2003         	movs	r0, #0x3
    2ed8: 6018         	str	r0, [r3]
    2eda: e373         	b	0x35c4 <TIMER1+0xeb0>   @ imm = #0x6e6
    2edc: bf10         	yield
    2ede: f8da 4000    	ldr.w	r4, [r10]
    2ee2: 2c00         	cmp	r4, #0x0
    2ee4: bf02         	ittt	eq
    2ee6: bf10         	yieldeq
    2ee8: f8da 4000    	ldreq.w	r4, [r10]
    2eec: 2c00         	cmpeq	r4, #0x0
    2eee: d109         	bne	0x2f04 <TIMER1+0x7f0>   @ imm = #0x12
    2ef0: bf10         	yield
    2ef2: f8da 4000    	ldr.w	r4, [r10]
    2ef6: 2c00         	cmp	r4, #0x0
    2ef8: bf02         	ittt	eq
    2efa: bf10         	yieldeq
    2efc: f8da 4000    	ldreq.w	r4, [r10]
    2f00: 2c00         	cmpeq	r4, #0x0
    2f02: d0eb         	beq	0x2edc <TIMER1+0x7c8>   @ imm = #-0x2a
    2f04: f8da 4410    	ldr.w	r4, [r10, #0x410]
    2f08: f8ca 3000    	str.w	r3, [r10]
    2f0c: 9805         	ldr	r0, [sp, #0x14]
    2f0e: b2c3         	uxtb	r3, r0
    2f10: 602b         	str	r3, [r5]
    2f12: e00a         	b	0x2f2a <TIMER1+0x816>   @ imm = #0x14
    2f14: bf10         	yield
    2f16: f8da 3000    	ldr.w	r3, [r10]
    2f1a: 2b00         	cmp	r3, #0x0
    2f1c: bf02         	ittt	eq
    2f1e: bf10         	yieldeq
    2f20: f8da 3000    	ldreq.w	r3, [r10]
    2f24: 2b00         	cmpeq	r3, #0x0
    2f26: d109         	bne	0x2f3c <TIMER1+0x828>   @ imm = #0x12
    2f28: bf10         	yield
    2f2a: f8da 3000    	ldr.w	r3, [r10]
    2f2e: 2b00         	cmp	r3, #0x0
    2f30: bf02         	ittt	eq
    2f32: bf10         	yieldeq
    2f34: f8da 3000    	ldreq.w	r3, [r10]
    2f38: 2b00         	cmpeq	r3, #0x0
    2f3a: d0eb         	beq	0x2f14 <TIMER1+0x800>   @ imm = #-0x2a
    2f3c: f8da 3410    	ldr.w	r3, [r10, #0x410]
    2f40: b2d2         	uxtb	r2, r2
    2f42: 2300         	movs	r3, #0x0
    2f44: f8ca 3000    	str.w	r3, [r10]
    2f48: 602a         	str	r2, [r5]
    2f4a: e00a         	b	0x2f62 <TIMER1+0x84e>   @ imm = #0x14
    2f4c: bf10         	yield
    2f4e: f8da 2000    	ldr.w	r2, [r10]
    2f52: 2a00         	cmp	r2, #0x0
    2f54: bf02         	ittt	eq
    2f56: bf10         	yieldeq
    2f58: f8da 2000    	ldreq.w	r2, [r10]
    2f5c: 2a00         	cmpeq	r2, #0x0
    2f5e: d109         	bne	0x2f74 <TIMER1+0x860>   @ imm = #0x12
    2f60: bf10         	yield
    2f62: f8da 2000    	ldr.w	r2, [r10]
    2f66: 2a00         	cmp	r2, #0x0
    2f68: bf02         	ittt	eq
    2f6a: bf10         	yieldeq
    2f6c: f8da 2000    	ldreq.w	r2, [r10]
    2f70: 2a00         	cmpeq	r2, #0x0
    2f72: d0eb         	beq	0x2f4c <TIMER1+0x838>   @ imm = #-0x2a
    2f74: f8da 2410    	ldr.w	r2, [r10, #0x410]
    2f78: fa5f f28b    	uxtb.w	r2, r11
    2f7c: f8ca 3000    	str.w	r3, [r10]
    2f80: 602a         	str	r2, [r5]
    2f82: e00a         	b	0x2f9a <TIMER1+0x886>   @ imm = #0x14
    2f84: bf10         	yield
    2f86: f8da 2000    	ldr.w	r2, [r10]
    2f8a: 2a00         	cmp	r2, #0x0
    2f8c: bf02         	ittt	eq
    2f8e: bf10         	yieldeq
    2f90: f8da 2000    	ldreq.w	r2, [r10]
    2f94: 2a00         	cmpeq	r2, #0x0
    2f96: d109         	bne	0x2fac <TIMER1+0x898>   @ imm = #0x12
    2f98: bf10         	yield
    2f9a: f8da 2000    	ldr.w	r2, [r10]
    2f9e: 2a00         	cmp	r2, #0x0
    2fa0: bf02         	ittt	eq
    2fa2: bf10         	yieldeq
    2fa4: f8da 2000    	ldreq.w	r2, [r10]
    2fa8: 2a00         	cmpeq	r2, #0x0
    2faa: d0eb         	beq	0x2f84 <TIMER1+0x870>   @ imm = #-0x2a
    2fac: f8da 2410    	ldr.w	r2, [r10, #0x410]
    2fb0: 2200         	movs	r2, #0x0
    2fb2: f8ca 2000    	str.w	r2, [r10]
    2fb6: f8c5 9000    	str.w	r9, [r5]
    2fba: e00a         	b	0x2fd2 <TIMER1+0x8be>   @ imm = #0x14
    2fbc: bf10         	yield
    2fbe: f8da 3000    	ldr.w	r3, [r10]
    2fc2: 2b00         	cmp	r3, #0x0
    2fc4: bf02         	ittt	eq
    2fc6: bf10         	yieldeq
    2fc8: f8da 3000    	ldreq.w	r3, [r10]
    2fcc: 2b00         	cmpeq	r3, #0x0
    2fce: d109         	bne	0x2fe4 <TIMER1+0x8d0>   @ imm = #0x12
    2fd0: bf10         	yield
    2fd2: f8da 3000    	ldr.w	r3, [r10]
    2fd6: 2b00         	cmp	r3, #0x0
    2fd8: bf02         	ittt	eq
    2fda: bf10         	yieldeq
    2fdc: f8da 3000    	ldreq.w	r3, [r10]
    2fe0: 2b00         	cmpeq	r3, #0x0
    2fe2: d0eb         	beq	0x2fbc <TIMER1+0x8a8>   @ imm = #-0x2a
    2fe4: f8da 3410    	ldr.w	r3, [r10, #0x410]
    2fe8: f8ca 2000    	str.w	r2, [r10]
    2fec: 9a06         	ldr	r2, [sp, #0x18]
    2fee: b2d2         	uxtb	r2, r2
    2ff0: 602a         	str	r2, [r5]
    2ff2: e00a         	b	0x300a <TIMER1+0x8f6>   @ imm = #0x14
    2ff4: bf10         	yield
    2ff6: f8da 2000    	ldr.w	r2, [r10]
    2ffa: 2a00         	cmp	r2, #0x0
    2ffc: bf02         	ittt	eq
    2ffe: bf10         	yieldeq
    3000: f8da 2000    	ldreq.w	r2, [r10]
    3004: 2a00         	cmpeq	r2, #0x0
    3006: d109         	bne	0x301c <TIMER1+0x908>   @ imm = #0x12
    3008: bf10         	yield
    300a: f8da 2000    	ldr.w	r2, [r10]
    300e: 2a00         	cmp	r2, #0x0
    3010: bf02         	ittt	eq
    3012: bf10         	yieldeq
    3014: f8da 2000    	ldreq.w	r2, [r10]
    3018: 2a00         	cmpeq	r2, #0x0
    301a: d0eb         	beq	0x2ff4 <TIMER1+0x8e0>   @ imm = #-0x2a
    301c: f8da 2410    	ldr.w	r2, [r10, #0x410]
    3020: fa5f f388    	uxtb.w	r3, r8
    3024: 2200         	movs	r2, #0x0
    3026: f8ca 2000    	str.w	r2, [r10]
    302a: 602b         	str	r3, [r5]
    302c: e00a         	b	0x3044 <TIMER1+0x930>   @ imm = #0x14
    302e: bf10         	yield
    3030: f8da 3000    	ldr.w	r3, [r10]
    3034: 2b00         	cmp	r3, #0x0
    3036: bf02         	ittt	eq
    3038: bf10         	yieldeq
    303a: f8da 3000    	ldreq.w	r3, [r10]
    303e: 2b00         	cmpeq	r3, #0x0
    3040: d109         	bne	0x3056 <TIMER1+0x942>   @ imm = #0x12
    3042: bf10         	yield
    3044: f8da 3000    	ldr.w	r3, [r10]
    3048: 2b00         	cmp	r3, #0x0
    304a: bf02         	ittt	eq
    304c: bf10         	yieldeq
    304e: f8da 3000    	ldreq.w	r3, [r10]
    3052: 2b00         	cmpeq	r3, #0x0
    3054: d0eb         	beq	0x302e <TIMER1+0x91a>   @ imm = #-0x2a
    3056: f8da 3410    	ldr.w	r3, [r10, #0x410]
    305a: f8ca 2000    	str.w	r2, [r10]
    305e: fa5f f28e    	uxtb.w	r2, lr
    3062: 602a         	str	r2, [r5]
    3064: e00a         	b	0x307c <TIMER1+0x968>   @ imm = #0x14
    3066: bf10         	yield
    3068: f8da 2000    	ldr.w	r2, [r10]
    306c: 2a00         	cmp	r2, #0x0
    306e: bf02         	ittt	eq
    3070: bf10         	yieldeq
    3072: f8da 2000    	ldreq.w	r2, [r10]
    3076: 2a00         	cmpeq	r2, #0x0
    3078: d109         	bne	0x308e <TIMER1+0x97a>   @ imm = #0x12
    307a: bf10         	yield
    307c: f8da 2000    	ldr.w	r2, [r10]
    3080: 2a00         	cmp	r2, #0x0
    3082: bf02         	ittt	eq
    3084: bf10         	yieldeq
    3086: f8da 2000    	ldreq.w	r2, [r10]
    308a: 2a00         	cmpeq	r2, #0x0
    308c: d0eb         	beq	0x3066 <TIMER1+0x952>   @ imm = #-0x2a
    308e: f8da 2410    	ldr.w	r2, [r10, #0x410]
    3092: 2200         	movs	r2, #0x0
    3094: f8ca 2000    	str.w	r2, [r10]
    3098: 9804         	ldr	r0, [sp, #0x10]
    309a: 6028         	str	r0, [r5]
    309c: e00a         	b	0x30b4 <TIMER1+0x9a0>   @ imm = #0x14
    309e: bf10         	yield
    30a0: f8da 3000    	ldr.w	r3, [r10]
    30a4: 2b00         	cmp	r3, #0x0
    30a6: bf02         	ittt	eq
    30a8: bf10         	yieldeq
    30aa: f8da 3000    	ldreq.w	r3, [r10]
    30ae: 2b00         	cmpeq	r3, #0x0
    30b0: d109         	bne	0x30c6 <TIMER1+0x9b2>   @ imm = #0x12
    30b2: bf10         	yield
    30b4: f8da 3000    	ldr.w	r3, [r10]
    30b8: 2b00         	cmp	r3, #0x0
    30ba: bf02         	ittt	eq
    30bc: bf10         	yieldeq
    30be: f8da 3000    	ldreq.w	r3, [r10]
    30c2: 2b00         	cmpeq	r3, #0x0
    30c4: d0eb         	beq	0x309e <TIMER1+0x98a>   @ imm = #-0x2a
    30c6: f8da 3410    	ldr.w	r3, [r10, #0x410]
    30ca: 2930         	cmp	r1, #0x30
    30cc: f640 0308    	movw	r3, #0x808
    30d0: f8ca 2000    	str.w	r2, [r10]
    30d4: f44f 7280    	mov.w	r2, #0x100
    30d8: f2c5 0300    	movt	r3, #0x5000
    30dc: 601a         	str	r2, [r3]
    30de: f04f 0381    	mov.w	r3, #0x81
    30e2: d003         	beq	0x30ec <TIMER1+0x9d8>   @ imm = #0x6
    30e4: 2940         	cmp	r1, #0x40
    30e6: bf14         	ite	ne
    30e8: 2384         	movne	r3, #0x84
    30ea: 2382         	moveq	r3, #0x82
    30ec: f640 0608    	movw	r6, #0x808
    30f0: f2c5 0600    	movt	r6, #0x5000
    30f4: 6072         	str	r2, [r6, #0x4]
    30f6: 602b         	str	r3, [r5]
    30f8: e00a         	b	0x3110 <TIMER1+0x9fc>   @ imm = #0x14
    30fa: bf10         	yield
    30fc: f8da 2000    	ldr.w	r2, [r10]
    3100: 2a00         	cmp	r2, #0x0
    3102: bf02         	ittt	eq
    3104: bf10         	yieldeq
    3106: f8da 2000    	ldreq.w	r2, [r10]
    310a: 2a00         	cmpeq	r2, #0x0
    310c: d109         	bne	0x3122 <TIMER1+0xa0e>   @ imm = #0x12
    310e: bf10         	yield
    3110: f8da 2000    	ldr.w	r2, [r10]
    3114: 2a00         	cmp	r2, #0x0
    3116: bf02         	ittt	eq
    3118: bf10         	yieldeq
    311a: f8da 2000    	ldreq.w	r2, [r10]
    311e: 2a00         	cmpeq	r2, #0x0
    3120: d0eb         	beq	0x30fa <TIMER1+0x9e6>   @ imm = #-0x2a
    3122: f640 0608    	movw	r6, #0x808
    3126: f8da 3410    	ldr.w	r3, [r10, #0x410]
    312a: 2200         	movs	r2, #0x0
    312c: 2930         	cmp	r1, #0x30
    312e: f44f 7380    	mov.w	r3, #0x100
    3132: f2c5 0600    	movt	r6, #0x5000
    3136: f8ca 2000    	str.w	r2, [r10]
    313a: 6033         	str	r3, [r6]
    313c: d003         	beq	0x3146 <TIMER1+0xa32>   @ imm = #0x6
    313e: 2950         	cmp	r1, #0x50
    3140: bf0c         	ite	eq
    3142: 2202         	moveq	r2, #0x2
    3144: 2201         	movne	r2, #0x1
    3146: 2101         	movs	r1, #0x1
    3148: f80c 1002    	strb.w	r1, [r12, r2]
    314c: e6a8         	b	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x2b0
    314e: f003 fe9a    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3d34
    3152: f240 0027    	movw	r0, #0x27
    3156: f2c0 0000    	movt	r0, #0x0
    315a: f003 fe19    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3c32
    315e: f003 fee7    	bl	0x6f30 <_defmt_release> @ imm = #0x3dce
    3162: 2000         	movs	r0, #0x0
    3164: 7520         	strb	r0, [r4, #0x14]
    3166: e69b         	b	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x2ca
    3168: f003 fe8d    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3d1a
    316c: f240 0025    	movw	r0, #0x25
    3170: f2c0 0000    	movt	r0, #0x0
    3174: f003 fe0c    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3c18
    3178: f003 feda    	bl	0x6f30 <_defmt_release> @ imm = #0x3db4
    317c: 4620         	mov	r0, r4
    317e: 2400         	movs	r4, #0x0
    3180: 2102         	movs	r1, #0x2
    3182: 7484         	strb	r4, [r0, #0x12]
    3184: f7fd fad0    	bl	0x728 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400> @ imm = #-0x2a60
    3188: f640 0108    	movw	r1, #0x808
    318c: f44f 7080    	mov.w	r0, #0x100
    3190: f2c5 0100    	movt	r1, #0x5000
    3194: 6048         	str	r0, [r1, #0x4]
    3196: f243 511c    	movw	r1, #0x351c
    319a: f2c4 0100    	movt	r1, #0x4000
    319e: 2003         	movs	r0, #0x3
    31a0: 6008         	str	r0, [r1]
    31a2: e1b8         	b	0x3516 <TIMER1+0xe02>   @ imm = #0x370
    31a4: f003 fe6f    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3cde
    31a8: f240 0026    	movw	r0, #0x26
    31ac: f2c0 0000    	movt	r0, #0x0
    31b0: f003 fdee    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3bdc
    31b4: f003 febc    	bl	0x6f30 <_defmt_release> @ imm = #0x3d78
    31b8: 2000         	movs	r0, #0x0
    31ba: 74e0         	strb	r0, [r4, #0x13]
    31bc: e670         	b	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x320
    31be: f003 fe62    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3cc4
    31c2: f240 0024    	movw	r0, #0x24
    31c6: f2c0 0000    	movt	r0, #0x0
    31ca: f003 fde1    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3bc2
    31ce: f003 feaf    	bl	0x6f30 <_defmt_release> @ imm = #0x3d5e
    31d2: e665         	b	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x336
    31d4: f003 fe57    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3cae
    31d8: f240 0028    	movw	r0, #0x28
    31dc: f2c0 0000    	movt	r0, #0x0
    31e0: f003 fdd6    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3bac
    31e4: f003 fea4    	bl	0x6f30 <_defmt_release> @ imm = #0x3d48
    31e8: f44f 70b0    	mov.w	r0, #0x160
    31ec: 4621         	mov	r1, r4
    31ee: 8020         	strh	r0, [r4]
    31f0: a80e         	add	r0, sp, #0x38
    31f2: f7fc ffb1    	bl	0x158 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb> @ imm = #-0x309e
    31f6: 4620         	mov	r0, r4
    31f8: f89d 4048    	ldrb.w	r4, [sp, #0x48]
    31fc: 2c09         	cmp	r4, #0x9
    31fe: f080 86ef    	bhs.w	0x3fe0 <TIMER1+0x18cc>  @ imm = #0xdde
    3202: 2100         	movs	r1, #0x0
    3204: f7fd fa90    	bl	0x728 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400> @ imm = #-0x2ae0
    3208: f89d 0047    	ldrb.w	r0, [sp, #0x47]
    320c: f89d 1046    	ldrb.w	r1, [sp, #0x46]
    3210: 9d10         	ldr	r5, [sp, #0x40]
    3212: f8bd 203a    	ldrh.w	r2, [sp, #0x3a]
    3216: f8bd 3038    	ldrh.w	r3, [sp, #0x38]
    321a: f240 28e0    	movw	r8, #0x2e0
    321e: 2b00         	cmp	r3, #0x0
    3220: f2c2 0800    	movt	r8, #0x2000
    3224: f47f ae3c    	bne.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x388
    3228: e127         	b	0x347a <TIMER1+0xd66>   @ imm = #0x24e
    322a: f003 fe2c    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3c58
    322e: f240 0029    	movw	r0, #0x29
    3232: f2c0 0000    	movt	r0, #0x0
    3236: f003 fdab    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3b56
    323a: f003 fe79    	bl	0x6f30 <_defmt_release> @ imm = #0x3cf2
    323e: f44f 70b8    	mov.w	r0, #0x170
    3242: 4621         	mov	r1, r4
    3244: 8020         	strh	r0, [r4]
    3246: 4648         	mov	r0, r9
    3248: 46a0         	mov	r8, r4
    324a: f7fc ff85    	bl	0x158 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::receive_can::h342a9f1552fe13cb> @ imm = #-0x30f6
    324e: e899 006e    	ldm.w	r9, {r1, r2, r3, r5, r6}
    3252: 4658         	mov	r0, r11
    3254: c06e         	stm	r0!, {r1, r2, r3, r5, r6}
    3256: f89d 0030    	ldrb.w	r0, [sp, #0x30]
    325a: 2809         	cmp	r0, #0x9
    325c: f080 86c8    	bhs.w	0x3ff0 <TIMER1+0x18dc>  @ imm = #0xd90
    3260: 1fc2         	subs	r2, r0, #0x7
    3262: bf38         	it	lo
    3264: 2200         	movlo	r2, #0x0
    3266: 2800         	cmp	r0, #0x0
    3268: f000 80f1    	beq.w	0x344e <TIMER1+0xd3a>   @ imm = #0x1e2
    326c: f10b 0c08    	add.w	r12, r11, #0x8
    3270: f1c0 0900    	rsb.w	r9, r0, #0x0
    3274: 2600         	movs	r6, #0x0
    3276: e003         	b	0x3280 <TIMER1+0xb6c>   @ imm = #0x6
    3278: 1c6e         	adds	r6, r5, #0x1
    327a: 4286         	cmp	r6, r0
    327c: f080 80e7    	bhs.w	0x344e <TIMER1+0xd3a>   @ imm = #0x1ce
    3280: f91c 1006    	ldrsb.w	r1, [r12, r6]
    3284: 2900         	cmp	r1, #0x0
    3286: d454         	bmi	0x3332 <TIMER1+0xc1e>   @ imm = #0xa8
    3288: 4271         	rsbs	r1, r6, #0
    328a: 0789         	lsls	r1, r1, #0x1e
    328c: d002         	beq	0x3294 <TIMER1+0xb80>   @ imm = #0x4
    328e: 3601         	adds	r6, #0x1
    3290: e7f3         	b	0x327a <TIMER1+0xb66>   @ imm = #-0x1a
    3292: 3620         	adds	r6, #0x20
    3294: 4296         	cmp	r6, r2
    3296: d226         	bhs	0x32e6 <TIMER1+0xbd2>   @ imm = #0x4c
    3298: eb0b 0506    	add.w	r5, r11, r6
    329c: e9d5 1402    	ldrd	r1, r4, [r5, #8]
    32a0: 4321         	orrs	r1, r4
    32a2: f011 3f80    	tst.w	r1, #0x80808080
    32a6: d11e         	bne	0x32e6 <TIMER1+0xbd2>   @ imm = #0x3c
    32a8: f106 0108    	add.w	r1, r6, #0x8
    32ac: 4291         	cmp	r1, r2
    32ae: d219         	bhs	0x32e4 <TIMER1+0xbd0>   @ imm = #0x32
    32b0: e9d5 4304    	ldrd	r4, r3, [r5, #16]
    32b4: 4323         	orrs	r3, r4
    32b6: f013 3f80    	tst.w	r3, #0x80808080
    32ba: d113         	bne	0x32e4 <TIMER1+0xbd0>   @ imm = #0x26
    32bc: f106 0110    	add.w	r1, r6, #0x10
    32c0: 4291         	cmp	r1, r2
    32c2: d20f         	bhs	0x32e4 <TIMER1+0xbd0>   @ imm = #0x1e
    32c4: e9d5 3406    	ldrd	r3, r4, [r5, #24]
    32c8: 4323         	orrs	r3, r4
    32ca: f013 3f80    	tst.w	r3, #0x80808080
    32ce: d109         	bne	0x32e4 <TIMER1+0xbd0>   @ imm = #0x12
    32d0: f106 0118    	add.w	r1, r6, #0x18
    32d4: 4291         	cmp	r1, r2
    32d6: d205         	bhs	0x32e4 <TIMER1+0xbd0>   @ imm = #0xa
    32d8: e9d5 3508    	ldrd	r3, r5, [r5, #32]
    32dc: 432b         	orrs	r3, r5
    32de: f013 3f80    	tst.w	r3, #0x80808080
    32e2: d0d6         	beq	0x3292 <TIMER1+0xb7e>   @ imm = #-0x54
    32e4: 460e         	mov	r6, r1
    32e6: 4286         	cmp	r6, r0
    32e8: d2c7         	bhs	0x327a <TIMER1+0xb66>   @ imm = #-0x72
    32ea: eb0b 0506    	add.w	r5, r11, r6
    32ee: f995 1008    	ldrsb.w	r1, [r5, #0x8]
    32f2: 2900         	cmp	r1, #0x0
    32f4: d4c1         	bmi	0x327a <TIMER1+0xb66>   @ imm = #-0x7e
    32f6: eb09 0406    	add.w	r4, r9, r6
    32fa: 1c61         	adds	r1, r4, #0x1
    32fc: f000 80a7    	beq.w	0x344e <TIMER1+0xd3a>   @ imm = #0x14e
    3300: f995 1009    	ldrsb.w	r1, [r5, #0x9]
    3304: 2900         	cmp	r1, #0x0
    3306: d4c2         	bmi	0x328e <TIMER1+0xb7a>   @ imm = #-0x7c
    3308: 1ca1         	adds	r1, r4, #0x2
    330a: f000 80a0    	beq.w	0x344e <TIMER1+0xd3a>   @ imm = #0x140
    330e: f995 100a    	ldrsb.w	r1, [r5, #0xa]
    3312: 2900         	cmp	r1, #0x0
    3314: f100 8097    	bmi.w	0x3446 <TIMER1+0xd32>   @ imm = #0x12e
    3318: 1ce1         	adds	r1, r4, #0x3
    331a: f000 8098    	beq.w	0x344e <TIMER1+0xd3a>   @ imm = #0x130
    331e: f995 100b    	ldrsb.w	r1, [r5, #0xb]
    3322: 2900         	cmp	r1, #0x0
    3324: f100 8091    	bmi.w	0x344a <TIMER1+0xd36>   @ imm = #0x122
    3328: 3604         	adds	r6, #0x4
    332a: eb19 0106    	adds.w	r1, r9, r6
    332e: d1dc         	bne	0x32ea <TIMER1+0xbd6>   @ imm = #-0x48
    3330: e08d         	b	0x344e <TIMER1+0xd3a>   @ imm = #0x11a
    3332: b2cd         	uxtb	r5, r1
    3334: f647 2114    	movw	r1, #0x7a14
    3338: f2c0 0100    	movt	r1, #0x0
    333c: 5d49         	ldrb	r1, [r1, r5]
    333e: 2904         	cmp	r1, #0x4
    3340: d020         	beq	0x3384 <TIMER1+0xc70>   @ imm = #0x40
    3342: 2903         	cmp	r1, #0x3
    3344: d00e         	beq	0x3364 <TIMER1+0xc50>   @ imm = #0x1c
    3346: 2902         	cmp	r1, #0x2
    3348: f040 8640    	bne.w	0x3fcc <TIMER1+0x18b8>  @ imm = #0xc80
    334c: 1c75         	adds	r5, r6, #0x1
    334e: 4285         	cmp	r5, r0
    3350: f080 8638    	bhs.w	0x3fc4 <TIMER1+0x18b0>  @ imm = #0xc70
    3354: f91c 1005    	ldrsb.w	r1, [r12, r5]
    3358: f111 0f41    	cmn.w	r1, #0x41
    335c: f77f af8c    	ble.w	0x3278 <TIMER1+0xb64>   @ imm = #-0xe8
    3360: f000 be34    	b.w	0x3fcc <TIMER1+0x18b8>  @ imm = #0xc68
    3364: 1c71         	adds	r1, r6, #0x1
    3366: 4281         	cmp	r1, r0
    3368: f080 862c    	bhs.w	0x3fc4 <TIMER1+0x18b0>  @ imm = #0xc58
    336c: f81c 4001    	ldrb.w	r4, [r12, r1]
    3370: 2de0         	cmp	r5, #0xe0
    3372: d017         	beq	0x33a4 <TIMER1+0xc90>   @ imm = #0x2e
    3374: 2ded         	cmp	r5, #0xed
    3376: d120         	bne	0x33ba <TIMER1+0xca6>   @ imm = #0x40
    3378: b261         	sxtb	r1, r4
    337a: f111 0f61    	cmn.w	r1, #0x61
    337e: dd56         	ble	0x342e <TIMER1+0xd1a>   @ imm = #0xac
    3380: f000 be24    	b.w	0x3fcc <TIMER1+0x18b8>  @ imm = #0xc48
    3384: 1c71         	adds	r1, r6, #0x1
    3386: 4281         	cmp	r1, r0
    3388: f080 861c    	bhs.w	0x3fc4 <TIMER1+0x18b0>  @ imm = #0xc38
    338c: f81c 1001    	ldrb.w	r1, [r12, r1]
    3390: 2df0         	cmp	r5, #0xf0
    3392: d00d         	beq	0x33b0 <TIMER1+0xc9c>   @ imm = #0x1a
    3394: 2df4         	cmp	r5, #0xf4
    3396: b249         	sxtb	r1, r1
    3398: d119         	bne	0x33ce <TIMER1+0xcba>   @ imm = #0x32
    339a: f111 0f71    	cmn.w	r1, #0x71
    339e: dd23         	ble	0x33e8 <TIMER1+0xcd4>   @ imm = #0x46
    33a0: f000 be14    	b.w	0x3fcc <TIMER1+0x18b8>  @ imm = #0xc28
    33a4: f004 01e0    	and	r1, r4, #0xe0
    33a8: 29a0         	cmp	r1, #0xa0
    33aa: d040         	beq	0x342e <TIMER1+0xd1a>   @ imm = #0x80
    33ac: f000 be0e    	b.w	0x3fcc <TIMER1+0x18b8>  @ imm = #0xc1c
    33b0: 3990         	subs	r1, #0x90
    33b2: 2930         	cmp	r1, #0x30
    33b4: d318         	blo	0x33e8 <TIMER1+0xcd4>   @ imm = #0x30
    33b6: f000 be09    	b.w	0x3fcc <TIMER1+0x18b8>  @ imm = #0xc12
    33ba: f1a5 01e1    	sub.w	r1, r5, #0xe1
    33be: 290c         	cmp	r1, #0xc
    33c0: b261         	sxtb	r1, r4
    33c2: d227         	bhs	0x3414 <TIMER1+0xd00>   @ imm = #0x4e
    33c4: f111 0f40    	cmn.w	r1, #0x40
    33c8: db31         	blt	0x342e <TIMER1+0xd1a>   @ imm = #0x62
    33ca: f000 bdff    	b.w	0x3fcc <TIMER1+0x18b8>  @ imm = #0xbfe
    33ce: f1a5 03f1    	sub.w	r3, r5, #0xf1
    33d2: f04f 0e01    	mov.w	lr, #0x1
    33d6: f44f 7580    	mov.w	r5, #0x100
    33da: 2b02         	cmp	r3, #0x2
    33dc: f200 8613    	bhi.w	0x4006 <TIMER1+0x18f2>  @ imm = #0xc26
    33e0: f111 0f40    	cmn.w	r1, #0x40
    33e4: f280 860f    	bge.w	0x4006 <TIMER1+0x18f2>  @ imm = #0xc1e
    33e8: 1cb1         	adds	r1, r6, #0x2
    33ea: 4281         	cmp	r1, r0
    33ec: f080 85ea    	bhs.w	0x3fc4 <TIMER1+0x18b0>  @ imm = #0xbd4
    33f0: f91c 1001    	ldrsb.w	r1, [r12, r1]
    33f4: f111 0f41    	cmn.w	r1, #0x41
    33f8: f300 85ed    	bgt.w	0x3fd6 <TIMER1+0x18c2>  @ imm = #0xbda
    33fc: 1cf5         	adds	r5, r6, #0x3
    33fe: 4285         	cmp	r5, r0
    3400: f080 85e0    	bhs.w	0x3fc4 <TIMER1+0x18b0>  @ imm = #0xbc0
    3404: f91c 1005    	ldrsb.w	r1, [r12, r5]
    3408: f111 0f41    	cmn.w	r1, #0x41
    340c: f77f af34    	ble.w	0x3278 <TIMER1+0xb64>   @ imm = #-0x198
    3410: f000 bdf5    	b.w	0x3ffe <TIMER1+0x18ea>  @ imm = #0xbea
    3414: f005 03fe    	and	r3, r5, #0xfe
    3418: f04f 0e01    	mov.w	lr, #0x1
    341c: f44f 7580    	mov.w	r5, #0x100
    3420: 2bee         	cmp	r3, #0xee
    3422: f040 85f0    	bne.w	0x4006 <TIMER1+0x18f2>  @ imm = #0xbe0
    3426: f111 0f40    	cmn.w	r1, #0x40
    342a: f280 85ec    	bge.w	0x4006 <TIMER1+0x18f2>  @ imm = #0xbd8
    342e: 1cb5         	adds	r5, r6, #0x2
    3430: 4285         	cmp	r5, r0
    3432: f080 85c7    	bhs.w	0x3fc4 <TIMER1+0x18b0>  @ imm = #0xb8e
    3436: f91c 1005    	ldrsb.w	r1, [r12, r5]
    343a: f111 0f41    	cmn.w	r1, #0x41
    343e: f77f af1b    	ble.w	0x3278 <TIMER1+0xb64>   @ imm = #-0x1ca
    3442: f000 bdc8    	b.w	0x3fd6 <TIMER1+0x18c2>  @ imm = #0xb90
    3446: 3602         	adds	r6, #0x2
    3448: e717         	b	0x327a <TIMER1+0xb66>   @ imm = #-0x1d2
    344a: 3603         	adds	r6, #0x3
    344c: e715         	b	0x327a <TIMER1+0xb66>   @ imm = #-0x1d6
    344e: 4640         	mov	r0, r8
    3450: 2101         	movs	r1, #0x1
    3452: f7fd f969    	bl	0x728 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400> @ imm = #-0x2d2e
    3456: f89d 405c    	ldrb.w	r4, [sp, #0x5c]
    345a: f89d 005b    	ldrb.w	r0, [sp, #0x5b]
    345e: f89d 105a    	ldrb.w	r1, [sp, #0x5a]
    3462: 9d15         	ldr	r5, [sp, #0x54]
    3464: f8bd 204e    	ldrh.w	r2, [sp, #0x4e]
    3468: f8bd 304c    	ldrh.w	r3, [sp, #0x4c]
    346c: f240 28e0    	movw	r8, #0x2e0
    3470: 2b00         	cmp	r3, #0x0
    3472: f2c2 0800    	movt	r8, #0x2000
    3476: f47f ad13    	bne.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x5da
    347a: 2a32         	cmp	r2, #0x32
    347c: f63f ad10    	bhi.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x5e0
    3480: 2300         	movs	r3, #0x0
    3482: e8df f012    	tbh	[pc, r2, lsl #1]
    3486: 68 05 68 05  	.word	0x05680568
    348a: 33 00 33 00  	.word	0x00330033
    348e: 33 00 33 00  	.word	0x00330033
    3492: 33 00 33 00  	.word	0x00330033
    3496: 33 00 33 00  	.word	0x00330033
    349a: 74 05 75 05  	.word	0x05750574
    349e: 6e 05 72 05  	.word	0x0572056e
    34a2: 70 05 82 05  	.word	0x05820570
    34a6: 33 00 33 00  	.word	0x00330033
    34aa: 33 00 33 00  	.word	0x00330033
    34ae: 33 00 33 00  	.word	0x00330033
    34b2: 33 00 33 00  	.word	0x00330033
    34b6: 33 00 33 00  	.word	0x00330033
    34ba: 33 00 33 00  	.word	0x00330033
    34be: 33 00 33 00  	.word	0x00330033
    34c2: 34 00 33 00  	.word	0x00330034
    34c6: 33 00 33 00  	.word	0x00330033
    34ca: 33 00 33 00  	.word	0x00330033
    34ce: 33 00 33 00  	.word	0x00330033
    34d2: 33 00 33 00  	.word	0x00330033
    34d6: 34 00 33 00  	.word	0x00330034
    34da: 33 00 33 00  	.word	0x00330033
    34de: 33 00 33 00  	.word	0x00330033
    34e2: 33 00 33 00  	.word	0x00330033
    34e6: 33 00 33 00  	.word	0x00330033
    34ea: 34 00        	.short	0x0034
    34ec: e4d8         	b	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x650
    34ee: 2c09         	cmp	r4, #0x9
    34f0: f080 8576    	bhs.w	0x3fe0 <TIMER1+0x18cc>  @ imm = #0xaec
    34f4: 2207         	movs	r2, #0x7
    34f6: 2c04         	cmp	r4, #0x4
    34f8: f4ff acd2    	blo.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x65c
    34fc: f000 bd52    	b.w	0x3fa4 <TIMER1+0x1890>  @ imm = #0xaa4
    3500: bf10         	yield
    3502: f8da 0000    	ldr.w	r0, [r10]
    3506: 2800         	cmp	r0, #0x0
    3508: bf02         	ittt	eq
    350a: bf10         	yieldeq
    350c: f8da 0000    	ldreq.w	r0, [r10]
    3510: 2800         	cmpeq	r0, #0x0
    3512: d109         	bne	0x3528 <TIMER1+0xe14>   @ imm = #0x12
    3514: bf10         	yield
    3516: f8da 0000    	ldr.w	r0, [r10]
    351a: 2800         	cmp	r0, #0x0
    351c: bf02         	ittt	eq
    351e: bf10         	yieldeq
    3520: f8da 0000    	ldreq.w	r0, [r10]
    3524: 2800         	cmpeq	r0, #0x0
    3526: d0eb         	beq	0x3500 <TIMER1+0xdec>   @ imm = #-0x2a
    3528: f8da 0410    	ldr.w	r0, [r10, #0x410]
    352c: 202c         	movs	r0, #0x2c
    352e: f8ca 4000    	str.w	r4, [r10]
    3532: 6008         	str	r0, [r1]
    3534: e00a         	b	0x354c <TIMER1+0xe38>   @ imm = #0x14
    3536: bf10         	yield
    3538: f8da 0000    	ldr.w	r0, [r10]
    353c: 2800         	cmp	r0, #0x0
    353e: bf02         	ittt	eq
    3540: bf10         	yieldeq
    3542: f8da 0000    	ldreq.w	r0, [r10]
    3546: 2800         	cmpeq	r0, #0x0
    3548: d109         	bne	0x355e <TIMER1+0xe4a>   @ imm = #0x12
    354a: bf10         	yield
    354c: f8da 0000    	ldr.w	r0, [r10]
    3550: 2800         	cmp	r0, #0x0
    3552: bf02         	ittt	eq
    3554: bf10         	yieldeq
    3556: f8da 0000    	ldreq.w	r0, [r10]
    355a: 2800         	cmpeq	r0, #0x0
    355c: d0eb         	beq	0x3536 <TIMER1+0xe22>   @ imm = #-0x2a
    355e: f8da 0410    	ldr.w	r0, [r10, #0x410]
    3562: 2000         	movs	r0, #0x0
    3564: f8ca 0000    	str.w	r0, [r10]
    3568: 6008         	str	r0, [r1]
    356a: e00a         	b	0x3582 <TIMER1+0xe6e>   @ imm = #0x14
    356c: bf10         	yield
    356e: f8da 1000    	ldr.w	r1, [r10]
    3572: 2900         	cmp	r1, #0x0
    3574: bf02         	ittt	eq
    3576: bf10         	yieldeq
    3578: f8da 1000    	ldreq.w	r1, [r10]
    357c: 2900         	cmpeq	r1, #0x0
    357e: d109         	bne	0x3594 <TIMER1+0xe80>   @ imm = #0x12
    3580: bf10         	yield
    3582: f8da 1000    	ldr.w	r1, [r10]
    3586: 2900         	cmp	r1, #0x0
    3588: bf02         	ittt	eq
    358a: bf10         	yieldeq
    358c: f8da 1000    	ldreq.w	r1, [r10]
    3590: 2900         	cmpeq	r1, #0x0
    3592: d0eb         	beq	0x356c <TIMER1+0xe58>   @ imm = #-0x2a
    3594: f8da 1410    	ldr.w	r1, [r10, #0x410]
    3598: f640 0108    	movw	r1, #0x808
    359c: f8ca 0000    	str.w	r0, [r10]
    35a0: f44f 7080    	mov.w	r0, #0x100
    35a4: f2c5 0100    	movt	r1, #0x5000
    35a8: 6008         	str	r0, [r1]
    35aa: f7ff bc79    	b.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x70e
    35ae: bf10         	yield
    35b0: f8da 0000    	ldr.w	r0, [r10]
    35b4: 2800         	cmp	r0, #0x0
    35b6: bf02         	ittt	eq
    35b8: bf10         	yieldeq
    35ba: f8da 0000    	ldreq.w	r0, [r10]
    35be: 2800         	cmpeq	r0, #0x0
    35c0: d109         	bne	0x35d6 <TIMER1+0xec2>   @ imm = #0x12
    35c2: bf10         	yield
    35c4: f8da 0000    	ldr.w	r0, [r10]
    35c8: 2800         	cmp	r0, #0x0
    35ca: bf02         	ittt	eq
    35cc: bf10         	yieldeq
    35ce: f8da 0000    	ldreq.w	r0, [r10]
    35d2: 2800         	cmpeq	r0, #0x0
    35d4: d0eb         	beq	0x35ae <TIMER1+0xe9a>   @ imm = #-0x2a
    35d6: f8da 1410    	ldr.w	r1, [r10, #0x410]
    35da: 2000         	movs	r0, #0x0
    35dc: 212c         	movs	r1, #0x2c
    35de: f8ca 0000    	str.w	r0, [r10]
    35e2: 6019         	str	r1, [r3]
    35e4: e00a         	b	0x35fc <TIMER1+0xee8>   @ imm = #0x14
    35e6: bf10         	yield
    35e8: f8da 1000    	ldr.w	r1, [r10]
    35ec: 2900         	cmp	r1, #0x0
    35ee: bf02         	ittt	eq
    35f0: bf10         	yieldeq
    35f2: f8da 1000    	ldreq.w	r1, [r10]
    35f6: 2900         	cmpeq	r1, #0x0
    35f8: d109         	bne	0x360e <TIMER1+0xefa>   @ imm = #0x12
    35fa: bf10         	yield
    35fc: f8da 1000    	ldr.w	r1, [r10]
    3600: 2900         	cmp	r1, #0x0
    3602: bf02         	ittt	eq
    3604: bf10         	yieldeq
    3606: f8da 1000    	ldreq.w	r1, [r10]
    360a: 2900         	cmpeq	r1, #0x0
    360c: d0eb         	beq	0x35e6 <TIMER1+0xed2>   @ imm = #-0x2a
    360e: f8da 1410    	ldr.w	r1, [r10, #0x410]
    3612: f8ca 0000    	str.w	r0, [r10]
    3616: 6018         	str	r0, [r3]
    3618: e00a         	b	0x3630 <TIMER1+0xf1c>   @ imm = #0x14
    361a: bf10         	yield
    361c: f8da 0000    	ldr.w	r0, [r10]
    3620: 2800         	cmp	r0, #0x0
    3622: bf02         	ittt	eq
    3624: bf10         	yieldeq
    3626: f8da 0000    	ldreq.w	r0, [r10]
    362a: 2800         	cmpeq	r0, #0x0
    362c: d109         	bne	0x3642 <TIMER1+0xf2e>   @ imm = #0x12
    362e: bf10         	yield
    3630: f8da 0000    	ldr.w	r0, [r10]
    3634: 2800         	cmp	r0, #0x0
    3636: bf02         	ittt	eq
    3638: bf10         	yieldeq
    363a: f8da 0000    	ldreq.w	r0, [r10]
    363e: 2800         	cmpeq	r0, #0x0
    3640: d0eb         	beq	0x361a <TIMER1+0xf06>   @ imm = #-0x2a
    3642: f8da 1410    	ldr.w	r1, [r10, #0x410]
    3646: f640 0208    	movw	r2, #0x808
    364a: 2000         	movs	r0, #0x0
    364c: 9105         	str	r1, [sp, #0x14]
    364e: f44f 7180    	mov.w	r1, #0x100
    3652: f2c5 0200    	movt	r2, #0x5000
    3656: f8ca 0000    	str.w	r0, [r10]
    365a: 6011         	str	r1, [r2]
    365c: 6051         	str	r1, [r2, #0x4]
    365e: 2103         	movs	r1, #0x3
    3660: 6019         	str	r1, [r3]
    3662: e00a         	b	0x367a <TIMER1+0xf66>   @ imm = #0x14
    3664: bf10         	yield
    3666: f8da 1000    	ldr.w	r1, [r10]
    366a: 2900         	cmp	r1, #0x0
    366c: bf02         	ittt	eq
    366e: bf10         	yieldeq
    3670: f8da 1000    	ldreq.w	r1, [r10]
    3674: 2900         	cmpeq	r1, #0x0
    3676: d109         	bne	0x368c <TIMER1+0xf78>   @ imm = #0x12
    3678: bf10         	yield
    367a: f8da 1000    	ldr.w	r1, [r10]
    367e: 2900         	cmp	r1, #0x0
    3680: bf02         	ittt	eq
    3682: bf10         	yieldeq
    3684: f8da 1000    	ldreq.w	r1, [r10]
    3688: 2900         	cmpeq	r1, #0x0
    368a: d0eb         	beq	0x3664 <TIMER1+0xf50>   @ imm = #-0x2a
    368c: f8da 1410    	ldr.w	r1, [r10, #0x410]
    3690: f8ca 0000    	str.w	r0, [r10]
    3694: 200e         	movs	r0, #0xe
    3696: 6018         	str	r0, [r3]
    3698: e00a         	b	0x36b0 <TIMER1+0xf9c>   @ imm = #0x14
    369a: bf10         	yield
    369c: f8da 0000    	ldr.w	r0, [r10]
    36a0: 2800         	cmp	r0, #0x0
    36a2: bf02         	ittt	eq
    36a4: bf10         	yieldeq
    36a6: f8da 0000    	ldreq.w	r0, [r10]
    36aa: 2800         	cmpeq	r0, #0x0
    36ac: d109         	bne	0x36c2 <TIMER1+0xfae>   @ imm = #0x12
    36ae: bf10         	yield
    36b0: f8da 0000    	ldr.w	r0, [r10]
    36b4: 2800         	cmp	r0, #0x0
    36b6: bf02         	ittt	eq
    36b8: bf10         	yieldeq
    36ba: f8da 0000    	ldreq.w	r0, [r10]
    36be: 2800         	cmpeq	r0, #0x0
    36c0: d0eb         	beq	0x369a <TIMER1+0xf86>   @ imm = #-0x2a
    36c2: f8da 0410    	ldr.w	r0, [r10, #0x410]
    36c6: 2000         	movs	r0, #0x0
    36c8: f8ca 0000    	str.w	r0, [r10]
    36cc: 6018         	str	r0, [r3]
    36ce: e00a         	b	0x36e6 <TIMER1+0xfd2>   @ imm = #0x14
    36d0: bf10         	yield
    36d2: f8da 1000    	ldr.w	r1, [r10]
    36d6: 2900         	cmp	r1, #0x0
    36d8: bf02         	ittt	eq
    36da: bf10         	yieldeq
    36dc: f8da 1000    	ldreq.w	r1, [r10]
    36e0: 2900         	cmpeq	r1, #0x0
    36e2: d109         	bne	0x36f8 <TIMER1+0xfe4>   @ imm = #0x12
    36e4: bf10         	yield
    36e6: f8da 1000    	ldr.w	r1, [r10]
    36ea: 2900         	cmp	r1, #0x0
    36ec: bf02         	ittt	eq
    36ee: bf10         	yieldeq
    36f0: f8da 1000    	ldreq.w	r1, [r10]
    36f4: 2900         	cmpeq	r1, #0x0
    36f6: d0eb         	beq	0x36d0 <TIMER1+0xfbc>   @ imm = #-0x2a
    36f8: f8da 5410    	ldr.w	r5, [r10, #0x410]
    36fc: f640 0108    	movw	r1, #0x808
    3700: f8ca 0000    	str.w	r0, [r10]
    3704: f44f 7080    	mov.w	r0, #0x100
    3708: f2c5 0100    	movt	r1, #0x5000
    370c: 6008         	str	r0, [r1]
    370e: 6048         	str	r0, [r1, #0x4]
    3710: 2003         	movs	r0, #0x3
    3712: 6018         	str	r0, [r3]
    3714: e00a         	b	0x372c <TIMER1+0x1018>  @ imm = #0x14
    3716: bf10         	yield
    3718: f8da 0000    	ldr.w	r0, [r10]
    371c: 2800         	cmp	r0, #0x0
    371e: bf02         	ittt	eq
    3720: bf10         	yieldeq
    3722: f8da 0000    	ldreq.w	r0, [r10]
    3726: 2800         	cmpeq	r0, #0x0
    3728: d109         	bne	0x373e <TIMER1+0x102a>  @ imm = #0x12
    372a: bf10         	yield
    372c: f8da 0000    	ldr.w	r0, [r10]
    3730: 2800         	cmp	r0, #0x0
    3732: bf02         	ittt	eq
    3734: bf10         	yieldeq
    3736: f8da 0000    	ldreq.w	r0, [r10]
    373a: 2800         	cmpeq	r0, #0x0
    373c: d0eb         	beq	0x3716 <TIMER1+0x1002>  @ imm = #-0x2a
    373e: f8da 1410    	ldr.w	r1, [r10, #0x410]
    3742: 2000         	movs	r0, #0x0
    3744: 212d         	movs	r1, #0x2d
    3746: f8ca 0000    	str.w	r0, [r10]
    374a: 6019         	str	r1, [r3]
    374c: e00a         	b	0x3764 <TIMER1+0x1050>  @ imm = #0x14
    374e: bf10         	yield
    3750: f8da 1000    	ldr.w	r1, [r10]
    3754: 2900         	cmp	r1, #0x0
    3756: bf02         	ittt	eq
    3758: bf10         	yieldeq
    375a: f8da 1000    	ldreq.w	r1, [r10]
    375e: 2900         	cmpeq	r1, #0x0
    3760: d109         	bne	0x3776 <TIMER1+0x1062>  @ imm = #0x12
    3762: bf10         	yield
    3764: f8da 1000    	ldr.w	r1, [r10]
    3768: 2900         	cmp	r1, #0x0
    376a: bf02         	ittt	eq
    376c: bf10         	yieldeq
    376e: f8da 1000    	ldreq.w	r1, [r10]
    3772: 2900         	cmpeq	r1, #0x0
    3774: d0eb         	beq	0x374e <TIMER1+0x103a>  @ imm = #-0x2a
    3776: 9406         	str	r4, [sp, #0x18]
    3778: f8da 1410    	ldr.w	r1, [r10, #0x410]
    377c: f8ca 0000    	str.w	r0, [r10]
    3780: 6018         	str	r0, [r3]
    3782: e00a         	b	0x379a <TIMER1+0x1086>  @ imm = #0x14
    3784: bf10         	yield
    3786: f8da 0000    	ldr.w	r0, [r10]
    378a: 2800         	cmp	r0, #0x0
    378c: bf02         	ittt	eq
    378e: bf10         	yieldeq
    3790: f8da 0000    	ldreq.w	r0, [r10]
    3794: 2800         	cmpeq	r0, #0x0
    3796: d109         	bne	0x37ac <TIMER1+0x1098>  @ imm = #0x12
    3798: bf10         	yield
    379a: f8da 0000    	ldr.w	r0, [r10]
    379e: 2800         	cmp	r0, #0x0
    37a0: bf02         	ittt	eq
    37a2: bf10         	yieldeq
    37a4: f8da 0000    	ldreq.w	r0, [r10]
    37a8: 2800         	cmpeq	r0, #0x0
    37aa: d0eb         	beq	0x3784 <TIMER1+0x1070>  @ imm = #-0x2a
    37ac: f640 0608    	movw	r6, #0x808
    37b0: 2000         	movs	r0, #0x0
    37b2: f44f 7280    	mov.w	r2, #0x100
    37b6: f2c5 0600    	movt	r6, #0x5000
    37ba: f8da 1410    	ldr.w	r1, [r10, #0x410]
    37be: f8ca 0000    	str.w	r0, [r10]
    37c2: 6032         	str	r2, [r6]
    37c4: 6072         	str	r2, [r6, #0x4]
    37c6: 2203         	movs	r2, #0x3
    37c8: 601a         	str	r2, [r3]
    37ca: b2cc         	uxtb	r4, r1
    37cc: f8da 2000    	ldr.w	r2, [r10]
    37d0: bbd2         	cbnz	r2, 0x3848 <TIMER1+0x1134> @ imm = #0x74
    37d2: e02b         	b	0x382c <TIMER1+0x1118>  @ imm = #0x56
    37d4: 07e0         	lsls	r0, r4, #0x1f
    37d6: bf08         	it	eq
    37d8: f003 fdd5    	bleq	0x7386 <__cpsie>        @ imm = #0x3baa
    37dc: f003 fdd8    	bl	0x7390 <__primask_r>    @ imm = #0x3bb0
    37e0: 4604         	mov	r4, r0
    37e2: f003 fdce    	bl	0x7382 <__cpsid>        @ imm = #0x3b9c
    37e6: 07e0         	lsls	r0, r4, #0x1f
    37e8: f8d8 5100    	ldr.w	r5, [r8, #0x100]
    37ec: bf08         	it	eq
    37ee: f003 fdca    	bleq	0x7386 <__cpsie>        @ imm = #0x3b94
    37f2: 9c07         	ldr	r4, [sp, #0x1c]
    37f4: b135         	cbz	r5, 0x3804 <TIMER1+0x10f0> @ imm = #0xc
    37f6: 2003         	movs	r0, #0x3
    37f8: 7720         	strb	r0, [r4, #0x1c]
    37fa: 7620         	strb	r0, [r4, #0x18]
    37fc: e00a         	b	0x3814 <TIMER1+0x1100>  @ imm = #0x14
    37fe: 9804         	ldr	r0, [sp, #0x10]
    3800: 07c0         	lsls	r0, r0, #0x1f
    3802: d1f8         	bne	0x37f6 <TIMER1+0x10e2>  @ imm = #-0x10
    3804: 2001         	movs	r0, #0x1
    3806: 7720         	strb	r0, [r4, #0x1c]
    3808: 7620         	strb	r0, [r4, #0x18]
    380a: 2000         	movs	r0, #0x0
    380c: f3bf 8f5f    	dmb	sy
    3810: f884 0020    	strb.w	r0, [r4, #0x20]
    3814: f240 2be0    	movw	r11, #0x2e0
    3818: f8dd a008    	ldr.w	r10, [sp, #0x8]
    381c: f2c2 0b00    	movt	r11, #0x2000
    3820: f7fe bfb6    	b.w	0x2790 <TIMER1+0x7c>    @ imm = #-0x1094
    3824: bf10         	yield
    3826: f8da 1000    	ldr.w	r1, [r10]
    382a: b969         	cbnz	r1, 0x3848 <TIMER1+0x1134> @ imm = #0x1a
    382c: bf10         	yield
    382e: f8da 1000    	ldr.w	r1, [r10]
    3832: b949         	cbnz	r1, 0x3848 <TIMER1+0x1134> @ imm = #0x12
    3834: bf10         	yield
    3836: f8da 1000    	ldr.w	r1, [r10]
    383a: 2900         	cmp	r1, #0x0
    383c: bf02         	ittt	eq
    383e: bf10         	yieldeq
    3840: f8da 1000    	ldreq.w	r1, [r10]
    3844: 2900         	cmpeq	r1, #0x0
    3846: d0ed         	beq	0x3824 <TIMER1+0x1110>  @ imm = #-0x26
    3848: f8da 1410    	ldr.w	r1, [r10, #0x410]
    384c: f8ca 0000    	str.w	r0, [r10]
    3850: 2030         	movs	r0, #0x30
    3852: 6018         	str	r0, [r3]
    3854: e00a         	b	0x386c <TIMER1+0x1158>  @ imm = #0x14
    3856: bf10         	yield
    3858: f8da 0000    	ldr.w	r0, [r10]
    385c: 2800         	cmp	r0, #0x0
    385e: bf02         	ittt	eq
    3860: bf10         	yieldeq
    3862: f8da 0000    	ldreq.w	r0, [r10]
    3866: 2800         	cmpeq	r0, #0x0
    3868: d109         	bne	0x387e <TIMER1+0x116a>  @ imm = #0x12
    386a: bf10         	yield
    386c: f8da 0000    	ldr.w	r0, [r10]
    3870: 2800         	cmp	r0, #0x0
    3872: bf02         	ittt	eq
    3874: bf10         	yieldeq
    3876: f8da 0000    	ldreq.w	r0, [r10]
    387a: 2800         	cmpeq	r0, #0x0
    387c: d0eb         	beq	0x3856 <TIMER1+0x1142>  @ imm = #-0x2a
    387e: f8da 0410    	ldr.w	r0, [r10, #0x410]
    3882: 2000         	movs	r0, #0x0
    3884: f8ca 0000    	str.w	r0, [r10]
    3888: 6018         	str	r0, [r3]
    388a: e00a         	b	0x38a2 <TIMER1+0x118e>  @ imm = #0x14
    388c: bf10         	yield
    388e: f8da 1000    	ldr.w	r1, [r10]
    3892: 2900         	cmp	r1, #0x0
    3894: bf02         	ittt	eq
    3896: bf10         	yieldeq
    3898: f8da 1000    	ldreq.w	r1, [r10]
    389c: 2900         	cmpeq	r1, #0x0
    389e: d109         	bne	0x38b4 <TIMER1+0x11a0>  @ imm = #0x12
    38a0: bf10         	yield
    38a2: f8da 1000    	ldr.w	r1, [r10]
    38a6: 2900         	cmp	r1, #0x0
    38a8: bf02         	ittt	eq
    38aa: bf10         	yieldeq
    38ac: f8da 1000    	ldreq.w	r1, [r10]
    38b0: 2900         	cmpeq	r1, #0x0
    38b2: d0eb         	beq	0x388c <TIMER1+0x1178>  @ imm = #-0x2a
    38b4: f8da 9410    	ldr.w	r9, [r10, #0x410]
    38b8: f640 0108    	movw	r1, #0x808
    38bc: f8ca 0000    	str.w	r0, [r10]
    38c0: f44f 7080    	mov.w	r0, #0x100
    38c4: f2c5 0100    	movt	r1, #0x5000
    38c8: 6008         	str	r0, [r1]
    38ca: 6048         	str	r0, [r1, #0x4]
    38cc: 2003         	movs	r0, #0x3
    38ce: 6018         	str	r0, [r3]
    38d0: e00a         	b	0x38e8 <TIMER1+0x11d4>  @ imm = #0x14
    38d2: bf10         	yield
    38d4: f8da 0000    	ldr.w	r0, [r10]
    38d8: 2800         	cmp	r0, #0x0
    38da: bf02         	ittt	eq
    38dc: bf10         	yieldeq
    38de: f8da 0000    	ldreq.w	r0, [r10]
    38e2: 2800         	cmpeq	r0, #0x0
    38e4: d109         	bne	0x38fa <TIMER1+0x11e6>  @ imm = #0x12
    38e6: bf10         	yield
    38e8: f8da 0000    	ldr.w	r0, [r10]
    38ec: 2800         	cmp	r0, #0x0
    38ee: bf02         	ittt	eq
    38f0: bf10         	yieldeq
    38f2: f8da 0000    	ldreq.w	r0, [r10]
    38f6: 2800         	cmpeq	r0, #0x0
    38f8: d0eb         	beq	0x38d2 <TIMER1+0x11be>  @ imm = #-0x2a
    38fa: f8da 1410    	ldr.w	r1, [r10, #0x410]
    38fe: 2000         	movs	r0, #0x0
    3900: 211c         	movs	r1, #0x1c
    3902: f8ca 0000    	str.w	r0, [r10]
    3906: 6019         	str	r1, [r3]
    3908: e00a         	b	0x3920 <TIMER1+0x120c>  @ imm = #0x14
    390a: bf10         	yield
    390c: f8da 1000    	ldr.w	r1, [r10]
    3910: 2900         	cmp	r1, #0x0
    3912: bf02         	ittt	eq
    3914: bf10         	yieldeq
    3916: f8da 1000    	ldreq.w	r1, [r10]
    391a: 2900         	cmpeq	r1, #0x0
    391c: d109         	bne	0x3932 <TIMER1+0x121e>  @ imm = #0x12
    391e: bf10         	yield
    3920: f8da 1000    	ldr.w	r1, [r10]
    3924: 2900         	cmp	r1, #0x0
    3926: bf02         	ittt	eq
    3928: bf10         	yieldeq
    392a: f8da 1000    	ldreq.w	r1, [r10]
    392e: 2900         	cmpeq	r1, #0x0
    3930: d0eb         	beq	0x390a <TIMER1+0x11f6>  @ imm = #-0x2a
    3932: f8da 1410    	ldr.w	r1, [r10, #0x410]
    3936: f8ca 0000    	str.w	r0, [r10]
    393a: 6018         	str	r0, [r3]
    393c: e00a         	b	0x3954 <TIMER1+0x1240>  @ imm = #0x14
    393e: bf10         	yield
    3940: f8da 0000    	ldr.w	r0, [r10]
    3944: 2800         	cmp	r0, #0x0
    3946: bf02         	ittt	eq
    3948: bf10         	yieldeq
    394a: f8da 0000    	ldreq.w	r0, [r10]
    394e: 2800         	cmpeq	r0, #0x0
    3950: d109         	bne	0x3966 <TIMER1+0x1252>  @ imm = #0x12
    3952: bf10         	yield
    3954: f8da 0000    	ldr.w	r0, [r10]
    3958: 2800         	cmp	r0, #0x0
    395a: bf02         	ittt	eq
    395c: bf10         	yieldeq
    395e: f8da 0000    	ldreq.w	r0, [r10]
    3962: 2800         	cmpeq	r0, #0x0
    3964: d0eb         	beq	0x393e <TIMER1+0x122a>  @ imm = #-0x2a
    3966: f640 0208    	movw	r2, #0x808
    396a: f8da b410    	ldr.w	r11, [r10, #0x410]
    396e: 2000         	movs	r0, #0x0
    3970: f44f 7180    	mov.w	r1, #0x100
    3974: f2c5 0200    	movt	r2, #0x5000
    3978: f8ca 0000    	str.w	r0, [r10]
    397c: 6011         	str	r1, [r2]
    397e: 6051         	str	r1, [r2, #0x4]
    3980: 2103         	movs	r1, #0x3
    3982: 6019         	str	r1, [r3]
    3984: e00a         	b	0x399c <TIMER1+0x1288>  @ imm = #0x14
    3986: bf10         	yield
    3988: f8da 1000    	ldr.w	r1, [r10]
    398c: 2900         	cmp	r1, #0x0
    398e: bf02         	ittt	eq
    3990: bf10         	yieldeq
    3992: f8da 1000    	ldreq.w	r1, [r10]
    3996: 2900         	cmpeq	r1, #0x0
    3998: d109         	bne	0x39ae <TIMER1+0x129a>  @ imm = #0x12
    399a: bf10         	yield
    399c: f8da 1000    	ldr.w	r1, [r10]
    39a0: 2900         	cmp	r1, #0x0
    39a2: bf02         	ittt	eq
    39a4: bf10         	yieldeq
    39a6: f8da 1000    	ldreq.w	r1, [r10]
    39aa: 2900         	cmpeq	r1, #0x0
    39ac: d0eb         	beq	0x3986 <TIMER1+0x1272>  @ imm = #-0x2a
    39ae: f8da 1410    	ldr.w	r1, [r10, #0x410]
    39b2: f8ca 0000    	str.w	r0, [r10]
    39b6: 201d         	movs	r0, #0x1d
    39b8: 6018         	str	r0, [r3]
    39ba: e00a         	b	0x39d2 <TIMER1+0x12be>  @ imm = #0x14
    39bc: bf10         	yield
    39be: f8da 0000    	ldr.w	r0, [r10]
    39c2: 2800         	cmp	r0, #0x0
    39c4: bf02         	ittt	eq
    39c6: bf10         	yieldeq
    39c8: f8da 0000    	ldreq.w	r0, [r10]
    39cc: 2800         	cmpeq	r0, #0x0
    39ce: d109         	bne	0x39e4 <TIMER1+0x12d0>  @ imm = #0x12
    39d0: bf10         	yield
    39d2: f8da 0000    	ldr.w	r0, [r10]
    39d6: 2800         	cmp	r0, #0x0
    39d8: bf02         	ittt	eq
    39da: bf10         	yieldeq
    39dc: f8da 0000    	ldreq.w	r0, [r10]
    39e0: 2800         	cmpeq	r0, #0x0
    39e2: d0eb         	beq	0x39bc <TIMER1+0x12a8>  @ imm = #-0x2a
    39e4: f8da 0410    	ldr.w	r0, [r10, #0x410]
    39e8: 2600         	movs	r6, #0x0
    39ea: f243 501c    	movw	r0, #0x351c
    39ee: f8ca 6000    	str.w	r6, [r10]
    39f2: f2c4 0000    	movt	r0, #0x4000
    39f6: 6006         	str	r6, [r0]
    39f8: e00a         	b	0x3a10 <TIMER1+0x12fc>  @ imm = #0x14
    39fa: bf10         	yield
    39fc: f8da 0000    	ldr.w	r0, [r10]
    3a00: 2800         	cmp	r0, #0x0
    3a02: bf02         	ittt	eq
    3a04: bf10         	yieldeq
    3a06: f8da 0000    	ldreq.w	r0, [r10]
    3a0a: 2800         	cmpeq	r0, #0x0
    3a0c: d109         	bne	0x3a22 <TIMER1+0x130e>  @ imm = #0x12
    3a0e: bf10         	yield
    3a10: f8da 0000    	ldr.w	r0, [r10]
    3a14: 2800         	cmp	r0, #0x0
    3a16: bf02         	ittt	eq
    3a18: bf10         	yieldeq
    3a1a: f8da 0000    	ldreq.w	r0, [r10]
    3a1e: 2800         	cmpeq	r0, #0x0
    3a20: d0eb         	beq	0x39fa <TIMER1+0x12e6>  @ imm = #-0x2a
    3a22: f8da 0410    	ldr.w	r0, [r10, #0x410]
    3a26: f640 0108    	movw	r1, #0x808
    3a2a: f44f 7080    	mov.w	r0, #0x100
    3a2e: f2c5 0100    	movt	r1, #0x5000
    3a32: f240 080d    	movw	r8, #0xd
    3a36: f8ca 6000    	str.w	r6, [r10]
    3a3a: 6008         	str	r0, [r1]
    3a3c: f04f 30ff    	mov.w	r0, #0xffffffff
    3a40: e9cd 0008    	strd	r0, r0, [sp, #32]
    3a44: 07e0         	lsls	r0, r4, #0x1f
    3a46: f2c0 0800    	movt	r8, #0x0
    3a4a: f040 8193    	bne.w	0x3d74 <TIMER1+0x1660>  @ imm = #0x326
    3a4e: 07a0         	lsls	r0, r4, #0x1e
    3a50: 9504         	str	r5, [sp, #0x10]
    3a52: f100 81ae    	bmi.w	0x3db2 <TIMER1+0x169e>  @ imm = #0x35c
    3a56: 0760         	lsls	r0, r4, #0x1d
    3a58: f100 81c9    	bmi.w	0x3dee <TIMER1+0x16da>  @ imm = #0x392
    3a5c: 0720         	lsls	r0, r4, #0x1c
    3a5e: f100 81e4    	bmi.w	0x3e2a <TIMER1+0x1716>  @ imm = #0x3c8
    3a62: 06e0         	lsls	r0, r4, #0x1b
    3a64: f100 81ff    	bmi.w	0x3e66 <TIMER1+0x1752>  @ imm = #0x3fe
    3a68: 06a0         	lsls	r0, r4, #0x1a
    3a6a: f100 821a    	bmi.w	0x3ea2 <TIMER1+0x178e>  @ imm = #0x434
    3a6e: 0660         	lsls	r0, r4, #0x19
    3a70: f100 8235    	bmi.w	0x3ede <TIMER1+0x17ca>  @ imm = #0x46a
    3a74: b260         	sxtb	r0, r4
    3a76: f1b0 3fff    	cmp.w	r0, #0xffffffff
    3a7a: f340 8250    	ble.w	0x3f1e <TIMER1+0x180a>  @ imm = #0x4a0
    3a7e: f003 fa02    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3404
    3a82: f240 0020    	movw	r0, #0x20
    3a86: f2c0 0000    	movt	r0, #0x0
    3a8a: f003 f981    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3302
    3a8e: f240 0003    	movw	r0, #0x3
    3a92: 2102         	movs	r1, #0x2
    3a94: f2c0 0000    	movt	r0, #0x0
    3a98: f8ad 0060    	strh.w	r0, [sp, #0x60]
    3a9c: a818         	add	r0, sp, #0x60
    3a9e: f003 fabe    	bl	0x701e <_defmt_write>   @ imm = #0x357c
    3aa2: a818         	add	r0, sp, #0x60
    3aa4: 2104         	movs	r1, #0x4
    3aa6: 9618         	str	r6, [sp, #0x60]
    3aa8: f003 fab9    	bl	0x701e <_defmt_write>   @ imm = #0x3572
    3aac: f003 fa40    	bl	0x6f30 <_defmt_release> @ imm = #0x3480
    3ab0: f003 f9e9    	bl	0x6e86 <_defmt_acquire> @ imm = #0x33d2
    3ab4: f240 0018    	movw	r0, #0x18
    3ab8: f2c0 0000    	movt	r0, #0x0
    3abc: f003 f968    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x32d0
    3ac0: f240 0505    	movw	r5, #0x5
    3ac4: a818         	add	r0, sp, #0x60
    3ac6: f2c0 0500    	movt	r5, #0x0
    3aca: 2102         	movs	r1, #0x2
    3acc: f8ad 5060    	strh.w	r5, [sp, #0x60]
    3ad0: f003 faa5    	bl	0x701e <_defmt_write>   @ imm = #0x354a
    3ad4: f3c9 00c0    	ubfx	r0, r9, #0x3, #0x1
    3ad8: f88d 0060    	strb.w	r0, [sp, #0x60]
    3adc: a818         	add	r0, sp, #0x60
    3ade: 2101         	movs	r1, #0x1
    3ae0: f003 fa9d    	bl	0x701e <_defmt_write>   @ imm = #0x353a
    3ae4: a818         	add	r0, sp, #0x60
    3ae6: 2102         	movs	r1, #0x2
    3ae8: f8ad 5060    	strh.w	r5, [sp, #0x60]
    3aec: f003 fa97    	bl	0x701e <_defmt_write>   @ imm = #0x352e
    3af0: f3c9 1000    	ubfx	r0, r9, #0x4, #0x1
    3af4: f88d 0060    	strb.w	r0, [sp, #0x60]
    3af8: a818         	add	r0, sp, #0x60
    3afa: 2101         	movs	r1, #0x1
    3afc: f003 fa8f    	bl	0x701e <_defmt_write>   @ imm = #0x351e
    3b00: a818         	add	r0, sp, #0x60
    3b02: 2102         	movs	r1, #0x2
    3b04: f8ad 5060    	strh.w	r5, [sp, #0x60]
    3b08: f003 fa89    	bl	0x701e <_defmt_write>   @ imm = #0x3512
    3b0c: f3c9 1040    	ubfx	r0, r9, #0x5, #0x1
    3b10: f88d 0060    	strb.w	r0, [sp, #0x60]
    3b14: a818         	add	r0, sp, #0x60
    3b16: 2101         	movs	r1, #0x1
    3b18: f003 fa81    	bl	0x701e <_defmt_write>   @ imm = #0x3502
    3b1c: f003 fa08    	bl	0x6f30 <_defmt_release> @ imm = #0x3410
    3b20: f003 f9b1    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3362
    3b24: f240 0019    	movw	r0, #0x19
    3b28: f2c0 0000    	movt	r0, #0x0
    3b2c: f003 f930    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3260
    3b30: f240 0901    	movw	r9, #0x1
    3b34: a818         	add	r0, sp, #0x60
    3b36: f2c0 0900    	movt	r9, #0x0
    3b3a: 2102         	movs	r1, #0x2
    3b3c: f8ad 9060    	strh.w	r9, [sp, #0x60]
    3b40: f003 fa6d    	bl	0x701e <_defmt_write>   @ imm = #0x34da
    3b44: a818         	add	r0, sp, #0x60
    3b46: 2101         	movs	r1, #0x1
    3b48: f88d b060    	strb.w	r11, [sp, #0x60]
    3b4c: f003 fa67    	bl	0x701e <_defmt_write>   @ imm = #0x34ce
    3b50: f003 f9ee    	bl	0x6f30 <_defmt_release> @ imm = #0x33dc
    3b54: f003 f997    	bl	0x6e86 <_defmt_acquire> @ imm = #0x332e
    3b58: f240 001a    	movw	r0, #0x1a
    3b5c: f2c0 0000    	movt	r0, #0x0
    3b60: f003 f916    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x322c
    3b64: a818         	add	r0, sp, #0x60
    3b66: 2102         	movs	r1, #0x2
    3b68: f8ad 9060    	strh.w	r9, [sp, #0x60]
    3b6c: f003 fa57    	bl	0x701e <_defmt_write>   @ imm = #0x34ae
    3b70: a818         	add	r0, sp, #0x60
    3b72: 2101         	movs	r1, #0x1
    3b74: f88d b060    	strb.w	r11, [sp, #0x60]
    3b78: f003 fa51    	bl	0x701e <_defmt_write>   @ imm = #0x34a2
    3b7c: f003 f9d8    	bl	0x6f30 <_defmt_release> @ imm = #0x33b0
    3b80: f003 f981    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3302
    3b84: f240 001b    	movw	r0, #0x1b
    3b88: f2c0 0000    	movt	r0, #0x0
    3b8c: f003 f900    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3200
    3b90: a818         	add	r0, sp, #0x60
    3b92: 2102         	movs	r1, #0x2
    3b94: f8ad 9060    	strh.w	r9, [sp, #0x60]
    3b98: f003 fa41    	bl	0x701e <_defmt_write>   @ imm = #0x3482
    3b9c: 9805         	ldr	r0, [sp, #0x14]
    3b9e: 2101         	movs	r1, #0x1
    3ba0: f88d 0060    	strb.w	r0, [sp, #0x60]
    3ba4: a818         	add	r0, sp, #0x60
    3ba6: f003 fa3a    	bl	0x701e <_defmt_write>   @ imm = #0x3474
    3baa: f003 f9c1    	bl	0x6f30 <_defmt_release> @ imm = #0x3382
    3bae: f003 f96a    	bl	0x6e86 <_defmt_acquire> @ imm = #0x32d4
    3bb2: f240 001c    	movw	r0, #0x1c
    3bb6: f2c0 0000    	movt	r0, #0x0
    3bba: f003 f8e9    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x31d2
    3bbe: a818         	add	r0, sp, #0x60
    3bc0: 2102         	movs	r1, #0x2
    3bc2: f8ad 9060    	strh.w	r9, [sp, #0x60]
    3bc6: f003 fa2a    	bl	0x701e <_defmt_write>   @ imm = #0x3454
    3bca: 9804         	ldr	r0, [sp, #0x10]
    3bcc: 2101         	movs	r1, #0x1
    3bce: f88d 0060    	strb.w	r0, [sp, #0x60]
    3bd2: a818         	add	r0, sp, #0x60
    3bd4: f003 fa23    	bl	0x701e <_defmt_write>   @ imm = #0x3446
    3bd8: f003 f9aa    	bl	0x6f30 <_defmt_release> @ imm = #0x3354
    3bdc: f003 f953    	bl	0x6e86 <_defmt_acquire> @ imm = #0x32a6
    3be0: f240 001d    	movw	r0, #0x1d
    3be4: f2c0 0000    	movt	r0, #0x0
    3be8: f003 f8d2    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x31a4
    3bec: a818         	add	r0, sp, #0x60
    3bee: 2102         	movs	r1, #0x2
    3bf0: f8ad 9060    	strh.w	r9, [sp, #0x60]
    3bf4: f003 fa13    	bl	0x701e <_defmt_write>   @ imm = #0x3426
    3bf8: a818         	add	r0, sp, #0x60
    3bfa: 2101         	movs	r1, #0x1
    3bfc: f88d 4060    	strb.w	r4, [sp, #0x60]
    3c00: f003 fa0d    	bl	0x701e <_defmt_write>   @ imm = #0x341a
    3c04: f003 f994    	bl	0x6f30 <_defmt_release> @ imm = #0x3328
    3c08: f003 f93d    	bl	0x6e86 <_defmt_acquire> @ imm = #0x327a
    3c0c: f240 001e    	movw	r0, #0x1e
    3c10: f2c0 0000    	movt	r0, #0x0
    3c14: f003 f8bc    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x3178
    3c18: f240 0006    	movw	r0, #0x6
    3c1c: 2102         	movs	r1, #0x2
    3c1e: f2c0 0000    	movt	r0, #0x0
    3c22: f8ad 0060    	strh.w	r0, [sp, #0x60]
    3c26: a818         	add	r0, sp, #0x60
    3c28: f003 f9f9    	bl	0x701e <_defmt_write>   @ imm = #0x33f2
    3c2c: a818         	add	r0, sp, #0x60
    3c2e: 2104         	movs	r1, #0x4
    3c30: 9618         	str	r6, [sp, #0x60]
    3c32: f003 f9f4    	bl	0x701e <_defmt_write>   @ imm = #0x33e8
    3c36: a818         	add	r0, sp, #0x60
    3c38: 2102         	movs	r1, #0x2
    3c3a: f8ad 8060    	strh.w	r8, [sp, #0x60]
    3c3e: f003 f9ee    	bl	0x701e <_defmt_write>   @ imm = #0x33dc
    3c42: 2e00         	cmp	r6, #0x0
    3c44: f000 808e    	beq.w	0x3d64 <TIMER1+0x1650>  @ imm = #0x11c
    3c48: f99d 0020    	ldrsb.w	r0, [sp, #0x20]
    3c4c: f647 4110    	movw	r1, #0x7c10
    3c50: f2c0 0100    	movt	r1, #0x0
    3c54: eb01 0080    	add.w	r0, r1, r0, lsl #2
    3c58: 2101         	movs	r1, #0x1
    3c5a: 6840         	ldr	r0, [r0, #0x4]
    3c5c: 7800         	ldrb	r0, [r0]
    3c5e: f88d 0060    	strb.w	r0, [sp, #0x60]
    3c62: a818         	add	r0, sp, #0x60
    3c64: f003 f9db    	bl	0x701e <_defmt_write>   @ imm = #0x33b6
    3c68: 2e01         	cmp	r6, #0x1
    3c6a: d07b         	beq	0x3d64 <TIMER1+0x1650>  @ imm = #0xf6
    3c6c: f99d 0021    	ldrsb.w	r0, [sp, #0x21]
    3c70: f647 4134    	movw	r1, #0x7c34
    3c74: f2c0 0100    	movt	r1, #0x0
    3c78: eb01 0080    	add.w	r0, r1, r0, lsl #2
    3c7c: 2101         	movs	r1, #0x1
    3c7e: 6840         	ldr	r0, [r0, #0x4]
    3c80: 7800         	ldrb	r0, [r0]
    3c82: f88d 0060    	strb.w	r0, [sp, #0x60]
    3c86: a818         	add	r0, sp, #0x60
    3c88: f003 f9c9    	bl	0x701e <_defmt_write>   @ imm = #0x3392
    3c8c: 2e02         	cmp	r6, #0x2
    3c8e: d069         	beq	0x3d64 <TIMER1+0x1650>  @ imm = #0xd2
    3c90: f99d 0022    	ldrsb.w	r0, [sp, #0x22]
    3c94: f647 4158    	movw	r1, #0x7c58
    3c98: f2c0 0100    	movt	r1, #0x0
    3c9c: eb01 0080    	add.w	r0, r1, r0, lsl #2
    3ca0: 2101         	movs	r1, #0x1
    3ca2: 6840         	ldr	r0, [r0, #0x4]
    3ca4: 7800         	ldrb	r0, [r0]
    3ca6: f88d 0060    	strb.w	r0, [sp, #0x60]
    3caa: a818         	add	r0, sp, #0x60
    3cac: f003 f9b7    	bl	0x701e <_defmt_write>   @ imm = #0x336e
    3cb0: 2e03         	cmp	r6, #0x3
    3cb2: d057         	beq	0x3d64 <TIMER1+0x1650>  @ imm = #0xae
    3cb4: f99d 0023    	ldrsb.w	r0, [sp, #0x23]
    3cb8: f647 417c    	movw	r1, #0x7c7c
    3cbc: f2c0 0100    	movt	r1, #0x0
    3cc0: eb01 0080    	add.w	r0, r1, r0, lsl #2
    3cc4: 2101         	movs	r1, #0x1
    3cc6: 6840         	ldr	r0, [r0, #0x4]
    3cc8: 7800         	ldrb	r0, [r0]
    3cca: f88d 0060    	strb.w	r0, [sp, #0x60]
    3cce: a818         	add	r0, sp, #0x60
    3cd0: f003 f9a5    	bl	0x701e <_defmt_write>   @ imm = #0x334a
    3cd4: 2e04         	cmp	r6, #0x4
    3cd6: d045         	beq	0x3d64 <TIMER1+0x1650>  @ imm = #0x8a
    3cd8: f99d 0024    	ldrsb.w	r0, [sp, #0x24]
    3cdc: f647 41a0    	movw	r1, #0x7ca0
    3ce0: f2c0 0100    	movt	r1, #0x0
    3ce4: eb01 0080    	add.w	r0, r1, r0, lsl #2
    3ce8: 2101         	movs	r1, #0x1
    3cea: 6840         	ldr	r0, [r0, #0x4]
    3cec: 7800         	ldrb	r0, [r0]
    3cee: f88d 0060    	strb.w	r0, [sp, #0x60]
    3cf2: a818         	add	r0, sp, #0x60
    3cf4: f003 f993    	bl	0x701e <_defmt_write>   @ imm = #0x3326
    3cf8: 2e05         	cmp	r6, #0x5
    3cfa: d033         	beq	0x3d64 <TIMER1+0x1650>  @ imm = #0x66
    3cfc: f99d 0025    	ldrsb.w	r0, [sp, #0x25]
    3d00: f647 41c4    	movw	r1, #0x7cc4
    3d04: f2c0 0100    	movt	r1, #0x0
    3d08: eb01 0080    	add.w	r0, r1, r0, lsl #2
    3d0c: 2101         	movs	r1, #0x1
    3d0e: 6840         	ldr	r0, [r0, #0x4]
    3d10: 7800         	ldrb	r0, [r0]
    3d12: f88d 0060    	strb.w	r0, [sp, #0x60]
    3d16: a818         	add	r0, sp, #0x60
    3d18: f003 f981    	bl	0x701e <_defmt_write>   @ imm = #0x3302
    3d1c: 2e06         	cmp	r6, #0x6
    3d1e: d021         	beq	0x3d64 <TIMER1+0x1650>  @ imm = #0x42
    3d20: f99d 0026    	ldrsb.w	r0, [sp, #0x26]
    3d24: f647 41e8    	movw	r1, #0x7ce8
    3d28: f2c0 0100    	movt	r1, #0x0
    3d2c: eb01 0080    	add.w	r0, r1, r0, lsl #2
    3d30: 2101         	movs	r1, #0x1
    3d32: 6840         	ldr	r0, [r0, #0x4]
    3d34: 7800         	ldrb	r0, [r0]
    3d36: f88d 0060    	strb.w	r0, [sp, #0x60]
    3d3a: a818         	add	r0, sp, #0x60
    3d3c: f003 f96f    	bl	0x701e <_defmt_write>   @ imm = #0x32de
    3d40: 2e07         	cmp	r6, #0x7
    3d42: d00f         	beq	0x3d64 <TIMER1+0x1650>  @ imm = #0x1e
    3d44: f99d 0027    	ldrsb.w	r0, [sp, #0x27]
    3d48: f647 510c    	movw	r1, #0x7d0c
    3d4c: f2c0 0100    	movt	r1, #0x0
    3d50: eb01 0080    	add.w	r0, r1, r0, lsl #2
    3d54: 2101         	movs	r1, #0x1
    3d56: 6840         	ldr	r0, [r0, #0x4]
    3d58: 7800         	ldrb	r0, [r0]
    3d5a: f88d 0060    	strb.w	r0, [sp, #0x60]
    3d5e: a818         	add	r0, sp, #0x60
    3d60: f003 f95d    	bl	0x701e <_defmt_write>   @ imm = #0x32ba
    3d64: f003 f8e4    	bl	0x6f30 <_defmt_release> @ imm = #0x31c8
    3d68: 9806         	ldr	r0, [sp, #0x18]
    3d6a: 2105         	movs	r1, #0x5
    3d6c: f7fc fcdc    	bl	0x728 <controller::drivers::can::Mcp2515Driver<SPI,PIN>::clear_interrupt_flag::h60bb1096fc712400> @ imm = #-0x3648
    3d70: f7ff b896    	b.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0xed4
    3d74: f003 f887    	bl	0x6e86 <_defmt_acquire> @ imm = #0x310e
    3d78: f240 001f    	movw	r0, #0x1f
    3d7c: f2c0 0000    	movt	r0, #0x0
    3d80: f003 f806    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x300c
    3d84: a818         	add	r0, sp, #0x60
    3d86: 2102         	movs	r1, #0x2
    3d88: f8ad 8060    	strh.w	r8, [sp, #0x60]
    3d8c: f003 f947    	bl	0x701e <_defmt_write>   @ imm = #0x328e
    3d90: 2007         	movs	r0, #0x7
    3d92: 2101         	movs	r1, #0x1
    3d94: f88d 0060    	strb.w	r0, [sp, #0x60]
    3d98: a818         	add	r0, sp, #0x60
    3d9a: 2601         	movs	r6, #0x1
    3d9c: f003 f93f    	bl	0x701e <_defmt_write>   @ imm = #0x327e
    3da0: f003 f8c6    	bl	0x6f30 <_defmt_release> @ imm = #0x318c
    3da4: 2000         	movs	r0, #0x0
    3da6: f88d 0020    	strb.w	r0, [sp, #0x20]
    3daa: 07a0         	lsls	r0, r4, #0x1e
    3dac: 9504         	str	r5, [sp, #0x10]
    3dae: f57f ae52    	bpl.w	0x3a56 <TIMER1+0x1342>  @ imm = #-0x35c
    3db2: f003 f868    	bl	0x6e86 <_defmt_acquire> @ imm = #0x30d0
    3db6: f240 001f    	movw	r0, #0x1f
    3dba: f2c0 0000    	movt	r0, #0x0
    3dbe: f002 ffe7    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2fce
    3dc2: a818         	add	r0, sp, #0x60
    3dc4: 2102         	movs	r1, #0x2
    3dc6: f8ad 8060    	strh.w	r8, [sp, #0x60]
    3dca: f003 f928    	bl	0x701e <_defmt_write>   @ imm = #0x3250
    3dce: 2006         	movs	r0, #0x6
    3dd0: 2101         	movs	r1, #0x1
    3dd2: f88d 0060    	strb.w	r0, [sp, #0x60]
    3dd6: a818         	add	r0, sp, #0x60
    3dd8: 2501         	movs	r5, #0x1
    3dda: f003 f920    	bl	0x701e <_defmt_write>   @ imm = #0x3240
    3dde: f003 f8a7    	bl	0x6f30 <_defmt_release> @ imm = #0x314e
    3de2: 3601         	adds	r6, #0x1
    3de4: f88d 5021    	strb.w	r5, [sp, #0x21]
    3de8: 0760         	lsls	r0, r4, #0x1d
    3dea: f57f ae37    	bpl.w	0x3a5c <TIMER1+0x1348>  @ imm = #-0x392
    3dee: f003 f84a    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3094
    3df2: f240 001f    	movw	r0, #0x1f
    3df6: f2c0 0000    	movt	r0, #0x0
    3dfa: f002 ffc9    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2f92
    3dfe: a818         	add	r0, sp, #0x60
    3e00: 2102         	movs	r1, #0x2
    3e02: f8ad 8060    	strh.w	r8, [sp, #0x60]
    3e06: 2502         	movs	r5, #0x2
    3e08: f003 f909    	bl	0x701e <_defmt_write>   @ imm = #0x3212
    3e0c: 2005         	movs	r0, #0x5
    3e0e: 2101         	movs	r1, #0x1
    3e10: f88d 0060    	strb.w	r0, [sp, #0x60]
    3e14: a818         	add	r0, sp, #0x60
    3e16: f003 f902    	bl	0x701e <_defmt_write>   @ imm = #0x3204
    3e1a: f003 f889    	bl	0x6f30 <_defmt_release> @ imm = #0x3112
    3e1e: 3601         	adds	r6, #0x1
    3e20: f88d 5022    	strb.w	r5, [sp, #0x22]
    3e24: 0720         	lsls	r0, r4, #0x1c
    3e26: f57f ae1c    	bpl.w	0x3a62 <TIMER1+0x134e>  @ imm = #-0x3c8
    3e2a: f003 f82c    	bl	0x6e86 <_defmt_acquire> @ imm = #0x3058
    3e2e: f240 001f    	movw	r0, #0x1f
    3e32: f2c0 0000    	movt	r0, #0x0
    3e36: f002 ffab    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2f56
    3e3a: a818         	add	r0, sp, #0x60
    3e3c: 2102         	movs	r1, #0x2
    3e3e: f8ad 8060    	strh.w	r8, [sp, #0x60]
    3e42: f003 f8ec    	bl	0x701e <_defmt_write>   @ imm = #0x31d8
    3e46: 2004         	movs	r0, #0x4
    3e48: 2101         	movs	r1, #0x1
    3e4a: f88d 0060    	strb.w	r0, [sp, #0x60]
    3e4e: a818         	add	r0, sp, #0x60
    3e50: f003 f8e5    	bl	0x701e <_defmt_write>   @ imm = #0x31ca
    3e54: f003 f86c    	bl	0x6f30 <_defmt_release> @ imm = #0x30d8
    3e58: 3601         	adds	r6, #0x1
    3e5a: 2003         	movs	r0, #0x3
    3e5c: f88d 0023    	strb.w	r0, [sp, #0x23]
    3e60: 06e0         	lsls	r0, r4, #0x1b
    3e62: f57f ae01    	bpl.w	0x3a68 <TIMER1+0x1354>  @ imm = #-0x3fe
    3e66: f003 f80e    	bl	0x6e86 <_defmt_acquire> @ imm = #0x301c
    3e6a: f240 001f    	movw	r0, #0x1f
    3e6e: f2c0 0000    	movt	r0, #0x0
    3e72: f002 ff8d    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2f1a
    3e76: a818         	add	r0, sp, #0x60
    3e78: 2102         	movs	r1, #0x2
    3e7a: f8ad 8060    	strh.w	r8, [sp, #0x60]
    3e7e: f003 f8ce    	bl	0x701e <_defmt_write>   @ imm = #0x319c
    3e82: 2003         	movs	r0, #0x3
    3e84: 2101         	movs	r1, #0x1
    3e86: f88d 0060    	strb.w	r0, [sp, #0x60]
    3e8a: a818         	add	r0, sp, #0x60
    3e8c: f003 f8c7    	bl	0x701e <_defmt_write>   @ imm = #0x318e
    3e90: f003 f84e    	bl	0x6f30 <_defmt_release> @ imm = #0x309c
    3e94: 3601         	adds	r6, #0x1
    3e96: 2004         	movs	r0, #0x4
    3e98: f88d 0024    	strb.w	r0, [sp, #0x24]
    3e9c: 06a0         	lsls	r0, r4, #0x1a
    3e9e: f57f ade6    	bpl.w	0x3a6e <TIMER1+0x135a>  @ imm = #-0x434
    3ea2: f002 fff0    	bl	0x6e86 <_defmt_acquire> @ imm = #0x2fe0
    3ea6: f240 001f    	movw	r0, #0x1f
    3eaa: f2c0 0000    	movt	r0, #0x0
    3eae: f002 ff6f    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2ede
    3eb2: a818         	add	r0, sp, #0x60
    3eb4: 2102         	movs	r1, #0x2
    3eb6: f8ad 8060    	strh.w	r8, [sp, #0x60]
    3eba: 2502         	movs	r5, #0x2
    3ebc: f003 f8af    	bl	0x701e <_defmt_write>   @ imm = #0x315e
    3ec0: a818         	add	r0, sp, #0x60
    3ec2: 2101         	movs	r1, #0x1
    3ec4: f88d 5060    	strb.w	r5, [sp, #0x60]
    3ec8: f003 f8a9    	bl	0x701e <_defmt_write>   @ imm = #0x3152
    3ecc: f003 f830    	bl	0x6f30 <_defmt_release> @ imm = #0x3060
    3ed0: 3601         	adds	r6, #0x1
    3ed2: 2005         	movs	r0, #0x5
    3ed4: f88d 0025    	strb.w	r0, [sp, #0x25]
    3ed8: 0660         	lsls	r0, r4, #0x19
    3eda: f57f adcb    	bpl.w	0x3a74 <TIMER1+0x1360>  @ imm = #-0x46a
    3ede: f002 ffd2    	bl	0x6e86 <_defmt_acquire> @ imm = #0x2fa4
    3ee2: f240 001f    	movw	r0, #0x1f
    3ee6: f2c0 0000    	movt	r0, #0x0
    3eea: f002 ff51    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2ea2
    3eee: a818         	add	r0, sp, #0x60
    3ef0: 2102         	movs	r1, #0x2
    3ef2: f8ad 8060    	strh.w	r8, [sp, #0x60]
    3ef6: f003 f892    	bl	0x701e <_defmt_write>   @ imm = #0x3124
    3efa: 2001         	movs	r0, #0x1
    3efc: 2101         	movs	r1, #0x1
    3efe: f88d 0060    	strb.w	r0, [sp, #0x60]
    3f02: a818         	add	r0, sp, #0x60
    3f04: f003 f88b    	bl	0x701e <_defmt_write>   @ imm = #0x3116
    3f08: f003 f812    	bl	0x6f30 <_defmt_release> @ imm = #0x3024
    3f0c: 3601         	adds	r6, #0x1
    3f0e: 2006         	movs	r0, #0x6
    3f10: f88d 0026    	strb.w	r0, [sp, #0x26]
    3f14: b260         	sxtb	r0, r4
    3f16: f1b0 3fff    	cmp.w	r0, #0xffffffff
    3f1a: f73f adb0    	bgt.w	0x3a7e <TIMER1+0x136a>  @ imm = #-0x4a0
    3f1e: f002 ffb2    	bl	0x6e86 <_defmt_acquire> @ imm = #0x2f64
    3f22: f240 001f    	movw	r0, #0x1f
    3f26: f2c0 0000    	movt	r0, #0x0
    3f2a: f002 ff31    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2e62
    3f2e: a818         	add	r0, sp, #0x60
    3f30: 2102         	movs	r1, #0x2
    3f32: f8ad 8060    	strh.w	r8, [sp, #0x60]
    3f36: f003 f872    	bl	0x701e <_defmt_write>   @ imm = #0x30e4
    3f3a: 2000         	movs	r0, #0x0
    3f3c: 2101         	movs	r1, #0x1
    3f3e: f88d 0060    	strb.w	r0, [sp, #0x60]
    3f42: a818         	add	r0, sp, #0x60
    3f44: f003 f86b    	bl	0x701e <_defmt_write>   @ imm = #0x30d6
    3f48: f002 fff2    	bl	0x6f30 <_defmt_release> @ imm = #0x2fe4
    3f4c: 3601         	adds	r6, #0x1
    3f4e: 2007         	movs	r0, #0x7
    3f50: f88d 0027    	strb.w	r0, [sp, #0x27]
    3f54: e593         	b	0x3a7e <TIMER1+0x136a>  @ imm = #-0x4da
    3f56: 2c09         	cmp	r4, #0x9
    3f58: d242         	bhs	0x3fe0 <TIMER1+0x18cc>  @ imm = #0x84
    3f5a: 2c04         	cmp	r4, #0x4
    3f5c: f4fe afa0    	blo.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x10c0
    3f60: e023         	b	0x3faa <TIMER1+0x1896>  @ imm = #0x46
    3f62: 2303         	movs	r3, #0x3
    3f64: e004         	b	0x3f70 <TIMER1+0x185c>  @ imm = #0x8
    3f66: 2204         	movs	r2, #0x4
    3f68: e010         	b	0x3f8c <TIMER1+0x1878>  @ imm = #0x20
    3f6a: 2302         	movs	r3, #0x2
    3f6c: e000         	b	0x3f70 <TIMER1+0x185c>  @ imm = #0x0
    3f6e: 2301         	movs	r3, #0x1
    3f70: 2c09         	cmp	r4, #0x9
    3f72: d235         	bhs	0x3fe0 <TIMER1+0x18cc>  @ imm = #0x6a
    3f74: 2c04         	cmp	r4, #0x4
    3f76: f4fe af93    	blo.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x10da
    3f7a: f647 5030    	movw	r0, #0x7d30
    3f7e: 9d01         	ldr	r5, [sp, #0x4]
    3f80: f2c0 0000    	movt	r0, #0x0
    3f84: f830 2013    	ldrh.w	r2, [r0, r3, lsl #1]
    3f88: e00c         	b	0x3fa4 <TIMER1+0x1890>  @ imm = #0x18
    3f8a: 2205         	movs	r2, #0x5
    3f8c: 2c08         	cmp	r4, #0x8
    3f8e: f47e af87    	bne.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x10f2
    3f92: 9b01         	ldr	r3, [sp, #0x4]
    3f94: ea41 2000    	orr.w	r0, r1, r0, lsl #8
    3f98: f64f 71ff    	movw	r1, #0xffff
    3f9c: ea23 0101    	bic.w	r1, r3, r1
    3fa0: 180d         	adds	r5, r1, r0
    3fa2: 9501         	str	r5, [sp, #0x4]
    3fa4: 2a06         	cmp	r2, #0x6
    3fa6: f47e af7b    	bne.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x110a
    3faa: f003 f9f9    	bl	0x73a0 <__basepri_r>    @ imm = #0x33f2
    3fae: 4604         	mov	r4, r0
    3fb0: 2080         	movs	r0, #0x80
    3fb2: f003 f9f2    	bl	0x739a <__basepri_max>  @ imm = #0x33e4
    3fb6: 4620         	mov	r0, r4
    3fb8: f8c8 5000    	str.w	r5, [r8]
    3fbc: f003 f9f4    	bl	0x73a8 <__basepri_w>    @ imm = #0x33e8
    3fc0: f7fe bf6e    	b.w	0x2ea0 <TIMER1+0x78c>   @ imm = #-0x1124
    3fc4: 2500         	movs	r5, #0x0
    3fc6: f04f 0e00    	mov.w	lr, #0x0
    3fca: e01c         	b	0x4006 <TIMER1+0x18f2>  @ imm = #0x38
    3fcc: f04f 0e01    	mov.w	lr, #0x1
    3fd0: f44f 7580    	mov.w	r5, #0x100
    3fd4: e017         	b	0x4006 <TIMER1+0x18f2>  @ imm = #0x2e
    3fd6: f04f 0e01    	mov.w	lr, #0x1
    3fda: f44f 7500    	mov.w	r5, #0x200
    3fde: e012         	b	0x4006 <TIMER1+0x18f2>  @ imm = #0x24
    3fe0: f647 32ac    	movw	r2, #0x7bac
    3fe4: 4620         	mov	r0, r4
    3fe6: f2c0 0200    	movt	r2, #0x0
    3fea: 2108         	movs	r1, #0x8
    3fec: f001 ffc4    	bl	0x5f78 <core::slice::index::slice_end_index_len_fail::h925ebdeb5ea0b7ba> @ imm = #0x1f88
    3ff0: f647 32ac    	movw	r2, #0x7bac
    3ff4: 2108         	movs	r1, #0x8
    3ff6: f2c0 0200    	movt	r2, #0x0
    3ffa: f001 ffbd    	bl	0x5f78 <core::slice::index::slice_end_index_len_fail::h925ebdeb5ea0b7ba> @ imm = #0x1f7a
    3ffe: f04f 0e01    	mov.w	lr, #0x1
    4002: f44f 7540    	mov.w	r5, #0x300
    4006: ea45 000e    	orr.w	r0, r5, lr
    400a: f247 6138    	movw	r1, #0x7638
    400e: f2c0 0100    	movt	r1, #0x0
    4012: e9cd 6018    	strd	r6, r0, [sp, #96]
    4016: a818         	add	r0, sp, #0x60
    4018: f002 fe14    	bl	0x6c44 <core::result::unwrap_failed::h6d5ad66472df96c4> @ imm = #0x2c28

0000401c <main>:
    401c: b580         	push	{r7, lr}
    401e: 466f         	mov	r7, sp
    4020: f5ad 7d16    	sub.w	sp, sp, #0x258
    4024: f003 f9ad    	bl	0x7382 <__cpsid>        @ imm = #0x335a
    4028: f24e 4006    	movw	r0, #0xe406
    402c: 21c0         	movs	r1, #0xc0
    402e: f2ce 0000    	movt	r0, #0xe000
    4032: f44f 7280    	mov.w	r2, #0x100
    4036: 7081         	strb	r1, [r0, #0x2]
    4038: f24e 1100    	movw	r1, #0xe100
    403c: f2ce 0100    	movt	r1, #0xe000
    4040: f44f 7300    	mov.w	r3, #0x200
    4044: 600a         	str	r2, [r1]
    4046: 22a0         	movs	r2, #0xa0
    4048: 70c2         	strb	r2, [r0, #0x3]
    404a: f04f 6480    	mov.w	r4, #0x4000000
    404e: 600b         	str	r3, [r1]
    4050: 2360         	movs	r3, #0x60
    4052: 7003         	strb	r3, [r0]
    4054: 2340         	movs	r3, #0x40
    4056: 600b         	str	r3, [r1]
    4058: 2380         	movs	r3, #0x80
    405a: 7503         	strb	r3, [r0, #0x14]
    405c: 600c         	str	r4, [r1]
    405e: f04f 5480    	mov.w	r4, #0x10000000
    4062: 7582         	strb	r2, [r0, #0x16]
    4064: 600c         	str	r4, [r1]
    4066: f240 24e0    	movw	r4, #0x2e0
    406a: 7042         	strb	r2, [r0, #0x1]
    406c: 2000         	movs	r0, #0x0
    406e: 600b         	str	r3, [r1]
    4070: f240 2150    	movw	r1, #0x250
    4074: f2c2 0100    	movt	r1, #0x2000
    4078: aa01         	add	r2, sp, #0x4
    407a: f8ad 0024    	strh.w	r0, [sp, #0x24]
    407e: f2c2 0400    	movt	r4, #0x2000
    4082: 638a         	str	r2, [r1, #0x38]
    4084: aa0a         	add	r2, sp, #0x28
    4086: f8ad 00e8    	strh.w	r0, [sp, #0xe8]
    408a: 63ca         	str	r2, [r1, #0x3c]
    408c: aa3c         	add	r2, sp, #0xf0
    408e: f8ad 0188    	strh.w	r0, [sp, #0x188]
    4092: 640a         	str	r2, [r1, #0x40]
    4094: f8ad 0238    	strh.w	r0, [sp, #0x238]
    4098: a864         	add	r0, sp, #0x190
    409a: 6448         	str	r0, [r1, #0x44]
    409c: f240 0000    	movw	r0, #0x0
    40a0: f2c2 0001    	movt	r0, #0x2001
    40a4: 42a0         	cmp	r0, r4
    40a6: d903         	bls	0x40b0 <main+0x94>      @ imm = #0x6
    40a8: f003 f96f    	bl	0x738a <__msp_r>        @ imm = #0x32de
    40ac: 42a0         	cmp	r0, r4
    40ae: d902         	bls	0x40b6 <main+0x9a>      @ imm = #0x4
    40b0: f000 f814    	bl	0x40dc <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115> @ imm = #0x28
    40b4: e7fe         	b	0x40b4 <main+0x98>      @ imm = #-0x4
    40b6: 2101         	movs	r1, #0x1
    40b8: 2000         	movs	r0, #0x0
    40ba: 9191         	str	r1, [sp, #0x244]
    40bc: f247 7184    	movw	r1, #0x7784
    40c0: f2c0 0100    	movt	r1, #0x0
    40c4: 9094         	str	r0, [sp, #0x250]
    40c6: 9093         	str	r0, [sp, #0x24c]
    40c8: 2004         	movs	r0, #0x4
    40ca: 9190         	str	r1, [sp, #0x240]
    40cc: f647 31ac    	movw	r1, #0x7bac
    40d0: 9092         	str	r0, [sp, #0x248]
    40d2: a890         	add	r0, sp, #0x240
    40d4: f2c0 0100    	movt	r1, #0x0
    40d8: f001 fe17    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #0x1c2e

000040dc <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115>:
    40dc: b5f0         	push	{r4, r5, r6, r7, lr}
    40de: af03         	add	r7, sp, #0xc
    40e0: e92d 0f00    	push.w	{r8, r9, r10, r11}
    40e4: b0b7         	sub	sp, #0xdc
    40e6: f240 2950    	movw	r9, #0x250
    40ea: 2401         	movs	r4, #0x1
    40ec: f2c2 0900    	movt	r9, #0x2000
    40f0: f889 4020    	strb.w	r4, [r9, #0x20]
    40f4: f002 fec7    	bl	0x6e86 <_defmt_acquire> @ imm = #0x2d8e
    40f8: f240 0008    	movw	r0, #0x8
    40fc: f2c0 0000    	movt	r0, #0x0
    4100: f002 fe46    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2c8c
    4104: f002 ff14    	bl	0x6f30 <_defmt_release> @ imm = #0x2e28
    4108: f04f 4080    	mov.w	r0, #0x40000000
    410c: 6004         	str	r4, [r0]
    410e: f240 1000    	movw	r0, #0x100
    4112: f2c4 0000    	movt	r0, #0x4000
    4116: 6801         	ldr	r1, [r0]
    4118: 2901         	cmp	r1, #0x1
    411a: d008         	beq	0x412e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x52> @ imm = #0x10
    411c: 6801         	ldr	r1, [r0]
    411e: 2901         	cmp	r1, #0x1
    4120: bf1c         	itt	ne
    4122: 6801         	ldrne	r1, [r0]
    4124: 2901         	cmpne	r1, #0x1
    4126: d002         	beq	0x412e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x52> @ imm = #0x4
    4128: 6801         	ldr	r1, [r0]
    412a: 2901         	cmp	r1, #0x1
    412c: d1f3         	bne	0x4116 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x3a> @ imm = #-0x1a
    412e: 2100         	movs	r1, #0x0
    4130: 6001         	str	r1, [r0]
    4132: 2101         	movs	r1, #0x1
    4134: f840 1cf8    	str	r1, [r0, #-248]
    4138: 6841         	ldr	r1, [r0, #0x4]
    413a: 2901         	cmp	r1, #0x1
    413c: d008         	beq	0x4150 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x74> @ imm = #0x10
    413e: 6841         	ldr	r1, [r0, #0x4]
    4140: 2901         	cmp	r1, #0x1
    4142: bf1c         	itt	ne
    4144: 6841         	ldrne	r1, [r0, #0x4]
    4146: 2901         	cmpne	r1, #0x1
    4148: d002         	beq	0x4150 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x74> @ imm = #0x4
    414a: 6841         	ldr	r1, [r0, #0x4]
    414c: 2901         	cmp	r1, #0x1
    414e: d1f3         	bne	0x4138 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x5c> @ imm = #-0x1a
    4150: f04f 0a00    	mov.w	r10, #0x0
    4154: f24b 0500    	movw	r5, #0xb000
    4158: 2602         	movs	r6, #0x2
    415a: f8c0 a004    	str.w	r10, [r0, #0x4]
    415e: f24b 5044    	movw	r0, #0xb544
    4162: f2c4 0500    	movt	r5, #0x4000
    4166: f2c0 0603    	movt	r6, #0x3
    416a: f2c4 0000    	movt	r0, #0x4000
    416e: f44f 0100    	mov.w	r1, #0x800000
    4172: f8c5 a508    	str.w	r10, [r5, #0x508]
    4176: f8c5 6308    	str.w	r6, [r5, #0x308]
    417a: f8c5 a540    	str.w	r10, [r5, #0x540]
    417e: 6001         	str	r1, [r0]
    4180: f003 f906    	bl	0x7390 <__primask_r>    @ imm = #0x320c
    4184: 4604         	mov	r4, r0
    4186: f003 f8fc    	bl	0x7382 <__cpsid>        @ imm = #0x31f8
    418a: f04f 0b01    	mov.w	r11, #0x1
    418e: f240 20c8    	movw	r0, #0x2c8
    4192: f8c5 b008    	str.w	r11, [r5, #0x8]
    4196: f2c2 0000    	movt	r0, #0x2000
    419a: f8c5 b000    	str.w	r11, [r5]
    419e: f8c5 a104    	str.w	r10, [r5, #0x104]
    41a2: f8c5 a140    	str.w	r10, [r5, #0x140]
    41a6: f8c5 a144    	str.w	r10, [r5, #0x144]
    41aa: f3bf 8f5f    	dmb	sy
    41ae: f8c9 a048    	str.w	r10, [r9, #0x48]
    41b2: f3bf 8f5f    	dmb	sy
    41b6: f3bf 8f5f    	dmb	sy
    41ba: f880 b014    	strb.w	r11, [r0, #0x14]
    41be: 07e0         	lsls	r0, r4, #0x1f
    41c0: f3bf 8f5f    	dmb	sy
    41c4: f8c5 6304    	str.w	r6, [r5, #0x304]
    41c8: f8c5 6344    	str.w	r6, [r5, #0x344]
    41cc: bf08         	it	eq
    41ce: f003 f8da    	bleq	0x7386 <__cpsie>        @ imm = #0x31b4
    41d2: f24e 400b    	movw	r0, #0xe40b
    41d6: 21c0         	movs	r1, #0xc0
    41d8: f2ce 0000    	movt	r0, #0xe000
    41dc: f640 0908    	movw	r9, #0x808
    41e0: 7001         	strb	r1, [r0]
    41e2: f24e 2100    	movw	r1, #0xe200
    41e6: f2ce 0100    	movt	r1, #0xe000
    41ea: f06f 02ff    	mvn	r2, #0xff
    41ee: f44f 6000    	mov.w	r0, #0x800
    41f2: f640 233c    	movw	r3, #0xa3c
    41f6: 5088         	str	r0, [r1, r2]
    41f8: f246 3204    	movw	r2, #0x6304
    41fc: f2c5 0900    	movt	r9, #0x5000
    4200: 200c         	movs	r0, #0xc
    4202: f44f 7180    	mov.w	r1, #0x100
    4206: f849 0cc4    	str	r0, [r9, #-196]
    420a: f2c4 0200    	movt	r2, #0x4000
    420e: f849 0cd0    	str	r0, [r9, #-208]
    4212: f2c5 0300    	movt	r3, #0x5000
    4216: f849 0cd4    	str	r0, [r9, #-212]
    421a: f04f 0803    	mov.w	r8, #0x3
    421e: f8c9 1000    	str.w	r1, [r9]
    4222: f640 5101    	movw	r1, #0xd01
    4226: f843 8c1c    	str	r8, [r3, #-28]
    422a: f2c0 0101    	movt	r1, #0x1
    422e: 6018         	str	r0, [r3]
    4230: f501 6080    	add.w	r0, r1, #0x400
    4234: f8c2 020c    	str.w	r0, [r2, #0x20c]
    4238: 2002         	movs	r0, #0x2
    423a: f8c2 b000    	str.w	r11, [r2]
    423e: f5a2 7501    	sub.w	r5, r2, #0x204
    4242: f859 6cc4    	ldr	r6, [r9, #-196]
    4246: f640 6c01    	movw	r12, #0xe01
    424a: f2c0 0c02    	movt	r12, #0x2
    424e: 2404         	movs	r4, #0x4
    4250: f360 4611    	bfi	r6, r0, #16, #2
    4254: f849 6cc4    	str	r6, [r9, #-196]
    4258: f859 6cc4    	ldr	r6, [r9, #-196]
    425c: f446 3640    	orr	r6, r6, #0x30000
    4260: f849 6cc4    	str	r6, [r9, #-196]
    4264: f24f 5604    	movw	r6, #0xf504
    4268: f2c4 0601    	movt	r6, #0x4001
    426c: 60f5         	str	r5, [r6, #0xc]
    426e: f5a2 7541    	sub.w	r5, r2, #0x304
    4272: 6135         	str	r5, [r6, #0x10]
    4274: f50c 7540    	add.w	r5, r12, #0x300
    4278: f8c6 b000    	str.w	r11, [r6]
    427c: f246 5b14    	movw	r11, #0x6514
    4280: f2c4 0b00    	movt	r11, #0x4000
    4284: f8cb 5000    	str.w	r5, [r11]
    4288: 6010         	str	r0, [r2]
    428a: f859 5cc4    	ldr	r5, [r9, #-196]
    428e: f360 4511    	bfi	r5, r0, #16, #2
    4292: f849 5cc4    	str	r5, [r9, #-196]
    4296: f859 5cc4    	ldr	r5, [r9, #-196]
    429a: f445 3540    	orr	r5, r5, #0x30000
    429e: f849 5cc4    	str	r5, [r9, #-196]
    42a2: f5a2 7500    	sub.w	r5, r2, #0x200
    42a6: 6175         	str	r5, [r6, #0x14]
    42a8: f5a2 7540    	sub.w	r5, r2, #0x300
    42ac: 61b5         	str	r5, [r6, #0x18]
    42ae: f501 7580    	add.w	r5, r1, #0x100
    42b2: 6030         	str	r0, [r6]
    42b4: f8cb 5004    	str.w	r5, [r11, #0x4]
    42b8: 2504         	movs	r5, #0x4
    42ba: 6015         	str	r5, [r2]
    42bc: f859 5cd0    	ldr	r5, [r9, #-208]
    42c0: f360 4511    	bfi	r5, r0, #16, #2
    42c4: f849 5cd0    	str	r5, [r9, #-208]
    42c8: f859 5cd0    	ldr	r5, [r9, #-208]
    42cc: f445 3540    	orr	r5, r5, #0x30000
    42d0: f849 5cd0    	str	r5, [r9, #-208]
    42d4: f5a2 75fe    	sub.w	r5, r2, #0x1fc
    42d8: 61f5         	str	r5, [r6, #0x1c]
    42da: f5a2 753f    	sub.w	r5, r2, #0x2fc
    42de: 6235         	str	r5, [r6, #0x20]
    42e0: 2508         	movs	r5, #0x8
    42e2: 6034         	str	r4, [r6]
    42e4: f8cb c008    	str.w	r12, [r11, #0x8]
    42e8: 2408         	movs	r4, #0x8
    42ea: 6015         	str	r5, [r2]
    42ec: f859 5cd0    	ldr	r5, [r9, #-208]
    42f0: f360 4511    	bfi	r5, r0, #16, #2
    42f4: f849 5cd0    	str	r5, [r9, #-208]
    42f8: f859 5cd0    	ldr	r5, [r9, #-208]
    42fc: f445 3540    	orr	r5, r5, #0x30000
    4300: f849 5cd0    	str	r5, [r9, #-208]
    4304: f5a2 75fc    	sub.w	r5, r2, #0x1f8
    4308: 6275         	str	r5, [r6, #0x24]
    430a: f5a2 753e    	sub.w	r5, r2, #0x2f8
    430e: 62b5         	str	r5, [r6, #0x28]
    4310: 2510         	movs	r5, #0x10
    4312: 6034         	str	r4, [r6]
    4314: f8cb 100c    	str.w	r1, [r11, #0xc]
    4318: f501 3180    	add.w	r1, r1, #0x10000
    431c: 6015         	str	r5, [r2]
    431e: f859 ecd4    	ldr	lr, [r9, #-212]
    4322: f360 4e11    	bfi	lr, r0, #16, #2
    4326: f849 ecd4    	str	lr, [r9, #-212]
    432a: f859 4cd4    	ldr	r4, [r9, #-212]
    432e: f444 3440    	orr	r4, r4, #0x30000
    4332: f849 4cd4    	str	r4, [r9, #-212]
    4336: f5a2 74fa    	sub.w	r4, r2, #0x1f4
    433a: 62f4         	str	r4, [r6, #0x2c]
    433c: f5a2 743d    	sub.w	r4, r2, #0x2f4
    4340: 6334         	str	r4, [r6, #0x30]
    4342: 6035         	str	r5, [r6]
    4344: f8cb 1010    	str.w	r1, [r11, #0x10]
    4348: 2120         	movs	r1, #0x20
    434a: 6011         	str	r1, [r2]
    434c: f859 5cd4    	ldr	r5, [r9, #-212]
    4350: f360 4511    	bfi	r5, r0, #16, #2
    4354: f849 5cd4    	str	r5, [r9, #-212]
    4358: f859 5cd4    	ldr	r5, [r9, #-212]
    435c: f445 3540    	orr	r5, r5, #0x30000
    4360: f849 5cd4    	str	r5, [r9, #-212]
    4364: f5a2 75f8    	sub.w	r5, r2, #0x1f0
    4368: 6375         	str	r5, [r6, #0x34]
    436a: f5a2 753c    	sub.w	r5, r2, #0x2f0
    436e: 63b5         	str	r5, [r6, #0x38]
    4370: f50c 5504    	add.w	r5, r12, #0x2100
    4374: 6031         	str	r1, [r6]
    4376: f8cb 5014    	str.w	r5, [r11, #0x14]
    437a: 2540         	movs	r5, #0x40
    437c: 6015         	str	r5, [r2]
    437e: 681c         	ldr	r4, [r3]
    4380: f360 4411    	bfi	r4, r0, #16, #2
    4384: 601c         	str	r4, [r3]
    4386: 681c         	ldr	r4, [r3]
    4388: f444 3440    	orr	r4, r4, #0x30000
    438c: 601c         	str	r4, [r3]
    438e: f5a2 73f6    	sub.w	r3, r2, #0x1ec
    4392: 63f3         	str	r3, [r6, #0x3c]
    4394: f5a2 733b    	sub.w	r3, r2, #0x2ec
    4398: 6433         	str	r3, [r6, #0x40]
    439a: 6035         	str	r5, [r6]
    439c: f64f 5604    	movw	r6, #0xfd04
    43a0: f6cf 76ff    	movt	r6, #0xffff
    43a4: f44f 2300    	mov.w	r3, #0x80000
    43a8: f849 3006    	str.w	r3, [r9, r6]
    43ac: f44f 1380    	mov.w	r3, #0x100000
    43b0: f44f 0500    	mov.w	r5, #0x800000
    43b4: f849 8cbc    	str	r8, [r9, #-188]
    43b8: 241c         	movs	r4, #0x1c
    43ba: f849 3006    	str.w	r3, [r9, r6]
    43be: f44f 1300    	mov.w	r3, #0x200000
    43c2: f849 8cb8    	str	r8, [r9, #-184]
    43c6: f849 3006    	str.w	r3, [r9, r6]
    43ca: f44f 0380    	mov.w	r3, #0x400000
    43ce: f849 8cb4    	str	r8, [r9, #-180]
    43d2: f849 3006    	str.w	r3, [r9, r6]
    43d6: f04f 7380    	mov.w	r3, #0x1000000
    43da: f849 8cb0    	str	r8, [r9, #-176]
    43de: f849 3006    	str.w	r3, [r9, r6]
    43e2: f849 8ca8    	str	r8, [r9, #-168]
    43e6: f849 5006    	str.w	r5, [r9, r6]
    43ea: f243 1508    	movw	r5, #0x3108
    43ee: f2c4 0500    	movt	r5, #0x4000
    43f2: f849 8cac    	str	r8, [r9, #-172]
    43f6: f849 1006    	str.w	r1, [r9, r6]
    43fa: 2104         	movs	r1, #0x4
    43fc: f849 8cf4    	str	r8, [r9, #-244]
    4400: f849 1006    	str.w	r1, [r9, r6]
    4404: f243 5618    	movw	r6, #0x3518
    4408: f06f 01ff    	mvn	r1, #0xff
    440c: f849 8001    	str.w	r8, [r9, r1]
    4410: 2105         	movs	r1, #0x5
    4412: f2c4 0600    	movt	r6, #0x4000
    4416: f849 ac98    	str	r10, [r9, #-152]
    441a: f8c5 1400    	str.w	r1, [r5, #0x400]
    441e: f846 0c0c    	str	r0, [r6, #-12]
    4422: f846 4c08    	str	r4, [r6, #-8]
    4426: 2401         	movs	r4, #0x1
    4428: f8c5 43f8    	str.w	r4, [r5, #0x3f8]
    442c: f04f 5480    	mov.w	r4, #0x10000000
    4430: f8c6 a03c    	str.w	r10, [r6, #0x3c]
    4434: 60f4         	str	r4, [r6, #0xc]
    4436: f04f 4400    	mov.w	r4, #0x80000000
    443a: 6014         	str	r4, [r2]
    443c: f247 5200    	movw	r2, #0x7500
    4440: f2c4 0200    	movt	r2, #0x4000
    4444: f8c2 80f0    	str.w	r8, [r2, #0xf0]
    4448: f8c2 10f4    	str.w	r1, [r2, #0xf4]
    444c: 2108         	movs	r1, #0x8
    444e: f8c2 a0f8    	str.w	r10, [r2, #0xf8]
    4452: 6193         	str	r3, [r2, #0x18]
    4454: 6111         	str	r1, [r2, #0x10]
    4456: 2107         	movs	r1, #0x7
    4458: f8c2 a014    	str.w	r10, [r2, #0x14]
    445c: 6293         	str	r3, [r2, #0x28]
    445e: 6211         	str	r1, [r2, #0x20]
    4460: 2106         	movs	r1, #0x6
    4462: f8c2 a024    	str.w	r10, [r2, #0x24]
    4466: 6393         	str	r3, [r2, #0x38]
    4468: 6311         	str	r1, [r2, #0x30]
    446a: f247 010c    	movw	r1, #0x700c
    446e: f8c2 a034    	str.w	r10, [r2, #0x34]
    4472: f2c4 0100    	movt	r1, #0x4000
    4476: 6493         	str	r3, [r2, #0x48]
    4478: 2301         	movs	r3, #0x1
    447a: f8c2 8040    	str.w	r8, [r2, #0x40]
    447e: f8c2 a044    	str.w	r10, [r2, #0x44]
    4482: 6013         	str	r3, [r2]
    4484: f8c1 a104    	str.w	r10, [r1, #0x104]
    4488: 600b         	str	r3, [r1]
    448a: f8c1 02f4    	str.w	r0, [r1, #0x2f4]
    448e: f8c1 02f8    	str.w	r0, [r1, #0x2f8]
    4492: f002 fcf8    	bl	0x6e86 <_defmt_acquire> @ imm = #0x29f0
    4496: f240 080c    	movw	r8, #0xc
    449a: f2c0 0800    	movt	r8, #0x0
    449e: 4640         	mov	r0, r8
    44a0: f002 fc76    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x28ec
    44a4: f240 0402    	movw	r4, #0x2
    44a8: a806         	add	r0, sp, #0x18
    44aa: f2c0 0400    	movt	r4, #0x0
    44ae: 2102         	movs	r1, #0x2
    44b0: f8ad 4018    	strh.w	r4, [sp, #0x18]
    44b4: f002 fdb3    	bl	0x701e <_defmt_write>   @ imm = #0x2b66
    44b8: a806         	add	r0, sp, #0x18
    44ba: 2102         	movs	r1, #0x2
    44bc: f8ad a018    	strh.w	r10, [sp, #0x18]
    44c0: f002 fdad    	bl	0x701e <_defmt_write>   @ imm = #0x2b5a
    44c4: a806         	add	r0, sp, #0x18
    44c6: 2102         	movs	r1, #0x2
    44c8: f8ad 4018    	strh.w	r4, [sp, #0x18]
    44cc: f002 fda7    	bl	0x701e <_defmt_write>   @ imm = #0x2b4e
    44d0: a806         	add	r0, sp, #0x18
    44d2: f240 7bff    	movw	r11, #0x7ff
    44d6: 2102         	movs	r1, #0x2
    44d8: f8ad b018    	strh.w	r11, [sp, #0x18]
    44dc: f002 fd9f    	bl	0x701e <_defmt_write>   @ imm = #0x2b3e
    44e0: f002 fd26    	bl	0x6f30 <_defmt_release> @ imm = #0x2a4c
    44e4: f002 fccf    	bl	0x6e86 <_defmt_acquire> @ imm = #0x299e
    44e8: 4640         	mov	r0, r8
    44ea: f002 fc51    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x28a2
    44ee: a806         	add	r0, sp, #0x18
    44f0: 2102         	movs	r1, #0x2
    44f2: f8ad 4018    	strh.w	r4, [sp, #0x18]
    44f6: f002 fd92    	bl	0x701e <_defmt_write>   @ imm = #0x2b24
    44fa: a806         	add	r0, sp, #0x18
    44fc: 2102         	movs	r1, #0x2
    44fe: f8ad a018    	strh.w	r10, [sp, #0x18]
    4502: f002 fd8c    	bl	0x701e <_defmt_write>   @ imm = #0x2b18
    4506: a806         	add	r0, sp, #0x18
    4508: 2102         	movs	r1, #0x2
    450a: f8ad 4018    	strh.w	r4, [sp, #0x18]
    450e: f002 fd86    	bl	0x701e <_defmt_write>   @ imm = #0x2b0c
    4512: a806         	add	r0, sp, #0x18
    4514: 2102         	movs	r1, #0x2
    4516: f8ad b018    	strh.w	r11, [sp, #0x18]
    451a: f002 fd80    	bl	0x701e <_defmt_write>   @ imm = #0x2b00
    451e: f002 fd07    	bl	0x6f30 <_defmt_release> @ imm = #0x2a0e
    4522: f002 fcb0    	bl	0x6e86 <_defmt_acquire> @ imm = #0x2960
    4526: f240 000f    	movw	r0, #0xf
    452a: f2c0 0000    	movt	r0, #0x0
    452e: f002 fc2f    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x285e
    4532: f002 fcfd    	bl	0x6f30 <_defmt_release> @ imm = #0x29fa
    4536: f002 fca6    	bl	0x6e86 <_defmt_acquire> @ imm = #0x294c
    453a: f240 000e    	movw	r0, #0xe
    453e: f2c0 0000    	movt	r0, #0x0
    4542: f002 fc25    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x284a
    4546: f002 fcf3    	bl	0x6f30 <_defmt_release> @ imm = #0x29e6
    454a: f44f 7280    	mov.w	r2, #0x100
    454e: 20c0         	movs	r0, #0xc0
    4550: f8c9 2004    	str.w	r2, [r9, #0x4]
    4554: 6070         	str	r0, [r6, #0x4]
    4556: e008         	b	0x456a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x48e> @ imm = #0x10
    4558: bf10         	yield
    455a: 6828         	ldr	r0, [r5]
    455c: 2800         	cmp	r0, #0x0
    455e: bf02         	ittt	eq
    4560: bf10         	yieldeq
    4562: 6828         	ldreq	r0, [r5]
    4564: 2800         	cmpeq	r0, #0x0
    4566: d107         	bne	0x4578 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x49c> @ imm = #0xe
    4568: bf10         	yield
    456a: 6828         	ldr	r0, [r5]
    456c: 2800         	cmp	r0, #0x0
    456e: bf02         	ittt	eq
    4570: bf10         	yieldeq
    4572: 6828         	ldreq	r0, [r5]
    4574: 2800         	cmpeq	r0, #0x0
    4576: d0ef         	beq	0x4558 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x47c> @ imm = #-0x22
    4578: 6831         	ldr	r1, [r6]
    457a: 2000         	movs	r0, #0x0
    457c: 2103         	movs	r1, #0x3
    457e: 6028         	str	r0, [r5]
    4580: f8c9 2000    	str.w	r2, [r9]
    4584: f8c9 2004    	str.w	r2, [r9, #0x4]
    4588: 6071         	str	r1, [r6, #0x4]
    458a: e008         	b	0x459e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x4c2> @ imm = #0x10
    458c: bf10         	yield
    458e: 6829         	ldr	r1, [r5]
    4590: 2900         	cmp	r1, #0x0
    4592: bf02         	ittt	eq
    4594: bf10         	yieldeq
    4596: 6829         	ldreq	r1, [r5]
    4598: 2900         	cmpeq	r1, #0x0
    459a: d107         	bne	0x45ac <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x4d0> @ imm = #0xe
    459c: bf10         	yield
    459e: 6829         	ldr	r1, [r5]
    45a0: 2900         	cmp	r1, #0x0
    45a2: bf02         	ittt	eq
    45a4: bf10         	yieldeq
    45a6: 6829         	ldreq	r1, [r5]
    45a8: 2900         	cmpeq	r1, #0x0
    45aa: d0ef         	beq	0x458c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x4b0> @ imm = #-0x22
    45ac: 6831         	ldr	r1, [r6]
    45ae: 6028         	str	r0, [r5]
    45b0: 200e         	movs	r0, #0xe
    45b2: 6070         	str	r0, [r6, #0x4]
    45b4: e008         	b	0x45c8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x4ec> @ imm = #0x10
    45b6: bf10         	yield
    45b8: 6828         	ldr	r0, [r5]
    45ba: 2800         	cmp	r0, #0x0
    45bc: bf02         	ittt	eq
    45be: bf10         	yieldeq
    45c0: 6828         	ldreq	r0, [r5]
    45c2: 2800         	cmpeq	r0, #0x0
    45c4: d107         	bne	0x45d6 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x4fa> @ imm = #0xe
    45c6: bf10         	yield
    45c8: 6828         	ldr	r0, [r5]
    45ca: 2800         	cmp	r0, #0x0
    45cc: bf02         	ittt	eq
    45ce: bf10         	yieldeq
    45d0: 6828         	ldreq	r0, [r5]
    45d2: 2800         	cmpeq	r0, #0x0
    45d4: d0ef         	beq	0x45b6 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x4da> @ imm = #-0x22
    45d6: 2000         	movs	r0, #0x0
    45d8: 6831         	ldr	r1, [r6]
    45da: 6028         	str	r0, [r5]
    45dc: 6070         	str	r0, [r6, #0x4]
    45de: e008         	b	0x45f2 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x516> @ imm = #0x10
    45e0: bf10         	yield
    45e2: 6829         	ldr	r1, [r5]
    45e4: 2900         	cmp	r1, #0x0
    45e6: bf02         	ittt	eq
    45e8: bf10         	yieldeq
    45ea: 6829         	ldreq	r1, [r5]
    45ec: 2900         	cmpeq	r1, #0x0
    45ee: d107         	bne	0x4600 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x524> @ imm = #0xe
    45f0: bf10         	yield
    45f2: 6829         	ldr	r1, [r5]
    45f4: 2900         	cmp	r1, #0x0
    45f6: bf02         	ittt	eq
    45f8: bf10         	yieldeq
    45fa: 6829         	ldreq	r1, [r5]
    45fc: 2900         	cmpeq	r1, #0x0
    45fe: d0ef         	beq	0x45e0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x504> @ imm = #-0x22
    4600: 6831         	ldr	r1, [r6]
    4602: 9102         	str	r1, [sp, #0x8]
    4604: 6028         	str	r0, [r5]
    4606: f44f 7080    	mov.w	r0, #0x100
    460a: f8c9 0000    	str.w	r0, [r9]
    460e: f8c9 0004    	str.w	r0, [r9, #0x4]
    4612: 2002         	movs	r0, #0x2
    4614: 6070         	str	r0, [r6, #0x4]
    4616: e008         	b	0x462a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x54e> @ imm = #0x10
    4618: bf10         	yield
    461a: 6828         	ldr	r0, [r5]
    461c: 2800         	cmp	r0, #0x0
    461e: bf02         	ittt	eq
    4620: bf10         	yieldeq
    4622: 6828         	ldreq	r0, [r5]
    4624: 2800         	cmpeq	r0, #0x0
    4626: d107         	bne	0x4638 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x55c> @ imm = #0xe
    4628: bf10         	yield
    462a: 6828         	ldr	r0, [r5]
    462c: 2800         	cmp	r0, #0x0
    462e: bf02         	ittt	eq
    4630: bf10         	yieldeq
    4632: 6828         	ldreq	r0, [r5]
    4634: 2800         	cmpeq	r0, #0x0
    4636: d0ef         	beq	0x4618 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x53c> @ imm = #-0x22
    4638: 6831         	ldr	r1, [r6]
    463a: 2000         	movs	r0, #0x0
    463c: 210f         	movs	r1, #0xf
    463e: 6028         	str	r0, [r5]
    4640: 6071         	str	r1, [r6, #0x4]
    4642: e008         	b	0x4656 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x57a> @ imm = #0x10
    4644: bf10         	yield
    4646: 6829         	ldr	r1, [r5]
    4648: 2900         	cmp	r1, #0x0
    464a: bf02         	ittt	eq
    464c: bf10         	yieldeq
    464e: 6829         	ldreq	r1, [r5]
    4650: 2900         	cmpeq	r1, #0x0
    4652: d107         	bne	0x4664 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x588> @ imm = #0xe
    4654: bf10         	yield
    4656: 6829         	ldr	r1, [r5]
    4658: 2900         	cmp	r1, #0x0
    465a: bf02         	ittt	eq
    465c: bf10         	yieldeq
    465e: 6829         	ldreq	r1, [r5]
    4660: 2900         	cmpeq	r1, #0x0
    4662: d0ef         	beq	0x4644 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x568> @ imm = #-0x22
    4664: 6831         	ldr	r1, [r6]
    4666: 6028         	str	r0, [r5]
    4668: 2084         	movs	r0, #0x84
    466a: 6070         	str	r0, [r6, #0x4]
    466c: e008         	b	0x4680 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x5a4> @ imm = #0x10
    466e: bf10         	yield
    4670: 6828         	ldr	r0, [r5]
    4672: 2800         	cmp	r0, #0x0
    4674: bf02         	ittt	eq
    4676: bf10         	yieldeq
    4678: 6828         	ldreq	r0, [r5]
    467a: 2800         	cmpeq	r0, #0x0
    467c: d107         	bne	0x468e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x5b2> @ imm = #0xe
    467e: bf10         	yield
    4680: 6828         	ldr	r0, [r5]
    4682: 2800         	cmp	r0, #0x0
    4684: bf02         	ittt	eq
    4686: bf10         	yieldeq
    4688: 6828         	ldreq	r0, [r5]
    468a: 2800         	cmpeq	r0, #0x0
    468c: d0ef         	beq	0x466e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x592> @ imm = #-0x22
    468e: 6831         	ldr	r1, [r6]
    4690: 2000         	movs	r0, #0x0
    4692: f44f 7180    	mov.w	r1, #0x100
    4696: 6028         	str	r0, [r5]
    4698: f8c9 1000    	str.w	r1, [r9]
    469c: f8c9 1004    	str.w	r1, [r9, #0x4]
    46a0: 2102         	movs	r1, #0x2
    46a2: 6071         	str	r1, [r6, #0x4]
    46a4: e008         	b	0x46b8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x5dc> @ imm = #0x10
    46a6: bf10         	yield
    46a8: 6829         	ldr	r1, [r5]
    46aa: 2900         	cmp	r1, #0x0
    46ac: bf02         	ittt	eq
    46ae: bf10         	yieldeq
    46b0: 6829         	ldreq	r1, [r5]
    46b2: 2900         	cmpeq	r1, #0x0
    46b4: d107         	bne	0x46c6 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x5ea> @ imm = #0xe
    46b6: bf10         	yield
    46b8: 6829         	ldr	r1, [r5]
    46ba: 2900         	cmp	r1, #0x0
    46bc: bf02         	ittt	eq
    46be: bf10         	yieldeq
    46c0: 6829         	ldreq	r1, [r5]
    46c2: 2900         	cmpeq	r1, #0x0
    46c4: d0ef         	beq	0x46a6 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x5ca> @ imm = #-0x22
    46c6: 6831         	ldr	r1, [r6]
    46c8: 6028         	str	r0, [r5]
    46ca: 200d         	movs	r0, #0xd
    46cc: 6070         	str	r0, [r6, #0x4]
    46ce: e008         	b	0x46e2 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x606> @ imm = #0x10
    46d0: bf10         	yield
    46d2: 6828         	ldr	r0, [r5]
    46d4: 2800         	cmp	r0, #0x0
    46d6: bf02         	ittt	eq
    46d8: bf10         	yieldeq
    46da: 6828         	ldreq	r0, [r5]
    46dc: 2800         	cmpeq	r0, #0x0
    46de: d107         	bne	0x46f0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x614> @ imm = #0xe
    46e0: bf10         	yield
    46e2: 6828         	ldr	r0, [r5]
    46e4: 2800         	cmp	r0, #0x0
    46e6: bf02         	ittt	eq
    46e8: bf10         	yieldeq
    46ea: 6828         	ldreq	r0, [r5]
    46ec: 2800         	cmpeq	r0, #0x0
    46ee: d0ef         	beq	0x46d0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x5f4> @ imm = #-0x22
    46f0: 6831         	ldr	r1, [r6]
    46f2: 2000         	movs	r0, #0x0
    46f4: 2103         	movs	r1, #0x3
    46f6: 6028         	str	r0, [r5]
    46f8: 6071         	str	r1, [r6, #0x4]
    46fa: e008         	b	0x470e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x632> @ imm = #0x10
    46fc: bf10         	yield
    46fe: 6829         	ldr	r1, [r5]
    4700: 2900         	cmp	r1, #0x0
    4702: bf02         	ittt	eq
    4704: bf10         	yieldeq
    4706: 6829         	ldreq	r1, [r5]
    4708: 2900         	cmpeq	r1, #0x0
    470a: d107         	bne	0x471c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x640> @ imm = #0xe
    470c: bf10         	yield
    470e: 6829         	ldr	r1, [r5]
    4710: 2900         	cmp	r1, #0x0
    4712: bf02         	ittt	eq
    4714: bf10         	yieldeq
    4716: 6829         	ldreq	r1, [r5]
    4718: 2900         	cmpeq	r1, #0x0
    471a: d0ef         	beq	0x46fc <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x620> @ imm = #-0x22
    471c: 6831         	ldr	r1, [r6]
    471e: 6028         	str	r0, [r5]
    4720: f44f 7080    	mov.w	r0, #0x100
    4724: f8c9 0000    	str.w	r0, [r9]
    4728: f8c9 0004    	str.w	r0, [r9, #0x4]
    472c: 2002         	movs	r0, #0x2
    472e: 6070         	str	r0, [r6, #0x4]
    4730: e008         	b	0x4744 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x668> @ imm = #0x10
    4732: bf10         	yield
    4734: 6828         	ldr	r0, [r5]
    4736: 2800         	cmp	r0, #0x0
    4738: bf02         	ittt	eq
    473a: bf10         	yieldeq
    473c: 6828         	ldreq	r0, [r5]
    473e: 2800         	cmpeq	r0, #0x0
    4740: d107         	bne	0x4752 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x676> @ imm = #0xe
    4742: bf10         	yield
    4744: 6828         	ldr	r0, [r5]
    4746: 2800         	cmp	r0, #0x0
    4748: bf02         	ittt	eq
    474a: bf10         	yieldeq
    474c: 6828         	ldreq	r0, [r5]
    474e: 2800         	cmpeq	r0, #0x0
    4750: d0ef         	beq	0x4732 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x656> @ imm = #-0x22
    4752: 6831         	ldr	r1, [r6]
    4754: 2000         	movs	r0, #0x0
    4756: 210c         	movs	r1, #0xc
    4758: 6028         	str	r0, [r5]
    475a: 6071         	str	r1, [r6, #0x4]
    475c: e008         	b	0x4770 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x694> @ imm = #0x10
    475e: bf10         	yield
    4760: 6829         	ldr	r1, [r5]
    4762: 2900         	cmp	r1, #0x0
    4764: bf02         	ittt	eq
    4766: bf10         	yieldeq
    4768: 6829         	ldreq	r1, [r5]
    476a: 2900         	cmpeq	r1, #0x0
    476c: d107         	bne	0x477e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x6a2> @ imm = #0xe
    476e: bf10         	yield
    4770: 6829         	ldr	r1, [r5]
    4772: 2900         	cmp	r1, #0x0
    4774: bf02         	ittt	eq
    4776: bf10         	yieldeq
    4778: 6829         	ldreq	r1, [r5]
    477a: 2900         	cmpeq	r1, #0x0
    477c: d0ef         	beq	0x475e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x682> @ imm = #-0x22
    477e: 6831         	ldr	r1, [r6]
    4780: 6028         	str	r0, [r5]
    4782: 2003         	movs	r0, #0x3
    4784: 6070         	str	r0, [r6, #0x4]
    4786: e008         	b	0x479a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x6be> @ imm = #0x10
    4788: bf10         	yield
    478a: 6829         	ldr	r1, [r5]
    478c: 2900         	cmp	r1, #0x0
    478e: bf02         	ittt	eq
    4790: bf10         	yieldeq
    4792: 6829         	ldreq	r1, [r5]
    4794: 2900         	cmpeq	r1, #0x0
    4796: d107         	bne	0x47a8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x6cc> @ imm = #0xe
    4798: bf10         	yield
    479a: 6829         	ldr	r1, [r5]
    479c: 2900         	cmp	r1, #0x0
    479e: bf02         	ittt	eq
    47a0: bf10         	yieldeq
    47a2: 6829         	ldreq	r1, [r5]
    47a4: 2900         	cmpeq	r1, #0x0
    47a6: d0ef         	beq	0x4788 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x6ac> @ imm = #-0x22
    47a8: 6832         	ldr	r2, [r6]
    47aa: 2100         	movs	r1, #0x0
    47ac: f44f 7280    	mov.w	r2, #0x100
    47b0: 6029         	str	r1, [r5]
    47b2: f8c9 2000    	str.w	r2, [r9]
    47b6: f8c9 2004    	str.w	r2, [r9, #0x4]
    47ba: 6070         	str	r0, [r6, #0x4]
    47bc: e008         	b	0x47d0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x6f4> @ imm = #0x10
    47be: bf10         	yield
    47c0: 6828         	ldr	r0, [r5]
    47c2: 2800         	cmp	r0, #0x0
    47c4: bf02         	ittt	eq
    47c6: bf10         	yieldeq
    47c8: 6828         	ldreq	r0, [r5]
    47ca: 2800         	cmpeq	r0, #0x0
    47cc: d107         	bne	0x47de <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x702> @ imm = #0xe
    47ce: bf10         	yield
    47d0: 6828         	ldr	r0, [r5]
    47d2: 2800         	cmp	r0, #0x0
    47d4: bf02         	ittt	eq
    47d6: bf10         	yieldeq
    47d8: 6828         	ldreq	r0, [r5]
    47da: 2800         	cmpeq	r0, #0x0
    47dc: d0ef         	beq	0x47be <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x6e2> @ imm = #-0x22
    47de: 6830         	ldr	r0, [r6]
    47e0: 200f         	movs	r0, #0xf
    47e2: 6029         	str	r1, [r5]
    47e4: 6070         	str	r0, [r6, #0x4]
    47e6: e008         	b	0x47fa <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x71e> @ imm = #0x10
    47e8: bf10         	yield
    47ea: 6828         	ldr	r0, [r5]
    47ec: 2800         	cmp	r0, #0x0
    47ee: bf02         	ittt	eq
    47f0: bf10         	yieldeq
    47f2: 6828         	ldreq	r0, [r5]
    47f4: 2800         	cmpeq	r0, #0x0
    47f6: d107         	bne	0x4808 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x72c> @ imm = #0xe
    47f8: bf10         	yield
    47fa: 6828         	ldr	r0, [r5]
    47fc: 2800         	cmp	r0, #0x0
    47fe: bf02         	ittt	eq
    4800: bf10         	yieldeq
    4802: 6828         	ldreq	r0, [r5]
    4804: 2800         	cmpeq	r0, #0x0
    4806: d0ef         	beq	0x47e8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x70c> @ imm = #-0x22
    4808: 2000         	movs	r0, #0x0
    480a: 6831         	ldr	r1, [r6]
    480c: 6028         	str	r0, [r5]
    480e: 6070         	str	r0, [r6, #0x4]
    4810: e008         	b	0x4824 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x748> @ imm = #0x10
    4812: bf10         	yield
    4814: 6829         	ldr	r1, [r5]
    4816: 2900         	cmp	r1, #0x0
    4818: bf02         	ittt	eq
    481a: bf10         	yieldeq
    481c: 6829         	ldreq	r1, [r5]
    481e: 2900         	cmpeq	r1, #0x0
    4820: d107         	bne	0x4832 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x756> @ imm = #0xe
    4822: bf10         	yield
    4824: 6829         	ldr	r1, [r5]
    4826: 2900         	cmp	r1, #0x0
    4828: bf02         	ittt	eq
    482a: bf10         	yieldeq
    482c: 6829         	ldreq	r1, [r5]
    482e: 2900         	cmpeq	r1, #0x0
    4830: d0ef         	beq	0x4812 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x736> @ imm = #-0x22
    4832: 6831         	ldr	r1, [r6]
    4834: 9101         	str	r1, [sp, #0x4]
    4836: 6028         	str	r0, [r5]
    4838: f44f 7080    	mov.w	r0, #0x100
    483c: f8c9 0000    	str.w	r0, [r9]
    4840: f8c9 0004    	str.w	r0, [r9, #0x4]
    4844: 2003         	movs	r0, #0x3
    4846: 6070         	str	r0, [r6, #0x4]
    4848: e008         	b	0x485c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x780> @ imm = #0x10
    484a: bf10         	yield
    484c: 6828         	ldr	r0, [r5]
    484e: 2800         	cmp	r0, #0x0
    4850: bf02         	ittt	eq
    4852: bf10         	yieldeq
    4854: 6828         	ldreq	r0, [r5]
    4856: 2800         	cmpeq	r0, #0x0
    4858: d107         	bne	0x486a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x78e> @ imm = #0xe
    485a: bf10         	yield
    485c: 6828         	ldr	r0, [r5]
    485e: 2800         	cmp	r0, #0x0
    4860: bf02         	ittt	eq
    4862: bf10         	yieldeq
    4864: 6828         	ldreq	r0, [r5]
    4866: 2800         	cmpeq	r0, #0x0
    4868: d0ef         	beq	0x484a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x76e> @ imm = #-0x22
    486a: 6831         	ldr	r1, [r6]
    486c: 2000         	movs	r0, #0x0
    486e: 210d         	movs	r1, #0xd
    4870: 6028         	str	r0, [r5]
    4872: 6071         	str	r1, [r6, #0x4]
    4874: e008         	b	0x4888 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x7ac> @ imm = #0x10
    4876: bf10         	yield
    4878: 6829         	ldr	r1, [r5]
    487a: 2900         	cmp	r1, #0x0
    487c: bf02         	ittt	eq
    487e: bf10         	yieldeq
    4880: 6829         	ldreq	r1, [r5]
    4882: 2900         	cmpeq	r1, #0x0
    4884: d107         	bne	0x4896 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x7ba> @ imm = #0xe
    4886: bf10         	yield
    4888: 6829         	ldr	r1, [r5]
    488a: 2900         	cmp	r1, #0x0
    488c: bf02         	ittt	eq
    488e: bf10         	yieldeq
    4890: 6829         	ldreq	r1, [r5]
    4892: 2900         	cmpeq	r1, #0x0
    4894: d0ef         	beq	0x4876 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x79a> @ imm = #-0x22
    4896: 6831         	ldr	r1, [r6]
    4898: 6028         	str	r0, [r5]
    489a: 6070         	str	r0, [r6, #0x4]
    489c: e008         	b	0x48b0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x7d4> @ imm = #0x10
    489e: bf10         	yield
    48a0: 6828         	ldr	r0, [r5]
    48a2: 2800         	cmp	r0, #0x0
    48a4: bf02         	ittt	eq
    48a6: bf10         	yieldeq
    48a8: 6828         	ldreq	r0, [r5]
    48aa: 2800         	cmpeq	r0, #0x0
    48ac: d107         	bne	0x48be <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x7e2> @ imm = #0xe
    48ae: bf10         	yield
    48b0: 6828         	ldr	r0, [r5]
    48b2: 2800         	cmp	r0, #0x0
    48b4: bf02         	ittt	eq
    48b6: bf10         	yieldeq
    48b8: 6828         	ldreq	r0, [r5]
    48ba: 2800         	cmpeq	r0, #0x0
    48bc: d0ef         	beq	0x489e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x7c2> @ imm = #-0x22
    48be: 6831         	ldr	r1, [r6]
    48c0: 2000         	movs	r0, #0x0
    48c2: 9100         	str	r1, [sp]
    48c4: f44f 7180    	mov.w	r1, #0x100
    48c8: 6028         	str	r0, [r5]
    48ca: f8c9 1000    	str.w	r1, [r9]
    48ce: f8c9 1004    	str.w	r1, [r9, #0x4]
    48d2: 2103         	movs	r1, #0x3
    48d4: 6071         	str	r1, [r6, #0x4]
    48d6: e008         	b	0x48ea <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x80e> @ imm = #0x10
    48d8: bf10         	yield
    48da: 6829         	ldr	r1, [r5]
    48dc: 2900         	cmp	r1, #0x0
    48de: bf02         	ittt	eq
    48e0: bf10         	yieldeq
    48e2: 6829         	ldreq	r1, [r5]
    48e4: 2900         	cmpeq	r1, #0x0
    48e6: d107         	bne	0x48f8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x81c> @ imm = #0xe
    48e8: bf10         	yield
    48ea: 6829         	ldr	r1, [r5]
    48ec: 2900         	cmp	r1, #0x0
    48ee: bf02         	ittt	eq
    48f0: bf10         	yieldeq
    48f2: 6829         	ldreq	r1, [r5]
    48f4: 2900         	cmpeq	r1, #0x0
    48f6: d0ef         	beq	0x48d8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x7fc> @ imm = #-0x22
    48f8: 6831         	ldr	r1, [r6]
    48fa: 6028         	str	r0, [r5]
    48fc: 200c         	movs	r0, #0xc
    48fe: 6070         	str	r0, [r6, #0x4]
    4900: e008         	b	0x4914 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x838> @ imm = #0x10
    4902: bf10         	yield
    4904: 6828         	ldr	r0, [r5]
    4906: 2800         	cmp	r0, #0x0
    4908: bf02         	ittt	eq
    490a: bf10         	yieldeq
    490c: 6828         	ldreq	r0, [r5]
    490e: 2800         	cmpeq	r0, #0x0
    4910: d107         	bne	0x4922 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x846> @ imm = #0xe
    4912: bf10         	yield
    4914: 6828         	ldr	r0, [r5]
    4916: 2800         	cmp	r0, #0x0
    4918: bf02         	ittt	eq
    491a: bf10         	yieldeq
    491c: 6828         	ldreq	r0, [r5]
    491e: 2800         	cmpeq	r0, #0x0
    4920: d0ef         	beq	0x4902 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x826> @ imm = #-0x22
    4922: 2000         	movs	r0, #0x0
    4924: 6831         	ldr	r1, [r6]
    4926: 6028         	str	r0, [r5]
    4928: 6070         	str	r0, [r6, #0x4]
    492a: e008         	b	0x493e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x862> @ imm = #0x10
    492c: bf10         	yield
    492e: 6829         	ldr	r1, [r5]
    4930: 2900         	cmp	r1, #0x0
    4932: bf02         	ittt	eq
    4934: bf10         	yieldeq
    4936: 6829         	ldreq	r1, [r5]
    4938: 2900         	cmpeq	r1, #0x0
    493a: d107         	bne	0x494c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x870> @ imm = #0xe
    493c: bf10         	yield
    493e: 6829         	ldr	r1, [r5]
    4940: 2900         	cmp	r1, #0x0
    4942: bf02         	ittt	eq
    4944: bf10         	yieldeq
    4946: 6829         	ldreq	r1, [r5]
    4948: 2900         	cmpeq	r1, #0x0
    494a: d0ef         	beq	0x492c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x850> @ imm = #-0x22
    494c: 6834         	ldr	r4, [r6]
    494e: 6028         	str	r0, [r5]
    4950: f44f 7080    	mov.w	r0, #0x100
    4954: f8c9 0000    	str.w	r0, [r9]
    4958: f8c9 0004    	str.w	r0, [r9, #0x4]
    495c: 2003         	movs	r0, #0x3
    495e: 6070         	str	r0, [r6, #0x4]
    4960: e008         	b	0x4974 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x898> @ imm = #0x10
    4962: bf10         	yield
    4964: 6828         	ldr	r0, [r5]
    4966: 2800         	cmp	r0, #0x0
    4968: bf02         	ittt	eq
    496a: bf10         	yieldeq
    496c: 6828         	ldreq	r0, [r5]
    496e: 2800         	cmpeq	r0, #0x0
    4970: d107         	bne	0x4982 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x8a6> @ imm = #0xe
    4972: bf10         	yield
    4974: 6828         	ldr	r0, [r5]
    4976: 2800         	cmp	r0, #0x0
    4978: bf02         	ittt	eq
    497a: bf10         	yieldeq
    497c: 6828         	ldreq	r0, [r5]
    497e: 2800         	cmpeq	r0, #0x0
    4980: d0ef         	beq	0x4962 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x886> @ imm = #-0x22
    4982: 6831         	ldr	r1, [r6]
    4984: 2000         	movs	r0, #0x0
    4986: 210e         	movs	r1, #0xe
    4988: 6028         	str	r0, [r5]
    498a: 6071         	str	r1, [r6, #0x4]
    498c: e008         	b	0x49a0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x8c4> @ imm = #0x10
    498e: bf10         	yield
    4990: 6829         	ldr	r1, [r5]
    4992: 2900         	cmp	r1, #0x0
    4994: bf02         	ittt	eq
    4996: bf10         	yieldeq
    4998: 6829         	ldreq	r1, [r5]
    499a: 2900         	cmpeq	r1, #0x0
    499c: d107         	bne	0x49ae <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x8d2> @ imm = #0xe
    499e: bf10         	yield
    49a0: 6829         	ldr	r1, [r5]
    49a2: 2900         	cmp	r1, #0x0
    49a4: bf02         	ittt	eq
    49a6: bf10         	yieldeq
    49a8: 6829         	ldreq	r1, [r5]
    49aa: 2900         	cmpeq	r1, #0x0
    49ac: d0ef         	beq	0x498e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x8b2> @ imm = #-0x22
    49ae: 6831         	ldr	r1, [r6]
    49b0: 6028         	str	r0, [r5]
    49b2: 6070         	str	r0, [r6, #0x4]
    49b4: e008         	b	0x49c8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x8ec> @ imm = #0x10
    49b6: bf10         	yield
    49b8: 6828         	ldr	r0, [r5]
    49ba: 2800         	cmp	r0, #0x0
    49bc: bf02         	ittt	eq
    49be: bf10         	yieldeq
    49c0: 6828         	ldreq	r0, [r5]
    49c2: 2800         	cmpeq	r0, #0x0
    49c4: d107         	bne	0x49d6 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x8fa> @ imm = #0xe
    49c6: bf10         	yield
    49c8: 6828         	ldr	r0, [r5]
    49ca: 2800         	cmp	r0, #0x0
    49cc: bf02         	ittt	eq
    49ce: bf10         	yieldeq
    49d0: 6828         	ldreq	r0, [r5]
    49d2: 2800         	cmpeq	r0, #0x0
    49d4: d0ef         	beq	0x49b6 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x8da> @ imm = #-0x22
    49d6: 2000         	movs	r0, #0x0
    49d8: f44f 7880    	mov.w	r8, #0x100
    49dc: f8d6 a000    	ldr.w	r10, [r6]
    49e0: f04f 0b00    	mov.w	r11, #0x0
    49e4: 6028         	str	r0, [r5]
    49e6: f8c9 8000    	str.w	r8, [r9]
    49ea: f002 fa4c    	bl	0x6e86 <_defmt_acquire> @ imm = #0x2498
    49ee: f240 0010    	movw	r0, #0x10
    49f2: f2c0 0000    	movt	r0, #0x0
    49f6: f002 f9cb    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2396
    49fa: f240 0901    	movw	r9, #0x1
    49fe: a806         	add	r0, sp, #0x18
    4a00: f2c0 0900    	movt	r9, #0x0
    4a04: 2102         	movs	r1, #0x2
    4a06: f8ad 9018    	strh.w	r9, [sp, #0x18]
    4a0a: f002 fb08    	bl	0x701e <_defmt_write>   @ imm = #0x2610
    4a0e: 9802         	ldr	r0, [sp, #0x8]
    4a10: 2101         	movs	r1, #0x1
    4a12: f88d 0018    	strb.w	r0, [sp, #0x18]
    4a16: a806         	add	r0, sp, #0x18
    4a18: f002 fb01    	bl	0x701e <_defmt_write>   @ imm = #0x2602
    4a1c: f002 fa88    	bl	0x6f30 <_defmt_release> @ imm = #0x2510
    4a20: f002 fa31    	bl	0x6e86 <_defmt_acquire> @ imm = #0x2462
    4a24: f240 0011    	movw	r0, #0x11
    4a28: f2c0 0000    	movt	r0, #0x0
    4a2c: f002 f9b0    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2360
    4a30: a806         	add	r0, sp, #0x18
    4a32: 2102         	movs	r1, #0x2
    4a34: f8ad 9018    	strh.w	r9, [sp, #0x18]
    4a38: f002 faf1    	bl	0x701e <_defmt_write>   @ imm = #0x25e2
    4a3c: 2084         	movs	r0, #0x84
    4a3e: 2101         	movs	r1, #0x1
    4a40: f88d 0018    	strb.w	r0, [sp, #0x18]
    4a44: a806         	add	r0, sp, #0x18
    4a46: f002 faea    	bl	0x701e <_defmt_write>   @ imm = #0x25d4
    4a4a: f002 fa71    	bl	0x6f30 <_defmt_release> @ imm = #0x24e2
    4a4e: f002 fa1a    	bl	0x6e86 <_defmt_acquire> @ imm = #0x2434
    4a52: f240 0012    	movw	r0, #0x12
    4a56: f2c0 0000    	movt	r0, #0x0
    4a5a: f002 f999    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2332
    4a5e: a806         	add	r0, sp, #0x18
    4a60: 2102         	movs	r1, #0x2
    4a62: f8ad 9018    	strh.w	r9, [sp, #0x18]
    4a66: f002 fada    	bl	0x701e <_defmt_write>   @ imm = #0x25b4
    4a6a: 9801         	ldr	r0, [sp, #0x4]
    4a6c: 2101         	movs	r1, #0x1
    4a6e: f88d 0018    	strb.w	r0, [sp, #0x18]
    4a72: a806         	add	r0, sp, #0x18
    4a74: f002 fad3    	bl	0x701e <_defmt_write>   @ imm = #0x25a6
    4a78: f002 fa5a    	bl	0x6f30 <_defmt_release> @ imm = #0x24b4
    4a7c: f002 fa03    	bl	0x6e86 <_defmt_acquire> @ imm = #0x2406
    4a80: f240 0013    	movw	r0, #0x13
    4a84: f2c0 0000    	movt	r0, #0x0
    4a88: f002 f982    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x2304
    4a8c: a806         	add	r0, sp, #0x18
    4a8e: 2102         	movs	r1, #0x2
    4a90: f8ad 9018    	strh.w	r9, [sp, #0x18]
    4a94: f002 fac3    	bl	0x701e <_defmt_write>   @ imm = #0x2586
    4a98: a806         	add	r0, sp, #0x18
    4a9a: 2101         	movs	r1, #0x1
    4a9c: f88d a018    	strb.w	r10, [sp, #0x18]
    4aa0: f002 fabd    	bl	0x701e <_defmt_write>   @ imm = #0x257a
    4aa4: f002 fa44    	bl	0x6f30 <_defmt_release> @ imm = #0x2488
    4aa8: f002 f9ed    	bl	0x6e86 <_defmt_acquire> @ imm = #0x23da
    4aac: f240 0014    	movw	r0, #0x14
    4ab0: f2c0 0000    	movt	r0, #0x0
    4ab4: f002 f96c    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x22d8
    4ab8: a806         	add	r0, sp, #0x18
    4aba: 2102         	movs	r1, #0x2
    4abc: f8ad 9018    	strh.w	r9, [sp, #0x18]
    4ac0: f002 faad    	bl	0x701e <_defmt_write>   @ imm = #0x255a
    4ac4: 9800         	ldr	r0, [sp]
    4ac6: 2101         	movs	r1, #0x1
    4ac8: f88d 0018    	strb.w	r0, [sp, #0x18]
    4acc: a806         	add	r0, sp, #0x18
    4ace: f002 faa6    	bl	0x701e <_defmt_write>   @ imm = #0x254c
    4ad2: f002 fa2d    	bl	0x6f30 <_defmt_release> @ imm = #0x245a
    4ad6: f002 f9d6    	bl	0x6e86 <_defmt_acquire> @ imm = #0x23ac
    4ada: f240 0015    	movw	r0, #0x15
    4ade: f2c0 0000    	movt	r0, #0x0
    4ae2: f002 f955    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #0x22aa
    4ae6: a806         	add	r0, sp, #0x18
    4ae8: 2102         	movs	r1, #0x2
    4aea: f8ad 9018    	strh.w	r9, [sp, #0x18]
    4aee: f002 fa96    	bl	0x701e <_defmt_write>   @ imm = #0x252c
    4af2: a806         	add	r0, sp, #0x18
    4af4: 2101         	movs	r1, #0x1
    4af6: f88d 4018    	strb.w	r4, [sp, #0x18]
    4afa: f002 fa90    	bl	0x701e <_defmt_write>   @ imm = #0x2520
    4afe: f002 fa17    	bl	0x6f30 <_defmt_release> @ imm = #0x242e
    4b02: f640 0208    	movw	r2, #0x808
    4b06: 2002         	movs	r0, #0x2
    4b08: f2c5 0200    	movt	r2, #0x5000
    4b0c: f8c2 8004    	str.w	r8, [r2, #0x4]
    4b10: 6070         	str	r0, [r6, #0x4]
    4b12: e008         	b	0x4b26 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xa4a> @ imm = #0x10
    4b14: bf10         	yield
    4b16: 6828         	ldr	r0, [r5]
    4b18: 2800         	cmp	r0, #0x0
    4b1a: bf02         	ittt	eq
    4b1c: bf10         	yieldeq
    4b1e: 6828         	ldreq	r0, [r5]
    4b20: 2800         	cmpeq	r0, #0x0
    4b22: d107         	bne	0x4b34 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xa58> @ imm = #0xe
    4b24: bf10         	yield
    4b26: 6828         	ldr	r0, [r5]
    4b28: 2800         	cmp	r0, #0x0
    4b2a: bf02         	ittt	eq
    4b2c: bf10         	yieldeq
    4b2e: 6828         	ldreq	r0, [r5]
    4b30: 2800         	cmpeq	r0, #0x0
    4b32: d0ef         	beq	0x4b14 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xa38> @ imm = #-0x22
    4b34: 6830         	ldr	r0, [r6]
    4b36: f240 2ac8    	movw	r10, #0x2c8
    4b3a: 202a         	movs	r0, #0x2a
    4b3c: f8c5 b000    	str.w	r11, [r5]
    4b40: 6070         	str	r0, [r6, #0x4]
    4b42: f2c2 0a00    	movt	r10, #0x2000
    4b46: 6828         	ldr	r0, [r5]
    4b48: b970         	cbnz	r0, 0x4b68 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xa8c> @ imm = #0x1c
    4b4a: e002         	b	0x4b52 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xa76> @ imm = #0x4
    4b4c: bf10         	yield
    4b4e: 6828         	ldr	r0, [r5]
    4b50: b950         	cbnz	r0, 0x4b68 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xa8c> @ imm = #0x14
    4b52: bf10         	yield
    4b54: 6828         	ldr	r0, [r5]
    4b56: b938         	cbnz	r0, 0x4b68 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xa8c> @ imm = #0xe
    4b58: bf10         	yield
    4b5a: 6828         	ldr	r0, [r5]
    4b5c: 2800         	cmp	r0, #0x0
    4b5e: bf02         	ittt	eq
    4b60: bf10         	yieldeq
    4b62: 6828         	ldreq	r0, [r5]
    4b64: 2800         	cmpeq	r0, #0x0
    4b66: d0f1         	beq	0x4b4c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xa70> @ imm = #-0x1e
    4b68: 6831         	ldr	r1, [r6]
    4b6a: 2000         	movs	r0, #0x0
    4b6c: 2101         	movs	r1, #0x1
    4b6e: 6028         	str	r0, [r5]
    4b70: 6071         	str	r1, [r6, #0x4]
    4b72: f240 2b50    	movw	r11, #0x250
    4b76: 6829         	ldr	r1, [r5]
    4b78: f2c2 0b00    	movt	r11, #0x2000
    4b7c: b971         	cbnz	r1, 0x4b9c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xac0> @ imm = #0x1c
    4b7e: e002         	b	0x4b86 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xaaa> @ imm = #0x4
    4b80: bf10         	yield
    4b82: 6829         	ldr	r1, [r5]
    4b84: b951         	cbnz	r1, 0x4b9c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xac0> @ imm = #0x14
    4b86: bf10         	yield
    4b88: 6829         	ldr	r1, [r5]
    4b8a: b939         	cbnz	r1, 0x4b9c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xac0> @ imm = #0xe
    4b8c: bf10         	yield
    4b8e: 6829         	ldr	r1, [r5]
    4b90: 2900         	cmp	r1, #0x0
    4b92: bf02         	ittt	eq
    4b94: bf10         	yieldeq
    4b96: 6829         	ldreq	r1, [r5]
    4b98: 2900         	cmpeq	r1, #0x0
    4b9a: d0f1         	beq	0x4b80 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xaa4> @ imm = #-0x1e
    4b9c: 6831         	ldr	r1, [r6]
    4b9e: 6028         	str	r0, [r5]
    4ba0: f44f 7080    	mov.w	r0, #0x100
    4ba4: 6010         	str	r0, [r2]
    4ba6: 6050         	str	r0, [r2, #0x4]
    4ba8: 2002         	movs	r0, #0x2
    4baa: 6070         	str	r0, [r6, #0x4]
    4bac: e008         	b	0x4bc0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xae4> @ imm = #0x10
    4bae: bf10         	yield
    4bb0: 6828         	ldr	r0, [r5]
    4bb2: 2800         	cmp	r0, #0x0
    4bb4: bf02         	ittt	eq
    4bb6: bf10         	yieldeq
    4bb8: 6828         	ldreq	r0, [r5]
    4bba: 2800         	cmpeq	r0, #0x0
    4bbc: d107         	bne	0x4bce <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xaf2> @ imm = #0xe
    4bbe: bf10         	yield
    4bc0: 6828         	ldr	r0, [r5]
    4bc2: 2800         	cmp	r0, #0x0
    4bc4: bf02         	ittt	eq
    4bc6: bf10         	yieldeq
    4bc8: 6828         	ldreq	r0, [r5]
    4bca: 2800         	cmpeq	r0, #0x0
    4bcc: d0ef         	beq	0x4bae <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xad2> @ imm = #-0x22
    4bce: 6831         	ldr	r1, [r6]
    4bd0: 2000         	movs	r0, #0x0
    4bd2: 2129         	movs	r1, #0x29
    4bd4: 6028         	str	r0, [r5]
    4bd6: 6071         	str	r1, [r6, #0x4]
    4bd8: e008         	b	0x4bec <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb10> @ imm = #0x10
    4bda: bf10         	yield
    4bdc: 6829         	ldr	r1, [r5]
    4bde: 2900         	cmp	r1, #0x0
    4be0: bf02         	ittt	eq
    4be2: bf10         	yieldeq
    4be4: 6829         	ldreq	r1, [r5]
    4be6: 2900         	cmpeq	r1, #0x0
    4be8: d107         	bne	0x4bfa <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb1e> @ imm = #0xe
    4bea: bf10         	yield
    4bec: 6829         	ldr	r1, [r5]
    4bee: 2900         	cmp	r1, #0x0
    4bf0: bf02         	ittt	eq
    4bf2: bf10         	yieldeq
    4bf4: 6829         	ldreq	r1, [r5]
    4bf6: 2900         	cmpeq	r1, #0x0
    4bf8: d0ef         	beq	0x4bda <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xafe> @ imm = #-0x22
    4bfa: 6831         	ldr	r1, [r6]
    4bfc: 6028         	str	r0, [r5]
    4bfe: 20b1         	movs	r0, #0xb1
    4c00: 6070         	str	r0, [r6, #0x4]
    4c02: e008         	b	0x4c16 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb3a> @ imm = #0x10
    4c04: bf10         	yield
    4c06: 6828         	ldr	r0, [r5]
    4c08: 2800         	cmp	r0, #0x0
    4c0a: bf02         	ittt	eq
    4c0c: bf10         	yieldeq
    4c0e: 6828         	ldreq	r0, [r5]
    4c10: 2800         	cmpeq	r0, #0x0
    4c12: d107         	bne	0x4c24 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb48> @ imm = #0xe
    4c14: bf10         	yield
    4c16: 6828         	ldr	r0, [r5]
    4c18: 2800         	cmp	r0, #0x0
    4c1a: bf02         	ittt	eq
    4c1c: bf10         	yieldeq
    4c1e: 6828         	ldreq	r0, [r5]
    4c20: 2800         	cmpeq	r0, #0x0
    4c22: d0ef         	beq	0x4c04 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb28> @ imm = #-0x22
    4c24: 6831         	ldr	r1, [r6]
    4c26: 2000         	movs	r0, #0x0
    4c28: f44f 7180    	mov.w	r1, #0x100
    4c2c: 6028         	str	r0, [r5]
    4c2e: 6011         	str	r1, [r2]
    4c30: 6051         	str	r1, [r2, #0x4]
    4c32: 2102         	movs	r1, #0x2
    4c34: 6071         	str	r1, [r6, #0x4]
    4c36: e008         	b	0x4c4a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb6e> @ imm = #0x10
    4c38: bf10         	yield
    4c3a: 6829         	ldr	r1, [r5]
    4c3c: 2900         	cmp	r1, #0x0
    4c3e: bf02         	ittt	eq
    4c40: bf10         	yieldeq
    4c42: 6829         	ldreq	r1, [r5]
    4c44: 2900         	cmpeq	r1, #0x0
    4c46: d107         	bne	0x4c58 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb7c> @ imm = #0xe
    4c48: bf10         	yield
    4c4a: 6829         	ldr	r1, [r5]
    4c4c: 2900         	cmp	r1, #0x0
    4c4e: bf02         	ittt	eq
    4c50: bf10         	yieldeq
    4c52: 6829         	ldreq	r1, [r5]
    4c54: 2900         	cmpeq	r1, #0x0
    4c56: d0ef         	beq	0x4c38 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb5c> @ imm = #-0x22
    4c58: 6831         	ldr	r1, [r6]
    4c5a: 6028         	str	r0, [r5]
    4c5c: 2028         	movs	r0, #0x28
    4c5e: 6070         	str	r0, [r6, #0x4]
    4c60: e008         	b	0x4c74 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb98> @ imm = #0x10
    4c62: bf10         	yield
    4c64: 6828         	ldr	r0, [r5]
    4c66: 2800         	cmp	r0, #0x0
    4c68: bf02         	ittt	eq
    4c6a: bf10         	yieldeq
    4c6c: 6828         	ldreq	r0, [r5]
    4c6e: 2800         	cmpeq	r0, #0x0
    4c70: d107         	bne	0x4c82 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xba6> @ imm = #0xe
    4c72: bf10         	yield
    4c74: 6828         	ldr	r0, [r5]
    4c76: 2800         	cmp	r0, #0x0
    4c78: bf02         	ittt	eq
    4c7a: bf10         	yieldeq
    4c7c: 6828         	ldreq	r0, [r5]
    4c7e: 2800         	cmpeq	r0, #0x0
    4c80: d0ef         	beq	0x4c62 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xb86> @ imm = #-0x22
    4c82: 6831         	ldr	r1, [r6]
    4c84: 2000         	movs	r0, #0x0
    4c86: 2185         	movs	r1, #0x85
    4c88: 6028         	str	r0, [r5]
    4c8a: 6071         	str	r1, [r6, #0x4]
    4c8c: e008         	b	0x4ca0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xbc4> @ imm = #0x10
    4c8e: bf10         	yield
    4c90: 6829         	ldr	r1, [r5]
    4c92: 2900         	cmp	r1, #0x0
    4c94: bf02         	ittt	eq
    4c96: bf10         	yieldeq
    4c98: 6829         	ldreq	r1, [r5]
    4c9a: 2900         	cmpeq	r1, #0x0
    4c9c: d107         	bne	0x4cae <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xbd2> @ imm = #0xe
    4c9e: bf10         	yield
    4ca0: 6829         	ldr	r1, [r5]
    4ca2: 2900         	cmp	r1, #0x0
    4ca4: bf02         	ittt	eq
    4ca6: bf10         	yieldeq
    4ca8: 6829         	ldreq	r1, [r5]
    4caa: 2900         	cmpeq	r1, #0x0
    4cac: d0ef         	beq	0x4c8e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xbb2> @ imm = #-0x22
    4cae: 6831         	ldr	r1, [r6]
    4cb0: 6028         	str	r0, [r5]
    4cb2: f44f 7080    	mov.w	r0, #0x100
    4cb6: 6010         	str	r0, [r2]
    4cb8: 6050         	str	r0, [r2, #0x4]
    4cba: 2002         	movs	r0, #0x2
    4cbc: 6070         	str	r0, [r6, #0x4]
    4cbe: e008         	b	0x4cd2 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xbf6> @ imm = #0x10
    4cc0: bf10         	yield
    4cc2: 6828         	ldr	r0, [r5]
    4cc4: 2800         	cmp	r0, #0x0
    4cc6: bf02         	ittt	eq
    4cc8: bf10         	yieldeq
    4cca: 6828         	ldreq	r0, [r5]
    4ccc: 2800         	cmpeq	r0, #0x0
    4cce: d107         	bne	0x4ce0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc04> @ imm = #0xe
    4cd0: bf10         	yield
    4cd2: 6828         	ldr	r0, [r5]
    4cd4: 2800         	cmp	r0, #0x0
    4cd6: bf02         	ittt	eq
    4cd8: bf10         	yieldeq
    4cda: 6828         	ldreq	r0, [r5]
    4cdc: 2800         	cmpeq	r0, #0x0
    4cde: d0ef         	beq	0x4cc0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xbe4> @ imm = #-0x22
    4ce0: 6831         	ldr	r1, [r6]
    4ce2: 2000         	movs	r0, #0x0
    4ce4: 212b         	movs	r1, #0x2b
    4ce6: 6028         	str	r0, [r5]
    4ce8: 6071         	str	r1, [r6, #0x4]
    4cea: e008         	b	0x4cfe <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc22> @ imm = #0x10
    4cec: bf10         	yield
    4cee: 6829         	ldr	r1, [r5]
    4cf0: 2900         	cmp	r1, #0x0
    4cf2: bf02         	ittt	eq
    4cf4: bf10         	yieldeq
    4cf6: 6829         	ldreq	r1, [r5]
    4cf8: 2900         	cmpeq	r1, #0x0
    4cfa: d107         	bne	0x4d0c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc30> @ imm = #0xe
    4cfc: bf10         	yield
    4cfe: 6829         	ldr	r1, [r5]
    4d00: 2900         	cmp	r1, #0x0
    4d02: bf02         	ittt	eq
    4d04: bf10         	yieldeq
    4d06: 6829         	ldreq	r1, [r5]
    4d08: 2900         	cmpeq	r1, #0x0
    4d0a: d0ef         	beq	0x4cec <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc10> @ imm = #-0x22
    4d0c: 6831         	ldr	r1, [r6]
    4d0e: 6028         	str	r0, [r5]
    4d10: 20bf         	movs	r0, #0xbf
    4d12: 6070         	str	r0, [r6, #0x4]
    4d14: e008         	b	0x4d28 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc4c> @ imm = #0x10
    4d16: bf10         	yield
    4d18: 6828         	ldr	r0, [r5]
    4d1a: 2800         	cmp	r0, #0x0
    4d1c: bf02         	ittt	eq
    4d1e: bf10         	yieldeq
    4d20: 6828         	ldreq	r0, [r5]
    4d22: 2800         	cmpeq	r0, #0x0
    4d24: d107         	bne	0x4d36 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc5a> @ imm = #0xe
    4d26: bf10         	yield
    4d28: 6828         	ldr	r0, [r5]
    4d2a: 2800         	cmp	r0, #0x0
    4d2c: bf02         	ittt	eq
    4d2e: bf10         	yieldeq
    4d30: 6828         	ldreq	r0, [r5]
    4d32: 2800         	cmpeq	r0, #0x0
    4d34: d0ef         	beq	0x4d16 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc3a> @ imm = #-0x22
    4d36: 6831         	ldr	r1, [r6]
    4d38: 2000         	movs	r0, #0x0
    4d3a: f44f 7180    	mov.w	r1, #0x100
    4d3e: 6028         	str	r0, [r5]
    4d40: 6011         	str	r1, [r2]
    4d42: 6051         	str	r1, [r2, #0x4]
    4d44: 2103         	movs	r1, #0x3
    4d46: 6071         	str	r1, [r6, #0x4]
    4d48: e008         	b	0x4d5c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc80> @ imm = #0x10
    4d4a: bf10         	yield
    4d4c: 6829         	ldr	r1, [r5]
    4d4e: 2900         	cmp	r1, #0x0
    4d50: bf02         	ittt	eq
    4d52: bf10         	yieldeq
    4d54: 6829         	ldreq	r1, [r5]
    4d56: 2900         	cmpeq	r1, #0x0
    4d58: d107         	bne	0x4d6a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc8e> @ imm = #0xe
    4d5a: bf10         	yield
    4d5c: 6829         	ldr	r1, [r5]
    4d5e: 2900         	cmp	r1, #0x0
    4d60: bf02         	ittt	eq
    4d62: bf10         	yieldeq
    4d64: 6829         	ldreq	r1, [r5]
    4d66: 2900         	cmpeq	r1, #0x0
    4d68: d0ef         	beq	0x4d4a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc6e> @ imm = #-0x22
    4d6a: 6831         	ldr	r1, [r6]
    4d6c: 6028         	str	r0, [r5]
    4d6e: 202b         	movs	r0, #0x2b
    4d70: 6070         	str	r0, [r6, #0x4]
    4d72: e008         	b	0x4d86 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xcaa> @ imm = #0x10
    4d74: bf10         	yield
    4d76: 6828         	ldr	r0, [r5]
    4d78: 2800         	cmp	r0, #0x0
    4d7a: bf02         	ittt	eq
    4d7c: bf10         	yieldeq
    4d7e: 6828         	ldreq	r0, [r5]
    4d80: 2800         	cmpeq	r0, #0x0
    4d82: d107         	bne	0x4d94 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xcb8> @ imm = #0xe
    4d84: bf10         	yield
    4d86: 6828         	ldr	r0, [r5]
    4d88: 2800         	cmp	r0, #0x0
    4d8a: bf02         	ittt	eq
    4d8c: bf10         	yieldeq
    4d8e: 6828         	ldreq	r0, [r5]
    4d90: 2800         	cmpeq	r0, #0x0
    4d92: d0ef         	beq	0x4d74 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xc98> @ imm = #-0x22
    4d94: 2000         	movs	r0, #0x0
    4d96: 6831         	ldr	r1, [r6]
    4d98: 6028         	str	r0, [r5]
    4d9a: 6070         	str	r0, [r6, #0x4]
    4d9c: e008         	b	0x4db0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xcd4> @ imm = #0x10
    4d9e: bf10         	yield
    4da0: 6829         	ldr	r1, [r5]
    4da2: 2900         	cmp	r1, #0x0
    4da4: bf02         	ittt	eq
    4da6: bf10         	yieldeq
    4da8: 6829         	ldreq	r1, [r5]
    4daa: 2900         	cmpeq	r1, #0x0
    4dac: d107         	bne	0x4dbe <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xce2> @ imm = #0xe
    4dae: bf10         	yield
    4db0: 6829         	ldr	r1, [r5]
    4db2: 2900         	cmp	r1, #0x0
    4db4: bf02         	ittt	eq
    4db6: bf10         	yieldeq
    4db8: 6829         	ldreq	r1, [r5]
    4dba: 2900         	cmpeq	r1, #0x0
    4dbc: d0ef         	beq	0x4d9e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xcc2> @ imm = #-0x22
    4dbe: 6831         	ldr	r1, [r6]
    4dc0: 6028         	str	r0, [r5]
    4dc2: f44f 7080    	mov.w	r0, #0x100
    4dc6: 6010         	str	r0, [r2]
    4dc8: 6050         	str	r0, [r2, #0x4]
    4dca: 2002         	movs	r0, #0x2
    4dcc: 6070         	str	r0, [r6, #0x4]
    4dce: e008         	b	0x4de2 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd06> @ imm = #0x10
    4dd0: bf10         	yield
    4dd2: 6828         	ldr	r0, [r5]
    4dd4: 2800         	cmp	r0, #0x0
    4dd6: bf02         	ittt	eq
    4dd8: bf10         	yieldeq
    4dda: 6828         	ldreq	r0, [r5]
    4ddc: 2800         	cmpeq	r0, #0x0
    4dde: d107         	bne	0x4df0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd14> @ imm = #0xe
    4de0: bf10         	yield
    4de2: 6828         	ldr	r0, [r5]
    4de4: 2800         	cmp	r0, #0x0
    4de6: bf02         	ittt	eq
    4de8: bf10         	yieldeq
    4dea: 6828         	ldreq	r0, [r5]
    4dec: 2800         	cmpeq	r0, #0x0
    4dee: d0ef         	beq	0x4dd0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xcf4> @ imm = #-0x22
    4df0: 6831         	ldr	r1, [r6]
    4df2: 2000         	movs	r0, #0x0
    4df4: 2130         	movs	r1, #0x30
    4df6: 6028         	str	r0, [r5]
    4df8: 6071         	str	r1, [r6, #0x4]
    4dfa: e008         	b	0x4e0e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd32> @ imm = #0x10
    4dfc: bf10         	yield
    4dfe: 6829         	ldr	r1, [r5]
    4e00: 2900         	cmp	r1, #0x0
    4e02: bf02         	ittt	eq
    4e04: bf10         	yieldeq
    4e06: 6829         	ldreq	r1, [r5]
    4e08: 2900         	cmpeq	r1, #0x0
    4e0a: d107         	bne	0x4e1c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd40> @ imm = #0xe
    4e0c: bf10         	yield
    4e0e: 6829         	ldr	r1, [r5]
    4e10: 2900         	cmp	r1, #0x0
    4e12: bf02         	ittt	eq
    4e14: bf10         	yieldeq
    4e16: 6829         	ldreq	r1, [r5]
    4e18: 2900         	cmpeq	r1, #0x0
    4e1a: d0ef         	beq	0x4dfc <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd20> @ imm = #-0x22
    4e1c: 6831         	ldr	r1, [r6]
    4e1e: 6028         	str	r0, [r5]
    4e20: 6070         	str	r0, [r6, #0x4]
    4e22: e008         	b	0x4e36 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd5a> @ imm = #0x10
    4e24: bf10         	yield
    4e26: 6828         	ldr	r0, [r5]
    4e28: 2800         	cmp	r0, #0x0
    4e2a: bf02         	ittt	eq
    4e2c: bf10         	yieldeq
    4e2e: 6828         	ldreq	r0, [r5]
    4e30: 2800         	cmpeq	r0, #0x0
    4e32: d107         	bne	0x4e44 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd68> @ imm = #0xe
    4e34: bf10         	yield
    4e36: 6828         	ldr	r0, [r5]
    4e38: 2800         	cmp	r0, #0x0
    4e3a: bf02         	ittt	eq
    4e3c: bf10         	yieldeq
    4e3e: 6828         	ldreq	r0, [r5]
    4e40: 2800         	cmpeq	r0, #0x0
    4e42: d0ef         	beq	0x4e24 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd48> @ imm = #-0x22
    4e44: 6831         	ldr	r1, [r6]
    4e46: 2000         	movs	r0, #0x0
    4e48: f44f 7180    	mov.w	r1, #0x100
    4e4c: 6028         	str	r0, [r5]
    4e4e: 6011         	str	r1, [r2]
    4e50: 6051         	str	r1, [r2, #0x4]
    4e52: 2102         	movs	r1, #0x2
    4e54: 6071         	str	r1, [r6, #0x4]
    4e56: e008         	b	0x4e6a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd8e> @ imm = #0x10
    4e58: bf10         	yield
    4e5a: 6829         	ldr	r1, [r5]
    4e5c: 2900         	cmp	r1, #0x0
    4e5e: bf02         	ittt	eq
    4e60: bf10         	yieldeq
    4e62: 6829         	ldreq	r1, [r5]
    4e64: 2900         	cmpeq	r1, #0x0
    4e66: d107         	bne	0x4e78 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd9c> @ imm = #0xe
    4e68: bf10         	yield
    4e6a: 6829         	ldr	r1, [r5]
    4e6c: 2900         	cmp	r1, #0x0
    4e6e: bf02         	ittt	eq
    4e70: bf10         	yieldeq
    4e72: 6829         	ldreq	r1, [r5]
    4e74: 2900         	cmpeq	r1, #0x0
    4e76: d0ef         	beq	0x4e58 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xd7c> @ imm = #-0x22
    4e78: 6831         	ldr	r1, [r6]
    4e7a: 6028         	str	r0, [r5]
    4e7c: 2040         	movs	r0, #0x40
    4e7e: 6070         	str	r0, [r6, #0x4]
    4e80: e008         	b	0x4e94 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xdb8> @ imm = #0x10
    4e82: bf10         	yield
    4e84: 6828         	ldr	r0, [r5]
    4e86: 2800         	cmp	r0, #0x0
    4e88: bf02         	ittt	eq
    4e8a: bf10         	yieldeq
    4e8c: 6828         	ldreq	r0, [r5]
    4e8e: 2800         	cmpeq	r0, #0x0
    4e90: d107         	bne	0x4ea2 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xdc6> @ imm = #0xe
    4e92: bf10         	yield
    4e94: 6828         	ldr	r0, [r5]
    4e96: 2800         	cmp	r0, #0x0
    4e98: bf02         	ittt	eq
    4e9a: bf10         	yieldeq
    4e9c: 6828         	ldreq	r0, [r5]
    4e9e: 2800         	cmpeq	r0, #0x0
    4ea0: d0ef         	beq	0x4e82 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xda6> @ imm = #-0x22
    4ea2: 2000         	movs	r0, #0x0
    4ea4: 6831         	ldr	r1, [r6]
    4ea6: 6028         	str	r0, [r5]
    4ea8: 6070         	str	r0, [r6, #0x4]
    4eaa: e008         	b	0x4ebe <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xde2> @ imm = #0x10
    4eac: bf10         	yield
    4eae: 6829         	ldr	r1, [r5]
    4eb0: 2900         	cmp	r1, #0x0
    4eb2: bf02         	ittt	eq
    4eb4: bf10         	yieldeq
    4eb6: 6829         	ldreq	r1, [r5]
    4eb8: 2900         	cmpeq	r1, #0x0
    4eba: d107         	bne	0x4ecc <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xdf0> @ imm = #0xe
    4ebc: bf10         	yield
    4ebe: 6829         	ldr	r1, [r5]
    4ec0: 2900         	cmp	r1, #0x0
    4ec2: bf02         	ittt	eq
    4ec4: bf10         	yieldeq
    4ec6: 6829         	ldreq	r1, [r5]
    4ec8: 2900         	cmpeq	r1, #0x0
    4eca: d0ef         	beq	0x4eac <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xdd0> @ imm = #-0x22
    4ecc: 6831         	ldr	r1, [r6]
    4ece: 6028         	str	r0, [r5]
    4ed0: f44f 7080    	mov.w	r0, #0x100
    4ed4: 6010         	str	r0, [r2]
    4ed6: 6050         	str	r0, [r2, #0x4]
    4ed8: 2002         	movs	r0, #0x2
    4eda: 6070         	str	r0, [r6, #0x4]
    4edc: e008         	b	0x4ef0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe14> @ imm = #0x10
    4ede: bf10         	yield
    4ee0: 6828         	ldr	r0, [r5]
    4ee2: 2800         	cmp	r0, #0x0
    4ee4: bf02         	ittt	eq
    4ee6: bf10         	yieldeq
    4ee8: 6828         	ldreq	r0, [r5]
    4eea: 2800         	cmpeq	r0, #0x0
    4eec: d107         	bne	0x4efe <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe22> @ imm = #0xe
    4eee: bf10         	yield
    4ef0: 6828         	ldr	r0, [r5]
    4ef2: 2800         	cmp	r0, #0x0
    4ef4: bf02         	ittt	eq
    4ef6: bf10         	yieldeq
    4ef8: 6828         	ldreq	r0, [r5]
    4efa: 2800         	cmpeq	r0, #0x0
    4efc: d0ef         	beq	0x4ede <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe02> @ imm = #-0x22
    4efe: 6831         	ldr	r1, [r6]
    4f00: 2000         	movs	r0, #0x0
    4f02: 2150         	movs	r1, #0x50
    4f04: 6028         	str	r0, [r5]
    4f06: 6071         	str	r1, [r6, #0x4]
    4f08: e008         	b	0x4f1c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe40> @ imm = #0x10
    4f0a: bf10         	yield
    4f0c: 6829         	ldr	r1, [r5]
    4f0e: 2900         	cmp	r1, #0x0
    4f10: bf02         	ittt	eq
    4f12: bf10         	yieldeq
    4f14: 6829         	ldreq	r1, [r5]
    4f16: 2900         	cmpeq	r1, #0x0
    4f18: d107         	bne	0x4f2a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe4e> @ imm = #0xe
    4f1a: bf10         	yield
    4f1c: 6829         	ldr	r1, [r5]
    4f1e: 2900         	cmp	r1, #0x0
    4f20: bf02         	ittt	eq
    4f22: bf10         	yieldeq
    4f24: 6829         	ldreq	r1, [r5]
    4f26: 2900         	cmpeq	r1, #0x0
    4f28: d0ef         	beq	0x4f0a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe2e> @ imm = #-0x22
    4f2a: 6831         	ldr	r1, [r6]
    4f2c: 6028         	str	r0, [r5]
    4f2e: 6070         	str	r0, [r6, #0x4]
    4f30: e008         	b	0x4f44 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe68> @ imm = #0x10
    4f32: bf10         	yield
    4f34: 6828         	ldr	r0, [r5]
    4f36: 2800         	cmp	r0, #0x0
    4f38: bf02         	ittt	eq
    4f3a: bf10         	yieldeq
    4f3c: 6828         	ldreq	r0, [r5]
    4f3e: 2800         	cmpeq	r0, #0x0
    4f40: d107         	bne	0x4f52 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe76> @ imm = #0xe
    4f42: bf10         	yield
    4f44: 6828         	ldr	r0, [r5]
    4f46: 2800         	cmp	r0, #0x0
    4f48: bf02         	ittt	eq
    4f4a: bf10         	yieldeq
    4f4c: 6828         	ldreq	r0, [r5]
    4f4e: 2800         	cmpeq	r0, #0x0
    4f50: d0ef         	beq	0x4f32 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe56> @ imm = #-0x22
    4f52: 6831         	ldr	r1, [r6]
    4f54: 2000         	movs	r0, #0x0
    4f56: f44f 7180    	mov.w	r1, #0x100
    4f5a: 6028         	str	r0, [r5]
    4f5c: 6011         	str	r1, [r2]
    4f5e: 6051         	str	r1, [r2, #0x4]
    4f60: 2102         	movs	r1, #0x2
    4f62: 6071         	str	r1, [r6, #0x4]
    4f64: e008         	b	0x4f78 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe9c> @ imm = #0x10
    4f66: bf10         	yield
    4f68: 6829         	ldr	r1, [r5]
    4f6a: 2900         	cmp	r1, #0x0
    4f6c: bf02         	ittt	eq
    4f6e: bf10         	yieldeq
    4f70: 6829         	ldreq	r1, [r5]
    4f72: 2900         	cmpeq	r1, #0x0
    4f74: d107         	bne	0x4f86 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xeaa> @ imm = #0xe
    4f76: bf10         	yield
    4f78: 6829         	ldr	r1, [r5]
    4f7a: 2900         	cmp	r1, #0x0
    4f7c: bf02         	ittt	eq
    4f7e: bf10         	yieldeq
    4f80: 6829         	ldreq	r1, [r5]
    4f82: 2900         	cmpeq	r1, #0x0
    4f84: d0ef         	beq	0x4f66 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xe8a> @ imm = #-0x22
    4f86: 6831         	ldr	r1, [r6]
    4f88: 6028         	str	r0, [r5]
    4f8a: 2006         	movs	r0, #0x6
    4f8c: 6070         	str	r0, [r6, #0x4]
    4f8e: e008         	b	0x4fa2 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xec6> @ imm = #0x10
    4f90: bf10         	yield
    4f92: 6828         	ldr	r0, [r5]
    4f94: 2800         	cmp	r0, #0x0
    4f96: bf02         	ittt	eq
    4f98: bf10         	yieldeq
    4f9a: 6828         	ldreq	r0, [r5]
    4f9c: 2800         	cmpeq	r0, #0x0
    4f9e: d107         	bne	0x4fb0 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xed4> @ imm = #0xe
    4fa0: bf10         	yield
    4fa2: 6828         	ldr	r0, [r5]
    4fa4: 2800         	cmp	r0, #0x0
    4fa6: bf02         	ittt	eq
    4fa8: bf10         	yieldeq
    4faa: 6828         	ldreq	r0, [r5]
    4fac: 2800         	cmpeq	r0, #0x0
    4fae: d0ef         	beq	0x4f90 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xeb4> @ imm = #-0x22
    4fb0: 6831         	ldr	r1, [r6]
    4fb2: 2000         	movs	r0, #0x0
    4fb4: 2124         	movs	r1, #0x24
    4fb6: 6028         	str	r0, [r5]
    4fb8: 6071         	str	r1, [r6, #0x4]
    4fba: e008         	b	0x4fce <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xef2> @ imm = #0x10
    4fbc: bf10         	yield
    4fbe: 6829         	ldr	r1, [r5]
    4fc0: 2900         	cmp	r1, #0x0
    4fc2: bf02         	ittt	eq
    4fc4: bf10         	yieldeq
    4fc6: 6829         	ldreq	r1, [r5]
    4fc8: 2900         	cmpeq	r1, #0x0
    4fca: d107         	bne	0x4fdc <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf00> @ imm = #0xe
    4fcc: bf10         	yield
    4fce: 6829         	ldr	r1, [r5]
    4fd0: 2900         	cmp	r1, #0x0
    4fd2: bf02         	ittt	eq
    4fd4: bf10         	yieldeq
    4fd6: 6829         	ldreq	r1, [r5]
    4fd8: 2900         	cmpeq	r1, #0x0
    4fda: d0ef         	beq	0x4fbc <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xee0> @ imm = #-0x22
    4fdc: 6831         	ldr	r1, [r6]
    4fde: 6028         	str	r0, [r5]
    4fe0: f44f 7080    	mov.w	r0, #0x100
    4fe4: 6010         	str	r0, [r2]
    4fe6: 6050         	str	r0, [r2, #0x4]
    4fe8: 2002         	movs	r0, #0x2
    4fea: 6070         	str	r0, [r6, #0x4]
    4fec: e008         	b	0x5000 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf24> @ imm = #0x10
    4fee: bf10         	yield
    4ff0: 6828         	ldr	r0, [r5]
    4ff2: 2800         	cmp	r0, #0x0
    4ff4: bf02         	ittt	eq
    4ff6: bf10         	yieldeq
    4ff8: 6828         	ldreq	r0, [r5]
    4ffa: 2800         	cmpeq	r0, #0x0
    4ffc: d107         	bne	0x500e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf32> @ imm = #0xe
    4ffe: bf10         	yield
    5000: 6828         	ldr	r0, [r5]
    5002: 2800         	cmp	r0, #0x0
    5004: bf02         	ittt	eq
    5006: bf10         	yieldeq
    5008: 6828         	ldreq	r0, [r5]
    500a: 2800         	cmpeq	r0, #0x0
    500c: d0ef         	beq	0x4fee <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf12> @ imm = #-0x22
    500e: 6831         	ldr	r1, [r6]
    5010: 2000         	movs	r0, #0x0
    5012: 2107         	movs	r1, #0x7
    5014: 6028         	str	r0, [r5]
    5016: 6071         	str	r1, [r6, #0x4]
    5018: e008         	b	0x502c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf50> @ imm = #0x10
    501a: bf10         	yield
    501c: 6829         	ldr	r1, [r5]
    501e: 2900         	cmp	r1, #0x0
    5020: bf02         	ittt	eq
    5022: bf10         	yieldeq
    5024: 6829         	ldreq	r1, [r5]
    5026: 2900         	cmpeq	r1, #0x0
    5028: d107         	bne	0x503a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf5e> @ imm = #0xe
    502a: bf10         	yield
    502c: 6829         	ldr	r1, [r5]
    502e: 2900         	cmp	r1, #0x0
    5030: bf02         	ittt	eq
    5032: bf10         	yieldeq
    5034: 6829         	ldreq	r1, [r5]
    5036: 2900         	cmpeq	r1, #0x0
    5038: d0ef         	beq	0x501a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf3e> @ imm = #-0x22
    503a: 6831         	ldr	r1, [r6]
    503c: 6028         	str	r0, [r5]
    503e: 2020         	movs	r0, #0x20
    5040: 6070         	str	r0, [r6, #0x4]
    5042: e008         	b	0x5056 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf7a> @ imm = #0x10
    5044: bf10         	yield
    5046: 6828         	ldr	r0, [r5]
    5048: 2800         	cmp	r0, #0x0
    504a: bf02         	ittt	eq
    504c: bf10         	yieldeq
    504e: 6828         	ldreq	r0, [r5]
    5050: 2800         	cmpeq	r0, #0x0
    5052: d107         	bne	0x5064 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf88> @ imm = #0xe
    5054: bf10         	yield
    5056: 6828         	ldr	r0, [r5]
    5058: 2800         	cmp	r0, #0x0
    505a: bf02         	ittt	eq
    505c: bf10         	yieldeq
    505e: 6828         	ldreq	r0, [r5]
    5060: 2800         	cmpeq	r0, #0x0
    5062: d0ef         	beq	0x5044 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xf68> @ imm = #-0x22
    5064: 6830         	ldr	r0, [r6]
    5066: f04f 0800    	mov.w	r8, #0x0
    506a: f44f 7480    	mov.w	r4, #0x100
    506e: f8c5 8000    	str.w	r8, [r5]
    5072: 6014         	str	r4, [r2]
    5074: 2060         	movs	r0, #0x60
    5076: f240 71ff    	movw	r1, #0x7ff
    507a: 2200         	movs	r2, #0x0
    507c: f7fb f9f6    	bl	0x46c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3> @ imm = #-0x4c14
    5080: 2070         	movs	r0, #0x70
    5082: f240 71ff    	movw	r1, #0x7ff
    5086: 2200         	movs	r2, #0x0
    5088: f7fb f9f0    	bl	0x46c <controller::drivers::can::Mcp2515Driver<SPI,PIN>::filter_message_id::hf52e48189e0f9aa3> @ imm = #-0x4c20
    508c: f640 0108    	movw	r1, #0x808
    5090: 2002         	movs	r0, #0x2
    5092: f2c5 0100    	movt	r1, #0x5000
    5096: 604c         	str	r4, [r1, #0x4]
    5098: 6070         	str	r0, [r6, #0x4]
    509a: e008         	b	0x50ae <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xfd2> @ imm = #0x10
    509c: bf10         	yield
    509e: 6828         	ldr	r0, [r5]
    50a0: 2800         	cmp	r0, #0x0
    50a2: bf02         	ittt	eq
    50a4: bf10         	yieldeq
    50a6: 6828         	ldreq	r0, [r5]
    50a8: 2800         	cmpeq	r0, #0x0
    50aa: d107         	bne	0x50bc <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xfe0> @ imm = #0xe
    50ac: bf10         	yield
    50ae: 6828         	ldr	r0, [r5]
    50b0: 2800         	cmp	r0, #0x0
    50b2: bf02         	ittt	eq
    50b4: bf10         	yieldeq
    50b6: 6828         	ldreq	r0, [r5]
    50b8: 2800         	cmpeq	r0, #0x0
    50ba: d0ef         	beq	0x509c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xfc0> @ imm = #-0x22
    50bc: 6830         	ldr	r0, [r6]
    50be: 200f         	movs	r0, #0xf
    50c0: f8c5 8000    	str.w	r8, [r5]
    50c4: 6070         	str	r0, [r6, #0x4]
    50c6: e008         	b	0x50da <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xffe> @ imm = #0x10
    50c8: bf10         	yield
    50ca: 6828         	ldr	r0, [r5]
    50cc: 2800         	cmp	r0, #0x0
    50ce: bf02         	ittt	eq
    50d0: bf10         	yieldeq
    50d2: 6828         	ldreq	r0, [r5]
    50d4: 2800         	cmpeq	r0, #0x0
    50d6: d107         	bne	0x50e8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x100c> @ imm = #0xe
    50d8: bf10         	yield
    50da: 6828         	ldr	r0, [r5]
    50dc: 2800         	cmp	r0, #0x0
    50de: bf02         	ittt	eq
    50e0: bf10         	yieldeq
    50e2: 6828         	ldreq	r0, [r5]
    50e4: 2800         	cmpeq	r0, #0x0
    50e6: d0ef         	beq	0x50c8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0xfec> @ imm = #-0x22
    50e8: 6830         	ldr	r0, [r6]
    50ea: 2200         	movs	r2, #0x0
    50ec: 2004         	movs	r0, #0x4
    50ee: 602a         	str	r2, [r5]
    50f0: 6070         	str	r0, [r6, #0x4]
    50f2: e008         	b	0x5106 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x102a> @ imm = #0x10
    50f4: bf10         	yield
    50f6: 6828         	ldr	r0, [r5]
    50f8: 2800         	cmp	r0, #0x0
    50fa: bf02         	ittt	eq
    50fc: bf10         	yieldeq
    50fe: 6828         	ldreq	r0, [r5]
    5100: 2800         	cmpeq	r0, #0x0
    5102: d107         	bne	0x5114 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1038> @ imm = #0xe
    5104: bf10         	yield
    5106: 6828         	ldr	r0, [r5]
    5108: 2800         	cmp	r0, #0x0
    510a: bf02         	ittt	eq
    510c: bf10         	yieldeq
    510e: 6828         	ldreq	r0, [r5]
    5110: 2800         	cmpeq	r0, #0x0
    5112: d0ef         	beq	0x50f4 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1018> @ imm = #-0x22
    5114: 6830         	ldr	r0, [r6]
    5116: f24c 5804    	movw	r8, #0xc504
    511a: f44f 7080    	mov.w	r0, #0x100
    511e: 602a         	str	r2, [r5]
    5120: 6008         	str	r0, [r1]
    5122: f24c 1108    	movw	r1, #0xc108
    5126: f2c4 0101    	movt	r1, #0x4001
    512a: 2601         	movs	r6, #0x1
    512c: f2c4 0801    	movt	r8, #0x4001
    5130: f647 73ff    	movw	r3, #0x7fff
    5134: f8c1 63f8    	str.w	r6, [r1, #0x3f8]
    5138: f8c8 2000    	str.w	r2, [r8]
    513c: f247 7ce4    	movw	r12, #0x77e4
    5140: f8c8 2008    	str.w	r2, [r8, #0x8]
    5144: f2c0 0c00    	movt	r12, #0x0
    5148: f8c8 3004    	str.w	r3, [r8, #0x4]
    514c: 2302         	movs	r3, #0x2
    514e: f8c8 2010    	str.w	r2, [r8, #0x10]
    5152: f8c8 300c    	str.w	r3, [r8, #0xc]
    5156: 2313         	movs	r3, #0x13
    5158: f8c8 2024    	str.w	r2, [r8, #0x24]
    515c: f8c8 2028    	str.w	r2, [r8, #0x28]
    5160: f8c8 2044    	str.w	r2, [r8, #0x44]
    5164: f8c8 2048    	str.w	r2, [r8, #0x48]
    5168: f8c8 305c    	str.w	r3, [r8, #0x5c]
    516c: 2314         	movs	r3, #0x14
    516e: f8c8 3060    	str.w	r3, [r8, #0x60]
    5172: f8d8 3008    	ldr.w	r3, [r8, #0x8]
    5176: f8d8 5000    	ldr.w	r5, [r8]
    517a: f003 0307    	and	r3, r3, #0x7
    517e: ed9f 1add    	vldr	s2, [pc, #884]          @ 0x54f4 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1418>
    5182: f005 0501    	and	r5, r5, #0x1
    5186: f85c 3023    	ldr.w	r3, [r12, r3, lsl #2]
    518a: 40eb         	lsrs	r3, r5
    518c: f8c8 3004    	str.w	r3, [r8, #0x4]
    5190: f8d8 3004    	ldr.w	r3, [r8, #0x4]
    5194: f36f 33df    	bfc	r3, #15, #17
    5198: ee00 3a10    	vmov	s0, r3
    519c: f8d8 3004    	ldr.w	r3, [r8, #0x4]
    51a0: eeb8 0a40    	vcvt.f32.u32	s0, s0
    51a4: f36f 33df    	bfc	r3, #15, #17
    51a8: ee20 2a01    	vmul.f32	s4, s0, s2
    51ac: ed9f 0ad2    	vldr	s0, [pc, #840]          @ 0x54f8 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x141c>
    51b0: eebc 3ac2    	vcvt.u32.f32	s6, s4
    51b4: eeb5 2a40    	vcmp.f32	s4, #0
    51b8: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    51bc: eeb4 2a40    	vcmp.f32	s4, s0
    51c0: ee13 5a10    	vmov	r5, s6
    51c4: bfb8         	it	lt
    51c6: 2500         	movlt	r5, #0x0
    51c8: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    51cc: bfc8         	it	gt
    51ce: f64f 75ff    	movwgt	r5, #0xffff
    51d2: 42ab         	cmp	r3, r5
    51d4: bf28         	it	hs
    51d6: 462b         	movhs	r3, r5
    51d8: 465d         	mov	r5, r11
    51da: f825 3f70    	strh	r3, [r5, #112]!
    51de: 80eb         	strh	r3, [r5, #0x6]
    51e0: 80ab         	strh	r3, [r5, #0x4]
    51e2: 806b         	strh	r3, [r5, #0x2]
    51e4: f8c8 2010    	str.w	r2, [r8, #0x10]
    51e8: f8c1 20f8    	str.w	r2, [r1, #0xf8]
    51ec: f8d8 200c    	ldr.w	r2, [r8, #0xc]
    51f0: f022 0203    	bic	r2, r2, #0x3
    51f4: f8c8 200c    	str.w	r2, [r8, #0xc]
    51f8: f8d8 2068    	ldr.w	r2, [r8, #0x68]
    51fc: f022 4200    	bic	r2, r2, #0x80000000
    5200: f8c8 2068    	str.w	r2, [r8, #0x68]
    5204: f06f 02ff    	mvn	r2, #0xff
    5208: f8c8 501c    	str.w	r5, [r8, #0x1c]
    520c: f8c8 6020    	str.w	r6, [r8, #0x20]
    5210: f8c1 63f8    	str.w	r6, [r1, #0x3f8]
    5214: 508e         	str	r6, [r1, r2]
    5216: f244 2240    	movw	r2, #0x4240
    521a: f2c0 020f    	movt	r2, #0xf
    521e: 4613         	mov	r3, r2
    5220: e000         	b	0x5224 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1148> @ imm = #0x0
    5222: 3b04         	subs	r3, #0x4
    5224: 680e         	ldr	r6, [r1]
    5226: b956         	cbnz	r6, 0x523e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1162> @ imm = #0x14
    5228: 2b00         	cmp	r3, #0x0
    522a: f000 8414    	beq.w	0x5a56 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x197a> @ imm = #0x828
    522e: 680e         	ldr	r6, [r1]
    5230: b92e         	cbnz	r6, 0x523e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1162> @ imm = #0xa
    5232: 680e         	ldr	r6, [r1]
    5234: 2e00         	cmp	r6, #0x0
    5236: bf04         	itt	eq
    5238: 680e         	ldreq	r6, [r1]
    523a: 2e00         	cmpeq	r6, #0x0
    523c: d0f1         	beq	0x5222 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1146> @ imm = #-0x1e
    523e: f241 1508    	movw	r5, #0x1108
    5242: f241 5904    	movw	r9, #0x1504
    5246: 2400         	movs	r4, #0x0
    5248: f2c4 0502    	movt	r5, #0x4002
    524c: f04f 0e01    	mov.w	lr, #0x1
    5250: f2c4 0902    	movt	r9, #0x4002
    5254: f647 73ff    	movw	r3, #0x7fff
    5258: 608c         	str	r4, [r1, #0x8]
    525a: 60cc         	str	r4, [r1, #0xc]
    525c: f8c5 e3f8    	str.w	lr, [r5, #0x3f8]
    5260: f8c9 4000    	str.w	r4, [r9]
    5264: f8c9 4008    	str.w	r4, [r9, #0x8]
    5268: f8c9 3004    	str.w	r3, [r9, #0x4]
    526c: 2302         	movs	r3, #0x2
    526e: f8c9 4010    	str.w	r4, [r9, #0x10]
    5272: f8c9 300c    	str.w	r3, [r9, #0xc]
    5276: 2315         	movs	r3, #0x15
    5278: f8c9 4024    	str.w	r4, [r9, #0x24]
    527c: f8c9 4028    	str.w	r4, [r9, #0x28]
    5280: f8c9 4044    	str.w	r4, [r9, #0x44]
    5284: f8c9 4048    	str.w	r4, [r9, #0x48]
    5288: f8c9 305c    	str.w	r3, [r9, #0x5c]
    528c: 2316         	movs	r3, #0x16
    528e: f8c9 3060    	str.w	r3, [r9, #0x60]
    5292: f8d9 3008    	ldr.w	r3, [r9, #0x8]
    5296: f8d9 0000    	ldr.w	r0, [r9]
    529a: f003 0307    	and	r3, r3, #0x7
    529e: f000 0001    	and	r0, r0, #0x1
    52a2: f85c 3023    	ldr.w	r3, [r12, r3, lsl #2]
    52a6: fa23 f000    	lsr.w	r0, r3, r0
    52aa: f8c9 0004    	str.w	r0, [r9, #0x4]
    52ae: f8d9 0004    	ldr.w	r0, [r9, #0x4]
    52b2: f36f 30df    	bfc	r0, #15, #17
    52b6: ee02 0a10    	vmov	s4, r0
    52ba: f8d9 0004    	ldr.w	r0, [r9, #0x4]
    52be: eeb8 2a42    	vcvt.f32.u32	s4, s4
    52c2: f36f 30df    	bfc	r0, #15, #17
    52c6: ee22 2a01    	vmul.f32	s4, s4, s2
    52ca: eebc 3ac2    	vcvt.u32.f32	s6, s4
    52ce: eeb5 2a40    	vcmp.f32	s4, #0
    52d2: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    52d6: eeb4 2a40    	vcmp.f32	s4, s0
    52da: ee13 3a10    	vmov	r3, s6
    52de: bfb8         	it	lt
    52e0: 2300         	movlt	r3, #0x0
    52e2: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    52e6: bfc8         	it	gt
    52e8: f64f 73ff    	movwgt	r3, #0xffff
    52ec: 4298         	cmp	r0, r3
    52ee: bf28         	it	hs
    52f0: 4618         	movhs	r0, r3
    52f2: f8aa 0006    	strh.w	r0, [r10, #0x6]
    52f6: f8aa 0004    	strh.w	r0, [r10, #0x4]
    52fa: 4613         	mov	r3, r2
    52fc: f8aa 0002    	strh.w	r0, [r10, #0x2]
    5300: f8aa 0000    	strh.w	r0, [r10]
    5304: f8c9 4010    	str.w	r4, [r9, #0x10]
    5308: f8c5 40f8    	str.w	r4, [r5, #0xf8]
    530c: f8d9 000c    	ldr.w	r0, [r9, #0xc]
    5310: f020 0003    	bic	r0, r0, #0x3
    5314: f8c9 000c    	str.w	r0, [r9, #0xc]
    5318: f8d9 0068    	ldr.w	r0, [r9, #0x68]
    531c: f020 4000    	bic	r0, r0, #0x80000000
    5320: f8c9 0068    	str.w	r0, [r9, #0x68]
    5324: f06f 00ff    	mvn	r0, #0xff
    5328: f8c9 a01c    	str.w	r10, [r9, #0x1c]
    532c: f8c9 e020    	str.w	lr, [r9, #0x20]
    5330: f8c5 e3f8    	str.w	lr, [r5, #0x3f8]
    5334: f845 e000    	str.w	lr, [r5, r0]
    5338: e000         	b	0x533c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1260> @ imm = #0x0
    533a: 3b04         	subs	r3, #0x4
    533c: 6828         	ldr	r0, [r5]
    533e: b950         	cbnz	r0, 0x5356 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x127a> @ imm = #0x14
    5340: 2b00         	cmp	r3, #0x0
    5342: f000 8388    	beq.w	0x5a56 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x197a> @ imm = #0x710
    5346: 6828         	ldr	r0, [r5]
    5348: b928         	cbnz	r0, 0x5356 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x127a> @ imm = #0xa
    534a: 6828         	ldr	r0, [r5]
    534c: 2800         	cmp	r0, #0x0
    534e: bf04         	itt	eq
    5350: 6828         	ldreq	r0, [r5]
    5352: 2800         	cmpeq	r0, #0x0
    5354: d0f1         	beq	0x533a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x125e> @ imm = #-0x1e
    5356: 2300         	movs	r3, #0x0
    5358: f242 1408    	movw	r4, #0x2108
    535c: 60ab         	str	r3, [r5, #0x8]
    535e: f2c4 0402    	movt	r4, #0x4002
    5362: 60eb         	str	r3, [r5, #0xc]
    5364: f242 5504    	movw	r5, #0x2504
    5368: f2c4 0502    	movt	r5, #0x4002
    536c: f04f 0e01    	mov.w	lr, #0x1
    5370: f647 70ff    	movw	r0, #0x7fff
    5374: f8c4 e3f8    	str.w	lr, [r4, #0x3f8]
    5378: 602b         	str	r3, [r5]
    537a: 60ab         	str	r3, [r5, #0x8]
    537c: 6068         	str	r0, [r5, #0x4]
    537e: 2002         	movs	r0, #0x2
    5380: 612b         	str	r3, [r5, #0x10]
    5382: 60e8         	str	r0, [r5, #0xc]
    5384: 2018         	movs	r0, #0x18
    5386: 626b         	str	r3, [r5, #0x24]
    5388: 62ab         	str	r3, [r5, #0x28]
    538a: 646b         	str	r3, [r5, #0x44]
    538c: 64ab         	str	r3, [r5, #0x48]
    538e: 65e8         	str	r0, [r5, #0x5c]
    5390: 2017         	movs	r0, #0x17
    5392: 6628         	str	r0, [r5, #0x60]
    5394: 68a8         	ldr	r0, [r5, #0x8]
    5396: 682e         	ldr	r6, [r5]
    5398: f000 0007    	and	r0, r0, #0x7
    539c: f006 0601    	and	r6, r6, #0x1
    53a0: f85c 0020    	ldr.w	r0, [r12, r0, lsl #2]
    53a4: 40f0         	lsrs	r0, r6
    53a6: 6068         	str	r0, [r5, #0x4]
    53a8: 6868         	ldr	r0, [r5, #0x4]
    53aa: f36f 30df    	bfc	r0, #15, #17
    53ae: ee02 0a10    	vmov	s4, r0
    53b2: 6868         	ldr	r0, [r5, #0x4]
    53b4: eeb8 2a42    	vcvt.f32.u32	s4, s4
    53b8: f36f 30df    	bfc	r0, #15, #17
    53bc: ee22 1a01    	vmul.f32	s2, s4, s2
    53c0: eebc 2ac1    	vcvt.u32.f32	s4, s2
    53c4: eeb5 1a40    	vcmp.f32	s2, #0
    53c8: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    53cc: eeb4 1a40    	vcmp.f32	s2, s0
    53d0: ee12 6a10    	vmov	r6, s4
    53d4: bfb8         	it	lt
    53d6: 2600         	movlt	r6, #0x0
    53d8: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    53dc: bfc8         	it	gt
    53de: f64f 76ff    	movwgt	r6, #0xffff
    53e2: 42b0         	cmp	r0, r6
    53e4: bf28         	it	hs
    53e6: 4630         	movhs	r0, r6
    53e8: f82a 0f08    	strh	r0, [r10, #8]!
    53ec: f8aa 0006    	strh.w	r0, [r10, #0x6]
    53f0: f8aa 0004    	strh.w	r0, [r10, #0x4]
    53f4: f8aa 0002    	strh.w	r0, [r10, #0x2]
    53f8: 612b         	str	r3, [r5, #0x10]
    53fa: f8c4 30f8    	str.w	r3, [r4, #0xf8]
    53fe: 68e8         	ldr	r0, [r5, #0xc]
    5400: f020 0003    	bic	r0, r0, #0x3
    5404: 60e8         	str	r0, [r5, #0xc]
    5406: 6ea8         	ldr	r0, [r5, #0x68]
    5408: f020 4000    	bic	r0, r0, #0x80000000
    540c: 66a8         	str	r0, [r5, #0x68]
    540e: f06f 00ff    	mvn	r0, #0xff
    5412: f8c5 a01c    	str.w	r10, [r5, #0x1c]
    5416: f8c5 e020    	str.w	lr, [r5, #0x20]
    541a: f8c4 e3f8    	str.w	lr, [r4, #0x3f8]
    541e: f844 e000    	str.w	lr, [r4, r0]
    5422: e000         	b	0x5426 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x134a> @ imm = #0x0
    5424: 3a04         	subs	r2, #0x4
    5426: 6820         	ldr	r0, [r4]
    5428: b950         	cbnz	r0, 0x5440 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1364> @ imm = #0x14
    542a: 2a00         	cmp	r2, #0x0
    542c: f000 8313    	beq.w	0x5a56 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x197a> @ imm = #0x626
    5430: 6820         	ldr	r0, [r4]
    5432: b928         	cbnz	r0, 0x5440 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1364> @ imm = #0xa
    5434: 6820         	ldr	r0, [r4]
    5436: 2800         	cmp	r0, #0x0
    5438: bf04         	itt	eq
    543a: 6820         	ldreq	r0, [r4]
    543c: 2800         	cmpeq	r0, #0x0
    543e: d0f1         	beq	0x5424 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1348> @ imm = #-0x1e
    5440: 2200         	movs	r2, #0x0
    5442: f240 1301    	movw	r3, #0x101
    5446: 60a2         	str	r2, [r4, #0x8]
    5448: 2604         	movs	r6, #0x4
    544a: 60e2         	str	r2, [r4, #0xc]
    544c: f8d1 01fc    	ldr.w	r0, [r1, #0x1fc]
    5450: f040 0040    	orr	r0, r0, #0x40
    5454: f8c1 01fc    	str.w	r0, [r1, #0x1fc]
    5458: f8d8 0000    	ldr.w	r0, [r8]
    545c: f24a 3104    	movw	r1, #0xa304
    5460: f2c4 0101    	movt	r1, #0x4001
    5464: f040 0001    	orr	r0, r0, #0x1
    5468: f8c8 0000    	str.w	r0, [r8]
    546c: f8d9 0000    	ldr.w	r0, [r9]
    5470: f040 0001    	orr	r0, r0, #0x1
    5474: f8c9 0000    	str.w	r0, [r9]
    5478: 6828         	ldr	r0, [r5]
    547a: f040 0001    	orr	r0, r0, #0x1
    547e: 6028         	str	r0, [r5]
    5480: f64f 60fc    	movw	r0, #0xfefc
    5484: f6cf 70ff    	movt	r0, #0xffff
    5488: 500b         	str	r3, [r1, r0]
    548a: f8c1 620c    	str.w	r6, [r1, #0x20c]
    548e: 2603         	movs	r6, #0x3
    5490: f8c1 6204    	str.w	r6, [r1, #0x204]
    5494: 680e         	ldr	r6, [r1]
    5496: f446 3680    	orr	r6, r6, #0x10000
    549a: 600e         	str	r6, [r1]
    549c: 500b         	str	r3, [r1, r0]
    549e: f64f 603c    	movw	r0, #0xfe3c
    54a2: f6cf 70ff    	movt	r0, #0xffff
    54a6: 500a         	str	r2, [r1, r0]
    54a8: f64f 5208    	movw	r2, #0xfd08
    54ac: f44f 707a    	mov.w	r0, #0x3e8
    54b0: f8c1 023c    	str.w	r0, [r1, #0x23c]
    54b4: f6cf 72ff    	movt	r2, #0xffff
    54b8: 2001         	movs	r0, #0x1
    54ba: 5088         	str	r0, [r1, r2]
    54bc: f64f 42fc    	movw	r2, #0xfcfc
    54c0: f6cf 72ff    	movt	r2, #0xffff
    54c4: 5088         	str	r0, [r1, r2]
    54c6: 680a         	ldr	r2, [r1]
    54c8: f442 3280    	orr	r2, r2, #0x10000
    54cc: 600a         	str	r2, [r1]
    54ce: f8db 403c    	ldr.w	r4, [r11, #0x3c]
    54d2: f104 01c0    	add.w	r1, r4, #0xc0
    54d6: e8d1 2f4f    	ldrexb	r2, [r1]
    54da: 2a00         	cmp	r2, #0x0
    54dc: d145         	bne	0x556a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x148e> @ imm = #0x8a
    54de: f3bf 8f5f    	dmb	sy
    54e2: e8c1 0f42    	strexb	r2, r0, [r1]
    54e6: b14a         	cbz	r2, 0x54fc <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1420> @ imm = #0x12
    54e8: e8d1 2f4f    	ldrexb	r2, [r1]
    54ec: 2a00         	cmp	r2, #0x0
    54ee: d0f8         	beq	0x54e2 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1406> @ imm = #-0x10
    54f0: e03b         	b	0x556a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x148e> @ imm = #0x76
    54f2: bf00         	nop
    54f4: 66 66 66 3f  	.word	0x3f666666
    54f8: 00 ff 7f 47  	.word	0x477fff00
    54fc: f240 3008    	movw	r0, #0x308
    5500: f3bf 8f5f    	dmb	sy
    5504: f2c2 0000    	movt	r0, #0x2000
    5508: ad06         	add	r5, sp, #0x18
    550a: 9005         	str	r0, [sp, #0x14]
    550c: f240 3008    	movw	r0, #0x308
    5510: f2c2 0000    	movt	r0, #0x2000
    5514: a903         	add	r1, sp, #0xc
    5516: 9004         	str	r0, [sp, #0x10]
    5518: f240 3008    	movw	r0, #0x308
    551c: f2c2 0000    	movt	r0, #0x2000
    5520: 9003         	str	r0, [sp, #0xc]
    5522: 4628         	mov	r0, r5
    5524: f7fc fbea    	bl	0x1cfc <etc_slightly_different::etc::motor_driver::h44161d4f3804bd0a> @ imm = #-0x382c
    5528: 4620         	mov	r0, r4
    552a: 4629         	mov	r1, r5
    552c: 22c0         	movs	r2, #0xc0
    552e: f002 f819    	bl	0x7564 <__aeabi_memcpy8> @ imm = #0x2032
    5532: 2101         	movs	r1, #0x1
    5534: f3bf 8f5f    	dmb	sy
    5538: f884 10c1    	strb.w	r1, [r4, #0xc1]
    553c: f24e 2400    	movw	r4, #0xe200
    5540: f44f 7080    	mov.w	r0, #0x100
    5544: f2ce 0400    	movt	r4, #0xe000
    5548: 6020         	str	r0, [r4]
    554a: f8db 0044    	ldr.w	r0, [r11, #0x44]
    554e: f100 02a8    	add.w	r2, r0, #0xa8
    5552: e8d2 3f4f    	ldrexb	r3, [r2]
    5556: b943         	cbnz	r3, 0x556a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x148e> @ imm = #0x10
    5558: f3bf 8f5f    	dmb	sy
    555c: e8c2 1f43    	strexb	r3, r1, [r2]
    5560: b15b         	cbz	r3, 0x557a <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x149e> @ imm = #0x16
    5562: e8d2 3f4f    	ldrexb	r3, [r2]
    5566: 2b00         	cmp	r3, #0x0
    5568: d0f8         	beq	0x555c <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1480> @ imm = #-0x10
    556a: f647 30ac    	movw	r0, #0x7bac
    556e: f3bf 8f2f    	clrex
    5572: f2c0 0000    	movt	r0, #0x0
    5576: f001 fb92    	bl	0x6c9e <core::option::unwrap_failed::h7d11d538d7f5b966> @ imm = #0x1724
    557a: f240 390c    	movw	r9, #0x30c
    557e: 2600         	movs	r6, #0x0
    5580: f2c2 0900    	movt	r9, #0x2000
    5584: f3bf 8f5f    	dmb	sy
    5588: f880 60a4    	strb.w	r6, [r0, #0xa4]
    558c: 2501         	movs	r5, #0x1
    558e: f8c0 9090    	str.w	r9, [r0, #0x90]
    5592: f3bf 8f5f    	dmb	sy
    5596: f880 50a9    	strb.w	r5, [r0, #0xa9]
    559a: f44f 7080    	mov.w	r0, #0x100
    559e: 6020         	str	r0, [r4]
    55a0: f001 fef6    	bl	0x7390 <__primask_r>    @ imm = #0x1dec
    55a4: 4604         	mov	r4, r0
    55a6: f001 feec    	bl	0x7382 <__cpsid>        @ imm = #0x1dd8
    55aa: f89b 0019    	ldrb.w	r0, [r11, #0x19]
    55ae: 2800         	cmp	r0, #0x0
    55b0: f040 8258    	bne.w	0x5a64 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1988> @ imm = #0x4b0
    55b4: 07e0         	lsls	r0, r4, #0x1f
    55b6: f88b 5019    	strb.w	r5, [r11, #0x19]
    55ba: bf08         	it	eq
    55bc: f001 fee3    	bleq	0x7386 <__cpsie>        @ imm = #0x1dc6
    55c0: f240 0038    	movw	r0, #0x38
    55c4: 2200         	movs	r2, #0x0
    55c6: f2c2 0000    	movt	r0, #0x2000
    55ca: f8d0 10dc    	ldr.w	r1, [r0, #0xdc]
    55ce: 4401         	add	r1, r0
    55d0: f881 20e0    	strb.w	r2, [r1, #0xe0]
    55d4: e9d0 2136    	ldrd	r2, r1, [r0, #216]
    55d8: 1c4b         	adds	r3, r1, #0x1
    55da: f1b3 010a    	subs.w	r1, r3, #0xa
    55de: bf18         	it	ne
    55e0: 4619         	movne	r1, r3
    55e2: f8c0 10dc    	str.w	r1, [r0, #0xdc]
    55e6: 428a         	cmp	r2, r1
    55e8: bf04         	itt	eq
    55ea: 2201         	moveq	r2, #0x1
    55ec: f880 20ea    	strbeq.w	r2, [r0, #0xea]
    55f0: 1842         	adds	r2, r0, r1
    55f2: 2101         	movs	r1, #0x1
    55f4: f882 10e0    	strb.w	r1, [r2, #0xe0]
    55f8: e9d0 3236    	ldrd	r3, r2, [r0, #216]
    55fc: 1c56         	adds	r6, r2, #0x1
    55fe: f1b6 020a    	subs.w	r2, r6, #0xa
    5602: bf18         	it	ne
    5604: 4632         	movne	r2, r6
    5606: f8c0 20dc    	str.w	r2, [r0, #0xdc]
    560a: 4293         	cmp	r3, r2
    560c: bf08         	it	eq
    560e: f880 10ea    	strbeq.w	r1, [r0, #0xea]
    5612: 1881         	adds	r1, r0, r2
    5614: 2202         	movs	r2, #0x2
    5616: f881 20e0    	strb.w	r2, [r1, #0xe0]
    561a: e9d0 2136    	ldrd	r2, r1, [r0, #216]
    561e: 1c4b         	adds	r3, r1, #0x1
    5620: f1b3 010a    	subs.w	r1, r3, #0xa
    5624: bf18         	it	ne
    5626: 4619         	movne	r1, r3
    5628: f8c0 10dc    	str.w	r1, [r0, #0xdc]
    562c: 428a         	cmp	r2, r1
    562e: bf04         	itt	eq
    5630: 2201         	moveq	r2, #0x1
    5632: f880 20ea    	strbeq.w	r2, [r0, #0xea]
    5636: 4401         	add	r1, r0
    5638: 2203         	movs	r2, #0x3
    563a: f881 20e0    	strb.w	r2, [r1, #0xe0]
    563e: e9d0 2136    	ldrd	r2, r1, [r0, #216]
    5642: 1c4b         	adds	r3, r1, #0x1
    5644: f1b3 010a    	subs.w	r1, r3, #0xa
    5648: bf18         	it	ne
    564a: 4619         	movne	r1, r3
    564c: f8c0 10dc    	str.w	r1, [r0, #0xdc]
    5650: 428a         	cmp	r2, r1
    5652: bf04         	itt	eq
    5654: 2201         	moveq	r2, #0x1
    5656: f880 20ea    	strbeq.w	r2, [r0, #0xea]
    565a: 4401         	add	r1, r0
    565c: 2204         	movs	r2, #0x4
    565e: f881 20e0    	strb.w	r2, [r1, #0xe0]
    5662: e9d0 2136    	ldrd	r2, r1, [r0, #216]
    5666: 1c4b         	adds	r3, r1, #0x1
    5668: f1b3 010a    	subs.w	r1, r3, #0xa
    566c: bf18         	it	ne
    566e: 4619         	movne	r1, r3
    5670: f8c0 10dc    	str.w	r1, [r0, #0xdc]
    5674: 428a         	cmp	r2, r1
    5676: bf04         	itt	eq
    5678: 2201         	moveq	r2, #0x1
    567a: f880 20ea    	strbeq.w	r2, [r0, #0xea]
    567e: 4401         	add	r1, r0
    5680: 2205         	movs	r2, #0x5
    5682: f881 20e0    	strb.w	r2, [r1, #0xe0]
    5686: e9d0 2136    	ldrd	r2, r1, [r0, #216]
    568a: 1c4b         	adds	r3, r1, #0x1
    568c: f1b3 010a    	subs.w	r1, r3, #0xa
    5690: bf18         	it	ne
    5692: 4619         	movne	r1, r3
    5694: f8c0 10dc    	str.w	r1, [r0, #0xdc]
    5698: 428a         	cmp	r2, r1
    569a: bf04         	itt	eq
    569c: 2201         	moveq	r2, #0x1
    569e: f880 20ea    	strbeq.w	r2, [r0, #0xea]
    56a2: 4401         	add	r1, r0
    56a4: 2206         	movs	r2, #0x6
    56a6: f881 20e0    	strb.w	r2, [r1, #0xe0]
    56aa: e9d0 2136    	ldrd	r2, r1, [r0, #216]
    56ae: 1c4b         	adds	r3, r1, #0x1
    56b0: f1b3 010a    	subs.w	r1, r3, #0xa
    56b4: bf18         	it	ne
    56b6: 4619         	movne	r1, r3
    56b8: f8c0 10dc    	str.w	r1, [r0, #0xdc]
    56bc: 428a         	cmp	r2, r1
    56be: bf04         	itt	eq
    56c0: 2201         	moveq	r2, #0x1
    56c2: f880 20ea    	strbeq.w	r2, [r0, #0xea]
    56c6: 4401         	add	r1, r0
    56c8: 2207         	movs	r2, #0x7
    56ca: f881 20e0    	strb.w	r2, [r1, #0xe0]
    56ce: e9d0 2136    	ldrd	r2, r1, [r0, #216]
    56d2: 1c4b         	adds	r3, r1, #0x1
    56d4: f1b3 010a    	subs.w	r1, r3, #0xa
    56d8: bf18         	it	ne
    56da: 4619         	movne	r1, r3
    56dc: f8c0 10dc    	str.w	r1, [r0, #0xdc]
    56e0: 428a         	cmp	r2, r1
    56e2: bf04         	itt	eq
    56e4: 2201         	moveq	r2, #0x1
    56e6: f880 20ea    	strbeq.w	r2, [r0, #0xea]
    56ea: 4401         	add	r1, r0
    56ec: 2208         	movs	r2, #0x8
    56ee: f881 20e0    	strb.w	r2, [r1, #0xe0]
    56f2: e9d0 2136    	ldrd	r2, r1, [r0, #216]
    56f6: 1c4b         	adds	r3, r1, #0x1
    56f8: f1b3 010a    	subs.w	r1, r3, #0xa
    56fc: bf18         	it	ne
    56fe: 4619         	movne	r1, r3
    5700: f8c0 10dc    	str.w	r1, [r0, #0xdc]
    5704: 428a         	cmp	r2, r1
    5706: bf04         	itt	eq
    5708: 2201         	moveq	r2, #0x1
    570a: f880 20ea    	strbeq.w	r2, [r0, #0xea]
    570e: 4401         	add	r1, r0
    5710: 2209         	movs	r2, #0x9
    5712: f881 20e0    	strb.w	r2, [r1, #0xe0]
    5716: e9d0 1236    	ldrd	r1, r2, [r0, #216]
    571a: 3201         	adds	r2, #0x1
    571c: f1b2 030a    	subs.w	r3, r2, #0xa
    5720: bf18         	it	ne
    5722: 4613         	movne	r3, r2
    5724: f8c0 30dc    	str.w	r3, [r0, #0xdc]
    5728: 4299         	cmp	r1, r3
    572a: bf04         	itt	eq
    572c: 2101         	moveq	r1, #0x1
    572e: f880 10ea    	strbeq.w	r1, [r0, #0xea]
    5732: f8c0 5100    	str.w	r5, [r0, #0x100]
    5736: f001 fe2b    	bl	0x7390 <__primask_r>    @ imm = #0x1c56
    573a: 4604         	mov	r4, r0
    573c: f001 fe21    	bl	0x7382 <__cpsid>        @ imm = #0x1c42
    5740: f89b 001a    	ldrb.w	r0, [r11, #0x1a]
    5744: 2800         	cmp	r0, #0x0
    5746: f040 8196    	bne.w	0x5a76 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x199a> @ imm = #0x32c
    574a: 07e0         	lsls	r0, r4, #0x1f
    574c: f88b 501a    	strb.w	r5, [r11, #0x1a]
    5750: bf08         	it	eq
    5752: f001 fe18    	bleq	0x7386 <__cpsie>        @ imm = #0x1c30
    5756: f240 1840    	movw	r8, #0x140
    575a: 2100         	movs	r1, #0x0
    575c: f2c2 0800    	movt	r8, #0x2000
    5760: f04f 5c7c    	mov.w	r12, #0x3f000000
    5764: f8d8 00dc    	ldr.w	r0, [r8, #0xdc]
    5768: 2600         	movs	r6, #0x0
    576a: f64c 44cd    	movw	r4, #0xcccd
    576e: f2c4 1620    	movt	r6, #0x4120
    5772: 4440         	add	r0, r8
    5774: f6cb 544c    	movt	r4, #0xbd4c
    5778: f64c 45cd    	movw	r5, #0xcccd
    577c: f880 10e0    	strb.w	r1, [r0, #0xe0]
    5780: f6cb 55cc    	movt	r5, #0xbdcc
    5784: e9d8 1036    	ldrd	r1, r0, [r8, #216]
    5788: 1c42         	adds	r2, r0, #0x1
    578a: f1b2 000a    	subs.w	r0, r2, #0xa
    578e: bf18         	it	ne
    5790: 4610         	movne	r0, r2
    5792: f8c8 00dc    	str.w	r0, [r8, #0xdc]
    5796: 4281         	cmp	r1, r0
    5798: bf04         	itt	eq
    579a: 2101         	moveq	r1, #0x1
    579c: f888 10ea    	strbeq.w	r1, [r8, #0xea]
    57a0: eb08 0100    	add.w	r1, r8, r0
    57a4: 2001         	movs	r0, #0x1
    57a6: f881 00e0    	strb.w	r0, [r1, #0xe0]
    57aa: e9d8 2136    	ldrd	r2, r1, [r8, #216]
    57ae: 1c4b         	adds	r3, r1, #0x1
    57b0: f1b3 010a    	subs.w	r1, r3, #0xa
    57b4: bf18         	it	ne
    57b6: 4619         	movne	r1, r3
    57b8: f8c8 10dc    	str.w	r1, [r8, #0xdc]
    57bc: 428a         	cmp	r2, r1
    57be: bf08         	it	eq
    57c0: f888 00ea    	strbeq.w	r0, [r8, #0xea]
    57c4: eb08 0001    	add.w	r0, r8, r1
    57c8: 2102         	movs	r1, #0x2
    57ca: f649 139a    	movw	r3, #0x999a
    57ce: f880 10e0    	strb.w	r1, [r0, #0xe0]
    57d2: f6c3 6399    	movt	r3, #0x3e99
    57d6: e9d8 1036    	ldrd	r1, r0, [r8, #216]
    57da: 1c42         	adds	r2, r0, #0x1
    57dc: f1b2 000a    	subs.w	r0, r2, #0xa
    57e0: bf18         	it	ne
    57e2: 4610         	movne	r0, r2
    57e4: f8c8 00dc    	str.w	r0, [r8, #0xdc]
    57e8: 4281         	cmp	r1, r0
    57ea: bf04         	itt	eq
    57ec: 2101         	moveq	r1, #0x1
    57ee: f888 10ea    	strbeq.w	r1, [r8, #0xea]
    57f2: 4440         	add	r0, r8
    57f4: 2103         	movs	r1, #0x3
    57f6: f880 10e0    	strb.w	r1, [r0, #0xe0]
    57fa: e9d8 1036    	ldrd	r1, r0, [r8, #216]
    57fe: 1c42         	adds	r2, r0, #0x1
    5800: f1b2 000a    	subs.w	r0, r2, #0xa
    5804: bf18         	it	ne
    5806: 4610         	movne	r0, r2
    5808: f8c8 00dc    	str.w	r0, [r8, #0xdc]
    580c: 4281         	cmp	r1, r0
    580e: bf04         	itt	eq
    5810: 2101         	moveq	r1, #0x1
    5812: f888 10ea    	strbeq.w	r1, [r8, #0xea]
    5816: 4440         	add	r0, r8
    5818: 2104         	movs	r1, #0x4
    581a: f880 10e0    	strb.w	r1, [r0, #0xe0]
    581e: e9d8 1036    	ldrd	r1, r0, [r8, #216]
    5822: 1c42         	adds	r2, r0, #0x1
    5824: f1b2 000a    	subs.w	r0, r2, #0xa
    5828: bf18         	it	ne
    582a: 4610         	movne	r0, r2
    582c: f8c8 00dc    	str.w	r0, [r8, #0xdc]
    5830: 4281         	cmp	r1, r0
    5832: bf04         	itt	eq
    5834: 2101         	moveq	r1, #0x1
    5836: f888 10ea    	strbeq.w	r1, [r8, #0xea]
    583a: 4440         	add	r0, r8
    583c: 2105         	movs	r1, #0x5
    583e: f880 10e0    	strb.w	r1, [r0, #0xe0]
    5842: e9d8 1036    	ldrd	r1, r0, [r8, #216]
    5846: 1c42         	adds	r2, r0, #0x1
    5848: f1b2 000a    	subs.w	r0, r2, #0xa
    584c: bf18         	it	ne
    584e: 4610         	movne	r0, r2
    5850: f8c8 00dc    	str.w	r0, [r8, #0xdc]
    5854: 4281         	cmp	r1, r0
    5856: bf04         	itt	eq
    5858: 2101         	moveq	r1, #0x1
    585a: f888 10ea    	strbeq.w	r1, [r8, #0xea]
    585e: 4440         	add	r0, r8
    5860: 2106         	movs	r1, #0x6
    5862: f880 10e0    	strb.w	r1, [r0, #0xe0]
    5866: e9d8 1036    	ldrd	r1, r0, [r8, #216]
    586a: 1c42         	adds	r2, r0, #0x1
    586c: f1b2 000a    	subs.w	r0, r2, #0xa
    5870: bf18         	it	ne
    5872: 4610         	movne	r0, r2
    5874: f8c8 00dc    	str.w	r0, [r8, #0xdc]
    5878: 4281         	cmp	r1, r0
    587a: bf04         	itt	eq
    587c: 2101         	moveq	r1, #0x1
    587e: f888 10ea    	strbeq.w	r1, [r8, #0xea]
    5882: 4440         	add	r0, r8
    5884: 2107         	movs	r1, #0x7
    5886: f880 10e0    	strb.w	r1, [r0, #0xe0]
    588a: e9d8 1036    	ldrd	r1, r0, [r8, #216]
    588e: 1c42         	adds	r2, r0, #0x1
    5890: f1b2 000a    	subs.w	r0, r2, #0xa
    5894: bf18         	it	ne
    5896: 4610         	movne	r0, r2
    5898: f8c8 00dc    	str.w	r0, [r8, #0xdc]
    589c: 4281         	cmp	r1, r0
    589e: bf04         	itt	eq
    58a0: 2101         	moveq	r1, #0x1
    58a2: f888 10ea    	strbeq.w	r1, [r8, #0xea]
    58a6: 4440         	add	r0, r8
    58a8: 2108         	movs	r1, #0x8
    58aa: f880 10e0    	strb.w	r1, [r0, #0xe0]
    58ae: e9d8 1036    	ldrd	r1, r0, [r8, #216]
    58b2: 1c42         	adds	r2, r0, #0x1
    58b4: f1b2 000a    	subs.w	r0, r2, #0xa
    58b8: bf18         	it	ne
    58ba: 4610         	movne	r0, r2
    58bc: f8c8 00dc    	str.w	r0, [r8, #0xdc]
    58c0: 4281         	cmp	r1, r0
    58c2: bf04         	itt	eq
    58c4: 2101         	moveq	r1, #0x1
    58c6: f888 10ea    	strbeq.w	r1, [r8, #0xea]
    58ca: 4440         	add	r0, r8
    58cc: 2109         	movs	r1, #0x9
    58ce: f880 10e0    	strb.w	r1, [r0, #0xe0]
    58d2: e9d8 0136    	ldrd	r0, r1, [r8, #216]
    58d6: 3101         	adds	r1, #0x1
    58d8: f1b1 020a    	subs.w	r2, r1, #0xa
    58dc: bf18         	it	ne
    58de: 460a         	movne	r2, r1
    58e0: f8c8 20dc    	str.w	r2, [r8, #0xdc]
    58e4: 4290         	cmp	r0, r2
    58e6: f240 22e0    	movw	r2, #0x2e0
    58ea: f2c2 0200    	movt	r2, #0x2000
    58ee: bf04         	itt	eq
    58f0: 2001         	moveq	r0, #0x1
    58f2: f888 00ea    	strbeq.w	r0, [r8, #0xea]
    58f6: f8c2 c000    	str.w	r12, [r2]
    58fa: 2201         	movs	r2, #0x1
    58fc: f8c8 2100    	str.w	r2, [r8, #0x100]
    5900: f240 22e8    	movw	r2, #0x2e8
    5904: 2100         	movs	r1, #0x0
    5906: f2c2 0200    	movt	r2, #0x2000
    590a: e9c2 1100    	strd	r1, r1, [r2]
    590e: f240 3000    	movw	r0, #0x300
    5912: e9c2 1102    	strd	r1, r1, [r2, #8]
    5916: f240 7238    	movw	r2, #0x738
    591a: f2c2 0200    	movt	r2, #0x2000
    591e: f2c2 0000    	movt	r0, #0x2000
    5922: 8011         	strh	r1, [r2]
    5924: f240 22f8    	movw	r2, #0x2f8
    5928: f2c2 0200    	movt	r2, #0x2000
    592c: 6041         	str	r1, [r0, #0x4]
    592e: 6011         	str	r1, [r2]
    5930: f240 22fc    	movw	r2, #0x2fc
    5934: f2c2 0200    	movt	r2, #0x2000
    5938: 6001         	str	r1, [r0]
    593a: 6013         	str	r3, [r2]
    593c: f240 3208    	movw	r2, #0x308
    5940: f2c2 0200    	movt	r2, #0x2000
    5944: f64c 40cd    	movw	r0, #0xcccd
    5948: 6011         	str	r1, [r2]
    594a: 2200         	movs	r2, #0x0
    594c: f6cb 7280    	movt	r2, #0xbf80
    5950: f8c9 1028    	str.w	r1, [r9, #0x28]
    5954: e9c9 6208    	strd	r6, r2, [r9, #32]
    5958: f64c 41cd    	movw	r1, #0xcccd
    595c: f243 3233    	movw	r2, #0x3333
    5960: f649 169a    	movw	r6, #0x999a
    5964: f6c3 50cc    	movt	r0, #0x3dcc
    5968: f6cb 614c    	movt	r1, #0xbe4c
    596c: e9c9 0400    	strd	r0, r4, [r9]
    5970: f109 0008    	add.w	r0, r9, #0x8
    5974: f6c3 7233    	movt	r2, #0x3f33
    5978: f6cb 6619    	movt	r6, #0xbe19
    597c: e880 1028    	stm.w	r0, {r3, r5, r12}
    5980: e9c9 6205    	strd	r6, r2, [r9, #20]
    5984: f8c9 101c    	str.w	r1, [r9, #0x1c]
    5988: f001 fcfd    	bl	0x7386 <__cpsie>        @ imm = #0x19fa
    598c: f001 fd00    	bl	0x7390 <__primask_r>    @ imm = #0x1a00
    5990: 4604         	mov	r4, r0
    5992: f001 fcf6    	bl	0x7382 <__cpsid>        @ imm = #0x19ec
    5996: f8d8 0100    	ldr.w	r0, [r8, #0x100]
    599a: 1e45         	subs	r5, r0, #0x1
    599c: 07e0         	lsls	r0, r4, #0x1f
    599e: f8c8 5100    	str.w	r5, [r8, #0x100]
    59a2: bf08         	it	eq
    59a4: f001 fcef    	bleq	0x7386 <__cpsie>        @ imm = #0x19de
    59a8: b98d         	cbnz	r5, 0x59ce <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x18f2> @ imm = #0x22
    59aa: f001 fcf1    	bl	0x7390 <__primask_r>    @ imm = #0x19e2
    59ae: 4604         	mov	r4, r0
    59b0: f001 fce7    	bl	0x7382 <__cpsid>        @ imm = #0x19ce
    59b4: e9d8 1000    	ldrd	r1, r0, [r8]
    59b8: 2200         	movs	r2, #0x0
    59ba: f8c8 2000    	str.w	r2, [r8]
    59be: 2900         	cmp	r1, #0x0
    59c0: bf1c         	itt	ne
    59c2: 6849         	ldrne	r1, [r1, #0x4]
    59c4: 4788         	blxne	r1
    59c6: 07e0         	lsls	r0, r4, #0x1f
    59c8: bf08         	it	eq
    59ca: f001 fcdc    	bleq	0x7386 <__cpsie>        @ imm = #0x19b8
    59ce: f001 fcdf    	bl	0x7390 <__primask_r>    @ imm = #0x19be
    59d2: 4604         	mov	r4, r0
    59d4: f001 fcd5    	bl	0x7382 <__cpsid>        @ imm = #0x19aa
    59d8: f04f 0901    	mov.w	r9, #0x1
    59dc: 07e0         	lsls	r0, r4, #0x1f
    59de: f888 9104    	strb.w	r9, [r8, #0x104]
    59e2: bf08         	it	eq
    59e4: f001 fccf    	bleq	0x7386 <__cpsie>        @ imm = #0x199e
    59e8: f04f 0a00    	mov.w	r10, #0x0
    59ec: f001 fcd0    	bl	0x7390 <__primask_r>    @ imm = #0x19a0
    59f0: 4604         	mov	r4, r0
    59f2: f001 fcc6    	bl	0x7382 <__cpsid>        @ imm = #0x198c
    59f6: f3bf 8f5f    	dmb	sy
    59fa: f8d8 50d0    	ldr.w	r5, [r8, #0xd0]
    59fe: b315         	cbz	r5, 0x5a46 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x196a> @ imm = #0x44
    5a00: 68a8         	ldr	r0, [r5, #0x8]
    5a02: f8c8 00d0    	str.w	r0, [r8, #0xd0]
    5a06: e9d5 1000    	ldrd	r1, r0, [r5]
    5a0a: 6809         	ldr	r1, [r1]
    5a0c: 4788         	blx	r1
    5a0e: 4606         	mov	r6, r0
    5a10: f8d8 00d4    	ldr.w	r0, [r8, #0xd4]
    5a14: 468b         	mov	r11, r1
    5a16: 4285         	cmp	r5, r0
    5a18: bf08         	it	eq
    5a1a: f8c8 a0d4    	streq.w	r10, [r8, #0xd4]
    5a1e: 68a8         	ldr	r0, [r5, #0x8]
    5a20: 2800         	cmp	r0, #0x0
    5a22: bf18         	it	ne
    5a24: f8c0 a00c    	strne.w	r10, [r0, #0xc]
    5a28: 07e0         	lsls	r0, r4, #0x1f
    5a2a: f8c5 a008    	str.w	r10, [r5, #0x8]
    5a2e: f8c5 a00c    	str.w	r10, [r5, #0xc]
    5a32: f885 9010    	strb.w	r9, [r5, #0x10]
    5a36: bf08         	it	eq
    5a38: f001 fca5    	bleq	0x7386 <__cpsie>        @ imm = #0x194a
    5a3c: b13e         	cbz	r6, 0x5a4e <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1972> @ imm = #0xe
    5a3e: 6871         	ldr	r1, [r6, #0x4]
    5a40: 4658         	mov	r0, r11
    5a42: 4788         	blx	r1
    5a44: e7d2         	b	0x59ec <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x1910> @ imm = #-0x5c
    5a46: 07e0         	lsls	r0, r4, #0x1f
    5a48: bf08         	it	eq
    5a4a: f001 fc9c    	bleq	0x7386 <__cpsie>        @ imm = #0x1938
    5a4e: b037         	add	sp, #0xdc
    5a50: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5a54: bdf0         	pop	{r4, r5, r6, r7, pc}
    5a56: f247 6148    	movw	r1, #0x7648
    5a5a: a806         	add	r0, sp, #0x18
    5a5c: f2c0 0100    	movt	r1, #0x0
    5a60: f001 f8f0    	bl	0x6c44 <core::result::unwrap_failed::h6d5ad66472df96c4> @ imm = #0x11e0
    5a64: f247 70bc    	movw	r0, #0x77bc
    5a68: 960a         	str	r6, [sp, #0x28]
    5a6a: f2c0 0000    	movt	r0, #0x0
    5a6e: 9507         	str	r5, [sp, #0x1c]
    5a70: 9006         	str	r0, [sp, #0x18]
    5a72: 9609         	str	r6, [sp, #0x24]
    5a74: e008         	b	0x5a88 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x19ac> @ imm = #0x10
    5a76: f247 71bc    	movw	r1, #0x77bc
    5a7a: 2000         	movs	r0, #0x0
    5a7c: f2c0 0100    	movt	r1, #0x0
    5a80: 900a         	str	r0, [sp, #0x28]
    5a82: 9507         	str	r5, [sp, #0x1c]
    5a84: 9106         	str	r1, [sp, #0x18]
    5a86: 9009         	str	r0, [sp, #0x24]
    5a88: 2004         	movs	r0, #0x4
    5a8a: f647 31ac    	movw	r1, #0x7bac
    5a8e: 9008         	str	r0, [sp, #0x20]
    5a90: a806         	add	r0, sp, #0x18
    5a92: f2c0 0100    	movt	r1, #0x0
    5a96: f000 f938    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #0x270
    5a9a: d4d4         	bmi	0x5a46 <etc_slightly_different::etc::main::__rtic_init_resources::h343b7af831465115+0x196a> @ imm = #-0x58

00005a9c <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430>:
    5a9c: 6843         	ldr	r3, [r0, #0x4]
    5a9e: f246 1c00    	movw	r12, #0x6100
    5aa2: f2c4 0c00    	movt	r12, #0x4000
    5aa6: e004         	b	0x5ab2 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0x16> @ imm = #0x8
    5aa8: f8dc 1010    	ldr.w	r1, [r12, #0x10]
    5aac: 4613         	mov	r3, r2
    5aae: 2900         	cmp	r1, #0x0
    5ab0: d147         	bne	0x5b42 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0xa6> @ imm = #0x8e
    5ab2: 1c5a         	adds	r2, r3, #0x1
    5ab4: 2b05         	cmp	r3, #0x5
    5ab6: d828         	bhi	0x5b0a <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0x6e> @ imm = #0x50
    5ab8: e8df f003    	tbb	[pc, r3]
    5abc: 04 16 0a 10  	.word	0x100a1604
    5ac0: 03 1c        	.short	0x1c03
    5ac2: e7f1         	b	0x5aa8 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0xc> @ imm = #-0x1e
    5ac4: f8dc 1000    	ldr.w	r1, [r12]
    5ac8: 4613         	mov	r3, r2
    5aca: 2900         	cmp	r1, #0x0
    5acc: d0f1         	beq	0x5ab2 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0x16> @ imm = #-0x1e
    5ace: e044         	b	0x5b5a <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0xbe> @ imm = #0x88
    5ad0: f8dc 1008    	ldr.w	r1, [r12, #0x8]
    5ad4: 4613         	mov	r3, r2
    5ad6: 2900         	cmp	r1, #0x0
    5ad8: d0eb         	beq	0x5ab2 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0x16> @ imm = #-0x2a
    5ada: e02c         	b	0x5b36 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0x9a> @ imm = #0x58
    5adc: f8dc 100c    	ldr.w	r1, [r12, #0xc]
    5ae0: 4613         	mov	r3, r2
    5ae2: 2900         	cmp	r1, #0x0
    5ae4: d0e5         	beq	0x5ab2 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0x16> @ imm = #-0x36
    5ae6: e032         	b	0x5b4e <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0xb2> @ imm = #0x64
    5ae8: f8dc 1004    	ldr.w	r1, [r12, #0x4]
    5aec: 4613         	mov	r3, r2
    5aee: 2900         	cmp	r1, #0x0
    5af0: d0df         	beq	0x5ab2 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0x16> @ imm = #-0x42
    5af2: e038         	b	0x5b66 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0xca> @ imm = #0x70
    5af4: f8dc 1014    	ldr.w	r1, [r12, #0x14]
    5af8: 4613         	mov	r3, r2
    5afa: 2900         	cmp	r1, #0x0
    5afc: d0d9         	beq	0x5ab2 <<bsp::events::Iter as core::iter::traits::iterator::Iterator>::next::h18066dafc96e5430+0x16> @ imm = #-0x4e
    5afe: 2101         	movs	r1, #0x1
    5b00: f84c 1c8c    	str	r1, [r12, #-140]
    5b04: 6042         	str	r2, [r0, #0x4]
    5b06: 2005         	movs	r0, #0x5
    5b08: 4770         	bx	lr
    5b0a: 2100         	movs	r1, #0x0
    5b0c: f8cc 1000    	str.w	r1, [r12]
    5b10: f8cc 1004    	str.w	r1, [r12, #0x4]
    5b14: f8cc 1008    	str.w	r1, [r12, #0x8]
    5b18: f8cc 100c    	str.w	r1, [r12, #0xc]
    5b1c: f8cc 1010    	str.w	r1, [r12, #0x10]
    5b20: f8cc 1014    	str.w	r1, [r12, #0x14]
    5b24: f8cc 1018    	str.w	r1, [r12, #0x18]
    5b28: 6042         	str	r2, [r0, #0x4]
    5b2a: 2007         	movs	r0, #0x7
    5b2c: f8cc 101c    	str.w	r1, [r12, #0x1c]
    5b30: f8cc 107c    	str.w	r1, [r12, #0x7c]
    5b34: 4770         	bx	lr
    5b36: 2101         	movs	r1, #0x1
    5b38: f84c 1c98    	str	r1, [r12, #-152]
    5b3c: 6042         	str	r2, [r0, #0x4]
    5b3e: 2002         	movs	r0, #0x2
    5b40: 4770         	bx	lr
    5b42: 2101         	movs	r1, #0x1
    5b44: f84c 1c90    	str	r1, [r12, #-144]
    5b48: 6042         	str	r2, [r0, #0x4]
    5b4a: 2004         	movs	r0, #0x4
    5b4c: 4770         	bx	lr
    5b4e: 2101         	movs	r1, #0x1
    5b50: f84c 1c94    	str	r1, [r12, #-148]
    5b54: 6042         	str	r2, [r0, #0x4]
    5b56: 2003         	movs	r0, #0x3
    5b58: 4770         	bx	lr
    5b5a: 2101         	movs	r1, #0x1
    5b5c: f84c 1ca0    	str	r1, [r12, #-160]
    5b60: 6042         	str	r2, [r0, #0x4]
    5b62: 2000         	movs	r0, #0x0
    5b64: 4770         	bx	lr
    5b66: 2101         	movs	r1, #0x1
    5b68: f84c 1c9c    	str	r1, [r12, #-156]
    5b6c: 6042         	str	r2, [r0, #0x4]
    5b6e: 2001         	movs	r0, #0x1
    5b70: 4770         	bx	lr

00005b72 <get_u8>:
    5b72: eeb5 0a40    	vcmp.f32	s0, #0
    5b76: eef1 fa10    	vmrs	APSR_nzcv, fpscr
    5b7a: bfb8         	it	lt
    5b7c: 2015         	movlt	r0, #0x15
    5b7e: 4770         	bx	lr

00005b80 <_defmt_timestamp>:
    5b80: b580         	push	{r7, lr}
    5b82: 466f         	mov	r7, sp
    5b84: b082         	sub	sp, #0x8
    5b86: f240 2048    	movw	r0, #0x248
    5b8a: f2c2 0000    	movt	r0, #0x2000
    5b8e: 6801         	ldr	r1, [r0]
    5b90: 1c4a         	adds	r2, r1, #0x1
    5b92: 6002         	str	r2, [r0]
    5b94: a801         	add	r0, sp, #0x4
    5b96: 9101         	str	r1, [sp, #0x4]
    5b98: 2104         	movs	r1, #0x4
    5b9a: f001 fa40    	bl	0x701e <_defmt_write>   @ imm = #0x1480
    5b9e: b002         	add	sp, #0x8
    5ba0: bd80         	pop	{r7, pc}

00005ba2 <core::slice::index::slice_start_index_len_fail::h0109ceeb844e56f0>:
    5ba2: b580         	push	{r7, lr}
    5ba4: 466f         	mov	r7, sp
    5ba6: b08c         	sub	sp, #0x30
    5ba8: e9cd 0100    	strd	r0, r1, [sp]
    5bac: 2000         	movs	r0, #0x0
    5bae: f647 11e4    	movw	r1, #0x79e4
    5bb2: 9006         	str	r0, [sp, #0x18]
    5bb4: 2002         	movs	r0, #0x2
    5bb6: f2c0 0100    	movt	r1, #0x0
    5bba: 9003         	str	r0, [sp, #0xc]
    5bbc: 9005         	str	r0, [sp, #0x14]
    5bbe: a808         	add	r0, sp, #0x20
    5bc0: 9102         	str	r1, [sp, #0x8]
    5bc2: a901         	add	r1, sp, #0x4
    5bc4: 9004         	str	r0, [sp, #0x10]
    5bc6: f645 401d    	movw	r0, #0x5c1d
    5bca: f2c0 0000    	movt	r0, #0x0
    5bce: e9cd 0109    	strd	r0, r1, [sp, #36]
    5bd2: 4611         	mov	r1, r2
    5bd4: 900b         	str	r0, [sp, #0x2c]
    5bd6: 4668         	mov	r0, sp
    5bd8: 9008         	str	r0, [sp, #0x20]
    5bda: a802         	add	r0, sp, #0x8
    5bdc: f000 f895    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #0x12a

00005be0 <core::panicking::panic_bounds_check::h69088854c118b8d4>:
    5be0: b580         	push	{r7, lr}
    5be2: 466f         	mov	r7, sp
    5be4: b08c         	sub	sp, #0x30
    5be6: e9cd 0100    	strd	r0, r1, [sp]
    5bea: 2000         	movs	r0, #0x0
    5bec: 9006         	str	r0, [sp, #0x18]
    5bee: 2002         	movs	r0, #0x2
    5bf0: 4908         	ldr	r1, [pc, #0x20]         @ 0x5c14 <core::panicking::panic_bounds_check::h69088854c118b8d4+0x34>
    5bf2: 9102         	str	r1, [sp, #0x8]
    5bf4: 4669         	mov	r1, sp
    5bf6: 9003         	str	r0, [sp, #0xc]
    5bf8: 9005         	str	r0, [sp, #0x14]
    5bfa: a808         	add	r0, sp, #0x20
    5bfc: 9004         	str	r0, [sp, #0x10]
    5bfe: 4806         	ldr	r0, [pc, #0x18]         @ 0x5c18 <core::panicking::panic_bounds_check::h69088854c118b8d4+0x38>
    5c00: e9cd 0109    	strd	r0, r1, [sp, #36]
    5c04: 4611         	mov	r1, r2
    5c06: 900b         	str	r0, [sp, #0x2c]
    5c08: a801         	add	r0, sp, #0x4
    5c0a: 9008         	str	r0, [sp, #0x20]
    5c0c: a802         	add	r0, sp, #0x8
    5c0e: f000 f87c    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #0xf8
    5c12: bf00         	nop
    5c14: 74 78 00 00  	.word	0x00007874
    5c18: 1d 5c 00 00  	.word	0x00005c1d

00005c1c <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt::h0c8c36a078f52568>:
    5c1c: b5f0         	push	{r4, r5, r6, r7, lr}
    5c1e: af03         	add	r7, sp, #0xc
    5c20: e92d 0f00    	push.w	{r8, r9, r10, r11}
    5c24: b08d         	sub	sp, #0x34
    5c26: 6804         	ldr	r4, [r0]
    5c28: f242 7010    	movw	r0, #0x2710
    5c2c: 460d         	mov	r5, r1
    5c2e: f64f 7e9c    	movw	lr, #0xff9c
    5c32: 4284         	cmp	r4, r0
    5c34: f647 00e6    	movw	r0, #0x78e6
    5c38: f2c0 0000    	movt	r0, #0x0
    5c3c: d33c         	blo	0x5cb8 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt::h0c8c36a078f52568+0x9c> @ imm = #0x78
    5c3e: f241 7859    	movw	r8, #0x1759
    5c42: f24e 02ff    	movw	r2, #0xe0ff
    5c46: f1a7 0b43    	sub.w	r11, r7, #0x43
    5c4a: 2100         	movs	r1, #0x0
    5c4c: f2cd 18b7    	movt	r8, #0xd1b7
    5c50: f64d 09f0    	movw	r9, #0xd8f0
    5c54: f241 4a7b    	movw	r10, #0x147b
    5c58: f2c0 52f5    	movt	r2, #0x5f5
    5c5c: 9502         	str	r5, [sp, #0x8]
    5c5e: fba4 3508    	umull	r3, r5, r4, r8
    5c62: eb0b 0c01    	add.w	r12, r11, r1
    5c66: 3904         	subs	r1, #0x4
    5c68: 4294         	cmp	r4, r2
    5c6a: ea4f 3355    	lsr.w	r3, r5, #0xd
    5c6e: fb03 4509    	mla	r5, r3, r9, r4
    5c72: 461c         	mov	r4, r3
    5c74: b2ae         	uxth	r6, r5
    5c76: ea4f 0696    	lsr.w	r6, r6, #0x2
    5c7a: fb06 f60a    	mul	r6, r6, r10
    5c7e: ea4f 4656    	lsr.w	r6, r6, #0x11
    5c82: fb06 550e    	mla	r5, r6, lr, r5
    5c86: f830 6016    	ldrh.w	r6, [r0, r6, lsl #1]
    5c8a: f8ac 6023    	strh.w	r6, [r12, #0x23]
    5c8e: b2ad         	uxth	r5, r5
    5c90: f830 5015    	ldrh.w	r5, [r0, r5, lsl #1]
    5c94: f8ac 5025    	strh.w	r5, [r12, #0x25]
    5c98: d8e1         	bhi	0x5c5e <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt::h0c8c36a078f52568+0x42> @ imm = #-0x3e
    5c9a: 9d02         	ldr	r5, [sp, #0x8]
    5c9c: 3127         	adds	r1, #0x27
    5c9e: 461c         	mov	r4, r3
    5ca0: 2c63         	cmp	r4, #0x63
    5ca2: d80c         	bhi	0x5cbe <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt::h0c8c36a078f52568+0xa2> @ imm = #0x18
    5ca4: 4622         	mov	r2, r4
    5ca6: 2a0a         	cmp	r2, #0xa
    5ca8: d31a         	blo	0x5ce0 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt::h0c8c36a078f52568+0xc4> @ imm = #0x34
    5caa: f830 0012    	ldrh.w	r0, [r0, r2, lsl #1]
    5cae: 3902         	subs	r1, #0x2
    5cb0: f1a7 0243    	sub.w	r2, r7, #0x43
    5cb4: 5250         	strh	r0, [r2, r1]
    5cb6: e019         	b	0x5cec <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt::h0c8c36a078f52568+0xd0> @ imm = #0x32
    5cb8: 2127         	movs	r1, #0x27
    5cba: 2c63         	cmp	r4, #0x63
    5cbc: d9f2         	bls	0x5ca4 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt::h0c8c36a078f52568+0x88> @ imm = #-0x1c
    5cbe: b2a2         	uxth	r2, r4
    5cc0: f241 437b    	movw	r3, #0x147b
    5cc4: 0892         	lsrs	r2, r2, #0x2
    5cc6: 3902         	subs	r1, #0x2
    5cc8: 435a         	muls	r2, r3, r2
    5cca: f1a7 0643    	sub.w	r6, r7, #0x43
    5cce: 0c52         	lsrs	r2, r2, #0x11
    5cd0: fb02 430e    	mla	r3, r2, lr, r4
    5cd4: b29b         	uxth	r3, r3
    5cd6: f830 3013    	ldrh.w	r3, [r0, r3, lsl #1]
    5cda: 5273         	strh	r3, [r6, r1]
    5cdc: 2a0a         	cmp	r2, #0xa
    5cde: d2e4         	bhs	0x5caa <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt::h0c8c36a078f52568+0x8e> @ imm = #-0x38
    5ce0: 3901         	subs	r1, #0x1
    5ce2: f042 0030    	orr	r0, r2, #0x30
    5ce6: f1a7 0243    	sub.w	r2, r7, #0x43
    5cea: 5450         	strb	r0, [r2, r1]
    5cec: f1c1 0027    	rsb.w	r0, r1, #0x27
    5cf0: 9000         	str	r0, [sp]
    5cf2: f1a7 0043    	sub.w	r0, r7, #0x43
    5cf6: 2200         	movs	r2, #0x0
    5cf8: 1843         	adds	r3, r0, r1
    5cfa: 4628         	mov	r0, r5
    5cfc: 2101         	movs	r1, #0x1
    5cfe: f000 f815    	bl	0x5d2c <core::fmt::Formatter::pad_integral::h9814144aa367965a> @ imm = #0x2a
    5d02: b00d         	add	sp, #0x34
    5d04: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5d08: bdf0         	pop	{r4, r5, r6, r7, pc}

00005d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026>:
    5d0a: b580         	push	{r7, lr}
    5d0c: 466f         	mov	r7, sp
    5d0e: b088         	sub	sp, #0x20
    5d10: e890 4174    	ldm.w	r0, {r2, r4, r5, r6, r8, lr}
    5d14: 46ec         	mov	r12, sp
    5d16: 4663         	mov	r3, r12
    5d18: 2001         	movs	r0, #0x1
    5d1a: e883 4174    	stm.w	r3, {r2, r4, r5, r6, r8, lr}
    5d1e: f8ad 001c    	strh.w	r0, [sp, #0x1c]
    5d22: 4660         	mov	r0, r12
    5d24: 9106         	str	r1, [sp, #0x18]
    5d26: f001 fa72    	bl	0x720e <rust_begin_unwind> @ imm = #0x14e4
    5d2a: d4d4         	bmi	0x5cd6 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt::h0c8c36a078f52568+0xba> @ imm = #-0x58

00005d2c <core::fmt::Formatter::pad_integral::h9814144aa367965a>:
    5d2c: b5f0         	push	{r4, r5, r6, r7, lr}
    5d2e: af03         	add	r7, sp, #0xc
    5d30: e92d 0f00    	push.w	{r8, r9, r10, r11}
    5d34: b085         	sub	sp, #0x14
    5d36: 460c         	mov	r4, r1
    5d38: 69c1         	ldr	r1, [r0, #0x1c]
    5d3a: f8d7 8008    	ldr.w	r8, [r7, #0x8]
    5d3e: 4693         	mov	r11, r2
    5d40: f011 0201    	ands	r2, r1, #0x1
    5d44: f04f 0a2b    	mov.w	r10, #0x2b
    5d48: eb02 0508    	add.w	r5, r2, r8
    5d4c: bf08         	it	eq
    5d4e: f44f 1a88    	moveq.w	r10, #0x110000
    5d52: 4699         	mov	r9, r3
    5d54: 074a         	lsls	r2, r1, #0x1d
    5d56: d412         	bmi	0x5d7e <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x52> @ imm = #0x24
    5d58: 2400         	movs	r4, #0x0
    5d5a: 6802         	ldr	r2, [r0]
    5d5c: bb7a         	cbnz	r2, 0x5dbe <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x92> @ imm = #0x5e
    5d5e: e9d0 5605    	ldrd	r5, r6, [r0, #20]
    5d62: 4652         	mov	r2, r10
    5d64: 4631         	mov	r1, r6
    5d66: 4623         	mov	r3, r4
    5d68: f8cd b000    	str.w	r11, [sp]
    5d6c: 4628         	mov	r0, r5
    5d6e: f000 f8e0    	bl	0x5f32 <core::fmt::Formatter::pad_integral::write_prefix::h7f7c9690b1602149> @ imm = #0x1c0
    5d72: b1d0         	cbz	r0, 0x5daa <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x7e> @ imm = #0x34
    5d74: 2001         	movs	r0, #0x1
    5d76: b005         	add	sp, #0x14
    5d78: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5d7c: bdf0         	pop	{r4, r5, r6, r7, pc}
    5d7e: f1bb 0f00    	cmp.w	r11, #0x0
    5d82: d016         	beq	0x5db2 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x86> @ imm = #0x2c
    5d84: f01b 0303    	ands	r3, r11, #0x3
    5d88: d015         	beq	0x5db6 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x8a> @ imm = #0x2a
    5d8a: f994 6000    	ldrsb.w	r6, [r4]
    5d8e: 2200         	movs	r2, #0x0
    5d90: f116 0f41    	cmn.w	r6, #0x41
    5d94: bfc8         	it	gt
    5d96: 2201         	movgt	r2, #0x1
    5d98: 2b01         	cmp	r3, #0x1
    5d9a: d00c         	beq	0x5db6 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x8a> @ imm = #0x18
    5d9c: f994 3001    	ldrsb.w	r3, [r4, #0x1]
    5da0: f113 0f41    	cmn.w	r3, #0x41
    5da4: bfc8         	it	gt
    5da6: 3201         	addgt	r2, #0x1
    5da8: e005         	b	0x5db6 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x8a> @ imm = #0xa
    5daa: 68f3         	ldr	r3, [r6, #0xc]
    5dac: 4628         	mov	r0, r5
    5dae: 4649         	mov	r1, r9
    5db0: e05a         	b	0x5e68 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x13c> @ imm = #0xb4
    5db2: 2200         	movs	r2, #0x0
    5db4: e7ff         	b	0x5db6 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x8a> @ imm = #-0x2
    5db6: 4415         	add	r5, r2
    5db8: 6802         	ldr	r2, [r0]
    5dba: 2a00         	cmp	r2, #0x0
    5dbc: d0cf         	beq	0x5d5e <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x32> @ imm = #-0x62
    5dbe: f8cd 9010    	str.w	r9, [sp, #0x10]
    5dc2: f8d0 9004    	ldr.w	r9, [r0, #0x4]
    5dc6: 45a9         	cmp	r9, r5
    5dc8: d910         	bls	0x5dec <core::fmt::Formatter::pad_integral::h9814144aa367965a+0xc0> @ imm = #0x20
    5dca: 0709         	lsls	r1, r1, #0x1c
    5dcc: f8cd 800c    	str.w	r8, [sp, #0xc]
    5dd0: d41c         	bmi	0x5e0c <core::fmt::Formatter::pad_integral::h9814144aa367965a+0xe0> @ imm = #0x38
    5dd2: f890 1020    	ldrb.w	r1, [r0, #0x20]
    5dd6: eba9 0905    	sub.w	r9, r9, r5
    5dda: 9402         	str	r4, [sp, #0x8]
    5ddc: e8df f001    	tbb	[pc, r1]
    5de0: 5c 02 56 02  	.word	0x0256025c
    5de4: 4649         	mov	r1, r9
    5de6: f04f 0900    	mov.w	r9, #0x0
    5dea: e055         	b	0x5e98 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x16c> @ imm = #0xaa
    5dec: e9d0 5605    	ldrd	r5, r6, [r0, #20]
    5df0: 4652         	mov	r2, r10
    5df2: 4631         	mov	r1, r6
    5df4: 4623         	mov	r3, r4
    5df6: f8cd b000    	str.w	r11, [sp]
    5dfa: 4628         	mov	r0, r5
    5dfc: f000 f899    	bl	0x5f32 <core::fmt::Formatter::pad_integral::write_prefix::h7f7c9690b1602149> @ imm = #0x132
    5e00: b378         	cbz	r0, 0x5e62 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x136> @ imm = #0x5e
    5e02: 2001         	movs	r0, #0x1
    5e04: b005         	add	sp, #0x14
    5e06: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5e0a: bdf0         	pop	{r4, r5, r6, r7, pc}
    5e0c: 6901         	ldr	r1, [r0, #0x10]
    5e0e: 4623         	mov	r3, r4
    5e10: e9d0 8605    	ldrd	r8, r6, [r0, #20]
    5e14: 4652         	mov	r2, r10
    5e16: 9101         	str	r1, [sp, #0x4]
    5e18: 2130         	movs	r1, #0x30
    5e1a: 6101         	str	r1, [r0, #0x10]
    5e1c: 2101         	movs	r1, #0x1
    5e1e: f890 4020    	ldrb.w	r4, [r0, #0x20]
    5e22: f880 1020    	strb.w	r1, [r0, #0x20]
    5e26: 4631         	mov	r1, r6
    5e28: 9002         	str	r0, [sp, #0x8]
    5e2a: 4640         	mov	r0, r8
    5e2c: f8cd b000    	str.w	r11, [sp]
    5e30: f000 f87f    	bl	0x5f32 <core::fmt::Formatter::pad_integral::write_prefix::h7f7c9690b1602149> @ imm = #0xfe
    5e34: 4601         	mov	r1, r0
    5e36: 2001         	movs	r0, #0x1
    5e38: 2900         	cmp	r1, #0x0
    5e3a: d19c         	bne	0x5d76 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x4a> @ imm = #-0xc8
    5e3c: f8dd a010    	ldr.w	r10, [sp, #0x10]
    5e40: eba9 0005    	sub.w	r0, r9, r5
    5e44: 46a3         	mov	r11, r4
    5e46: 1c44         	adds	r4, r0, #0x1
    5e48: 3c01         	subs	r4, #0x1
    5e4a: d014         	beq	0x5e76 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x14a> @ imm = #0x28
    5e4c: 6932         	ldr	r2, [r6, #0x10]
    5e4e: 4640         	mov	r0, r8
    5e50: 2130         	movs	r1, #0x30
    5e52: 4790         	blx	r2
    5e54: 2800         	cmp	r0, #0x0
    5e56: d0f7         	beq	0x5e48 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x11c> @ imm = #-0x12
    5e58: 2001         	movs	r0, #0x1
    5e5a: b005         	add	sp, #0x14
    5e5c: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5e60: bdf0         	pop	{r4, r5, r6, r7, pc}
    5e62: 68f3         	ldr	r3, [r6, #0xc]
    5e64: 4628         	mov	r0, r5
    5e66: 9904         	ldr	r1, [sp, #0x10]
    5e68: 4642         	mov	r2, r8
    5e6a: b005         	add	sp, #0x14
    5e6c: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5e70: e8bd 40f0    	pop.w	{r4, r5, r6, r7, lr}
    5e74: 4718         	bx	r3
    5e76: 68f3         	ldr	r3, [r6, #0xc]
    5e78: 4640         	mov	r0, r8
    5e7a: 9a03         	ldr	r2, [sp, #0xc]
    5e7c: 4651         	mov	r1, r10
    5e7e: 4798         	blx	r3
    5e80: b3b8         	cbz	r0, 0x5ef2 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x1c6> @ imm = #0x6e
    5e82: 2001         	movs	r0, #0x1
    5e84: b005         	add	sp, #0x14
    5e86: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5e8a: bdf0         	pop	{r4, r5, r6, r7, pc}
    5e8c: f109 0201    	add.w	r2, r9, #0x1
    5e90: ea4f 0159    	lsr.w	r1, r9, #0x1
    5e94: ea4f 0952    	lsr.w	r9, r2, #0x1
    5e98: e9d0 6504    	ldrd	r6, r5, [r0, #16]
    5e9c: 1c4c         	adds	r4, r1, #0x1
    5e9e: f8d0 8018    	ldr.w	r8, [r0, #0x18]
    5ea2: 3c01         	subs	r4, #0x1
    5ea4: d00b         	beq	0x5ebe <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x192> @ imm = #0x16
    5ea6: f8d8 2010    	ldr.w	r2, [r8, #0x10]
    5eaa: 4628         	mov	r0, r5
    5eac: 4631         	mov	r1, r6
    5eae: 4790         	blx	r2
    5eb0: 2800         	cmp	r0, #0x0
    5eb2: d0f6         	beq	0x5ea2 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x176> @ imm = #-0x14
    5eb4: 2001         	movs	r0, #0x1
    5eb6: b005         	add	sp, #0x14
    5eb8: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5ebc: bdf0         	pop	{r4, r5, r6, r7, pc}
    5ebe: f8cd b000    	str.w	r11, [sp]
    5ec2: 4628         	mov	r0, r5
    5ec4: 9b02         	ldr	r3, [sp, #0x8]
    5ec6: 4641         	mov	r1, r8
    5ec8: 4652         	mov	r2, r10
    5eca: f000 f832    	bl	0x5f32 <core::fmt::Formatter::pad_integral::write_prefix::h7f7c9690b1602149> @ imm = #0x64
    5ece: b120         	cbz	r0, 0x5eda <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x1ae> @ imm = #0x8
    5ed0: 2001         	movs	r0, #0x1
    5ed2: b005         	add	sp, #0x14
    5ed4: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5ed8: bdf0         	pop	{r4, r5, r6, r7, pc}
    5eda: f8d8 300c    	ldr.w	r3, [r8, #0xc]
    5ede: 4628         	mov	r0, r5
    5ee0: e9dd 2103    	ldrd	r2, r1, [sp, #12]
    5ee4: 4798         	blx	r3
    5ee6: b170         	cbz	r0, 0x5f06 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x1da> @ imm = #0x1c
    5ee8: 2001         	movs	r0, #0x1
    5eea: b005         	add	sp, #0x14
    5eec: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5ef0: bdf0         	pop	{r4, r5, r6, r7, pc}
    5ef2: 9802         	ldr	r0, [sp, #0x8]
    5ef4: 9901         	ldr	r1, [sp, #0x4]
    5ef6: f880 b020    	strb.w	r11, [r0, #0x20]
    5efa: 6101         	str	r1, [r0, #0x10]
    5efc: 2000         	movs	r0, #0x0
    5efe: b005         	add	sp, #0x14
    5f00: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5f04: bdf0         	pop	{r4, r5, r6, r7, pc}
    5f06: 2400         	movs	r4, #0x0
    5f08: 45a1         	cmp	r9, r4
    5f0a: d009         	beq	0x5f20 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x1f4> @ imm = #0x12
    5f0c: f8d8 2010    	ldr.w	r2, [r8, #0x10]
    5f10: 4628         	mov	r0, r5
    5f12: 4631         	mov	r1, r6
    5f14: 4790         	blx	r2
    5f16: 3401         	adds	r4, #0x1
    5f18: 2800         	cmp	r0, #0x0
    5f1a: d0f5         	beq	0x5f08 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x1dc> @ imm = #-0x16
    5f1c: 1e61         	subs	r1, r4, #0x1
    5f1e: e000         	b	0x5f22 <core::fmt::Formatter::pad_integral::h9814144aa367965a+0x1f6> @ imm = #0x0
    5f20: 4649         	mov	r1, r9
    5f22: 2000         	movs	r0, #0x0
    5f24: 4549         	cmp	r1, r9
    5f26: bf38         	it	lo
    5f28: 2001         	movlo	r0, #0x1
    5f2a: b005         	add	sp, #0x14
    5f2c: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    5f30: bdf0         	pop	{r4, r5, r6, r7, pc}

00005f32 <core::fmt::Formatter::pad_integral::write_prefix::h7f7c9690b1602149>:
    5f32: b5f0         	push	{r4, r5, r6, r7, lr}
    5f34: af03         	add	r7, sp, #0xc
    5f36: f84d 8d04    	str	r8, [sp, #-4]!
    5f3a: f8d7 8008    	ldr.w	r8, [r7, #0x8]
    5f3e: 461c         	mov	r4, r3
    5f40: 460e         	mov	r6, r1
    5f42: f5b2 1f88    	cmp.w	r2, #0x110000
    5f46: d00a         	beq	0x5f5e <core::fmt::Formatter::pad_integral::write_prefix::h7f7c9690b1602149+0x2c> @ imm = #0x14
    5f48: 6933         	ldr	r3, [r6, #0x10]
    5f4a: 4611         	mov	r1, r2
    5f4c: 4605         	mov	r5, r0
    5f4e: 4798         	blx	r3
    5f50: 4601         	mov	r1, r0
    5f52: 4628         	mov	r0, r5
    5f54: b119         	cbz	r1, 0x5f5e <core::fmt::Formatter::pad_integral::write_prefix::h7f7c9690b1602149+0x2c> @ imm = #0x6
    5f56: 2001         	movs	r0, #0x1
    5f58: f85d 8b04    	ldr	r8, [sp], #4
    5f5c: bdf0         	pop	{r4, r5, r6, r7, pc}
    5f5e: b13c         	cbz	r4, 0x5f70 <core::fmt::Formatter::pad_integral::write_prefix::h7f7c9690b1602149+0x3e> @ imm = #0xe
    5f60: 68f3         	ldr	r3, [r6, #0xc]
    5f62: 4621         	mov	r1, r4
    5f64: 4642         	mov	r2, r8
    5f66: f85d 8b04    	ldr	r8, [sp], #4
    5f6a: e8bd 40f0    	pop.w	{r4, r5, r6, r7, lr}
    5f6e: 4718         	bx	r3
    5f70: 2000         	movs	r0, #0x0
    5f72: f85d 8b04    	ldr	r8, [sp], #4
    5f76: bdf0         	pop	{r4, r5, r6, r7, pc}

00005f78 <core::slice::index::slice_end_index_len_fail::h925ebdeb5ea0b7ba>:
    5f78: b580         	push	{r7, lr}
    5f7a: 466f         	mov	r7, sp
    5f7c: b08c         	sub	sp, #0x30
    5f7e: e9cd 0100    	strd	r0, r1, [sp]
    5f82: 2000         	movs	r0, #0x0
    5f84: f647 2104    	movw	r1, #0x7a04
    5f88: 9006         	str	r0, [sp, #0x18]
    5f8a: 2002         	movs	r0, #0x2
    5f8c: f2c0 0100    	movt	r1, #0x0
    5f90: 9003         	str	r0, [sp, #0xc]
    5f92: 9005         	str	r0, [sp, #0x14]
    5f94: a808         	add	r0, sp, #0x20
    5f96: 9102         	str	r1, [sp, #0x8]
    5f98: a901         	add	r1, sp, #0x4
    5f9a: 9004         	str	r0, [sp, #0x10]
    5f9c: f645 401d    	movw	r0, #0x5c1d
    5fa0: f2c0 0000    	movt	r0, #0x0
    5fa4: e9cd 0109    	strd	r0, r1, [sp, #36]
    5fa8: 4611         	mov	r1, r2
    5faa: 900b         	str	r0, [sp, #0x2c]
    5fac: 4668         	mov	r0, sp
    5fae: 9008         	str	r0, [sp, #0x20]
    5fb0: a802         	add	r0, sp, #0x8
    5fb2: f7ff feaa    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #-0x2ac
    5fb6: d4d4         	bmi	0x5f62 <core::fmt::Formatter::pad_integral::write_prefix::h7f7c9690b1602149+0x30> @ imm = #-0x58

00005fb8 <core::fmt::Formatter::pad::h2e2439709cd2f134>:
    5fb8: b5f0         	push	{r4, r5, r6, r7, lr}
    5fba: af03         	add	r7, sp, #0xc
    5fbc: e92d 0f00    	push.w	{r8, r9, r10, r11}
    5fc0: b087         	sub	sp, #0x1c
    5fc2: 468b         	mov	r11, r1
    5fc4: f8d0 c000    	ldr.w	r12, [r0]
    5fc8: 6881         	ldr	r1, [r0, #0x8]
    5fca: f1bc 0f00    	cmp.w	r12, #0x0
    5fce: bf08         	it	eq
    5fd0: ea5f 73c1    	lslseq.w	r3, r1, #0x1f
    5fd4: d00a         	beq	0x5fec <core::fmt::Formatter::pad::h2e2439709cd2f134+0x34> @ imm = #0x14
    5fd6: 07c9         	lsls	r1, r1, #0x1f
    5fd8: d037         	beq	0x604a <core::fmt::Formatter::pad::h2e2439709cd2f134+0x92> @ imm = #0x6e
    5fda: 68c6         	ldr	r6, [r0, #0xc]
    5fdc: eb0b 0e02    	add.w	lr, r11, r2
    5fe0: f04f 0800    	mov.w	r8, #0x0
    5fe4: b1de         	cbz	r6, 0x601e <core::fmt::Formatter::pad::h2e2439709cd2f134+0x66> @ imm = #0x36
    5fe6: 2500         	movs	r5, #0x0
    5fe8: 4659         	mov	r1, r11
    5fea: e00b         	b	0x6004 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x4c> @ imm = #0x16
    5fec: e9d0 0105    	ldrd	r0, r1, [r0, #20]
    5ff0: e324         	b	0x663c <core::fmt::Formatter::pad::h2e2439709cd2f134+0x684> @ imm = #0x648
    5ff2: 29f0         	cmp	r1, #0xf0
    5ff4: bf2c         	ite	hs
    5ff6: 1d21         	addhs	r1, r4, #0x4
    5ff8: 1ce1         	addlo	r1, r4, #0x3
    5ffa: 1b0b         	subs	r3, r1, r4
    5ffc: 3501         	adds	r5, #0x1
    5ffe: 4498         	add	r8, r3
    6000: 42ae         	cmp	r6, r5
    6002: d00d         	beq	0x6020 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x68> @ imm = #0x1a
    6004: 4571         	cmp	r1, lr
    6006: d020         	beq	0x604a <core::fmt::Formatter::pad::h2e2439709cd2f134+0x92> @ imm = #0x40
    6008: 460c         	mov	r4, r1
    600a: f911 3b01    	ldrsb	r3, [r1], #1
    600e: f1b3 3fff    	cmp.w	r3, #0xffffffff
    6012: dcf2         	bgt	0x5ffa <core::fmt::Formatter::pad::h2e2439709cd2f134+0x42> @ imm = #-0x1c
    6014: b2d9         	uxtb	r1, r3
    6016: 29e0         	cmp	r1, #0xe0
    6018: d2eb         	bhs	0x5ff2 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x3a> @ imm = #-0x2a
    601a: 1ca1         	adds	r1, r4, #0x2
    601c: e7ed         	b	0x5ffa <core::fmt::Formatter::pad::h2e2439709cd2f134+0x42> @ imm = #-0x26
    601e: 4659         	mov	r1, r11
    6020: 4571         	cmp	r1, lr
    6022: d012         	beq	0x604a <core::fmt::Formatter::pad::h2e2439709cd2f134+0x92> @ imm = #0x24
    6024: f991 1000    	ldrsb.w	r1, [r1]
    6028: f1b1 3fff    	cmp.w	r1, #0xffffffff
    602c: bfdc         	itt	le
    602e: b2c9         	uxtble	r1, r1
    6030: 29e0         	cmple	r1, #0xe0
    6032: f1b8 0f00    	cmp.w	r8, #0x0
    6036: d00e         	beq	0x6056 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x9e> @ imm = #0x1c
    6038: 4590         	cmp	r8, r2
    603a: d20b         	bhs	0x6054 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x9c> @ imm = #0x16
    603c: f91b 1008    	ldrsb.w	r1, [r11, r8]
    6040: f111 0f41    	cmn.w	r1, #0x41
    6044: dc07         	bgt	0x6056 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x9e> @ imm = #0xe
    6046: 2100         	movs	r1, #0x0
    6048: e006         	b	0x6058 <core::fmt::Formatter::pad::h2e2439709cd2f134+0xa0> @ imm = #0xc
    604a: 4690         	mov	r8, r2
    604c: f1bc 0f00    	cmp.w	r12, #0x0
    6050: d10a         	bne	0x6068 <core::fmt::Formatter::pad::h2e2439709cd2f134+0xb0> @ imm = #0x14
    6052: e2f0         	b	0x6636 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x67e> @ imm = #0x5e0
    6054: d1f7         	bne	0x6046 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x8e> @ imm = #-0x12
    6056: 4659         	mov	r1, r11
    6058: 2900         	cmp	r1, #0x0
    605a: bf0c         	ite	eq
    605c: 4690         	moveq	r8, r2
    605e: 468b         	movne	r11, r1
    6060: f1bc 0f00    	cmp.w	r12, #0x0
    6064: f000 82e7    	beq.w	0x6636 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x67e> @ imm = #0x5ce
    6068: 6844         	ldr	r4, [r0, #0x4]
    606a: f1b8 0f10    	cmp.w	r8, #0x10
    606e: d20c         	bhs	0x608a <core::fmt::Formatter::pad::h2e2439709cd2f134+0xd2> @ imm = #0x18
    6070: f1b8 0f00    	cmp.w	r8, #0x0
    6074: d023         	beq	0x60be <core::fmt::Formatter::pad::h2e2439709cd2f134+0x106> @ imm = #0x46
    6076: f008 0103    	and	r1, r8, #0x3
    607a: 2500         	movs	r5, #0x0
    607c: f1b8 0f04    	cmp.w	r8, #0x4
    6080: d222         	bhs	0x60c8 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x110> @ imm = #0x44
    6082: 2200         	movs	r2, #0x0
    6084: 2900         	cmp	r1, #0x0
    6086: d172         	bne	0x616e <core::fmt::Formatter::pad::h2e2439709cd2f134+0x1b6> @ imm = #0xe4
    6088: e291         	b	0x65ae <core::fmt::Formatter::pad::h2e2439709cd2f134+0x5f6> @ imm = #0x522
    608a: f10b 0103    	add.w	r1, r11, #0x3
    608e: 9401         	str	r4, [sp, #0x4]
    6090: f021 0903    	bic	r9, r1, #0x3
    6094: f8cd 800c    	str.w	r8, [sp, #0xc]
    6098: ebb9 010b    	subs.w	r1, r9, r11
    609c: 9002         	str	r0, [sp, #0x8]
    609e: eba8 0801    	sub.w	r8, r8, r1
    60a2: f008 0303    	and	r3, r8, #0x3
    60a6: d101         	bne	0x60ac <core::fmt::Formatter::pad::h2e2439709cd2f134+0xf4> @ imm = #0x2
    60a8: 2100         	movs	r1, #0x0
    60aa: e0ff         	b	0x62ac <core::fmt::Formatter::pad::h2e2439709cd2f134+0x2f4> @ imm = #0x1fe
    60ac: e9cd 1305    	strd	r1, r3, [sp, #20]
    60b0: ebab 0109    	sub.w	r1, r11, r9
    60b4: f111 0f04    	cmn.w	r1, #0x4
    60b8: d97a         	bls	0x61b0 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x1f8> @ imm = #0xf4
    60ba: 2100         	movs	r1, #0x0
    60bc: e0de         	b	0x627c <core::fmt::Formatter::pad::h2e2439709cd2f134+0x2c4> @ imm = #0x1bc
    60be: 2500         	movs	r5, #0x0
    60c0: 42ac         	cmp	r4, r5
    60c2: f200 8276    	bhi.w	0x65b2 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x5fa> @ imm = #0x4ec
    60c6: e2b6         	b	0x6636 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x67e> @ imm = #0x56c
    60c8: f99b 2001    	ldrsb.w	r2, [r11, #0x1]
    60cc: 4684         	mov	r12, r0
    60ce: f99b 6000    	ldrsb.w	r6, [r11]
    60d2: f112 0f41    	cmn.w	r2, #0x41
    60d6: f99b 3002    	ldrsb.w	r3, [r11, #0x2]
    60da: f99b 0003    	ldrsb.w	r0, [r11, #0x3]
    60de: bfc8         	it	gt
    60e0: 2501         	movgt	r5, #0x1
    60e2: f116 0f41    	cmn.w	r6, #0x41
    60e6: bfc8         	it	gt
    60e8: 3501         	addgt	r5, #0x1
    60ea: f113 0f41    	cmn.w	r3, #0x41
    60ee: bfc8         	it	gt
    60f0: 3501         	addgt	r5, #0x1
    60f2: f110 0f41    	cmn.w	r0, #0x41
    60f6: f008 020c    	and	r2, r8, #0xc
    60fa: bfc8         	it	gt
    60fc: 3501         	addgt	r5, #0x1
    60fe: 2a04         	cmp	r2, #0x4
    6100: d031         	beq	0x6166 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x1ae> @ imm = #0x62
    6102: f99b 6004    	ldrsb.w	r6, [r11, #0x4]
    6106: f99b 3005    	ldrsb.w	r3, [r11, #0x5]
    610a: f99b 0006    	ldrsb.w	r0, [r11, #0x6]
    610e: f116 0f41    	cmn.w	r6, #0x41
    6112: f99b e007    	ldrsb.w	lr, [r11, #0x7]
    6116: bfc8         	it	gt
    6118: 3501         	addgt	r5, #0x1
    611a: f113 0f41    	cmn.w	r3, #0x41
    611e: bfc8         	it	gt
    6120: 3501         	addgt	r5, #0x1
    6122: f110 0f41    	cmn.w	r0, #0x41
    6126: bfc8         	it	gt
    6128: 3501         	addgt	r5, #0x1
    612a: f11e 0f41    	cmn.w	lr, #0x41
    612e: bfc8         	it	gt
    6130: 3501         	addgt	r5, #0x1
    6132: 2a08         	cmp	r2, #0x8
    6134: d017         	beq	0x6166 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x1ae> @ imm = #0x2e
    6136: f99b 6008    	ldrsb.w	r6, [r11, #0x8]
    613a: f99b 3009    	ldrsb.w	r3, [r11, #0x9]
    613e: f99b 000a    	ldrsb.w	r0, [r11, #0xa]
    6142: f116 0f41    	cmn.w	r6, #0x41
    6146: f99b e00b    	ldrsb.w	lr, [r11, #0xb]
    614a: bfc8         	it	gt
    614c: 3501         	addgt	r5, #0x1
    614e: f113 0f41    	cmn.w	r3, #0x41
    6152: bfc8         	it	gt
    6154: 3501         	addgt	r5, #0x1
    6156: f110 0f41    	cmn.w	r0, #0x41
    615a: bfc8         	it	gt
    615c: 3501         	addgt	r5, #0x1
    615e: f11e 0f41    	cmn.w	lr, #0x41
    6162: bfc8         	it	gt
    6164: 3501         	addgt	r5, #0x1
    6166: 4660         	mov	r0, r12
    6168: 2900         	cmp	r1, #0x0
    616a: f000 8220    	beq.w	0x65ae <core::fmt::Formatter::pad::h2e2439709cd2f134+0x5f6> @ imm = #0x440
    616e: 4603         	mov	r3, r0
    6170: f91b 0002    	ldrsb.w	r0, [r11, r2]
    6174: f110 0f41    	cmn.w	r0, #0x41
    6178: bfc8         	it	gt
    617a: 3501         	addgt	r5, #0x1
    617c: 4618         	mov	r0, r3
    617e: 2901         	cmp	r1, #0x1
    6180: f000 8215    	beq.w	0x65ae <core::fmt::Formatter::pad::h2e2439709cd2f134+0x5f6> @ imm = #0x42a
    6184: 445a         	add	r2, r11
    6186: f992 0001    	ldrsb.w	r0, [r2, #0x1]
    618a: f110 0f41    	cmn.w	r0, #0x41
    618e: bfc8         	it	gt
    6190: 3501         	addgt	r5, #0x1
    6192: 4618         	mov	r0, r3
    6194: 2902         	cmp	r1, #0x2
    6196: f000 820a    	beq.w	0x65ae <core::fmt::Formatter::pad::h2e2439709cd2f134+0x5f6> @ imm = #0x414
    619a: f992 0002    	ldrsb.w	r0, [r2, #0x2]
    619e: f110 0f41    	cmn.w	r0, #0x41
    61a2: 4618         	mov	r0, r3
    61a4: bfc8         	it	gt
    61a6: 3501         	addgt	r5, #0x1
    61a8: 42ac         	cmp	r4, r5
    61aa: f200 8202    	bhi.w	0x65b2 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x5fa> @ imm = #0x404
    61ae: e242         	b	0x6636 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x67e> @ imm = #0x484
    61b0: 2100         	movs	r1, #0x0
    61b2: 2500         	movs	r5, #0x0
    61b4: eb0b 0205    	add.w	r2, r11, r5
    61b8: f91b c005    	ldrsb.w	r12, [r11, r5]
    61bc: f992 e001    	ldrsb.w	lr, [r2, #0x1]
    61c0: f992 a00e    	ldrsb.w	r10, [r2, #0xe]
    61c4: f11c 0f41    	cmn.w	r12, #0x41
    61c8: f992 300d    	ldrsb.w	r3, [r2, #0xd]
    61cc: f992 400c    	ldrsb.w	r4, [r2, #0xc]
    61d0: bfc8         	it	gt
    61d2: 3101         	addgt	r1, #0x1
    61d4: f992 6002    	ldrsb.w	r6, [r2, #0x2]
    61d8: f11e 0f41    	cmn.w	lr, #0x41
    61dc: bfc8         	it	gt
    61de: 3101         	addgt	r1, #0x1
    61e0: f992 c003    	ldrsb.w	r12, [r2, #0x3]
    61e4: f116 0f41    	cmn.w	r6, #0x41
    61e8: bfc8         	it	gt
    61ea: 3101         	addgt	r1, #0x1
    61ec: f992 6004    	ldrsb.w	r6, [r2, #0x4]
    61f0: f11c 0f41    	cmn.w	r12, #0x41
    61f4: bfc8         	it	gt
    61f6: 3101         	addgt	r1, #0x1
    61f8: f992 c005    	ldrsb.w	r12, [r2, #0x5]
    61fc: f116 0f41    	cmn.w	r6, #0x41
    6200: bfc8         	it	gt
    6202: 3101         	addgt	r1, #0x1
    6204: f992 6006    	ldrsb.w	r6, [r2, #0x6]
    6208: f11c 0f41    	cmn.w	r12, #0x41
    620c: bfc8         	it	gt
    620e: 3101         	addgt	r1, #0x1
    6210: f992 c007    	ldrsb.w	r12, [r2, #0x7]
    6214: f116 0f41    	cmn.w	r6, #0x41
    6218: bfc8         	it	gt
    621a: 3101         	addgt	r1, #0x1
    621c: f992 6008    	ldrsb.w	r6, [r2, #0x8]
    6220: f11c 0f41    	cmn.w	r12, #0x41
    6224: bfc8         	it	gt
    6226: 3101         	addgt	r1, #0x1
    6228: f992 c009    	ldrsb.w	r12, [r2, #0x9]
    622c: f116 0f41    	cmn.w	r6, #0x41
    6230: bfc8         	it	gt
    6232: 3101         	addgt	r1, #0x1
    6234: f992 600b    	ldrsb.w	r6, [r2, #0xb]
    6238: f11c 0f41    	cmn.w	r12, #0x41
    623c: bfc8         	it	gt
    623e: 3101         	addgt	r1, #0x1
    6240: f992 c00f    	ldrsb.w	r12, [r2, #0xf]
    6244: f992 200a    	ldrsb.w	r2, [r2, #0xa]
    6248: f112 0f41    	cmn.w	r2, #0x41
    624c: bfc8         	it	gt
    624e: 3101         	addgt	r1, #0x1
    6250: f116 0f41    	cmn.w	r6, #0x41
    6254: bfc8         	it	gt
    6256: 3101         	addgt	r1, #0x1
    6258: f114 0f41    	cmn.w	r4, #0x41
    625c: bfc8         	it	gt
    625e: 3101         	addgt	r1, #0x1
    6260: f113 0f41    	cmn.w	r3, #0x41
    6264: bfc8         	it	gt
    6266: 3101         	addgt	r1, #0x1
    6268: f11a 0f41    	cmn.w	r10, #0x41
    626c: bfc8         	it	gt
    626e: 3101         	addgt	r1, #0x1
    6270: f11c 0f41    	cmn.w	r12, #0x41
    6274: bfc8         	it	gt
    6276: 3101         	addgt	r1, #0x1
    6278: 3510         	adds	r5, #0x10
    627a: d19b         	bne	0x61b4 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x1fc> @ imm = #-0xca
    627c: f99b 2000    	ldrsb.w	r2, [r11]
    6280: f112 0f41    	cmn.w	r2, #0x41
    6284: bfc8         	it	gt
    6286: 3101         	addgt	r1, #0x1
    6288: 9805         	ldr	r0, [sp, #0x14]
    628a: 9b06         	ldr	r3, [sp, #0x18]
    628c: 2801         	cmp	r0, #0x1
    628e: d00d         	beq	0x62ac <core::fmt::Formatter::pad::h2e2439709cd2f134+0x2f4> @ imm = #0x1a
    6290: f99b 2001    	ldrsb.w	r2, [r11, #0x1]
    6294: f112 0f41    	cmn.w	r2, #0x41
    6298: bfc8         	it	gt
    629a: 3101         	addgt	r1, #0x1
    629c: 2802         	cmp	r0, #0x2
    629e: d005         	beq	0x62ac <core::fmt::Formatter::pad::h2e2439709cd2f134+0x2f4> @ imm = #0xa
    62a0: f99b 2002    	ldrsb.w	r2, [r11, #0x2]
    62a4: f112 0f41    	cmn.w	r2, #0x41
    62a8: bfc8         	it	gt
    62aa: 3101         	addgt	r1, #0x1
    62ac: ea4f 0a98    	lsr.w	r10, r8, #0x2
    62b0: 2200         	movs	r2, #0x0
    62b2: f8cd b010    	str.w	r11, [sp, #0x10]
    62b6: b1cb         	cbz	r3, 0x62ec <core::fmt::Formatter::pad::h2e2439709cd2f134+0x334> @ imm = #0x32
    62b8: f028 0603    	bic	r6, r8, #0x3
    62bc: eb09 0506    	add.w	r5, r9, r6
    62c0: f995 6000    	ldrsb.w	r6, [r5]
    62c4: f116 0f41    	cmn.w	r6, #0x41
    62c8: bfc8         	it	gt
    62ca: 2201         	movgt	r2, #0x1
    62cc: 2b01         	cmp	r3, #0x1
    62ce: d00d         	beq	0x62ec <core::fmt::Formatter::pad::h2e2439709cd2f134+0x334> @ imm = #0x1a
    62d0: f995 6001    	ldrsb.w	r6, [r5, #0x1]
    62d4: f116 0f41    	cmn.w	r6, #0x41
    62d8: bfc8         	it	gt
    62da: 3201         	addgt	r2, #0x1
    62dc: 2b02         	cmp	r3, #0x2
    62de: d005         	beq	0x62ec <core::fmt::Formatter::pad::h2e2439709cd2f134+0x334> @ imm = #0xa
    62e0: f995 6002    	ldrsb.w	r6, [r5, #0x2]
    62e4: f116 0f41    	cmn.w	r6, #0x41
    62e8: bfc8         	it	gt
    62ea: 3201         	addgt	r2, #0x1
    62ec: 1855         	adds	r5, r2, r1
    62ee: e012         	b	0x6316 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x35e> @ imm = #0x24
    62f0: f04f 0c00    	mov.w	r12, #0x0
    62f4: fa3f f08c    	uxtb16	r0, r12
    62f8: fa3f f29c    	uxtb16	r2, r12, ror #8
    62fc: 4410         	add	r0, r2
    62fe: ebaa 0a0b    	sub.w	r10, r10, r11
    6302: eb08 098b    	add.w	r9, r8, r11, lsl #2
    6306: f01b 0103    	ands	r1, r11, #0x3
    630a: eb00 4000    	add.w	r0, r0, r0, lsl #16
    630e: eb05 4510    	add.w	r5, r5, r0, lsr #16
    6312: f040 815c    	bne.w	0x65ce <core::fmt::Formatter::pad::h2e2439709cd2f134+0x616> @ imm = #0x2b8
    6316: f1ba 0f00    	cmp.w	r10, #0x0
    631a: f000 8144    	beq.w	0x65a6 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x5ee> @ imm = #0x288
    631e: f1ba 0fc0    	cmp.w	r10, #0xc0
    6322: 46d3         	mov	r11, r10
    6324: bf28         	it	hs
    6326: f04f 0bc0    	movhs.w	r11, #0xc0
    632a: 46c8         	mov	r8, r9
    632c: f1ba 0f04    	cmp.w	r10, #0x4
    6330: d3de         	blo	0x62f0 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x338> @ imm = #-0x44
    6332: f06f 000f    	mvn	r0, #0xf
    6336: eb00 018b    	add.w	r1, r0, r11, lsl #2
    633a: 2001         	movs	r0, #0x1
    633c: 2930         	cmp	r1, #0x30
    633e: eb00 1011    	add.w	r0, r0, r1, lsr #4
    6342: 9506         	str	r5, [sp, #0x18]
    6344: 9005         	str	r0, [sp, #0x14]
    6346: d203         	bhs	0x6350 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x398> @ imm = #0x6
    6348: f04f 0c00    	mov.w	r12, #0x0
    634c: 4646         	mov	r6, r8
    634e: e0aa         	b	0x64a6 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x4ee> @ imm = #0x154
    6350: f020 0e03    	bic	lr, r0, #0x3
    6354: f04f 0c00    	mov.w	r12, #0x0
    6358: 4646         	mov	r6, r8
    635a: e896 0216    	ldm.w	r6, {r1, r2, r4, r9}
    635e: f1be 0e04    	subs.w	lr, lr, #0x4
    6362: 69b5         	ldr	r5, [r6, #0x18]
    6364: ea6f 0001    	mvn.w	r0, r1
    6368: ea4f 10d0    	lsr.w	r0, r0, #0x7
    636c: ea40 1091    	orr.w	r0, r0, r1, lsr #6
    6370: ea6f 0102    	mvn.w	r1, r2
    6374: ea4f 11d1    	lsr.w	r1, r1, #0x7
    6378: f020 30fe    	bic	r0, r0, #0xfefefefe
    637c: ea41 1192    	orr.w	r1, r1, r2, lsr #6
    6380: 4460         	add	r0, r12
    6382: f021 31fe    	bic	r1, r1, #0xfefefefe
    6386: 6972         	ldr	r2, [r6, #0x14]
    6388: 4408         	add	r0, r1
    638a: ea6f 0104    	mvn.w	r1, r4
    638e: ea4f 13d1    	lsr.w	r3, r1, #0x7
    6392: 6bb1         	ldr	r1, [r6, #0x38]
    6394: ea43 1394    	orr.w	r3, r3, r4, lsr #6
    6398: 6934         	ldr	r4, [r6, #0x10]
    639a: f023 33fe    	bic	r3, r3, #0xfefefefe
    639e: f8d6 c03c    	ldr.w	r12, [r6, #0x3c]
    63a2: 4418         	add	r0, r3
    63a4: ea6f 0309    	mvn.w	r3, r9
    63a8: ea4f 13d3    	lsr.w	r3, r3, #0x7
    63ac: ea43 1399    	orr.w	r3, r3, r9, lsr #6
    63b0: f023 33fe    	bic	r3, r3, #0xfefefefe
    63b4: 4418         	add	r0, r3
    63b6: ea6f 0304    	mvn.w	r3, r4
    63ba: ea4f 13d3    	lsr.w	r3, r3, #0x7
    63be: ea43 1394    	orr.w	r3, r3, r4, lsr #6
    63c2: 69f4         	ldr	r4, [r6, #0x1c]
    63c4: f023 33fe    	bic	r3, r3, #0xfefefefe
    63c8: 4418         	add	r0, r3
    63ca: ea6f 0302    	mvn.w	r3, r2
    63ce: ea4f 13d3    	lsr.w	r3, r3, #0x7
    63d2: ea43 1292    	orr.w	r2, r3, r2, lsr #6
    63d6: f022 32fe    	bic	r2, r2, #0xfefefefe
    63da: 6a33         	ldr	r3, [r6, #0x20]
    63dc: 4410         	add	r0, r2
    63de: ea6f 0205    	mvn.w	r2, r5
    63e2: ea4f 12d2    	lsr.w	r2, r2, #0x7
    63e6: ea42 1295    	orr.w	r2, r2, r5, lsr #6
    63ea: 6a75         	ldr	r5, [r6, #0x24]
    63ec: f022 32fe    	bic	r2, r2, #0xfefefefe
    63f0: 4410         	add	r0, r2
    63f2: ea6f 0204    	mvn.w	r2, r4
    63f6: ea4f 12d2    	lsr.w	r2, r2, #0x7
    63fa: ea42 1294    	orr.w	r2, r2, r4, lsr #6
    63fe: 6ab4         	ldr	r4, [r6, #0x28]
    6400: f022 32fe    	bic	r2, r2, #0xfefefefe
    6404: 4410         	add	r0, r2
    6406: ea6f 0203    	mvn.w	r2, r3
    640a: ea4f 12d2    	lsr.w	r2, r2, #0x7
    640e: ea42 1293    	orr.w	r2, r2, r3, lsr #6
    6412: 6af3         	ldr	r3, [r6, #0x2c]
    6414: f022 32fe    	bic	r2, r2, #0xfefefefe
    6418: 4410         	add	r0, r2
    641a: ea6f 0205    	mvn.w	r2, r5
    641e: ea4f 12d2    	lsr.w	r2, r2, #0x7
    6422: ea42 1295    	orr.w	r2, r2, r5, lsr #6
    6426: 6b35         	ldr	r5, [r6, #0x30]
    6428: f022 32fe    	bic	r2, r2, #0xfefefefe
    642c: 4410         	add	r0, r2
    642e: ea6f 0204    	mvn.w	r2, r4
    6432: ea4f 12d2    	lsr.w	r2, r2, #0x7
    6436: ea42 1294    	orr.w	r2, r2, r4, lsr #6
    643a: 6b74         	ldr	r4, [r6, #0x34]
    643c: f022 32fe    	bic	r2, r2, #0xfefefefe
    6440: f106 0640    	add.w	r6, r6, #0x40
    6444: 4410         	add	r0, r2
    6446: ea6f 0203    	mvn.w	r2, r3
    644a: ea4f 12d2    	lsr.w	r2, r2, #0x7
    644e: ea42 1293    	orr.w	r2, r2, r3, lsr #6
    6452: f022 32fe    	bic	r2, r2, #0xfefefefe
    6456: 4410         	add	r0, r2
    6458: ea6f 0205    	mvn.w	r2, r5
    645c: ea4f 12d2    	lsr.w	r2, r2, #0x7
    6460: ea42 1295    	orr.w	r2, r2, r5, lsr #6
    6464: f022 32fe    	bic	r2, r2, #0xfefefefe
    6468: 4410         	add	r0, r2
    646a: ea6f 0204    	mvn.w	r2, r4
    646e: ea4f 12d2    	lsr.w	r2, r2, #0x7
    6472: ea42 1294    	orr.w	r2, r2, r4, lsr #6
    6476: f022 32fe    	bic	r2, r2, #0xfefefefe
    647a: 4410         	add	r0, r2
    647c: ea6f 0201    	mvn.w	r2, r1
    6480: ea4f 12d2    	lsr.w	r2, r2, #0x7
    6484: ea42 1191    	orr.w	r1, r2, r1, lsr #6
    6488: f021 31fe    	bic	r1, r1, #0xfefefefe
    648c: 4408         	add	r0, r1
    648e: ea6f 010c    	mvn.w	r1, r12
    6492: ea4f 11d1    	lsr.w	r1, r1, #0x7
    6496: ea41 119c    	orr.w	r1, r1, r12, lsr #6
    649a: f021 31fe    	bic	r1, r1, #0xfefefefe
    649e: eb01 0c00    	add.w	r12, r1, r0
    64a2: f47f af5a    	bne.w	0x635a <core::fmt::Formatter::pad::h2e2439709cd2f134+0x3a2> @ imm = #-0x14c
    64a6: 9805         	ldr	r0, [sp, #0x14]
    64a8: 9d06         	ldr	r5, [sp, #0x18]
    64aa: f010 0e03    	ands	lr, r0, #0x3
    64ae: f43f af21    	beq.w	0x62f4 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x33c> @ imm = #-0x1be
    64b2: e896 000d    	ldm.w	r6, {r0, r2, r3}
    64b6: f1be 0f01    	cmp.w	lr, #0x1
    64ba: 68f1         	ldr	r1, [r6, #0xc]
    64bc: ea6f 0400    	mvn.w	r4, r0
    64c0: ea4f 14d4    	lsr.w	r4, r4, #0x7
    64c4: ea44 1090    	orr.w	r0, r4, r0, lsr #6
    64c8: ea6f 0402    	mvn.w	r4, r2
    64cc: f020 30fe    	bic	r0, r0, #0xfefefefe
    64d0: ea4f 14d4    	lsr.w	r4, r4, #0x7
    64d4: ea44 1292    	orr.w	r2, r4, r2, lsr #6
    64d8: 4460         	add	r0, r12
    64da: f022 32fe    	bic	r2, r2, #0xfefefefe
    64de: 4410         	add	r0, r2
    64e0: ea6f 0203    	mvn.w	r2, r3
    64e4: ea4f 12d2    	lsr.w	r2, r2, #0x7
    64e8: ea42 1293    	orr.w	r2, r2, r3, lsr #6
    64ec: f022 32fe    	bic	r2, r2, #0xfefefefe
    64f0: 4410         	add	r0, r2
    64f2: ea6f 0201    	mvn.w	r2, r1
    64f6: ea4f 12d2    	lsr.w	r2, r2, #0x7
    64fa: ea42 1291    	orr.w	r2, r2, r1, lsr #6
    64fe: f022 32fe    	bic	r2, r2, #0xfefefefe
    6502: eb02 0c00    	add.w	r12, r2, r0
    6506: f43f aef5    	beq.w	0x62f4 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x33c> @ imm = #-0x216
    650a: f106 0310    	add.w	r3, r6, #0x10
    650e: 69f1         	ldr	r1, [r6, #0x1c]
    6510: f1be 0f02    	cmp.w	lr, #0x2
    6514: cb0d         	ldm	r3, {r0, r2, r3}
    6516: ea6f 0400    	mvn.w	r4, r0
    651a: ea4f 14d4    	lsr.w	r4, r4, #0x7
    651e: ea44 1090    	orr.w	r0, r4, r0, lsr #6
    6522: ea6f 0402    	mvn.w	r4, r2
    6526: f020 30fe    	bic	r0, r0, #0xfefefefe
    652a: ea4f 14d4    	lsr.w	r4, r4, #0x7
    652e: ea44 1292    	orr.w	r2, r4, r2, lsr #6
    6532: 4460         	add	r0, r12
    6534: f022 32fe    	bic	r2, r2, #0xfefefefe
    6538: 4410         	add	r0, r2
    653a: ea6f 0203    	mvn.w	r2, r3
    653e: ea4f 12d2    	lsr.w	r2, r2, #0x7
    6542: ea42 1293    	orr.w	r2, r2, r3, lsr #6
    6546: f022 32fe    	bic	r2, r2, #0xfefefefe
    654a: 4410         	add	r0, r2
    654c: ea6f 0201    	mvn.w	r2, r1
    6550: ea4f 12d2    	lsr.w	r2, r2, #0x7
    6554: ea42 1291    	orr.w	r2, r2, r1, lsr #6
    6558: f022 32fe    	bic	r2, r2, #0xfefefefe
    655c: eb02 0c00    	add.w	r12, r2, r0
    6560: f43f aec8    	beq.w	0x62f4 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x33c> @ imm = #-0x270
    6564: f106 0320    	add.w	r3, r6, #0x20
    6568: cb0f         	ldm	r3, {r0, r1, r2, r3}
    656a: 43c6         	mvns	r6, r0
    656c: 09f6         	lsrs	r6, r6, #0x7
    656e: ea46 1090    	orr.w	r0, r6, r0, lsr #6
    6572: 43ce         	mvns	r6, r1
    6574: f020 30fe    	bic	r0, r0, #0xfefefefe
    6578: 09f6         	lsrs	r6, r6, #0x7
    657a: ea46 1191    	orr.w	r1, r6, r1, lsr #6
    657e: 4460         	add	r0, r12
    6580: f021 31fe    	bic	r1, r1, #0xfefefefe
    6584: 4408         	add	r0, r1
    6586: 43d1         	mvns	r1, r2
    6588: 09c9         	lsrs	r1, r1, #0x7
    658a: ea41 1192    	orr.w	r1, r1, r2, lsr #6
    658e: f021 31fe    	bic	r1, r1, #0xfefefefe
    6592: 4408         	add	r0, r1
    6594: 43d9         	mvns	r1, r3
    6596: 09c9         	lsrs	r1, r1, #0x7
    6598: ea41 1193    	orr.w	r1, r1, r3, lsr #6
    659c: f021 31fe    	bic	r1, r1, #0xfefefefe
    65a0: eb01 0c00    	add.w	r12, r1, r0
    65a4: e6a6         	b	0x62f4 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x33c> @ imm = #-0x2b4
    65a6: e9dd 8b03    	ldrd	r8, r11, [sp, #12]
    65aa: e9dd 4001    	ldrd	r4, r0, [sp, #4]
    65ae: 42ac         	cmp	r4, r5
    65b0: d941         	bls	0x6636 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x67e> @ imm = #0x82
    65b2: f890 2020    	ldrb.w	r2, [r0, #0x20]
    65b6: 4603         	mov	r3, r0
    65b8: eba4 0905    	sub.w	r9, r4, r5
    65bc: 2100         	movs	r1, #0x0
    65be: e8df f002    	tbb	[pc, r2]
    65c2: 4b 02 45 4b  	.word	0x4b45024b
    65c6: 4649         	mov	r1, r9
    65c8: f04f 0900    	mov.w	r9, #0x0
    65cc: e044         	b	0x6658 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x6a0> @ imm = #0x88
    65ce: f00b 03fc    	and	r3, r11, #0xfc
    65d2: e9dd 4c01    	ldrd	r4, r12, [sp, #4]
    65d6: 2901         	cmp	r1, #0x1
    65d8: f858 0023    	ldr.w	r0, [r8, r3, lsl #2]
    65dc: ea6f 0200    	mvn.w	r2, r0
    65e0: ea4f 12d2    	lsr.w	r2, r2, #0x7
    65e4: ea42 1090    	orr.w	r0, r2, r0, lsr #6
    65e8: f020 32fe    	bic	r2, r0, #0xfefefefe
    65ec: d015         	beq	0x661a <core::fmt::Formatter::pad::h2e2439709cd2f134+0x662> @ imm = #0x2a
    65ee: eb08 0383    	add.w	r3, r8, r3, lsl #2
    65f2: 2902         	cmp	r1, #0x2
    65f4: 6858         	ldr	r0, [r3, #0x4]
    65f6: ea6f 0600    	mvn.w	r6, r0
    65fa: ea4f 16d6    	lsr.w	r6, r6, #0x7
    65fe: ea46 1090    	orr.w	r0, r6, r0, lsr #6
    6602: f020 30fe    	bic	r0, r0, #0xfefefefe
    6606: 4402         	add	r2, r0
    6608: d007         	beq	0x661a <core::fmt::Formatter::pad::h2e2439709cd2f134+0x662> @ imm = #0xe
    660a: 6898         	ldr	r0, [r3, #0x8]
    660c: 43c1         	mvns	r1, r0
    660e: 09c9         	lsrs	r1, r1, #0x7
    6610: ea41 1090    	orr.w	r0, r1, r0, lsr #6
    6614: f020 30fe    	bic	r0, r0, #0xfefefefe
    6618: 4402         	add	r2, r0
    661a: fa3f f082    	uxtb16	r0, r2
    661e: fa3f f192    	uxtb16	r1, r2, ror #8
    6622: 4408         	add	r0, r1
    6624: e9dd 8b03    	ldrd	r8, r11, [sp, #12]
    6628: eb00 4000    	add.w	r0, r0, r0, lsl #16
    662c: eb05 4510    	add.w	r5, r5, r0, lsr #16
    6630: 4660         	mov	r0, r12
    6632: 42ac         	cmp	r4, r5
    6634: d8bd         	bhi	0x65b2 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x5fa> @ imm = #-0x86
    6636: e9d0 0105    	ldrd	r0, r1, [r0, #20]
    663a: 4642         	mov	r2, r8
    663c: 68cb         	ldr	r3, [r1, #0xc]
    663e: 4659         	mov	r1, r11
    6640: b007         	add	sp, #0x1c
    6642: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    6646: e8bd 40f0    	pop.w	{r4, r5, r6, r7, lr}
    664a: 4718         	bx	r3
    664c: f109 0001    	add.w	r0, r9, #0x1
    6650: ea4f 0159    	lsr.w	r1, r9, #0x1
    6654: ea4f 0950    	lsr.w	r9, r0, #0x1
    6658: e9d3 a404    	ldrd	r10, r4, [r3, #16]
    665c: 1c4e         	adds	r6, r1, #0x1
    665e: 699d         	ldr	r5, [r3, #0x18]
    6660: 3e01         	subs	r6, #0x1
    6662: d006         	beq	0x6672 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x6ba> @ imm = #0xc
    6664: 692a         	ldr	r2, [r5, #0x10]
    6666: 4620         	mov	r0, r4
    6668: 4651         	mov	r1, r10
    666a: 4790         	blx	r2
    666c: 2800         	cmp	r0, #0x0
    666e: d0f7         	beq	0x6660 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x6a8> @ imm = #-0x12
    6670: e005         	b	0x667e <core::fmt::Formatter::pad::h2e2439709cd2f134+0x6c6> @ imm = #0xa
    6672: 68eb         	ldr	r3, [r5, #0xc]
    6674: 4620         	mov	r0, r4
    6676: 4659         	mov	r1, r11
    6678: 4642         	mov	r2, r8
    667a: 4798         	blx	r3
    667c: b120         	cbz	r0, 0x6688 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x6d0> @ imm = #0x8
    667e: 2001         	movs	r0, #0x1
    6680: b007         	add	sp, #0x1c
    6682: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    6686: bdf0         	pop	{r4, r5, r6, r7, pc}
    6688: 2600         	movs	r6, #0x0
    668a: 45b1         	cmp	r9, r6
    668c: d008         	beq	0x66a0 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x6e8> @ imm = #0x10
    668e: 692a         	ldr	r2, [r5, #0x10]
    6690: 4620         	mov	r0, r4
    6692: 4651         	mov	r1, r10
    6694: 4790         	blx	r2
    6696: 3601         	adds	r6, #0x1
    6698: 2800         	cmp	r0, #0x0
    669a: d0f6         	beq	0x668a <core::fmt::Formatter::pad::h2e2439709cd2f134+0x6d2> @ imm = #-0x14
    669c: 1e71         	subs	r1, r6, #0x1
    669e: e000         	b	0x66a2 <core::fmt::Formatter::pad::h2e2439709cd2f134+0x6ea> @ imm = #0x0
    66a0: 4649         	mov	r1, r9
    66a2: 2000         	movs	r0, #0x0
    66a4: 4549         	cmp	r1, r9
    66a6: bf38         	it	lo
    66a8: 2001         	movlo	r0, #0x1
    66aa: b007         	add	sp, #0x1c
    66ac: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    66b0: bdf0         	pop	{r4, r5, r6, r7, pc}

000066b2 <core::panicking::panic::h26f8d9ea1f810f6d>:
    66b2: b580         	push	{r7, lr}
    66b4: 466f         	mov	r7, sp
    66b6: b088         	sub	sp, #0x20
    66b8: 4694         	mov	r12, r2
    66ba: 2201         	movs	r2, #0x1
    66bc: 9201         	str	r2, [sp, #0x4]
    66be: aa06         	add	r2, sp, #0x18
    66c0: 2300         	movs	r3, #0x0
    66c2: e9cd 0106    	strd	r0, r1, [sp, #24]
    66c6: 9200         	str	r2, [sp]
    66c8: 2204         	movs	r2, #0x4
    66ca: 4668         	mov	r0, sp
    66cc: 4661         	mov	r1, r12
    66ce: 9304         	str	r3, [sp, #0x10]
    66d0: 9303         	str	r3, [sp, #0xc]
    66d2: 9202         	str	r2, [sp, #0x8]
    66d4: f7ff fb19    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #-0x9ce

000066d8 <<&T as core::fmt::Display>::fmt::h4a02c691c83f1cae>:
    66d8: 460b         	mov	r3, r1
    66da: e9d0 1200    	ldrd	r1, r2, [r0]
    66de: 4618         	mov	r0, r3
    66e0: f7ff bc6a    	b.w	0x5fb8 <core::fmt::Formatter::pad::h2e2439709cd2f134> @ imm = #-0x72c

000066e4 <<&T as core::fmt::Debug>::fmt::h5010d033ceaf782b>:
    66e4: e9d0 0200    	ldrd	r0, r2, [r0]
    66e8: 68d2         	ldr	r2, [r2, #0xc]
    66ea: 4710         	bx	r2

000066ec <core::fmt::write::h5c77e528eed792ce>:
    66ec: b5f0         	push	{r4, r5, r6, r7, lr}
    66ee: af03         	add	r7, sp, #0xc
    66f0: e92d 0f00    	push.w	{r8, r9, r10, r11}
    66f4: b08d         	sub	sp, #0x34
    66f6: 2303         	movs	r3, #0x3
    66f8: 6916         	ldr	r6, [r2, #0x10]
    66fa: f88d 3030    	strb.w	r3, [sp, #0x30]
    66fe: 2320         	movs	r3, #0x20
    6700: 9308         	str	r3, [sp, #0x20]
    6702: 2300         	movs	r3, #0x0
    6704: 2e00         	cmp	r6, #0x0
    6706: e9cd 130a    	strd	r1, r3, [sp, #40]
    670a: 9009         	str	r0, [sp, #0x24]
    670c: 9306         	str	r3, [sp, #0x18]
    670e: 9304         	str	r3, [sp, #0x10]
    6710: d067         	beq	0x67e2 <core::fmt::write::h5c77e528eed792ce+0xf6> @ imm = #0xce
    6712: 6950         	ldr	r0, [r2, #0x14]
    6714: 2800         	cmp	r0, #0x0
    6716: f000 808d    	beq.w	0x6834 <core::fmt::write::h5c77e528eed792ce+0x148> @ imm = #0x11a
    671a: f8d2 a000    	ldr.w	r10, [r2]
    671e: 0141         	lsls	r1, r0, #0x5
    6720: f8d2 b008    	ldr.w	r11, [r2, #0x8]
    6724: 3801         	subs	r0, #0x1
    6726: f020 4078    	bic	r0, r0, #0xf8000000
    672a: f04f 0800    	mov.w	r8, #0x0
    672e: f04f 0900    	mov.w	r9, #0x0
    6732: 3001         	adds	r0, #0x1
    6734: 9202         	str	r2, [sp, #0x8]
    6736: 9103         	str	r1, [sp, #0xc]
    6738: 9001         	str	r0, [sp, #0x4]
    673a: eb0a 0109    	add.w	r1, r10, r9
    673e: 684a         	ldr	r2, [r1, #0x4]
    6740: b13a         	cbz	r2, 0x6752 <core::fmt::write::h5c77e528eed792ce+0x66> @ imm = #0xe
    6742: e9dd 0309    	ldrd	r0, r3, [sp, #36]
    6746: 6809         	ldr	r1, [r1]
    6748: 68db         	ldr	r3, [r3, #0xc]
    674a: 4798         	blx	r3
    674c: 2800         	cmp	r0, #0x0
    674e: f040 8081    	bne.w	0x6854 <core::fmt::write::h5c77e528eed792ce+0x168> @ imm = #0x102
    6752: eb06 0008    	add.w	r0, r6, r8
    6756: 6903         	ldr	r3, [r0, #0x10]
    6758: eb06 0089    	add.w	r0, r6, r9, lsl #2
    675c: e9d0 2102    	ldrd	r2, r1, [r0, #8]
    6760: 7f05         	ldrb	r5, [r0, #0x1c]
    6762: 6984         	ldr	r4, [r0, #0x18]
    6764: 9308         	str	r3, [sp, #0x20]
    6766: f88d 5030    	strb.w	r5, [sp, #0x30]
    676a: 940b         	str	r4, [sp, #0x2c]
    676c: b172         	cbz	r2, 0x678c <core::fmt::write::h5c77e528eed792ce+0xa0> @ imm = #0x1c
    676e: 2a01         	cmp	r2, #0x1
    6770: d103         	bne	0x677a <core::fmt::write::h5c77e528eed792ce+0x8e> @ imm = #0x6
    6772: eb0b 01c1    	add.w	r1, r11, r1, lsl #3
    6776: 684a         	ldr	r2, [r1, #0x4]
    6778: b13a         	cbz	r2, 0x678a <core::fmt::write::h5c77e528eed792ce+0x9e> @ imm = #0xe
    677a: 2200         	movs	r2, #0x0
    677c: f856 3008    	ldr.w	r3, [r6, r8]
    6780: e9cd 2104    	strd	r2, r1, [sp, #16]
    6784: 2b02         	cmp	r3, #0x2
    6786: d108         	bne	0x679a <core::fmt::write::h5c77e528eed792ce+0xae> @ imm = #0x10
    6788: e00e         	b	0x67a8 <core::fmt::write::h5c77e528eed792ce+0xbc> @ imm = #0x1c
    678a: 6809         	ldr	r1, [r1]
    678c: 2201         	movs	r2, #0x1
    678e: f856 3008    	ldr.w	r3, [r6, r8]
    6792: e9cd 2104    	strd	r2, r1, [sp, #16]
    6796: 2b02         	cmp	r3, #0x2
    6798: d006         	beq	0x67a8 <core::fmt::write::h5c77e528eed792ce+0xbc> @ imm = #0xc
    679a: 6841         	ldr	r1, [r0, #0x4]
    679c: 2b01         	cmp	r3, #0x1
    679e: d106         	bne	0x67ae <core::fmt::write::h5c77e528eed792ce+0xc2> @ imm = #0xc
    67a0: eb0b 01c1    	add.w	r1, r11, r1, lsl #3
    67a4: 684a         	ldr	r2, [r1, #0x4]
    67a6: b10a         	cbz	r2, 0x67ac <core::fmt::write::h5c77e528eed792ce+0xc0> @ imm = #0x2
    67a8: 2200         	movs	r2, #0x0
    67aa: e001         	b	0x67b0 <core::fmt::write::h5c77e528eed792ce+0xc4> @ imm = #0x2
    67ac: 6809         	ldr	r1, [r1]
    67ae: 2201         	movs	r2, #0x1
    67b0: 6943         	ldr	r3, [r0, #0x14]
    67b2: 9107         	str	r1, [sp, #0x1c]
    67b4: 9206         	str	r2, [sp, #0x18]
    67b6: eb0b 01c3    	add.w	r1, r11, r3, lsl #3
    67ba: f85b 0033    	ldr.w	r0, [r11, r3, lsl #3]
    67be: 684b         	ldr	r3, [r1, #0x4]
    67c0: a904         	add	r1, sp, #0x10
    67c2: 4798         	blx	r3
    67c4: 2800         	cmp	r0, #0x0
    67c6: d145         	bne	0x6854 <core::fmt::write::h5c77e528eed792ce+0x168> @ imm = #0x8a
    67c8: 9803         	ldr	r0, [sp, #0xc]
    67ca: f108 0820    	add.w	r8, r8, #0x20
    67ce: f109 0908    	add.w	r9, r9, #0x8
    67d2: 4540         	cmp	r0, r8
    67d4: d1b1         	bne	0x673a <core::fmt::write::h5c77e528eed792ce+0x4e> @ imm = #-0x9e
    67d6: e9dd b201    	ldrd	r11, r2, [sp, #4]
    67da: 6850         	ldr	r0, [r2, #0x4]
    67dc: 4583         	cmp	r11, r0
    67de: d32e         	blo	0x683e <core::fmt::write::h5c77e528eed792ce+0x152> @ imm = #0x5c
    67e0: e03d         	b	0x685e <core::fmt::write::h5c77e528eed792ce+0x172> @ imm = #0x7a
    67e2: 68d0         	ldr	r0, [r2, #0xc]
    67e4: b330         	cbz	r0, 0x6834 <core::fmt::write::h5c77e528eed792ce+0x148> @ imm = #0x4c
    67e6: 00c6         	lsls	r6, r0, #0x3
    67e8: 3801         	subs	r0, #0x1
    67ea: f020 4060    	bic	r0, r0, #0xe0000000
    67ee: 6894         	ldr	r4, [r2, #0x8]
    67f0: f8d2 9000    	ldr.w	r9, [r2]
    67f4: f100 0b01    	add.w	r11, r0, #0x1
    67f8: f10d 0810    	add.w	r8, sp, #0x10
    67fc: 4692         	mov	r10, r2
    67fe: 2500         	movs	r5, #0x0
    6800: eb09 01c5    	add.w	r1, r9, r5, lsl #3
    6804: 684a         	ldr	r2, [r1, #0x4]
    6806: b12a         	cbz	r2, 0x6814 <core::fmt::write::h5c77e528eed792ce+0x128> @ imm = #0xa
    6808: e9dd 0309    	ldrd	r0, r3, [sp, #36]
    680c: 6809         	ldr	r1, [r1]
    680e: 68db         	ldr	r3, [r3, #0xc]
    6810: 4798         	blx	r3
    6812: b9f8         	cbnz	r0, 0x6854 <core::fmt::write::h5c77e528eed792ce+0x168> @ imm = #0x3e
    6814: eb04 01c5    	add.w	r1, r4, r5, lsl #3
    6818: f854 0035    	ldr.w	r0, [r4, r5, lsl #3]
    681c: 684a         	ldr	r2, [r1, #0x4]
    681e: 4641         	mov	r1, r8
    6820: 4790         	blx	r2
    6822: b9b8         	cbnz	r0, 0x6854 <core::fmt::write::h5c77e528eed792ce+0x168> @ imm = #0x2e
    6824: 3501         	adds	r5, #0x1
    6826: 3e08         	subs	r6, #0x8
    6828: d1ea         	bne	0x6800 <core::fmt::write::h5c77e528eed792ce+0x114> @ imm = #-0x2c
    682a: 4652         	mov	r2, r10
    682c: 6850         	ldr	r0, [r2, #0x4]
    682e: 4583         	cmp	r11, r0
    6830: d305         	blo	0x683e <core::fmt::write::h5c77e528eed792ce+0x152> @ imm = #0xa
    6832: e014         	b	0x685e <core::fmt::write::h5c77e528eed792ce+0x172> @ imm = #0x28
    6834: f04f 0b00    	mov.w	r11, #0x0
    6838: 6850         	ldr	r0, [r2, #0x4]
    683a: 4583         	cmp	r11, r0
    683c: d20f         	bhs	0x685e <core::fmt::write::h5c77e528eed792ce+0x172> @ imm = #0x1e
    683e: 6812         	ldr	r2, [r2]
    6840: e9dd 0109    	ldrd	r0, r1, [sp, #36]
    6844: 68cb         	ldr	r3, [r1, #0xc]
    6846: f852 103b    	ldr.w	r1, [r2, r11, lsl #3]
    684a: eb02 02cb    	add.w	r2, r2, r11, lsl #3
    684e: 6852         	ldr	r2, [r2, #0x4]
    6850: 4798         	blx	r3
    6852: b120         	cbz	r0, 0x685e <core::fmt::write::h5c77e528eed792ce+0x172> @ imm = #0x8
    6854: 2001         	movs	r0, #0x1
    6856: b00d         	add	sp, #0x34
    6858: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    685c: bdf0         	pop	{r4, r5, r6, r7, pc}
    685e: 2000         	movs	r0, #0x0
    6860: b00d         	add	sp, #0x34
    6862: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    6866: bdf0         	pop	{r4, r5, r6, r7, pc}

00006868 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714>:
    6868: b5f0         	push	{r4, r5, r6, r7, lr}
    686a: af03         	add	r7, sp, #0xc
    686c: e92d 0f00    	push.w	{r8, r9, r10, r11}
    6870: b08f         	sub	sp, #0x3c
    6872: 4698         	mov	r8, r3
    6874: 4603         	mov	r3, r0
    6876: 7900         	ldrb	r0, [r0, #0x4]
    6878: f04f 0901    	mov.w	r9, #0x1
    687c: 2800         	cmp	r0, #0x0
    687e: f04f 0001    	mov.w	r0, #0x1
    6882: d17b         	bne	0x697c <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x114> @ imm = #0xf6
    6884: 681d         	ldr	r5, [r3]
    6886: 469a         	mov	r10, r3
    6888: f8cd 8004    	str.w	r8, [sp, #0x4]
    688c: f8d7 8008    	ldr.w	r8, [r7, #0x8]
    6890: 69e8         	ldr	r0, [r5, #0x1c]
    6892: 795b         	ldrb	r3, [r3, #0x5]
    6894: 0746         	lsls	r6, r0, #0x1d
    6896: d42d         	bmi	0x68f4 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x8c> @ imm = #0x5a
    6898: 460c         	mov	r4, r1
    689a: f647 01b0    	movw	r1, #0x78b0
    689e: 4693         	mov	r11, r2
    68a0: f647 02b3    	movw	r2, #0x78b3
    68a4: e9d5 0605    	ldrd	r0, r6, [r5, #20]
    68a8: f2c0 0100    	movt	r1, #0x0
    68ac: f2c0 0200    	movt	r2, #0x0
    68b0: 2b00         	cmp	r3, #0x0
    68b2: bf18         	it	ne
    68b4: 4611         	movne	r1, r2
    68b6: 68f3         	ldr	r3, [r6, #0xc]
    68b8: f04f 0203    	mov.w	r2, #0x3
    68bc: bf18         	it	ne
    68be: 2202         	movne	r2, #0x2
    68c0: 4798         	blx	r3
    68c2: 2800         	cmp	r0, #0x0
    68c4: d158         	bne	0x6978 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x110> @ imm = #0xb0
    68c6: e9d5 0105    	ldrd	r0, r1, [r5, #20]
    68ca: 465a         	mov	r2, r11
    68cc: 68cb         	ldr	r3, [r1, #0xc]
    68ce: 4621         	mov	r1, r4
    68d0: 4798         	blx	r3
    68d2: 2800         	cmp	r0, #0x0
    68d4: d150         	bne	0x6978 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x110> @ imm = #0xa0
    68d6: e9d5 0105    	ldrd	r0, r1, [r5, #20]
    68da: 2202         	movs	r2, #0x2
    68dc: 68cb         	ldr	r3, [r1, #0xc]
    68de: f647 0184    	movw	r1, #0x7884
    68e2: f2c0 0100    	movt	r1, #0x0
    68e6: 4798         	blx	r3
    68e8: 2800         	cmp	r0, #0x0
    68ea: d145         	bne	0x6978 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x110> @ imm = #0x8a
    68ec: 9801         	ldr	r0, [sp, #0x4]
    68ee: 4629         	mov	r1, r5
    68f0: 47c0         	blx	r8
    68f2: e042         	b	0x697a <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x112> @ imm = #0x84
    68f4: b973         	cbnz	r3, 0x6914 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0xac> @ imm = #0x1c
    68f6: 460c         	mov	r4, r1
    68f8: e9d5 0105    	ldrd	r0, r1, [r5, #20]
    68fc: 68cb         	ldr	r3, [r1, #0xc]
    68fe: f647 01b5    	movw	r1, #0x78b5
    6902: 4616         	mov	r6, r2
    6904: f2c0 0100    	movt	r1, #0x0
    6908: 2203         	movs	r2, #0x3
    690a: 4798         	blx	r3
    690c: bba0         	cbnz	r0, 0x6978 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x110> @ imm = #0x68
    690e: 69e8         	ldr	r0, [r5, #0x1c]
    6910: 4632         	mov	r2, r6
    6912: 4621         	mov	r1, r4
    6914: 900d         	str	r0, [sp, #0x34]
    6916: f647 0098    	movw	r0, #0x7898
    691a: f2c0 0000    	movt	r0, #0x0
    691e: f1a7 0641    	sub.w	r6, r7, #0x41
    6922: 900c         	str	r0, [sp, #0x30]
    6924: 2301         	movs	r3, #0x1
    6926: 69a8         	ldr	r0, [r5, #0x18]
    6928: 9604         	str	r6, [sp, #0x10]
    692a: 696c         	ldr	r4, [r5, #0x14]
    692c: 9003         	str	r0, [sp, #0xc]
    692e: a802         	add	r0, sp, #0x8
    6930: e9d5 6802    	ldrd	r6, r8, [r5, #8]
    6934: f807 3c41    	strb	r3, [r7, #-65]
    6938: e9d5 e300    	ldrd	lr, r3, [r5]
    693c: f895 b020    	ldrb.w	r11, [r5, #0x20]
    6940: 9402         	str	r4, [sp, #0x8]
    6942: ac08         	add	r4, sp, #0x20
    6944: f8d5 c010    	ldr.w	r12, [r5, #0x10]
    6948: 900b         	str	r0, [sp, #0x2c]
    694a: f88d b038    	strb.w	r11, [sp, #0x38]
    694e: e884 1140    	stm.w	r4, {r6, r8, r12}
    6952: e9cd e306    	strd	lr, r3, [sp, #24]
    6956: f000 f823    	bl	0x69a0 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a> @ imm = #0x46
    695a: b968         	cbnz	r0, 0x6978 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x110> @ imm = #0x1a
    695c: f647 0184    	movw	r1, #0x7884
    6960: a802         	add	r0, sp, #0x8
    6962: f2c0 0100    	movt	r1, #0x0
    6966: 2202         	movs	r2, #0x2
    6968: f000 f81a    	bl	0x69a0 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a> @ imm = #0x34
    696c: b920         	cbnz	r0, 0x6978 <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x110> @ imm = #0x8
    696e: 9801         	ldr	r0, [sp, #0x4]
    6970: a906         	add	r1, sp, #0x18
    6972: 68ba         	ldr	r2, [r7, #0x8]
    6974: 4790         	blx	r2
    6976: b148         	cbz	r0, 0x698c <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x124> @ imm = #0x12
    6978: 2001         	movs	r0, #0x1
    697a: 4653         	mov	r3, r10
    697c: 7118         	strb	r0, [r3, #0x4]
    697e: 4618         	mov	r0, r3
    6980: f883 9005    	strb.w	r9, [r3, #0x5]
    6984: b00f         	add	sp, #0x3c
    6986: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    698a: bdf0         	pop	{r4, r5, r6, r7, pc}
    698c: e9dd 010b    	ldrd	r0, r1, [sp, #44]
    6990: 2202         	movs	r2, #0x2
    6992: 68cb         	ldr	r3, [r1, #0xc]
    6994: f647 01b8    	movw	r1, #0x78b8
    6998: f2c0 0100    	movt	r1, #0x0
    699c: 4798         	blx	r3
    699e: e7ec         	b	0x697a <core::fmt::builders::DebugStruct::field::h9b8b78f1dc05a714+0x112> @ imm = #-0x28

000069a0 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a>:
    69a0: b5f0         	push	{r4, r5, r6, r7, lr}
    69a2: af03         	add	r7, sp, #0xc
    69a4: e92d 0f00    	push.w	{r8, r9, r10, r11}
    69a8: b087         	sub	sp, #0x1c
    69aa: 4693         	mov	r11, r2
    69ac: 6802         	ldr	r2, [r0]
    69ae: 9204         	str	r2, [sp, #0x10]
    69b0: f04f 0800    	mov.w	r8, #0x0
    69b4: 6842         	ldr	r2, [r0, #0x4]
    69b6: f04f 0900    	mov.w	r9, #0x0
    69ba: 6880         	ldr	r0, [r0, #0x8]
    69bc: f04f 0a00    	mov.w	r10, #0x0
    69c0: 9005         	str	r0, [sp, #0x14]
    69c2: 1c48         	adds	r0, r1, #0x1
    69c4: 9000         	str	r0, [sp]
    69c6: f1cb 0000    	rsb.w	r0, r11, #0x0
    69ca: 9002         	str	r0, [sp, #0x8]
    69cc: 1e48         	subs	r0, r1, #0x1
    69ce: 9203         	str	r2, [sp, #0xc]
    69d0: 9106         	str	r1, [sp, #0x18]
    69d2: 9001         	str	r0, [sp, #0x4]
    69d4: e013         	b	0x69fe <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x5e> @ imm = #0x26
    69d6: 9801         	ldr	r0, [sp, #0x4]
    69d8: 5d00         	ldrb	r0, [r0, r4]
    69da: 380a         	subs	r0, #0xa
    69dc: fab0 f080    	clz	r0, r0
    69e0: 0940         	lsrs	r0, r0, #0x5
    69e2: 9b03         	ldr	r3, [sp, #0xc]
    69e4: eba4 0208    	sub.w	r2, r4, r8
    69e8: 9d05         	ldr	r5, [sp, #0x14]
    69ea: 9906         	ldr	r1, [sp, #0x18]
    69ec: 68db         	ldr	r3, [r3, #0xc]
    69ee: 7028         	strb	r0, [r5]
    69f0: 4441         	add	r1, r8
    69f2: 9804         	ldr	r0, [sp, #0x10]
    69f4: 4798         	blx	r3
    69f6: 2800         	cmp	r0, #0x0
    69f8: 46b0         	mov	r8, r6
    69fa: f040 80f1    	bne.w	0x6be0 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x240> @ imm = #0x1e2
    69fe: ea5f 70ca    	lsls.w	r0, r10, #0x1f
    6a02: f040 80e8    	bne.w	0x6bd6 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x236> @ imm = #0x1d0
    6a06: 45d9         	cmp	r9, r11
    6a08: f200 80cd    	bhi.w	0x6ba6 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x206> @ imm = #0x19a
    6a0c: f240 1a00    	movw	r10, #0x100
    6a10: f2c0 1a01    	movt	r10, #0x101
    6a14: e002         	b	0x6a1c <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x7c> @ imm = #0x4
    6a16: 45d9         	cmp	r9, r11
    6a18: f200 80c5    	bhi.w	0x6ba6 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x206> @ imm = #0x18a
    6a1c: 9806         	ldr	r0, [sp, #0x18]
    6a1e: ebab 0e09    	sub.w	lr, r11, r9
    6a22: f1be 0f07    	cmp.w	lr, #0x7
    6a26: 4448         	add	r0, r9
    6a28: d822         	bhi	0x6a70 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0xd0> @ imm = #0x44
    6a2a: 45cb         	cmp	r11, r9
    6a2c: f000 80ba    	beq.w	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0x174
    6a30: 9902         	ldr	r1, [sp, #0x8]
    6a32: 2200         	movs	r2, #0x0
    6a34: 4449         	add	r1, r9
    6a36: 5c83         	ldrb	r3, [r0, r2]
    6a38: 2b0a         	cmp	r3, #0xa
    6a3a: f000 80a0    	beq.w	0x6b7e <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1de> @ imm = #0x140
    6a3e: 188b         	adds	r3, r1, r2
    6a40: 1c5e         	adds	r6, r3, #0x1
    6a42: f000 80af    	beq.w	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0x15e
    6a46: 1886         	adds	r6, r0, r2
    6a48: 7875         	ldrb	r5, [r6, #0x1]
    6a4a: 2d0a         	cmp	r5, #0xa
    6a4c: f000 8081    	beq.w	0x6b52 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1b2> @ imm = #0x102
    6a50: 1c9d         	adds	r5, r3, #0x2
    6a52: f000 80a7    	beq.w	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0x14e
    6a56: 78b5         	ldrb	r5, [r6, #0x2]
    6a58: 2d0a         	cmp	r5, #0xa
    6a5a: d07c         	beq	0x6b56 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1b6> @ imm = #0xf8
    6a5c: 3303         	adds	r3, #0x3
    6a5e: f000 80a1    	beq.w	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0x142
    6a62: 78f3         	ldrb	r3, [r6, #0x3]
    6a64: 2b0a         	cmp	r3, #0xa
    6a66: d078         	beq	0x6b5a <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1ba> @ imm = #0xf0
    6a68: 3204         	adds	r2, #0x4
    6a6a: 188b         	adds	r3, r1, r2
    6a6c: d1e3         	bne	0x6a36 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x96> @ imm = #-0x3a
    6a6e: e099         	b	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0x132
    6a70: 1cc1         	adds	r1, r0, #0x3
    6a72: f021 0c03    	bic	r12, r1, #0x3
    6a76: ebbc 0100    	subs.w	r1, r12, r0
    6a7a: d01c         	beq	0x6ab6 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x116> @ imm = #0x38
    6a7c: 1e8d         	subs	r5, r1, #0x2
    6a7e: f1a1 0a03    	sub.w	r10, r1, #0x3
    6a82: 1e4c         	subs	r4, r1, #0x1
    6a84: 2200         	movs	r2, #0x0
    6a86: 5c83         	ldrb	r3, [r0, r2]
    6a88: 2b0a         	cmp	r3, #0xa
    6a8a: d074         	beq	0x6b76 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1d6> @ imm = #0xe8
    6a8c: 4294         	cmp	r4, r2
    6a8e: d015         	beq	0x6abc <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x11c> @ imm = #0x2a
    6a90: 1883         	adds	r3, r0, r2
    6a92: 785e         	ldrb	r6, [r3, #0x1]
    6a94: 2e0a         	cmp	r6, #0xa
    6a96: d062         	beq	0x6b5e <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1be> @ imm = #0xc4
    6a98: 4295         	cmp	r5, r2
    6a9a: d00f         	beq	0x6abc <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x11c> @ imm = #0x1e
    6a9c: 789e         	ldrb	r6, [r3, #0x2]
    6a9e: 2e0a         	cmp	r6, #0xa
    6aa0: d05f         	beq	0x6b62 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1c2> @ imm = #0xbe
    6aa2: 4592         	cmp	r10, r2
    6aa4: d00a         	beq	0x6abc <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x11c> @ imm = #0x14
    6aa6: 78db         	ldrb	r3, [r3, #0x3]
    6aa8: 2b0a         	cmp	r3, #0xa
    6aaa: d05e         	beq	0x6b6a <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1ca> @ imm = #0xbc
    6aac: 3204         	adds	r2, #0x4
    6aae: 5c83         	ldrb	r3, [r0, r2]
    6ab0: 2b0a         	cmp	r3, #0xa
    6ab2: d1eb         	bne	0x6a8c <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0xec> @ imm = #-0x2a
    6ab4: e05f         	b	0x6b76 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1d6> @ imm = #0xbe
    6ab6: f1ae 0208    	sub.w	r2, lr, #0x8
    6aba: e007         	b	0x6acc <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x12c> @ imm = #0xe
    6abc: f240 1a00    	movw	r10, #0x100
    6ac0: f1ae 0208    	sub.w	r2, lr, #0x8
    6ac4: 4291         	cmp	r1, r2
    6ac6: f2c0 1a01    	movt	r10, #0x101
    6aca: d818         	bhi	0x6afe <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x15e> @ imm = #0x30
    6acc: 2304         	movs	r3, #0x4
    6ace: eb03 060c    	add.w	r6, r3, r12
    6ad2: e956 5301    	ldrd	r5, r3, [r6, #-4]
    6ad6: f083 340a    	eor	r4, r3, #0xa0a0a0a
    6ada: ebaa 0404    	sub.w	r4, r10, r4
    6ade: 4323         	orrs	r3, r4
    6ae0: f085 340a    	eor	r4, r5, #0xa0a0a0a
    6ae4: ebaa 0404    	sub.w	r4, r10, r4
    6ae8: 4325         	orrs	r5, r4
    6aea: 402b         	ands	r3, r5
    6aec: f023 337f    	bic	r3, r3, #0x7f7f7f7f
    6af0: f1b3 3f80    	cmp.w	r3, #0x80808080
    6af4: d103         	bne	0x6afe <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x15e> @ imm = #0x6
    6af6: 3108         	adds	r1, #0x8
    6af8: 3608         	adds	r6, #0x8
    6afa: 4291         	cmp	r1, r2
    6afc: d9e9         	bls	0x6ad2 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x132> @ imm = #-0x2e
    6afe: 4571         	cmp	r1, lr
    6b00: d050         	beq	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0xa0
    6b02: 9b00         	ldr	r3, [sp]
    6b04: eb01 0209    	add.w	r2, r1, r9
    6b08: 9e02         	ldr	r6, [sp, #0x8]
    6b0a: 4413         	add	r3, r2
    6b0c: eb02 0c06    	add.w	r12, r2, r6
    6b10: ebab 0201    	sub.w	r2, r11, r1
    6b14: eba2 0a09    	sub.w	r10, r2, r9
    6b18: 2200         	movs	r2, #0x0
    6b1a: f1aa 0e01    	sub.w	lr, r10, #0x1
    6b1e: 189e         	adds	r6, r3, r2
    6b20: f816 4c01    	ldrb	r4, [r6, #-1]
    6b24: 2c0a         	cmp	r4, #0xa
    6b26: d025         	beq	0x6b74 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1d4> @ imm = #0x4a
    6b28: 4596         	cmp	lr, r2
    6b2a: d03b         	beq	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0x76
    6b2c: 5c9c         	ldrb	r4, [r3, r2]
    6b2e: 2c0a         	cmp	r4, #0xa
    6b30: d019         	beq	0x6b66 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1c6> @ imm = #0x32
    6b32: eb0c 0402    	add.w	r4, r12, r2
    6b36: 1ca5         	adds	r5, r4, #0x2
    6b38: d034         	beq	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0x68
    6b3a: 7875         	ldrb	r5, [r6, #0x1]
    6b3c: 2d0a         	cmp	r5, #0xa
    6b3e: d016         	beq	0x6b6e <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1ce> @ imm = #0x2c
    6b40: 3403         	adds	r4, #0x3
    6b42: d02f         	beq	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0x5e
    6b44: 78b4         	ldrb	r4, [r6, #0x2]
    6b46: 2c0a         	cmp	r4, #0xa
    6b48: d013         	beq	0x6b72 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1d2> @ imm = #0x26
    6b4a: 3204         	adds	r2, #0x4
    6b4c: 4592         	cmp	r10, r2
    6b4e: d1e6         	bne	0x6b1e <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x17e> @ imm = #-0x34
    6b50: e028         	b	0x6ba4 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x204> @ imm = #0x50
    6b52: 3201         	adds	r2, #0x1
    6b54: e013         	b	0x6b7e <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1de> @ imm = #0x26
    6b56: 3202         	adds	r2, #0x2
    6b58: e011         	b	0x6b7e <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1de> @ imm = #0x22
    6b5a: 3203         	adds	r2, #0x3
    6b5c: e00f         	b	0x6b7e <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1de> @ imm = #0x1e
    6b5e: 3201         	adds	r2, #0x1
    6b60: e009         	b	0x6b76 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1d6> @ imm = #0x12
    6b62: 3202         	adds	r2, #0x2
    6b64: e007         	b	0x6b76 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1d6> @ imm = #0xe
    6b66: 3201         	adds	r2, #0x1
    6b68: e004         	b	0x6b74 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1d4> @ imm = #0x8
    6b6a: 3203         	adds	r2, #0x3
    6b6c: e003         	b	0x6b76 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1d6> @ imm = #0x6
    6b6e: 3202         	adds	r2, #0x2
    6b70: e000         	b	0x6b74 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x1d4> @ imm = #0x0
    6b72: 3203         	adds	r2, #0x3
    6b74: 440a         	add	r2, r1
    6b76: f240 1a00    	movw	r10, #0x100
    6b7a: f2c0 1a01    	movt	r10, #0x101
    6b7e: eb02 0109    	add.w	r1, r2, r9
    6b82: f101 0901    	add.w	r9, r1, #0x1
    6b86: 4559         	cmp	r1, r11
    6b88: f4bf af45    	bhs.w	0x6a16 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x76> @ imm = #-0x176
    6b8c: 5c80         	ldrb	r0, [r0, r2]
    6b8e: 280a         	cmp	r0, #0xa
    6b90: f47f af41    	bne.w	0x6a16 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x76> @ imm = #-0x17e
    6b94: f04f 0a00    	mov.w	r10, #0x0
    6b98: 464e         	mov	r6, r9
    6b9a: 464c         	mov	r4, r9
    6b9c: 9805         	ldr	r0, [sp, #0x14]
    6b9e: 7800         	ldrb	r0, [r0]
    6ba0: b950         	cbnz	r0, 0x6bb8 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x218> @ imm = #0x14
    6ba2: e013         	b	0x6bcc <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x22c> @ imm = #0x26
    6ba4: 46d9         	mov	r9, r11
    6ba6: 45d8         	cmp	r8, r11
    6ba8: d015         	beq	0x6bd6 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x236> @ imm = #0x2a
    6baa: f04f 0a01    	mov.w	r10, #0x1
    6bae: 4646         	mov	r6, r8
    6bb0: 465c         	mov	r4, r11
    6bb2: 9805         	ldr	r0, [sp, #0x14]
    6bb4: 7800         	ldrb	r0, [r0]
    6bb6: b148         	cbz	r0, 0x6bcc <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x22c> @ imm = #0x12
    6bb8: 9803         	ldr	r0, [sp, #0xc]
    6bba: f247 61bf    	movw	r1, #0x76bf
    6bbe: f2c0 0100    	movt	r1, #0x0
    6bc2: 2204         	movs	r2, #0x4
    6bc4: 68c3         	ldr	r3, [r0, #0xc]
    6bc6: 9804         	ldr	r0, [sp, #0x10]
    6bc8: 4798         	blx	r3
    6bca: b948         	cbnz	r0, 0x6be0 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x240> @ imm = #0x12
    6bcc: 4544         	cmp	r4, r8
    6bce: f47f af02    	bne.w	0x69d6 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x36> @ imm = #-0x1fc
    6bd2: 2000         	movs	r0, #0x0
    6bd4: e705         	b	0x69e2 <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_str::h0d9dc3fb98f1cb0a+0x42> @ imm = #-0x1f6
    6bd6: 2000         	movs	r0, #0x0
    6bd8: b007         	add	sp, #0x1c
    6bda: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    6bde: bdf0         	pop	{r4, r5, r6, r7, pc}
    6be0: 2001         	movs	r0, #0x1
    6be2: b007         	add	sp, #0x1c
    6be4: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    6be8: bdf0         	pop	{r4, r5, r6, r7, pc}

00006bea <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_char::h8bf1b6e7a77f6c14>:
    6bea: b5f0         	push	{r4, r5, r6, r7, lr}
    6bec: af03         	add	r7, sp, #0xc
    6bee: f84d 8d04    	str	r8, [sp, #-4]!
    6bf2: 6885         	ldr	r5, [r0, #0x8]
    6bf4: e9d0 4600    	ldrd	r4, r6, [r0]
    6bf8: 7828         	ldrb	r0, [r5]
    6bfa: b178         	cbz	r0, 0x6c1c <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_char::h8bf1b6e7a77f6c14+0x32> @ imm = #0x1e
    6bfc: f247 62bf    	movw	r2, #0x76bf
    6c00: 68f3         	ldr	r3, [r6, #0xc]
    6c02: f2c0 0200    	movt	r2, #0x0
    6c06: 4688         	mov	r8, r1
    6c08: 4611         	mov	r1, r2
    6c0a: 4620         	mov	r0, r4
    6c0c: 2204         	movs	r2, #0x4
    6c0e: 4798         	blx	r3
    6c10: 4641         	mov	r1, r8
    6c12: b118         	cbz	r0, 0x6c1c <<core::fmt::builders::PadAdapter as core::fmt::Write>::write_char::h8bf1b6e7a77f6c14+0x32> @ imm = #0x6
    6c14: 2001         	movs	r0, #0x1
    6c16: f85d 8b04    	ldr	r8, [sp], #4
    6c1a: bdf0         	pop	{r4, r5, r6, r7, pc}
    6c1c: f1a1 000a    	sub.w	r0, r1, #0xa
    6c20: 6932         	ldr	r2, [r6, #0x10]
    6c22: fab0 f080    	clz	r0, r0
    6c26: 0940         	lsrs	r0, r0, #0x5
    6c28: 7028         	strb	r0, [r5]
    6c2a: 4620         	mov	r0, r4
    6c2c: f85d 8b04    	ldr	r8, [sp], #4
    6c30: e8bd 40f0    	pop.w	{r4, r5, r6, r7, lr}
    6c34: 4710         	bx	r2

00006c36 <core::fmt::Write::write_fmt::hd0505bb066a99ab9>:
    6c36: 460a         	mov	r2, r1
    6c38: f647 0198    	movw	r1, #0x7898
    6c3c: f2c0 0100    	movt	r1, #0x0
    6c40: f7ff bd54    	b.w	0x66ec <core::fmt::write::h5c77e528eed792ce> @ imm = #-0x558

00006c44 <core::result::unwrap_failed::h6d5ad66472df96c4>:
    6c44: b580         	push	{r7, lr}
    6c46: 466f         	mov	r7, sp
    6c48: b08e         	sub	sp, #0x38
    6c4a: f247 620c    	movw	r2, #0x760c
    6c4e: 9103         	str	r1, [sp, #0xc]
    6c50: f2c0 0200    	movt	r2, #0x0
    6c54: f647 0188    	movw	r1, #0x7888
    6c58: 9200         	str	r2, [sp]
    6c5a: 222b         	movs	r2, #0x2b
    6c5c: e9cd 2001    	strd	r2, r0, [sp, #4]
    6c60: 2000         	movs	r0, #0x0
    6c62: 9008         	str	r0, [sp, #0x20]
    6c64: 2002         	movs	r0, #0x2
    6c66: 9005         	str	r0, [sp, #0x14]
    6c68: f2c0 0100    	movt	r1, #0x0
    6c6c: 9007         	str	r0, [sp, #0x1c]
    6c6e: a80a         	add	r0, sp, #0x28
    6c70: 9006         	str	r0, [sp, #0x18]
    6c72: f246 60e5    	movw	r0, #0x66e5
    6c76: f2c0 0000    	movt	r0, #0x0
    6c7a: 9104         	str	r1, [sp, #0x10]
    6c7c: 900d         	str	r0, [sp, #0x34]
    6c7e: a802         	add	r0, sp, #0x8
    6c80: 900c         	str	r0, [sp, #0x30]
    6c82: f246 60d9    	movw	r0, #0x66d9
    6c86: f2c0 0000    	movt	r0, #0x0
    6c8a: f647 31ac    	movw	r1, #0x7bac
    6c8e: 900b         	str	r0, [sp, #0x2c]
    6c90: 4668         	mov	r0, sp
    6c92: 900a         	str	r0, [sp, #0x28]
    6c94: a804         	add	r0, sp, #0x10
    6c96: f2c0 0100    	movt	r1, #0x0
    6c9a: f7ff f836    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #-0xf94

00006c9e <core::option::unwrap_failed::h7d11d538d7f5b966>:
    6c9e: b580         	push	{r7, lr}
    6ca0: 466f         	mov	r7, sp
    6ca2: 4602         	mov	r2, r0
    6ca4: f647 000e    	movw	r0, #0x780e
    6ca8: f2c0 0000    	movt	r0, #0x0
    6cac: 212b         	movs	r1, #0x2b
    6cae: f7ff fd00    	bl	0x66b2 <core::panicking::panic::h26f8d9ea1f810f6d> @ imm = #-0x600

00006cb2 <core::panicking::panic_const::panic_const_async_fn_resumed::h274f44943fe874ff>:
    6cb2: b580         	push	{r7, lr}
    6cb4: 466f         	mov	r7, sp
    6cb6: b086         	sub	sp, #0x18
    6cb8: 2101         	movs	r1, #0x1
    6cba: 2000         	movs	r0, #0x0
    6cbc: 9101         	str	r1, [sp, #0x4]
    6cbe: f647 3138    	movw	r1, #0x7b38
    6cc2: f2c0 0100    	movt	r1, #0x0
    6cc6: 9004         	str	r0, [sp, #0x10]
    6cc8: 9100         	str	r1, [sp]
    6cca: f647 31ac    	movw	r1, #0x7bac
    6cce: 9003         	str	r0, [sp, #0xc]
    6cd0: 2004         	movs	r0, #0x4
    6cd2: 9002         	str	r0, [sp, #0x8]
    6cd4: f2c0 0100    	movt	r1, #0x0
    6cd8: 4668         	mov	r0, sp
    6cda: f7ff f816    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #-0xfd4

00006cde <WDT>:
    6cde: e7fe         	b	0x6cde <WDT>            @ imm = #-0x4

00006ce0 <__pre_init>:
    6ce0: 4770         	bx	lr

00006ce2 <<defmt::export::FmtWrite as core::fmt::Write>::write_str::h20f567989cf760d9>:
    6ce2: b580         	push	{r7, lr}
    6ce4: 466f         	mov	r7, sp
    6ce6: 4608         	mov	r0, r1
    6ce8: 4611         	mov	r1, r2
    6cea: f000 f998    	bl	0x701e <_defmt_write>   @ imm = #0x330
    6cee: 2000         	movs	r0, #0x0
    6cf0: bd80         	pop	{r7, pc}

00006cf2 <core::fmt::Write::write_char::h984e8cea0a715b69>:
    6cf2: b580         	push	{r7, lr}
    6cf4: 466f         	mov	r7, sp
    6cf6: b082         	sub	sp, #0x8
    6cf8: 2000         	movs	r0, #0x0
    6cfa: 2980         	cmp	r1, #0x80
    6cfc: 9001         	str	r0, [sp, #0x4]
    6cfe: d203         	bhs	0x6d08 <core::fmt::Write::write_char::h984e8cea0a715b69+0x16> @ imm = #0x6
    6d00: f88d 1004    	strb.w	r1, [sp, #0x4]
    6d04: 2101         	movs	r1, #0x1
    6d06: e032         	b	0x6d6e <core::fmt::Write::write_char::h984e8cea0a715b69+0x7c> @ imm = #0x64
    6d08: 2002         	movs	r0, #0x2
    6d0a: 460a         	mov	r2, r1
    6d0c: f5b1 6f00    	cmp.w	r1, #0x800
    6d10: d208         	bhs	0x6d24 <core::fmt::Write::write_char::h984e8cea0a715b69+0x32> @ imm = #0x10
    6d12: f360 129f    	bfi	r2, r0, #6, #26
    6d16: 20c0         	movs	r0, #0xc0
    6d18: ea40 1091    	orr.w	r0, r0, r1, lsr #6
    6d1c: f88d 2005    	strb.w	r2, [sp, #0x5]
    6d20: 2102         	movs	r1, #0x2
    6d22: e022         	b	0x6d6a <core::fmt::Write::write_char::h984e8cea0a715b69+0x78> @ imm = #0x44
    6d24: f360 129f    	bfi	r2, r0, #6, #26
    6d28: f5b1 3f80    	cmp.w	r1, #0x10000
    6d2c: d20d         	bhs	0x6d4a <core::fmt::Write::write_char::h984e8cea0a715b69+0x58> @ imm = #0x1a
    6d2e: f88d 2006    	strb.w	r2, [sp, #0x6]
    6d32: 098a         	lsrs	r2, r1, #0x6
    6d34: f360 129f    	bfi	r2, r0, #6, #26
    6d38: 20e0         	movs	r0, #0xe0
    6d3a: ea40 3011    	orr.w	r0, r0, r1, lsr #12
    6d3e: f88d 2005    	strb.w	r2, [sp, #0x5]
    6d42: f88d 0004    	strb.w	r0, [sp, #0x4]
    6d46: 2103         	movs	r1, #0x3
    6d48: e011         	b	0x6d6e <core::fmt::Write::write_char::h984e8cea0a715b69+0x7c> @ imm = #0x22
    6d4a: f88d 2007    	strb.w	r2, [sp, #0x7]
    6d4e: 098a         	lsrs	r2, r1, #0x6
    6d50: f360 129f    	bfi	r2, r0, #6, #26
    6d54: f88d 2006    	strb.w	r2, [sp, #0x6]
    6d58: 0b0a         	lsrs	r2, r1, #0xc
    6d5a: f360 129f    	bfi	r2, r0, #6, #26
    6d5e: 20f0         	movs	r0, #0xf0
    6d60: ea40 4091    	orr.w	r0, r0, r1, lsr #18
    6d64: 2104         	movs	r1, #0x4
    6d66: f88d 2005    	strb.w	r2, [sp, #0x5]
    6d6a: f88d 0004    	strb.w	r0, [sp, #0x4]
    6d6e: a801         	add	r0, sp, #0x4
    6d70: f000 f955    	bl	0x701e <_defmt_write>   @ imm = #0x2aa
    6d74: 2000         	movs	r0, #0x0
    6d76: b002         	add	sp, #0x8
    6d78: bd80         	pop	{r7, pc}

00006d7a <core::fmt::Write::write_fmt::h9bb4270745667721>:
    6d7a: 460a         	mov	r2, r1
    6d7c: 6849         	ldr	r1, [r1, #0x4]
    6d7e: 2901         	cmp	r1, #0x1
    6d80: bf18         	it	ne
    6d82: 2900         	cmpne	r1, #0x0
    6d84: f647 3140    	movw	r1, #0x7b40
    6d88: f2c0 0100    	movt	r1, #0x0
    6d8c: f7ff bcae    	b.w	0x66ec <core::fmt::write::h5c77e528eed792ce> @ imm = #-0x6a4

00006d90 <defmt::export::header::hbf0481df49aa0de6>:
    6d90: b580         	push	{r7, lr}
    6d92: 466f         	mov	r7, sp
    6d94: b082         	sub	sp, #0x8
    6d96: f827 0c02    	strh	r0, [r7, #-2]
    6d9a: 1eb8         	subs	r0, r7, #0x2
    6d9c: 2102         	movs	r1, #0x2
    6d9e: f000 f93e    	bl	0x701e <_defmt_write>   @ imm = #0x27c
    6da2: f7fe feed    	bl	0x5b80 <_defmt_timestamp> @ imm = #-0x1226
    6da6: b002         	add	sp, #0x8
    6da8: bd80         	pop	{r7, pc}

00006daa <defmt_rtt::channel::Channel::blocking_write::h9d55bf33b4dff0de>:
    6daa: b5f0         	push	{r4, r5, r6, r7, lr}
    6dac: af03         	add	r7, sp, #0xc
    6dae: e92d 0b00    	push.w	{r8, r9, r11}
    6db2: b33a         	cbz	r2, 0x6e04 <defmt_rtt::channel::Channel::blocking_write::h9d55bf33b4dff0de+0x5a> @ imm = #0x4e
    6db4: 4604         	mov	r4, r0
    6db6: 6900         	ldr	r0, [r0, #0x10]
    6db8: 68e3         	ldr	r3, [r4, #0xc]
    6dba: f3bf 8f5f    	dmb	sy
    6dbe: 4298         	cmp	r0, r3
    6dc0: d917         	bls	0x6df2 <defmt_rtt::channel::Channel::blocking_write::h9d55bf33b4dff0de+0x48> @ imm = #0x2e
    6dc2: 43de         	mvns	r6, r3
    6dc4: 1985         	adds	r5, r0, r6
    6dc6: b1ed         	cbz	r5, 0x6e04 <defmt_rtt::channel::Channel::blocking_write::h9d55bf33b4dff0de+0x5a> @ imm = #0x3a
    6dc8: 42aa         	cmp	r2, r5
    6dca: bf38         	it	lo
    6dcc: 4615         	movlo	r5, r2
    6dce: 18ee         	adds	r6, r5, r3
    6dd0: f5b6 6f80    	cmp.w	r6, #0x400
    6dd4: d91b         	bls	0x6e0e <defmt_rtt::channel::Channel::blocking_write::h9d55bf33b4dff0de+0x64> @ imm = #0x36
    6dd6: 6860         	ldr	r0, [r4, #0x4]
    6dd8: f5c3 6980    	rsb.w	r9, r3, #0x400
    6ddc: 4688         	mov	r8, r1
    6dde: 4418         	add	r0, r3
    6de0: 464a         	mov	r2, r9
    6de2: f000 fb19    	bl	0x7418 <__aeabi_memcpy> @ imm = #0x632
    6de6: 6860         	ldr	r0, [r4, #0x4]
    6de8: eb08 0109    	add.w	r1, r8, r9
    6dec: eba5 0209    	sub.w	r2, r5, r9
    6df0: e010         	b	0x6e14 <defmt_rtt::channel::Channel::blocking_write::h9d55bf33b4dff0de+0x6a> @ imm = #0x20
    6df2: 2800         	cmp	r0, #0x0
    6df4: bf12         	itee	ne
    6df6: f5c3 6580    	rsbne.w	r5, r3, #0x400
    6dfa: f240 30ff    	movweq	r0, #0x3ff
    6dfe: 1ac5         	subeq	r5, r0, r3
    6e00: 2d00         	cmp	r5, #0x0
    6e02: d1e1         	bne	0x6dc8 <defmt_rtt::channel::Channel::blocking_write::h9d55bf33b4dff0de+0x1e> @ imm = #-0x3e
    6e04: 2500         	movs	r5, #0x0
    6e06: 4628         	mov	r0, r5
    6e08: e8bd 0b00    	pop.w	{r8, r9, r11}
    6e0c: bdf0         	pop	{r4, r5, r6, r7, pc}
    6e0e: 6860         	ldr	r0, [r4, #0x4]
    6e10: 462a         	mov	r2, r5
    6e12: 4418         	add	r0, r3
    6e14: f000 fb00    	bl	0x7418 <__aeabi_memcpy> @ imm = #0x600
    6e18: f3bf 8f5f    	dmb	sy
    6e1c: f36f 269f    	bfc	r6, #10, #22
    6e20: 60e6         	str	r6, [r4, #0xc]
    6e22: 4628         	mov	r0, r5
    6e24: e8bd 0b00    	pop.w	{r8, r9, r11}
    6e28: bdf0         	pop	{r4, r5, r6, r7, pc}

00006e2a <defmt_rtt::channel::Channel::nonblocking_write::hbf939c924409534d>:
    6e2a: b5f0         	push	{r4, r5, r6, r7, lr}
    6e2c: af03         	add	r7, sp, #0xc
    6e2e: e92d 0b00    	push.w	{r8, r9, r11}
    6e32: 4614         	mov	r4, r2
    6e34: 68c2         	ldr	r2, [r0, #0xc]
    6e36: f5b4 6f80    	cmp.w	r4, #0x400
    6e3a: bf28         	it	hs
    6e3c: f44f 6480    	movhs.w	r4, #0x400
    6e40: 1916         	adds	r6, r2, r4
    6e42: 4605         	mov	r5, r0
    6e44: f5b6 6f80    	cmp.w	r6, #0x400
    6e48: f3bf 8f5f    	dmb	sy
    6e4c: d90d         	bls	0x6e6a <defmt_rtt::channel::Channel::nonblocking_write::hbf939c924409534d+0x40> @ imm = #0x1a
    6e4e: 6868         	ldr	r0, [r5, #0x4]
    6e50: f5c2 6980    	rsb.w	r9, r2, #0x400
    6e54: 4688         	mov	r8, r1
    6e56: 4410         	add	r0, r2
    6e58: 464a         	mov	r2, r9
    6e5a: f000 fadd    	bl	0x7418 <__aeabi_memcpy> @ imm = #0x5ba
    6e5e: 6868         	ldr	r0, [r5, #0x4]
    6e60: eb08 0109    	add.w	r1, r8, r9
    6e64: eba4 0209    	sub.w	r2, r4, r9
    6e68: e002         	b	0x6e70 <defmt_rtt::channel::Channel::nonblocking_write::hbf939c924409534d+0x46> @ imm = #0x4
    6e6a: 6868         	ldr	r0, [r5, #0x4]
    6e6c: 4410         	add	r0, r2
    6e6e: 4622         	mov	r2, r4
    6e70: f000 fad2    	bl	0x7418 <__aeabi_memcpy> @ imm = #0x5a4
    6e74: f3bf 8f5f    	dmb	sy
    6e78: f36f 269f    	bfc	r6, #10, #22
    6e7c: 60ee         	str	r6, [r5, #0xc]
    6e7e: 4620         	mov	r0, r4
    6e80: e8bd 0b00    	pop.w	{r8, r9, r11}
    6e84: bdf0         	pop	{r4, r5, r6, r7, pc}

00006e86 <_defmt_acquire>:
    6e86: b5f0         	push	{r4, r5, r6, r7, lr}
    6e88: af03         	add	r7, sp, #0xc
    6e8a: f84d bd04    	str	r11, [sp, #-4]!
    6e8e: b086         	sub	sp, #0x18
    6e90: f000 fa7e    	bl	0x7390 <__primask_r>    @ imm = #0x4fc
    6e94: 4604         	mov	r4, r0
    6e96: f000 fa74    	bl	0x7382 <__cpsid>        @ imm = #0x4e8
    6e9a: f240 2050    	movw	r0, #0x250
    6e9e: f2c2 0000    	movt	r0, #0x2000
    6ea2: 7ec1         	ldrb	r1, [r0, #0x1b]
    6ea4: bb89         	cbnz	r1, 0x6f0a <_defmt_acquire+0x84> @ imm = #0x62
    6ea6: 2101         	movs	r1, #0x1
    6ea8: ea21 0304    	bic.w	r3, r1, r4
    6eac: 76c1         	strb	r1, [r0, #0x1b]
    6eae: 7f42         	ldrb	r2, [r0, #0x1d]
    6eb0: 7703         	strb	r3, [r0, #0x1c]
    6eb2: b9fa         	cbnz	r2, 0x6ef4 <_defmt_acquire+0x6e> @ imm = #0x3e
    6eb4: 7741         	strb	r1, [r0, #0x1d]
    6eb6: f240 0500    	movw	r5, #0x0
    6eba: 2000         	movs	r0, #0x0
    6ebc: f2c2 0500    	movt	r5, #0x2000
    6ec0: f88d 0000    	strb.w	r0, [sp]
    6ec4: f646 662b    	movw	r6, #0x6e2b
    6ec8: 6ae8         	ldr	r0, [r5, #0x2c]
    6eca: f646 51ab    	movw	r1, #0x6dab
    6ece: f2c0 0600    	movt	r6, #0x0
    6ed2: 466c         	mov	r4, sp
    6ed4: f000 0003    	and	r0, r0, #0x3
    6ed8: f2c0 0100    	movt	r1, #0x0
    6edc: 2802         	cmp	r0, #0x2
    6ede: bf08         	it	eq
    6ee0: 460e         	moveq	r6, r1
    6ee2: f105 0018    	add.w	r0, r5, #0x18
    6ee6: 4621         	mov	r1, r4
    6ee8: 2201         	movs	r2, #0x1
    6eea: 47b0         	blx	r6
    6eec: 2800         	cmp	r0, #0x0
    6eee: d0f8         	beq	0x6ee2 <_defmt_acquire+0x5c> @ imm = #-0x10
    6ef0: 2801         	cmp	r0, #0x1
    6ef2: d103         	bne	0x6efc <_defmt_acquire+0x76> @ imm = #0x6
    6ef4: b006         	add	sp, #0x18
    6ef6: f85d bb04    	ldr	r11, [sp], #4
    6efa: bdf0         	pop	{r4, r5, r6, r7, pc}
    6efc: f647 32ac    	movw	r2, #0x7bac
    6f00: 2101         	movs	r1, #0x1
    6f02: f2c0 0200    	movt	r2, #0x0
    6f06: f7fe fe4c    	bl	0x5ba2 <core::slice::index::slice_start_index_len_fail::h0109ceeb844e56f0> @ imm = #-0x1368
    6f0a: 2101         	movs	r1, #0x1
    6f0c: 2000         	movs	r0, #0x0
    6f0e: 9101         	str	r1, [sp, #0x4]
    6f10: f647 3178    	movw	r1, #0x7b78
    6f14: f2c0 0100    	movt	r1, #0x0
    6f18: 9004         	str	r0, [sp, #0x10]
    6f1a: 9100         	str	r1, [sp]
    6f1c: f647 31ac    	movw	r1, #0x7bac
    6f20: 9003         	str	r0, [sp, #0xc]
    6f22: 2004         	movs	r0, #0x4
    6f24: 9002         	str	r0, [sp, #0x8]
    6f26: f2c0 0100    	movt	r1, #0x0
    6f2a: 4668         	mov	r0, sp
    6f2c: f7fe feed    	bl	0x5d0a <core::panicking::panic_fmt::h5c2c1e8b1a9fb026> @ imm = #-0x1226

00006f30 <_defmt_release>:
    6f30: b5f0         	push	{r4, r5, r6, r7, lr}
    6f32: af03         	add	r7, sp, #0xc
    6f34: e92d 0700    	push.w	{r8, r9, r10}
    6f38: b082         	sub	sp, #0x8
    6f3a: f240 2850    	movw	r8, #0x250
    6f3e: f240 0600    	movw	r6, #0x0
    6f42: f2c2 0800    	movt	r8, #0x2000
    6f46: f646 59ab    	movw	r9, #0x6dab
    6f4a: f898 001e    	ldrb.w	r0, [r8, #0x1e]
    6f4e: f646 652b    	movw	r5, #0x6e2b
    6f52: f2c2 0600    	movt	r6, #0x2000
    6f56: f2c0 0900    	movt	r9, #0x0
    6f5a: f2c0 0500    	movt	r5, #0x0
    6f5e: b3a0         	cbz	r0, 0x6fca <_defmt_release+0x9a> @ imm = #0x68
    6f60: 2807         	cmp	r0, #0x7
    6f62: d21b         	bhs	0x6f9c <_defmt_release+0x6c> @ imm = #0x36
    6f64: f04f 32ff    	mov.w	r2, #0xffffffff
    6f68: f898 101f    	ldrb.w	r1, [r8, #0x1f]
    6f6c: fa02 f000    	lsl.w	r0, r2, r0
    6f70: f1a7 0a1a    	sub.w	r10, r7, #0x1a
    6f74: 4308         	orrs	r0, r1
    6f76: 462c         	mov	r4, r5
    6f78: f000 007f    	and	r0, r0, #0x7f
    6f7c: f807 0c1a    	strb	r0, [r7, #-26]
    6f80: 6af0         	ldr	r0, [r6, #0x2c]
    6f82: f000 0003    	and	r0, r0, #0x3
    6f86: 2802         	cmp	r0, #0x2
    6f88: bf08         	it	eq
    6f8a: 464c         	moveq	r4, r9
    6f8c: f106 0018    	add.w	r0, r6, #0x18
    6f90: 4651         	mov	r1, r10
    6f92: 2201         	movs	r2, #0x1
    6f94: 47a0         	blx	r4
    6f96: 2800         	cmp	r0, #0x0
    6f98: d0f8         	beq	0x6f8c <_defmt_release+0x5c> @ imm = #-0x10
    6f9a: e014         	b	0x6fc6 <_defmt_release+0x96> @ imm = #0x28
    6f9c: 3079         	adds	r0, #0x79
    6f9e: f1a7 0a1b    	sub.w	r10, r7, #0x1b
    6fa2: f040 0080    	orr	r0, r0, #0x80
    6fa6: f807 0c1b    	strb	r0, [r7, #-27]
    6faa: 6af0         	ldr	r0, [r6, #0x2c]
    6fac: 462c         	mov	r4, r5
    6fae: f000 0003    	and	r0, r0, #0x3
    6fb2: 2802         	cmp	r0, #0x2
    6fb4: bf08         	it	eq
    6fb6: 464c         	moveq	r4, r9
    6fb8: f106 0018    	add.w	r0, r6, #0x18
    6fbc: 4651         	mov	r1, r10
    6fbe: 2201         	movs	r2, #0x1
    6fc0: 47a0         	blx	r4
    6fc2: 2800         	cmp	r0, #0x0
    6fc4: d0f8         	beq	0x6fb8 <_defmt_release+0x88> @ imm = #-0x10
    6fc6: 2801         	cmp	r0, #0x1
    6fc8: d122         	bne	0x7010 <_defmt_release+0xe0> @ imm = #0x44
    6fca: 2000         	movs	r0, #0x0
    6fcc: f1a7 0419    	sub.w	r4, r7, #0x19
    6fd0: f807 0c19    	strb	r0, [r7, #-25]
    6fd4: 6af0         	ldr	r0, [r6, #0x2c]
    6fd6: f000 0003    	and	r0, r0, #0x3
    6fda: 2802         	cmp	r0, #0x2
    6fdc: bf08         	it	eq
    6fde: 464d         	moveq	r5, r9
    6fe0: f106 0018    	add.w	r0, r6, #0x18
    6fe4: 4621         	mov	r1, r4
    6fe6: 2201         	movs	r2, #0x1
    6fe8: 47a8         	blx	r5
    6fea: 2800         	cmp	r0, #0x0
    6fec: d0f8         	beq	0x6fe0 <_defmt_release+0xb0> @ imm = #-0x10
    6fee: 2801         	cmp	r0, #0x1
    6ff0: d10e         	bne	0x7010 <_defmt_release+0xe0> @ imm = #0x1c
    6ff2: 2000         	movs	r0, #0x0
    6ff4: f8a8 001e    	strh.w	r0, [r8, #0x1e]
    6ff8: f888 001b    	strb.w	r0, [r8, #0x1b]
    6ffc: f898 001c    	ldrb.w	r0, [r8, #0x1c]
    7000: 2801         	cmp	r0, #0x1
    7002: bf08         	it	eq
    7004: f000 f9bf    	bleq	0x7386 <__cpsie>        @ imm = #0x37e
    7008: b002         	add	sp, #0x8
    700a: e8bd 0700    	pop.w	{r8, r9, r10}
    700e: bdf0         	pop	{r4, r5, r6, r7, pc}
    7010: f647 32ac    	movw	r2, #0x7bac
    7014: 2101         	movs	r1, #0x1
    7016: f2c0 0200    	movt	r2, #0x0
    701a: f7fe fdc2    	bl	0x5ba2 <core::slice::index::slice_start_index_len_fail::h0109ceeb844e56f0> @ imm = #-0x147c

0000701e <_defmt_write>:
    701e: b5f0         	push	{r4, r5, r6, r7, lr}
    7020: af03         	add	r7, sp, #0xc
    7022: e92d 0f00    	push.w	{r8, r9, r10, r11}
    7026: b085         	sub	sp, #0x14
    7028: 2900         	cmp	r1, #0x0
    702a: f000 80e5    	beq.w	0x71f8 <_defmt_write+0x1da> @ imm = #0x1ca
    702e: f240 2350    	movw	r3, #0x250
    7032: f240 0a00    	movw	r10, #0x0
    7036: f2c2 0300    	movt	r3, #0x2000
    703a: 1846         	adds	r6, r0, r1
    703c: 7f9c         	ldrb	r4, [r3, #0x1e]
    703e: f1a7 0921    	sub.w	r9, r7, #0x21
    7042: f1a7 0b1e    	sub.w	r11, r7, #0x1e
    7046: f2c2 0a00    	movt	r10, #0x2000
    704a: 9601         	str	r6, [sp, #0x4]
    704c: e004         	b	0x7058 <_defmt_write+0x3a> @ imm = #0x8
    704e: 2400         	movs	r4, #0x0
    7050: 83dc         	strh	r4, [r3, #0x1e]
    7052: 42b0         	cmp	r0, r6
    7054: f000 80d0    	beq.w	0x71f8 <_defmt_write+0x1da> @ imm = #0x1a0
    7058: f810 1b01    	ldrb	r1, [r0], #1
    705c: b2e2         	uxtb	r2, r4
    705e: 2a07         	cmp	r2, #0x7
    7060: d225         	bhs	0x70ae <_defmt_write+0x90> @ imm = #0x4a
    7062: 2900         	cmp	r1, #0x0
    7064: d071         	beq	0x714a <_defmt_write+0x12c> @ imm = #0xe2
    7066: f807 1c21    	strb	r1, [r7, #-33]
    706a: 4680         	mov	r8, r0
    706c: f8da 002c    	ldr.w	r0, [r10, #0x2c]
    7070: f646 652b    	movw	r5, #0x6e2b
    7074: f2c0 0500    	movt	r5, #0x0
    7078: f000 0003    	and	r0, r0, #0x3
    707c: 2802         	cmp	r0, #0x2
    707e: f646 50ab    	movw	r0, #0x6dab
    7082: f2c0 0000    	movt	r0, #0x0
    7086: bf08         	it	eq
    7088: 4605         	moveq	r5, r0
    708a: f10a 0018    	add.w	r0, r10, #0x18
    708e: 4649         	mov	r1, r9
    7090: 2201         	movs	r2, #0x1
    7092: 47a8         	blx	r5
    7094: 2800         	cmp	r0, #0x0
    7096: d0f8         	beq	0x708a <_defmt_write+0x6c> @ imm = #-0x10
    7098: 2801         	cmp	r0, #0x1
    709a: f040 80b1    	bne.w	0x7200 <_defmt_write+0x1e2> @ imm = #0x162
    709e: f240 2350    	movw	r3, #0x250
    70a2: 4640         	mov	r0, r8
    70a4: f2c2 0300    	movt	r3, #0x2000
    70a8: 7f9c         	ldrb	r4, [r3, #0x1e]
    70aa: 7fd9         	ldrb	r1, [r3, #0x1f]
    70ac: e053         	b	0x7156 <_defmt_write+0x138> @ imm = #0xa6
    70ae: 2900         	cmp	r1, #0x0
    70b0: 9002         	str	r0, [sp, #0x8]
    70b2: d07b         	beq	0x71ac <_defmt_write+0x18e> @ imm = #0xf6
    70b4: f807 1c1e    	strb	r1, [r7, #-30]
    70b8: f646 682b    	movw	r8, #0x6e2b
    70bc: f8da 002c    	ldr.w	r0, [r10, #0x2c]
    70c0: f2c0 0800    	movt	r8, #0x0
    70c4: f000 0003    	and	r0, r0, #0x3
    70c8: 2802         	cmp	r0, #0x2
    70ca: f646 50ab    	movw	r0, #0x6dab
    70ce: f2c0 0000    	movt	r0, #0x0
    70d2: bf08         	it	eq
    70d4: 4680         	moveq	r8, r0
    70d6: f10a 0518    	add.w	r5, r10, #0x18
    70da: 4659         	mov	r1, r11
    70dc: 2201         	movs	r2, #0x1
    70de: 4628         	mov	r0, r5
    70e0: 47c0         	blx	r8
    70e2: 2800         	cmp	r0, #0x0
    70e4: d0f7         	beq	0x70d6 <_defmt_write+0xb8> @ imm = #-0x12
    70e6: f1a7 081d    	sub.w	r8, r7, #0x1d
    70ea: 2801         	cmp	r0, #0x1
    70ec: f040 8088    	bne.w	0x7200 <_defmt_write+0x1e2> @ imm = #0x110
    70f0: f240 2350    	movw	r3, #0x250
    70f4: f2c2 0300    	movt	r3, #0x2000
    70f8: 7f98         	ldrb	r0, [r3, #0x1e]
    70fa: 1c44         	adds	r4, r0, #0x1
    70fc: e9dd 6001    	ldrd	r6, r0, [sp, #4]
    7100: 779c         	strb	r4, [r3, #0x1e]
    7102: b2e1         	uxtb	r1, r4
    7104: 2986         	cmp	r1, #0x86
    7106: d1a4         	bne	0x7052 <_defmt_write+0x34> @ imm = #-0xb8
    7108: 20ff         	movs	r0, #0xff
    710a: f646 662b    	movw	r6, #0x6e2b
    710e: f807 0c1d    	strb	r0, [r7, #-29]
    7112: f2c0 0600    	movt	r6, #0x0
    7116: f8da 002c    	ldr.w	r0, [r10, #0x2c]
    711a: f000 0003    	and	r0, r0, #0x3
    711e: 2802         	cmp	r0, #0x2
    7120: f646 50ab    	movw	r0, #0x6dab
    7124: f2c0 0000    	movt	r0, #0x0
    7128: bf08         	it	eq
    712a: 4606         	moveq	r6, r0
    712c: 4628         	mov	r0, r5
    712e: 4641         	mov	r1, r8
    7130: 2201         	movs	r2, #0x1
    7132: 47b0         	blx	r6
    7134: 2800         	cmp	r0, #0x0
    7136: d0f9         	beq	0x712c <_defmt_write+0x10e> @ imm = #-0xe
    7138: 2801         	cmp	r0, #0x1
    713a: d161         	bne	0x7200 <_defmt_write+0x1e2> @ imm = #0xc2
    713c: e9dd 6001    	ldrd	r6, r0, [sp, #4]
    7140: f240 2350    	movw	r3, #0x250
    7144: f2c2 0300    	movt	r3, #0x2000
    7148: e781         	b	0x704e <_defmt_write+0x30> @ imm = #-0xfe
    714a: 7fd9         	ldrb	r1, [r3, #0x1f]
    714c: 2501         	movs	r5, #0x1
    714e: fa05 f202    	lsl.w	r2, r5, r2
    7152: 4311         	orrs	r1, r2
    7154: 77d9         	strb	r1, [r3, #0x1f]
    7156: 3401         	adds	r4, #0x1
    7158: 779c         	strb	r4, [r3, #0x1e]
    715a: b2e2         	uxtb	r2, r4
    715c: 2a07         	cmp	r2, #0x7
    715e: f47f af78    	bne.w	0x7052 <_defmt_write+0x34> @ imm = #-0x110
    7162: 060a         	lsls	r2, r1, #0x18
    7164: f43f af75    	beq.w	0x7052 <_defmt_write+0x34> @ imm = #-0x116
    7168: f88d 1010    	strb.w	r1, [sp, #0x10]
    716c: 4680         	mov	r8, r0
    716e: f8da 002c    	ldr.w	r0, [r10, #0x2c]
    7172: f646 652b    	movw	r5, #0x6e2b
    7176: ac04         	add	r4, sp, #0x10
    7178: f2c0 0500    	movt	r5, #0x0
    717c: f000 0003    	and	r0, r0, #0x3
    7180: 2802         	cmp	r0, #0x2
    7182: f646 50ab    	movw	r0, #0x6dab
    7186: f2c0 0000    	movt	r0, #0x0
    718a: bf08         	it	eq
    718c: 4605         	moveq	r5, r0
    718e: f10a 0018    	add.w	r0, r10, #0x18
    7192: 4621         	mov	r1, r4
    7194: 2201         	movs	r2, #0x1
    7196: 47a8         	blx	r5
    7198: 2800         	cmp	r0, #0x0
    719a: d0f8         	beq	0x718e <_defmt_write+0x170> @ imm = #-0x10
    719c: 2801         	cmp	r0, #0x1
    719e: d12f         	bne	0x7200 <_defmt_write+0x1e2> @ imm = #0x5e
    71a0: f240 2350    	movw	r3, #0x250
    71a4: 4640         	mov	r0, r8
    71a6: f2c2 0300    	movt	r3, #0x2000
    71aa: e750         	b	0x704e <_defmt_write+0x30> @ imm = #-0x160
    71ac: f104 0079    	add.w	r0, r4, #0x79
    71b0: f646 652b    	movw	r5, #0x6e2b
    71b4: f040 0080    	orr	r0, r0, #0x80
    71b8: f807 0c1f    	strb	r0, [r7, #-31]
    71bc: f8da 002c    	ldr.w	r0, [r10, #0x2c]
    71c0: f1a7 081f    	sub.w	r8, r7, #0x1f
    71c4: f2c0 0500    	movt	r5, #0x0
    71c8: f000 0003    	and	r0, r0, #0x3
    71cc: 2802         	cmp	r0, #0x2
    71ce: f646 50ab    	movw	r0, #0x6dab
    71d2: f2c0 0000    	movt	r0, #0x0
    71d6: bf08         	it	eq
    71d8: 4605         	moveq	r5, r0
    71da: f10a 0018    	add.w	r0, r10, #0x18
    71de: 4641         	mov	r1, r8
    71e0: 2201         	movs	r2, #0x1
    71e2: 47a8         	blx	r5
    71e4: 2800         	cmp	r0, #0x0
    71e6: d0f8         	beq	0x71da <_defmt_write+0x1bc> @ imm = #-0x10
    71e8: 2801         	cmp	r0, #0x1
    71ea: d109         	bne	0x7200 <_defmt_write+0x1e2> @ imm = #0x12
    71ec: f240 2350    	movw	r3, #0x250
    71f0: 9802         	ldr	r0, [sp, #0x8]
    71f2: f2c2 0300    	movt	r3, #0x2000
    71f6: e72a         	b	0x704e <_defmt_write+0x30> @ imm = #-0x1ac
    71f8: b005         	add	sp, #0x14
    71fa: e8bd 0f00    	pop.w	{r8, r9, r10, r11}
    71fe: bdf0         	pop	{r4, r5, r6, r7, pc}
    7200: f647 32ac    	movw	r2, #0x7bac
    7204: 2101         	movs	r1, #0x1
    7206: f2c0 0200    	movt	r2, #0x0
    720a: f7fe fcca    	bl	0x5ba2 <core::slice::index::slice_start_index_len_fail::h0109ceeb844e56f0> @ imm = #-0x166c

0000720e <rust_begin_unwind>:
    720e: b580         	push	{r7, lr}
    7210: 466f         	mov	r7, sp
    7212: b08c         	sub	sp, #0x30
    7214: 4604         	mov	r4, r0
    7216: f000 f8b4    	bl	0x7382 <__cpsid>        @ imm = #0x168
    721a: f240 204c    	movw	r0, #0x24c
    721e: f2c2 0000    	movt	r0, #0x2000
    7222: 7801         	ldrb	r1, [r0]
    7224: 2900         	cmp	r1, #0x0
    7226: d13e         	bne	0x72a6 <rust_begin_unwind+0x98> @ imm = #0x7c
    7228: 2501         	movs	r5, #0x1
    722a: 7005         	strb	r5, [r0]
    722c: 9400         	str	r4, [sp]
    722e: f7ff fe2a    	bl	0x6e86 <_defmt_acquire> @ imm = #-0x3ac
    7232: f240 0032    	movw	r0, #0x32
    7236: f2c0 0000    	movt	r0, #0x0
    723a: f7ff fda9    	bl	0x6d90 <defmt::export::header::hbf0481df49aa0de6> @ imm = #-0x4ae
    723e: f240 0007    	movw	r0, #0x7
    7242: 2102         	movs	r1, #0x2
    7244: f2c0 0000    	movt	r0, #0x0
    7248: f8ad 000c    	strh.w	r0, [sp, #0xc]
    724c: a803         	add	r0, sp, #0xc
    724e: f7ff fee6    	bl	0x701e <_defmt_write>   @ imm = #-0x234
    7252: f647 3080    	movw	r0, #0x7b80
    7256: f647 3140    	movw	r1, #0x7b40
    725a: f2c0 0000    	movt	r0, #0x0
    725e: aa03         	add	r2, sp, #0xc
    7260: 9002         	str	r0, [sp, #0x8]
    7262: 4668         	mov	r0, sp
    7264: 9001         	str	r0, [sp, #0x4]
    7266: 2000         	movs	r0, #0x0
    7268: 9007         	str	r0, [sp, #0x1c]
    726a: f247 7040    	movw	r0, #0x7740
    726e: f2c0 0000    	movt	r0, #0x0
    7272: f2c0 0100    	movt	r1, #0x0
    7276: 9003         	str	r0, [sp, #0xc]
    7278: a809         	add	r0, sp, #0x24
    727a: 9005         	str	r0, [sp, #0x14]
    727c: f246 60e5    	movw	r0, #0x66e5
    7280: f2c0 0000    	movt	r0, #0x0
    7284: 9504         	str	r5, [sp, #0x10]
    7286: 900a         	str	r0, [sp, #0x28]
    7288: a801         	add	r0, sp, #0x4
    728a: 9009         	str	r0, [sp, #0x24]
    728c: 1e78         	subs	r0, r7, #0x1
    728e: 9506         	str	r5, [sp, #0x18]
    7290: f7ff fa2c    	bl	0x66ec <core::fmt::write::h5c77e528eed792ce> @ imm = #-0xba8
    7294: f647 3058    	movw	r0, #0x7b58
    7298: 2101         	movs	r1, #0x1
    729a: f2c0 0000    	movt	r0, #0x0
    729e: f7ff febe    	bl	0x701e <_defmt_write>   @ imm = #-0x284
    72a2: f7ff fe45    	bl	0x6f30 <_defmt_release> @ imm = #-0x376
    72a6: f000 f800    	bl	0x72aa <panic_probe::hard_fault::h8043dd188930637e> @ imm = #0x0

000072aa <panic_probe::hard_fault::h8043dd188930637e>:
    72aa: b580         	push	{r7, lr}
    72ac: 466f         	mov	r7, sp
    72ae: f64e 5024    	movw	r0, #0xed24
    72b2: f2ce 0000    	movt	r0, #0xe000
    72b6: 6801         	ldr	r1, [r0]
    72b8: f421 2180    	bic	r1, r1, #0x40000
    72bc: 6001         	str	r1, [r0]
    72be: f000 f86a    	bl	0x7396 <__udf>          @ imm = #0xd4

000072c2 <<&T as core::fmt::Display>::fmt::h94a932a234ad05ab>:
    72c2: b5f0         	push	{r4, r5, r6, r7, lr}
    72c4: af03         	add	r7, sp, #0xc
    72c6: f84d 8d04    	str	r8, [sp, #-4]!
    72ca: b08c         	sub	sp, #0x30
    72cc: e9d1 6505    	ldrd	r6, r5, [r1, #20]
    72d0: f647 0154    	movw	r1, #0x7854
    72d4: 68ec         	ldr	r4, [r5, #0xc]
    72d6: f2c0 0100    	movt	r1, #0x0
    72da: f8d0 8000    	ldr.w	r8, [r0]
    72de: 220c         	movs	r2, #0xc
    72e0: 4630         	mov	r0, r6
    72e2: 47a0         	blx	r4
    72e4: b120         	cbz	r0, 0x72f0 <<&T as core::fmt::Display>::fmt::h94a932a234ad05ab+0x2e> @ imm = #0x8
    72e6: 2001         	movs	r0, #0x1
    72e8: b00c         	add	sp, #0x30
    72ea: f85d 8b04    	ldr	r8, [sp], #4
    72ee: bdf0         	pop	{r4, r5, r6, r7, pc}
    72f0: 2100         	movs	r1, #0x0
    72f2: f647 023c    	movw	r2, #0x783c
    72f6: 9104         	str	r1, [sp, #0x10]
    72f8: 2103         	movs	r1, #0x3
    72fa: f8d8 0018    	ldr.w	r0, [r8, #0x18]
    72fe: f2c0 0200    	movt	r2, #0x0
    7302: 9101         	str	r1, [sp, #0x4]
    7304: 9103         	str	r1, [sp, #0xc]
    7306: a906         	add	r1, sp, #0x18
    7308: 9200         	str	r2, [sp]
    730a: f100 020c    	add.w	r2, r0, #0xc
    730e: 9102         	str	r1, [sp, #0x8]
    7310: f645 411d    	movw	r1, #0x5c1d
    7314: f2c0 0100    	movt	r1, #0x0
    7318: e9cd 1209    	strd	r1, r2, [sp, #36]
    731c: 466a         	mov	r2, sp
    731e: 910b         	str	r1, [sp, #0x2c]
    7320: f100 0108    	add.w	r1, r0, #0x8
    7324: 9108         	str	r1, [sp, #0x20]
    7326: f246 61d9    	movw	r1, #0x66d9
    732a: f2c0 0100    	movt	r1, #0x0
    732e: e9cd 0106    	strd	r0, r1, [sp, #24]
    7332: 4630         	mov	r0, r6
    7334: 4629         	mov	r1, r5
    7336: f7ff f9d9    	bl	0x66ec <core::fmt::write::h5c77e528eed792ce> @ imm = #-0xc4e
    733a: b120         	cbz	r0, 0x7346 <<&T as core::fmt::Display>::fmt::h94a932a234ad05ab+0x84> @ imm = #0x8
    733c: 2001         	movs	r0, #0x1
    733e: b00c         	add	sp, #0x30
    7340: f85d 8b04    	ldr	r8, [sp], #4
    7344: bdf0         	pop	{r4, r5, r6, r7, pc}
    7346: f647 0160    	movw	r1, #0x7860
    734a: 4630         	mov	r0, r6
    734c: f2c0 0100    	movt	r1, #0x0
    7350: 2202         	movs	r2, #0x2
    7352: 47a0         	blx	r4
    7354: b120         	cbz	r0, 0x7360 <<&T as core::fmt::Display>::fmt::h94a932a234ad05ab+0x9e> @ imm = #0x8
    7356: 2001         	movs	r0, #0x1
    7358: b00c         	add	sp, #0x30
    735a: f85d 8b04    	ldr	r8, [sp], #4
    735e: bdf0         	pop	{r4, r5, r6, r7, pc}
    7360: 4630         	mov	r0, r6
    7362: 4629         	mov	r1, r5
    7364: 4642         	mov	r2, r8
    7366: f7ff f9c1    	bl	0x66ec <core::fmt::write::h5c77e528eed792ce> @ imm = #-0xc7e
    736a: b00c         	add	sp, #0x30
    736c: f85d 8b04    	ldr	r8, [sp], #4
    7370: bdf0         	pop	{r4, r5, r6, r7, pc}

00007372 <rtic::export::executor::waker_clone::hb3709afbf1c107f0>:
    7372: 4601         	mov	r1, r0
    7374: f647 3090    	movw	r0, #0x7b90
    7378: f2c0 0000    	movt	r0, #0x0
    737c: 4770         	bx	lr

0000737e <rtic::export::executor::waker_wake::h0531fb1e6e7d4b72>:
    737e: 4700         	bx	r0

00007380 <rtic::export::executor::waker_drop::hc849bf0c0997b4b4>:
    7380: 4770         	bx	lr

00007382 <__cpsid>:
    7382: b672         	cpsid i
    7384: 4770         	bx	lr

00007386 <__cpsie>:
    7386: b662         	cpsie i
    7388: 4770         	bx	lr

0000738a <__msp_r>:
    738a: f3ef 8008    	mrs	r0, msp
    738e: 4770         	bx	lr

00007390 <__primask_r>:
    7390: f3ef 8010    	mrs	r0, primask
    7394: 4770         	bx	lr

00007396 <__udf>:
    7396: de00         	udf	#0x0
    7398: defe         	trap

0000739a <__basepri_max>:
    739a: f380 8812    	msr	basepri_max, r0
    739e: 4770         	bx	lr

000073a0 <__basepri_r>:
    73a0: f3ef 8011    	mrs	r0, basepri
    73a4: b2c0         	uxtb	r0, r0
    73a6: 4770         	bx	lr

000073a8 <__basepri_w>:
    73a8: f380 8811    	msr	basepri, r0
    73ac: 4770         	bx	lr

000073ae <__aeabi_ul2f>:
    73ae: b5d0         	push	{r4, r6, r7, lr}
    73b0: af02         	add	r7, sp, #0x8
    73b2: fab0 f280    	clz	r2, r0
    73b6: 2900         	cmp	r1, #0x0
    73b8: f102 0e20    	add.w	lr, r2, #0x20
    73bc: bf18         	it	ne
    73be: fab1 fe81    	clzne	lr, r1
    73c2: f00e 043f    	and	r4, lr, #0x3f
    73c6: f1c4 0220    	rsb.w	r2, r4, #0x20
    73ca: f1b4 0320    	subs.w	r3, r4, #0x20
    73ce: fa01 fc04    	lsl.w	r12, r1, r4
    73d2: fa00 f404    	lsl.w	r4, r0, r4
    73d6: fa20 f202    	lsr.w	r2, r0, r2
    73da: ea42 020c    	orr.w	r2, r2, r12
    73de: bf58         	it	pl
    73e0: fa00 f203    	lslpl.w	r2, r0, r3
    73e4: f3c2 13c0    	ubfx	r3, r2, #0x7, #0x1
    73e8: bf58         	it	pl
    73ea: 2400         	movpl	r4, #0x0
    73ec: ea23 2c12    	bic.w	r12, r3, r2, lsr #8
    73f0: 0a23         	lsrs	r3, r4, #0x8
    73f2: b2a4         	uxth	r4, r4
    73f4: ea43 6302    	orr.w	r3, r3, r2, lsl #24
    73f8: 4308         	orrs	r0, r1
    73fa: ea43 0304    	orr.w	r3, r3, r4
    73fe: ea4f 2412    	lsr.w	r4, r2, #0x8
    7402: eba4 54ce    	sub.w	r4, r4, lr, lsl #23
    7406: eba3 030c    	sub.w	r3, r3, r12
    740a: f104 44bd    	add.w	r4, r4, #0x5e800000
    740e: bf08         	it	eq
    7410: 0a14         	lsreq	r4, r2, #0x8
    7412: eb04 70d3    	add.w	r0, r4, r3, lsr #31
    7416: bdd0         	pop	{r4, r6, r7, pc}

00007418 <__aeabi_memcpy>:
    7418: f000 b800    	b.w	0x741c <compiler_builtins::mem::memcpy::h2f28530863632d79> @ imm = #0x0

0000741c <compiler_builtins::mem::memcpy::h2f28530863632d79>:
    741c: b5f0         	push	{r4, r5, r6, r7, lr}
    741e: af03         	add	r7, sp, #0xc
    7420: e92d 0700    	push.w	{r8, r9, r10}
    7424: 2a10         	cmp	r2, #0x10
    7426: d363         	blo	0x74f0 <compiler_builtins::mem::memcpy::h2f28530863632d79+0xd4> @ imm = #0xc6
    7428: 4243         	rsbs	r3, r0, #0
    742a: f013 0403    	ands	r4, r3, #0x3
    742e: eb00 0c04    	add.w	r12, r0, r4
    7432: d016         	beq	0x7462 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x46> @ imm = #0x2c
    7434: 4603         	mov	r3, r0
    7436: 460e         	mov	r6, r1
    7438: 7835         	ldrb	r5, [r6]
    743a: f803 5b01    	strb	r5, [r3], #1
    743e: 4563         	cmp	r3, r12
    7440: d20f         	bhs	0x7462 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x46> @ imm = #0x1e
    7442: 7875         	ldrb	r5, [r6, #0x1]
    7444: f803 5b01    	strb	r5, [r3], #1
    7448: 4563         	cmp	r3, r12
    744a: bf3e         	ittt	lo
    744c: 78b5         	ldrblo	r5, [r6, #0x2]
    744e: f803 5b01    	strblo	r5, [r3], #1
    7452: 4563         	cmplo	r3, r12
    7454: d205         	bhs	0x7462 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x46> @ imm = #0xa
    7456: 78f5         	ldrb	r5, [r6, #0x3]
    7458: 3604         	adds	r6, #0x4
    745a: f803 5b01    	strb	r5, [r3], #1
    745e: 4563         	cmp	r3, r12
    7460: d3ea         	blo	0x7438 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x1c> @ imm = #-0x2c
    7462: eba2 0e04    	sub.w	lr, r2, r4
    7466: eb01 0904    	add.w	r9, r1, r4
    746a: f02e 0803    	bic	r8, lr, #0x3
    746e: eb0c 0308    	add.w	r3, r12, r8
    7472: ea5f 7189    	lsls.w	r1, r9, #0x1e
    7476: d03e         	beq	0x74f6 <compiler_builtins::mem::memcpy::h2f28530863632d79+0xda> @ imm = #0x7c
    7478: f1b8 0f01    	cmp.w	r8, #0x1
    747c: db54         	blt	0x7528 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x10c> @ imm = #0xa8
    747e: 2118         	movs	r1, #0x18
    7480: f029 0203    	bic	r2, r9, #0x3
    7484: ea01 0ac9    	and.w	r10, r1, r9, lsl #3
    7488: ea4f 01c9    	lsl.w	r1, r9, #0x3
    748c: f102 0508    	add.w	r5, r2, #0x8
    7490: 4249         	rsbs	r1, r1, #0
    7492: 6812         	ldr	r2, [r2]
    7494: f001 0618    	and	r6, r1, #0x18
    7498: f855 1c04    	ldr	r1, [r5, #-4]
    749c: fa22 f20a    	lsr.w	r2, r2, r10
    74a0: fa01 f406    	lsl.w	r4, r1, r6
    74a4: 4322         	orrs	r2, r4
    74a6: f84c 2b04    	str	r2, [r12], #4
    74aa: 459c         	cmp	r12, r3
    74ac: d23c         	bhs	0x7528 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x10c> @ imm = #0x78
    74ae: 682a         	ldr	r2, [r5]
    74b0: fa21 f10a    	lsr.w	r1, r1, r10
    74b4: fa02 f406    	lsl.w	r4, r2, r6
    74b8: 4321         	orrs	r1, r4
    74ba: f84c 1b04    	str	r1, [r12], #4
    74be: 459c         	cmp	r12, r3
    74c0: bf3f         	itttt	lo
    74c2: 6869         	ldrlo	r1, [r5, #0x4]
    74c4: fa22 f20a    	lsrlo.w	r2, r2, r10
    74c8: fa01 f406    	lsllo.w	r4, r1, r6
    74cc: 4322         	orrlo	r2, r4
    74ce: bf3c         	itt	lo
    74d0: f84c 2b04    	strlo	r2, [r12], #4
    74d4: 459c         	cmplo	r12, r3
    74d6: d227         	bhs	0x7528 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x10c> @ imm = #0x4e
    74d8: 68aa         	ldr	r2, [r5, #0x8]
    74da: fa21 f10a    	lsr.w	r1, r1, r10
    74de: 3510         	adds	r5, #0x10
    74e0: fa02 f406    	lsl.w	r4, r2, r6
    74e4: 4321         	orrs	r1, r4
    74e6: f84c 1b04    	str	r1, [r12], #4
    74ea: 459c         	cmp	r12, r3
    74ec: d3d4         	blo	0x7498 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x7c> @ imm = #-0x58
    74ee: e01b         	b	0x7528 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x10c> @ imm = #0x36
    74f0: 4603         	mov	r3, r0
    74f2: b9f2         	cbnz	r2, 0x7532 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x116> @ imm = #0x3c
    74f4: e033         	b	0x755e <compiler_builtins::mem::memcpy::h2f28530863632d79+0x142> @ imm = #0x66
    74f6: f1b8 0f01    	cmp.w	r8, #0x1
    74fa: db15         	blt	0x7528 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x10c> @ imm = #0x2a
    74fc: 464c         	mov	r4, r9
    74fe: 6821         	ldr	r1, [r4]
    7500: f84c 1b04    	str	r1, [r12], #4
    7504: 459c         	cmp	r12, r3
    7506: d20f         	bhs	0x7528 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x10c> @ imm = #0x1e
    7508: 6861         	ldr	r1, [r4, #0x4]
    750a: f84c 1b04    	str	r1, [r12], #4
    750e: 459c         	cmp	r12, r3
    7510: bf3e         	ittt	lo
    7512: 68a1         	ldrlo	r1, [r4, #0x8]
    7514: f84c 1b04    	strlo	r1, [r12], #4
    7518: 459c         	cmplo	r12, r3
    751a: d205         	bhs	0x7528 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x10c> @ imm = #0xa
    751c: 68e1         	ldr	r1, [r4, #0xc]
    751e: 3410         	adds	r4, #0x10
    7520: f84c 1b04    	str	r1, [r12], #4
    7524: 459c         	cmp	r12, r3
    7526: d3ea         	blo	0x74fe <compiler_builtins::mem::memcpy::h2f28530863632d79+0xe2> @ imm = #-0x2c
    7528: eb09 0108    	add.w	r1, r9, r8
    752c: f00e 0203    	and	r2, lr, #0x3
    7530: b1aa         	cbz	r2, 0x755e <compiler_builtins::mem::memcpy::h2f28530863632d79+0x142> @ imm = #0x2a
    7532: 441a         	add	r2, r3
    7534: 780e         	ldrb	r6, [r1]
    7536: f803 6b01    	strb	r6, [r3], #1
    753a: 4293         	cmp	r3, r2
    753c: d20f         	bhs	0x755e <compiler_builtins::mem::memcpy::h2f28530863632d79+0x142> @ imm = #0x1e
    753e: 784e         	ldrb	r6, [r1, #0x1]
    7540: f803 6b01    	strb	r6, [r3], #1
    7544: 4293         	cmp	r3, r2
    7546: bf3e         	ittt	lo
    7548: 788e         	ldrblo	r6, [r1, #0x2]
    754a: f803 6b01    	strblo	r6, [r3], #1
    754e: 4293         	cmplo	r3, r2
    7550: d205         	bhs	0x755e <compiler_builtins::mem::memcpy::h2f28530863632d79+0x142> @ imm = #0xa
    7552: 78ce         	ldrb	r6, [r1, #0x3]
    7554: 3104         	adds	r1, #0x4
    7556: f803 6b01    	strb	r6, [r3], #1
    755a: 4293         	cmp	r3, r2
    755c: d3ea         	blo	0x7534 <compiler_builtins::mem::memcpy::h2f28530863632d79+0x118> @ imm = #-0x2c
    755e: e8bd 0700    	pop.w	{r8, r9, r10}
    7562: bdf0         	pop	{r4, r5, r6, r7, pc}

00007564 <__aeabi_memcpy8>:
    7564: 2a04         	cmp	r2, #0x4
    7566: d318         	blo	0x759a <__aeabi_memcpy8+0x36> @ imm = #0x30
    7568: b5b0         	push	{r4, r5, r7, lr}
    756a: af02         	add	r7, sp, #0x8
    756c: f1a2 0e04    	sub.w	lr, r2, #0x4
    7570: 2301         	movs	r3, #0x1
    7572: eb03 039e    	add.w	r3, r3, lr, lsr #2
    7576: f013 0403    	ands	r4, r3, #0x3
    757a: d014         	beq	0x75a6 <__aeabi_memcpy8+0x42> @ imm = #0x28
    757c: 460b         	mov	r3, r1
    757e: 4684         	mov	r12, r0
    7580: f853 5b04    	ldr	r5, [r3], #4
    7584: 2c01         	cmp	r4, #0x1
    7586: f84c 5b04    	str	r5, [r12], #4
    758a: d110         	bne	0x75ae <__aeabi_memcpy8+0x4a> @ imm = #0x20
    758c: 4660         	mov	r0, r12
    758e: 4619         	mov	r1, r3
    7590: 4672         	mov	r2, lr
    7592: f1be 0f0c    	cmp.w	lr, #0xc
    7596: d21c         	bhs	0x75d2 <__aeabi_memcpy8+0x6e> @ imm = #0x38
    7598: e030         	b	0x75fc <__aeabi_memcpy8+0x98> @ imm = #0x60
    759a: 460b         	mov	r3, r1
    759c: 4684         	mov	r12, r0
    759e: 4660         	mov	r0, r12
    75a0: 4619         	mov	r1, r3
    75a2: f7ff bf3b    	b.w	0x741c <compiler_builtins::mem::memcpy::h2f28530863632d79> @ imm = #-0x18a
    75a6: f1be 0f0c    	cmp.w	lr, #0xc
    75aa: d212         	bhs	0x75d2 <__aeabi_memcpy8+0x6e> @ imm = #0x24
    75ac: e026         	b	0x75fc <__aeabi_memcpy8+0x98> @ imm = #0x4c
    75ae: 684b         	ldr	r3, [r1, #0x4]
    75b0: 2c02         	cmp	r4, #0x2
    75b2: 6043         	str	r3, [r0, #0x4]
    75b4: d103         	bne	0x75be <__aeabi_memcpy8+0x5a> @ imm = #0x6
    75b6: 3a08         	subs	r2, #0x8
    75b8: 3108         	adds	r1, #0x8
    75ba: 3008         	adds	r0, #0x8
    75bc: e004         	b	0x75c8 <__aeabi_memcpy8+0x64> @ imm = #0x8
    75be: 688b         	ldr	r3, [r1, #0x8]
    75c0: 3a0c         	subs	r2, #0xc
    75c2: 6083         	str	r3, [r0, #0x8]
    75c4: 310c         	adds	r1, #0xc
    75c6: 300c         	adds	r0, #0xc
    75c8: 4684         	mov	r12, r0
    75ca: 460b         	mov	r3, r1
    75cc: f1be 0f0c    	cmp.w	lr, #0xc
    75d0: d314         	blo	0x75fc <__aeabi_memcpy8+0x98> @ imm = #0x28
    75d2: 4684         	mov	r12, r0
    75d4: 460b         	mov	r3, r1
    75d6: 6818         	ldr	r0, [r3]
    75d8: 3a10         	subs	r2, #0x10
    75da: f8cc 0000    	str.w	r0, [r12]
    75de: 2a03         	cmp	r2, #0x3
    75e0: 6858         	ldr	r0, [r3, #0x4]
    75e2: f8cc 0004    	str.w	r0, [r12, #0x4]
    75e6: 6898         	ldr	r0, [r3, #0x8]
    75e8: f8cc 0008    	str.w	r0, [r12, #0x8]
    75ec: 68d8         	ldr	r0, [r3, #0xc]
    75ee: f103 0310    	add.w	r3, r3, #0x10
    75f2: f8cc 000c    	str.w	r0, [r12, #0xc]
    75f6: f10c 0c10    	add.w	r12, r12, #0x10
    75fa: d8ec         	bhi	0x75d6 <__aeabi_memcpy8+0x72> @ imm = #-0x28
    75fc: e8bd 40b0    	pop.w	{r4, r5, r7, lr}
    7600: 4660         	mov	r0, r12
    7602: 4619         	mov	r1, r3
    7604: f7ff bf0a    	b.w	0x741c <compiler_builtins::mem::memcpy::h2f28530863632d79> @ imm = #-0x1ec

00007608 <HardFault_>:
    7608: e7fe         	b	0x7608 <HardFault_>     @ imm = #-0x4
    760a: d4d4         	bmi	0x75b6 <__aeabi_memcpy8+0x52> @ imm = #-0x58
