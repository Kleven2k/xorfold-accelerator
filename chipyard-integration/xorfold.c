#include "rocc.h"
#include <stdint.h>

unsigned long data = 0x1000;

/*
 * Known 3-word test buffer:
 *   0x0011
 * ^ 0x0022
 * ^ 0x0044
 * --------
 *   0x0077
*/
unsigned long data_n[3] = {
	0x11,
	0x22,
	0x44
};

/*
 * RFC 1071 checksum test buffer.
 * 
 * Bytes are stored in network order:
 *
 *	00 01  00 02  00 03  00 04
 *
 * Therefore the conceptual 16-bit words are:
 *
 *	0x0001
 *	0x0002
 *	0x0003
 *	0x0004
 *
 * Sum:
 *
 *	0x0001 + 0x0002 + 0x0003 + 0x0004 = 0x000A
*/

__attribute__((aligned(8))) unsigned char checksum_data[8] = {
	0x00, 0x01,
	0x00, 0x02,
	0x00, 0x03,
	0x00, 0x04
};

/*
 * Must be volatile because the accelerator writes this location
 * through its own memory interface. GCC cannot otherwise see that
 * the custom instruction modifies this C object.
*/
volatile unsigned long writeback = 0;

/*
 * Broadcast-write test destination.
 *
 * Also volatile because funct=10 modifies this buffer through the
 * accelerator memory interface rather than through ordinary C stores.
*/
volatile unsigned long writeback_n[3] = {
	0,
	0,
	0
};

/* 
 * ------------------------------------------------------------ 
 * XOR-fold operations 
 * ------------------------------------------------------------ 
*/

static inline void xorfold_reset(void)
{
	ROCC_INSTRUCTION(3, 0);
}

static inline void xorfold_fold(unsigned long a, unsigned long b)
{
	ROCC_INSTRUCTION_SS(3, a, b, 1);
}

static inline unsigned long xorfold_read(void)
{
	unsigned long value;
	ROCC_INSTRUCTION_D(3, value, 2);
	return value;
}

static inline void xorfold_fold_mem(unsigned long *ptr)
{
	ROCC_INSTRUCTION_S(3, ptr, 3);
}

static inline void xorfold_fold_mem_n(
	unsigned long *ptr,
	unsigned long n
)
{
	/*
	 * funct = 4
	 * 
	 * rs1 = starting pointer
	 * rs2 = number of 64-bit words 
	*/
	ROCC_INSTRUCTION_SS(3, ptr, n, 4);
}

static inline void xorfold_write_result(unsigned long *dst_ptr)
{
	/*
	 * funct = 5
	 *
	 * rs1 = destination address
	 * 
	 * Write one copy of the current XOR accum value.
	*/
	ROCC_INSTRUCTION_S(3, dst_ptr, 5);
}

/*
 * funct = 10
 * 
 * write_result_n(dst_ptr, n)
 *
 * rs1 = starting destination address
 * rs2 = number of 64-bit words
 *
 * The accelerator captures the current XOR accum value once and
 * broadcasts that same value to n consecutive 64-bit words.
*/
static inline void xorfold_write_result_n(
	unsigned long *dst_ptr,
	unsigned long n
)
{
	ROCC_INSTRUCTION_SS(3, dst_ptr, n, 10);
}

/*
 * ------------------------------------------------------------ 
 * RFC 1071 / RFC 1624 checksum operations 
 * ------------------------------------------------------------
*/

static inline void checksum_reset(void)
{
	/*
	 * funct = 6
	 *
	 * checksumAccum = 0
	*/
	ROCC_INSTRUCTION(3, 6);
}

static inline void checksum_add_n(
	void *ptr,
	unsigned long n 
)
{
	/*
	 * funct = 7
	 *
	 * rs1 = starting buffer address
	 * rs2 = number of 8-byte memory beats
	 *
	 * This operation accumulates on top of the existing
	 * checksumAccum value. It does not reset it first.
	*/
	ROCC_INSTRUCTION_SS(3, ptr, n, 7);
}

static inline unsigned long checksum_finalize(void)
{
	unsigned long value;

	/*
	 * funct = 8
	 *
	 * checksumAccum already contains the scalar 16-bit running
	 * one's-complement sum.
	 *
	 * Hardware complements that value and returns the finalized
	 * RFC 1071 checksum zero-extended in rd.
	*/
	ROCC_INSTRUCTION_D(3, value, 8);

	return value;
}

static inline void checksum_update(
	unsigned long old_word,
	unsigned long new_word
)
{
	/*
	 * funct = 9
	 *
	 * RFC 1624 incremental update.
	 *
	 * rs1[15:0] = old 16-bit word
	 * rs2[15:0] = new 16-bit word
	 *
	 * No rd response. The caller uses checksum_finalize()
	 * afterward to read the new checksum.
	*/
	ROCC_INSTRUCTION_SS(3, old_word, new_word, 9);
}

/*
 * ------------------------------------------------------------
 * Independent software RFC 1071 implementation
 * ------------------------------------------------------------
 *
 * This operates directly on the byte stream in network order.
 * It is intentionally independent of the accelerator datapath.
*/
static uint16_t checksum_software(
	const unsigned char *buf,
	unsigned long len
)
{
	uint32_t sum = 0;

	while (len >= 2) {
		uint16_t word =
			((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);

		sum += word;

		/*
		 * End-around carry.
		*/
		sum = (sum & 0xFFFFU) + (sum >> 16);

		buf += 2;
		len -= 2;
	}

	/*
	 * RFC 1071 permits an odd final byte. It occupies the high
	 * byte of the final 16-bit word; the low byte is zero.
	 *
	 * The current hardware funct=7 does not yet support arbitrary
	 * odd lengths, but keeping the software reference correct makes
	 * it useful when that feature is added later.
	*/
	if (len != 0) {
		sum += ((uint16_t)buf[0] << 8);

		sum = (sum & 0xFFFFU) + (sum >> 16);
	}

	/*
	 * One more fold in case the previous addition produced
	 * another carry.
	*/
	sum = (sum & 0xFFFFU) + (sum >> 16);

	return (uint16_t)(~sum);
}


int main(void)
{
	unsigned long result;
	uint16_t software_checksum;

	/* 
	 * ------------------------------------------------------------
	 * Test 1: normal register folding
	 * 
	 * 	 0 ^ (5 + 3) = 8 
	 * 	 8 ^ (1 + 1) = 10 
	 * ------------------------------------------------------------
	*/

	xorfold_reset();

	xorfold_fold(5, 3);
	xorfold_fold(1, 1);

	result = xorfold_read();

	if (result != 10)
		return 1;

    /* 
	 * ------------------------------------------------------------
	 * Test 2: single-word fold from memory
	 * 
	 * 	 accum = 0x000a
	 *	 data  = 0x1000
	 * 
	 * 	 0x000a ^ 0x1000 = 0x100a 
	 * ------------------------------------------------------------
	*/

	xorfold_fold_mem(&data);

	result = xorfold_read();

	if (result != 0x100a)
		return 2;

	/*
	 * ------------------------------------------------------------
	 * Test 3: XOR accumulator reset
	 * ------------------------------------------------------------
	*/
	xorfold_reset();

	result = xorfold_read();
	if (result != 0)
		return 3;

	/*
	 * ------------------------------------------------------------
	 * Test 4: autonomous N-word memory fold
	 * 
	 * One funct=4 instruction causes the accelerator to load:
	 *
	 *   data_n[0] = 0x11
	 *   data_n[1] = 0x22
	 *   data_n[2] = 0x44
	 *
	 * Expected:
	 *
	 * 	 0 ^ 0x11 ^ 0x22 ^ 0x44 = 0x77
	 * ------------------------------------------------------------
	*/
	xorfold_fold_mem_n(data_n, 3);

	result = xorfold_read();

	if (result != 0x77)
		return 4;

	/*
	 * ------------------------------------------------------------
	 * Test 5: write the accumulator result back to memory
	 *
	 * Current accumulator value:
	 *
	 *   accum = 0x77
	 *
	 * funct=5 should perform:
	 * 
	 *   writeback = 0x77
	 * ------------------------------------------------------------
	*/
	writeback = 0;

	xorfold_write_result((unsigned long *)&writeback);

	/*
	 * Synchronize with the accelerator.
	 *
	 * write_result has no rd response. A following xorfold_read()
	 * cannot execute until the store has completed and the memory
	 * FSM has returned to sIdle.
	 *
	 * The returned value also verifies that the store operation did
	 * not modify accum.
	*/
	result = xorfold_read();

	if (result != 0x77)
		return 5;

	/*
	 * Order memory accesses before the CPU reloads writeback.
	 *
	 * The "memory" clobber prevents GCC from moving C memory accesses
	 * accros this point.
	 *
	 * The RISC-V fence provides architectural memory ordering.
	 *
	 * writeback is volatile so GCC must emit an actual load instead of
	 * replacing the read with a compile-time constant.
	*/
	asm volatile(
		"fence rw, rw"
		:
		:
		: "memory"
	);

	result = writeback;

	if (result != 0x77)
		return 6;

	/*
	 * ------------------------------------------------------------
	 * Test 6: independently verify the stored value through the
	 * accelerator itself.
	 * 
	 * Reset accum, then fold writeback from memory:
	 * 
	 *   0 ^ 0x77 = 0x77
	 * ------------------------------------------------------------
	*/
	xorfold_reset();

	xorfold_fold_mem((unsigned long *)&writeback);

	result = xorfold_read();

	if (result != 0x77)
		return 7;

	/*
	 * ------------------------------------------------------------
	 * Test 7: software RFC 1071 reference
	 *
	 *   0001 + 0002 + 0003 + 0004 = 000A
	 *	 ~000A = FFF5
	 *
	 * This verifies our hand-computed expected value independently
	 * of the accelerator.
	 * ------------------------------------------------------------
	*/
	software_checksum =
		checksum_software(
			checksum_data,
			sizeof(checksum_data)
		);

	if (software_checksum != 0xFFF5)
		return 8;

	/*
	 * ------------------------------------------------------------
	 * Test 8: accelerator RFC 1071 checksum
	 * ------------------------------------------------------------
	*/
	checksum_reset();

	/*
	 * checksum_data is exactly 8 bytes, so funct=7 needs one
	 * 64-bit memory beat.
	*/
	checksum_add_n(
		checksum_data,
		1
	);

	/*
	 * funct=8 cannot execute until funct=7's final memory response 
	 * has returned and the FSM is back in sIdle.
	*/
	result = checksum_finalize();

	/*
	 * First compare agains the hand-computed RFC 1071 result.
	*/
	if (result != 0xFFF5)
		return 9;

	/*
	 * Then compare hardware against the independent software
	 * implementation.
	*/
	if (result != software_checksum)
		return 10;

	/*
	 * ------------------------------------------------------------
	 * Test 9:  RFC 1624 incremental checksum update
	 *
	 * Original words:
	 *
	 *   0001 0002 0003 0004
	 *
	 * Original uncomplemented sum:
	 *
	 *   S = 000A
	 *
	 * Change:
	 *
	 *   old = 0002
	 *   new = 0005
	 * 
	 * New full sum by inspection:
	 *
	 *   0001 + 0005 + 0003 + 0004
	 * = 000D
	 *
	 * Expected new checksum:
	 *
	 *   ~000D = FFF2
	 *
	 * RFC 1624 equivalent:
	 *
	 *   C' = ~(~C + ~old + new)
	 * ------------------------------------------------------------
	*/
	checksum_update(
		0x0002,
		0x0005
	);

	result = checksum_finalize();

	if (result != 0xFFF2)
		return 11;

	/*
	 * ------------------------------------------------------------
	 * Test 10: verify checksum operations did not affect XOR accum
	 *
	 * XOR accum should still contain 0x77.
	 * ------------------------------------------------------------
	*/

	result = xorfold_read();

	if (result != 0x77)
		return 12;

	/*
	 * ------------------------------------------------------------
	 * Test 11: funct=10 broadcast write_result_n
	 *
	 * Current XOR accum:
	 *
	 *   0x77
	 *
	 * Operation:
	 *
	 *   write_result_(writeback_n, 3)
	 *
	 * Expected:
	 *
	 *   writeback_n[0] = 0x77
	 *	 writeback_n[1] = 0x77
	 *   writeback_n[2] = 0x77
	 * ------------------------------------------------------------
	*/
	
	writeback_n[0] = 0;
	writeback_n[1] = 0;
	writeback_n[2] = 0;
	
	xorfold_write_result_n(
		(unsigned long *)writeback_n,
		3
	);

	/*
	 * Synchronize through another RoCC command.
	 * 
	 * This read cannot execute until all three stores have
	 * completed and the memory FSM returns to sIdle.
	 *
	 * It also verifies that funct=10 does not modify accum.
	*/
	result = xorfold_read();

	if (result != 0x77)
		return 13;

	/*
	 * Ensure the CPU observes the accelerator stores before loading
	 * writeback_n[].
	*/
	asm volatile(
		"fence rw, rw"
		:
		:
		: "memory"
	);

	if (writeback_n[0] != 0x77)
		return 14;

	if (writeback_n[1] != 0x77)
		return 15;

	if (writeback_n[2] != 0x77)
		return 16;

	/*
	 * ------------------------------------------------------------
	 * Test 12: funct=10 with n=0 must be a no-op
	 *
	 * Preload recognizable destination values:
	 *
	 *   writeback_n[0] = 0x1111
	 *   writeback_n[1] = 0x2222
	 *   writeback_n[2] = 0x3333
	 *
	 * Then issue:
	 *
	 *   write_result_n(writeback_n, 0)
	 *
	 * Hardware should issue no memory request and leave all three
	 * values unchanged.
	 * ------------------------------------------------------------
	*/

	writeback_n[0] = 0x1111;
	writeback_n[1] = 0x2222;
	writeback_n[2] = 0x3333;

	/*
	 * Make the initialization stores architecturally visible before
	 * issuing the custom instruction.
	*/
	asm volatile(
		"fence rw, rw"
		:
		:
		: "memory"
	);

	xorfold_write_result_n(
		(unsigned long *)writeback_n,
		0
	);

	/*
	 * n=0 should not start the memory FSM.
	 *
	 * This read should therefore proceed without waiting for any
	 * accelerator memory transaction.
	*/
	result = xorfold_read();

	if (result != 0x77)
		return 17;

	asm volatile(
		"fence rw, rw"
		:
		:
		: "memory"
	);

	if (writeback_n[0] != 0x1111)
		return 18;

	if (writeback_n[1] != 0x2222)
		return 19;

	if (writeback_n[2] != 0x3333)
		return 20;

	return 0;
}
