#include "rocc.h"

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
 * Must be volatile because the accelerator writes this location
 * through its own memory interface. GCC cannot otherwise see that
 * the custom instruction modifies this C object.
 */
volatile unsigned long writeback = 0;

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
	 * rs2 = number of words 
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
	 * The accelerator writes the current accum value to memory.
	 */
	ROCC_INSTRUCTION_S(3, dst_ptr, 5);
}

int main(void)
{
	unsigned long result;

	/* 
	* Test 1: normal register folding. 
	* 
	* 	0 ^ (5 + 3) = 8 
	* 	8 ^ (1 + 1) = 10 
	*/

	xorfold_reset();

	xorfold_fold(5, 3);
	xorfold_fold(1, 1);

	result = xorfold_read();

	if (result != 10)
		return 1;

	/* 
	* Test 2: single-word fold from memory. 
	* 
	* 	accum = 0x000a
	*	data  = 0x1000
	*
	* 	0x000a ^ 0x1000 = 0x100a 
	*/

	xorfold_fold_mem(&data);

	result = xorfold_read();

	if (result != 0x100a)
		return 2;

	/*
	 * Test 3: reset.
	 */
	xorfold_reset();

	result = xorfold_read();
	if (result != 0)
		return 3;

	/*
	 * Test 4: autonomous N-word memory fold.
	 * 
	 * One funct=4 instruction causes the accelerator to load:
	 *
	 *	data_n[0] = 0x11
	 *  data_n[1] = 0x22
	 *  data_n[2] = 0x44
	 *
	 * Expected:
	 *
	 * 	0 ^ 0x11 ^ 0x22 ^ 0x44 = 0x77
	 */
	xorfold_fold_mem_n(data_n, 3);

	result = xorfold_read();

	if (result != 0x77)
		return 4;

	/*
	 * Test 5: write the accumulator result back to memory.
	 *
	 * Current accumulator value:
	 *
	 *	accum = 0x77
	 *
	 * funct=5 should perform:
	 * 
	 *   writeback = 0x77
	 */
	writeback = 0;

	xorfold_write_result((unsigned long *)&writeback);

	/*
	 * Synchronize with the accelerator.
	 *
	 * write_result has no rd response, so its custom instruction can
	 * be accepted before the accelerator's store transaction has fully
	 * completed.
	 *
	 * A following RoCC read cannot execute until the memory FSM returns
	 * to sIdle. For funct=5, that happens only after the store response
	 * arrives.
	 *
	 * The returned vaue also verifies that write_result did not modify
	 * accum.
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
	 * Test 6: independently verify the stored value through the
	 * accelerator itself.
	 * 
	 * Reset accum, then fold writeback from memory:
	 * 
	 *	0 ^ 0x77 = 0x77
	 *
	 * The explicit cast is used because writeback is volatile while
	 * xorfold_fold_mem() only consumes the address and does not
	 * dereference the pointer in C.
	 */
	xorfold_reset();

	xorfold_fold_mem((unsigned long *)&writeback);

	result = xorfold_read();

	if (result != 0x77)
		return 7;

	return 0;
}
