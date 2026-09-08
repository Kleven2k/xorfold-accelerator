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

int main(void)
{
	unsigned long result;

	/* 
	* Test ordinary register folding. 
	* 
	* 0 ^ (5 + 3) = 8 
	* 8 ^ (1 + 1) = 10 
	*/

	xorfold_reset();

	xorfold_fold(5, 3);
	xorfold_fold(1, 1);

	result = xorfold_read();
	if (result != 10)
		return 1;

	/* 
	* Test single-word memory fold. 
	* 
	* 0x000a ^ 0x1000 = 0x100a 
	*/

	xorfold_fold_mem(&data);

	result = xorfold_read();

	if (result != 0x100a)
		return 2;

	/*
	 * Test reset.
	 */
	xorfold_reset();

	result = xorfold_read();
	if (result != 0)
		return 3;

	/*
	 * Test autonomous N-word memory fold.
	 * 
	 * One funct=4 instruction causes the accelerator to load:
	 *
	 *	data_n[0] = 0x11
	 *  data_n[1] = 0x22
	 *  data_n[2] = 0x44
	 *
	 * Expected:
	 *
	 *	0 ^ 0x11 ^ 0x22 ^ 0x44 = 0x77 
	 */
	xorfold_fold_mem_n(data_n, 3);

	result = xorfold_read();

	if (result != 0x77)
		return 4;

	return 0;
}
