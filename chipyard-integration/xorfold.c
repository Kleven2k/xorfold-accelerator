#include "rocc.h"

unsigned long data = 0x1000;

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

int main(void)
{
	unsigned long result;

	xorfold_reset();

	xorfold_fold(5, 3);
	xorfold_fold(1, 1);

	result = xorfold_read();
	if (result != 10)
		return 1;

	xorfold_fold_mem(&data);

	result = xorfold_read();

	if (result != 0x100a)
		return 2;

	xorfold_reset();

	result = xorfold_read();
	if (result != 0)
		return 3;

	return 0;
}
