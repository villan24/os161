#include <types.h>
#include <lib.h>
#include <test.h>


int
mytest(int nargs, char **args)
{
  //left in because i dont know what it does yet..
	(void)nargs;
	(void)args;
	kprintf(args[1]);
	kprintf("Beginning lab 4 test...\n");
	kprintf("Lab 4 test complete\n");
	return 0;
}
