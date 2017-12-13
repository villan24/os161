#include <types.h>
#include <lib.h>
#include <test.h>


int
mytest(int nargs, char **args)
{
  //left in because i dont know what it does yet..
	//(void)nargs;
	while(nargs--)
                printf("%s\n", *args++);
        exit(EXIT_SUCCESS);
	//(void)args;
	
	kprintf("Beginning lab 4 test...\n");
	kprintf("Lab 4 test complete\n");
	return 0;
}
