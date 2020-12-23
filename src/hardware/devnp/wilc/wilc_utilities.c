#include <sys/slogcodes.h>
#include <malloc.h>
#include "wilc_utilities.h"


void* create_ptr(size_t size)
{
	void *ptr;
	ptr = malloc(size);

	return ptr;
}

void free_ptr(void* ptr)
{
	free(ptr);
	return;
}

void kfree(void* ptr)
{
	free(ptr);
	return;
}
