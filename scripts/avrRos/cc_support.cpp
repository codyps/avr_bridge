#include <stdlib.h>

void *operator new(size_t size)
{
	return malloc(size);
}

void *operator new[] (size_t size)
{
	return ::operator new(size);
}

void operator delete(void *ptr)
{
	free(ptr);
}

void operator delete[](void *ptr)
{
	::operator delete(ptr);
}

