#include <stdlib.h>

// implementation de new pour le c++ a partir de malloc
// pas de delete car on ne souhaite pas en faire

void* operator new(size_t size)
{
	return malloc(size);
}

void* operator new[](size_t size)
{
	return malloc(size);
}
