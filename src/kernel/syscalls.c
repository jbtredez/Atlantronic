//! @file syscalls.c
//! @brief Syscalls
//! @author Atlantronic

//! @todo tout voir

#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

int _read_r (struct _reent *r, int file, char * ptr, int len)
{
	(void) r;
	(void) file;
	(void) ptr;
	(void) len;

	errno = EINVAL;
	return -1;
}

int _lseek_r (struct _reent *r, int file, int ptr, int dir)
{
	(void) r;
	(void) file;
	(void) ptr;
	(void) dir;

	return 0;
}

int _write_r (struct _reent *r, int file, char * ptr, int len)
{  
	(void) r;
	(void) file;
	(void) ptr;

	return len;
}

int _close_r (struct _reent *r, int file)
{
	(void) r;
	(void) file;

	return 0;
}

register char * stack_ptr asm ("sp");

static char* heap_end_for_debug;

caddr_t _sbrk_r (struct _reent *r, int incr)
{
	(void) r;

	extern char   end asm ("end");
	static char * heap_end = NULL;
	char *        prev_heap_end;

	if (heap_end == NULL)
		heap_end = & end;

	prev_heap_end = heap_end;

	if (heap_end + incr > stack_ptr)
	{
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;
	heap_end_for_debug = heap_end;

	return (caddr_t) prev_heap_end;
}

int _fstat_r (struct _reent *r, int file, struct stat * st)
{
	(void) r; 
	(void) file;

	memset (st, 0, sizeof (* st));
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty_r(struct _reent *r, int fd)
{
	(void) r;
	(void) fd;

	return 1;
}
