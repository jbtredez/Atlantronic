#ifndef COM_H
#define COM_H

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>

struct com
{
	int fd;
	unsigned char buffer[65536];
	int buffer_end;
	int buffer_begin;
	int buffer_size;
};

void com_init(struct com* com);
void com_close(struct com* com);
void com_open(struct com* com, const char* file);
int com_read(struct com* com, int min_buffer_size);
int com_read_header(struct com* com, uint16_t* type, uint16_t* size);
void com_copy_msg(struct com* com, char* msg, int size);
void com_skip(struct com* com, int count);

#endif
