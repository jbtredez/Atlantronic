#include <string.h>
#include "linux/tools/com.h"
#include "linux/tools/cli.h"

void com_init(struct com* com, const char* file)
{
	com->fd = -1;
	com->file = malloc(strlen (file) + 1);
	strcpy(com->file, file);
	com->buffer_end = 0;
	com->buffer_begin = 0;
	com->buffer_size = 0;
}

int com_close(struct com* com)
{
	int res = 0;

	if(com->fd == -1)
	{
		res = -1;
		goto end;
	}

	res = close(com->fd);
	if(res)
	{
		log_error_errno("close");
		if(errno == EBADF)
		{
			com->fd = -1;
		}
		goto end;
	}

	com->fd = -1;
	log_info("close usb");

end:
	return res;
}

int com_open(struct com* com)
{
	int res = com_close(com);

	if(com->fd != -1)
	{
		goto end;
	}

	com->buffer_end = 0;
	com->buffer_begin = 0;
	com->buffer_size = 0;
	com->fd = open(com->file, O_RDWR);

	if(com->fd <= 0)
	{
		res = -1;
		com->fd = -1;
		log_error_errno("open");
		goto end;
	}

	log_info("open usb");

end:
	return res;
}

void com_open_block(struct com* com)
{
	struct timespec req;
	struct timespec rem;

	while( com_open(com) )
	{
		req.tv_sec = 1;
		req.tv_nsec = 0;
		while( nanosleep(&req, &rem) )
		{
			req = rem;
		}
	}
}

int com_read(struct com* com, int min_buffer_size)
{
	int res = 0;

	if( min_buffer_size > (int) sizeof(com->buffer) )
	{
		res = -1;
		log_error("error : buffer trop petit : %i > %i", min_buffer_size, (int) sizeof(com->buffer));
		goto end;
	}

	while( com->buffer_size < min_buffer_size )
	{
		int max1 = sizeof(com->buffer) - com->buffer_size;
		int max2 = sizeof(com->buffer) - com->buffer_end;
		int max = max1;

		if( max2 < max1)
		{
			max = max2;
		}

		int size = read(com->fd, com->buffer + com->buffer_end, max);
		if(size == 0)
		{
			com_close(com);
			res = -1;
			goto end;
		}

		if(size < 0)
		{
			log_error_errno("sync - read");
			res = -1;
			goto end;
		}

		com->buffer_end = (com->buffer_end + size) % sizeof(com->buffer);
		com->buffer_size += size;
	}

end:
	return res;
}

int com_read_header(struct com* com, uint16_t* type, uint16_t* size)
{
	int res = 0;
	unsigned char a, b, c, d;

	int err = com_read(com, 4);
	if(err)
	{
		res = err;
		goto end;
	}

	a = com->buffer[com->buffer_begin];
	b = com->buffer[(com->buffer_begin + 1) % sizeof(com->buffer)];
	c = com->buffer[(com->buffer_begin + 2) % sizeof(com->buffer)];
	d = com->buffer[(com->buffer_begin + 3) % sizeof(com->buffer)];
	*type = ( a << 8 ) + b;
	*size = ( c << 8 ) + d;

end:
	return res;
}

void com_copy_msg(struct com* com, char* msg, int size)
{
	int i, j;
	if(size > 1)
	{
		// copie du message (vers un buffer non circulaire)
		i = (com->buffer_begin + 4) % sizeof(com->buffer);
		for(j = 0; j < size-1 ; j++)
		{
			msg[j] = com->buffer[i];
			i = (i + 1) % sizeof(com->buffer);
		}
		msg[size-1] = 0;
	}
}

void com_skip(struct com* com, int count)
{
	com->buffer_size -= count;
	com->buffer_begin = (com->buffer_begin + count) % sizeof(com->buffer);
}

int com_write(struct com* com, const char* buf, int size)
{
	int fd = com->fd;
	if(fd == -1)
	{
		return -1;
	}

	return write(fd, buf, size);
}