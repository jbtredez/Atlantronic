#include "linux/tools/com.h"

void com_init(struct com* com)
{
	com->fd = -1;
	com->buffer_end = 0;
	com->buffer_begin = 0;
	com->buffer_size = 0;
}

void com_close(struct com* com)
{
	if(com->fd != -1)
	{
		close(com->fd);
		com->fd = -1;
	}
}

void com_open(struct com* com, const char* file)
{
	int last_error = 0;
	com->buffer_end = 0;
	com->buffer_begin = 0;
	com->buffer_size = 0;

	if(com->fd > 0)
	{
		printf("close usb (=> reopen)\n");
		if( close(com->fd) )
		{
			perror("close");
		}
	}

	while(1)
	{
		com->fd = open(file, O_RDWR);
		if(com->fd <= 0)
		{
			if( last_error != errno )
			{
				perror("open");
				last_error = errno;
			}
		}
		else
		{
			printf("open usb\n");
			return;
		}
		usleep(1000000);
	}
}

int com_read(struct com* com, int min_buffer_size)
{
	if( min_buffer_size > (int) sizeof(com->buffer) )
	{
		printf("error : buffer trop petit : %i > %i\n", min_buffer_size, (int) sizeof(com->buffer));
		return -1;
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
			printf("close usb\n");
			close(com->fd);
			com->fd = -1;
			return -1;
		}

		if(size < 0)
		{
			perror("sync - read");
			return -1;
		}

		com->buffer_end = (com->buffer_end + size) % sizeof(com->buffer);
		com->buffer_size += size;
	}

	return 0;
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
	if(com->fd == -1)
	{
		return -1;
	}

	return write(com->fd, buf, size);
}