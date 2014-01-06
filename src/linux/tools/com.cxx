#include <string.h>
#include <stdlib.h>
#include "linux/tools/com.h"
#include "linux/tools/cli.h"

void Com::init(const char* File_read, const char* File_write)
{
	fd_read = -1;
	fd_write = -1;
	file_read = (char*)malloc(strlen (File_read) + 1);
	file_write = (char*)malloc(strlen (File_write) + 1);
	strcpy(file_read, File_read);
	strcpy(file_write, File_write);
	buffer_end = 0;
	buffer_begin = 0;
	buffer_size = 0;
}

void Com::destroy()
{
	if(file_read)
	{
		free(file_read);
	}
	if(file_write)
	{
		free(file_write);
	}
	file_read = NULL;
	file_write = NULL;
}

int Com::close()
{
	int res = -1;

	if(fd_read == -1 && fd_write == -1)
	{
		goto end;
	}

	if(fd_read != -1)
	{
		int res2 = ::close(fd_read);
		if(res2)
		{
			log_error_errno("close %s", file_read);
			if(errno == EBADF)
			{
				if(fd_read == fd_write)
				{
					fd_write = -1;
				}
				fd_read = -1;
			}
		}
		else
		{
			if(fd_read == fd_write)
			{
				fd_write = -1;
			}
			fd_read = -1;
		}
	}

	if(fd_write != -1)
	{
		int res2 = ::close(fd_write);
		if(res2)
		{
			log_error_errno("close %s", file_write);
			if(errno == EBADF)
			{
				fd_write = -1;
			}
		}
		else
		{
			fd_write = -1;
		}
	}

	if(fd_write == -1 && fd_read == -1)
	{
		res = 0;
	}

	log_info("close %s %s", file_read, file_write);

end:
	return res;
}

int Com::open()
{
	int res = close();

	if(fd_read != -1 || fd_write != -1)
	{
		goto end;
	}

	buffer_end = 0;
	buffer_begin = 0;
	buffer_size = 0;

	if(strcmp(file_read, file_write) == 0)
	{
		fd_read = ::open(file_read, O_RDWR);
		fd_write = fd_read;
	}
	else
	{
		fd_read = ::open(file_read, O_RDONLY);
		fd_write = ::open(file_write, O_WRONLY);
	}

	if(fd_read <= 0)
	{
		res = -1;
		fd_read = -1;
		log_error_errno("open");
		goto end;
	}

	if(fd_write <= 0)
	{
		res = -1;
		fd_write = -1;
		log_error_errno("open");
		close();
		goto end;
	}

	log_info("open  %s %s", file_read, file_write);

end:
	if(fd_write == -1 || fd_read == -1)
	{
		close();
	}
	return res;
}

void Com::open_block()
{
	struct timespec req;
	struct timespec rem;

	while( open() )
	{
		req.tv_sec = 0;
		req.tv_nsec = 100000000;
		while( nanosleep(&req, &rem) )
		{
			req = rem;
		}
	}
}

int Com::read(int min_buffer_size)
{
	int res = 0;

	if( min_buffer_size > (int) sizeof(buffer) )
	{
		res = -1;
		log_error("error : buffer trop petit : %i > %i", min_buffer_size, (int) sizeof(buffer));
		goto end;
	}

	while( buffer_size < min_buffer_size )
	{
		int max1 = sizeof(buffer) - buffer_size;
		int max2 = sizeof(buffer) - buffer_end;
		int max = max1;

		if( max2 < max1)
		{
			max = max2;
		}

		int size = ::read(fd_read, buffer + buffer_end, max);
		if(size == 0)
		{
			close();
			res = -1;
			goto end;
		}

		if(size < 0)
		{
			log_error_errno("sync - read");
			res = -1;
			goto end;
		}

		buffer_end = (buffer_end + size) % sizeof(buffer);
		buffer_size += size;
	}

end:
	return res;
}

int Com::read_header(uint16_t* type, uint16_t* size)
{
	int res = 0;
	unsigned char a, b, c, d;

	int err = read(4);
	if(err)
	{
		res = err;
		goto end;
	}

	a = buffer[buffer_begin];
	b = buffer[(buffer_begin + 1) % sizeof(buffer)];
	c = buffer[(buffer_begin + 2) % sizeof(buffer)];
	d = buffer[(buffer_begin + 3) % sizeof(buffer)];
	*type = ( a << 8 ) + b;
	*size = ( c << 8 ) + d;

end:
	return res;
}

void Com::copy_msg(char* msg, int size)
{
	int i, j;
	if(size > 1)
	{
		// copie du message (vers un buffer non circulaire)
		i = (buffer_begin + 4) % sizeof(buffer);
		for(j = 0; j < size-1 ; j++)
		{
			msg[j] = buffer[i];
			i = (i + 1) % sizeof(buffer);
		}
		msg[size-1] = 0;
	}
}

void Com::skip(int count)
{
	buffer_size -= count;
	buffer_begin = (buffer_begin + count) % sizeof(buffer);
}

int Com::write(const void* buf, int size)
{
	if(fd_write == -1)
	{
		return -1;
	}

	return ::write(fd_write, buf, size);
}
