#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "linux/tools/com/com.h"
#include "kernel/driver/usb.h"

Com::Com()
{
	buffer_end = 0;
	buffer_begin = 0;
	buffer_size = 0;
}

int Com::open()
{
	buffer_end = 0;
	buffer_begin = 0;
	buffer_size = 0;

	return 0;
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

		int size = read(buffer + buffer_end, max);
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

int Com::read_header(struct usb_header* header)
{
	int res = 0;

	int err = read(sizeof(*header));
	if(err)
	{
		res = err;
		goto end;
	}

	copy(header, 0, sizeof(*header));

end:
	return res;
}

void Com::copy(void* data, int offset, int size)
{
	int i, j;
	// copie du message vers un buffer non circulaire
	i = (buffer_begin + offset) % sizeof(buffer);
	for(j = 0; j < size ; j++)
	{
		((unsigned char*)data)[j] = buffer[i];
		i = (i + 1) % sizeof(buffer);
	}
}

void Com::skip(int count)
{
	buffer_size -= count;
	buffer_begin = (buffer_begin + count) % sizeof(buffer);
}
