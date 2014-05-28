#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "linux/tools/com.h"
#include "linux/tools/cli.h"
#include "kernel/driver/usb.h"

void Com::init(const char* File_read, const char* File_write, const char* Ip)
{
	fd_read = -1;
	fd_write = -1;
	if(File_read != NULL)
	{
		file_read = (char*)malloc(strlen (File_read) + 1);
		strcpy(file_read, File_read);
	}
	else
	{
		file_read = NULL;
	}

	if(File_write != NULL)
	{
		file_write = (char*)malloc(strlen (File_write) + 1);
		strcpy(file_write, File_write);
	}
	else
	{
		file_write = NULL;
	}
	if(Ip != NULL)
	{
		ip = (char*)malloc(strlen (Ip) + 1);
		strcpy(ip, Ip);
	}
	else
	{
		ip = NULL;
	}

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

	if( file_read )
	{
		log_info("close %s %s", file_read, file_write);
	}
	else if( ip )
	{
		if(opened)
		{
			log_info("close %s", ip);
		}
	}

end:
	opened = false;
	return res;
}

int Com::open()
{
	int res = 0;

	close();

	if(fd_read != -1 || fd_write != -1)
	{
		goto end;
	}

	buffer_end = 0;
	buffer_begin = 0;
	buffer_size = 0;

	if( file_read != NULL && file_write != NULL)
	{
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
	}
	else if( ip != NULL )
	{
		struct sockaddr_in addr;
		int res;

		fd_read = socket(AF_INET, SOCK_STREAM, 0);
		if( fd_read < 0)
		{
			fd_read = -1;
			//log_error_errno("socket");
			close();
			goto end;
		}

		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = inet_addr (ip);
		addr.sin_port = htons(41666);
		res = connect(fd_read, (struct sockaddr *) &addr, sizeof(addr));
		if( res < 0 )
		{
			close();
			//log_error_errno("connect");
			fd_read = -1;
			goto end;
		}
		fd_write = fd_read;
		opened = true;
		log_info("connected to %s", ip);
	}

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

int Com::write(const void* buf, int size)
{
	if(fd_write == -1)
	{
		return -1;
	}

	int ret = ::write(fd_write, buf, size);
	if( ret != size )
	{
		if( ret < 0 )
		{
			log_error("write error %d : %s", errno, strerror(errno));
		}
		else
		{
			log_error("partial write %d / %d", ret, size);
		}
		return -1;
	}
	else
	{
		return 0;
	}
}
