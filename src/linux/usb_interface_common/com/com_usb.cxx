#include <string.h>
#include <stdlib.h>

#include "com_usb.h"

ComUsb::ComUsb(const char* File_read, const char* File_write)
{
	fd_read = -1;
	fd_write = -1;
	file_read = NULL;
	file_write = NULL;

	if(File_read != NULL)
	{
		file_read = (char*)malloc(strlen (File_read) + 1);
		strcpy(file_read, File_read);
	}

	if(File_write != NULL)
	{
		file_write = (char*)malloc(strlen (File_write) + 1);
		strcpy(file_write, File_write);
	}
}

ComUsb::~ComUsb()
{
	if(file_read)
	{
		free(file_read);
	}
	if(file_write)
	{
		free(file_write);
	}
}

int ComUsb::open()
{
	int res = 0;

	close();
	Com::open();

	if( file_read == NULL || file_write == NULL)
	{
		log_error("file name == NULL");
		goto end;
	}

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

int ComUsb::close()
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

end:
	opened = false;
	return res;
}

int ComUsb::write(const void* buf, int size)
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

int ComUsb::read(void* buf, int size)
{
	return ::read(fd_read, buf, size);
}
