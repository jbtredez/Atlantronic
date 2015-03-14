#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "com_tcp.h"

ComTcp::ComTcp(const char* Ip)
{
	ip = NULL;
	if(Ip != NULL)
	{
		ip = (char*)malloc(strlen (Ip) + 1);
		strcpy(ip, Ip);
	}
}

ComTcp::~ComTcp()
{
	if( ip )
	{
		free(ip);
	}
}

int ComTcp::open()
{
	int res = 0;
	struct sockaddr_in addr;

	close();
	Com::open();

	if( ip == NULL )
	{
		log_error("ip == NULL");
		goto end;
	}

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if( fd < 0)
	{
		fd = -1;
		//log_error_errno("socket");
		close();
		goto end;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr (ip);
	addr.sin_port = htons(41666);
	res = connect(fd, (struct sockaddr *) &addr, sizeof(addr));
	if( res < 0 )
	{
		close();
		//log_error_errno("connect");
		fd = -1;
		goto end;
	}
	opened = true;
	log_info("connected to %s", ip);

end:
	return res;
}

int ComTcp::close()
{
	int res = -1;

	if(fd == -1)
	{
		goto end;
	}

	res = ::close(fd);
	if(res)
	{
		log_error_errno("close %s", ip);
	}

	fd = -1;

	log_info("close %s", ip);

end:
	opened = false;
	return res;
}

int ComTcp::write(const void* buf, int size)
{
	if(fd == -1)
	{
		return -1;
	}

	int ret = ::write(fd, buf, size);
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

int ComTcp::read(void* buf, int size)
{
	return ::read(fd, buf, size);
}
