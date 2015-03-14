#include "rs.h"

Rs::Rs()
{
	m_fd = -1;
}

bool Rs::configure(const char* fileName, int baudrate, int nData, int vTimeMs)
{
	tcflag_t tcflagBaud;
	tcflag_t tcflagNData;

	strncpy(m_fileName, fileName, sizeof(m_fileName));
	m_vTimeMs = vTimeMs;

	switch( baudrate )
	{
		case 9600:
			tcflagBaud = B9600;
			break;
		case 115200:
			tcflagBaud = B115200;
			break;
/*		case 250000:
			tcflagBaud = B250000;
			break;*/
		case 500000:
			tcflagBaud = B500000;
			break;
		case 1000000:
			tcflagBaud = B1000000;
			break;
		default:
			tcflagBaud = 0;
			return false;
			break;
	}

	tcflagNData = 0;
	switch ( nData )
	{
		case 5:
			tcflagNData = CS5;
			break;
		case 6:
			tcflagNData = CS6;
			break;
		case 7:
			tcflagNData = CS7;
			break;
		case 8:
			tcflagNData = CS8;
			break;
		default:
			return false;
			break;
	}

	m_cflags = m_cflags | tcflagBaud | tcflagNData;

	return true;
}

int Rs::open()
{
	struct termios tios;

	m_fd = ::open(m_fileName, O_RDWR | O_NOCTTY);
	if ( m_fd < 0)
	{
		m_fd = -1;
		return -1;
	}

	memset(&tios, 0, sizeof(struct termios));
	tcgetattr( m_fd, &tios );

	tios.c_cflag = m_cflags;
	tios.c_cflag &= ~(CRTSCTS);
	tios.c_iflag= IGNPAR & ~(IXON | IXOFF | IXANY);
	tios.c_oflag= ~OPOST;
	tios.c_lflag= ~( ICANON | ECHO | ECHOE | ISIG );

	// Time out inter caractere
	tios.c_cc[VTIME]= m_vTimeMs / 100;
	tios.c_cc[VMIN]= 1;
	tcflush(m_fd, TCIFLUSH);
	tcsetattr(m_fd,TCSANOW,&tios);
	return 0;
}

ssize_t Rs::read(void *buffer, int length)
{
	int readCount = 0;
	while( readCount != length )
	{
		int ret = ::read(m_fd, ((char*)buffer) + readCount, length - readCount);
		if( ret > 0)
		{
			readCount += ret;
		}
		else
		{
			return ret;
		}
	}
	return readCount;
}

ssize_t Rs::write(const void *buffer, int length)
{
	return ::write(m_fd, buffer, length);
}

int Rs::close()
{
	int res = -1;

	if( m_fd != -1)
	{
		res = ::close(m_fd);
	}

	return res;
}
