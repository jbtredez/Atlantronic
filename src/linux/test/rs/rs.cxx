#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int main()
{
	printf("open\n");
	int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	if( fd < 0 )
	{
		fprintf(stderr, "open : %s\n", strerror(errno));
		return -1;
	}

	struct termios s_termios;
	memset(&s_termios, 0, sizeof(struct termios));
	tcgetattr( fd, &s_termios );


	s_termios.c_cflag = CREAD | CLOCAL | CS8 | B500000;
	s_termios.c_cflag &= ~(CRTSCTS);
	s_termios.c_iflag= IGNPAR & ~(IXON | IXOFF | IXANY);
	s_termios.c_oflag= ~OPOST;
	s_termios.c_lflag= ~( ICANON | ECHO | ECHOE | ISIG );
	s_termios.c_cc[VMIN]= 0;
	tcflush(fd,TCIFLUSH);
	tcsetattr(fd,TCSANOW,&s_termios);

	char buffer[4096];
	int offset = 0;
	while(1)
	{
		int size = read(fd, buffer + offset, sizeof(buffer) - offset);
		if( size > 0)
		{
			if(buffer[offset + size - 1] == '\n')
			{
				buffer[offset + size] = 0;
				printf("%s", buffer);
				offset = 0;
			}
			else
			{
				offset += size;
			}
		}
	}

	close(fd);

	return 0;
}
