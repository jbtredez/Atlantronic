#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <math.h>
#include "kernel/hokuyo_tools.h"

const char* usart_name;
int fd = -1;
static uint16_t hokuyo_distance[682]; //!< distances des angles 44 à 725 du hokuyo
static float hokuyo_x[682]; //!< x des points 44 à 725
static float hokuyo_y[682]; //!< y des points 44 à 725

void usart_open()
{
	fd = open(usart_name, O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
		perror("open");
		return;
	}

	int flags = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

	struct termios options;
	tcgetattr(fd, &options);

	// CS8: 8 bits, no parity, 1 stopbit
	// CLOCAL: local connection (no modem control)
	// CREAD: activer reception	
	options.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
	options.c_cflag |= CS8 | CLOCAL | CREAD;
	options.c_iflag = 0;
	options.c_oflag = 0;
	options.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);
	options.c_cc[VTIME] = 0;		// tps d'attente des donnees: +infini
	options.c_cc[VMIN] = 1;		// min de caractères à lire avec read (marche pas si on met plus que 1...)

	cfsetospeed(&options, B19200);
	cfsetispeed(&options, B19200);

	tcsetattr(fd, TCSADRAIN, &options);
	tcdrain(fd);
	tcflush(fd, TCIOFLUSH);
}

void usart_set500k()
{
	struct termios options;
	tcgetattr(fd, &options);

	cfsetospeed(&options, B500000);
	cfsetispeed(&options, B500000);

	tcsetattr(fd, TCSADRAIN, &options);
//	tcdrain(fd);
//	tcflush(fd, TCIOFLUSH);
}

int main(int argc, char** argv)
{
	unsigned char buffer[10240];

	if(argc < 2)
	{
		printf("indiquer le peripherique\n");
		return -1;
	}

	usart_name = argv[1];

	usart_open();

	// TODO : tests, temps pour voir, a modifier
	write(fd, "SCIP2.0\nSS500000\n", 8+9);
	usleep(50000);
	usart_set500k();
	write(fd, "SCIP2.0\nSS500000\n", 8+9);
	write(fd, "BM\n", 3);

	usleep(150000);
	unsigned int size = read(fd, buffer, 10240);
	write(fd, "GS0044072500\n", 13);//44 à 725
	usleep(150000);

	unsigned int i = 0;
	size = read(fd, buffer, 10240);

	close(fd);

	if( size != 1432 )
	{
		printf("erreur de lecture, size == %i\n", size);
		return 0;
	}

	hokuyo_tools_decode_buffer(buffer, 1432, hokuyo_distance, 682);
	hokuyo_compute_xy(hokuyo_distance, 682, hokuyo_x, hokuyo_y);

	FILE* f = fopen("log/test_hokuyo.txt", "w");
	if(f == NULL)
	{
		perror("fopen");
		return 0;
	}
	
	for(i=0; i< 682; i++)
	{
		fprintf(f, "%f\t%f\n", hokuyo_x[i], hokuyo_y[i]);
	}

	fclose(f);

	FILE* p = popen("gnuplot --persist", "w");
	if(p == NULL)
	{
		perror("popen");
		return 0;
	}

	fprintf(p, "set term x11\n");
	fprintf(p, "set mouse\n");
	fprintf(p, "set xlabel \"y\"\n");
	fprintf(p, "plot \"log/test_hokuyo.txt\" using 2:1 t \"hokuyo\" with lines\n");
	fflush(p);

	// on ne ferme pas le programme tout de suite pour permettre de zoomer par exemple
	getchar();

	fclose(p);

	return 0;
}
