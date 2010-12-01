#include "CpuEmu.h"

CpuEmu::CpuEmu()
{
	mkfifo("/tmp/to_qemu", 0666);
	mkfifo("/tmp/to_simu", 0666);

	fd_to_qemu = open("/tmp/to_qemu", O_WRONLY);
	fd_to_simu = open("/tmp/to_simu", O_RDONLY);

	pthread_create(&id, NULL, lecture, this);
}

CpuEmu::~CpuEmu()
{

}

void* CpuEmu::lecture(void* arg)
{
	CpuEmu* cpu = (CpuEmu*) arg;
	return  cpu->lecture();
}

void* CpuEmu::lecture()
{
	struct atlantronic_memory_io io;
	int n;
	uint8_t ack = ACK;

	while(1)
	{
		n = read(fd_to_simu, &io, sizeof(io) );
		if( n == sizeof(io))
		{
			if(io.cmd == WRITE_MEMORY)
			{
				write(fd_to_qemu, &ack, sizeof(ack));
				printf("commande : %i, offset : %#.4lx val : %#.2x\n", io.cmd, io.offset, io.val);
				mem_write(io.offset, io.val);
			}
			else if(io.cmd == READ_MEMORY)
			{
				io.val = mem_read(io.offset);
				write(fd_to_qemu, &io, sizeof(io));
			}
			else
			{
				printf("erreur protocole\n");
				return NULL;
			}
		}
		else if(n < 0)
		{
			perror("read");
			return NULL;
		}
		else
		{
			printf("erreur protocole\n");
			return NULL;
		}
	}

	return NULL;
}
