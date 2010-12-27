#include "CpuEmu.h"

CpuEmu::CpuEmu()
{
	fd_to_qemu = -1;
	fd_to_simu = -1;
	id = 0;
}

CpuEmu::~CpuEmu()
{

}

void CpuEmu::start(const char* pipe_name, const char* prog)
{
	if(id == 0)
	{
		char pipe_name_to_qemu[1024];
		char pipe_name_to_simu[1024];

		snprintf(pipe_name_to_qemu, sizeof(pipe_name_to_qemu), "%s_to_qemu", pipe_name);
		snprintf(pipe_name_to_simu, sizeof(pipe_name_to_simu), "%s_to_simu", pipe_name);

		mkfifo(pipe_name_to_qemu, 0666);
		mkfifo(pipe_name_to_simu, 0666);

		int pid = fork();

		if(pid == 0)
		{
			fclose(stdin);

			char* arg[ ] =
			{
				(char*) "qemu/arm-softmmu/qemu-system-arm",
				(char*) "-M",
				(char*) "atlantronic",
				(char*) "-nographic",
				(char*) "-pipe",
				(char*) pipe_name,
				(char*) "-kernel",
				(char*) prog,
				NULL
			};

			execv("qemu/arm-softmmu/qemu-system-arm", arg);
		}
		else if(pid > 0)
		{
			fd_to_qemu = open(pipe_name_to_qemu, O_WRONLY);
			fd_to_simu = open(pipe_name_to_simu, O_RDONLY);

			id = 0;
		}
		pthread_create(&id, NULL, lecture, this);
	}
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

		update_hardware( io.vm_clk );

		if( n == sizeof(io))
		{
			if(io.cmd == WRITE_MEMORY)
			{
				write(fd_to_qemu, &ack, sizeof(ack));
				//printf("commande : %i, offset : %#.4lx val : %#.2x\n", io.cmd, io.offset, io.val);
				mem_write(io.offset, io.val);
			}
			else if(io.cmd == READ_MEMORY)
			{
				io.val = mem_read( io.offset);
				//printf("read : %i, offset : %#.4lx val : %#.2x\n", io.cmd, io.offset, io.val);
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
