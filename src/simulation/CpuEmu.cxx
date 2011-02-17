#include "CpuEmu.h"

#define IRQ               1
#define WRITE_MEMORY      2
#define READ_MEMORY       3

struct atlantronic_memory_io
{
	uint8_t cmd;
	uint64_t vm_clk;
	uint64_t offset;
	uint32_t val;
	uint32_t it;
};

CpuEmu::CpuEmu()
{
	fd_to_qemu = -1;
	fd_to_simu = -1;
	id = 0;
}

CpuEmu::~CpuEmu()
{

}

void CpuEmu::start(const char* pipe_name, const char* prog, int gdb_port)
{
	if(id == 0)
	{
		char pipe_name_to_qemu[1024];
		char pipe_name_to_simu[1024];

		snprintf(pipe_name_to_qemu, sizeof(pipe_name_to_qemu), "%s_to_qemu", pipe_name);
		snprintf(pipe_name_to_simu, sizeof(pipe_name_to_simu), "%s_to_simu", pipe_name);

		mkfifo(pipe_name_to_qemu, 0666);
		mkfifo(pipe_name_to_simu, 0666);

		qemu_pid = fork();

		if(qemu_pid == 0)
		{
			fclose(stdin);

			char* arg[12];
			char buf_tcp[64];

			arg[0] = (char*) "qemu/arm-softmmu/qemu-system-arm";
			arg[1] = (char*) "-M";
			arg[2] = (char*) "atlantronic";
			arg[3] = (char*) "-nographic";
			arg[4] = (char*) "-pipe";
			arg[5] = (char*) pipe_name;
			arg[6] = (char*) "-kernel";
			arg[7] = (char*) prog;
			if(gdb_port)
			{
				arg[8] = (char*) "-S";
				arg[9] = (char*) "-gdb";
				snprintf(buf_tcp, sizeof(buf_tcp), "tcp::%i", gdb_port);
				arg[10] = buf_tcp;
				arg[11] = NULL;
			}
			else
			{
				arg[8] = NULL;
			}

			execv(arg[0], arg);
			logerror("execv");
			exit(-1);
		}
		else if(qemu_pid > 0)
		{
			fd_to_qemu = open(pipe_name_to_qemu, O_WRONLY);
			fd_to_simu = open(pipe_name_to_simu, O_RDONLY);

			pthread_create(&id, NULL, lecture, this);
		}
		else
		{
			logerror("fork");
		}
	}
}

void CpuEmu::stop()
{
	if(id)
	{
		pthread_cancel(id);
		void* ret;
		pthread_join(id, &ret);
		// TODO un peu bourrin, faire mieux
		kill(qemu_pid, SIGILL);
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

	while(1)
	{
		n = read(fd_to_simu, &io, sizeof(io) );

		update_hardware( io.vm_clk );

		if( n == sizeof(io))
		{
			if(io.cmd == WRITE_MEMORY)
			{
				//printf("commande : %i, offset : %#.4lx val : %#.2x\n", io.cmd, io.offset, io.val);
				mem_write(io.offset, io.val);
			}
			else if(io.cmd == READ_MEMORY)
			{
				io.val = mem_read( io.offset);
				//printf("read : %i, offset : %#.4lx val : %#.2x\n", io.cmd, io.offset, io.val);
			}
			else
			{
				printf("erreur protocole\n");
				return NULL;
			}

			write_irq();
			write(fd_to_qemu, &io, sizeof(io));
		}
		else if(n < 0)
		{
			perror("read");
			return NULL;
		}
		else
		{
			printf("erreur protocole ( n = %d != %lu)\n", n, sizeof(io));
			return NULL;
		}
	}

	return NULL;
}

void CpuEmu::write_irq()
{
	struct atlantronic_memory_io io;
	io.cmd = IRQ;

	while(! irq.empty())
	{
		io.val = irq.front();
		write(fd_to_qemu, &io, sizeof(io));
		irq.pop_front();
	}
}

void CpuEmu::set_it(uint32_t it)
{
	irq.push_back(it);
}

