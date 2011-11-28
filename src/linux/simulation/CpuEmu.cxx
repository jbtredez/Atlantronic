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

void CpuEmu::connect_io(uint64_t base, uint64_t range, CpuIoInterface* interface)
{
	struct cpu_io io;

	if(base < PERIPH_BASE)
	{
		meslog(_erreur_, "base < PERIPH_BASE = %#"PRIx64, (uint64_t) PERIPH_BASE);
		return;	
	}

	base -= PERIPH_BASE;

	if(interface == NULL)
	{
		meslog(_erreur_, "interface == NULL");
		return;
	}

	if(range < sizeof(uint32_t))
	{
		meslog(_erreur_, "range < 4");
		return;
	}

	io.base = base;
	io.end_base = base + range - sizeof(uint32_t);
	io.interface = interface;

	cpu_io.push_back(io);
}

void CpuEmu::memory_write(uint64_t offset, uint32_t val)
{
	for(unsigned int i = 0; i < cpu_io.size() ; i++)
	{
		if(cpu_io[i].base <= offset && offset <= cpu_io[i].end_base)
		{
			cpu_io[i].interface->memory_write(offset - cpu_io[i].base, val);
			return;
		}
	}

	meslog(_erreur_, "ecriture non supporté - offset %#"PRIx64", val %"PRIx32, offset, val);
}

uint32_t CpuEmu::memory_read(uint64_t offset)
{
	for(unsigned int i = 0; i < cpu_io.size() ; i++)
	{
		if(cpu_io[i].base <= offset && offset <= cpu_io[i].end_base)
		{
			return cpu_io[i].interface->memory_read(offset - cpu_io[i].base);
		}
	}

	meslog(_erreur_, "lecture non supporté - offset %#"PRIx64, offset);

	return 0;
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
				memory_write(io.offset, io.val);
			}
			else if(io.cmd == READ_MEMORY)
			{
				io.val = memory_read( io.offset);
				//printf("read : %i, offset : %#.4lx val : %#.2x\n", io.cmd, io.offset, io.val);
			}
			else
			{
				printf("erreur protocole\n");
				return NULL;
			}

			write_irq();
			int res = write(fd_to_qemu, &io, sizeof(io));
			if(res < 0 )
			{
				logerror("write");
			}
			else if(res != sizeof(io))
			{
				meslog(_erreur_, "write too short (%i instead of %zd)", res, sizeof(io));
			}
		}
		else if(n < 0)
		{
			perror("read");
			return NULL;
		}
		else
		{
			printf("erreur protocole ( n = %d != %zd)\n", n, sizeof(io));
			return NULL;
		}
	}

	return NULL;
}

void CpuEmu::write_irq()
{
	struct atlantronic_memory_io io;
	io.cmd = IRQ;
	int res;

	while(! irq.empty())
	{
		io.val = irq.front();
		res = write(fd_to_qemu, &io, sizeof(io));
		if(res < 0 )
		{
			logerror("write_irq");
		}
		else if(res != sizeof(io))
		{
			meslog(_erreur_, "write too short (%i instead of %zd)", res, sizeof(io));
		}
		irq.pop_front();
	}
}

void CpuEmu::set_it(uint32_t it)
{
	irq.push_back(it);
}

