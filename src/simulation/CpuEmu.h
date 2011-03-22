#ifndef CPU_EMU_H
#define CPU_EMU_H

#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <signal.h>
#include <stdlib.h>
#include "log.h"
#include <list>
#include <vector>
#include "cpu_io_interface.h"

struct cpu_io
{
	uint64_t base;
	uint64_t end_base;
	CpuIoInterface* interface;
};

class CpuEmu
{
public:
	CpuEmu();
	virtual ~CpuEmu();
	void start(const char* pipe_name, const char* prog, int gdb_port);
	void stop();
	void set_it(uint32_t it);
	void connect_io(uint64_t base, uint64_t range, CpuIoInterface* interface);

protected:
	void memory_write(uint64_t offset, uint32_t val);
	uint32_t memory_read(uint64_t offset);
	virtual void update_hardware(uint64_t vm_clk) = 0;
	std::vector<struct cpu_io> cpu_io;

private:
	void* lecture();
	static void* lecture(void* arg);
	void write_irq();

	int fd_to_qemu;
	int fd_to_simu;
	std::list<uint32_t> irq;
	pthread_t id;
	int qemu_pid;
};

#endif
