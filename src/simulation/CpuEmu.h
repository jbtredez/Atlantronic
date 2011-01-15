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

class CpuEmu
{
public:
	CpuEmu();
	virtual ~CpuEmu();
	void start(const char* pipe_name, const char* prog, int gdb_port);
	void stop();
	void set_it(uint32_t it);

protected:
	virtual void mem_write(uint64_t offset, uint32_t val) = 0;
	virtual uint32_t mem_read(uint64_t offset) = 0;
	virtual void update_hardware(uint64_t vm_clk) = 0;

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
