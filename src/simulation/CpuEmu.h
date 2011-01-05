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

#define ACK               1
#define WRITE_MEMORY      2
#define READ_MEMORY       3

struct atlantronic_memory_io
{
	uint8_t cmd;
	uint64_t vm_clk;
	uint64_t offset;
	uint32_t val;
};

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
	int fd_to_qemu;
	int fd_to_simu;
	int fd_ctrl;
	pthread_t id;
	int qemu_pid;
	void* lecture();
	static void* lecture(void* arg);
};

#endif
