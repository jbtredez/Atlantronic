#ifndef SERVER_UDP_H
#define SERVER_UDP_H

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>
#include <list>
#include "linux/tools/com/com.h"

class ServerUdp;

struct ServerClientUdp
{
	public:
		int socket;
		int id;
		Com* com;
		ServerUdp* server;
		char buffer[1024];

		static void* task_wrapper(void* arg);
		void task();
		volatile bool stopTask;
};

class ServerUdp
{
	public:
		ServerUdp();
		void configure(Com* com, int port);
		bool start();
		void stop();

		static void* task_wrapper(void* arg);
		void write(void* buffer, unsigned int size);
		void deleteClient(ServerClientUdp* client);

	protected:
		void task();

		bool started;
		int socketFd;
		int clientId;
		volatile bool stopTask;
		pthread_t tid;
		std::list<ServerClientUdp*> clientList;
		Com* com;
		int port;
};

#endif
