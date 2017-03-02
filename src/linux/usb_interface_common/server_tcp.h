#ifndef SERVER_TCP_H
#define SERVER_TCP_H

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>
#include <list>
#include "linux/usb_interface_common/com/com.h"

class ServerTcp;

struct ServerClient
{
	public:
		int socket;
		int id;
		Com* com;
		ServerTcp* server;
		char buffer[1024];

		static void* task_wrapper(void* arg);
		void task();
		volatile bool stopTask;
};

class ServerTcp
{
	public:
		ServerTcp();
		void configure(Com* com, int port);
		bool start();
		void stop();

		static void* task_wrapper(void* arg);
		void write(void* buffer, unsigned int size);
		void deleteClient(ServerClient* client);

	protected:
		void task();

		bool started;
		int socketFd;
		int clientId;
		volatile bool stopTask;
		pthread_t tid;
		std::list<ServerClient*> clientList;
		Com* com;
		int port;
};

#endif
