#ifndef SERVER_UDP_H
#define SERVER_UDP_H
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>
#include <vector>
#include "linux/tools/com/com.h"
#include "linux/tools/com/com_udp.h"


class ServerUdp;
struct ServerClientUdp
{
	public:
		int id;
		Com* com;
		ServerUdp* server;
		unsigned char buffer[1024];
		sockaddr_in socket_in;
		static void* task_wrapper(void* arg);
		void task();
		volatile bool stopTask;
};

class ServerUdp
{
	public:
		ServerUdp();
		void configure( int port);
		bool start();
		void stop();

		void createclient( Com * pcom,const char* ip);
		static void* task_wrapper(void* arg);
		void write(void* buffer, unsigned int size,	sockaddr_in addr);
	//	void deleteClient(ServerClientUdp* client);

	protected:
		void task();
		unsigned char buffer[1024];
		bool started;
		int socketFd;
		int clientId;
		volatile bool stopTask;
		pthread_t tid;
		std::vector<ServerClientUdp*> clientList;
		int port;
};

#endif
