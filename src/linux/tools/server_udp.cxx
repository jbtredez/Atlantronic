#include "server_udp.h"
#include "linux/tools/cli.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

ServerUdp::ServerUdp()
{
	started = false;
}

void ServerUdp::configure(Com* Com, int Port)
{
	com = Com;
	port = Port;
}

bool ServerUdp::start()
{
	struct sockaddr_in addr;
	clientId = 0;

	socketFd = socket(PF_INET, SOCK_STREAM, IPPROTO_UDP);
	if( socketFd < 0)
	{
		log_error_errno("socketFd");
		return false;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port);

	int res = bind(socketFd, (struct sockaddr *) &addr, sizeof(addr));
	if( res < 0)
	{
		log_error_errno("bind");
		return false;
	}

	stopTask = 0;
	res = pthread_create(&tid, NULL, ServerUdp::task_wrapper, this);
	if( res < 0 )
	{
		log_error_errno("pthread_create");
		return false;
	}

	started = true;

	return true;
}

void* ServerUdp::task_wrapper(void* arg)
{
	ServerUdp* server = (ServerUdp*) arg;
	server->task();
	return NULL;
}

void ServerUdp::task()
{
	struct sockaddr_in addr;
	socklen_t size = sizeof(addr);

	while( ! stopTask )
	{
		int res = listen(socketFd, 5);
		if( res < 0)
		{
			log_error_errno("pthread_create");
			continue;
		}

		int clientSocket = accept(socketFd, (struct sockaddr *) &addr, &size);
		if( clientSocket < 0 )
		{
			log_error_errno("accept");
			continue;
		}

		log_info("nouveau client tcp %d : %s", clientId, inet_ntoa(addr.sin_addr));
		ServerClientUdp* client = new ServerClientUdp;
		client->socket = clientSocket;
		client->stopTask = false;
		client->id = clientId;
		client->com = com;
		client->server = this;

		res = pthread_create(&tid, NULL, ServerClientUdp::task_wrapper, client);
		if( res < 0 )
		{
			log_error_errno("pthread_create");
			continue;
		}

		clientId++;
		clientList.push_back(client);
	}
}

void ServerUdp::stop()
{
	if( started )
	{
		stopTask = 1;
		std::list<ServerClientUdp*>::iterator it;
		for(it = clientList.begin(); it != clientList.end(); ++it)
		{
			(*it)->stopTask = true;
		}
		usleep(100000);
		pthread_cancel(tid);
	}
}

void ServerUdp::deleteClient(ServerClientUdp* client)
{
	std::list<ServerClientUdp*>::iterator it;
	for(it = clientList.begin(); it != clientList.end(); ++it)
	{
		if( (*it) == client )
		{
			clientList.erase(it);
		}
		break;
	}
}

void ServerUdp::write(void* buffer, unsigned int size)
{
	std::list<ServerClientUdp*>::iterator it;
	for(it = clientList.begin(); it != clientList.end(); ++it)
	{
		int res = ::write((*it)->socket, buffer, size);
		if( res < 0)
		{
			log_error_errno("write");
		}
	}
}

void* ServerClientUdp::task_wrapper(void* arg)
{
	ServerClientUdp* client = (ServerClientUdp*) arg;
	client->task();
	client->server->deleteClient(client);
	return NULL;
}

void ServerClientUdp::task()
{
	while( ! stopTask )
	{
		int res = read(socket, buffer, sizeof(buffer));
		if( res > 0)
		{
			com->write(buffer, res);
		}
		else if( res == 0)
		{
			stopTask = 1;
		}
		else if( res < 0)
		{
			stopTask = 1;
			log_error_errno("read");
		}
	}

	log_info("client %d disconnected", id);
	close(socket);
}

