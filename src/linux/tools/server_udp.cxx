#include "linux/tools/cli.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "server_udp.h"

ServerUdp::ServerUdp()
{
	started = false;

	log_error("Création serveur UDP");
}

void ServerUdp::configure( int Port)
{
	port = Port;
}

void ServerUdp::createclient( Com * pcom, const char* ip)
{
	ServerClientUdp* client = new ServerClientUdp;
	inet_pton(AF_INET, ip, &(client->socket_in.sin_addr));
	client->id = clientId;
	client->com = pcom;
	client->server = this;
	clientId++;
	clientList.push_back(client);
}

bool ServerUdp::start()
{
	struct sockaddr_in addr;
	clientId = 0;

	socketFd = socket(AF_INET,SOCK_DGRAM, IPPROTO_UDP);

	log_error("Création socket UDP");
	if( socketFd < 0)
	{
		log_error_errno("socketFd");
		return false;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port);

	log_error("Création Bind");
	int res = bind(socketFd, (struct sockaddr *) &addr, sizeof(addr));
	if( res < 0)
	{
		log_error_errno("bind");
		return false;
	}

	log_error("Création pthread_create");
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

	while( ! stopTask )
	{

		sockaddr from ;
		int fromsize = sizeof from;
		int res;


		if((res = recvfrom(socketFd, buffer, sizeof(buffer), 0,&from, (socklen_t*)&fromsize)) < 0)
		{
			log_error_errno("recvfrom");
		}
		else if( res < 0)
		{
			stopTask = 1;
			log_error_errno("read");
		}
		if( res > 0)
		{

			int ClientLiseSize = clientList.size();
			sockaddr_in *sin = ( sockaddr_in *) &from;
			for(int i =0; i < ClientLiseSize; ++i)
			{

				if (from.sa_family == AF_INET)
				{
					sockaddr_in * clientTP = &(clientList[i]->socket_in);

					if(clientTP->sin_port == sin->sin_port)
					{
						ComUdp *comudp = (ComUdp *) clientList[i]->com;

						usb_header *  header =(usb_header *) buffer;

						comudp->save(buffer, header->size +sizeof(usb_header));
					}

				}

			}
		}
	}
}

void ServerUdp::stop()
{
	if( started )
	{
		stopTask = 1;
		std::vector<ServerClientUdp*>::iterator it;
		for(it = clientList.begin(); it != clientList.end(); ++it)
		{
			(*it)->stopTask = true;
		}
		usleep(100000);
		pthread_cancel(tid);
	}
}


void ServerUdp::write(void* buffer, unsigned int size,sockaddr_in addr)
{
	std::vector<ServerClientUdp*>::iterator it;
	for(it = clientList.begin(); it != clientList.end(); ++it)
	{
		int res = ::write((*it)->socket_in.sin_port, buffer, size);
		if( res < 0)
		{
			log_error_errno("write");
		}
	}
}
