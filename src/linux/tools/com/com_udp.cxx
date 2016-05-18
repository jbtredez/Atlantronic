#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "linux/tools/server_udp.h"
ComUdp::ComUdp(ServerUdp *Serveur,const char * ip):
	m_rxData(65536)
{
	m_pServeur = Serveur;
	buffer_end = 0;
	buffer_begin = 0;
	buffer_size = 0;
	m_pServeur->createclient(this,ip);
}

ComUdp::~ComUdp()
{

}


int ComUdp::open()
{
	int res = 0;

	close();
	Com::open();
	return res;
}
int ComUdp::write(const void* buf, int size)
{
	///TODO
	if(!m_pServeur )
	{
		return -1;
	}
	if( buf && size)
	{
		return 1;
	}
	return 0;
	//sockaddr_in addr;
	//m_Serveur->write(buffer,size,addr);

}
///Sauvegarde les données dans com
int ComUdp::read(void* buf, int size)
{
	while( size > 0)
	{
		// on prend les donnees dans rxData
		int nMax = m_rxData.m_count;
		if( nMax >= size )
		{
			nMax = size;
		}

		if( nMax > 0 )
		{

			m_rxData.pop((unsigned char*)buf, nMax);
			return nMax;
		}
/*
		if( size > 0)
		{
			// TODO rendre robuste avec buffer circulaire (modifier rs pour qu'il utilise la classe com ?)
			// on n'a pas assez de donnees, on va en lire
			int ret = m_rs.read(rxBuffer, 3);
			if( rxBuffer[0] == 0x7e )
			{
				// trame recue
				uint16_t api_specific_size = (((int)rxBuffer[1]) << 8) + rxBuffer[2];
				ret = m_rs.read(rxBuffer+3, api_specific_size+1);
				if( ret > 0 )
				{
					decodeApiFrame(api_specific_size);
				}
			}
		}
		*/
		return 0;
	}
	return 0;
}


void ComUdp::save(unsigned char * msg,int size )
{

	int res = m_rxData.push(msg, size);
	if( res )
	{
		log_error("rxData buffer full");
	}
}
