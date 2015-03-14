#include <string.h>
#include <stdlib.h>

#include "com_xbee.h"

ComXbee::ComXbee(const char* fileName) :
	m_rxData(65536)
{
	m_rs.configure(fileName, 115200, 8, 1);
}

ComXbee::~ComXbee()
{

}

int ComXbee::open()
{
	close();
	Com::open();

	int res = m_rs.open();
	if( res )
	{
		return -1;
	}

	res = configure(XBEE_AT_NETWORK_ID, XBEE_NETWORK_ID);
	if( res )
	{
		return -1;
	}

	log_info("open xbee");

	return 0;
}

// TODO mettre en commun avec code xbee de la carte
uint32_t ComXbee::configure(uint16_t at_cmd, uint32_t val)
{
	uint16_t api_specific_size = 8;

	log_info("xbee AT %c%c : %.4x", at_cmd>>8, at_cmd&0xff, (unsigned int)val);

	txBuffer[0] = 0x7e;
	txBuffer[1] = (api_specific_size >> 8) & 0xff;
	txBuffer[2] = api_specific_size & 0xff;
	txBuffer[3] = XBEE_CMD_AT;
	txBuffer[4] = 0x01; // id
	txBuffer[5] = ( at_cmd >> 8 ) & 0xff;
	txBuffer[6] = ( at_cmd & 0xff);
	txBuffer[7] = (val >> 24) & 0xff;
	txBuffer[8] = (val >> 16) & 0xff;
	txBuffer[9] = (val >> 8) & 0xff;
	txBuffer[10] = val & 0xff;

	// TODO gestion escaped character pour API mode 2 (API mode 1 pour le moment)
	uint8_t checksum = 0;
	for(int i = 3; i < api_specific_size + 3; i++)
	{
		checksum += txBuffer[i];
	}

	txBuffer[api_specific_size + 3] = 0xff - checksum;

	m_rs.write(txBuffer, api_specific_size + 4);
	int ret = m_rs.read(rxBuffer, 9);
	if( ret != 9 )
	{
		log_error("xbee - read error %d errno %d", ret, errno);
		return -1;
	}

	if( rxBuffer[0] != 0x7e || rxBuffer[1] != 0 ||
		rxBuffer[2] != 5 || rxBuffer[3] != 0x88 ||
		rxBuffer[4] != txBuffer[4] ||
		rxBuffer[5] != txBuffer[5] ||
		rxBuffer[6] != txBuffer[6] )
	{
		// erreur de protocole
		log_error("xbee protocol error");
		return -1;

	}

	int res = -1;
	// status de la commande
	switch( rxBuffer[7] )
	{
		case 0:
			// OK
			res = 0;
			break;
		default:
		case 1:
			log_error("xbee at error");
			res = 0;// TODO
			break;
		case 2:
			log_error("xbee invalid at command");
			break;
		case 3:
			log_error("xbee invalid at parameters");
			break;
	}

	return res;
}

int ComXbee::close()
{
	int res = m_rs.close();
	if( res == 0 )
	{
		log_info("close xbee");
	}
	opened = false;
	return 0;
}

int ComXbee::write(const void* /*buf*/, int size)
{
	// TODO
	return size;
}

int ComXbee::read(void* buf, int size)
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
	}

	return 0;
}

void ComXbee::decodeApiFrame(int frameSize)
{
	switch(rxBuffer[3])
	{
		case XBEE_CMD_TX | XBEE_CMD_RX_MASK:
			decodeApiDataFrame(frameSize);
			break;
		default:
			// TODO
			break;
	}
}

void ComXbee::decodeApiDataFrame(int frameSize)
{
	if( frameSize < 12)
	{
		log_error("frame error");
		return;
	}

	int res = m_rxData.push(&rxBuffer[15], frameSize - 12);
	if( res )
	{
		log_error("rxData buffer full");
	}
}
