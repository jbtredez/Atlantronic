#ifndef ESP8266_H
#define ESP8266_H

//! @file esp8266.h
//! @brief ESP8266 module
//! @author Atlantronic

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEAK_ESP8266
#define WEAK_ESP8266 __attribute__((weak, alias("nop_function") ))
#endif

#define ESP8266_CMD_SPI_WRITE           0x02
#define ESP8266_CMD_SPI_READ			0x03      //!< commande SPI
#define ESP8266_CMD_SPI_STATUS			0x04      //!< commande SPI
#define ESP8266_CMD_SPI_STATUS2			0x05      //!< commande SPI

#define ESP8266_CMD_SPI_ADDR_DATA		0x00

#define ESP8266_STATUS_WRITE			0x00	//Status du Buffer d'écriture du Master 0 Full,1 Empty
#define ESP8266_STATUS_READ				0x01	//Status du Buffer de lecture du Master 0 something,1 Empty

#define ESP8266_MSG_SIZE_MAX			256

struct esp8266_cmd_param
{
	uint8_t Id;			//ID de la commande
	uint8_t Size;			//taille des données
	uint8_t Data[ESP8266_MSG_SIZE_MAX];		//Données

}__attribute((packed));


typedef enum
{
	ESP8266_STATUS_DISCONNECTED = 0,
	ESP8266_STATUS_CONNECTED,
} ESP8266Status;


//CMD Data
#define  ESP8266_CMD_DATA	 0x00 //Donnees Transmisent avec une taille MAX de 256

// CMD Definit dans le mode de la puce (STATION OU/ET AP)
#define  ESP8266_CM_MODE	 0x01

//Données possibles
#define  ESP8266_DT_AP	 	0x00
#define  ESP8266_DT_STA	 	0x01
#define  ESP8266_DT_STAAP	0x02

///CMD Pour le mode STATION
#define  ESP8266_CM_STASSID 	0x10
#define  ESP8266_CM_STAPWD  	0x11
#define  ESP8266_CM_STADEVNAME	0x12
#define  ESP8266_CM_STAEN   	0x13

//CMD pour le mode AP
#define  ESP8266_CM_APSSID 	0x20
#define  ESP8266_CM_APPWD  	0x21
#define  ESP8266_CM_APEN   	0x22


//Defenition des port TCP ou UDP
// Acitvation du serveur TCP et selection du port TCP
#define  ESP8266_CM_TCPEN 	0x30
#define  ESP8266_CM_TCPPORT	0x31  //Donnée du port sur 2 octets

// Acitvation du serveur UDP et selection du port UDP
#define  ESP8266_CM_UDPEN 	0x40
#define  ESP8266_CM_UDPPORT	0x41  //Donnée du port sur 2 octets


#define  ESP8266_CM_RST		0xF0
#define  ESP8266_CM_NOCMD	0xFF


void esp8266_add(uint16_t type, void* msg, uint16_t size) WEAK_ESP8266;

void esp82666_add_log(unsigned char level, const char* func, uint16_t line, const char* msg) WEAK_ESP8266;

#ifdef __cplusplus
}
#endif

#endif
