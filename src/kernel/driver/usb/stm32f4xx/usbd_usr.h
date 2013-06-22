#ifndef __USBD_USR_H__
#define __USBD_USR_H__

#include "usbd_core.h"

extern  USBD_Usr_cb_TypeDef USR_cb;
extern  USBD_Usr_cb_TypeDef USR_FS_cb;
extern  USBD_Usr_cb_TypeDef USR_HS_cb;

void     USBD_USR_Init(void);
void     USBD_USR_DeviceReset (uint8_t speed);
void     USBD_USR_DeviceConfigured (void);
void     USBD_USR_DeviceSuspended(void);
void     USBD_USR_DeviceResumed(void);

void     USBD_USR_DeviceConnected(void);
void     USBD_USR_DeviceDisconnected(void); 

void     USBD_USR_FS_Init(void);
void     USBD_USR_FS_DeviceReset (uint8_t speed);
void     USBD_USR_FS_DeviceConfigured (void);
void     USBD_USR_FS_DeviceSuspended(void);
void     USBD_USR_FS_DeviceResumed(void);

void     USBD_USR_FS_DeviceConnected(void);
void     USBD_USR_FS_DeviceDisconnected(void);  

void     USBD_USR_HS_Init(void);
void     USBD_USR_HS_DeviceReset (uint8_t speed);
void     USBD_USR_HS_DeviceConfigured (void);
void     USBD_USR_HS_DeviceSuspended(void);
void     USBD_USR_HS_DeviceResumed(void);

void     USBD_USR_HS_DeviceConnected(void);
void     USBD_USR_HS_DeviceDisconnected(void);  


#endif /*__USBD_USR_H__*/
