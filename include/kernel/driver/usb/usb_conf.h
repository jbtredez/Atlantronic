#ifndef __USB_CONF_H
#define __USB_CONF_H

//! @file usb_conf.c
//! @brief USB configuration
//! @author Atlantronic

/*-------------------------------------------------------------*/
/* EP_NUM */
/* defines how many endpoints are used by the device */
/*-------------------------------------------------------------*/
#define EP_NUM                           3

#ifdef STM32F10X_CL
/*******************************************************************************
*                              FIFO Size Configuration
*
*  (i) Dedicated data FIFO SPRAM of 1.25 Kbytes = 1280 bytes = 320 32-bits words
*      available for the endpoints IN and OUT.
*      Device mode features:
*      -1 bidirectional CTRL EP 0
*      -3 IN EPs to support any kind of Bulk, Interrupt or Isochronous transfer
*      -3 OUT EPs to support any kind of Bulk, Interrupt or Isochronous transfer
*
*  ii) Receive data FIFO size = RAM for setup packets +
*                   OUT endpoint control information +
*                   data OUT packets + miscellaneous
*      Space = ONE 32-bits words
*     --> RAM for setup packets = 4 * n + 6 space
*        (n is the nbr of CTRL EPs the device core supports)
*     --> OUT EP CTRL info      = 1 space
*        (one space for status information written to the FIFO along with each
*        received packet)
*     --> data OUT packets      = (Largest Packet Size / 4) + 1 spaces
*        (MINIMUM to receive packets)
*     --> OR data OUT packets  = at least 2*(Largest Packet Size / 4) + 1 spaces
*        (if high-bandwidth EP is enabled or multiple isochronous EPs)
*     --> miscellaneous = 1 space per OUT EP
*        (one space for transfer complete status information also pushed to the
*        FIFO with each endpoint's last packet)
*
*  (iii)MINIMUM RAM space required for each IN EP Tx FIFO = MAX packet size for
*       that particular IN EP. More space allocated in the IN EP Tx FIFO results
*       in a better performance on the USB and can hide latencies on the AHB.
*
*  (iv) TXn min size = 16 words. (n  : Transmit FIFO index)
*   (v) When a TxFIFO is not used, the Configuration should be as follows:
*       case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
*       --> Txm can use the space allocated for Txn.
*       case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
*       --> Txn should be configured with the minimum space of 16 words
*  (vi) The FIFO is used optimally when used TxFIFOs are allocated in the top
*       of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
*******************************************************************************/

#define RX_FIFO_SIZE                          128
#define TX0_FIFO_SIZE                          64
#define TX1_FIFO_SIZE                          64
#define TX2_FIFO_SIZE                          64
#define TX3_FIFO_SIZE                          64

/* OTGD-FS-DEVICE IP interrupts Enable definitions */
/* Uncomment the define to enable the selected interrupt */
//#define INTR_MODEMISMATCH
#define INTR_SOFINTR
#define INTR_RXSTSQLVL           /* Mandatory */
//#define INTR_NPTXFEMPTY
//#define INTR_GINNAKEFF
//#define INTR_GOUTNAKEFF
//#define INTR_ERLYSUSPEND
#define INTR_USBSUSPEND          /* Mandatory */
#define INTR_USBRESET            /* Mandatory */
#define INTR_ENUMDONE            /* Mandatory */
//#define INTR_ISOOUTDROP
//#define INTR_EOPFRAME
//#define INTR_EPMISMATCH
#define INTR_INEPINTR            /* Mandatory */
#define INTR_OUTEPINTR           /* Mandatory */
//#define INTR_INCOMPLISOIN
//#define INTR_INCOMPLISOOUT
#define INTR_WKUPINTR            /* Mandatory */

/* Isochronous transfer parameters */
/* Size of a single Isochronous buffer (size of a single transfer) */
#define ISOC_BUFFER_SZE                  1
/* Number of sub-buffers (number of single buffers/transfers), should be even */
#define NUM_SUB_BUFFERS                  2

#endif /* STM32F10X_CL */

#endif /* __USB_CONF_H */
