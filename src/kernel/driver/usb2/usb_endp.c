#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "usb_istr.h"
#include "usb_pwr.h"

void EP1_IN_Callback(void)
{
	// TODO test
	extern volatile unsigned int int_rdy;
	int_rdy = 1;
}

void EP2_IN_Callback(void)
{

}

void INTR_SOFINTR_Callback(void)
{
	if(bDeviceState == CONFIGURED)
	{
		/* Check the data to be sent through IN pipe */
		//      Handle_USBAsynchXfer();
	}
}
