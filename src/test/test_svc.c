//! @file svc.c
//! @brief Test bug svc
//! @author Jean-Baptiste Trédez

void vPortSVCHandler( void ) __attribute(( naked ));
void xPortPendSVHandler( void ) __attribute__ (( naked ));

void vPortSVCHandler( void )
{

}

void xPortPendSVHandler( void )
{

}

void xPortSysTickHandler( void )
{

}

int main()
{
	__asm volatile(" svc 0\n");

	while(1) ;

	return 0;
}

