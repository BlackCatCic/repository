#include "Driver_USART.h"

#if EN_PRINTF == 1   /*非常讨厌这种阻塞式发送方式*/
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       

void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);
	USART3->DR = (u8) ch;      
	return ch;
}
#else


#endif 

