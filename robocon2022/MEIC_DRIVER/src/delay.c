#include "delay.h"    								   
void delay_us(u32 nus)
{		
   u16 i=0;  
   while(nus--)
   {
      i=23;  
      while(i--) ;    
   }
}
void delay_ms(u16 nms)
{		
u16 i=0;  
   while(nms--)
   {
      i=28000;  
      while(i--) ;    
   }
}

