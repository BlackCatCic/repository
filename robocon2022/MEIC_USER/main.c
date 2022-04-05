#include "Include.h"
struct rt_timer timer1,timer2,timer3,timer4;
rt_thread_t tid = RT_NULL;
int main(void)
{	
	rt_thread_mdelay(2000);
  /* 创建线程 1 */
  tid = rt_thread_create("thread1",      //线程名字
                          thread_entry1, //线程入口函数
	                        RT_NULL,       //线程入口参数
                          1024,           //线程栈大小
                          9,             //线程优先级
													5);            //线程时间片
  if (tid != RT_NULL)
      rt_thread_startup(tid);
	
  /* 创建线程 2 */
  tid = rt_thread_create("thread2",      //线程名字
                          thread_entry2, //线程入口函数
	                        RT_NULL,       //线程入口参数
                          1024,           //线程栈大小
                          8,             //线程优先级
													5);            //线程时间片
  if (tid != RT_NULL)
      rt_thread_startup(tid);

  /* 创建线程 2 */
  tid = rt_thread_create("thread3",      //线程名字
                          thread_entry3, //线程入口函数
	                        RT_NULL,       //线程入口参数
                          1024,           //线程栈大小
                          7,             //线程优先级
													5);            //线程时间片
  if (tid != RT_NULL)
      rt_thread_startup(tid);
	
		rt_timer_init(&timer1, "timer1",         /* 定时器名字是timer1 */
                    timeout1,                /* 超时时回调的处理函数 */
                    RT_NULL,                 /* 超时函数的入口参数 */
										10,                     /* 定时长度,以OSTick为单位1ms*/
                    RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */
    rt_timer_start(&timer1);
		
//		rt_timer_init(&timer2, "timer2",         /* 定时器名字是timer1 */
//                    timeout2,                /* 超时时回调的处理函数 */
//                    RT_NULL,                 /* 超时函数的入口参数 */
//                    5,                     /* 定时长度,以OSTick为单位1ms*/
//                    RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */
//    rt_timer_start(&timer2);
//		
//		rt_timer_init(&timer3, "timer3",         /* 定时器名字是timer1 */
//                    timeout3,                /* 超时时回调的处理函数 */
//                    RT_NULL,                 /* 超时函数的入口参数 */
//                    2,                     /* 定时长度,以OSTick为单位1ms*/
//                    RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */
//    rt_timer_start(&timer3);

	return 0;
}
void printf_text(int argc, char**argv)
{

printf("int:%d",return_int(argv[1]));	
printf("float:%f \r\n",return_float(argv[1]));
rt_thread_mdelay(500);

}
MSH_CMD_EXPORT(printf_text, atcmd sample: atcmd <server|client>);
