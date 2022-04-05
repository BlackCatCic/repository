#include "Include.h"
struct rt_timer timer1,timer2,timer3,timer4;
rt_thread_t tid = RT_NULL;
int main(void)
{	
	rt_thread_mdelay(2000);
  /* �����߳� 1 */
  tid = rt_thread_create("thread1",      //�߳�����
                          thread_entry1, //�߳���ں���
	                        RT_NULL,       //�߳���ڲ���
                          1024,           //�߳�ջ��С
                          9,             //�߳����ȼ�
													5);            //�߳�ʱ��Ƭ
  if (tid != RT_NULL)
      rt_thread_startup(tid);
	
  /* �����߳� 2 */
  tid = rt_thread_create("thread2",      //�߳�����
                          thread_entry2, //�߳���ں���
	                        RT_NULL,       //�߳���ڲ���
                          1024,           //�߳�ջ��С
                          8,             //�߳����ȼ�
													5);            //�߳�ʱ��Ƭ
  if (tid != RT_NULL)
      rt_thread_startup(tid);

  /* �����߳� 2 */
  tid = rt_thread_create("thread3",      //�߳�����
                          thread_entry3, //�߳���ں���
	                        RT_NULL,       //�߳���ڲ���
                          1024,           //�߳�ջ��С
                          7,             //�߳����ȼ�
													5);            //�߳�ʱ��Ƭ
  if (tid != RT_NULL)
      rt_thread_startup(tid);
	
		rt_timer_init(&timer1, "timer1",         /* ��ʱ��������timer1 */
                    timeout1,                /* ��ʱʱ�ص��Ĵ����� */
                    RT_NULL,                 /* ��ʱ��������ڲ��� */
										10,                     /* ��ʱ����,��OSTickΪ��λ1ms*/
                    RT_TIMER_FLAG_PERIODIC); /* �����Զ�ʱ�� */
    rt_timer_start(&timer1);
		
//		rt_timer_init(&timer2, "timer2",         /* ��ʱ��������timer1 */
//                    timeout2,                /* ��ʱʱ�ص��Ĵ����� */
//                    RT_NULL,                 /* ��ʱ��������ڲ��� */
//                    5,                     /* ��ʱ����,��OSTickΪ��λ1ms*/
//                    RT_TIMER_FLAG_PERIODIC); /* �����Զ�ʱ�� */
//    rt_timer_start(&timer2);
//		
//		rt_timer_init(&timer3, "timer3",         /* ��ʱ��������timer1 */
//                    timeout3,                /* ��ʱʱ�ص��Ĵ����� */
//                    RT_NULL,                 /* ��ʱ��������ڲ��� */
//                    2,                     /* ��ʱ����,��OSTickΪ��λ1ms*/
//                    RT_TIMER_FLAG_PERIODIC); /* �����Զ�ʱ�� */
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
