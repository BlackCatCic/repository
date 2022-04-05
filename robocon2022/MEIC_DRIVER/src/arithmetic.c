#include "arithmetic.h"

/**************************************************/

// ����ƽ���˲���(�ֳƻ���ƽ���˲���)
#define FILTER_N 10
u8 i;
float filter_buf[FILTER_N + 1];
float Filter(float * data) 
{
	float filter_sum = 0;
	filter_buf[FILTER_N] = *data;
	for(i = 0; i < FILTER_N; i++) 
	 {
		filter_buf[i] = filter_buf[i + 1]; // �����������ƣ���λ�Ե�
		filter_sum += filter_buf[i];
	 }
	if(i==FILTER_N)i=0;
	return (filter_sum / FILTER_N);
}


int16_t filter_buf2[FILTER_N + 1];
int16_t Filter2(int16_t * data) 
{
	int16_t filter_sum = 0;
	filter_buf2[FILTER_N] = *data;
	for(i = 0; i < FILTER_N; i++) 
	 {
		filter_buf2[i] = filter_buf2[i + 1]; // �����������ƣ���λ�Ե�
		filter_sum += filter_buf2[i];
	 }
	if(i==FILTER_N)i=0;
	return (filter_sum / FILTER_N);
}
