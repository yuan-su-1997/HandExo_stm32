#include "udp_demo.h"
#include "lwip_comm.h"
#include "usart.h"
#include "led.h"
#include "includes.h"
#include "lwip/api.h"
#include "lwip/lwip_sys.h"
#include "string.h"
#include "sensor.h"
//#include "get_pose.h"


//TCP�ͻ�������
#define UDP_PRIO		6
//�����ջ��С
#define UDP_STK_SIZE	300
//�����ջ
OS_STK UDP_TASK_STK[UDP_STK_SIZE];


u8 udp_demo_recvbuf[UDP_DEMO_RX_BUFSIZE];	//UDP�������ݻ�����

//UDP������������
const u8 *udp_demo_sendbuf="Explorer STM32F407 NETCONN UDP demo send data\r\n";
u8 udp_flag;							//UDP���ݷ��ͱ�־λ
//����ҷ��͵�������һ������
float test[] = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.1, 11.1, 12.2, 13.3, 14.4, 15.5, 16.6, 17.7, 18.8};

	
	

//udp������
static void udp_thread(void *arg)
{
	
	extern float angle[6];
	extern float angle2[6];
	extern float angle3[6];	
	extern float angle_udp[18];
	
	OS_CPU_SR cpu_sr;
	err_t err;
	static struct netconn *udpconn;
	static struct netbuf  *recvbuf;
	static struct netbuf  *sentbuf;
	struct ip_addr destipaddr;
	u32 data_len = 0;
	struct pbuf *q;
	
	LWIP_UNUSED_ARG(arg);
	udpconn = netconn_new(NETCONN_UDP);  																																			//����һ��UDP����
	udpconn->recv_timeout = 1;  
	
	if(udpconn != NULL)  																																											//����UDP���ӳɹ�
	{
		err = netconn_bind(udpconn,IP_ADDR_ANY,UDP_DEMO_PORT); 
		IP4_ADDR(&destipaddr,lwipdev.remoteip[0],lwipdev.remoteip[1], lwipdev.remoteip[2],lwipdev.remoteip[3]); //����Ŀ��IP��ַ
		netconn_connect(udpconn,&destipaddr,UDP_DEMO_PORT); 																										//���ӵ�Զ������
		
		//��ȡ��������ʼ�Ƕ�
		GetDegreeo();
		
		if(err == ERR_OK)//�����
		{
			
			while(1)
			{
				
				GetDegree();
							
				// if((udp_flag & LWIP_SEND_DATA) == LWIP_SEND_DATA) //������Ҫ����
				if( 1 ) 
				{
					sentbuf = netbuf_new();						//����һ��netbuf�Ľṹ��
						
					netbuf_alloc(sentbuf,sizeof(angle_udp));
									
					memcpy(sentbuf->p->payload,(void*)angle_udp,sizeof(angle_udp));
										
					err = netconn_send(udpconn,sentbuf);  	//��netbuf�е����ݷ��ͳ�ȥ
					
					if(err != ERR_OK)
					{
						printf("����ʧ��\r\n");
						netbuf_delete(sentbuf);      //ɾ��buf
					}
					udp_flag &= ~LWIP_SEND_DATA;	//������ݷ��ͱ�־
					netbuf_delete(sentbuf);      	//ɾ��buf
				}	
								
				netconn_recv(udpconn,&recvbuf); //�������� �����ε����д��벻�����lwip_dhcp_task(); ���ﻹҪ�о�һ��Ϊʲô������
				
				
			}
		}else printf("UDP��ʧ��\r\n");
	}else printf("UDP���Ӵ���ʧ��\r\n");
}


//����UDP�߳�
//����ֵ:0 UDP�����ɹ�
//		���� UDP����ʧ��
INT8U udp_demo_init(void)
{
	
	INT8U res;
	OS_CPU_SR cpu_sr;
	
	OS_ENTER_CRITICAL();	//���ж�
	res = OSTaskCreate(udp_thread,(void*)0,(OS_STK*)&UDP_TASK_STK[UDP_STK_SIZE-1],UDP_PRIO); 		// Created UDP thread
	OS_EXIT_CRITICAL();		//���ж�
	
	return res;
}

