
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32_eth_bsp.h"
#include "netconf.h"

#include "bsp.h"
#include "miniDebug.h"
#include "ili_lcd.h"

#include "lwip/sockets.h"

#define LED_TASK_PRIO			( tskIDLE_PRIORITY + 1 )

void led_task(void *pvParam);
void Touch_task(void *pvParam);
void TCPClient(void *arg);

xSemaphoreHandle gTouchxSem;

RCC_ClocksTypeDef RCC_Clocks;
void main(void)
{
	bsp_init();
	
	/* configure ethernet (GPIOs, clocks, MAC, DMA) */
	ETH_BSP_Config();
	/* Initilaize the LwIP stack */
	LwIP_Init();
RCC_GetClocksFreq(&RCC_Clocks);
//	xTaskCreate(led_task, "LED", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);
	xTaskCreate(TCPClient/*Touch_task*/, "Touch", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);

	/* Start scheduler */
	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler */
	for( ;; );
}

void led_task(void *pvParam)
{

	for( ; ; )
	{
                LED0 =!LED0;
		vTaskDelay( 500 );
	}
}
//------------------------------------------------------------------------------
typedef struct _XTCPCLIENTSOCK{
	int s;						/*socket 标识符 -1无效，>= 0 有效*/
	int bconnect;				/*socket 是否连接成功，0 未连接，>= 1 连接*/
	xSemaphoreHandle	sxMutex;/*互斥量，socket不是线程安全的，为了实现socket
									在一个线程里接收数据，而在其他线程发送，
									建立互斥量来做互斥操作*/
}XTCPCLIENTSOCK;

XTCPCLIENTSOCK xSeverSocket;
XTCPCLIENTSOCK xClientSocket;

#define BUF_SIZE		1024
char ClientRevBuf[BUF_SIZE];

#if 1

#define MYPORT 8088    // the port users will be connecting to  
#define BACKLOG 5     // how many pending connections queue will hold  
int fd_A[BACKLOG];    // accepted connection fd  
int conn_amount;      // current connection amount 


void TCPClient(void *arg)
{
      int sock_fd, new_fd;             // listen on sock_fd, new connection on new_fd  
      struct sockaddr_in server_addr;  // server address information  
      struct sockaddr_in client_addr;  // connector's address information  
      socklen_t sin_size;  
      int yes = 1;  
      char buf[BUF_SIZE];  
      int ret;  
      int i;  
      
      if ((sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {  

      }  
      
      if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {  

      }  
      server_addr.sin_family = AF_INET;         // host byte order  
      server_addr.sin_port = htons(MYPORT);     // short, network byte order  
      server_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP  
      memset(server_addr.sin_zero, '/0', sizeof(server_addr.sin_zero));  
      
      
      if (bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {  

      }  
      
      
      if (listen(sock_fd, BACKLOG) == -1) {  
 
      }  
      
  
  
      fd_set fdsr;  
      int maxsock;  
      struct timeval tv;  
      conn_amount = 0;  
      sin_size = sizeof(client_addr);  
      maxsock = sock_fd;  
      
      
      while (1)   
      {  
        // initialize file descriptor set  
        FD_ZERO(&fdsr);  
        FD_SET(sock_fd, &fdsr);  // add fd  
        
        
        // timeout setting  
        tv.tv_sec = 10;  
        tv.tv_usec = 0;  
        
        
        // add active connection to fd set  
        for (i = 0; i < BACKLOG; i++) {  
          if (fd_A[i] != 0) {  
            FD_SET(fd_A[i], &fdsr);  
          }  
        }  
        
        
        ret = select(maxsock + 1, &fdsr, NULL, NULL, &tv);  
        if (ret < 0) {          // error   
          break;  
        } else if (ret == 0) {  // time out  
//          printf("timeout/n");  
          continue;  
        }  
        
        
        // check every fd in the set  
        for (i = 0; i < conn_amount; i++)   
        {  
          if (FD_ISSET(fd_A[i], &fdsr)) // check which fd is ready  
          {  
            ret = recv(fd_A[i], buf, sizeof(buf), 0);  
            if (ret <= 0)   
            {        // client close  
//              printf("ret : %d and client[%d] close/n", ret, i);  
              close(fd_A[i]);  
              FD_CLR(fd_A[i], &fdsr);  // delete fd   
              fd_A[i] = 0;  
              conn_amount--;  
            }  
            else   
            {        // receive data  
              if (ret < BUF_SIZE)  
//                memset(&buf[ret], '/0', 1); // add NULL('/0')  
              
//              printf("client[%d] send:%s/n", i, buf);  
              
              //send
              send(fd_A[i],buf,ret,0);
            }  
          }  
        }  
        
        // check whether a new connection comes  
        if (FD_ISSET(sock_fd, &fdsr))   
        {  
            // accept new connection  
            new_fd = accept(sock_fd, (struct sockaddr *)&client_addr, &sin_size);  
            if (new_fd <= 0)   
            {  
                continue;  
            }  
          
          
          // add to fd queue  
          if (conn_amount < BACKLOG)   
          {  
            fd_A[conn_amount++] = new_fd;  
//            printf("new connection client[%d] %s:%d/n", conn_amount,  
//                   inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));  
            if (new_fd > maxsock)  // update the maxsock fd for select function  
              maxsock = new_fd;  
          }  
          else   
          {  
//              printf("max connections arrive, exit/n");  
              send(new_fd, "bye", 4, 0);  
              close(new_fd);  
              break;     
          }  
        }  
        
      }  
      
      
      // close other connections  
      for (i = 0; i < BACKLOG; i++)   
      {  
        if (fd_A[i] != 0)   
        {   
          close(fd_A[i]);  
        }  
      } 
  
}


#else
void TCPClient(void *arg)
{
	struct sockaddr_in ServerAddr;
	int optval = 1;
	fd_set fdsr;
	struct timeval tv;
	
	(void)arg;
	xClientSocket.bconnect = 0;
	//建立互斥量
	xClientSocket.sxMutex = xSemaphoreCreateMutex();
	
	if(xClientSocket.sxMutex == NULL )
	{
		while(1);
	}
	while(1)
	{
		//创建客户端
		xClientSocket.s = lwip_socket( AF_INET,SOCK_STREAM,IPPROTO_TCP);
		if(xClientSocket.s == -1 ){
			continue;
		}
		//创建成功
		optval = 1;
		
		lwip_setsockopt(xClientSocket.s,SOL_SOCKET,SO_KEEPALIVE,&optval,sizeof(optval));
		ServerAddr.sin_family = AF_INET;
		ServerAddr.sin_addr.s_addr = inet_addr("10.129.200.79");//inet_addr("122.224.200.89");//
		ServerAddr.sin_port = htons(8080);
		
		xClientSocket.bconnect = 0;
		//连接服务器
		if( lwip_connect(xClientSocket.s,(struct sockaddr*)&ServerAddr,sizeof(ServerAddr) ) == - 1)
		{
			//connect error!
			lwip_close(xClientSocket.s);
			continue;
		}
		xClientSocket.bconnect = 1;
		
		//是否接收到数据
		while(1)
		{
			FD_ZERO(&fdsr);
			FD_SET(xClientSocket.s,&fdsr);
			tv.tv_sec = 10;
			tv.tv_usec = 0 ;

			if( lwip_select(xClientSocket.s + 1,&fdsr,NULL,0,&tv) == 0 )
				continue;
			
			if( FD_ISSET(xClientSocket.s,& fdsr) )
			{
				int datalen;
				int ret;
				//有数据，互斥操作
				if( xSemaphoreTake( xClientSocket.sxMutex, portMAX_DELAY ) != pdTRUE ){
					while(1);
				}

				datalen = lwip_recv(xClientSocket.s,ClientRevBuf,BUF_SIZE,0);
				if(datalen > 0)
				{
					//接收的数据,测试，回送给服务器
					ret = lwip_send(xClientSocket.s,ClientRevBuf,datalen,0);
					
				}else{
					//服务器关闭等异常
					xClientSocket.bconnect = 0 ;
					lwip_close(xClientSocket.s);
					xSemaphoreGive( xClientSocket.sxMutex );
					break;	//重新连接服务器	
				}
				xSemaphoreGive( xClientSocket.sxMutex );
			}			
		}	
	}
}
#endif
//------------------------------------------------------------------------------
//触摸屏任务，触摸屏触摸中断后发出信号，任务得于运行处理
//char str[16];
//void Touch_task(void *pvParam)
//{
//	unsigned short TouchXVal,TouchYVal;
//	LCD_TouchEnable();
//	vSemaphoreCreateBinary(gTouchxSem);	//创建触摸输入中断发生信号量
//	xSemaphoreTake( gTouchxSem,0);		//清除信号
//	for(; ; )
//	{
//		if( xSemaphoreTake( gTouchxSem, portMAX_DELAY ) == pdTRUE )
//		{
//			TouchXVal = LCD_ReadTouchXY(0);
//			TouchYVal = LCD_ReadTouchXY(1);
//			PRINTF("Touch X output:%4X\r\n",TouchXVal);
//			sprintf(str,"X:%4d",TouchXVal / 8);
//			//LCD_DisplayStringLine(128,"      ");
//			LCD_DisplayStringLine(128,(unsigned char*)str);
//			
//			PRINTF("Touch Y output:%4X\r\n",TouchYVal);
//			sprintf(str,"Y:%4d",TouchYVal / 8);
//			//LCD_DisplayStringLine(152,"      ");
//			LCD_DisplayStringLine(152,(unsigned char*)str);
//		}	
//	}
//}
//触摸屏有触摸后的中断
void EXTI9_5_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line7);
		xSemaphoreGiveFromISR( gTouchxSem, &xHigherPriorityTaskWoken );
	}
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
//------------------------------------------------------------------------------