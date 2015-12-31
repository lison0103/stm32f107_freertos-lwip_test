#ifndef __MINIDEBUG_H__
#define __MINIDEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _EMB_SIMULATION
	void format_and_log(const char* fmtstr, ...);
#else
	int printf(const char *format, ...);
	int sprintf(char *out, const char *format, ...);
	int snprintf( char *buf, unsigned int count, const char *format, ... );
#endif

//调试使用的输出
#ifdef DEBUG 
	#ifdef _EMB_SIMULATION	
		#include <windows.h>
		#include <crtdbg.h>
		//vc下的输出
		#define PRINTF(fmtstr, ...)	format_and_log(fmtstr, __VA_ARGS__)

		#define ASSERT(expr)	(void) ((!!(expr)) || \
										(1 != _CrtDbgReport(_CRT_ASSERT, __FILE__, __LINE__, NULL, #expr)) || \
										(_CrtDbgBreak(), 0))
	#else	//_EMB_SIMULATION
		// IAR下的输出
		#define PRINTF(...)	printf(__VA_ARGS__)
		extern void assert_failed(char * file, int line);
		#define ASSERT(expr) ((expr) ? (void)0 : assert_failed((char *)__FILE__, __LINE__))
	#endif
		

#else   
	#define PRINTF(...) ;
	#define ASSERT(expr)
#endif	//_DEBUG

		
/*
		使用样例:
		PRINTF("App start...\r\n");
		PRINTF("Test debug out:%s,%d\r\n",(char *)__FILE__, __LINE__);
*/

#ifdef __cplusplus
}
#endif

#endif
/*------------------------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/ yanxpemail@163.com
                								  \_)
------------------------------------------------------------------------------------------------------------------------*/
