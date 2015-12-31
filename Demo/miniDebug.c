#include "miniDebug.h"
void assert_failed(char * file, int line)
{
	PRINTF("error at %s line %d \r\n",file,line);
}
