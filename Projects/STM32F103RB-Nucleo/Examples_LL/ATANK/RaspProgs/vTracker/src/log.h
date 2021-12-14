#ifndef __LOG_H__

#include <stdio.h>

#define MAIN_DBG_PRINT(format, ...)    {fprintf(stdout, format, ##__VA_ARGS__); fprintf(stdout, "\n");}
#define FRAME_DBG_PRINT(format, ...)   {fprintf(stdout, format, ##__VA_ARGS__); fprintf(stdout, "\n");}
//#define FRAME_DBG_PRINT(format, ...)
#define VIDEO_DBG_PRINT(format, ...)   {fprintf(stdout, format, ##__VA_ARGS__); fprintf(stdout, "\n");}

#endif // __LOG_H__
