#ifndef __LOGGER_H_
#define __LOGGER_H_

#define FHLOG_LEVEL_TRACE	0x1
#define FHLOG_LEVEL_DEBUG	0x2
#define FHLOG_LEVEL_INFO	0x4
#define FHLOG_LEVEL_WARNING 0x8
#define FHLOG_LEVEL_ERROR   0x10
#define FHLOG_LEVEL_EXCEPT  0x20
#define FHLOG_LEVEL_FATAL   0x40

#define FHLOG_LEVELS_FATAL		FHLOG_LEVEL_FATAL
#define FHLOG_LEVELS_EXCEPT		(FHLOG_LEVEL_EXCEPT | FHLOG_LEVELS_FATAL)
#define FHLOG_LEVELS_ERROR		(FHLOG_LEVEL_ERROR | FHLOG_LEVELS_EXCEPT)
#define FHLOG_LEVELS_WARNING	(FHLOG_LEVEL_WARNING | FHLOG_LEVELS_ERROR)
#define FHLOG_LEVELS_INFO		(FHLOG_LEVEL_INFO | FHLOG_LEVELS_WARNING)
#define FHLOG_LEVELS_DEBUG		(FHLOG_LEVEL_DEBUG | FHLOG_LEVELS_INFO)
#define FHLOG_LEVELS_TRACE		(FHLOG_LEVEL_TRACE | FHLOG_LEVELS_DEBUG)


#define FHLOG_TO_FILE  0x1
#define FHLOG_TO_CONSOLE 0x2
#define FHLOG_TO_BOTH  0x3

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

struct logger
{
    int nOutputDevices_m;   /*输出设备*/
    int nLogLevels_m;       /*Log冗余等级*/
    size_t nCurrentSize_m;
    int nFd_m;              /*文件描述符*/ 
    unsigned short nMaxFileCount_m;  /*日志文件最大个数*/ 
    unsigned short nMaxFileBytes_m;  /*日志文件最大容量*/
    bool nToFileEnable;             /*写入文件标志*/
    char * strPath_m;               /*日志文件保存的根路径*/
    char * strPrefix_m;             /*日志文件前缀*/
    char * strPostfix_m;            /*日志文件扩展名*/
    


};



#endif // !__LOGGER_H_

