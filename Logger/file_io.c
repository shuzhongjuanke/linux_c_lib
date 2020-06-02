
#include "file_io.h"
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
//构造函数
void file_io_ctor(file_io_t * const obj, char *pathname)
{
    obj->path = pathname;

}

//打开文件
fl_err_t file_io_open(file_io_t * const obj, int flags)
{
    fl_err_t ret = FL_OK;
    obj->fd = open(obj->path, flags);
    if(-1 == obj->fd)
    {
        perror("open file:");
        ret = FL_FAIL;
    }
    return ret;    

}


