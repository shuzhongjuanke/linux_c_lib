#include "uart.h"

#include <string.h>     /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>     /*UNIX 标准函数定义*/
#include <fcntl.h>      /*文件 控制定义*/
#include <error.h>      /*错误号定义*/



#define SERIAL_DEVICCE_PATH  "/dev/tty%d" 
#define USBSERIAL_DEVICE_PATH "/dev/ttyUSB%d"
#define BAUD_RATE_SET(x)    B##x
/********************************************
 * @func: void serial_port_init(void *, uart_config_t * cfg,  unsigned char commno, bool is_usb_serial)
 * @brief: malloc the space and Initialize the 
 * @param: t point to the struct of uart;
 * cfg point to the uart setting para
 * commno serial port num
 * is_usb_serail serial device or usb2ttl device 
 * @return: NONE
 * @note:
 *******************************************/
void serial_port_init(void *t , uart_config_t * cfg,  unsigned char commno, bool is_usb_serial)
{
    uart * cthis = (uart *)t;
    memcpy(&cthis->congfig, cfg, sizeof(uart_config_t));
    cthis->commNo = commno;
    cthis->is_usb2ttl = is_usb_serial; 
}

/********************************************
 * @func: static int serial_port_set_opt(void *t)
 * @brief: setting serial port
 * @param: t point to the struct of uart;
 * @return: -1 failed   0 ok
 * @note:
 *******************************************/
static int serial_port_set_opt(void *t)
{
    uart * cthis = (uart *)t;
    struct termios newtio, oldtio;
    speed_t  temp_baud = CBAUD;
    unsigned short temp_databit = 0;
    /*获取与句柄cthis->uart_fd对应对象的配置参数，并将其保存在oldtio指向的空间*/
    if(tcgetattr(cthis->uart_fd, &oldtio) != 0)
    {
        perror("get comm attr failed");
        return -1;
    }
    bzero(&newtio, sizeof(struct termios));
    /*激活选项(本地连接和接收使能)  保存程序不会占用串口、使能从串口中读取输入数据*/
    newtio.c_cflag |= CLOCAL | CREAD;
    
    /*set baud*/
    switch(cthis->congfig.baudRate)
    {
        case Baud1200:
            temp_baud = B1200;
            break;
        case Baud2400:
            temp_baud = B2400;
            break;
        case Baud4800:
            temp_baud = B4800;
            break;
        case Baud9600:
            temp_baud = B9600;
            break;
        case Baud19200:
            temp_baud = B19200;
        break;
        case Baud38400:
            temp_baud = B38400;
        break;
        case Baud57600:
             temp_baud = B57600;
        break;
        case Baud115200:
             temp_baud = B115200;
        break;
        default:
        break;
    }
     cfsetispeed (&newtio, temp_baud);
     cfsetospeed (&newtio, temp_baud);

     /*set flow control*/
     switch(cthis->congfig.flowControl)
     {
         case NoFlowControl:
            newtio.c_cflag &= ~(CRTSCTS);
         break;
         case HardwareControl:
            newtio.c_cflag |= (CRTSCTS);
         break;
         case SoftwareControl:
            newtio.c_iflag |= (IXON | IXOFF | IXANY);  //启动输入、输出软件流控，允许字符重启输出
         break;

     }

    /*set data bit*/
    newtio.c_cflag &= ~(CSIZE);

    switch(cthis->congfig.dataBits)
    {
        case Data5:
        temp_databit =CS5;
        break;
        case Data6:
         temp_databit =CS6;
        break;
        case Data7:
         temp_databit =CS7;
        break;
        case Data8:
         temp_databit =CS8;
        break;
        default:
        break;
    }
    newtio.c_cflag |= temp_databit;

    /*设置奇偶校验(1、激活校验位使能和是否进行偶校验；2、激活奇偶校验)*/
   
    switch(cthis->congfig.parity)
    {
        case NoParity:  /*无校验*/
             newtio.c_cflag &= ~(PARENB);
             newtio.c_iflag &= ~(INPCK);
        break;
        case  EvenParity:/*偶校验*/
             newtio.c_cflag |= (PARENB);
             newtio.c_cflag &= ~(PARODD);
             newtio.c_iflag |= (INPCK | ISTRIP);
        break;
        case OddParity:/*奇校验*/
            newtio.c_cflag |= (PARENB);
             newtio.c_cflag |= (PARODD);
             newtio.c_iflag |= (INPCK | ISTRIP);
        break;
        default:
        break;
    }

    /*设置停止位 激活并设置*/
    switch(cthis->congfig.stopbits)
    {        
        case TwoStop:
        newtio.c_cflag |= (CSTOPB);
        break;
        case  OneStop:
        default:
        newtio.c_cflag &= ~(CSTOPB);
        break; 
    }
    /*修改输出模式，原始数据输出*/
    newtio.c_oflag &= ~(OPOST);
    /*设置终端模式，设置为原始模式*/
    //ewtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    cfmakeraw(&newtio);
    /*设置等待时间和最小接收字符数，有数据即时读取；无数据返回0*/
    newtio.c_cc[VTIME] = 0;   /*读取一个字符等待1*100ms时间，单位是100ms*/
    newtio.c_cc[VMIN] = 0;
    /*刷新输入输出缓冲区*/
    tcflush(cthis->uart_fd, TCIOFLUSH);
    /*激活新配置*/
    if(tcsetattr(cthis->uart_fd, TCSANOW, &newtio) != 0)  
    {
        perror("serial set error");
        return -1;
    }
    return 0;

}
/********************************************
 * @func: void serial_port_init(void *, uart_config_t * cfg,  unsigned char commno, bool is_usb_serial)
 * @brief: malloc the space and Initialize the 
 * @param: t point to the struct of uart;
 * cfg point to the uart setting para
 * commno serial port num
 * is_usb_serail serial device or usb2ttl device 
 * @return: NONE
 * @note:
 *******************************************/
int serial_port_open(void * t)
{
    uart * cthis = (uart *)t;
    char devicePath[32] = {0};
    int flag = 0;
    if(cthis->is_usb2ttl)
    {
        sprintf(devicePath, USBSERIAL_DEVICE_PATH, cthis->commNo);
    }
    else
    {
        sprintf(devicePath, SERIAL_DEVICCE_PATH, cthis->commNo);
    }
    
     /*
    /dev/ttySn  一般为串行端口终端，接串口线使用的端口设备；
    /dev/ttyUSBn 一般为USB转串口终端，接USB转串口线可用此端口设备
    */
    cthis->uart_fd =  open(devicePath, O_RDWR | O_NOCTTY | O_NDELAY);
    if(-1 == cthis->uart_fd)
    {
        perror("open serial port failed");
        return (-1);
    }
    /*检查open结果*/
    /*1.取得文件的状态旗标，此旗标为open()的参数flags*/
    flag = fcntl(cthis->uart_fd, F_GETFL, 0); 
    if(flag < 0)
    {
        perror("fcntl");
        return -1;
    }
    /*按照具体情况，设置新的旗标装填，只允许O_APPEND、O_NONOBLOCK、O_ASYNC位的改变*/
    flag |= O_NONBLOCK;
    if(fcntl(cthis->uart_fd, F_SETFL, flag) < 0)
    {
        perror("fcntl");
        return -1;
    }
    /*测试是否是终端设备*/
    if(0 == isatty(STDIN_FIFENO))
    {
        printf("standard input is not a terminal device\n");
        return -1;
    }


    serial_port_set_opt(t);




}

/********************************************
 * @func: void serial_port_close(void *t)
 * @brief: close serial port
 * @param: t point to the struct of uart;
 * @return: NONE
 * @note:
 *******************************************/
void serial_port_close(void *t)
{
     uart * cthis = (uart *)malloc(sizeof(uart));
     
     close(cthis->uart_fd);
     cthis->is_open = false;
}

/********************************************
 * @func: void serial_port_close(void *t)
 * @brief: close serial port
 * @param: t point to the struct of uart;
 * @return: NONE
 * @note:
 *******************************************/
int serial_port_write(void * t, unsigned char *input_buf, unsigned short write_cnt)
{
    uart * cthis = (uart *)t;
    ssize_t wrote_len = write(cthis->uart_fd, (void *)input_buf, write_cnt);
    if(wrote_len == write_cnt)
    {
        printf("send data success!");
    }
    else
    {
        tcflush(cthis->uart_fd, TCOFLUSH);
    }
    
    return wrote_len;  
}

int serial_port_read(void * t, unsigned short read_cnt, unsigned char *output_buf)
{
    uart * cthis = (uart *)t;
    ssize_t read_len = read(cthis->uart_fd, output_buf, read_cnt);
    return read_len;

}


/********************************************
 * @func: uart * uart_new(void )
 * @brief: create uart object
 * @param: NONE
 * @return: NONE
 * @note:
 *******************************************/
uart * uart_new(void )
{
    uart * cthis = (uart *)malloc(sizeof(uart));
    if(!cthis)
    {
        printf("create uart object space failed.");
        return NULL;  
    }
    memset(cthis, 0, sizeof(uart));
    cthis->serial_init = serial_port_init;
    cthis->serail_open = serial_port_open;
    cthis->serial_close = serial_port_close;


    return cthis;
}
/********************************************
 * @func: void uart_delete(uart * t)
 * @brief: create uart object
 * @param: NONE
 * @return: NONE
 * @note:
 *******************************************/
void uart_delete(uart * t)
{
    uart * cthis = t;
    free(cthis);
    cthis = NULL;

}
