#ifndef __UART_H_
#define __UART_H_
#include <termios.h>  /*POSIX 终端控制定义*/
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>


#define DEVICE_NAME_LEN (64)

typedef enum
{
Baud1200 = 1200,
Baud2400 = 2400,
Baud4800 = 4800,
Baud9600 = 9600,
Baud19200 = 19200,
Baud38400 = 38400,
Baud57600 = 57600,
Baud115200 = 115200,
UnknownBaud = -1
}SerialPort_BaudRate_t;



typedef enum
{
    Data5 = 5,
    Data6 = 6,
    Data7 = 7,
    Data8 = 8,
    UnknownDataBits = -1
}SerialPort_DataBits_t;



typedef enum
{
    NoFlowControl = 0,
    HardwareControl = 1,
    SoftwareControl = 2,
    UnknownFlowControl = -1    
}SerailPort_FlowControl_t;


typedef enum
{
    NoParity = 0,
    EvenParity = 2,
    OddParity = 3,
    UnknownParity = -1,
}SerailPort_Parity_t;


typedef enum
{
    OneStop = 1,
    OneAndHalfStop = 3,
    TwoStop = 2,
    UnknownStopBits = -1,
}SerialPort_StopBits_t;

typedef struct 
{
     SerialPort_BaudRate_t  baudRate;
    SerialPort_DataBits_t  dataBits;
    SerailPort_FlowControl_t flowControl;
    SerailPort_Parity_t  parity;
    SerialPort_StopBits_t stopbits; 
}uart_config_t;

typedef struct uart
{
    uart_config_t congfig; 
    unsigned char  commNo;  
    bool is_usb2ttl;
    bool is_open; 
    int uart_fd;
    void (*serial_init)(void *, uart_config_t *,  unsigned char , bool );
    int (*serail_open)(void *  );
    void (*serial_close)(void * );
    int (*serial_write)(void *, unsigned char *, unsigned short );
    int (*serial_read)(void *, unsigned short, unsigned char*);

    
}uart;
#endif // !__UART_H_



