#ifndef H_UART_CMDS
#define H_UART_CMDS

typedef enum
{
    not_specified,
    flash,
    msg_queue,
    tof,
    imu,
    motor,
    led,
    mesh,
    uart,
    nav,
    error,
    dispatcher_max,
} dispatcher_type_t;

void UART_INIT(void);

#endif