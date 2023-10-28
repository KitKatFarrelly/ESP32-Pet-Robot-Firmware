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
    error,
} dispatcher_type_t;

void UART_INIT(void);

#endif