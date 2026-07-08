#ifndef _DRV_UART_H_
#define _DRV_UART_H_

#include "dev_uart.h"

typedef enum {
    DRV_UART1 = 0,
    DRV_UART2,
    DRV_UART3,
    DRV_UART_COUNT,
} uart_drv_id_e;

extern const uart_driver_t uart1_driver;
extern const uart_driver_t uart2_driver;
extern const uart_driver_t uart3_driver;

const uart_driver_t *uart_drv_get(uart_drv_id_e id);

#endif /* _DRV_UART_H_ */
