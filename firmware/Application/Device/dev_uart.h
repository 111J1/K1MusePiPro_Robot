#ifndef _DEV_UART_H_
#define _DEV_UART_H_

#include <stdint.h>

typedef void (*uart_rx_callback_t)(const uint8_t *data, uint16_t len);

typedef struct uart_driver_class {
    const void *ctx;
    uint8_t (*init)(const void *ctx);
    uint8_t (*send)(const void *ctx, const uint8_t *data, uint16_t len);
    uint8_t (*is_tx_ready)(const void *ctx);
    uint16_t (*available)(const void *ctx);
    uint16_t (*read)(const void *ctx, uint8_t *data, uint16_t len);
    void (*set_rx_callback)(const void *ctx, uart_rx_callback_t callback);
} uart_driver_t;

#endif /* _DEV_UART_H_ */
