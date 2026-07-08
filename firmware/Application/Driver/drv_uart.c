#include "drv_uart.h"

#include "dma.h"
#include "usart.h"
#include <string.h>

#define UART_DRV_DMA_RX_BUF_SIZE 128U
#define UART_DRV_RX_FIFO_SIZE 256U
#define UART_DRV_TX_BUF_SIZE 256U

typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t *dma_rx_buf;
    uint8_t *rx_fifo;
    uint8_t *rx_frame;
    uint8_t *tx_buf;
    uint16_t dma_rx_size;
    uint16_t rx_fifo_size;
    uint16_t tx_buf_size;
    volatile uint16_t rx_fifo_head;
    volatile uint16_t rx_fifo_tail;
    volatile uint16_t rx_fifo_count;
    volatile uint8_t tx_busy;
    uart_rx_callback_t rx_callback;
} uart_drv_ctx_t;

static uint8_t uart1_dma_rx_buf[UART_DRV_DMA_RX_BUF_SIZE];
static uint8_t uart1_rx_fifo[UART_DRV_RX_FIFO_SIZE];
static uint8_t uart1_rx_frame[UART_DRV_RX_FIFO_SIZE];
static uint8_t uart1_tx_buf[UART_DRV_TX_BUF_SIZE];

static uint8_t uart2_dma_rx_buf[UART_DRV_DMA_RX_BUF_SIZE];
static uint8_t uart2_rx_fifo[UART_DRV_RX_FIFO_SIZE];
static uint8_t uart2_rx_frame[UART_DRV_RX_FIFO_SIZE];
static uint8_t uart2_tx_buf[UART_DRV_TX_BUF_SIZE];

static uint8_t uart3_dma_rx_buf[UART_DRV_DMA_RX_BUF_SIZE];
static uint8_t uart3_rx_fifo[UART_DRV_RX_FIFO_SIZE];
static uint8_t uart3_rx_frame[UART_DRV_RX_FIFO_SIZE];
static uint8_t uart3_tx_buf[UART_DRV_TX_BUF_SIZE];

static uart_drv_ctx_t uart_ctx[DRV_UART_COUNT] = {
    [DRV_UART1] = {&huart1, uart1_dma_rx_buf, uart1_rx_fifo, uart1_rx_frame, uart1_tx_buf,
                   UART_DRV_DMA_RX_BUF_SIZE, UART_DRV_RX_FIFO_SIZE, UART_DRV_TX_BUF_SIZE, 0U, 0U, 0U, 0U, 0},
    [DRV_UART2] = {&huart2, uart2_dma_rx_buf, uart2_rx_fifo, uart2_rx_frame, uart2_tx_buf,
                   UART_DRV_DMA_RX_BUF_SIZE, UART_DRV_RX_FIFO_SIZE, UART_DRV_TX_BUF_SIZE, 0U, 0U, 0U, 0U, 0},
    [DRV_UART3] = {&huart3, uart3_dma_rx_buf, uart3_rx_fifo, uart3_rx_frame, uart3_tx_buf,
                   UART_DRV_DMA_RX_BUF_SIZE, UART_DRV_RX_FIFO_SIZE, UART_DRV_TX_BUF_SIZE, 0U, 0U, 0U, 0U, 0},
};

static uint16_t uart_drv_fifo_next(const uart_drv_ctx_t *ctx, uint16_t index)
{
    return (uint16_t)((index + 1U) % ctx->rx_fifo_size);
}

static void uart_drv_fifo_push(uart_drv_ctx_t *ctx, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0U; i < len; ++i) {
        if (ctx->rx_fifo_count >= ctx->rx_fifo_size) {
            ctx->rx_fifo_tail = uart_drv_fifo_next(ctx, ctx->rx_fifo_tail);
            ctx->rx_fifo_count--;
        }

        ctx->rx_fifo[ctx->rx_fifo_head] = data[i];
        ctx->rx_fifo_head = uart_drv_fifo_next(ctx, ctx->rx_fifo_head);
        ctx->rx_fifo_count++;
    }
}

static uint16_t uart_drv_fifo_pop(uart_drv_ctx_t *ctx, uint8_t *data, uint16_t max_len)
{
    uint16_t len = 0U;

    while ((ctx->rx_fifo_count > 0U) && (len < max_len)) {
        data[len++] = ctx->rx_fifo[ctx->rx_fifo_tail];
        ctx->rx_fifo_tail = uart_drv_fifo_next(ctx, ctx->rx_fifo_tail);
        ctx->rx_fifo_count--;
    }

    return len;
}

static uart_drv_ctx_t *uart_drv_find_ctx(UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0U; i < DRV_UART_COUNT; ++i) {
        if (uart_ctx[i].uart == huart) {
            return &uart_ctx[i];
        }
    }

    return 0;
}

static uint8_t uart_drv_start_rx(uart_drv_ctx_t *ctx)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(ctx->uart, ctx->dma_rx_buf, ctx->dma_rx_size) != HAL_OK) {
        return 0U;
    }

    __HAL_DMA_DISABLE_IT(ctx->uart->hdmarx, DMA_IT_HT);
    return 1U;
}

static uint8_t uart_drv_init(const void *ctx)
{
    uart_drv_ctx_t *uart_ctx = (uart_drv_ctx_t *)ctx;

    uart_ctx->rx_fifo_head = 0U;
    uart_ctx->rx_fifo_tail = 0U;
    uart_ctx->rx_fifo_count = 0U;
    uart_ctx->tx_busy = 0U;

    return uart_drv_start_rx(uart_ctx);
}

static uint8_t uart_drv_send(const void *ctx, const uint8_t *data, uint16_t len)
{
    uart_drv_ctx_t *uart_ctx = (uart_drv_ctx_t *)ctx;

    if ((data == 0) || (len == 0U) || (len > uart_ctx->tx_buf_size) ||
        (uart_ctx->tx_busy != 0U)) {
        return 0U;
    }

    uart_ctx->tx_busy = 1U;
    memcpy(uart_ctx->tx_buf, data, len);

    if (HAL_UART_Transmit_DMA(uart_ctx->uart, uart_ctx->tx_buf, len) != HAL_OK) {
        uart_ctx->tx_busy = 0U;
        return 0U;
    }

    return 1U;
}

static uint8_t uart_drv_is_tx_ready(const void *ctx)
{
    uart_drv_ctx_t *uart_ctx = (uart_drv_ctx_t *)ctx;

    return (uart_ctx->tx_busy == 0U) ? 1U : 0U;
}

static uint16_t uart_drv_available(const void *ctx)
{
    uart_drv_ctx_t *uart_ctx = (uart_drv_ctx_t *)ctx;
    uint16_t available;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    available = uart_ctx->rx_fifo_count;
    if (primask == 0U) {
        __enable_irq();
    }

    return available;
}

static uint16_t uart_drv_read(const void *ctx, uint8_t *data, uint16_t len)
{
    uart_drv_ctx_t *uart_ctx = (uart_drv_ctx_t *)ctx;
    uint16_t read_len;
    uint32_t primask;

    if ((data == 0) || (len == 0U)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    read_len = uart_drv_fifo_pop(uart_ctx, data, len);
    if (primask == 0U) {
        __enable_irq();
    }

    return read_len;
}

static void uart_drv_set_rx_callback(const void *ctx, uart_rx_callback_t callback)
{
    uart_drv_ctx_t *uart_ctx = (uart_drv_ctx_t *)ctx;

    uart_ctx->rx_callback = callback;
}

// instance definition
#define UART_DRIVER(_id) \
    {                    \
        .ctx = &uart_ctx[_id], .init = uart_drv_init, .send = uart_drv_send, .is_tx_ready = uart_drv_is_tx_ready, .available = uart_drv_available, .read = uart_drv_read, .set_rx_callback = uart_drv_set_rx_callback}

const uart_driver_t uart1_driver = UART_DRIVER(DRV_UART1);
const uart_driver_t uart2_driver = UART_DRIVER(DRV_UART2);
const uart_driver_t uart3_driver = UART_DRIVER(DRV_UART3);

const uart_driver_t *uart_drv_get(uart_drv_id_e id)
{
    static const uart_driver_t *const drivers[DRV_UART_COUNT] = {
        &uart1_driver,
        &uart2_driver,
        &uart3_driver,
    };

    if ((id < DRV_UART1) || (id >= DRV_UART_COUNT)) {
        return 0;
    }

    return drivers[id];
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uart_drv_ctx_t *ctx = uart_drv_find_ctx(huart);

    if (ctx != 0) {
        HAL_UART_RxEventTypeTypeDef event_type = HAL_UARTEx_GetRxEventType(huart);

        if (Size > ctx->dma_rx_size) {
            Size = ctx->dma_rx_size;
        }

        if (Size > 0U) {
            uart_drv_fifo_push(ctx, ctx->dma_rx_buf, Size);
        }

        if ((event_type == HAL_UART_RXEVENT_IDLE) && (ctx->rx_callback != 0)) {
            uint16_t frame_len = uart_drv_fifo_pop(ctx, ctx->rx_frame, ctx->rx_fifo_size);

            if (frame_len > 0U) {
                ctx->rx_callback(ctx->rx_frame, frame_len);
            }
        }

        (void)uart_drv_start_rx(ctx);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_drv_ctx_t *ctx = uart_drv_find_ctx(huart);

    if (ctx != 0) {
        ctx->tx_busy = 0U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart_drv_ctx_t *ctx = uart_drv_find_ctx(huart);

    if (ctx != 0) {
        ctx->tx_busy = 0U;
        (void)uart_drv_start_rx(ctx);
    }
}
