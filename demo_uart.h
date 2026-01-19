/**
 * @file demo_uart.h
 * @brief UART DMA+Interrupt Module Interface
 */

#ifndef DEMO_UART_H
#define DEMO_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize UART with DMA TX and Interrupt RX
 * @return 0 on success
 */
int demo_uart_init(void);

/**
 * @brief De-initialize UART
 */
void demo_uart_deinit(void);

/**
 * @brief Get UART RX ring buffer (for reading received data)
 */
ring_buffer_t* demo_uart_get_rx_ring(void);

/**
 * @brief Get UART TX ring buffer (for writing data to send)
 */
ring_buffer_t* demo_uart_get_tx_ring(void);

/**
 * @brief Process UART TX - send pending data via DMA
 * Call this from main loop
 * @return Bytes sent this call
 */
uint32_t demo_uart_tx_process(void);

/**
 * @brief Check if UART TX is busy (DMA in progress)
 */
bool demo_uart_tx_is_busy(void);

/**
 * @brief Send data directly via DMA (blocking until complete)
 * @param data Data buffer
 * @param len Data length
 * @return Bytes sent
 */
uint32_t demo_uart_tx_send(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* DEMO_UART_H */
